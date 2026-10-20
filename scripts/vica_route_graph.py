#!/usr/bin/env python3
"""레일(route graph)을 그린다 — nav2_route 가 읽는 GeoJSON 을 만든다."""
from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vica_corridor_measure import robot_radii  # noqa: E402

WS = Path(__file__).resolve().parent.parent / 'vica_ros2_ws'

sys.setrecursionlimit(50000)

WANT_CLEAR = 1.00
PREFER_CLEAR = 6.0
SIMPLIFY_M = 0.25
MERGE_M = 0.60
MAX_EDGE_M = 1.00
FILLET_R = 1.00
FILLET_R_FALLBACK = (0.75, 0.50)
FILLET_MIN_DEG = 15
FILLET_MAX_DEG = 120
FILLET_STEP_DEG = 20
SPECK_MAX_PX = 40
STUB_M = 2.00
JUNCTION_PENALTY = 0.1


def load_map(name):
    base = WS / 'maps' / name
    meta = yaml.safe_load(base.with_suffix('.yaml').read_text(encoding='utf-8'))
    return np.array(Image.open(str(base) + '.pgm')), meta


def free_mask(img):
    """자유 칸. 회색만으로 된 작은 섬(잡티)은 자유로 친다. (mask, 지운 섬 수)"""
    free = img > 250
    lab, n = ndimage.label(~free)
    if n == 0:
        return free, 0
    idx = range(1, n + 1)
    sizes = ndimage.sum(np.ones(img.shape, dtype=np.int32), lab, idx)
    blacks = ndimage.sum(img < 200, lab, idx)
    specks = [k + 1 for k in range(n) if sizes[k] <= SPECK_MAX_PX and blacks[k] == 0]
    if specks:
        free = free | np.isin(lab, specks)
    return free, len(specks)


def to_px(x, y, meta, shape):
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    res = float(meta['resolution'])
    return int(round(shape[0] - 1 - (y - oy) / res)), int(round((x - ox) / res))


def to_world(r, c, meta, shape):
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    res = float(meta['resolution'])
    return ox + c * res, oy + (shape[0] - 1 - r) * res


def dijkstra(mask, clearance, start, goal, res):
    """mask 안에서 start -> goal 최단경로. 벽에 가까울수록 비용이 붙는다."""
    h, w = mask.shape
    INF = math.inf
    dist = np.full((h, w), INF)
    prev = {}
    dist[start] = 0.0
    pq = [(0.0, start)]
    nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    hit = (lambda rc: rc == goal) if isinstance(goal, tuple) else (lambda rc: bool(goal[rc]))
    found = None
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        if hit((r, c)):
            found = (r, c)
            break
        for dr, dc, step in nb:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < h and 0 <= nc < w) or not mask[nr, nc]:
                continue
            short = max(0.0, WANT_CLEAR - clearance[nr, nc])
            nd = d + step * res * (1.0 + PREFER_CLEAR * short)
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                prev[(nr, nc)] = (r, c)
                heapq.heappush(pq, (nd, (nr, nc)))
    if found is None:
        return None
    path, cur = [found], found
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    return path[::-1]


def simplify(points, tol):
    """Douglas-Peucker. 꺾이는 점만 남긴다."""
    if len(points) < 3:
        return list(points)
    a, b = np.array(points[0], float), np.array(points[-1], float)
    ab = b - a
    n = np.hypot(*ab)
    if n < 1e-9:
        d = [np.hypot(*(np.array(p, float) - a)) for p in points]
    else:
        d = [abs(np.cross(ab, np.array(p, float) - a)) / n for p in points]
    i = int(np.argmax(d))
    if d[i] <= tol:
        return [points[0], points[-1]]
    return simplify(points[:i + 1], tol)[:-1] + simplify(points[i:], tol)


def nearest_true(mask, r, c):
    """(r,c) 에서 가장 가까운 mask=True 셀."""
    if mask[r, c]:
        return (r, c)
    idx = ndimage.distance_transform_edt(~mask, return_distances=False,
                                         return_indices=True)
    return int(idx[0][r, c]), int(idx[1][r, c])


def _fill_between(nd, nb, drivable, meta, shape):
    """두 노드 사이를 MAX_EDGE_M 이하로 쪼갤 사이 점들."""
    out = []
    seg = math.dist(nd['xy'], nb['xy'])
    k = int(math.ceil(seg / MAX_EDGE_M)) - 1
    for j in range(1, k + 1):
        t = j / (k + 1)
        wx = nd['xy'][0] + t * (nb['xy'][0] - nd['xy'][0])
        wy = nd['xy'][1] + t * (nb['xy'][1] - nd['xy'][1])
        r, c = to_px(wx, wy, meta, shape)
        r = min(max(r, 0), shape[0] - 1)
        c = min(max(c, 0), shape[1] - 1)
        rr, cc = nearest_true(drivable, r, c)
        if (rr, cc) != (r, c):
            wx, wy = to_world(rr, cc, meta, shape)
        out.append({'xy': (wx, wy), 'px': (rr, cc), 'filler': True})
    return out


def split_long_edges(nodes, drivable, meta, shape):
    """고리의 모든 엣지를 MAX_EDGE_M 이하로 쪼갠다."""
    out = []
    n = len(nodes)
    for i, nd in enumerate(nodes):
        out.append(nd)
        out.extend(_fill_between(nd, nodes[(i + 1) % n], drivable, meta, shape))
    return out


def split_chain_long(nodes, drivable, meta, shape):
    """열린 사슬의 엣지를 MAX_EDGE_M 이하로 쪼갠다. 끝점은 그대로."""
    out = []
    for i in range(len(nodes) - 1):
        out.append(nodes[i])
        out.extend(_fill_between(nodes[i], nodes[i + 1], drivable, meta, shape))
    out.append(nodes[-1])
    return out


def fillet_arc(a, b, c, drivable, meta, shape, t=None, r_max=None):
    """코너 b(a→b→c)를 호(arc) 위의 점들로 바꾼다. 못 바꾸면 None."""
    va = (a[0] - b[0], a[1] - b[1]); vc = (c[0] - b[0], c[1] - b[1])
    la, lc = math.hypot(*va), math.hypot(*vc)
    if la < 1e-6 or lc < 1e-6:
        return None
    cosphi = max(-1.0, min(1.0, (va[0]*vc[0] + va[1]*vc[1]) / (la * lc)))
    phi = math.acos(cosphi)
    theta = math.pi - phi
    if not (math.radians(FILLET_MIN_DEG) <= theta <= math.radians(FILLET_MAX_DEG)):
        return None
    tan_half = math.tan(phi / 2.0)
    if t is None:
        R = min(r_max or FILLET_R, 0.45 * min(la, lc) * tan_half)
    else:
        R = t * tan_half
    if R < 0.15:
        return None
    t = R / tan_half
    ua = (va[0] / la, va[1] / la); uc = (vc[0] / lc, vc[1] / lc)
    p1 = (b[0] + ua[0] * t, b[1] + ua[1] * t)
    p2 = (b[0] + uc[0] * t, b[1] + uc[1] * t)
    bis = (ua[0] + uc[0], ua[1] + uc[1]); lb = math.hypot(*bis)
    if lb < 1e-6:
        return None
    d_center = R / math.sin(phi / 2.0)
    o = (b[0] + bis[0] / lb * d_center, b[1] + bis[1] / lb * d_center)
    a1 = math.atan2(p1[1] - o[1], p1[0] - o[0]); a2 = math.atan2(p2[1] - o[1], p2[0] - o[0])
    sweep = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    steps = max(2, int(math.ceil(abs(sweep) / math.radians(FILLET_STEP_DEG))))
    pts = []
    for k in range(steps + 1):
        ang = a1 + sweep * k / steps
        x, y = o[0] + R * math.cos(ang), o[1] + R * math.sin(ang)
        r, col = to_px(x, y, meta, shape)
        if not (0 <= r < shape[0] and 0 <= col < shape[1]) or not drivable[r, col]:
            return None
        pts.append({'xy': (x, y), 'px': (r, col), 'fillet': True})
    return pts


def fillet_try(a, b, c, drivable, meta, shape):
    """상한 반지름부터 시도해 들어가는 첫 호를 돌려준다. 다 안 들어가면 None(뾰족한 채)."""
    for r in (FILLET_R,) + tuple(FILLET_R_FALLBACK):
        pts = fillet_arc(a, b, c, drivable, meta, shape, r_max=r)
        if pts:
            return pts
    return None


def fillet_corners(nodes, drivable, meta, shape):
    """고리의 꺾이는 노드를 호 위의 점 여러 개로 바꾼다."""
    n = len(nodes)
    out = []
    for i, nd in enumerate(nodes):
        pts = fillet_try(nodes[i - 1]['xy'], nd['xy'], nodes[(i + 1) % n]['xy'],
                         drivable, meta, shape)
        out.extend(pts if pts else [nd])
    return out


def fillet_chain(nodes, drivable, meta, shape):
    """열린 사슬의 안쪽 코너만 둥글린다. 끝점(접점·진입점)과 이미 호인 점은 그대로."""
    if len(nodes) < 3:
        return nodes
    out = [nodes[0]]
    for i in range(1, len(nodes) - 1):
        nd = nodes[i]
        pts = None if nd.get('fillet') else fillet_try(
            nodes[i - 1]['xy'], nd['xy'], nodes[i + 1]['xy'], drivable, meta, shape)
        out.extend(pts if pts else [nd])
    out.append(nodes[-1])
    return out


def _tangent_limit(a, b, c, r_max=None):
    """코너 b 에서 fillet_arc 가 기본으로 잡는 접점 거리(엣지 45 %·R 상한)."""
    va = (a[0] - b[0], a[1] - b[1]); vc = (c[0] - b[0], c[1] - b[1])
    la, lc = math.hypot(*va), math.hypot(*vc)
    if la < 1e-6 or lc < 1e-6:
        return None
    cosphi = max(-1.0, min(1.0, (va[0]*vc[0] + va[1]*vc[1]) / (la * lc)))
    phi = math.acos(cosphi)
    tan_half = math.tan(phi / 2.0)
    if tan_half < 1e-6:
        return None
    return min(r_max or FILLET_R, 0.45 * min(la, lc) * tan_half) / tan_half


def fillet_junction(chains, j_px, spur_i, drivable, meta, shape):
    """곁가지 접점을 Y 자로 만든다. 등뼈 양쪽에서 곁가지로 드는 호 두 개를 놓고,"""
    spur = chains[spur_i]
    ins = [k for k, ch in enumerate(chains) if k != spur_i and ch[-1]['px'] == j_px]
    outs = [k for k, ch in enumerate(chains) if k != spur_i and ch[0]['px'] == j_px]
    if len(ins) != 1 or len(outs) != 1 or len(spur) < 2:
        return False
    W, E = chains[ins[0]], chains[outs[0]]
    if len(W) < 2 or len(E) < 2:
        return False
    J, w1, e1, s1 = spur[0]['xy'], W[-2]['xy'], E[1]['xy'], spur[1]['xy']
    arc_w = arc_e = None
    for r in (FILLET_R,) + tuple(FILLET_R_FALLBACK):
        tw, te = _tangent_limit(w1, J, s1, r), _tangent_limit(e1, J, s1, r)
        if tw is None or te is None:
            return False
        t = min(tw, te)
        arc_w = fillet_arc(w1, J, s1, drivable, meta, shape, t=t)
        arc_e = fillet_arc(e1, J, s1, drivable, meta, shape, t=t)
        if arc_w is not None and arc_e is not None:
            break
    if arc_w is None or arc_e is None:
        return False
    s_new = arc_w[-1]
    arc_e[-1] = s_new
    W.insert(len(W) - 1, arc_w[0])
    E.insert(1, arc_e[0])
    spur[0] = s_new
    chains.append(arc_w)
    chains.append(arc_e)
    return True


def merge_chain(pts_rc, meta, shape):
    """가까운 점 합치기. 사슬 끝점(접점·진입점)은 반드시 남긴다."""
    nodes = []
    last_i = len(pts_rc) - 1
    for k, rc in enumerate(pts_rc):
        wx, wy = to_world(rc[0], rc[1], meta, shape)
        if nodes and math.dist((wx, wy), nodes[-1]['xy']) < MERGE_M:
            if k != last_i:
                continue
            if len(nodes) > 1:
                nodes.pop()
        nodes.append({'xy': (wx, wy), 'px': rc})
    return nodes


def split_chain_at(chains, cell):
    """cell 을 품은 사슬을 그 자리에서 둘로 쪼갠다(셀을 둘이 공유). 끝점이면 그대로."""
    for k, ch in enumerate(chains):
        if cell in ch:
            i = ch.index(cell)
            if i == 0 or i == len(ch) - 1:
                return
            chains[k:k + 1] = [ch[:i + 1], ch[i:]]
            return
    raise RuntimeError(f'접점 {cell} 이 어느 사슬에도 없다')


def load_destinations(name):
    """미션 매니저가 **실제로 쓰는** 목적지를 읽는다."""
    out = []
    root = Path.home() / 'vica_data' / 'destinations' / name
    doc = root / 'destinations.yaml'
    if doc.exists():
        data = yaml.safe_load(doc.read_text(encoding='utf-8')) or {}
        for d in data.get('destinations', []):
            po = d.get('pose') or {}
            if po.get('x') is None:
                continue
            out.append({'name': d.get('name', '?'), 'x': float(po['x']),
                        'y': float(po['y']), 'yaw': float(po.get('yaw', 0.0)),
                        'src': 'destinations.yaml'})
        home = root / 'home.yaml'
        if home.exists():
            h = (yaml.safe_load(home.read_text(encoding='utf-8')) or {}).get('pose') or {}
            if h.get('x') is not None:
                out.append({'name': '홈', 'x': float(h['x']), 'y': float(h['y']),
                            'yaw': float(h.get('yaw', 0.0)), 'src': 'home.yaml'})
        print(f'  목적지 출처: {doc}')
        return out

    alt = WS / 'location' / name / 'locations.json'
    if alt.exists():
        print(f'  [!] {doc} 가 없다. 저장소 사본을 쓴다: {alt}')
        print('      미션 매니저가 쓰는 파일이 아니라 좌표가 다를 수 있다.')
        return [{'name': d['name'], 'x': d['x'], 'y': d['y'],
                 'yaw': float(d.get('yaw', 0.0)), 'src': 'locations.json'}
                for d in json.loads(alt.read_text(encoding='utf-8'))]
    print('  목적지 파일을 못 찾았다.')
    return []


def _path_len_m(path, res):
    return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)) * res


def build_loop(entries, drivable, clearance, meta, shape, res, loop_only):
    """고리: 진입점을 방 중심 기준 각도 순으로 이어 닫는다. (노드 목록, 엣지 목록)"""
    cx = float(np.mean([e['world'][0] for e in entries]))
    cy = float(np.mean([e['world'][1] for e in entries]))
    entries.sort(key=lambda e: math.atan2(e['world'][1] - cy, e['world'][0] - cx))

    polyline = []
    m = len(entries)
    for i in range(m if not loop_only else m):
        a, b = entries[i], entries[(i + 1) % m]
        path = dijkstra(drivable, clearance, a['px'], b['px'], res)
        if path is None:
            print(f'    [X] {a["name"]} -> {b["name"]} 길 없음')
            continue
        pts = simplify(path, SIMPLIFY_M / res)
        if polyline and pts and polyline[-1] == pts[0]:
            pts = pts[1:]
        polyline.extend(pts)

    nodes = []
    for rc in polyline:
        wx, wy = to_world(rc[0], rc[1], meta, shape)
        if nodes and math.dist((wx, wy), nodes[-1]['xy']) < MERGE_M:
            continue
        nodes.append({'xy': (wx, wy), 'px': rc})
    if len(nodes) > 2 and math.dist(nodes[0]['xy'], nodes[-1]['xy']) < MERGE_M:
        nodes.pop()

    nodes = fillet_corners(nodes, drivable, meta, shape)
    nodes = split_long_edges(nodes, drivable, meta, shape)

    for i, nd in enumerate(nodes):
        nd['id'] = i + 1
    edges = [(nodes[i], nodes[(i + 1) % len(nodes)]) for i in range(len(nodes))]
    return nodes, edges


def build_tree(entries, drivable, clearance, meta, shape, res):
    """나무: 등뼈(가장 먼 두 진입점) + 곁가지. (노드 목록, 엣지 목록) 또는 None"""
    a, b = max(((p, q) for p in entries for q in entries),
               key=lambda pq: math.dist(pq[0]['world'], pq[1]['world']))
    spine = dijkstra(drivable, clearance, a['px'], b['px'], res)
    if spine is None:
        print(f'    [X] 등뼈 {a["name"]} -> {b["name"]} 길 없음')
        return None
    print(f'  등뼈 {a["name"]} -> {b["name"]}  {_path_len_m(spine, res):.1f} m')
    chains = [list(spine)]
    rail = np.zeros_like(drivable)
    rail[tuple(np.array(spine).T)] = True
    a['kind'] = b['kind'] = '등뼈 끝'
    spurs = []
    for e in entries:
        if e is a or e is b:
            continue
        path = dijkstra(drivable, clearance, e['px'], rail, res)
        if path is None:
            print(f'    [X] {e["name"]} -> 레일 길 없음')
            e['kind'] = '연결 실패'
            continue
        L = _path_len_m(path, res)
        if L < STUB_M:
            e['kind'] = f'등뼈 옆 {L:.2f} m — 곁가지 없음(BT 인계 거리 {STUB_M:.1f} m 안)'
            continue
        junction = path[-1]
        split_chain_at(chains, junction)
        spur = path[::-1]
        chains.append(spur)
        rail[tuple(np.array(spur).T)] = True
        spurs.append((junction, len(chains) - 1))
        e['kind'] = f'곁가지 {L:.2f} m'

    node_chains = []
    for ch in chains:
        pts = simplify(ch, SIMPLIFY_M / res)
        node_chains.append(merge_chain(pts, meta, shape))
    sharp = 0
    arc_chain_ids = set()
    for j_px, spur_i in spurs:
        before = len(node_chains)
        if not fillet_junction(node_chains, j_px, spur_i, drivable, meta, shape):
            sharp += 1
        arc_chain_ids.update(range(before, len(node_chains)))
    if sharp:
        print(f'  [!] 호를 놓지 못한 접점 {sharp}개 — 뾰족한 채 둔다')
    node_chains = [split_chain_long(fillet_chain(ch, drivable, meta, shape), drivable, meta, shape)
                   for ch in node_chains]

    nodes, by_px = [], {}
    for ch in node_chains:
        for k, nd in enumerate(ch):
            if nd['px'] in by_px:
                ch[k] = by_px[nd['px']]
                continue
            nd['id'] = len(nodes) + 1
            by_px[nd['px']] = nd
            nodes.append(nd)
    edges = [(ch[i], ch[i + 1]) for ch in node_chains for i in range(len(ch) - 1)
             if ch[i] is not ch[i + 1]]
    penalized = {frozenset((ch[i]['id'], ch[i + 1]['id']))
                 for k, ch in enumerate(node_chains) if k in arc_chain_ids
                 for i in range(len(ch) - 1) if ch[i] is not ch[i + 1]}
    return nodes, edges, penalized


def build(name, loop_only, tree=False):
    inner, outer, pad = robot_radii(WS)
    img, meta = load_map(name)
    res = float(meta['resolution'])
    free, specks = free_mask(img)
    clearance = ndimage.distance_transform_edt(free) * res
    turnable = free & (clearance >= outer)
    drivable = free & (clearance >= inner)
    if not drivable.any():
        print('로봇이 설 수 있는 곳이 없다. 지도나 footprint 를 확인하라.')
        return None

    lab, n = ndimage.label(drivable)
    if n > 1:
        big = 1 + int(np.argmax(np.bincount(lab.ravel())[1:]))
        drivable = lab == big
        turnable = turnable & drivable

    dests = load_destinations(name)
    if len(dests) < 2:
        print('목적지가 2개 미만이라 고리를 못 만든다.')
        return None

    entries = []
    for d in dests:
        r, c = to_px(d['x'], d['y'], meta, img.shape)
        r = min(max(r, 0), img.shape[0] - 1)
        c = min(max(c, 0), img.shape[1] - 1)
        rr, cc = nearest_true(drivable, r, c)
        entries.append({'name': d['name'], 'px': (rr, cc),
                        'world': to_world(rr, cc, meta, img.shape),
                        'dest': (d['x'], d['y']), 'yaw': d.get('yaw', 0.0),
                        'can_turn': bool(turnable[rr, cc]), 'kind': ''})

    print(f'  다닐 수 있는 구역 {drivable.sum()*res*res:.1f} m2 '
          f'(그중 회전 가능 {turnable.sum()*res*res:.1f} m2) · 목적지 {len(entries)}개 '
          f'· 자유로 친 회색 잡티 섬 {specks}개')

    if tree:
        built = build_tree(entries, drivable, clearance, meta, img.shape, res)
        if built is None:
            return None
    else:
        built = build_loop(entries, drivable, clearance, meta, img.shape, res, loop_only)
        built = (*built, set())
    nodes, edges, penalized = built

    print('  레일 진입점')
    for e in entries:
        off = math.dist(e['world'], e['dest'])
        print(f'    {e["name"]:<8} ({e["world"][0]:6.2f}, {e["world"][1]:6.2f})  '
              f'목적지에서 {off:.2f} m  yaw {e.get("yaw", 0):4.0f}도  '
              f'{"여기서 돈다" if e["can_turn"] else "여기선 못 돈다"}'
              + (f'  · {e["kind"]}' if e['kind'] else ''))

    for nd in nodes:
        nd['can_turn'] = bool(turnable[nd['px'][0], nd['px'][1]])
    return {'nodes': nodes, 'edges': edges, 'penalized': penalized,
            'entries': entries, 'turnable': turnable,
            'drivable': drivable,
            'free': free, 'clearance': clearance, 'meta': meta,
            'img': img, 'inner': inner, 'outer': outer, 'res': res, 'tree': tree}


def write_geojson(g, out):
    feats = []
    for nd in g['nodes']:
        feats.append({'type': 'Feature',
                      'properties': {'id': nd['id'], 'frame': 'map'},
                      'geometry': {'type': 'Point',
                                   'coordinates': [round(nd['xy'][0], 4),
                                                   round(nd['xy'][1], 4)]}})
    eid = 1000
    for nd, nb in g['edges']:
        pen = frozenset((nd['id'], nb['id'])) in g.get('penalized', set())
        for a, b in ((nd, nb), (nb, nd)):
            eid += 1
            props = {'id': eid, 'startid': a['id'], 'endid': b['id']}
            if pen:
                props['metadata'] = {'penalty': JUNCTION_PENALTY}
            feats.append({'type': 'Feature',
                          'properties': props,
                          'geometry': {'type': 'MultiLineString',
                                       'coordinates': [[[round(a['xy'][0], 4), round(a['xy'][1], 4)],
                                                        [round(b['xy'][0], 4), round(b['xy'][1], 4)]]]}})
    doc = {'type': 'FeatureCollection', 'name': 'vica_route_graph',
           'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::3857'}},
           'features': feats}
    Path(out).write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding='utf-8')
    return len(g['nodes']), eid - 1000


def draw(g, out, name):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for cand in ('NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK JP'):
        try:
            matplotlib.font_manager.findfont(cand, fallback_to_default=False)
            matplotlib.rcParams['font.family'] = cand
            break
        except Exception:
            continue
    matplotlib.rcParams['axes.unicode_minus'] = False

    h, w = g['free'].shape
    rgb = np.zeros((h, w, 3))
    rgb[:] = (0.20, 0.20, 0.22)
    rgb[(g['img'] > 100) & (g['img'] <= 250)] = (0.55, 0.55, 0.57)
    rgb[g['free']] = (0.90, 0.90, 0.92)
    rgb[g['turnable']] = (0.80, 0.90, 0.85)

    fig, ax = plt.subplots(figsize=(max(8.0, w / 90), max(8.0, w / 90) * h / w + 1.0), dpi=150)
    ax.imshow(rgb, interpolation='nearest')
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)

    nodes = g['nodes']
    for nd, nb in g['edges']:
        ax.plot([nd['px'][1], nb['px'][1]], [nd['px'][0], nb['px'][0]],
                '-', color='#1F6F8B', lw=2.4, zorder=3)
    for nd in nodes:
        ax.plot(nd['px'][1], nd['px'][0], 'o', ms=6.5 if nd['can_turn'] else 4.5,
                mfc=('#2E7D5B' if nd['can_turn'] else '#B5721E'),
                mec='white', mew=1.2, zorder=4)
    for e in g['entries']:
        ax.plot(e['px'][1], e['px'][0], 'o', ms=10, mfc='white',
                mec='#A2337E', mew=2.4, zorder=5)
        ax.annotate(e['name'], (e['px'][1], e['px'][0]), textcoords='offset points',
                    xytext=(9, 6), fontsize=9, zorder=6,
                    bbox=dict(boxstyle='round,pad=0.22', fc='white', ec='none', alpha=0.85))

    turn_n = sum(1 for nd in nodes if nd['can_turn'])
    shape = '나무형 레일' if g.get('tree') else '레일'
    ax.set_title(f'{name} — {shape} (노드 {len(nodes)}개, 그중 회전 가능 {turn_n}개)   '
                 f'연한 초록 바닥 = 제자리 회전 가능 구역\n'
                 f'초록 점 = 여기서 돌 수 있다 · 주황 점 = 지나가기만',
                 fontsize=10, pad=10)
    fig.tight_layout()
    fig.savefig(out, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description='nav2_route 용 레일 GeoJSON 을 만든다')
    ap.add_argument('map')
    ap.add_argument('--loop-only', action='store_true')
    ap.add_argument('--tree', action='store_true',
                    help='고리 대신 등뼈+곁가지(일자 복도용)')
    ap.add_argument('--out-dir', default=str(WS / 'maps'))
    args = ap.parse_args()

    print(f'=== {args.map} 레일 만들기 ({"나무형" if args.tree else "고리"}) ===')
    g = build(args.map, args.loop_only, tree=args.tree)
    if g is None:
        return 1
    od = Path(args.out_dir)
    gj = od / f'{args.map}_route.geojson'
    png = od / f'{args.map}_route.png'
    n, e = write_geojson(g, gj)
    draw(g, png, args.map)
    lens = [math.dist(a['xy'], b['xy']) for a, b in g['edges']]
    print()
    print(f'  노드 {n}개 · 엣지 {e}개(양방향) · 레일 총 길이 {sum(lens):.1f} m')
    print(f'  가장 긴 엣지 {max(lens):.2f} m (상한 {MAX_EDGE_M:.2f} m)')
    adj = {}
    for a, b in g['edges']:
        adj.setdefault(a['id'], []).append(b)
        adj.setdefault(b['id'], []).append(a)
    turns = []
    for nd in g['nodes']:
        nbs = adj.get(nd['id'], [])
        if len(nbs) != 2:
            continue
        a, b, c = nbs[0]['xy'], nd['xy'], nbs[1]['xy']
        v1 = (b[0] - a[0], b[1] - a[1]); v2 = (c[0] - b[0], c[1] - b[1])
        turns.append(abs(math.degrees(math.atan2(v1[0]*v2[1] - v1[1]*v2[0], v1[0]*v2[0] + v1[1]*v2[1]))))
    mid = [t for t in turns if 30 <= t <= FILLET_MAX_DEG]
    print(f'  30~{FILLET_MAX_DEG}° 꺾임 남은 노드 {len(mid)}개 (둥글리기 목표 0) · 되돌림(>{FILLET_MAX_DEG}°) {sum(1 for t in turns if t > FILLET_MAX_DEG)}개')
    if g.get('tree'):
        junctions = sum(1 for v in adj.values() if len(v) >= 3)
        print(f'  접점(엣지 3개 이상) {junctions}개 · 고리 수 {len(g["edges"]) - n + 1}개(Y 접점마다 1개가 정상)')
        print(f'  갈림길 호 벌점 엣지 {len(g.get("penalized", ()))}개(penalty {JUNCTION_PENALTY})')
    print(f'  {gj}')
    print(f'  {png}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
