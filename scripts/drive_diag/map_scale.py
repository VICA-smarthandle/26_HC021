#!/usr/bin/env python3
"""지도 척도(길이) 고치기 — 지도와 그 지도에 묶인 레일·금지구역 좌표를 같은 배율로(2026-11-02)."""
import argparse
import datetime as dt
import json
import math
import os
import shutil
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAPS = os.path.join(REPO, 'vica_ros2_ws', 'maps')
DEST_ROOT = os.path.expanduser('~/vica_data/destinations')
DEST_FILES = ('destinations.yaml', 'home.yaml')
OCC, FREE, UNKNOWN = 0, 254, 205


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'P5 PGM 만'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        a = np.frombuffer(f.read(), np.uint8).reshape(h, w).copy()
    return a, maxval


def write_pgm(path, a, maxval=255):
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n%d\n' % (a.shape[1], a.shape[0], maxval))
        f.write(np.ascontiguousarray(a, np.uint8).tobytes())


def scale_grid(a, origin, res, s, priority=(OCC, FREE, UNKNOWN)):
    """(0, 0) 중심 배율 s. 반환 (새 그림, 새 원점 [x, y]). 해상도는 그대로."""
    h, w = a.shape
    ox, oy = origin[0] * s, origin[1] * s
    W, H = int(math.ceil(w * s)), int(math.ceil(h * s))
    if s <= 1.0:
        rr, cc = np.mgrid[0:h, 0:w]
        x = origin[0] + (cc + 0.5) * res
        y = origin[1] + (h - 1 - rr + 0.5) * res
        c2 = np.clip(((s * x - ox) / res).astype(int), 0, W - 1)
        r2 = np.clip(H - 1 - ((s * y - oy) / res).astype(int), 0, H - 1)
        vals = np.unique(a)
        rank = {v: (priority.index(v) if v in priority else len(priority)) for v in vals.tolist()}
        out = np.full((H, W), 255, np.int32)
        best = np.full((H, W), 10 ** 6, np.int32)
        for v in sorted(vals.tolist(), key=lambda v: -rank[v]):
            m = a == v
            out[r2[m], c2[m]] = v
            best[r2[m], c2[m]] = rank[v]
        if (out == 255).any():
            out[out == 255] = UNKNOWN if UNKNOWN in vals else int(np.bincount(a.ravel()).argmax())
        return out.astype(np.uint8), [ox, oy]
    rr, cc = np.mgrid[0:H, 0:W]
    x = ox + (cc + 0.5) * res
    y = oy + (H - 1 - rr + 0.5) * res
    c0 = np.clip(((x / s - origin[0]) / res).astype(int), 0, w - 1)
    r0 = np.clip(h - 1 - ((y / s - origin[1]) / res).astype(int), 0, h - 1)
    return a[r0, c0], [ox, oy]


def scale_json(obj, s):
    """x·y 키와 GeoJSON coordinates 의 숫자를 s 배. 다른 값은 그대로."""
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if k in ('x', 'y') and isinstance(v, (int, float)) and not isinstance(v, bool):
                out[k] = round(v * s, 4)
            elif k == 'coordinates':
                out[k] = _scale_coords(v, s)
            else:
                out[k] = scale_json(v, s)
        return out
    if isinstance(obj, list):
        return [scale_json(v, s) for v in obj]
    return obj


def _scale_coords(c, s):
    if isinstance(c, list) and c and all(isinstance(v, (int, float)) for v in c):
        return [round(v * s, 4) for v in c[:2]] + list(c[2:])
    return [_scale_coords(v, s) for v in c] if isinstance(c, list) else c


def occupied_points(map_id, maps_dir=MAPS):
    y = yaml.safe_load(open(os.path.join(maps_dir, map_id + '.yaml')))
    a, _ = read_pgm(os.path.join(maps_dir, y['image']))
    h = a.shape[0]
    rr, cc = np.where(a < 50)
    r, (ox, oy) = y['resolution'], y['origin'][:2]
    return np.column_stack([ox + (cc + 0.5) * r, oy + (h - 1 - rr + 0.5) * r])


def register(src, dst, keep=0.7):
    """src → dst 아핀(A, t). 처음 자리는 이동만 격자로 찾는다(회전은 작다고 봄)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(dst)

    def icp(A, t, iters):
        for _ in range(iters):
            d, j = tree.query(src @ A.T + t)
            sel = d <= np.quantile(d, keep)
            X = np.column_stack([src[sel], np.ones(sel.sum())])
            sol, *_ = np.linalg.lstsq(X, dst[j[sel]], rcond=None)
            A, t = sol[:2].T, sol[2]
        d, _ = tree.query(src @ A.T + t)
        return A, t, d

    c0 = dst.mean(0) - src.mean(0)
    best = None
    for dx in np.arange(-3, 3.01, 0.5):
        for dy in np.arange(-3, 3.01, 0.5):
            A, t, d = icp(np.eye(2), c0 + [dx, dy], 15)
            sc = float(np.mean(d < 0.10))
            if best is None or sc > best[0]:
                best = (sc, A, t)
    A, t, d = icp(best[1], best[2], 80)
    return A, t, d


def files_of(map_id, maps_dir=MAPS):
    names = [f'{map_id}.yaml', f'{map_id}.pgm', f'{map_id}.png', f'{map_id}.meta.json',
             f'{map_id}_keepout.yaml', f'{map_id}_keepout.pgm', f'{map_id}_keepout.json',
             f'{map_id}_route.geojson', f'{map_id}_route_edit.json']
    return [n for n in names if os.path.exists(os.path.join(maps_dir, n))]


def build(map_id, s, out_dir, maps_dir=MAPS):
    """out_dir 에 배율 s 판을 쓴다. 바뀐 파일 이름 목록."""
    os.makedirs(out_dir, exist_ok=True)
    done = []
    for base in (map_id, map_id + '_keepout'):
        yp = os.path.join(maps_dir, base + '.yaml')
        if not os.path.exists(yp):
            continue
        y = yaml.safe_load(open(yp))
        a, mx = read_pgm(os.path.join(maps_dir, y['image']))
        b, origin = scale_grid(a, y['origin'], float(y['resolution']), s)
        write_pgm(os.path.join(out_dir, y['image']), b, mx)
        y['origin'] = [round(origin[0], 4), round(origin[1], 4), y['origin'][2] if len(y['origin']) > 2 else 0]
        with open(os.path.join(out_dir, base + '.yaml'), 'w') as f:
            for k in ('image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
                if k in y:
                    v = y[k]
                    f.write(f'{k}: {json.dumps(v) if isinstance(v, list) else v}\n')
        done += [base + '.yaml', y['image']]
        if base == map_id and os.path.exists(os.path.join(maps_dir, map_id + '.png')):
            from PIL import Image
            Image.fromarray(b, mode='L').save(os.path.join(out_dir, map_id + '.png'))
            done.append(map_id + '.png')
    for n in (f'{map_id}_route.geojson', f'{map_id}_route_edit.json', f'{map_id}_keepout.json'):
        p = os.path.join(maps_dir, n)
        if os.path.exists(p):
            with open(os.path.join(out_dir, n), 'w') as f:
                json.dump(scale_json(json.load(open(p)), s), f, ensure_ascii=False, indent=2)
            done.append(n)
    mp = os.path.join(maps_dir, f'{map_id}.meta.json')
    if os.path.exists(mp):
        meta = json.load(open(mp))
        meta.setdefault('scale_history', []).append(
            dict(scale=s, center=[0, 0], at=dt.datetime.now().isoformat(timespec='seconds'),
                 note='지도 길이 보정(drive_diag/map_scale.py) — 레일·금지구역 좌표도 같은 배율'))
        with open(os.path.join(out_dir, f'{map_id}.meta.json'), 'w') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        done.append(f'{map_id}.meta.json')
    return done


def build_destinations(map_id, s, out_dir, dest_root=DEST_ROOT):
    """목적지·홈 yaml 의 x·y 를 s 배해 out_dir/destinations/ 에 쓴다. 바뀐 파일 이름 목록."""
    src = os.path.join(dest_root, map_id)
    done = []
    for n in DEST_FILES:
        p = os.path.join(src, n)
        if not os.path.exists(p):
            continue
        d = yaml.safe_load(open(p, encoding='utf-8')) or {}
        os.makedirs(os.path.join(out_dir, 'destinations'), exist_ok=True)
        with open(os.path.join(out_dir, 'destinations', n), 'w', encoding='utf-8') as f:
            yaml.safe_dump(scale_json(d, s), f, allow_unicode=True, sort_keys=False)
        done.append(n)
    return done


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_id')
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--to', help='기준 지도 이름 — 배율만 추정해 보여 준다')
    g.add_argument('--scale', type=float, help='배율(새 좌표 = 배율 × 옛 좌표)')
    ap.add_argument('--apply', action='store_true', help='지도 폴더에 덮어쓴다(원본은 ~/vica_data/map_backup 에 복사)')
    ap.add_argument('--maps', default=MAPS)
    ap.add_argument('--dest-root', default=DEST_ROOT, help='목적지·홈 폴더 뿌리(빈 문자열이면 안 바꿈)')
    a = ap.parse_args(argv)
    if a.to:
        A, t, d = register(occupied_points(a.map_id, a.maps), occupied_points(a.to, a.maps))
        sx, sy = math.hypot(A[0, 0], A[1, 0]), math.hypot(A[0, 1], A[1, 1])
        rot = math.degrees(math.atan2(A[1, 0] - A[0, 1], A[0, 0] + A[1, 1]))
        print(f'{a.map_id} → {a.to}: x 배율 {sx:.4f} · y 배율 {sy:.4f} · 회전 {rot:+.2f}° · '
              f'10 cm 안 점 {np.mean(d < 0.10) * 100:.0f} % · 중앙 거리 {np.median(d) * 100:.1f} cm')
        print(f'→ 길이만 맞추려면 --scale {(sx + sy) / 2:.4f}')
        return 0
    if not 0.5 < a.scale < 2.0:
        raise SystemExit(f'배율이 이상하다: {a.scale}')
    tag = f'{a.map_id}_s{a.scale:.4f}'
    out = os.path.expanduser(f'~/vica_data/map_scale/{tag}')
    done = build(a.map_id, a.scale, out, a.maps)
    dests = build_destinations(a.map_id, a.scale, out, a.dest_root) if a.dest_root else []
    print(f'시험본: {out}\n  ' + ' · '.join(done + [f'destinations/{n}' for n in dests]))
    if a.apply:
        bk = os.path.expanduser(f"~/vica_data/map_backup/{a.map_id}_{dt.datetime.now():%m%d_%H%M%S}")
        os.makedirs(bk, exist_ok=True)
        for n in files_of(a.map_id, a.maps):
            shutil.copy2(os.path.join(a.maps, n), bk)
        for n in done:
            shutil.copy2(os.path.join(out, n), os.path.join(a.maps, n))
        for n in dests:
            os.makedirs(os.path.join(bk, 'destinations'), exist_ok=True)
            dp = os.path.join(a.dest_root, a.map_id, n)
            shutil.copy2(dp, os.path.join(bk, 'destinations', n))
            shutil.copy2(os.path.join(out, 'destinations', n), dp)
        print(f'적용: {a.maps} ← {len(done)}개 · 목적지·홈 {len(dests)}개 · 원본 백업 {bk}\n'
              'Nav2 를 다시 띄워야 새 지도를 읽는다. 목적지는 /vica/mission/reload_destinations 로 다시 읽힌다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
