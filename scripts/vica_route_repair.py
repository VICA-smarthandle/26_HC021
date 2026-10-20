#!/usr/bin/env python3
"""레일(route geojson)이 벽에 너무 가까운 곳을 밀어낸다."""
import argparse, json, math, shutil, sys
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

WS = Path(__file__).resolve().parent.parent / 'vica_ros2_ws'
MAX_MOVE = 0.50
STEP = 0.05
SEARCH = 0.05


def load(map_name):
    img = np.array(Image.open(WS / 'maps' / f'{map_name}.pgm'))
    meta = yaml.safe_load((WS / 'maps' / f'{map_name}.yaml').read_text())
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    dist = ndimage.distance_transform_edt(~(img == 0)) * res
    H, W = img.shape

    def clear(x, y):
        c = int((x - ox) / res)
        r = H - 1 - int((y - oy) / res)
        if 0 <= r < H and 0 <= c < W:
            return float(dist[r, c])
        return float('nan')
    return clear


def edge_clear(clear, A, B):
    """두 점을 잇는 직선의 최소 여유."""
    L = math.hypot(B[0] - A[0], B[1] - A[1])
    n = max(int(L / STEP), 2)
    vals = []
    for t in np.linspace(0, 1, n):
        c = clear(A[0] + (B[0] - A[0]) * t, A[1] + (B[1] - A[1]) * t)
        if np.isfinite(c):
            vals.append(c)
    return min(vals) if vals else float('nan')


def node_score(clear, nid, pos, nbrs, N):
    """그 노드가 낀 모든 간선 + 자기 자신의 최소 여유."""
    vals = [clear(*pos)]
    for m in nbrs[nid]:
        vals.append(edge_clear(clear, pos, N[m]))
    vals = [v for v in vals if np.isfinite(v)]
    return min(vals) if vals else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('--min-clear', type=float, default=0.70)
    ap.add_argument('--apply', action='store_true', help='실제로 파일을 고친다(기본은 점검만)')
    A = ap.parse_args()

    gpath = WS / 'maps' / f'{A.map}_route.geojson'
    gj = json.loads(gpath.read_text())
    N = {f['properties']['id']: list(f['geometry']['coordinates'])
         for f in gj['features'] if f['geometry']['type'] == 'Point'}
    E = sorted({tuple(sorted((f['properties']['startid'], f['properties']['endid'])))
                for f in gj['features'] if f['geometry']['type'] != 'Point'})
    nbrs = {k: [] for k in N}
    for a, b in E:
        nbrs[a].append(b)
        nbrs[b].append(a)
    clear = load(A.map)

    def report(tag):
        worst = sorted((edge_clear(clear, N[a], N[b]), a, b) for a, b in E)
        bad = [w for w in worst if w[0] < A.min_clear]
        print(f'  [{tag}] 간선 최소 {worst[0][0]:.2f} m · '
              f'{A.min_clear} m 미만 {len(bad)}/{len(E)}개')
        return worst, bad

    print(f'== {A.map} : 노드 {len(N)} · 간선 {len(E)} · 기준 {A.min_clear} m ==')
    worst0, bad0 = report('수리 전')
    for mn, a, b in worst0[:8]:
        if mn < A.min_clear:
            print(f'      {mn:.2f} m   {a} - {b}')

    if not bad0:
        print('  고칠 것이 없다.')
        return 0

    moved = {}
    for _ in range(6):
        targets = set()
        for mn, a, b in ((edge_clear(clear, N[a], N[b]), a, b) for a, b in E):
            if mn < A.min_clear:
                targets.add(a)
                targets.add(b)
        if not targets:
            break
        改 = False
        for nid in sorted(targets):
            x0, y0 = N[nid]
            base = node_score(clear, nid, [x0, y0], nbrs, N)
            best, bp = base, None
            rng = np.arange(-MAX_MOVE, MAX_MOVE + 1e-9, SEARCH)
            for dx in rng:
                for dy in rng:
                    if math.hypot(dx, dy) > MAX_MOVE:
                        continue
                    p = [x0 + dx, y0 + dy]
                    s = node_score(clear, nid, p, nbrs, N)
                    if not np.isfinite(s):
                        continue
                    if s > best + 1e-4:
                        best, bp = s, p
            if bp is not None:
                N[nid] = bp
                moved[nid] = (math.hypot(bp[0] - x0, bp[1] - y0), base, best)
                改 = True
        if not 改:
            break

    worst1, bad1 = report('수리 후')
    print(f'  옮긴 노드 {len(moved)}개')
    for nid, (d, b0, b1) in sorted(moved.items(), key=lambda kv: -kv[1][0])[:10]:
        print(f'      {nid:>3}  {d*100:5.1f} cm 이동   여유 {b0:.2f} -> {b1:.2f} m')
    if bad1:
        print('  ※ 아직 기준 미달인 간선:')
        for mn, a, b in bad1:
            print(f'      {mn:.2f} m   {a} - {b}')

    dfile = Path.home() / 'vica_data' / 'destinations' / A.map / 'destinations.yaml'
    if dfile.is_file():
        far = []
        for d in (yaml.safe_load(dfile.read_text()) or {}).get('destinations', []):
            po = d.get('pose') or {}
            if po.get('x') is None:
                continue
            dd = min(math.dist((po['x'], po['y']), tuple(v)) for v in N.values())
            if dd > 0.60:
                far.append((d.get('name'), round(dd, 2)))
        print(f'  목적지 검사: {"통과" if not far else f"※ 레일에서 먼 목적지 {far}"}')

    if not A.apply:
        print('\n  점검만 했다. 실제로 고치려면 --apply 를 붙인다.')
        return 0

    shutil.copy2(gpath, gpath.with_suffix('.geojson.bak'))
    for f in gj['features']:
        if f['geometry']['type'] == 'Point':
            f['geometry']['coordinates'] = N[f['properties']['id']]
    gpath.write_text(json.dumps(gj, ensure_ascii=False, indent=1))
    print(f'\n  고쳤다. 원본은 {gpath.name}.bak')
    return 0


if __name__ == '__main__':
    sys.exit(main())
