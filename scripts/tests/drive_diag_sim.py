"""drive_diag 시험용 가짜 복도: 벽 선분·레이캐스트·지도 래스터화·PGM 쓰기."""
import math
import os

import numpy as np

# 복도 1.3 m 폭(y = ±0.65), x −1~11 m, 양끝 벽, 2 m 마다 문틀 돌출(복도 방향 관측성)
WALLS = [((-1.0, 0.65), (11.0, 0.65)), ((-1.0, -0.65), (11.0, -0.65)), ((11.0, -0.65), (11.0, 0.65)),
         ((-1.0, -0.65), (-1.0, 0.65))]
for _k in range(0, 11, 2):
    WALLS.append(((_k, 0.65), (_k, 0.50)))
    WALLS.append(((_k + 0.4, -0.65), (_k + 0.4, -0.50)))


def raycast(px, py, heading, n=720, rmax=11.0, noise=0.005, rng=None, walls=WALLS):
    """센서 자세(px, py, heading)에서 본 스캔 → (ranges, angle_min, angle_inc)."""
    ang = -math.pi + np.arange(n) * (2 * math.pi / n)
    dx, dy = np.cos(heading + ang), np.sin(heading + ang)
    r = np.full(n, np.inf)
    for (ax, ay), (bx, by) in walls:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        ok = np.abs(den) > 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((ax - px) * ey - (ay - py) * ex) / den
            s = ((ax - px) * dy - (ay - py) * dx) / den
        hit = ok & (t > 0) & (s >= 0) & (s <= 1)
        r = np.where(hit & (t < r), t, r)
    if rng is not None:
        r = r + rng.normal(0, noise, n)
    r[r > rmax] = np.inf
    return r, -math.pi, 2 * math.pi / n


def rasterize(walls=WALLS, res=0.05, margin=1.0):
    """벽 선분 → OccupancyGrid 식 배열(100 점유, 0 빈칸), 0행 = origin_y 쪽. → (data, origin_x, origin_y)."""
    xs = [p[0] for w in walls for p in w]
    ys = [p[1] for w in walls for p in w]
    # 벽(res 의 배수 좌표)이 칸 가운데를 지나게 원점을 반 칸 옮긴다 — 래스터화 자체의 치우침을 없앤다
    ox = math.floor((min(xs) - margin) / res) * res - res / 2
    oy = math.floor((min(ys) - margin) / res) * res - res / 2
    W = int(math.ceil((max(xs) + margin - ox) / res))
    H = int(math.ceil((max(ys) + margin - oy) / res))
    data = np.zeros((H, W), dtype=np.int8)
    for (ax, ay), (bx, by) in walls:
        n = int(math.hypot(bx - ax, by - ay) / (res / 4)) + 2
        for s in np.linspace(0, 1, n):
            c = int((ax + s * (bx - ax) - ox) / res)
            r = int((ay + s * (by - ay) - oy) / res)
            data[r, c] = 100
    return data, ox, oy


def write_map(folder, data, ox, oy, res, name='sim_map'):
    """OccupancyGrid 식 배열을 map_server 형식(PGM trinary + yaml)으로 쓴다. 이미지 0행 = 위(최대 y)."""
    img = np.where(data >= 65, 0, np.where(data == 0, 254, 205)).astype(np.uint8)[::-1]
    H, W = img.shape
    pgm = os.path.join(folder, f'{name}.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())
    yml = os.path.join(folder, f'{name}.yaml')
    with open(yml, 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\norigin: [{ox}, {oy}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yml
