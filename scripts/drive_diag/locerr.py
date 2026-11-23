"""x·y·θ 오차 계산(ROS 없이 시험). bag 처리는 analyze_loc.py, 화면은 serve.py.

① 위치추정 오차 = AMCL 자세 − 스캔이 지도에 가장 잘 맞는 자세
   AMCL 자세 주변만 좁게(기본 ±25 cm·±6°, 3 단계) 훑는다. 잣대는 AMCL·pose_bootstrap 과 같은
   vica_nav2/pose_score.py 의 likelihood field·sigma_hit 0.2 이고, 거리 필드만 칸 사이를 선형 보간해 읽는다
   (칸 단위로 읽으면 지도 해상도 5 cm 의 절반인 2.5 cm 계단이 생겨 오차 잣대로 못 쓴다).
   찾은 자세가 탐색 범위 끝에 걸리면 edge=True — 오차가 범위보다 크거나 대조가 못 미더운 것.
② 오도메트리 오차 = EKF·바퀴가 믿은 이동 − 스캔-스캔 ICP 이동(구간 시작 기준)
   지도를 쓰지 않는다. 라이다 이동을 base_link→laser_frame TF 로 차체 이동으로 바꾼다.

부호: 옆(lat)·방향(dyaw) 은 + = 왼쪽(반시계). 앞(fwd)·옆은 기준 자세의 방향으로 나눈다.
"""
import collections
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / 'vica_ros2_ws' / 'src' / 'vica_nav2'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vica_nav2 import pose_score as ps  # noqa: E402
import laser_yaw  # noqa: E402

scan_points = laser_yaw.scan_points


# ── 자세 연산 (x, y, yaw) ────────────────────────────────

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """a 기준으로 본 b."""
    return compose(inverse(a), b)


def pose_error(est, ref):
    """est − ref. 앞·옆은 ref 방향 기준(+ 옆 = 왼쪽)."""
    dx, dy = est[0] - ref[0], est[1] - ref[1]
    c, s = math.cos(ref[2]), math.sin(ref[2])
    return dict(dx=dx, dy=dy, dyaw=wrap(est[2] - ref[2]), fwd=dx * c + dy * s, lat=-dx * s + dy * c)


# ── 지도 ─────────────────────────────────────────────────

def _read_pgm(path):
    with open(path, 'rb') as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError(f'{path}: 8 bit P5 PGM 만 읽는다')
    return np.frombuffer(raw[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)


def load_map(yaml_path):
    """map_server 지도(yaml+pgm) → (pose_score.MapGrid, meta). map_server trinary 규칙과 같다."""
    with open(yaml_path) as f:
        y = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), y['image'])
    img = _read_pgm(img_path).astype(np.float64)
    p = img / 255.0 if y.get('negate', 0) else (255.0 - img) / 255.0
    data = np.full(img.shape, -1, dtype=np.int8)
    data[p > float(y.get('occupied_thresh', 0.65))] = 100
    data[p < float(y.get('free_thresh', 0.25))] = 0
    data = np.ascontiguousarray(data[::-1])  # 이미지 0행 = 위 → 격자 0행 = origin_y 쪽
    res, ox, oy = float(y['resolution']), float(y['origin'][0]), float(y['origin'][1])
    meta = dict(resolution=res, origin=[ox, oy], width=int(data.shape[1]), height=int(data.shape[0]),
                image=img_path, yaml=os.path.abspath(yaml_path))
    return ps.MapGrid(data, res, ox, oy), meta


# ── ① 지도 대조 ──────────────────────────────────────────

Match = collections.namedtuple('Match', 'x y yaw score amcl_score edge')


def beams_from_scan(ranges, angle_min, angle_inc):
    return ps.filter_beams(ranges, angle_min, angle_inc)


def _axis(c, r, step):
    n = int(round(r / step))
    return c + step * np.arange(-n, n + 1)


def _lookup_bilinear(field, grid, wx, wy, max_dist):
    """거리 필드를 칸 중심 사이 선형 보간으로 읽는다(pose_score 는 칸 단위 — AMCL 과 같지만 2.5 cm 계단)."""
    fx = (wx - grid.origin_x) / grid.resolution - 0.5
    fy = (wy - grid.origin_y) / grid.resolution - 0.5
    c0, r0 = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
    tx, ty = fx - c0, fy - r0
    H, W = field.shape

    def at(r, c):
        ok = (r >= 0) & (r < H) & (c >= 0) & (c < W)
        out = np.full(r.shape, float(max_dist))
        out[ok] = field[r[ok], c[ok]]
        return out

    return (at(r0, c0) * (1 - tx) * (1 - ty) + at(r0, c0 + 1) * tx * (1 - ty)
            + at(r0 + 1, c0) * (1 - tx) * ty + at(r0 + 1, c0 + 1) * tx * ty)


def _sweep(field, grid, beams, xs, ys, yaws, sensor):
    """(xs × ys × yaws) 전부 채점 → 평평한 배열 (x, y, yaw, 점수 0~1). 변환 순서는 pose_score 와 같다."""
    bx, by = beams.ranges * np.cos(beams.angles), beams.ranges * np.sin(beams.angles)
    mx, my = np.meshgrid(xs, ys, indexing='ij')
    mx, my = mx.ravel(), my.ravel()
    sx, sy, syaw = sensor
    out = np.empty((len(yaws), mx.size))
    for i, yaw in enumerate(yaws):
        c, s = math.cos(yaw), math.sin(yaw)
        cb, sb = math.cos(yaw + syaw), math.sin(yaw + syaw)
        rx = c * sx - s * sy + cb * bx - sb * by
        ry = s * sx + c * sy + sb * bx + cb * by
        d = _lookup_bilinear(field, grid, mx[:, None] + rx[None, :], my[:, None] + ry[None, :], ps.MAX_DIST)
        out[i] = np.exp(-(d * d) / (2.0 * ps.SIGMA_HIT ** 2)).mean(axis=1)
    return np.tile(mx, len(yaws)), np.tile(my, len(yaws)), np.repeat(yaws, mx.size), out.ravel()


def local_match(field, grid, beams, guess, sensor, xy_radius=0.25, yaw_radius=math.radians(6.0)):
    """guess(AMCL 자세) 주변에서 스캔이 지도에 가장 잘 맞는 자세. 점수 0~100.

    3 단계: ±xy_radius·±yaw_radius 를 5 cm·1° 로 훑고 → 1등 주변 ±5 cm·±1° 를 1.25 cm·0.25° →
    ±1.25 cm·±0.25° 를 2.5 mm·0.05°. 1 단계 1등이 범위 끝에 걸리면 edge=True.
    """
    gx, gy, gyaw = guess
    if beams.used == 0:
        return Match(gx, gy, gyaw, 0.0, 0.0, True)
    amcl_score = _sweep(field, grid, beams, np.array([gx]), np.array([gy]), np.array([gyaw]), sensor)[3][0] * 100
    stages = [(xy_radius, 0.05, yaw_radius, math.radians(1.0)),
              (0.05, 0.0125, math.radians(1.0), math.radians(0.25)),
              (0.0125, 0.0025, math.radians(0.25), math.radians(0.05))]
    cx, cy, cyaw, edge = gx, gy, gyaw, False
    for k, (r, st, yr, yst) in enumerate(stages):
        ax, ay, ayaw, sc = _sweep(field, grid, beams, _axis(cx, r, st), _axis(cy, r, st), _axis(cyaw, yr, yst), sensor)
        i = int(np.argmax(sc))
        if k == 0:
            edge = (abs(ax[i] - gx) > r - st / 2 or abs(ay[i] - gy) > r - st / 2 or abs(ayaw[i] - gyaw) > yr - yst / 2)
        cx, cy, cyaw, best = float(ax[i]), float(ay[i]), float(ayaw[i]), float(sc[i])
    return Match(cx, cy, wrap(cyaw), best * 100.0, float(amcl_score), bool(edge))


# ── ② 라이다 오도메트리 ──────────────────────────────────

def lidar_chain(keyscans, guesses, sensor):
    """키 스캔 사이 점-선 ICP 를 이어 차체(base_link) 자세 목록(첫 자세 = 0). guesses: 차체 상대 이동 초기값."""
    T, Ti = tuple(sensor), inverse(sensor)
    pose, poses, inl = (0.0, 0.0, 0.0), [(0.0, 0.0, 0.0)], []
    for k in range(len(keyscans) - 1):
        dl0 = compose(compose(Ti, guesses[k]), T)
        tx, ty, th, fr = laser_yaw.icp(keyscans[k + 1], keyscans[k], dl0)
        db = compose(compose(T, (tx, ty, th)), Ti)
        pose = compose(pose, db)
        poses.append(pose)
        inl.append(fr)
    return poses, inl


def _interp_pose(t, tt, P):
    P = np.asarray(P, float)
    yaw = np.unwrap(P[:, 2])
    return np.c_[np.interp(t, tt, P[:, 0]), np.interp(t, tt, P[:, 1]), np.interp(t, tt, yaw)]


def odom_error(t_ref, ref_poses, t_est, est_poses, t0):
    """구간 시작 t0 부터 est(EKF·바퀴) 상대 이동 − ref(라이다) 상대 이동. 기준 시각은 t_ref(≥ t0)."""
    t_ref = np.asarray(t_ref, float)
    sel = t_ref >= t0
    ts = t_ref[sel]
    R = _interp_pose(ts, t_ref, ref_poses)
    E = _interp_pose(ts, np.asarray(t_est, float), est_poses)
    e0 = tuple(_interp_pose([t0], np.asarray(t_est, float), est_poses)[0])
    # 구간 시작의 라이다 자세 = 바로 앞 키프레임 + 그 뒤 EKF 이동(짧다). 키프레임 사이를 시간 비례로 보간하면
    # 긴 정지 뒤 출발에서 '이미 움직였다'고 잘못 잡힌다(11-01 54.6 s 시작 −18.5 cm 버그).
    k = max(int(np.searchsorted(t_ref, t0, side='right')) - 1, 0)
    ek = tuple(_interp_pose([t_ref[k]], np.asarray(t_est, float), est_poses)[0])
    r0 = compose(tuple(np.asarray(ref_poses, float)[k]), relative(ek, e0))
    out = {k: np.empty(len(ts)) for k in ('dx', 'dy', 'dyaw', 'fwd', 'lat')}
    for i in range(len(ts)):
        rr = relative(r0, tuple(R[i]))
        er = relative(e0, tuple(E[i]))
        e = pose_error(er, rr)
        for k in out:
            out[k][i] = e[k]
    out['t'] = ts
    return out


# ── 시각 ─────────────────────────────────────────────────

def tf_time_shift(mo_stamp, mo_rt, od_stamp, od_rt, scan_stamps=None):
    """AMCL 은 map→odom 을 '스캔 시각 + transform_tolerance' 로 찍는다. 그 몫(초)을 돌려준다.

    대략값 = 기록 시각과의 차(처리 지연이 섞여 0.1 s 쯤 작게 나온다). scan_stamps 를 주면 대략값 ±0.3 s 안에서
    'map→odom 시각 − 후보' 가 스캔 시각과 1 ms 안으로 가장 많이 겹치는 후보(0.05 s 간격)를 고른다.
    """
    d = float(np.median(np.asarray(mo_stamp) - np.asarray(mo_rt)) - np.median(np.asarray(od_stamp) - np.asarray(od_rt)))
    if d <= 0.3:
        return 0.0
    if scan_stamps is None or len(scan_stamps) == 0:
        return round(d, 1)
    sc = np.sort(np.asarray(scan_stamps, float))
    mo = np.asarray(mo_stamp, float)
    best, hits = round(d, 1), -1
    for c in np.round(np.arange(round(d, 1) - 0.3, round(d, 1) + 0.3001, 0.05), 3):
        q = mo - c
        i = np.clip(np.searchsorted(sc, q), 1, len(sc) - 1)
        near = np.minimum(np.abs(sc[i] - q), np.abs(sc[i - 1] - q))
        n = int((near < 1e-3).sum())
        if n > hits:
            best, hits = float(c), n
    return best


def select_keyframes(poses, step=0.25, yaw_step=math.radians(3.0)):
    """앞 키프레임에서 step(m) 이상 움직이거나 yaw_step 이상 돌면 새 키프레임. → 인덱스 목록."""
    idx, last = [0], tuple(poses[0])
    for i in range(1, len(poses)):
        r = relative(last, tuple(poses[i]))
        if math.hypot(r[0], r[1]) >= step - 1e-9 or abs(wrap(r[2])) >= yaw_step - 1e-9:
            idx.append(i)
            last = tuple(poses[i])
    return idx


def static_pose(entries, base, child):
    """/tf_static (부모, 자식, (x, y, yaw)) 목록에서 base 기준 child 자세. 사슬이 끊기면 None."""
    up = {c: (p, tuple(x)) for p, c, x in entries}
    T, cur = (0.0, 0.0, 0.0), child
    for _ in range(len(up) + 1):
        if cur == base:
            return T
        if cur not in up:
            return None
        cur, pose = up[cur]
        T = compose(pose, T)
    return None
