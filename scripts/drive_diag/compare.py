#!/usr/bin/env python3
"""주행 bag 비교 — AMCL 보정량·위치추정 오차·지도 정합·흔들림을 표 하나로(2026-11-02).

    source /opt/ros/humble/setup.bash
    # 첫 bag = 새 주행, 나머지 = 비교 기준. bag 은 폴더 경로나 ~/vica_data/bags 아래 이름.
    python3 scripts/drive_diag/compare.py 1102_HHMMSS_map_1102_170853_nav \\
        1101_170326_laser_yaw_drive@vica_map_0630 1101_171752_laser_yaw_drive@vica_map_0630

지도: `bag@지도` (지도 이름 또는 .yaml 경로). 안 주면 bag 이름 안의 지도 이름(record.sh 가 붙임)으로 찾는다.
계산은 analyze_loc.py 캐시(~/vica_data/drive_diag_cache)를 쓰므로 처음 한 번은 bag 길이만큼 걸린다.

모든 값은 **직진 구간**만 센다 — evaluate.segment_motion 의 전진·후진(1 s 평균 |v| > 0.1·|자이로| < 0.15, 3 s 이상), 양 끝 1 s 는
뺀다(회전 직후 AMCL 이 방향을 따라잡는 보정 제외). 실제 궤적(EKF)이 시작-끝 직선에서 20 cm(--curve-cm) 넘게 휜 구간은 플래너가
돌아간 길로 보고 뺀다. Nav2·teleop 처럼 주행 방식이 달라도 견줄 수 있게.
기본은 **전진만**(--dir) — Nav2 는 전진만 하고, 11-01 teleop 기준 bag 에서 후진은 AMCL 방향 보정 |합|이 전진의 약 2배였다
(전진 4.0~4.1 · 후진 8.7~8.8 °/m).
흔들림은 주행 방식에 따라 다르다(teleop w=0 은 컨트롤러가 없어 작다) — Nav2 주행끼리, 또는 맨바닥 Nav2 2.4~3.5° 와 견준다.
초기 위치 지정(/initialpose) 앞뒤 5 s 의 AMCL 보정은 뺀다(사람이 옮긴 것).

항목
  AMCL 순 보정   map→odom 이 바뀐 양을 로봇 자리에서 본 것의 합 ÷ 직진 거리. 옆·앞은 진행 방향 기준(왼쪽 +).
                 오도메트리가 한쪽으로 꾸준히 틀리면 커진다.
  AMCL 1회 크기  보정 한 번의 |옆|·|방향| 95 % 값 — 작을수록 AMCL 이 덜 튄다(지도·라이다가 잘 맞음).
  방향 |합|/m    보정 방향의 절댓값 합 ÷ 거리 — 순 보정과 차이가 클수록 왔다 갔다(잡음).
  ① 위치 오차    AMCL 자세 − 지도 대조 자세(스캔이 지도에 가장 잘 맞는 자세). 95 % 값.
  지도 정합      그 자리 스캔이 지도에 맞는 정도(pose_score, %). 새 지도가 맞으면 올라간다.
  흔들림         직선 4 s 이상 구간 차체 방향 흔들림 폭 중앙(°) — 맨바닥 2.4~3.5°, 카펫 6.4~7.2°.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))

import analyze_loc  # noqa: E402
import evaluate  # noqa: E402
from scripts.drive_diag.metrics import amcl_corrections  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
MAPS = os.path.join(REPO, 'vica_ros2_ws', 'maps')
BAGS = os.path.expanduser('~/vica_data/bags')
CACHE = os.path.expanduser('~/vica_data/drive_diag_cache')
INIT_GUARD_S = 5.0
CHANGE = 0.20   # 비교 판정: 기준 대비 20 % 이상 바뀌면 '줄었음/늘었음'

# (키, 이름, 단위, 형식, 좋은 방향) — 좋은 방향 -1 = 작을수록 좋음, +1 = 클수록 좋음, 0 = 0 에 가까울수록 좋음
ROWS = [
    ('dist', '직진 거리', 'm', '.1f', None),
    ('n_per_m', 'AMCL 보정 횟수', '회/m', '.1f', None),
    ('side_net', 'AMCL 순 보정 옆', 'cm/m', '+.2f', 0),
    ('fwd_net', 'AMCL 순 보정 앞', 'cm/m', '+.2f', 0),
    ('yaw_net', 'AMCL 순 보정 방향', '°/m', '+.3f', 0),
    ('side_p95', 'AMCL 1회 옆 95 %', 'cm', '.2f', -1),
    ('yaw_p95', 'AMCL 1회 방향 95 %', '°', '.2f', -1),
    ('yaw_gross', 'AMCL 방향 |합|', '°/m', '.2f', -1),
    ('loc_lat_p95', '① 위치 오차 옆 95 %', 'cm', '.1f', -1),
    ('loc_yaw_p95', '① 위치 오차 방향 95 %', '°', '.2f', -1),
    ('score_amcl', '지도 정합(AMCL 자세)', '%', '.1f', +1),
    ('score_best', '지도 정합(최고)', '%', '.1f', +1),
    ('wobble', '흔들림 폭(중앙)', '°', '.1f', -1),
]


# ── bag·지도 찾기 ───────────────────────────────────────────
def resolve_bag(arg):
    """'bag[@지도]' → (bag 폴더, 지도 yaml 또는 None)."""
    bag, _, mp = arg.partition('@')
    path = bag if os.path.isdir(bag) else os.path.join(BAGS, bag)
    if not os.path.isdir(path):
        raise SystemExit(f'bag 폴더 없음: {bag}')
    return os.path.normpath(path), (resolve_map(mp) if mp else None)


def resolve_map(name, maps_dir=MAPS):
    if name.endswith('.yaml') and os.path.exists(name):
        return os.path.abspath(name)
    p = os.path.join(maps_dir, name if name.endswith('.yaml') else name + '.yaml')
    if not os.path.exists(p):
        raise SystemExit(f'지도 없음: {name} ({maps_dir})')
    return p


def map_from_bag_name(bag_dir, maps_dir=MAPS):
    """bag 폴더 이름 안에 든 지도 이름(가장 긴 것). keepout 등 보조 yaml 은 뺀다."""
    base = os.path.basename(os.path.normpath(bag_dir))
    names = [os.path.basename(p)[:-5] for p in glob.glob(os.path.join(maps_dir, '*.yaml'))]
    names = [n for n in names if not n.endswith(('_keepout', '_route', '_speed'))]
    hit = sorted((n for n in names if n in base), key=len, reverse=True)
    return os.path.join(maps_dir, hit[0] + '.yaml') if hit else None


# ── 계산(캐시 z 만 씀 — 시험은 가짜 z 로) ─────────────────────
GRID_S, TRIM_S, MIN_SEG_S = 0.05, 1.0, 3.0
CURVE_CM = 20.0   # 직진 구간의 실제 궤적(EKF)이 시작-끝 직선에서 이만큼 넘게 벗어나면 '경로 휨'(플래너가 돌아간 길) — 뺀다


def path_width_cm(z, a, b):
    """[a, b] 실제 궤적(EKF /odom — 자이로 방향·바퀴 거리, AMCL 점프 없음)의 시작-끝 직선 기준 옆 폭(최대 − 최소, cm).

    지도(AMCL) 경로로 재면 지도가 안 맞을 때 AMCL 이 옆으로 왔다 갔다 한 것까지 휨으로 잡힌다(11-01 옛 지도 bag 26~46 cm)."""
    t = np.asarray(z['odom_t'])
    sel = (t >= a) & (t <= b)
    P = np.asarray(z['odom'])[sel]
    if len(P) < 5:
        return None
    u = P[-1, :2] - P[0, :2]
    L = float(np.hypot(*u))
    if L < 0.5:
        return None
    u = u / L
    d = P[:, :2] - P[0, :2]
    lat = d[:, 0] * -u[1] + d[:, 1] * u[0]
    return float(lat.max() - lat.min()) * 100


def straight_segments(z, curve_cm=CURVE_CM):
    """직진 구간(evaluate.segment_motion 의 전진·후진, 1 s 평균 |자이로| < 0.15) — 양 끝 1 s 는 뺀다(회전 직후 AMCL 이
    방향을 따라잡는 보정 제외). 지도 경로가 curve_cm 넘게 휜 구간은 skipped 로 따로 낸다."""
    if 'gz' in z:
        g_t, g = z['imu_t'], z['gz']
    else:
        g_t, g = z['odom_t'], np.zeros(len(z['odom_t']))
    keep, skipped = [], []
    for s in evaluate.segment_motion(z['vx_t'], z['vx'], g_t, g):
        if s['kind'] not in ('전진', '후진') or s['t1'] - s['t0'] < MIN_SEG_S:
            continue
        a, b = s['t0'] + TRIM_S, s['t1'] - TRIM_S
        seg = dict(t0=a, t1=b, sign=1 if s['kind'] == '전진' else -1, width=path_width_cm(z, a, b))
        (skipped if curve_cm and seg['width'] is not None and seg['width'] > curve_cm else keep).append(seg)
    return keep, skipped


def straight_grid(z, curve_cm=CURVE_CM):
    """0.05 s 격자의 직진 여부·진행 부호."""
    t = np.arange(z['odom_t'][0], z['odom_t'][-1] + GRID_S, GRID_S)
    held = np.zeros(len(t), bool)
    sgn = np.zeros(len(t))
    keep, _ = straight_segments(z, curve_cm)
    for s in keep:
        m = (t >= s['t0']) & (t <= s['t1'])
        held |= m
        sgn[m] = s['sign']
    return t, held, sgn


def straight_mask(z, t, grid=None):
    gt, held, sgn = grid if grid is not None else straight_grid(z)
    i = np.clip(np.round((np.asarray(t, float) - gt[0]) / GRID_S).astype(int), 0, len(gt) - 1)
    return held[i], sgn[i]


def _dir_ok(sgn, direction):
    """direction: +1 전진만 · −1 후진만 · 0 둘 다."""
    return np.ones_like(sgn, bool) if direction == 0 else (sgn == direction)


def straight_distance(z, grid=None, direction=0):
    t = np.asarray(z['odom_t'])
    P = np.asarray(z['odom'])
    step = np.r_[0.0, np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))]
    step[step > 0.2] = 0.0
    ok, sgn = straight_mask(z, t, grid)
    return float(step[ok & _dir_ok(sgn, direction)].sum())


def amcl_stats(z, direction=0, curve_cm=CURVE_CM):
    """직진 중 AMCL 보정량. map→odom 원본(mo)이 없는 옛 캐시면 None."""
    if 'mo_t' not in z or len(z['mo_t']) < 2:
        return None
    grid = straight_grid(z, curve_cm)
    L = straight_distance(z, grid, direction)
    mo = np.column_stack([z['mo_t'], z['mo']])
    ek = np.column_stack([z['odom_t'], z['odom']])
    init = np.asarray(z['init_t']) if 'init_t' in z else np.zeros(0)
    rows = []
    for c in amcl_corrections(mo, ek):
        ok, sgn = straight_mask(z, np.array([c['t']]), grid)
        if not (ok & _dir_ok(sgn, direction))[0] or (len(init) and np.min(np.abs(init - c['t'])) < INIT_GUARD_S):
            continue
        s = float(sgn[0])
        rows.append((s * c['side'], s * c['fwd'], math.degrees(c['dyaw'])))
    out = dict(dist=L, n=len(rows))
    if L < 1.0 or not rows:
        return out
    r = np.array(rows)
    out.update(n_per_m=len(r) / L,
               side_net=r[:, 0].sum() / L * 100, fwd_net=r[:, 1].sum() / L * 100, yaw_net=r[:, 2].sum() / L,
               side_p95=float(np.percentile(np.abs(r[:, 0]), 95) * 100),
               yaw_p95=float(np.percentile(np.abs(r[:, 2]), 95)),
               yaw_gross=float(np.abs(r[:, 2]).sum() / L))
    return out


def loc_stats(z, direction=0, curve_cm=CURVE_CM):
    """직진 중 ① AMCL − 지도 대조(탐색 범위 안만)."""
    if 'm_t' not in z or not len(z['m_t']):
        return {}
    ok, sgn = straight_mask(z, np.asarray(z['m_t']), straight_grid(z, curve_cm))
    ok &= _dir_ok(sgn, direction) & (np.asarray(z['m_edge']) < 0.5)
    if ok.sum() < 3:
        return {}
    return dict(loc_lat_p95=float(np.percentile(np.abs(z['m_lat'][ok]), 95) * 100),
                loc_yaw_p95=float(np.percentile(np.abs(np.degrees(z['m_dyaw'][ok])), 95)),
                score_amcl=float(np.median(z['m_score_amcl'][ok])),
                score_best=float(np.median(z['m_score_best'][ok])),
                loc_n=int(ok.sum()))


def wobble_stats(z, direction=0, curve_cm=CURVE_CM):
    if 'gz' not in z:
        return {}
    keep, _ = straight_segments(z, curve_cm)
    amps = []
    for s in keep:
        if direction and s['sign'] != direction:
            continue
        w = evaluate._wobble(z, s['t0'], s['t1'])
        if w is not None:
            amps.append(w)
    return dict(wobble=float(np.median(amps)), wobble_n=len(amps)) if amps else {}


def bag_stats(z, direction=0, curve_cm=CURVE_CM):
    out = {}
    a = amcl_stats(z, direction, curve_cm)
    if a:
        out.update(a)
    else:
        out['dist'] = straight_distance(z, straight_grid(z, curve_cm), direction)
    out.update(loc_stats(z, direction, curve_cm))
    out.update(wobble_stats(z, direction, curve_cm))
    return out


def verdict(new, base, better):
    """새 값 vs 기준(평균): '줄었음/늘었음/비슷' + 좋아졌는지."""
    if new is None or base is None or better is None:
        return ''
    if better == 0:
        new, base = abs(new), abs(base)
    if base == 0:
        return ''
    ch = (new - base) / abs(base)
    if abs(ch) < CHANGE:
        return f'비슷({ch * 100:+.0f} %)'
    word = '늘었음' if ch > 0 else '줄었음'
    good = (ch < 0) if better <= 0 else (ch > 0)
    return f"{word}({ch * 100:+.0f} %) {'좋아짐' if good else '나빠짐'}"


# ── 출력 ────────────────────────────────────────────────────
def print_table(names, stats):
    w = max(12, max(len(n) for n in names) + 2)
    short = [n if len(n) <= w - 2 else n[:w - 3] + '…' for n in names]
    head = f"{'항목':<24}" + ''.join(f'{s:>{w}}' for s in short)
    if len(stats) > 1:
        head += '   새 주행(1열) vs 나머지 평균'
    print(head)
    for key, label, unit, fmt, better in ROWS:
        vals = [s.get(key) for s in stats]
        cells = ''.join(f"{(format(v, fmt) if v is not None else '–'):>{w}}" for v in vals)
        line = f"{label + ' (' + unit + ')':<24}" + cells
        if len(stats) > 1:
            rest = [v for v in vals[1:] if v is not None]
            line += '   ' + verdict(vals[0], float(np.mean(rest)) if rest else None, better)
        print(line)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='bag[@지도] — 첫 번째가 새 주행')
    ap.add_argument('--cache', default=CACHE)
    ap.add_argument('--force', action='store_true', help='캐시 무시하고 다시 계산')
    ap.add_argument('--dir', choices=('fwd', 'back', 'both'), default='fwd',
                    help='직진 방향: fwd 전진만(기본 — Nav2 는 전진만 함) · back 후진만 · both 둘 다')
    ap.add_argument('--curve-cm', type=float, default=CURVE_CM,
                    help='실제 궤적(EKF)이 이만큼(cm) 넘게 휜 직진 구간은 뺀다(플래너가 돌아간 길). 0 이면 안 뺀다(기본 20)')
    a = ap.parse_args(argv)
    direction = {'fwd': 1, 'back': -1, 'both': 0}[a.dir]
    names, stats = [], []
    for arg in a.bags:
        bag, mp = resolve_bag(arg)
        mp = mp or map_from_bag_name(bag)
        if mp is None:
            raise SystemExit(f'지도를 못 찾음: {arg} — bag@지도이름 으로 주세요')
        print(f'· {os.path.basename(bag)}  지도 {os.path.basename(mp)[:-5]}', flush=True)
        path = analyze_loc.run(bag, mp, a.cache, 2.0, force=a.force)
        z = dict(np.load(path, allow_pickle=False))
        names.append(os.path.basename(bag))
        stats.append(bag_stats(z, direction, a.curve_cm))
        keep, skipped = straight_segments(z, a.curve_cm)
        T0 = float(z['odom_t'][0])
        fmt = lambda S: ', '.join(f"{x['t0'] - T0:.0f}~{x['t1'] - T0:.0f} s"
                                  + (f"({x['width']:.0f} cm)" if x['width'] is not None else '') for x in S)
        print(f'  직진 구간 {len(keep)}개: {fmt(keep)}')
        if skipped:
            print(f'  뺀 구간(경로 휨 > {a.curve_cm:g} cm) {len(skipped)}개: {fmt(skipped)}')
    print(f"\n직진 방향: {({1: '전진만', -1: '후진만', 0: '전진+후진'})[direction]}"
          + (' (후진은 AMCL 방향 보정이 전진의 약 2배로 튄다 — 11-01 기준 bag)' if direction != 1 else ''))
    print_table(names, stats)
    print('\n값은 직진 중만. 옆·방향 + = 왼쪽(반시계). ① 은 지도가 실제와 맞을 때만 믿을 수 있다(지도 정합 75 % 이상 권장).')


if __name__ == '__main__':
    main()
