#!/usr/bin/env python3
"""bag → x·y·θ 오차(① 위치추정·② 오도메트리) 계산·캐시. ROS 2 환경 필요.

    source /opt/ros/humble/setup.bash
    python3 scripts/drive_diag/analyze_loc.py <bag 폴더> <지도.yaml> [--rate 2] [--cache DIR]

필요 토픽: /scan /tf(map→odom) /tf_static(base→laser) /odom (/wheel/odom /imu/base_link /cmd_vel_safe 는 있으면 씀)
결과는 캐시(기본 ~/vica_data/drive_diag_cache/<bag>__<지도>.npz)에 저장하고 serve.py 가 읽는다.
녹화 중인 bag 에는 쓰지 않는다.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import locerr as le  # noqa: E402

VERSION = 3
BASE = 'base_footprint'
LASER = 'laser_frame'


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def cache_path(bag_dir, map_yaml, cache_dir, rate):
    dbs = sorted(f for f in os.listdir(bag_dir) if f.endswith('.db3'))
    sig = f'{VERSION}|{os.path.abspath(bag_dir)}|{[os.path.getsize(os.path.join(bag_dir, d)) for d in dbs]}|' \
          f'{os.path.abspath(map_yaml)}|{os.path.getmtime(map_yaml)}|{rate}'
    h = hashlib.sha1(sig.encode()).hexdigest()[:10]
    name = f'{os.path.basename(os.path.normpath(bag_dir))}__{os.path.splitext(os.path.basename(map_yaml))[0]}__{h}'
    return os.path.join(cache_dir, name + '.npz')


def read_bag(bag_dir):
    import bagio
    D = {k: [] for k in ('mo', 'odom', 'wheel', 'imu', 'cmd', 'scan', 'static')}
    topics = ['/scan', '/tf', '/tf_static', '/odom', '/wheel/odom', '/imu/base_link', '/cmd_vel_safe']
    for name, rt, m in bagio.read(bag_dir, topics):
        if name == '/scan':
            D['scan'].append((_stamp(m.header), rt, np.asarray(m.ranges, np.float32), m.angle_min, m.angle_increment))
        elif name == '/tf':
            for t in m.transforms:
                if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                    tr = t.transform
                    D['mo'].append((_stamp(t.header), rt, tr.translation.x, tr.translation.y, _yaw(tr.rotation)))
        elif name == '/tf_static':
            for t in m.transforms:
                tr = t.transform
                D['static'].append((t.header.frame_id, t.child_frame_id,
                                    (tr.translation.x, tr.translation.y, _yaw(tr.rotation))))
        elif name in ('/odom', '/wheel/odom'):
            p = m.pose.pose
            D['odom' if name == '/odom' else 'wheel'].append(
                (_stamp(m.header), rt, p.position.x, p.position.y, _yaw(p.orientation), m.twist.twist.linear.x))
        elif name == '/imu/base_link':
            D['imu'].append((_stamp(m.header), m.angular_velocity.z))
        else:
            D['cmd'].append((rt, m.linear.x, m.angular.z))
    for k in ('mo', 'odom', 'wheel', 'imu', 'cmd'):
        D[k] = np.array(D[k], dtype=float)
    return D


def analyze(bag_dir, map_yaml, rate=2.0, key_step=0.25, key_yaw=math.radians(3.0), log=print):
    t_start = time.time()
    D = read_bag(bag_dir)
    if len(D['odom']) < 10 or not D['scan']:
        raise SystemExit('/odom 또는 /scan 이 없다')
    sensor = le.static_pose(D['static'], BASE, LASER) or le.static_pose(D['static'], 'base_link', LASER)
    if sensor is None:
        raise SystemExit('/tf_static 에 base_footprint(base_link)→laser_frame 이 없다')
    od = D['odom']
    od_t, od_P = od[:, 0], od[:, 2:5].copy()
    od_P[:, 2] = np.unwrap(od_P[:, 2])
    off = float(np.median(od[:, 1] - od[:, 0]))  # 녹화 시각 − 센서 시각

    def ekf_at(t):
        return (float(np.interp(t, od_t, od_P[:, 0])), float(np.interp(t, od_t, od_P[:, 1])),
                float(np.interp(t, od_t, od_P[:, 2])))

    out = dict(sensor=np.array(sensor), off=off)
    # ── ① 위치추정 오차 ──
    mo = D['mo']
    if len(mo) > 5:
        shift = le.tf_time_shift(mo[:, 0], mo[:, 1], od[:, 0], od[:, 1], scan_stamps=[x[0] for x in D['scan']])
        mo_t = mo[:, 0] - shift
        grid, meta = le.load_map(map_yaml)
        field = le.ps.build_likelihood_field(grid)
        rows, last = [], -1e9
        scans = [s for s in D['scan'] if mo_t[0] <= s[0] <= od_t[-1]]
        for k, (st, rt, rng, amin, ainc) in enumerate(scans):
            if st - last < 1.0 / rate:
                continue
            last = st
            i = int(np.searchsorted(mo_t, st, side='right')) - 1
            if i < 0:
                continue
            amcl = le.compose(tuple(mo[i, 2:5]), ekf_at(st))
            beams = le.beams_from_scan(rng, amin, ainc)
            m = le.local_match(field, grid, beams, amcl, sensor)
            e = le.pose_error(amcl, (m.x, m.y, m.yaw))
            rows.append((st, amcl[0], amcl[1], amcl[2], m.x, m.y, m.yaw, m.amcl_score, m.score, float(m.edge),
                         e['fwd'], e['lat'], e['dyaw'], float(beams.used)))
            if len(rows) % 100 == 0:
                log(f'  ① 지도 대조 {len(rows)}장')
        cols = 't amcl_x amcl_y amcl_yaw best_x best_y best_yaw score_amcl score_best edge fwd lat dyaw beams'.split()
        R = np.array(rows) if rows else np.zeros((0, len(cols)))
        for j, c in enumerate(cols):
            out['m_' + c] = R[:, j]
        out['map_meta'] = np.array(json.dumps(meta))
        out['tf_shift'] = shift
        # 지도 위 EKF 경로(10 Hz)
        tt = np.arange(od_t[0], od_t[-1], 0.1)
        mi = np.clip(np.searchsorted(mo_t, tt, side='right') - 1, 0, len(mo) - 1)
        ok = tt >= mo_t[0]
        P = np.array([le.compose(tuple(mo[j, 2:5]), ekf_at(t)) for j, t in zip(mi[ok], tt[ok])])
        out['path_t'], out['path'] = tt[ok], P
    else:
        log('  ① map→odom 이 없다 — AMCL 위치 미설정. 위치추정 오차는 건너뜀')
    # ── ② 오도메트리 오차: 라이다 키프레임 사슬 ──
    sc = D['scan']
    st = np.array([s[0] for s in sc])
    inside = (st >= od_t[0]) & (st <= od_t[-1])
    cand = np.where(inside)[0]
    P = [ekf_at(st[j]) for j in cand]
    kidx = [cand[i] for i in le.select_keyframes(P, key_step, key_yaw)]
    keys = [le.scan_points(sc[j][2], sc[j][3], sc[j][4]) for j in kidx]
    guess = [le.relative(ekf_at(st[a]), ekf_at(st[b])) for a, b in zip(kidx[:-1], kidx[1:])]
    log(f'  ② 라이다 키프레임 {len(kidx)}개 ICP')
    chain, inl = le.lidar_chain(keys, guess, sensor)
    out['lid_t'], out['lid'], out['lid_inl'] = st[kidx], np.array(chain), np.r_[1.0, inl]
    # 참고 신호
    out['odom_t'], out['odom'] = od_t, od_P
    if len(D['wheel']):
        w = D['wheel']
        wp = w[:, 2:5].copy()
        wp[:, 2] = np.unwrap(wp[:, 2])
        out['wheel_t'], out['wheel'] = w[:, 0], wp
    out['vx_t'], out['vx'] = od_t, od[:, 5]
    if len(D['imu']):
        out['imu_t'], out['gz'] = D['imu'][:, 0], D['imu'][:, 1]
    if len(D['cmd']):
        out['cmd_t'], out['cmd_v'], out['cmd_w'] = D['cmd'][:, 0] - off, D['cmd'][:, 1], D['cmd'][:, 2]
    # 화면용 스캔(① 에 쓴 시각만, 거른 빔)
    if 'm_t' in out and len(out['m_t']):
        ts = set(np.round(out['m_t'], 6))
        sel = [s for s in sc if round(s[0], 6) in ts]
        out['scan_t'] = np.array([s[0] for s in sel])
        out['scan_r'] = np.array([s[2] for s in sel], dtype=np.float32)
        out['scan_amin'] = np.array([s[3] for s in sel])
        out['scan_ainc'] = np.array([s[4] for s in sel])
    out['bag'] = np.array(os.path.abspath(bag_dir))
    out['map_yaml'] = np.array(os.path.abspath(map_yaml))
    log(f'  계산 {time.time() - t_start:.0f} s')
    return out


def run(bag_dir, map_yaml, cache_dir, rate=2.0, log=print, force=False):
    os.makedirs(cache_dir, exist_ok=True)
    path = cache_path(bag_dir, map_yaml, cache_dir, rate)
    if os.path.exists(path) and not force:
        return path
    res = analyze(bag_dir, map_yaml, rate, log=log)
    np.savez(path, **res)
    return path


def summary(path):
    z = np.load(path, allow_pickle=False)
    print(f'캐시: {path}')
    print(f'  라이다 장착(TF): x {z["sensor"][0]:.3f} m · yaw {math.degrees(z["sensor"][2]):+.2f}°')
    if 'm_t' in z and len(z['m_t']):
        ok = z['m_edge'] < 0.5
        v = np.interp(z['m_t'], z['vx_t'], np.abs(z['vx']))
        g = np.abs(np.interp(z['m_t'], z['imu_t'], z['gz'])) if 'gz' in z else np.zeros_like(v)
        state = {'직진 주행': (v > 0.15) & (g < 0.10), '회전 중': g >= 0.10, '정지': (v < 0.05) & (g < 0.10)}
        print(f'  ① 지도 대조 {len(z["m_t"])}장 · map→odom 시각 보정 {float(z["tf_shift"]):.2f} s')
        for lab, s in state.items():
            n_all, s = int(s.sum()), s & ok
            if n_all == 0:
                continue
            line = f'    {lab:6} {n_all:4d}장 (범위 밖 {n_all - int(s.sum())})'
            if s.sum() >= 3:
                q = lambda a: f'{np.mean(a[s]) * 100:+5.1f} ± {np.std(a[s]) * 100:4.1f}'
                line += (f' | AMCL − 지도대조: 앞 {q(z["m_fwd"])} cm · 옆 {q(z["m_lat"])} cm · '
                         f'방향 {math.degrees(np.mean(z["m_dyaw"][s])):+5.2f} ± {math.degrees(np.std(z["m_dyaw"][s])):4.2f}° · '
                         f'점수 AMCL {np.median(z["m_score_amcl"][s]):.1f} / 최고 {np.median(z["m_score_best"][s]):.1f}')
            print(line)
    print(f'  ② 라이다 키프레임 {len(z["lid_t"])}개 · ICP 맞은 점 최소 {z["lid_inl"].min() * 100:.0f} % · 중앙 {np.median(z["lid_inl"]) * 100:.0f} %')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('map_yaml')
    ap.add_argument('--rate', type=float, default=2.0)
    ap.add_argument('--cache', default=os.path.expanduser('~/vica_data/drive_diag_cache'))
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    summary(run(a.bag, a.map_yaml, a.cache, a.rate, force=a.force))


if __name__ == '__main__':
    main()
