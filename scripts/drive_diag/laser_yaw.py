#!/usr/bin/env python3
"""라이다 장착각(yaw) 측정 — 정석 2단계.

    source /opt/ros/humble/setup.bash
    # 방법 1: 차체를 곧은 벽과 평행하게(앞뒤 두 점 줄자 차이 ≤ 2 mm) 세우고 /scan 30 s 녹화
    python3 scripts/drive_diag/laser_yaw.py wall <bag 폴더> [--sector 30:150]
    # 방법 2: teleop 명령 w=0 으로 4 m 전진 ×3, 같은 선 후진 ×3 녹화
    python3 scripts/drive_diag/laser_yaw.py drive <bag 폴더>

녹화는 scripts/drive_diag/laser_yaw_record.sh 를 쓴다.

부호: 라이다 정면이 차체(진행 방향)보다 오른쪽으로 돌아가 있으면 yaw 는 −. URDF laser_joint rpy 의
세 번째 값에 그대로 들어갈 값이다(적용은 TF 변경이라 선승인).

방법 1 = 차체 기준(라이다가 차체에 비틀려 붙었나). 방법 2 = 진행 방향 기준(바퀴가 실제로 굴러가는 방향).
TF 에 넣는 값은 방법 2 다 — base_link x 축은 실제 진행 방향이어야 오도메트리·AMCL 이 맞는다.
진행각은 라이다 원본 좌표계에서 재므로 TF 를 고쳐도 같은 값이 나온다. 검증은 출력의 '잔여(실측 − TF)'(bag 의
/tf_static 과 비교)가 0 근처인지, 그리고 직진 중 AMCL 옆 보정이 0 근처인지로 한다.
방법 1 ≈ 0 이고 방법 2 ≠ 0 이면 라이다가 아니라 구동축 정렬이 틀어진 것이다.

방법 2 판정: 전진·후진의 진행각(라이다 좌표계에서 진행 방향의 각도)이
  같으면 → 장착각(로봇에 붙은 오차)     장착각 = −(전진 + 후진)/2
  반대면 → 미끄럼·바닥 기울기          미끄럼 = (전진 − 후진)/2
배경: 2026-11-01 run99·100 곧게 가기에서 라이다 옆 이동이 EKF 보다 2~4.5 cm/m(≈1.7~2.2°) 컸다(전진만, [미검증]).
"""
import argparse
import math
import os
import sys
import warnings

import numpy as np
from scipy.spatial import cKDTree

MIN_RANGE = 0.25   # pose_score.py 와 같은 값(차체 근접 반사 제외)
MAX_RANGE = 11.0
KEY_STEP_M = 0.5


# ── 계산(ROS 없이 시험) ───────────────────────────────────

def scan_points(ranges, angle_min, angle_inc, rmin=MIN_RANGE, rmax=MAX_RANGE):
    r = np.asarray(ranges, float)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def _wrap_half(a):
    """직선 방향은 180° 주기 → (−90°, 90°]."""
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def _pca_line(P):
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    d, n = vt[0], vt[1]
    return c, d, n


def fit_wall(P, sector_deg=None, tol=0.02, iters=400, seed=0):
    """가장 많은 점이 붙는 직선(RANSAC → 직교 최소제곱 2회). 라이다 좌표계 각도(rad)와 yaw 후보(°)."""
    P = np.asarray(P, float)
    if sector_deg is not None:
        ang = np.degrees(np.arctan2(P[:, 1], P[:, 0]))
        P = P[(ang >= sector_deg[0]) & (ang <= sector_deg[1])]
    if len(P) < 20:
        raise ValueError(f'점이 너무 적다({len(P)}) — 구역(--sector)을 확인')
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i, j = rng.choice(len(P), 2, replace=False)
        d = P[j] - P[i]
        L = np.hypot(*d)
        if L < 0.2:
            continue
        n = np.array([-d[1], d[0]]) / L
        inl = np.abs((P - P[i]) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    inl = best
    for _ in range(2):
        c, d, n = _pca_line(P[inl])
        inl = np.abs((P - c) @ n) < tol
    c, d, n = _pca_line(P[inl])
    res = (P[inl] - c) @ n
    along = (P[inl] - c) @ d
    angle = _wrap_half(math.atan2(d[1], d[0]))
    return dict(angle=angle, yaw_deg=-math.degrees(angle), dist=float(abs(c @ n)), inliers=int(inl.sum()),
                rms=float(np.sqrt(np.mean(res ** 2))), length=float(along.max() - along.min()))


def _normals(Q, tree, k=8):
    _, idx = tree.query(Q, k=k)
    N = np.zeros_like(Q)
    good = np.zeros(len(Q), bool)
    for i, nb in enumerate(idx):
        X = Q[nb] - Q[nb].mean(0)
        w, v = np.linalg.eigh(X.T @ X)
        if w[1] > 1e-9 and w[0] / w[1] < 0.05:
            N[i] = v[:, 0]
            good[i] = True
    return N, good


def icp(src, dst, x0, iters=40):
    """점-선 ICP: src(나중 스캔)를 dst(앞 스캔) 좌표계로 → (tx, ty, th, 맞은 점 비율)."""
    tree = cKDTree(dst)
    N, good = _normals(dst, tree)
    tx, ty, th = x0
    for k in range(iters):
        c, s = math.cos(th), math.sin(th)
        p = src @ np.array([[c, s], [-s, c]]) + [tx, ty]
        dd, j = tree.query(p)
        m = (dd < max(0.05, 0.5 * (0.8 ** k))) & good[j]
        if m.sum() < 30:
            break
        n, q, pm = N[j[m]], dst[j[m]], p[m]
        r = np.sum((pm - q) * n, 1)
        J = np.c_[n[:, 0], n[:, 1], -n[:, 0] * pm[:, 1] + n[:, 1] * pm[:, 0]]
        w = 1.0 / (1.0 + (r / 0.03) ** 2)
        dx = -np.linalg.solve((J * w[:, None]).T @ J + 1e-9 * np.eye(3), (J * w[:, None]).T @ r)
        cd, sd = math.cos(dx[2]), math.sin(dx[2])
        tx, ty = cd * tx - sd * ty + dx[0], sd * tx + cd * ty + dx[1]
        th += dx[2]
        if np.hypot(dx[0], dx[1]) < 1e-6 and abs(dx[2]) < 1e-7:
            break
    c, s = math.cos(th), math.sin(th)
    p = src @ np.array([[c, s], [-s, c]]) + [tx, ty]
    dd, j = tree.query(p)
    return tx, ty, th, float(((dd < 0.05) & good[j]).mean())


def motion_direction(keys, step, direction, dyaw=None):
    """키 스캔 사이 ICP 로 라이다 좌표계 진행각(진행 방향 − 라이다 정면)을 거리 가중 평균.

    keys: 키 스캔 점 목록(시간 순), step: 키 간 바퀴 거리(스칼라 또는 목록), direction: +1 전진 / −1 후진,
    dyaw: 키 간 자이로 회전(초기값).
    """
    n = len(keys) - 1
    steps = [step] * n if np.isscalar(step) else list(step)
    dyaw = [0.0] * n if dyaw is None else list(dyaw)
    rows = []
    for k in range(n):
        tx, ty, th, inl = icp(keys[k + 1], keys[k], (direction * steps[k], 0.0, dyaw[k]))
        mx, my = (tx, ty) if direction > 0 else (-tx, -ty)
        rows.append((math.atan2(my, mx) - th / 2, math.hypot(tx, ty), th, inl))
    r = np.array(rows)
    phi = float(np.sum(r[:, 0] * r[:, 1]) / np.sum(r[:, 1]))
    return dict(phi=phi, dist=float(r[:, 1].sum()), rot=float(r[:, 2].sum()), inlier_min=float(r[:, 3].min()),
                steps=[float(x) for x in r[:, 0]])


def combine(phi_fwd, phi_bwd):
    """전진·후진 진행각 → 장착각(°)·미끄럼(°)·표준오차."""
    if not phi_fwd or not phi_bwd:
        return dict(yaw_deg=None, slip_deg=None, se_deg=None)
    f, b = np.array(phi_fwd), np.array(phi_bwd)
    mf, mb = f.mean(), b.mean()
    var = lambda x: (x.std(ddof=1) ** 2 / len(x)) if len(x) > 1 else float('nan')
    se = 0.5 * math.sqrt(var(f) + var(b)) if len(f) > 1 and len(b) > 1 else None
    return dict(yaw_deg=-math.degrees((mf + mb) / 2), slip_deg=math.degrees((mf - mb) / 2),
                se_deg=math.degrees(se) if se is not None else None,
                fwd_deg=math.degrees(mf), bwd_deg=math.degrees(mb))


def laser_tf_yaw(entries, base='base_link', laser='laser_frame'):
    """(부모, 자식, yaw) 목록에서 base 기준 laser 의 yaw 합. 사슬이 base 에 안 닿으면 None."""
    up = {child: (parent, yaw) for parent, child, yaw in entries}
    total, cur = 0.0, laser
    for _ in range(len(up) + 1):  # 순환 TF 에서 멈추게
        if cur == base:
            return total
        if cur not in up:
            return None
        cur, yaw = up[cur]
        total += yaw
    return None


def drive_runs(cmd, min_len=2.0, v_min=0.1, max_gap=0.3):
    """명령 w 가 정확히 0 이고 |v| ≥ v_min 인 같은 방향 구간. cmd: (t, v, w) → [(t0, t1, ±1)]."""
    t, v, w = cmd[:, 0], cmd[:, 1], cmd[:, 2]
    sgn = np.where((np.abs(w) < 1e-9) & (np.abs(v) >= v_min), np.sign(v), 0)
    out, s = [], None
    for i in range(len(t) + 1):
        brk = i == len(t) or sgn[i] == 0 or (s is not None and (sgn[i] != sgn[s] or t[i] - t[i - 1] > max_gap))
        if brk and s is not None:
            if t[i - 1] - t[s] >= min_len:
                out.append((float(t[s]), float(t[i - 1]), int(sgn[s])))
            s = None
        if i < len(t) and sgn[i] != 0 and s is None:
            s = i
    return out


# ── bag 처리(ROS 필요) ────────────────────────────────────

def _scan_avg(scans):
    """여러 정지 스캔의 빔별 중앙값."""
    R = np.array([np.where(np.isfinite(s[0]), s[0], np.nan) for s in scans])
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)  # 한 번도 안 맞은 빔(전부 NaN)
        r = np.nanmedian(R, axis=0)
    return scan_points(r, scans[0][1], scans[0][2])


def cmd_wall(a):
    import bagio
    scans = [(np.array(m.ranges, float), m.angle_min, m.angle_increment)
             for _, _, m in bagio.read(a.bag, ['/scan'])]
    if not scans:
        sys.exit('/scan 이 없다')
    P = _scan_avg(scans)
    sector = tuple(float(x) for x in a.sector.split(':')) if a.sector else None
    r = fit_wall(P, sector)
    print(f'스캔 {len(scans)}장 중앙값 · 구역 {a.sector or "전체"}')
    print(f'  벽 직선: 라이다 좌표계 각도 {math.degrees(r["angle"]):+.2f}° · 거리 {r["dist"]:.3f} m · '
          f'길이 {r["length"]:.2f} m · 점 {r["inliers"]} · 잔차 RMS {r["rms"] * 1000:.1f} mm')
    print(f'  → 차체 기준 라이다 yaw {r["yaw_deg"]:+.2f}° (차체가 벽과 평행했다는 전제. 0.5 m 간격 2 mm 차이 = 0.23°)')
    if r['rms'] > 0.01 or r['length'] < 1.0:
        print('  [주의] 잔차가 크거나 벽이 짧다 — 벽이 곧은지, 구역 안에 다른 물체가 없는지 확인.'
              ' 벽이 길면 구역을 넓혀(예: 오른쪽 -178:-2, 왼쪽 2:178) 다시 맞춘다 — 벽 밖 물체는 RANSAC 이 뺀다')


def cmd_drive(a):
    import bagio
    topics = ['/cmd_vel_safe', '/scan', '/wheel/odom', '/imu/base_link']
    cmd, scans, wheel, imu, tfs = [], [], [], [], []
    for name, rt, m in bagio.read(a.bag, topics + ['/tf_static']):
        if name == '/tf_static':
            for tr in m.transforms:
                q = tr.transform.rotation
                tfs.append((tr.header.frame_id, tr.child_frame_id,
                            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif name == '/cmd_vel_safe':
            cmd.append((rt, m.linear.x, m.angular.z))
        elif name == '/scan':
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            scans.append((st, scan_points(np.array(m.ranges, float), m.angle_min, m.angle_increment)))
        elif name == '/wheel/odom':
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            wheel.append((st, m.pose.pose.position.x, m.pose.pose.position.y, rt))
        else:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            imu.append((st, m.angular_velocity.z))
    if not (cmd and scans and wheel and imu):
        sys.exit('필요 토픽 부족: ' + ', '.join(f'{t} {len(x)}' for t, x in zip(topics, (cmd, scans, wheel, imu))))
    cmd, wheel, imu = np.array(cmd), np.array(wheel), np.array(imu)
    off = float(np.median(wheel[:, 3] - wheel[:, 0]))
    cmd[:, 0] -= off
    st = np.array([s[0] for s in scans])
    wd = np.r_[0, np.cumsum(np.hypot(np.diff(wheel[:, 1]), np.diff(wheel[:, 2])))]
    gyaw = np.r_[0, np.cumsum(0.5 * (imu[1:, 1] + imu[:-1, 1]) * np.diff(imu[:, 0]))]
    runs = drive_runs(cmd, min_len=a.min_len)
    print(f'명령 w=0 직진 구간 {len(runs)}개 (|v| ≥ 0.1 m/s, {a.min_len:.0f} s 이상)')
    phis = {+1: [], -1: []}
    for t0, t1, sgn in runs:
        t0 += 0.5  # 출발 가속 뺌
        idx = np.where((st >= t0) & (st <= t1))[0]
        if len(idx) < 3:
            continue
        d_at = np.interp(st[idx], wheel[:, 0], wd)
        keys, last = [idx[0]], d_at[0]
        for k, d in zip(idx[1:], d_at[1:]):
            if d - last >= KEY_STEP_M:
                keys.append(k)
                last = d
        if len(keys) < 3:
            continue
        steps = np.diff(np.interp(st[keys], wheel[:, 0], wd))
        dy = np.diff(np.interp(st[keys], imu[:, 0], gyaw))
        r = motion_direction([scans[k][1] for k in keys], steps, sgn, dy)
        phis[sgn].append(r['phi'])
        print(f'  {"전진" if sgn > 0 else "후진"} {r["dist"]:.2f} m · 진행각 {math.degrees(r["phi"]):+.2f}° '
              f'(키 간 범위 {math.degrees(min(r["steps"])):+.2f}~{math.degrees(max(r["steps"])):+.2f}°) · '
              f'회전 {math.degrees(r["rot"]):+.2f}° · 맞은 점 최소 {r["inlier_min"] * 100:.0f} %')
    c = combine(phis[+1], phis[-1])
    if c['yaw_deg'] is None:
        print('전진·후진이 다 있어야 장착각과 미끄럼을 가른다.'
              f' 지금 전진 {len(phis[+1])} · 후진 {len(phis[-1])}')
        return
    se = f' ± {c["se_deg"]:.2f}' if c['se_deg'] is not None else ''
    print(f'전진 평균 {c["fwd_deg"]:+.2f}° · 후진 평균 {c["bwd_deg"]:+.2f}°')
    print(f'  → 진행 방향 기준 라이다 yaw {c["yaw_deg"]:+.2f}{se}°  (URDF laser_joint rpy 세 번째 값 후보, 선승인)')
    print(f'  → 미끄럼·바닥 기울기 {c["slip_deg"]:+.2f}{se}°  (전진·후진에서 반대로 나타난 몫, TF 로는 못 고침)')
    ty = laser_tf_yaw(tfs)
    if ty is None:
        print('  (bag 에 /tf_static base_link→laser_frame 이 없어 TF 와 비교 못 함)')
    else:
        print(f'  → 이 bag 의 TF laser_joint yaw {math.degrees(ty):+.2f}° · 잔여(실측 − TF) {c["yaw_deg"] - math.degrees(ty):+.2f}°'
              '  (TF 가 맞으면 0 근처)')


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    w = sub.add_parser('wall', help='방법 1: 정지 벽 맞춤')
    w.add_argument('bag')
    w.add_argument('--sector', help='벽이 있는 각도 구역(°, 라이다 기준, 왼쪽 = +). 예: 30:150')
    d = sub.add_parser('drive', help='방법 2: 전진·후진')
    d.add_argument('bag')
    d.add_argument('--min-len', type=float, default=3.0)
    a = ap.parse_args()
    (cmd_wall if a.mode == 'wall' else cmd_drive)(a)


if __name__ == '__main__':
    main()
