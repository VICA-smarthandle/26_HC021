#!/usr/bin/env python3
"""Nav2(VCC)와 같은 가감속으로 회전 없이 직진 — 개루프 직진 시험(2026-11-23)."""
import argparse
import csv
import math
import os
import signal
import sys
import time

V1, A1, A, DECEL, VMAX = 0.25, 0.5, 0.2, 0.7, 0.7
RATE = 20.0
MIN_TAIL_V = 0.05
STOP_EPS = 0.01


def plan(dist, vmax=VMAX, v1=V1, a1=A1, a=A, decel=DECEL):
    """시간 기준 가속 + 남은 거리 기준 감속 프로필을 미리 따라가 본다(오도메트리 = 명령 그대로 가정)."""
    dt = 1.0 / RATE
    t = v = x = 0.0
    rows = []
    while x < dist - STOP_EPS and t < 120:
        v = next_speed(v, dist - x, dt, vmax, v1, a1, a, decel)
        x += v * dt
        t += dt
        rows.append((t, v, x))
    return rows


def next_speed(v, remaining, dt, vmax=VMAX, v1=V1, a1=A1, a=A, decel=DECEL):
    acc = a1 if v < v1 else a
    v_acc = min(vmax, v + acc * dt)
    v_dec = math.sqrt(2.0 * decel * max(remaining, 0.0))
    v_new = min(v_acc, v_dec)
    if remaining > STOP_EPS:
        v_new = max(v_new, min(MIN_TAIL_V, v_acc))
    return v_new


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dist', type=float, default=6.0)
    ap.add_argument('--vmax', type=float, default=VMAX)
    ap.add_argument('--go', action='store_true', help='실제로 명령을 보낸다(없으면 계획만)')
    ap.add_argument('--w-amp', type=float, default=0.0, help='정속 구간 회전 사인 진폭 rad/s(0 = 회전 없음, 최대 0.1)')
    ap.add_argument('--w-freq', type=float, default=0.45, help='회전 사인 주파수 Hz(0.1~1.0)')
    ap.add_argument('--log', default=os.path.expanduser('~/vica_data/drive_diag_rec'))
    a = ap.parse_args(argv)
    if not 0.5 <= a.dist <= 15.0 or not 0.05 <= a.vmax <= 0.7:
        raise SystemExit(f'범위 밖: dist {a.dist} (0.5~15) · vmax {a.vmax} (0.05~0.7)')
    if not 0.0 <= a.w_amp <= 0.1 or not 0.1 <= a.w_freq <= 1.0:
        raise SystemExit(f'범위 밖: w-amp {a.w_amp} (0~0.1) · w-freq {a.w_freq} (0.1~1.0)')
    rows = plan(a.dist, a.vmax)
    vmax_t = sum(1 for r in rows if r[1] >= a.vmax - 1e-6) / RATE
    print(f'계획: {a.dist:.2f} m · 최고 {a.vmax:.2f} m/s 유지 {vmax_t:.1f} s · 전체 {rows[-1][0]:.1f} s '
          f'(가속 0→{V1} @{A1} · →{a.vmax} @{A} · 감속 @{DECEL} m/s²)')
    if a.w_amp > 0:
        om = 2 * math.pi * a.w_freq
        print(f'회전 사인: ±{a.w_amp:.3f} rad/s · {a.w_freq:.2f} Hz · 정속 {vmax_t:.1f} s 동안 {vmax_t * a.w_freq:.1f} 주기 · '
              f'방향 흔들림 ±{math.degrees(a.w_amp / om):.2f}° · 옆 흔들림 ±{a.vmax * a.w_amp / om / om * 100:.1f} cm(차체 이득 1 일 때)')
    else:
        print('회전 0')
    if not a.go:
        print('명령은 보내지 않았다(--go 없음).')
        return 0

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('straight_drive_test')
    pub = node.create_publisher(Twist, '/cmd_vel_req', 10)
    st = {'p': None, 't': None}
    node.create_subscription(Odometry, '/odom', lambda m: st.update(
        p=(m.pose.pose.position.x, m.pose.pose.position.y), t=time.monotonic()), qos_profile_sensor_data)

    def send(v, w_=0.0):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w_)
        pub.publish(msg)

    def stop(reason):
        for _ in range(int(0.5 * RATE)):
            send(0.0)
            time.sleep(1.0 / RATE)
        print(f'정지: {reason}')

    interrupted = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: interrupted.update(v=True))
    t0 = time.monotonic()
    while st['p'] is None and time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    if st['p'] is None:
        node.destroy_node(); rclpy.shutdown()
        raise SystemExit('/odom 이 안 들어온다 — 명령 안 보냄')
    p0 = st['p']
    os.makedirs(a.log, exist_ok=True)
    logp = os.path.join(a.log, time.strftime('straight_%m%d_%H%M%S.csv'))
    f = open(logp, 'w', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'v_cmd', 'w_cmd', 'traveled_m'])
    dt = 1.0 / RATE
    v = 0.0
    t_start = time.monotonic()
    last_prog = (t_start, 0.0)
    reason = '목표 거리'
    sine_t0 = None
    w_cmd = 0.0
    print(f'출발 — {a.dist:.2f} m 직진. 멈추려면 Ctrl-C 또는 비상정지 버튼. 기록 {logp}', flush=True)
    try:
        while True:
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            traveled = math.hypot(st['p'][0] - p0[0], st['p'][1] - p0[1])
            rem = a.dist - traveled
            if interrupted['v']:
                reason = 'Ctrl-C'
                break
            if tick - st['t'] > 0.5:
                reason = '/odom 0.5 s 끊김'
                break
            if rem <= STOP_EPS:
                break
            if traveled - last_prog[1] > 0.02:
                last_prog = (tick, traveled)
            elif v > 0.1 and tick - last_prog[0] > 3.0:
                reason = '명령 중 3 s 동안 안 움직임(E-stop·Safety 확인)'
                break
            if tick - t_start > a.dist / a.vmax * 2 + 10:
                reason = '시간 초과'
                break
            v = next_speed(v, rem, dt, a.vmax)
            cruising = v >= a.vmax - 1e-6 and math.sqrt(2.0 * DECEL * max(rem, 0.0)) >= a.vmax
            if a.w_amp > 0 and cruising:
                sine_t0 = tick if sine_t0 is None else sine_t0
                w_cmd = a.w_amp * math.sin(2 * math.pi * a.w_freq * (tick - sine_t0))
            else:
                w_cmd = 0.0
            send(v, w_cmd)
            w.writerow([f'{tick - t_start:.3f}', f'{v:.4f}', f'{w_cmd:.4f}', f'{traveled:.4f}'])
            time.sleep(max(0.0, dt - (time.monotonic() - tick)))
    finally:
        stop(reason)
        rclpy.spin_once(node, timeout_sec=0.3)
        traveled = math.hypot(st['p'][0] - p0[0], st['p'][1] - p0[1])
        print(f'오도메트리 이동 {traveled:.3f} m · {time.monotonic() - t_start:.1f} s')
        f.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
