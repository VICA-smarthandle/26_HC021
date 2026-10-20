#!/usr/bin/env python3
"""AMCL 이 얼마나 자주 눈을 뜨는지 재는 진단 노드 (NAV2-B1 before/after 비교용)."""

import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import BehaviorTreeLog
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

MOVING_LIN = 0.02
MOVING_ANG = 0.05

BLIND_THRESHOLD_S = 2.0

ODOM_JUMP_M = 0.5

SINGLE_PUBLISHER_TOPICS = ['/odom', '/wheel/odom', '/scan', '/cmd_vel_safe']

AUTOSAVE_SEC = 30.0

CPU_SAMPLE_SEC = 5.0
WATCHED_PROCS = ['amcl', 'controller_serv', 'planner_server', 'nvblox_node',
                 'cartographer_no', 'rviz2']

WATCHED_PARAMS = [
    'update_min_d',
    'update_min_a',
    'max_beams',
    'alpha1',
    'alpha2',
    'alpha3',
    'alpha4',
    'laser_model_type',
    'resample_interval',
]


def yaw_of(q):
    """쿼터니언에서 yaw 만 뽑는다. 2D 로봇이라 roll/pitch 는 보지 않는다."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angdiff(a, b):
    """두 각의 차이를 -pi..pi 로 접는다."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _cpu_total():
    """전체 CPU 시간(jiffies)과 그중 놀고 있던 시간을 돌려준다."""
    with open('/proc/stat') as f:
        vals = [int(x) for x in f.readline().split()[1:9]]
    return sum(vals), vals[3] + vals[4]


def _proc_cpu(pid):
    """그 프로세스가 쓴 CPU 시간(jiffies). 죽었으면 None."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            parts = f.read().rsplit(') ', 1)[-1].split()
        return int(parts[11]) + int(parts[12])
    except (OSError, IndexError, ValueError):
        return None


def find_pids(names):
    """comm 으로 프로세스를 찾는다. 같은 이름이 둘이면 먼저 찾은 것을 쓴다."""
    found = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/comm') as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm in names and comm not in found:
            found[comm] = int(entry)
    return found


def pct(values, p):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return s[lo]
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


class AmclProbe(Node):

    def __init__(self, label, outdir):
        super().__init__('vica_amcl_probe')
        self.label = label
        self.outdir = outdir

        self.t0 = time.monotonic()
        self.moving_time = 0.0
        self.odom_dist = 0.0
        self.odom_rot = 0.0
        self.is_moving = False
        self.last_tick = self.t0

        self.odom_xy = None
        self.odom_yaw = None

        self.prev = None
        self.rows = []
        self.first_pose_t = None

        self.odom_jumps = 0
        self.odom_jump_max = 0.0

        self.bt_counts = {}
        self.params = None

        self.ncpu = os.cpu_count() or 1
        self.cpu_pids = find_pids(WATCHED_PROCS)
        self.cpu_prev = _cpu_total()
        self.cpu_prev_proc = {n: _proc_cpu(p) for n, p in self.cpu_pids.items()}
        self.cpu_samples = []
        self.proc_samples = {n: [] for n in self.cpu_pids}

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 20)
        self.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.on_bt_log, 20)

        self.create_timer(0.05, self.tick)
        self.create_timer(15.0, self.report_progress)
        self.create_timer(AUTOSAVE_SEC, self.autosave)
        self.create_timer(CPU_SAMPLE_SEC, self.sample_cpu)

    def on_odom(self, msg):
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.is_moving = (
            math.hypot(v.x, v.y) > MOVING_LIN or abs(w.z) > MOVING_ANG)

        p = msg.pose.pose.position
        y = yaw_of(msg.pose.pose.orientation)
        if self.odom_xy is not None:
            d = math.hypot(p.x - self.odom_xy[0], p.y - self.odom_xy[1])
            if d < ODOM_JUMP_M:
                self.odom_dist += d
                self.odom_rot += abs(angdiff(y, self.odom_yaw))
            else:
                self.odom_jumps += 1
                self.odom_jump_max = max(self.odom_jump_max, d)
                if self.odom_jumps <= 3 or self.odom_jumps % 500 == 0:
                    self.get_logger().error(
                        f'오도메트리 점프 {d:.2f} m (누적 {self.odom_jumps}회). '
                        f'노드 중복을 의심하라: ros2 topic info /wheel/odom')
                if self.odom_jumps == 5:
                    self.get_logger().error(
                        '=' * 62 + '\n'
                        '  이 회차는 판정에 쓸 수 없다. 주행을 멈추고 노드 중복을\n'
                        '  잡아라. /odom 과 /wheel/odom 의 발행자가 각각 1개여야 한다.\n'
                        + '=' * 62)
        self.odom_xy = (p.x, p.y)
        self.odom_yaw = y

    def sample_cpu(self):
        """직전 표본 이후의 CPU 점유율을 잰다. 순간값이 아니라 구간 평균이다."""
        total, idle = _cpu_total()
        d_total = total - self.cpu_prev[0]
        d_idle = idle - self.cpu_prev[1]
        if d_total > 0:
            self.cpu_samples.append(100.0 * (d_total - d_idle) / d_total)
        for name, pid in self.cpu_pids.items():
            now = _proc_cpu(pid)
            was = self.cpu_prev_proc.get(name)
            if now is not None and was is not None and d_total > 0:
                self.proc_samples[name].append(
                    100.0 * self.ncpu * (now - was) / d_total)
            self.cpu_prev_proc[name] = now
        self.cpu_prev = (total, idle)

    def on_bt_log(self, msg):
        """복구 동작 호출 횟수를 센다. RUNNING 으로 들어간 것만 1회로 본다."""
        for ev in msg.event_log:
            if ev.current_status == 'RUNNING' and ev.previous_status != 'RUNNING':
                self.bt_counts[ev.node_name] = self.bt_counts.get(ev.node_name, 0) + 1

    def tick(self):
        now = time.monotonic()
        if self.is_moving:
            self.moving_time += now - self.last_tick
        self.last_tick = now

    def on_amcl(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        snap = dict(t=now, moving_time=self.moving_time, dist=self.odom_dist,
                    rot=self.odom_rot, x=p.x, y=p.y, yaw=yaw)

        if self.first_pose_t is None:
            self.first_pose_t = now

        if self.prev is not None:
            gap_dist = snap['dist'] - self.prev['dist']
            map_move = math.hypot(snap['x'] - self.prev['x'],
                                  snap['y'] - self.prev['y'])
            self.rows.append({
                't': round(now - self.t0, 3),
                'gap_wall_s': round(now - self.prev['t'], 3),
                'gap_moving_s': round(snap['moving_time'] - self.prev['moving_time'], 3),
                'gap_dist_m': round(gap_dist, 4),
                'gap_rot_rad': round(snap['rot'] - self.prev['rot'], 4),
                'correction_m': round(abs(map_move - gap_dist), 4),
                'x': round(p.x, 4),
                'y': round(p.y, 4),
                'yaw_deg': round(math.degrees(yaw), 2),
                'moving': int(self.is_moving),
            })
        self.prev = snap

    def report_progress(self):
        el = time.monotonic() - self.t0
        moving_rows = [r for r in self.rows if r['gap_moving_s'] > 0.001]
        med = pct([r['gap_dist_m'] for r in moving_rows], 50)
        worst = max((r['gap_moving_s'] for r in moving_rows), default=0.0)
        bad = f"  \033[31m점프 {self.odom_jumps}회\033[0m" if self.odom_jumps else ""
        spin = sum(v for k, v in self.bt_counts.items() if 'Spin' in k)
        cpu = _mean(self.cpu_samples)
        amcl_cpu = _mean(self.proc_samples.get('amcl', []))
        self.get_logger().info(
            f"[{self.label}] {el:5.0f}s  이동 {self.moving_time:5.0f}s / "
            f"{self.odom_dist:5.1f}m  갱신 {len(self.rows):4d}회  "
            f"실효 update_min_d {med if med is None else round(med, 3)}  "
            f"최악 무갱신 {worst:.1f}s  Spin {spin}  "
            f"CPU {0 if cpu is None else cpu:.0f}% "
            f"(amcl {0 if amcl_cpu is None else amcl_cpu:.0f}%){bad}")

    def summarize(self, params):
        moving_rows = [r for r in self.rows if r['gap_moving_s'] > 0.001]
        gaps = [r['gap_moving_s'] for r in moving_rows]
        dists = [r['gap_dist_m'] for r in moving_rows]
        corrs = [r['correction_m'] for r in moving_rows]
        blind = [g for g in gaps if g >= BLIND_THRESHOLD_S]

        return {
            'label': self.label,
            'started': datetime.now().isoformat(timespec='seconds'),
            'odom_jumps': self.odom_jumps,
            'odom_jump_max_m': _r(self.odom_jump_max),
            'valid': self.odom_jumps == 0,
            'amcl_params': params,
            'duration_s': round(time.monotonic() - self.t0, 1),
            'moving_time_s': round(self.moving_time, 1),
            'odom_distance_m': round(self.odom_dist, 2),
            'odom_rotation_rad': round(self.odom_rot, 2),
            'amcl_updates': len(self.rows),
            'updates_while_moving': len(moving_rows),
            'effective_update_min_d': _r(pct(dists, 50)),
            'gap_dist_mean_m': _r(_mean(dists)),
            'gap_moving_max_s': _r(max(gaps, default=0.0)),
            'gap_moving_p95_s': _r(pct(gaps, 95)),
            'gap_moving_median_s': _r(pct(gaps, 50)),
            f'blind_over_{BLIND_THRESHOLD_S:g}s_count': len(blind),
            'blind_total_s': _r(sum(blind)),
            'updates_per_meter': _r(
                len(moving_rows) / self.odom_dist if self.odom_dist > 0.1 else None),
            'updates_per_moving_min': _r(
                len(moving_rows) / (self.moving_time / 60.0)
                if self.moving_time > 1 else None),
            'correction_mean_m': _r(_mean(corrs)),
            'correction_max_m': _r(max(corrs, default=0.0)),
            'spin_invocations': sum(
                v for k, v in self.bt_counts.items() if 'Spin' in k),
            'wait_invocations': sum(
                v for k, v in self.bt_counts.items() if 'Wait' in k),
            'clearing_invocations': sum(
                v for k, v in self.bt_counts.items() if 'Clear' in k),
            'bt_nodes': dict(sorted(self.bt_counts.items(),
                                    key=lambda kv: -kv[1])[:12]),
            'cpu_cores': self.ncpu,
            'cpu_mean_pct': _r(_mean(self.cpu_samples)),
            'cpu_p95_pct': _r(pct(self.cpu_samples, 95)),
            'cpu_max_pct': _r(max(self.cpu_samples, default=0.0)),
            'proc_cpu_mean_pct': {
                n: _r(_mean(v)) for n, v in sorted(self.proc_samples.items())
                if v and _mean(v) and _mean(v) > 1.0
            },
        }

    def autosave(self):
        """주기적으로 디스크에 남긴다."""
        if self.rows or self.moving_time > 1.0:
            self.save()

    def save(self, params=None):
        os.makedirs(self.outdir, exist_ok=True)
        summary = self.summarize(self.params if params is None else params)

        with open(os.path.join(self.outdir, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        if self.rows:
            with open(os.path.join(self.outdir, 'updates.csv'), 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
        return summary


def _mean(v):
    return sum(v) / len(v) if v else None


def _r(v, n=3):
    return round(v, n) if isinstance(v, float) else v


def fetch_params(node, timeout=None):
    """AMCL 이 실제로 들고 있는 값을 읽어 둔다."""
    if timeout is None:
        timeout = float(os.environ.get('VICA_PROBE_WAIT', '600'))
    cli = node.create_client(GetParameters, '/amcl/get_parameters')
    node.get_logger().info('AMCL 을 기다린다...')
    if not cli.wait_for_service(timeout_sec=timeout):
        node.get_logger().warn(
            'AMCL 파라미터 서비스를 못 찾았다. 값 기록 없이 계속한다.')
        return None
    req = GetParameters.Request()
    req.names = WATCHED_PARAMS
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        return None
    out = {}
    for name, val in zip(WATCHED_PARAMS, res.values):
        if val.type == 3:
            out[name] = val.double_value
        elif val.type == 2:
            out[name] = val.integer_value
        elif val.type == 4:
            out[name] = val.string_value
        else:
            out[name] = None
    return out


def check_publishers(node, settle=5.0):
    """토픽마다 발행자가 정확히 하나인지 본다."""
    end = time.monotonic() + settle
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    counts = {t: node.count_publishers(t) for t in SINGLE_PUBLISHER_TOPICS}
    return counts, {t: c for t, c in counts.items() if c > 1}


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'run'
    stamp = datetime.now().strftime('%m%d_%H%M')
    outdir = os.path.expanduser(f'~/vica_amcl_logs/{label}_{stamp}')

    rclpy.init()
    node = AmclProbe(label, outdir)

    params = fetch_params(node)
    if params:
        node.get_logger().info('=' * 62)
        node.get_logger().info(f'회차 [{label}] 의 AMCL 실제 값:')
        for k in WATCHED_PARAMS:
            node.get_logger().info(f'    {k:20s} = {params.get(k)}')
        node.get_logger().info('=' * 62)
    node.params = params

    counts, dup = check_publishers(node)
    node.get_logger().info('토픽 발행자 수 (각 1개여야 한다):')
    for topic, cnt in counts.items():
        mark = 'OK' if cnt == 1 else ('*** 중복 ***' if cnt > 1 else '없음')
        node.get_logger().info(f'    {topic:16s} {cnt}  {mark}')
    if dup:
        node.get_logger().error(
            '=' * 62 + '\n'
            f'  노드가 중복 실행됐다: {", ".join(dup)}\n'
            '  이대로 달리면 오도메트리가 뒤섞여 회차 전체가 무효가 된다.\n'
            '  중복된 노드를 끄고 다시 시작하라 (ps 로 PID 확인 후 개별 kill).\n'
            '  그래도 진행하려면: VICA_PROBE_FORCE=1 을 붙여 실행한다.\n'
            + '=' * 62)
        if not os.environ.get('VICA_PROBE_FORCE'):
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(2)
    node.get_logger().info(f'기록 시작. 로그: {outdir}')

    stop = {'now': False}

    def on_sigint(_sig, _frm):
        stop['now'] = True

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    try:
        while rclpy.ok() and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        summary = node.save(params)
        print('\n' + '=' * 62)
        print(f'  회차 [{label}] 요약')
        print('=' * 62)
        for k, v in summary.items():
            if k == 'amcl_params':
                continue
            print(f'  {k:32s} {v}')
        print('=' * 62)
        if not summary.get('valid'):
            print('  \033[31m이 회차는 판정에 쓸 수 없다.\033[0m '
                  f"오도메트리가 {summary['odom_jumps']}회 튀었다 "
                  f"(최대 {summary['odom_jump_max_m']} m).")
            print('  노드 중복을 잡고 다시 재라.')
        print(f'  저장: {outdir}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
