#!/usr/bin/env python3
"""제자리 회전 벤치 — 센서별 누적 yaw 를 나란히 보여 준다."""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Bench(Node):
    def __init__(self):
        super().__init__("vica_yaw_bench")
        self.gyro = 0.0
        self.t_imu = None
        self.wheel = None
        self.wheel_acc = 0.0
        self.ekf = None
        self.ekf_acc = 0.0
        self.create_subscription(Imu, "/imu/base_link", self.on_imu, 50)
        self.create_subscription(Odometry, "/wheel/odom", self.on_wheel, 20)
        self.create_subscription(Odometry, "/odom", self.on_ekf, 50)
        self.create_timer(2.0, self.show)
        self.t0 = time.time()

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t_imu is not None:
            self.gyro += m.angular_velocity.z * (t - self.t_imu)
        self.t_imu = t

    def _acc(self, prev, now):
        d = now - prev
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def on_wheel(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.wheel is not None:
            self.wheel_acc += self._acc(self.wheel, y)
        self.wheel = y

    def on_ekf(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.ekf is not None:
            self.ekf_acc += self._acc(self.ekf, y)
        self.ekf = y

    def line(self):
        g, w, e = (math.degrees(v) for v in (self.gyro, self.wheel_acc, self.ekf_acc))
        turns = g / 360.0
        return (f"{time.time() - self.t0:6.0f}s  자이로 {g:+8.1f}°  바퀴 {w:+8.1f}°  "
                f"EKF {e:+8.1f}°   ({turns:+.2f} 바퀴, EKF-자이로 {e - g:+.1f}°, 바퀴-자이로 {w - g:+.1f}°)")

    def show(self):
        print(self.line(), flush=True)


def main():
    rclpy.init()
    n = Bench()
    print("제자리 회전 시작 — Ctrl+C 로 종료. 정답은 바퀴수 x 360°", flush=True)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    print("\n=== 최종 ===")
    print(n.line())
    g = math.degrees(n.gyro)
    turns = round(g / 360.0)
    if turns:
        e = math.degrees(n.ekf_acc)
        print(f"EKF−자이로 {e - g:+.1f}°  → {'합격' if abs(e - g) <= 3 else '불합격'} (±3°)")
        print(f"참고: 정답 {turns} 바퀴 = {turns * 360}° 대비 EKF {e - turns * 360:+.1f}°, "
              f"자이로 {g - turns * 360:+.1f}° (세운 방향 오차 포함)")
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
