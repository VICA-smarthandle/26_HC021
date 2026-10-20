#!/usr/bin/env python3
"""주행 멈칫 채증 — cmd_vel 발행 끊김을 시각(epoch)과 함께 기록한다."""
from __future__ import annotations

import argparse
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

MOVING_V = 0.02
MOVING_W = 0.05


class Watch:
    def __init__(self, topic: str, gap_sec: float) -> None:
        self.topic = topic
        self.gap_sec = gap_sec
        self.last_at: float | None = None
        self.last_moving = False
        self.gaps: list[tuple[float, float]] = []

    def on_msg(self, msg: Twist) -> None:
        now = time.time()
        if self.last_at is not None:
            dt = now - self.last_at
            if dt >= self.gap_sec and self.last_moving:
                self.gaps.append((self.last_at, dt))
                stamp = datetime.fromtimestamp(self.last_at).strftime("%H:%M:%S")
                print(f"[{self.last_at:.3f}] 끊김 {self.topic}: {dt:.2f}s "
                      f"(시작 {stamp}, 움직이던 중)", flush=True)
        self.last_at = now
        self.last_moving = (abs(msg.linear.x) > MOVING_V
                            or abs(msg.angular.z) > MOVING_W)


class StutterProbe(Node):
    def __init__(self, gap_sec: float) -> None:
        super().__init__("vica_stutter_probe")
        self.watches = [Watch("/cmd_vel_req", gap_sec),
                        Watch("/cmd_vel_safe", gap_sec)]
        for w in self.watches:
            self.create_subscription(Twist, w.topic, w.on_msg, 10)
        print(f"멈칫 채증 시작 — 문턱 {gap_sec}s, 대상: "
              f"{', '.join(w.topic for w in self.watches)} (Ctrl+C 로 요약)",
              flush=True)

    def summary(self) -> None:
        print("\n===== 요약 =====")
        for w in self.watches:
            if not w.gaps:
                print(f"{w.topic}: 끊김 0회")
                continue
            longest = sorted(w.gaps, key=lambda g: -g[1])[:5]
            print(f"{w.topic}: 끊김 {len(w.gaps)}회, "
                  f"최장 {longest[0][1]:.2f}s")
            for at, dt in longest:
                stamp = datetime.fromtimestamp(at).strftime("%H:%M:%S")
                print(f"  [{at:.3f}] {stamp} — {dt:.2f}s")
        print("STT 대조:  grep \"계측\" ~/.ros/log/<웨이크워드>.log")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap", type=float, default=0.3,
                    help="끊김으로 볼 최소 공백(초), 기본 0.3")
    args = ap.parse_args()
    rclpy.init()
    node = StutterProbe(args.gap)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()


if __name__ == "__main__":
    main()
