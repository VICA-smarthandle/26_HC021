#!/usr/bin/env python3
"""모드 노드 — 앱이 주행 묶음(vica-drive.target)을 켜고 끄는 창구.

    /vica/mode/drive_start     std_srvs/Trigger   주행 묶음 올림(이미 떠 있으면 그대로 성공)
    /vica/mode/drive_stop      std_srvs/Trigger   주행 묶음만 내림(기본 묶음·motor 는 그대로)
    /vica/mode/nav2_restart    std_srvs/Trigger   Nav2 유닛만 다시 띄움(기동 실패 때)
    /vica/mode_status          std_msgs/String    JSON, 2초마다 + TRANSIENT_LOCAL

프로세스를 직접 소유하지 않는다. systemctl --user 에 맡기고 결과는 매 초 다시 읽는다.
이 노드가 죽어도 주행 묶음은 그대로 돌고, 다시 뜬 노드는 같은 상태를 다시 읽는다.
판정은 mode_logic.py(시험: scripts/tests/test_mode_logic.py).

서비스는 곧바로 답한다(systemctl --no-block). 2026-08-21 rosbridge 가 응답 없는 서비스
하나에 통째로 막힌 일이 있어 매핑 감독 노드와 같은 방식을 따른다. 끝났는지는 상태 토픽으로 본다.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vica_interfaces.msg import RobotState

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mode_logic as ml  # noqa: E402

DEFAULT_MANIFEST = HERE.parent / "units.json"
NAV2_UNIT = "vica-nav2.service"
NAV2_JOURNAL_GREP = "lifecycle_manager_(navigation|localization)|Please set the initial pose"
CAN_FLAGS = Path("/sys/class/net/can1/flags")
FRESH_SEC = 3.0
SYSTEMCTL_TIMEOUT = 5.0
# 상태를 모으는 주기. 한 번에 CPU 약 35 ms(systemctl 20 · /proc 12) — 1초면 노드만 4 % 남짓이라
# 2초로 둔다. 앱이 보는 진행 표시는 2초면 충분하다.
TICK_SEC = 2.0


def systemctl(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["systemctl", "--user", *args], capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT
    )


class ModeNode(Node):
    def __init__(self, manifest_path: Path) -> None:
        super().__init__("vica_mode_node")
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.units = [e["unit"] for e in self.manifest.values()]
        self.patterns = {
            e["guard_cmdline"]: re.compile(e["guard_cmdline"])
            for e in self.manifest.values() if e.get("guard_cmdline")
        }
        self.names = {n for e in self.manifest.values() for n in e.get("guard", ())}
        self.names |= {"cartographer_no", "cartographer_oc"}

        self.lock = threading.Lock()
        self.safety: tuple[str, float] | None = None
        self.mapping: tuple[str, float] | None = None
        self.moving: tuple[bool, float] | None = None
        self.nav2_cache: tuple[str, str, float] | None = None   # (호출 ID, 단계, 읽은 시각)
        self.last_state = None
        # 자동 재시도를 이미 쓴 주행 묶음(그 묶음이 active 가 된 시각, systemd 단조 μs). 묶음을
        # 새로 올리면 값이 바뀌어 다시 한 번 쓸 수 있다. 노드가 다시 뜨면 잊는다 — 그때는 한 번 더.
        self.retried_for = None
        self.drive_entered = None
        # 다시 띄우라고 한 Nav2 호출 ID. 그 호출이 내려가는 동안 저널에는 옛 '중단' 이 남아 있으므로
        # 새 호출이 뜰 때까지 단계를 starting 으로 본다(11-16 실기: 재시도 직후 '재시도도 실패' 로 보임).
        self.nav2_inv = None
        self.replaced_inv = None

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "/vica/mode_status", latched)
        self.create_subscription(String, "/safety_state", self.on_safety, 10)
        self.create_subscription(String, "/vica/mapping_status", self.on_mapping, latched)
        self.create_subscription(RobotState, "/vica/robot_state", self.on_robot_state, 10)

        self.create_service(Trigger, "/vica/mode/drive_start", self.on_drive_start)
        self.create_service(Trigger, "/vica/mode/drive_stop", self.on_drive_stop)
        self.create_service(Trigger, "/vica/mode/nav2_restart", self.on_nav2_restart)
        self.create_timer(TICK_SEC, self.tick)
        self.get_logger().info(
            f"모드 노드 시작: 유닛 {len(self.units)}개 ({manifest_path}). "
            "/vica/mode/drive_start · drive_stop · nav2_restart → /vica/mode_status"
        )

    # ---- 구독 ---------------------------------------------------------------
    def on_safety(self, msg: String) -> None:
        with self.lock:
            self.safety = (msg.data, time.monotonic())

    def on_mapping(self, msg: String) -> None:
        try:
            state = str(json.loads(msg.data).get("state", ""))
        except (ValueError, AttributeError):
            return
        with self.lock:
            self.mapping = (state, time.monotonic())

    def on_robot_state(self, msg: RobotState) -> None:
        with self.lock:
            self.moving = (bool(msg.is_moving), time.monotonic())

    # ---- 사실 모으기 ------------------------------------------------------------
    def scan_processes(self) -> tuple[dict[str, int], dict[str, int]]:
        """프로세스 표를 한 번 훑는다. 500개 남짓이라 Path 대신 open 으로 읽는다(28 → 12 ms)."""
        procs = {n: 0 for n in self.names}
        hits = {p: 0 for p in self.patterns}
        # 자기 자신도 센다 — 모드 유닛이 살아 있음을 증명하는 것이 이 프로세스다.
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                with open(f"/proc/{pid}/comm", "rb") as f:
                    comm = f.read()[:-1].decode(errors="replace")
                if comm in procs:
                    procs[comm] += 1
                if hits and ml.needs_cmdline(comm):
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
                    for text, pattern in self.patterns.items():
                        if pattern.search(cmd):
                            hits[text] += 1
            except OSError:
                continue
        return procs, hits

    def nav2_phase(self, units, invocations) -> str:
        if units.get(NAV2_UNIT, ("inactive",))[0] not in ("active", "activating", "deactivating"):
            return "down"
        inv = invocations.get(NAV2_UNIT, "")
        if inv and inv == self.replaced_inv:
            return "starting"
        cached = self.nav2_cache
        now = time.monotonic()
        if cached and cached[0] == inv and (cached[1] in ("active", "aborted") or now - cached[2] < 2.0):
            return cached[1]
        try:
            out = subprocess.run(
                ["journalctl", "--user", f"_SYSTEMD_INVOCATION_ID={inv}", "-o", "cat",
                 "--no-pager", "-g", NAV2_JOURNAL_GREP],
                capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT,
            ).stdout.splitlines()
        except (OSError, subprocess.TimeoutExpired):
            out = []
        phase = ml.nav2_phase(out)
        self.nav2_cache = (inv, phase, now)
        return phase

    def collect(self) -> ml.Snapshot:
        show = systemctl(
            "show", "-p", "Id,ActiveState,SubState,Result,InvocationID,ActiveEnterTimestampMonotonic",
            ml.BASE_TARGET, ml.DRIVE_TARGET, *self.units,
        )
        units, invocations, entered = ml.parse_show(show.stdout)
        mono_us = time.monotonic() * 1e6   # systemd 의 ...Monotonic 과 같은 시계
        unit_age = {u: (mono_us - us) / 1e6 for u, us in entered.items()}
        self.drive_entered = entered.get(ml.DRIVE_TARGET)
        self.nav2_inv = invocations.get(NAV2_UNIT)
        procs, hits = self.scan_processes()
        try:
            can_up = bool(int(CAN_FLAGS.read_text().strip(), 16) & 0x1)
        except (OSError, ValueError):
            can_up = False
        now = time.monotonic()
        with self.lock:
            safety = self.safety[0] if self.safety and now - self.safety[1] < FRESH_SEC else None
            mapping = self.mapping[0] if self.mapping else None
            moving = self.moving[0] if self.moving and now - self.moving[1] < FRESH_SEC else None
        return ml.Snapshot(
            units=units, procs=procs, cmdline_hits=hits,
            nav2_phase=self.nav2_phase(units, invocations),
            safety_state=safety, mapping_state=mapping, robot_moving=moving, can_up=can_up,
            unit_age=unit_age,
            auto_retried=self.retried_for is not None and self.retried_for == self.drive_entered,
        )

    # ---- 주기 발행 ------------------------------------------------------------
    def tick(self) -> None:
        try:
            snap = self.collect()
            if ml.should_auto_retry(snap):
                # Nav2 기동 중단(bond·응답 유실) → Nav2 만 한 번 다시 띄운다(사용자 11-16).
                self.retried_for = self.drive_entered
                self.replaced_inv = self.nav2_inv
                done = systemctl("restart", "--no-block", NAV2_UNIT)
                self.get_logger().warn(
                    f"[자동 재시도] Nav2 기동 중단 → {NAV2_UNIT} 다시 띄움"
                    f" (rc={done.returncode}). 두 번째도 중단되면 사람에게 넘긴다."
                )
                snap.auto_retried = True
                snap.nav2_phase = "starting"
            status = ml.derive_status(snap, self.manifest)
        except (OSError, subprocess.TimeoutExpired) as exc:
            status = {"mode": "unknown", "state": "error", "detail": f"상태를 읽지 못했습니다: {exc}"}
        status["timestamp"] = time.time()
        self.pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        key = (status.get("mode"), status.get("state"), status.get("detail"), status.get("reset_needed"))
        if key != self.last_state:
            self.last_state = key
            self.get_logger().info(
                f"[모드] {status.get('mode')} · {status.get('state')}"
                f"{' · ' + status['detail'] if status.get('detail') else ''}"
                f" · nav2={status.get('nav2')} · safety={status.get('safety')}"
            )

    # ---- 서비스 ---------------------------------------------------------------
    def run(self, response, refusal: str | None, args: list[str], ok: str) -> Trigger.Response:
        if refusal:
            response.success, response.message = False, refusal
            self.get_logger().warn(f"[거부] {' '.join(args)}: {refusal}")
            return response
        try:
            done = systemctl(*args)
        except (OSError, subprocess.TimeoutExpired) as exc:
            response.success, response.message = False, f"systemctl 실패: {exc}"
            return response
        if done.returncode != 0:
            response.success = False
            response.message = f"systemctl 실패: {done.stderr.strip()[:200]}"
            self.get_logger().error(f"[실패] {' '.join(args)}: {done.stderr.strip()}")
            return response
        response.success, response.message = True, ok
        self.get_logger().info(f"[요청] {' '.join(args)}")
        return response

    def on_drive_start(self, _request, response):
        snap = self.collect()
        if ml._state(snap, ml.DRIVE_TARGET) in ("active", "activating"):
            response.success, response.message = True, "이미 주행 모드입니다."
            return response
        return self.run(
            response, ml.drive_start_refusal(snap, self.manifest),
            ["start", "--no-block", ml.DRIVE_TARGET],
            "주행 묶음을 올립니다. 진행은 /vica/mode_status 로 봅니다.",
        )

    def on_drive_stop(self, _request, response):
        snap = self.collect()
        if ml._state(snap, ml.DRIVE_TARGET) == "inactive":
            response.success, response.message = True, "주행 묶음이 이미 꺼져 있습니다."
            return response
        return self.run(
            response, ml.drive_stop_refusal(snap),
            ["stop", "--no-block", ml.DRIVE_TARGET],
            "주행 묶음을 내립니다(기본 묶음은 그대로).",
        )

    def on_nav2_restart(self, _request, response):
        snap = self.collect()
        inv = self.nav2_inv
        response = self.run(
            response, ml.nav2_restart_refusal(snap),
            ["restart", "--no-block", NAV2_UNIT],
            "Nav2 를 다시 띄웁니다. 초기 위치를 다시 잡아야 합니다.",
        )
        if response.success:
            self.replaced_inv = inv
        return response


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=[sys.argv[0], *ros_args])
    node = ModeNode(args.manifest)
    # 단일 스레드. 서비스는 --no-block 이라 0.1초 안에 끝나고, 다중 스레드 실행기는
    # 30 Hz /safety_state 만으로도 CPU 를 더 쓴다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
