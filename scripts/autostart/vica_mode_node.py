#!/usr/bin/env python3
"""모드 노드 — 앱이 주행 묶음(vica-drive.target)을 켜고 끄는 창구.

    /vica/mode/drive_start     std_srvs/Trigger   주행 묶음 올림(이미 떠 있으면 그대로 성공)
    /vica/mode/drive_stop      std_srvs/Trigger   주행 묶음만 내림(기본 묶음·motor 는 그대로)
    /vica/mode/nav2_restart    std_srvs/Trigger   Nav2 유닛만 다시 띄움(기동 실패 때)
    /vica/mode/detector_on|off std_srvs/Trigger   ⑮ 감지기만 켜고 끔(잠시 끄기, 주행을 다시 켜면 켜짐)
    /vica/mode/voice_online|offline  Trigger      음성을 ⑫ 온라인 / ⑬ 오프라인 믿음으로(voice_mode 파일)
    /vica/mode_status          std_msgs/String    JSON, 2초마다 + TRANSIENT_LOCAL

프로세스를 직접 소유하지 않는다. systemctl --user 에 맡기고 결과는 매 초 다시 읽는다.
이 노드가 죽어도 주행 묶음은 그대로 돌고, 다시 뜬 노드는 같은 상태를 다시 읽는다.
판정은 mode_logic.py(시험: scripts/tests/test_mode_logic.py).

서비스는 곧바로 답한다(systemctl --no-block). 2026-08-21 rosbridge 가 응답 없는 서비스
하나에 통째로 막힌 일이 있어 매핑 감독 노드와 같은 방식을 따른다. 끝났는지는 상태 토픽으로 본다.
"""
from __future__ import annotations

import argparse
import collections
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
# 음성 유닛(vica-llm)이 기동 때 읽는 파일. 생성기 VOICE_MODE_FILE 과 같은 곳.
VOICE_MODE_PATH = HERE.parent / "voice_mode"
# 음성 연결 알림(사용자 11-16)에 쓰는 저널 줄만 고른다. 판정은 ml.VoiceEventTracker.
VOICE_JOURNAL_GREP = "음성: |\\[LLM\\]|LLM 워밍업|Realtime 연결 예열"
# 같은 알림(예: 말마다 나는 실패)은 이 안에 한 번만.
VOICE_EVENT_REPEAT_SEC = 60.0
NAV2_UNIT = ml.NAV2_UNIT
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
        # 음성 연결 알림. 노드가 뜨기 전 일은 상태만 맞추고 알리지 않는다(다시 떠도 지난 알림이 안 온다).
        self.started_wall = time.time()
        self.invocations: dict[str, str] = {}
        self.voice_inv: str | None = None
        self.voice_cursor: str | None = None
        self.voice_tracker = ml.VoiceEventTracker()
        self.voice_events: collections.deque = collections.deque(maxlen=20)
        self.voice_event_id = 0
        self.voice_last_kind_at: dict[str, float] = {}
        # 미뤄 둔 정지 요청을 받은 시각(time.monotonic). 기동 중 정지는 Nav2 가 다 뜬 뒤 내린다(사용자 11-16).
        self.stop_pending_since: float | None = None
        self.drive_units = [e["unit"] for e in self.manifest.values() if e["bundle"] == "drive"]

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
        self.create_service(Trigger, "/vica/mode/detector_on", self.on_detector_on)
        self.create_service(Trigger, "/vica/mode/detector_off", self.on_detector_off)
        self.create_service(Trigger, "/vica/mode/voice_online", lambda q, r: self.on_voice(r, "online"))
        self.create_service(Trigger, "/vica/mode/voice_offline", lambda q, r: self.on_voice(r, "offline"))
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

    def nav2_phase(self, units, invocations, age_sec) -> str:
        state = units.get(NAV2_UNIT, ("inactive",))[0]
        if state not in ("active", "activating", "deactivating"):
            return "down"
        if state != "active":
            # activating 동안 ActiveEnterTimestamp 는 지난 호출 값이라 멈춤 판정에 못 쓴다.
            age_sec = None
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
        phase = ml.nav2_phase(out, age_sec)
        self.nav2_cache = (inv, phase, now)
        return phase

    def collect(self) -> ml.Snapshot:
        show = systemctl(
            "show", "-p",
            "Id,ActiveState,SubState,Result,InvocationID,ActiveEnterTimestampMonotonic,"
            "ActiveExitTimestampMonotonic",
            ml.BASE_TARGET, ml.DRIVE_TARGET, *self.units,
        )
        units, invocations, entered = ml.parse_show(show.stdout)
        mono_us = time.monotonic() * 1e6   # systemd 의 ...Monotonic 과 같은 시계
        unit_age = {u: (mono_us - us) / 1e6 for u, us in entered.items()}
        exited = ml.parse_exits(show.stdout)
        going_down = [(mono_us - exited[u]) / 1e6 for u in self.drive_units
                      if units.get(u, ("",))[0] == "deactivating" and u in exited]
        self.drive_entered = entered.get(ml.DRIVE_TARGET)
        self.nav2_inv = invocations.get(NAV2_UNIT)
        self.invocations = invocations
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
            nav2_phase=self.nav2_phase(units, invocations, unit_age.get(NAV2_UNIT)),
            safety_state=safety, mapping_state=mapping, robot_moving=moving, can_up=can_up,
            unit_age=unit_age,
            auto_retried=self.retried_for is not None and self.retried_for == self.drive_entered,
            stop_pending_sec=None if self.stop_pending_since is None else now - self.stop_pending_since,
            stopping_sec=max(going_down) if going_down else None,
            voice_mode=self.voice_mode(),
        )

    # ---- 주기 발행 ------------------------------------------------------------
    def tick(self) -> None:
        try:
            snap = self.collect()
            if ml.deferred_stop_due(snap, self.manifest):
                # 미뤄 둔 정지: Nav2 가 '초기 위치 대기'에 이르렀거나 45 s 가 지났다.
                done = systemctl("stop", "--no-block", ml.DRIVE_TARGET)
                self.get_logger().info(
                    f"[예약 정지] {snap.stop_pending_sec:.1f} s 기다린 뒤 주행 묶음을 내린다"
                    f" (nav2={snap.nav2_phase}, rc={done.returncode})"
                )
                # 이번 주기의 상태는 '내리는 중'으로 둔다(snap 은 내리기 전 사실이다).
                self.stop_pending_since = None
            elif ml.should_auto_retry(snap):
                # elif: 방금 정지를 내렸으면 재시도하지 않는다 — snap 의 target 은 아직 active 다.
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
            self.poll_voice()
            status["voice_events"] = list(self.voice_events)[-5:]
            status["voice_events_epoch"] = self.started_wall
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
        if self.stop_pending_since is not None:
            # 기동 중 정지를 미뤄 두었는데 다시 주행을 눌렀다 — 정지를 취소하고 그대로 둔다.
            self.stop_pending_since = None
            self.get_logger().info("[예약 정지 취소] 주행 요청")
            response.success, response.message = True, "정지 예약을 취소했습니다. 주행 모드를 유지합니다."
            return response
        snap = self.collect()
        if ml.drive_on(snap, self.manifest):
            response.success, response.message = True, "이미 주행 모드입니다(켜져 있거나 뜨는 중)."
            return response
        return self.run(
            response, ml.drive_start_refusal(snap, self.manifest),
            ["start", "--no-block", ml.DRIVE_TARGET],
            "주행 묶음을 올립니다. 진행은 /vica/mode_status 로 봅니다.",
        )

    def on_drive_stop(self, _request, response):
        snap = self.collect()
        if not ml.drive_on(snap, self.manifest):
            # target 만 보면 뜨는 중에도 inactive 다(11-16 16:10 실기). 소속 유닛까지 본다.
            going = any(snap.units.get(u, ("",))[0] == "deactivating" for u in self.drive_units)
            response.success = True
            response.message = "주행 묶음을 내리는 중입니다." if going else "주행 묶음이 이미 꺼져 있습니다."
            return response
        if self.stop_pending_since is not None:
            response.success, response.message = True, "이미 정지를 예약했습니다. Nav2 가 다 뜬 뒤 내립니다."
            return response
        refusal = ml.drive_stop_refusal(snap)
        if refusal is None and ml.stop_should_wait(snap, self.manifest):
            # 기동 중에 내리면 강제 종료·정지 지연이 는다(11-16 시험). 다 뜬 뒤(늦어도 45 s) 내린다.
            self.stop_pending_since = time.monotonic()
            self.get_logger().info("[예약 정지] Nav2 기동 중 — 다 뜬 뒤 내린다")
            response.success = True
            response.message = (f"Nav2 가 뜨는 중입니다. 다 뜬 뒤(늦어도 {ml.STOP_DEFER_MAX_SEC:.0f} s 안) "
                                "내립니다. 진행은 /vica/mode_status 로 봅니다.")
            return response
        return self.run(
            response, refusal,
            ["stop", "--no-block", ml.DRIVE_TARGET],
            "주행 묶음을 내립니다(기본 묶음은 그대로).",
        )

    def poll_voice(self) -> None:
        """음성 유닛의 이번 기동 저널에서 새 줄만 읽어 연결 알림을 만든다."""
        unit = self.manifest.get("llm", {}).get("unit")
        inv = self.invocations.get(unit, "") if unit else ""
        if not inv:
            return
        if inv != self.voice_inv:
            self.voice_inv, self.voice_cursor = inv, None
            self.voice_tracker = ml.VoiceEventTracker()
        args = ["journalctl", "--user", f"_SYSTEMD_INVOCATION_ID={inv}", "-o", "short-unix",
                "--no-pager", "--show-cursor", "-g", VOICE_JOURNAL_GREP]
        if self.voice_cursor:
            args.append(f"--after-cursor={self.voice_cursor}")
        try:
            lines = subprocess.run(args, capture_output=True, text=True,
                                   timeout=SYSTEMCTL_TIMEOUT).stdout.splitlines()
        except (OSError, subprocess.TimeoutExpired):
            return
        for line in lines:
            if line.startswith("-- cursor: "):
                self.voice_cursor = line[len("-- cursor: "):].strip()
                continue
            stamp, _, rest = line.partition(" ")
            try:
                at = float(stamp)
            except ValueError:
                continue
            message = rest.split(": ", 1)[1] if ": " in rest else rest
            for kind, text in self.voice_tracker.feed(message):
                if at < self.started_wall - 1.0:
                    continue
                last = self.voice_last_kind_at.get(kind)
                if last is not None and at - last < VOICE_EVENT_REPEAT_SEC:
                    continue
                self.voice_last_kind_at[kind] = at
                self.voice_event_id += 1
                self.voice_events.append({"id": self.voice_event_id, "at": at, "kind": kind, "text": text})
                self.get_logger().info(f"[음성 알림] {text}")

    def voice_mode(self) -> str:
        try:
            return ml.read_voice_mode(VOICE_MODE_PATH.read_text(encoding="utf-8"))
        except OSError:
            return "offline"

    def on_detector_on(self, _request, response):
        unit = self.manifest.get("detector", {}).get("unit", "vica-detector.service")
        return self.run(
            response, ml.detector_refusal(self.collect(), self.manifest),
            ["start", "--no-block", unit],
            "감지기를 켭니다(엔진을 올리는 데 10~20초).",
        )

    def on_detector_off(self, _request, response):
        unit = self.manifest.get("detector", {}).get("unit", "vica-detector.service")
        return self.run(
            response, ml.detector_refusal(self.collect(), self.manifest),
            ["stop", "--no-block", unit],
            "감지기를 잠시 끕니다. 주행을 다시 켜면 다시 켜집니다.",
        )

    def on_voice(self, response, mode: str):
        """고른 음성을 파일에 적고, 음성이 떠 있으면 다시 띄운다(말이 끊긴다)."""
        snap = self.collect()
        refusal = ml.voice_switch_refusal(snap)
        label = "온라인" if mode == "online" else "오프라인 믿음"
        if refusal:
            response.success, response.message = False, refusal
            self.get_logger().warn(f"[거부] 음성 {mode}: {refusal}")
            return response
        try:
            VOICE_MODE_PATH.write_text(mode + "\n", encoding="utf-8")
        except OSError as exc:
            response.success, response.message = False, f"음성 설정을 저장하지 못했습니다: {exc}"
            return response
        unit = self.manifest.get("llm", {}).get("unit", "vica-llm.service")
        if ml._state(snap, unit) in ("active", "activating"):
            return self.run(response, None, ["restart", "--no-block", unit],
                            f"음성을 {label}(으)로 다시 띄웁니다(약 10초).")
        self.get_logger().info(f"[음성] 다음 기동부터 {mode}")
        response.success, response.message = True, f"음성을 {label}(으)로 정했습니다. 주행을 켤 때 적용됩니다."
        return response

    def on_nav2_restart(self, _request, response):
        snap = self.collect()
        inv = self.nav2_inv
        refusal = ml.nav2_restart_refusal(snap)
        if refusal is None and self.stop_pending_since is not None:
            refusal = "정지를 예약해 두었습니다. 다시 띄우려면 먼저 주행을 다시 누르세요."
        response = self.run(
            response, refusal,
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
