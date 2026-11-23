#!/usr/bin/env python3
"""모드 노드 — 앱이 주행 묶음(vica-drive.target)을 켜고 끄는 창구."""
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
from geometry_msgs.msg import Twist
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
VOICE_MODE_PATH = HERE.parent / "voice_mode"
VOICE_JOURNAL_GREP = "음성: |\\[LLM\\]|LLM 워밍업|Realtime 연결 예열|\\[Ollama\\] "
VOICE_EVENT_REPEAT_SEC = 60.0
NAV2_UNIT = ml.NAV2_UNIT
NAV2_JOURNAL_GREP = "lifecycle_manager_(navigation|localization)|Please set the initial pose"
MAPS_DIR = Path(os.environ.get("VICA_ROS_WS", str(Path.home() / "VICA-smarthandle/vica_ros2_ws"))) / "maps"
CURRENT_MAP = MAPS_DIR / "CURRENT_MAP"
MAP_JOURNAL_GREP = "\\[vica-map\\] "
CAN_FLAGS = Path("/sys/class/net/can1/flags")
FRESH_SEC = 3.0
SYSTEMCTL_TIMEOUT = 5.0
TICK_SEC = 2.0
BRIDGE_URL = os.environ.get("VICA_BRIDGE_URL", "ws://127.0.0.1:9090")
BRIDGE_WATCH_ON = os.environ.get("VICA_BRIDGE_WATCH", "1") != "0"
SHUTDOWN_SCRIPT = HERE / "vica_shutdown.sh"
SHUTDOWN_UNIT = "vica-shutdown"
VOICE_ROOT = Path(os.environ.get("VICA_VOICE_ROOT", str(Path.home() / "VICA-smarthandle/vica-voice-llm")))
DESTINATIONS_DIR = Path.home() / "vica_data" / "destinations"


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
        self.dialog: tuple[str, float] | None = None
        self.cmd_nonzero_at: float | None = None
        self.map_marks: dict[str, str] = {}
        self.map_change: dict | None = None
        self.handled_map_requests: collections.deque = collections.deque(maxlen=32)
        self.nav2_cache: tuple[str, str, float] | None = None
        self.nav2_read_failed = False
        self.last_state = None
        self.retried_for = None
        self.drive_entered = None
        self.nav2_inv = None
        self.replaced_inv = None
        self.started_wall = time.time()
        self.invocations: dict[str, str] = {}
        self.voice_inv: str | None = None
        self.voice_cursor: str | None = None
        self.voice_tracker = ml.VoiceEventTracker()
        self.voice_events: collections.deque = collections.deque(maxlen=20)
        self.voice_event_id = 0
        self.voice_last_kind_at: dict[str, float] = {}
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
        self.create_subscription(String, "/vica/mode/map_request", self.on_map_request, 5)
        self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_safe, 10)

        self.create_service(Trigger, "/vica/mode/drive_start", self.on_drive_start)
        self.create_service(Trigger, "/vica/mode/drive_stop", self.on_drive_stop)
        self.create_service(Trigger, "/vica/mode/shutdown", self.on_shutdown)
        self.create_service(Trigger, "/vica/mode/nav2_restart", self.on_nav2_restart)
        self.create_service(Trigger, "/vica/mode/detector_on", self.on_detector_on)
        self.create_service(Trigger, "/vica/mode/detector_off", self.on_detector_off)
        self.create_service(Trigger, "/vica/mode/voice_on", lambda q, r: self.on_voice_node(r, True))
        self.create_service(Trigger, "/vica/mode/voice_off", lambda q, r: self.on_voice_node(r, False))
        self.create_service(Trigger, "/vica/mode/mic_check_start", lambda q, r: self.on_mic_check_start(r))
        self.create_service(Trigger, "/vica/mode/mic_check_stop", lambda q, r: self.on_mic_check_stop(r))
        self.create_service(Trigger, "/vica/mode/voice_online", lambda q, r: self.on_voice(r, "online"))
        self.create_service(Trigger, "/vica/mode/voice_offline", lambda q, r: self.on_voice(r, "offline"))
        self.create_timer(TICK_SEC, self.tick)
        self.get_logger().info(
            f"모드 노드 시작: 유닛 {len(self.units)}개 ({manifest_path}). "
            "/vica/mode/drive_start · drive_stop · nav2_restart → /vica/mode_status"
        )
        self.bridge = ml.BridgeWatch()
        self.bridge_lock = threading.Lock()
        self.bridge_stop = threading.Event()
        if BRIDGE_WATCH_ON:
            threading.Thread(target=self.watch_bridge, name="bridge_watch", daemon=True).start()

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
        now = time.monotonic()
        with self.lock:
            self.moving = (bool(msg.is_moving), now)
            self.dialog = (str(msg.dialog_state), now)

    def on_cmd_safe(self, msg: Twist) -> None:
        if ml.twist_moving(msg.linear.x, msg.angular.z):
            with self.lock:
                self.cmd_nonzero_at = time.monotonic()

    def scan_processes(self) -> tuple[dict[str, int], dict[str, int]]:
        """프로세스 표를 한 번 훑는다. 500개 남짓이라 Path 대신 open 으로 읽는다(28 → 12 ms)."""
        procs = {n: 0 for n in self.names}
        hits = {p: 0 for p in self.patterns}
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
            age_sec = None
        inv = invocations.get(NAV2_UNIT, "")
        if inv and inv == self.replaced_inv:
            return "starting"
        cached = self.nav2_cache
        now = time.monotonic()
        if cached and cached[0] == inv and (cached[1] in ("active", "aborted") or now - cached[2] < 2.0):
            return cached[1]
        previous = cached[1] if cached and cached[0] == inv else None
        try:
            out = subprocess.run(
                ["journalctl", "--user", f"_SYSTEMD_INVOCATION_ID={inv}", "-o", "cat",
                 "--no-pager", "-g", NAV2_JOURNAL_GREP],
                capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT,
            ).stdout.splitlines()
        except (OSError, subprocess.TimeoutExpired) as exc:
            out = None
            if not self.nav2_read_failed:
                self.get_logger().warn(
                    f"[Nav2 단계] 저널을 못 읽음({type(exc).__name__}) — 직전 판정 유지: {previous or 'starting'}")
        self.nav2_read_failed = out is None
        phase = ml.settle_nav2_phase(previous, out, age_sec)
        self.nav2_cache = (inv, phase, now)
        return phase

    def collect(self) -> ml.Snapshot:
        show = systemctl(
            "show", "-p",
            "Id,ActiveState,SubState,Result,InvocationID,ActiveEnterTimestampMonotonic,"
            "ActiveExitTimestampMonotonic",
            ml.BASE_TARGET, ml.DRIVE_TARGET, ml.MIC_CHECK_UNIT, *self.units,
        )
        units, invocations, entered = ml.parse_show(show.stdout)
        mono_us = time.monotonic() * 1e6
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
            dialog = self.dialog[0] if self.dialog and now - self.dialog[1] < FRESH_SEC else None
            cmd_moving = self.cmd_nonzero_at is not None and now - self.cmd_nonzero_at < ml.CMD_MOVING_SEC
        return ml.Snapshot(
            units=units, procs=procs, cmdline_hits=hits,
            nav2_phase=self.nav2_phase(units, invocations, unit_age.get(NAV2_UNIT)),
            safety_state=safety, mapping_state=mapping, robot_moving=moving, can_up=can_up,
            unit_age=unit_age,
            auto_retried=self.retried_for is not None and self.retried_for == self.drive_entered,
            stop_pending_sec=None if self.stop_pending_since is None else now - self.stop_pending_since,
            stopping_sec=max(going_down) if going_down else None,
            voice_mode=self.voice_mode(),
            dialog_state=dialog,
            cmd_moving=cmd_moving,
            current_map=self.read_current_map(),
            running_maps=self.running_maps(units, invocations),
        )

    def read_current_map(self) -> str:
        try:
            return CURRENT_MAP.read_text(encoding="utf-8").strip().splitlines()[0].strip()
        except (OSError, IndexError):
            return ""

    def running_maps(self, units, invocations) -> dict[str, str]:
        """지도를 쥔 유닛마다 이번 기동이 읽은 지도. 저널은 그 호출에서 줄을 찾을 때까지만 읽는다."""
        found: dict[str, str] = {}
        live = set()
        for key in ml.MAP_TRACKED:
            unit = self.manifest.get(key, {}).get("unit")
            inv = invocations.get(unit, "") if unit else ""
            if not inv or units.get(unit, ("",))[0] not in ("active", "activating"):
                continue
            live.add(inv)
            if inv not in self.map_marks:
                try:
                    lines = subprocess.run(
                        ["journalctl", "--user", f"_SYSTEMD_INVOCATION_ID={inv}", "-o", "cat",
                         "--no-pager", "-g", MAP_JOURNAL_GREP],
                        capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT,
                    ).stdout.splitlines()
                except (OSError, subprocess.TimeoutExpired):
                    lines = []
                mark = ml.parse_map_mark(lines)
                if mark:
                    self.map_marks[inv] = mark
            if inv in self.map_marks:
                found[key] = self.map_marks[inv]
        self.map_marks = {inv: m for inv, m in self.map_marks.items() if inv in live}
        return found

    def tick(self) -> None:
        try:
            snap = self.collect()
            if ml.deferred_stop_due(snap, self.manifest):
                done = systemctl("stop", "--no-block", ml.DRIVE_TARGET)
                self.get_logger().info(
                    f"[예약 정지] {snap.stop_pending_sec:.1f} s 기다린 뒤 주행 묶음을 내린다"
                    f" (nav2={snap.nav2_phase}, rc={done.returncode})"
                )
                self.stop_pending_since = None
            elif ml.should_auto_retry(snap):
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
            status["map_change"] = self.map_change
        except (OSError, subprocess.TimeoutExpired) as exc:
            status = {"mode": "unknown", "state": "error", "detail": f"상태를 읽지 못했습니다: {exc}"}
        status["timestamp"] = time.time()
        self.pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        with self.bridge_lock:
            self.bridge.published(time.monotonic())
        key = (status.get("mode"), status.get("state"), status.get("detail"), status.get("reset_needed"))
        if key != self.last_state:
            self.last_state = key
            self.get_logger().info(
                f"[모드] {status.get('mode')} · {status.get('state')}"
                f"{' · ' + status['detail'] if status.get('detail') else ''}"
                f" · nav2={status.get('nav2')} · safety={status.get('safety')}"
            )

    def robot_moving(self) -> bool:
        """움직이는 중이라고 확실할 때만 True. robot_state 가 끊겼으면(미션 꺼짐) 멈춘 것으로 본다."""
        with self.lock:
            m = self.moving
        return bool(m and m[0] and time.monotonic() - m[1] < FRESH_SEC)

    def watch_bridge(self) -> None:
        """실행기 밖 스레드. 웹소켓 수신이 서비스·상태 발행을 막지 않게 한다."""
        try:
            import websocket
        except ImportError as exc:
            self.get_logger().warn(f"[rosbridge 감시] 끔: websocket-client 없음 ({exc})")
            return
        log = self.get_logger()
        sub = json.dumps({"op": "subscribe", "topic": "/vica/mode_status", "type": "std_msgs/String"})
        last_hold = ""
        while not self.bridge_stop.is_set():
            try:
                ws = websocket.create_connection(BRIDGE_URL, timeout=3)
                ws.send(sub)
                ws.settimeout(1.0)
            except (OSError, websocket.WebSocketException):
                with self.bridge_lock:
                    self.bridge.disconnected()
                self.bridge_stop.wait(5.0)
                continue
            with self.bridge_lock:
                self.bridge.connected(time.monotonic())
            try:
                while not self.bridge_stop.is_set():
                    try:
                        raw = ws.recv()
                        if ml.bridge_message_is_ours(raw):
                            with self.bridge_lock:
                                self.bridge.received(time.monotonic())
                    except websocket.WebSocketTimeoutException:
                        pass
                    moving = self.robot_moving()
                    with self.bridge_lock:
                        verdict = self.bridge.verdict(time.monotonic(), moving)
                        hold = self.bridge.hold_reason
                    if verdict == "hold" and hold != last_hold:
                        log.warn(f"[rosbridge 감시] {ml.BRIDGE_SILENT_SEC:.0f} s 넘게 받은 것 없음 — "
                                 f"재시작 미룸({hold})")
                    last_hold = hold if verdict == "hold" else ""
                    if verdict == "restart":
                        done = systemctl("restart", "--no-block", ml.BRIDGE_APP_UNIT)
                        log.error(f"[rosbridge 감시] 소켓은 붙었는데 {ml.BRIDGE_SILENT_SEC:.0f} s 넘게 "
                                  f"/vica/mode_status 가 안 옴 → {ml.BRIDGE_APP_UNIT} 다시 띄움 "
                                  f"(rc={done.returncode})")
                        with self.bridge_lock:
                            self.bridge.restarted(time.monotonic())
                        break
            except (OSError, websocket.WebSocketException, subprocess.TimeoutExpired):
                pass
            finally:
                try:
                    ws.close()
                except (OSError, websocket.WebSocketException):
                    pass
                with self.bridge_lock:
                    self.bridge.disconnected()
            self.bridge_stop.wait(2.0)

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
            going = any(snap.units.get(u, ("",))[0] == "deactivating" for u in self.drive_units)
            response.success = True
            response.message = "주행 묶음을 내리는 중입니다." if going else "주행 묶음이 이미 꺼져 있습니다."
            return response
        if self.stop_pending_since is not None:
            response.success, response.message = True, "이미 정지를 예약했습니다. Nav2 가 다 뜬 뒤 내립니다."
            return response
        refusal = ml.drive_stop_refusal(snap)
        if refusal is None and ml.stop_should_wait(snap, self.manifest):
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

    def on_shutdown(self, _request, response):
        """로봇 끄기: 주행 묶음 → 기본 묶음 → sync → 시스템 종료(앱 '로봇 끄기', 사용자 11-18)."""
        refusal = ml.shutdown_refusal(self.collect())
        if refusal is not None:
            self.get_logger().warn(f"[로봇 끄기 거절] {refusal}")
            response.success, response.message = False, refusal
            return response
        try:
            done = subprocess.run(
                ["systemd-run", "--user", f"--unit={SHUTDOWN_UNIT}", "--collect", "--no-block",
                 str(SHUTDOWN_SCRIPT)],
                capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            response.success, response.message = False, f"로봇 끄기를 시작하지 못했습니다: {exc}"
            return response
        if done.returncode != 0:
            if "already" in done.stderr or "exists" in done.stderr:
                response.success, response.message = True, "이미 끄는 중입니다."
                return response
            response.success = False
            response.message = f"로봇 끄기를 시작하지 못했습니다: {done.stderr.strip()[:200]}"
            return response
        self.get_logger().warn("[로봇 끄기] 주행 묶음 → 기본 묶음 → sync → 시스템 종료")
        response.success, response.message = True, "로봇을 끕니다. 곧 연결이 끊깁니다."
        return response

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
            response, ml.detector_refusal(self.collect(), self.manifest, True),
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

    def on_voice_node(self, response, on: bool):
        """음성 노드(vica-llm)만 켜거나 끈다. 점검 앱이 칩 점검 전에 끈다. 점검이 끝나도 자동으로 켜지 않는다(설계 7절, 켜기는 앱에서)."""
        unit = self.manifest.get("llm", {}).get("unit", "vica-llm.service")
        return self.run(
            response, ml.voice_node_refusal(self.collect(), self.manifest, on),
            ["start" if on else "stop", "--no-block", unit],
            "음성 노드를 켭니다" if on else "음성 노드를 껐습니다",
        )

    def on_mic_check_start(self, response):
        """점검기를 일회성 유닛으로 띄운다. 주행 중 감지기가 떠 있으면 먼저 내린다."""
        snap = self.collect()
        refusal = ml.mic_check_start_refusal(snap, self.manifest)
        if refusal:
            response.success, response.message = False, refusal
            self.get_logger().warn(f"[거부] mic_check_start: {refusal}")
            return response
        try:
            for unit in ml.mic_check_side_stops(snap, self.manifest):
                systemctl("stop", "--no-block", unit)
            dest = DESTINATIONS_DIR / snap.current_map / "destinations.yaml"
            argv = ml.mic_check_command(
                str(VOICE_ROOT), snap.current_map, str(dest) if snap.current_map and dest.is_file() else None)
            done = subprocess.run(argv, capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT)
        except (OSError, subprocess.TimeoutExpired) as exc:
            response.success, response.message = False, f"systemctl 실패: {exc}"
            return response
        if done.returncode != 0:
            response.success = False
            response.message = f"systemctl 실패: {done.stderr.strip()[:200]}"
            self.get_logger().error(f"[실패] mic_check_start: {done.stderr.strip()}")
            return response
        response.success, response.message = True, "마이크 점검을 시작합니다 · 준비 1~2분"
        self.get_logger().info("[요청] mic_check_start")
        self.tick()
        return response

    def on_mic_check_stop(self, response):
        return self.run(
            response, ml.mic_check_stop_refusal(self.collect()),
            ["stop", "--no-block", ml.MIC_CHECK_UNIT], "점검을 끝냅니다",
        )

    def on_voice(self, response, mode: str):
        """고른 음성을 파일에 적고, 음성이 떠 있으면 다시 띄운다(말이 끊긴다)."""
        snap = self.collect()
        refusal = ml.voice_switch_refusal(snap)
        target = "외부 API로" if mode == "online" else "로컬 LLM으로"
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
                            f"음성을 {target} 다시 띄웁니다.")
        self.get_logger().info(f"[음성] 다음 기동부터 {mode}")
        response.success, response.message = True, f"음성을 {target} 정했습니다. 주행을 켤 때 적용됩니다."
        return response

    def map_problem(self, map_id: str) -> str | None:
        """지도 파일이 갖춰졌는가. yaml 과 그것이 가리키는 그림(image:)이 둘 다 있어야 한다."""
        yaml_path = MAPS_DIR / f"{map_id}.yaml"
        try:
            text = yaml_path.read_text(encoding="utf-8")
        except OSError:
            return f"지도 파일 없음({map_id}.yaml)"
        match = re.search(r"^image:\s*['\"]?([^'\"\n]+?)['\"]?\s*$", text, re.MULTILINE)
        if not match:
            return f"지도 파일에 그림 경로가 없음({map_id}.yaml)"
        image = Path(match.group(1))
        if not image.is_absolute():
            image = yaml_path.parent / image
        if not image.is_file():
            return f"지도 그림 없음({image.name})"
        return None

    def write_current_map(self, map_id: str) -> None:
        """CURRENT_MAP 을 원자적으로 바꾼다(쓰는 도중 유닛이 읽어도 반쪽 값을 보지 않게)."""
        tmp = CURRENT_MAP.with_name(f".CURRENT_MAP.{os.getpid()}.tmp")
        tmp.write_text(map_id + "\n", encoding="utf-8")
        os.replace(tmp, CURRENT_MAP)

    def on_map_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            request_id = str(payload.get("request_id", ""))[:64]
            map_id = str(payload.get("map_id", "")).strip()
        except (ValueError, AttributeError):
            self.get_logger().warn(f"[지도] 요청을 읽지 못함: {msg.data[:120]!r}")
            return
        if request_id and request_id in self.handled_map_requests:
            self.tick()
            return
        if request_id:
            self.handled_map_requests.append(request_id)
        snap = self.collect()
        state = ml.derive_status(snap, self.manifest)["state"]
        before = snap.current_map
        result = {"request_id": request_id, "map_id": map_id, "from": before, "at": time.time(),
                  "restarted": []}
        refusal = ml.map_change_refusal(snap, self.manifest, state, map_id,
                                        self.map_problem(map_id) if ml.MAP_ID_RE.match(map_id) else None)
        if refusal:
            self.map_change = dict(result, state="refused", message=refusal)
            self.get_logger().warn(f"[지도 거부] {before} → {map_id}: {refusal}")
            self.tick()
            return
        units = ml.map_restart_units(snap, self.manifest, map_id)
        try:
            if map_id != before:
                self.write_current_map(map_id)
            if units:
                done = systemctl("restart", "--no-block", *units)
                if done.returncode != 0:
                    raise OSError(done.stderr.strip()[:200])
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.map_change = dict(result, state="failed", message=f"바꾸지 못함: {exc}")
            self.get_logger().error(f"[지도 실패] {before} → {map_id}: {exc}")
            self.tick()
            return
        keys = [k for k in ml.MAP_RESTART if self.manifest.get(k, {}).get("unit") in units]
        if NAV2_UNIT in units:
            self.replaced_inv = self.nav2_inv
            self.retried_for = None
        message = "Nav2·미션·음성 재시작 · 약 1분" if units else "다음 주행부터 적용"
        self.map_change = dict(result, state="applied", message=message, restarted=keys)
        self.get_logger().info(f"[지도] {before} → {map_id} · 다시 띄움: {', '.join(keys) or '없음'}")
        self.tick()

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
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.bridge_stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
