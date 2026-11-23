"""모드 노드(vica_mode_node.py)의 판정 로직. ROS·systemd 를 부르지 않는다."""
from __future__ import annotations

import collections
import json
import math
import re
from dataclasses import dataclass, field

DRIVE_TARGET = "vica-drive.target"
NAV2_UNIT = "vica-nav2.service"
BASE_TARGET = "vica-base.target"
MIC_CHECK_UNIT = "vica-miccheck.service"
MIC_CHECK_PORT = 8770
MIC_CHECK_LAUNCHER_ENV = "VICA_MIC_CHECK_LAUNCHER"
CMD_MOVING_SEC = 1.0
MOVING_REFUSAL = "로봇이 움직이는 중입니다. 멈춘 뒤 다시 누르세요."
VOICE_OFF_GRACE_SEC = 60.0
MAPPING_IDLE = ("idle", "", None)
STARTUP_GRACE_SEC = 45.0
NAV2_STALL_SEC = 90.0
STOP_DEFER_MAX_SEC = 45.0
STOP_SLOW_SEC = 30.0

_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_NAV = "[lifecycle_manager_navigation]:"
_LOC = "[lifecycle_manager_localization]:"
_FAILED = "Failed to bring up all requested nodes"


@dataclass
class Snapshot:
    units: dict[str, tuple[str, str, str]]
    procs: dict[str, int]
    cmdline_hits: dict[str, int] = field(default_factory=dict)
    nav2_phase: str = "down"
    safety_state: str | None = None
    mapping_state: str | None = None
    robot_moving: bool | None = None
    can_up: bool = False
    unit_age: dict[str, float] = field(default_factory=dict)
    auto_retried: bool = False
    stop_pending_sec: float | None = None
    stopping_sec: float | None = None
    voice_mode: str = "offline"
    dialog_state: str | None = None
    current_map: str = ""
    running_maps: dict[str, str] = field(default_factory=dict)
    cmd_moving: bool = False


def parse_show(text: str) -> tuple[dict, dict[str, str], dict[str, int]]:
    """`systemctl --user show -p Id,ActiveState,SubState,Result,InvocationID,"""
    units: dict[str, tuple[str, str, str]] = {}
    invocations: dict[str, str] = {}
    entered: dict[str, int] = {}
    for block in text.strip().split("\n\n"):
        props = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
        name = props.get("Id")
        if not name:
            continue
        units[name] = (
            props.get("ActiveState", ""), props.get("SubState", ""), props.get("Result", "")
        )
        if props.get("InvocationID"):
            invocations[name] = props["InvocationID"]
        if props.get("ActiveEnterTimestampMonotonic", "0") not in ("", "0"):
            entered[name] = int(props["ActiveEnterTimestampMonotonic"])
    return units, invocations, entered


def parse_exits(text: str) -> dict[str, int]:
    """같은 show 출력에서 ActiveExitTimestampMonotonic(μs, 내려가기 시작한 때)."""
    exited: dict[str, int] = {}
    for block in text.strip().split("\n\n"):
        props = dict(line.split("=", 1) for line in block.splitlines() if "=" in line)
        if props.get("Id") and props.get("ActiveExitTimestampMonotonic", "0") not in ("", "0"):
            exited[props["Id"]] = int(props["ActiveExitTimestampMonotonic"])
    return exited


def nav2_phase(lines: list[str], age_sec: float | None = None) -> str:
    """이번 vica-nav2 호출의 저널 줄에서 Nav2 기동 단계를 고른다."""
    active = aborted = pose = False
    for raw in lines:
        line = _ANSI.sub("", raw)
        if _NAV in line and "Managed nodes are active" in line:
            active = True
        elif (_NAV in line or _LOC in line) and _FAILED in line:
            aborted = True
        elif "Please set the initial pose" in line:
            pose = True
    if aborted:
        return "aborted"
    if active:
        return "active"
    if pose:
        return "waiting_pose"
    if age_sec is not None and age_sec >= NAV2_STALL_SEC:
        return "stalled"
    return "starting"


def settle_nav2_phase(previous: str | None, lines: list[str] | None, age_sec: float | None) -> str:
    """같은 Nav2 호출에서 저널을 다시 읽은 결과로 단계를 정한다."""
    if lines is None:
        return previous or "starting"
    phase = nav2_phase(lines, age_sec)
    if phase == "stalled" and previous in ("waiting_pose", "active"):
        return previous
    return phase


def _state(snap: Snapshot, unit: str) -> str:
    return snap.units.get(unit, ("inactive", "dead", ""))[0]


def needs_cmdline(comm: str) -> bool:
    """명령줄까지 읽어야 하는 프로세스인가."""
    return comm in ("python3", "python", "ros2")


def _alive(snap: Snapshot, entry: dict) -> bool:
    if entry.get("guard_cmdline"):
        return snap.cmdline_hits.get(entry["guard_cmdline"], 0) > 0
    guard = entry.get("guard", ())
    if not guard:
        return True
    return any(snap.procs.get(name, 0) > 0 for name in guard)


def _duplicated(snap: Snapshot, manifest: dict) -> list[str]:
    dup = sorted(name for name, count in snap.procs.items() if count > 1)
    for key, entry in manifest.items():
        pattern = entry.get("guard_cmdline")
        if pattern and snap.cmdline_hits.get(pattern, 0) > 1:
            dup.append(key)
    return dup


MAPPING_SHARED = frozenset({"ekf_node", "encoder_feedbac", "cartographer_no", "cartographer_oc"})


def pane_nodes(snap: Snapshot, manifest: dict) -> list[str]:
    """유닛 밖(칸·터미널)에서 뜬 주행 노드의 감시 이름. 그 주행 유닛이 꺼져(inactive·failed) 있는데 프로세스가 있다."""
    names: set[str] = set()
    for entry in _bundle(manifest, "drive").values():
        if _state(snap, entry["unit"]) not in ("inactive", "failed"):
            continue
        names.update(n for n in entry.get("guard", ()) if n not in MAPPING_SHARED and snap.procs.get(n, 0))
    return sorted(names)


def _bundle(manifest: dict, bundle: str) -> dict:
    return {k: v for k, v in manifest.items() if v["bundle"] == bundle}


def drive_on(snap: Snapshot, manifest: dict) -> bool:
    """주행 묶음이 켜져 있거나 뜨는 중인가."""
    if _state(snap, DRIVE_TARGET) in ("active", "activating"):
        return True
    return any(_state(snap, e["unit"]) in ("active", "activating") for e in _bundle(manifest, "drive").values())


def derive_status(snap: Snapshot, manifest: dict) -> dict:
    """/vica/mode_status 에 실을 내용."""
    status = _derive_core(snap, manifest)
    status.update(map_fields(snap, manifest, status["state"]))
    return status


def _derive_core(snap: Snapshot, manifest: dict) -> dict:
    drive = _bundle(manifest, "drive")
    target = _state(snap, DRIVE_TARGET)

    failed = sorted(k for k, e in manifest.items() if _state(snap, e["unit"]) == "failed")
    absent = [k for k, e in manifest.items() if _state(snap, e["unit"]) == "active" and not _alive(snap, e)]
    young = [k for k in absent if snap.unit_age.get(manifest[k]["unit"], STARTUP_GRACE_SEC) < STARTUP_GRACE_SEC]
    missing = sorted(k for k in absent if k not in young)
    status = {
        "mode": "idle",
        "state": "idle",
        "detail": "",
        "nav2": snap.nav2_phase,
        "safety": snap.safety_state or "unknown",
        "reset_needed": snap.safety_state == "ESTOP_RELEASED_WAIT_RESET",
        "failed_units": failed,
        "missing_nodes": missing,
        "duplicated": _duplicated(snap, manifest),
        "pane_nodes": pane_nodes(snap, manifest),
        "can_up": snap.can_up,
        "stop_timeouts": sorted(k for k, e in drive.items()
                                if snap.units.get(e["unit"], ("", "", ""))[2] == "timeout"),
        "detector": _state(snap, manifest["detector"]["unit"]) if "detector" in manifest else "none",
        "voice": _state(snap, manifest["llm"]["unit"]) if "llm" in manifest else "none",
        "voice_mode": snap.voice_mode,
        "mic_check": {"state": _state(snap, MIC_CHECK_UNIT), "port": MIC_CHECK_PORT},
    }

    drive_states = {k: _state(snap, e["unit"]) for k, e in drive.items()}
    going_down = [k for k, st in drive_states.items() if st == "deactivating"]
    if drive_on(snap, manifest) or target == "deactivating" or going_down:
        status["mode"] = "drive"
        drive_failed = [k for k in failed if k in drive]
        drive_missing = [k for k in missing if k in drive]
        if snap.stop_pending_sec is not None and drive_on(snap, manifest):
            status["state"] = "stopping"
            status["stop_in_sec"] = round(max(0.0, STOP_DEFER_MAX_SEC - snap.stop_pending_sec))
            status["detail"] = f"Nav2 가 다 뜬 뒤 내립니다(늦어도 {STOP_DEFER_MAX_SEC:.0f} s 안)"
        elif target == "deactivating" or (going_down and target != "active"):
            status["state"] = "stopping"
            if snap.stopping_sec is not None and snap.stopping_sec >= STOP_SLOW_SEC:
                status["detail"] = (f"정지가 늦어지고 있습니다({snap.stopping_sec:.0f} s) — "
                                    "되풀이되면 재부팅이 필요합니다")
        elif drive_failed:
            status["state"] = "error"
            status["detail"] = "실패한 유닛: " + ", ".join(drive_failed)
        elif drive_missing:
            status["state"] = "error"
            status["detail"] = "노드가 없습니다: " + ", ".join(drive_missing)
        elif going_down:
            status["state"] = "starting"
            status["detail"] = "다시 띄우는 중: " + ", ".join(sorted(going_down))
        elif snap.nav2_phase in ("aborted", "stalled"):
            what = "중단" if snap.nav2_phase == "aborted" else f"{NAV2_STALL_SEC:.0f} s 넘게 진행 없음"
            if not snap.auto_retried:
                status["state"] = "starting"
                status["detail"] = f"Nav2 기동 {what} — 자동으로 한 번 다시 띄웁니다"
            else:
                status["state"] = "error"
                status["detail"] = (f"Nav2 기동 {what}(자동 재시도 뒤에도) — "
                                    "/vica/mode/nav2_restart 로 다시 띄웁니다")
        elif (target == "activating" or "activating" in drive_states.values()
              or snap.nav2_phase == "starting" or any(k in drive for k in young)):
            status["state"] = "starting"
        elif snap.nav2_phase == "waiting_pose":
            status["state"] = "need_initial_pose"
            status["detail"] = "초기 위치를 잡아 주세요"
        else:
            status["state"] = "running"
        return status

    if snap.mapping_state not in MAPPING_IDLE:
        status["mode"] = "mapping"
        status["state"] = snap.mapping_state
    elif status["stop_timeouts"]:
        status["detail"] = ("지난 주행 정지에서 시간 초과로 강제 종료: " + ", ".join(status["stop_timeouts"])
                            + " — 되풀이되면 재부팅이 필요합니다")
    return status


def drive_start_refusal(snap: Snapshot, manifest: dict) -> str | None:
    """주행 묶음을 올리면 안 되는 이유. 없으면 None. 이미 떠 있으면 None(같은 요청을 두 번 받아도 된다)."""
    if drive_on(snap, manifest):
        return None
    if mic_check_active(snap):
        return "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요."
    if snap.mapping_state not in MAPPING_IDLE:
        return f"지도 그리기 중입니다({snap.mapping_state}). 지도 그리기를 끝낸 뒤 주행을 시작합니다."
    for name in ("cartographer_no", "cartographer_oc"):
        if snap.procs.get(name, 0):
            return f"지도 그리기 노드({name})가 떠 있습니다."
    for entry in _bundle(manifest, "drive").values():
        running = [n for n in entry.get("guard", ()) if snap.procs.get(n, 0)]
        if running and _state(snap, entry["unit"]) != "active":
            return "칸에서 띄운 주행 노드가 있습니다: " + ", ".join(running)
    down = sorted(k for k, e in _bundle(manifest, "base").items() if _state(snap, e["unit"]) != "active")
    if down:
        return "기본 묶음이 다 떠 있지 않습니다: " + ", ".join(down)
    if not snap.can_up:
        return "can1 이 내려가 있습니다."
    dup = _duplicated(snap, manifest)
    if dup:
        return "두 벌 떠 있는 노드가 있습니다: " + ", ".join(
            "rosbridge" if "rosbridge" in manifest.get(d, {}).get("guard_cmdline", "") else d for d in dup
        )
    return None


def shutdown_refusal(snap: Snapshot) -> str | None:
    """로봇 끄기(주행·기본 묶음 → sync → 시스템 종료)를 하면 안 되는 이유. 없으면 None."""
    if snap.robot_moving:
        return "주행 중입니다. 멈춘 뒤 끕니다."
    if snap.mapping_state not in MAPPING_IDLE:
        return f"지도 그리기 중입니다({snap.mapping_state}). 저장하거나 끝낸 뒤 끕니다."
    return None


def drive_stop_refusal(snap: Snapshot) -> str | None:
    """주행 묶음을 내리면 안 되는 이유. 움직임을 모르면(None) 막지 않는다 — Nav2 를 내리면 멈춘다."""
    if snap.robot_moving:
        return "주행 중입니다. 정지한 뒤 내립니다."
    return None


def should_auto_retry(snap: Snapshot) -> bool:
    """Nav2 기동이 중단되거나 멈췄으면 한 번만 자동으로 다시 띄운다(사용자 11-16)."""
    return (
        _state(snap, DRIVE_TARGET) == "active"
        and _state(snap, NAV2_UNIT) == "active"
        and snap.nav2_phase in ("aborted", "stalled")
        and not snap.auto_retried
        and not snap.robot_moving
        and snap.stop_pending_sec is None
    )


def stop_should_wait(snap: Snapshot, manifest: dict) -> bool:
    """정지 요청을 미뤄야 하는가 — 주행 묶음이 켜졌거나 뜨는 중이고 Nav2 가 아직 '초기 위치 대기' 전."""
    return drive_on(snap, manifest) and snap.nav2_phase == "starting"


def deferred_stop_due(snap: Snapshot, manifest: dict) -> bool:
    """미뤄 둔 정지를 지금 실행할 때인가."""
    if snap.stop_pending_sec is None:
        return False
    return not stop_should_wait(snap, manifest) or snap.stop_pending_sec >= STOP_DEFER_MAX_SEC


VOICE_MODES = ("online", "offline")


def read_voice_mode(text: str) -> str:
    """voice_mode 파일 내용. 없거나 모르는 값이면 offline(⑬, 인터넷 없이 일정한 쪽)."""
    value = text.strip()
    return value if value in VOICE_MODES else "offline"


def detector_refusal(snap: Snapshot, manifest: dict, on: bool = False) -> str | None:
    """감지기 켜기·끄기를 거절할 이유. 주행 묶음 밖에서는 다루지 않는다. 점검 중에는 켜지 못한다(끄기는 허용)."""
    if "detector" not in manifest:
        return "감지기 유닛이 없습니다(--units --stage 2c)."
    if not drive_on(snap, manifest):
        return "주행 모드가 아닙니다."
    if on and mic_check_active(snap):
        return "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요."
    return None


def voice_switch_refusal(snap: Snapshot) -> str | None:
    """음성을 바꾸면 음성 유닛을 다시 띄워 말이 끊긴다. 움직이는 중(안내 중 대화)에는 하지 않는다."""
    if snap.robot_moving:
        return "주행 중입니다. 정지한 뒤 바꿉니다."
    return None


def twist_moving(linear_x: float, angular_z: float) -> bool:
    """/cmd_vel_safe 한 개가 0 이 아닌가(문턱 1e-3)."""
    return abs(linear_x) > 1e-3 or abs(angular_z) > 1e-3


def robot_in_motion(snap: Snapshot) -> bool:
    """미션이 움직인다고 하거나, 최근 /cmd_vel_safe 가 0 이 아니었다."""
    return bool(snap.robot_moving) or snap.cmd_moving


def mic_check_active(snap: Snapshot) -> bool:
    """점검 유닛이 뜨는 중·떠 있는 중·내려가는 중인가(내려가는 중에 칩을 되돌린다)."""
    return _state(snap, MIC_CHECK_UNIT) in ("active", "activating", "deactivating")


def mic_check_start_refusal(snap: Snapshot, manifest: dict) -> str | None:
    """마이크 점검을 시작하면 안 되는 이유. 없으면 None(주행 꺼짐·음성 꺼짐인 모드 선택 화면도 허용)."""
    if mic_check_active(snap):
        return "이미 점검 중입니다."
    if snap.mapping_state not in MAPPING_IDLE or any(
            snap.procs.get(name, 0) for name in ("cartographer_no", "cartographer_oc")):
        return "지도를 그리는 중입니다."
    if robot_in_motion(snap):
        return MOVING_REFUSAL
    llm = manifest.get("llm")
    if llm:
        if _state(snap, llm["unit"]) in ("active", "activating", "deactivating"):
            return "음성 노드가 켜져 있습니다. 먼저 끄세요."
        pattern = llm.get("guard_cmdline")
        if (any(snap.procs.get(n, 0) for n in llm.get("guard", ()))
                or (pattern and snap.cmdline_hits.get(pattern, 0) > 0)):
            return "터미널에서 띄운 음성이 있습니다. 그 칸을 끈 뒤 다시 누르세요."
    return None


def mic_check_stop_refusal(snap: Snapshot) -> str | None:
    """점검을 멈추라는 요청을 거절할 이유."""
    if not mic_check_active(snap):
        return "점검 중이 아닙니다."
    return None


def mic_check_side_stops(snap: Snapshot, manifest: dict) -> list[str]:
    """점검을 시작하기 전에 같이 내릴 유닛. 말없이 다가가는 것을 막으려고 주행 중 감지기를 내린다(사용자 11-19 ①)."""
    det = manifest.get("detector")
    if det and drive_on(snap, manifest) and _state(snap, det["unit"]) in ("active", "activating"):
        return [det["unit"]]
    return []


def mic_check_command(voice_root: str, map_id: str, destinations: str | None) -> list[str]:
    """점검기를 일회성 유닛으로 띄우는 명령. --collect 로 실패한 같은 이름 유닛이 다음 시작을 막지 않게 한다."""
    cmd = ["systemd-run", "--user", "--collect", f"--unit={MIC_CHECK_UNIT}",
           f"--setenv={MIC_CHECK_LAUNCHER_ENV}=mode", "--setenv=PYTHONUNBUFFERED=1",
           f"--working-directory={voice_root}", f"{voice_root}/.venv/bin/python", "-m",
           "tools.mic_venue_check", "--name", map_id or "점검"]
    if destinations:
        cmd += ["--destinations", destinations]
    return cmd


def voice_node_refusal(snap: Snapshot, manifest: dict, on: bool) -> str | None:
    """음성 노드(vica-llm)만 켜거나 끌 때 거절할 이유."""
    if "llm" not in manifest:
        return "음성 유닛이 없습니다."
    if not drive_on(snap, manifest):
        return "주행 모드가 아닙니다."
    if robot_in_motion(snap):
        return MOVING_REFUSAL
    if on and mic_check_active(snap):
        return "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요."
    age = snap.unit_age.get(manifest["llm"]["unit"])
    if not on and age is not None and age < VOICE_OFF_GRACE_SEC:
        return f"음성 노드가 켜지는 중입니다. {math.ceil(VOICE_OFF_GRACE_SEC - age)}초 뒤 다시 누르세요."
    return None


class VoiceEventTracker:
    """음성 유닛 저널 줄에서 연결 알림을 가린다(사용자 11-16: 외부 API·로컬 API 연결 성공/실패)."""

    TEXTS = {
        "external_ok": "외부 API 연결 성공",
        "external_fail": "외부 API 연결 실패",
        "external_fallback": "외부 API 연결 실패 → 로컬 API 로 전환",
        "external_back": "외부 API 연결 복귀",
        "external_auth": "외부 API 인증 실패 — 키를 확인하세요",
        "realtime_fail": "외부 API(Realtime) 연결 실패",
        "local_ok": "로컬 API 연결 성공",
        "local_fail": "로컬 API 연결 실패",
        "both_fail": "외부 API·로컬 API 모두 연결 실패",
    }

    def __init__(self) -> None:
        self.mode = "offline"
        self.failed = False

    def _event(self, kind: str, detail: str = "") -> list[tuple[str, str]]:
        text = self.TEXTS[kind]
        if detail:
            text += f" ({detail.strip()[:80]})"
        return [(kind, text)]

    def feed(self, raw: str) -> list[tuple[str, str]]:
        line = _ANSI.sub("", raw)
        if "음성: online" in line or "음성: offline" in line:
            self.mode = "online" if "음성: online" in line else "offline"
            self.failed = False
            return []
        online = self.mode == "online"
        if "[LLM] 클라우드 실패(" in line:
            self.failed = True
            return self._event("external_fallback")
        if "[LLM] 호출 실패(로컬까지)" in line:
            self.failed = True
            return self._event("both_fail", line.split("):", 1)[-1] if "):" in line else "")
        if "[LLM] 호출 실패(클라우드만" in line or "LLM 워밍업 실패" in line:
            self.failed = True
            detail = line.split(":", 1)[-1] if "LLM 워밍업 실패" in line else line.split("):", 1)[-1]
            return self._event("external_fail" if online else "local_fail", detail)
        if "[Ollama] " in line and "올리기 실패" in line:
            self.failed = True
            return self._event("local_fail", "모델 올리기 실패")
        if "[LLM] 로컬 백엔드 준비 실패" in line:
            return self._event("local_fail", "준비 실패")
        if "Realtime 연결 예열 실패" in line:
            return self._event("realtime_fail")
        if "클라우드 건너뛰고 로컬" in line:
            return self._event("external_fallback", "Realtime 연결 오류")
        if "클라우드는 닿지만 인증 실패" in line:
            return self._event("external_auth")
        if "→ 클라우드 복귀" in line:
            return self._event("external_back")
        if "LLM 워밍업 완료" in line and not self.failed:
            return self._event("external_ok" if online else "local_ok")
        return []


def nav2_restart_refusal(snap: Snapshot) -> str | None:
    if _state(snap, DRIVE_TARGET) != "active":
        return "주행 모드가 아닙니다."
    if snap.robot_moving:
        return "주행 중입니다."
    return None


MAP_TRACKED = ("nav2", "mission", "llm")
MAP_RESTART = ("nav2", "mission", "llm", "posecheck")
MAP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_MAP_MARK = re.compile(r"\[vica-map\] (\S+)")
MAP_CHANGE_OK = ("idle", "arrived", "failed", "paused", "paused_handle", "patrol_paused")
_DIALOG_LABEL = {
    "navigating": "안내 중", "confirming": "목적지 확인 중", "estopped": "비상정지 중",
    "grip_wait": "손잡이 잡기 대기 중", "seeking": "부른 사람 찾는 중",
    "approaching": "사람 접근 중", "awaiting_user": "사람 접근 중", "turning": "사람 접근 중",
    "returning": "사람 접근 중",
    "asking_next": "도착 뒤 안내 중", "asking_wait_time": "도착 뒤 안내 중", "waiting": "도착 뒤 기다리는 중",
    "waiting_release": "도착 뒤 기다리는 중", "moving_to_wait_spot": "대기 장소로 가는 중",
    "moving_back_to_dest": "목적지로 돌아가는 중",
    "patrolling": "순찰 중", "patrol_dwell": "순찰 중", "patrol_alarm": "순찰 경보 중",
    "patrol_light": "순찰 경보 중",
}


def parse_map_mark(lines: list[str]) -> str:
    """실행 스크립트가 기동 때 찍는 '[vica-map] <id>' 줄의 마지막 값. 없으면 ""."""
    found = ""
    for raw in lines:
        match = _MAP_MARK.search(_ANSI.sub("", raw))
        if match:
            found = match.group(1)
    return found


def map_change_block(snap: Snapshot, manifest: dict, state: str) -> str | None:
    """지금 로봇 상태 때문에 주행 지도를 바꿀 수 없는 이유. 없으면 None. state 는 derive_status 의 state."""
    if mic_check_active(snap):
        return "마이크 점검 중입니다. 점검을 끝낸 뒤 바꾸세요."
    if snap.mapping_state not in MAPPING_IDLE:
        return "지도 그리는 중엔 불가"
    if any(snap.procs.get(name, 0) for name in ("cartographer_no", "cartographer_oc")):
        return "지도 그리기 노드가 떠 있음"
    if not drive_on(snap, manifest) and state != "stopping":
        return None
    if (state in ("starting", "stopping") or snap.stop_pending_sec is not None
            or snap.nav2_phase == "starting"):
        return "주행을 켜거나 끄는 중엔 불가"
    if snap.robot_moving is None or snap.dialog_state is None:
        return "로봇 상태를 받지 못함 · 주행을 끈 뒤 바꾸세요"
    if snap.robot_moving:
        return "로봇이 움직이는 중엔 불가"
    if snap.dialog_state not in MAP_CHANGE_OK:
        label = _DIALOG_LABEL.get(snap.dialog_state, f"지금 상태({snap.dialog_state})")
        return f"{label}엔 불가 · 끝나거나 일시정지하면 가능"
    return None


def map_change_refusal(snap: Snapshot, manifest: dict, state: str, map_id: str,
                       map_problem: str | None) -> str | None:
    """이 지도로 바꾸는 요청을 거절할 이유. map_problem 은 노드가 파일을 보고 넘긴다(없으면 None)."""
    if not MAP_ID_RE.match(map_id or ""):
        return "지도 이름이 올바르지 않음"
    if map_id.endswith(("_keepout", "_route")):
        return "금지구역·레일 파일은 주행 지도가 아님"
    if map_problem:
        return map_problem
    return map_change_block(snap, manifest, state)


def map_restart_units(snap: Snapshot, manifest: dict, map_id: str) -> list[str]:
    """지도를 바꾼 뒤 다시 띄울 유닛. 주행이 꺼져 있거나, 떠 있는 유닛이 이미 그 지도면 없다."""
    if not drive_on(snap, manifest):
        return []
    live = ("active", "activating", "failed")
    tracked = [k for k in MAP_TRACKED if k in manifest and _state(snap, manifest[k]["unit"]) in live]
    if tracked and all(snap.running_maps.get(k) == map_id for k in tracked):
        return []
    return [manifest[k]["unit"] for k in MAP_RESTART
            if k in manifest and _state(snap, manifest[k]["unit"]) in live]


def map_fields(snap: Snapshot, manifest: dict, state: str) -> dict:
    """/vica/mode_status 에 싣는 지도 칸. 어긋남 = 주행 중 유닛이 읽은 지도가 CURRENT_MAP 과 다름."""
    running = {k: snap.running_maps[k] for k in MAP_TRACKED
               if k in manifest and _state(snap, manifest[k]["unit"]) == "active"
               and snap.running_maps.get(k)}
    mismatch = bool(snap.current_map) and any(v != snap.current_map for v in running.values())
    return {
        "current_map_id": snap.current_map,
        "running_maps": running,
        "map_mismatch": mismatch,
        "map_change_block": map_change_block(snap, manifest, state) or "",
        "dialog_state": snap.dialog_state or "",
    }


BRIDGE_APP_UNIT = "vica-app.service"
BRIDGE_SILENT_SEC = 15.0
BRIDGE_MIN_OWN_PUBS = 4
BRIDGE_GRACE_SEC = 60.0
BRIDGE_COOLDOWN_SEC = 300.0


def bridge_message_is_ours(raw) -> bool:
    """rosbridge 가 보낸 한 줄이 /vica/mode_status 발행인가."""
    try:
        msg = json.loads(raw)
    except (TypeError, ValueError):
        return False
    return isinstance(msg, dict) and msg.get("op") == "publish" and msg.get("topic") == "/vica/mode_status"


class BridgeWatch:
    """rosbridge 가 데이터를 넘기는지 판정한다. 시각은 time.monotonic() 초. 스레드 안전하지 않다 — 호출 쪽이 잠근다."""

    def __init__(self) -> None:
        self.connected_at: float | None = None
        self.last_rx: float | None = None
        self.own_pubs: collections.deque = collections.deque(maxlen=64)
        self.restarted_at: float | None = None
        self.hold_reason = ""

    def connected(self, now: float) -> None:
        self.connected_at, self.last_rx = now, None

    def disconnected(self) -> None:
        self.connected_at, self.last_rx = None, None

    def received(self, now: float) -> None:
        self.last_rx = now

    def published(self, now: float) -> None:
        self.own_pubs.append(now)

    def restarted(self, now: float) -> None:
        self.restarted_at = now
        self.disconnected()

    def silent(self, now: float) -> bool:
        if self.connected_at is None or now - self.connected_at < BRIDGE_SILENT_SEC:
            return False
        if self.last_rx is not None and now - self.last_rx < BRIDGE_SILENT_SEC:
            return False
        since = max(self.connected_at, self.last_rx or self.connected_at)
        return sum(1 for t in self.own_pubs if t >= since) >= BRIDGE_MIN_OWN_PUBS

    def verdict(self, now: float, moving: bool) -> str:
        if self.restarted_at is not None and now - self.restarted_at < BRIDGE_GRACE_SEC:
            return "wait"
        if not self.silent(now):
            return "ok" if self.last_rx is not None else "wait"
        if moving:
            self.hold_reason = "로봇이 움직이는 중"
            return "hold"
        if self.restarted_at is not None and now - self.restarted_at < BRIDGE_COOLDOWN_SEC:
            self.hold_reason = f"직전 재시작 뒤 {BRIDGE_COOLDOWN_SEC:.0f} s 안"
            return "hold"
        self.hold_reason = ""
        return "restart"
