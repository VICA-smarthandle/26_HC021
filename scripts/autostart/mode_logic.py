"""모드 노드(vica_mode_node.py)의 판정 로직. ROS·systemd 를 부르지 않는다.

모드 노드는 상태를 저장하지 않는다. 매 초 사실을 모아 Snapshot 을 만들고, 여기 함수로
상태와 거부 사유를 유도한다. 노드가 다시 떠도 systemd·프로세스 표에서 다시 읽으므로 꼬이지 않는다.

    사실                       어디서
    유닛 상태                  systemctl --user show
    노드 프로세스 수           /proc/*/comm · cmdline (생성기가 쓴 units.json 의 guard)
    Nav2 기동 단계             저널(이번 vica-nav2 호출)의 lifecycle_manager_navigation·localization 줄
    safety 상태                /safety_state
    지도 그리기 상태           /vica/mapping_status
    움직이는 중                /vica/robot_state.is_moving
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

DRIVE_TARGET = "vica-drive.target"
NAV2_UNIT = "vica-nav2.service"
BASE_TARGET = "vica-base.target"
MAPPING_IDLE = ("idle", "", None)
# 유닛이 active 가 된 뒤 노드가 다 뜰 때까지의 유예. 2026-11-14 실기에서 미션·감시가 18 s 걸렸다.
# 그 사이 노드가 없다고 error 로 보이지 않게 한다.
STARTUP_GRACE_SEC = 45.0
# Nav2 유닛이 active 가 된 뒤 이만큼 지나도 '초기 위치' 줄·중단·active 가 하나도 없으면 멈춘 기동으로
# 본다(사용자 11-16, 90 s). 정상 기동 40회(11-14·16)는 위치추정이 8~20 s 에 '초기 위치' 줄을 냈다.
# 11-16 10:09 회차는 map_server 응답 유실('failed to send response to /map_server/change_state')
# 뒤 관리자가 실패 줄 없이 189 s 멈췄다 — Humble lifecycle_manager 는 응답을 시한 없이 기다린다.
NAV2_STALL_SEC = 90.0
# 기동 중 정지는 Nav2 가 '초기 위치 대기'에 이를 때까지 미룬다(사용자 11-16). 재부팅 직후 33회 시험에서
# 켜고 20 s 에 내리면 정지 4.9~23.8 s·시간 초과 1·강제 종료 회차 4/20, 다 뜬 뒤 내리면 3.7~12.6 s·
# 시간 초과 0·2/13. 정상 기동은 15~20 s 에 '초기 위치 대기'에 이른다. 45 s 가 지나면 그냥 내린다.
STOP_DEFER_MAX_SEC = 45.0
# 이보다 오래 내려가는 중이면 상태에 드러낸다. 11-16 앞 부팅에서 정지가 61~72 s 였다(재부팅으로 풀림).
STOP_SLOW_SEC = 30.0

_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_NAV = "[lifecycle_manager_navigation]:"
_LOC = "[lifecycle_manager_localization]:"
_FAILED = "Failed to bring up all requested nodes"


@dataclass
class Snapshot:
    # 유닛 이름 → (ActiveState, SubState, Result)
    units: dict[str, tuple[str, str, str]]
    # guard 이름(comm 15자) → 프로세스 수
    procs: dict[str, int]
    # 명령줄 패턴 → 프로세스 수
    cmdline_hits: dict[str, int] = field(default_factory=dict)
    # starting · waiting_pose · active · aborted · stalled · down
    nav2_phase: str = "down"
    # /safety_state 값. 최근에 못 받았으면 None
    safety_state: str | None = None
    # /vica/mapping_status 의 state. 못 받았으면 None
    mapping_state: str | None = None
    # /vica/robot_state.is_moving. 미션이 없거나 오래됐으면 None
    robot_moving: bool | None = None
    can_up: bool = False
    # 유닛 이름 → active 가 된 뒤 지난 초. 모르면 없다(유예 없이 판정).
    unit_age: dict[str, float] = field(default_factory=dict)
    # 이번 주행 묶음에서 Nav2 를 이미 한 번 자동으로 다시 띄웠는가.
    auto_retried: bool = False
    # 미뤄 둔 정지 요청이 있으면 그 뒤 지난 초. 없으면 None.
    stop_pending_sec: float | None = None
    # 주행 유닛 가운데 가장 오래 내려가는 중인 것의 초. 내려가는 중이 없으면 None.
    stopping_sec: float | None = None
    # 앱에서 고른 음성(voice_mode 파일). online(⑫) · offline(⑬ 오프라인 믿음)
    voice_mode: str = "offline"


def parse_show(text: str) -> tuple[dict, dict[str, str], dict[str, int]]:
    """`systemctl --user show -p Id,ActiveState,SubState,Result,InvocationID,
    ActiveEnterTimestampMonotonic <유닛…>` 출력. 마지막 값은 μs(CLOCK_MONOTONIC)."""
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
    """이번 vica-nav2 호출의 저널 줄에서 Nav2 기동 단계를 고른다.

    active 는 lifecycle_manager_navigation 줄로만 판정한다(localization 관리자는 초기 위치
    없이도 active 다). 중단은 navigation·localization 두 관리자를 본다 — 11-16 실기에서
    map_server 응답 유실로 localization 이 먼저 중단됐는데 navigation 만 봐서 27 s 늦었다.
    지도·위치가 없으면 못 움직이므로 중단이 active 보다 우선이다.
    collision_monitor 관리자는 launch 의 12초 재시도 때 늘 'Failed to bring up' 을 남겨 뺀다.
    age_sec 는 Nav2 유닛이 active 가 된 뒤 지난 초. 모르면 멈춤 판정을 하지 않는다.
    """
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


def _state(snap: Snapshot, unit: str) -> str:
    return snap.units.get(unit, ("inactive", "dead", ""))[0]


def needs_cmdline(comm: str) -> bool:
    """명령줄까지 읽어야 하는 프로세스인가.

    명령줄 검사(guard_cmdline)는 `#!/usr/bin/env python3` 로 떠서 comm 이 python3 인
    프로세스를 찾는 데만 쓴다. 나머지까지 매 초 명령줄을 읽으면 노드가 CPU 를 14~19 % 썼다.
    'ros2' 는 ros2 launch 앞단(유닛마다 하나) — 음성 유닛은 이것으로만 알아본다(2026-11-16).
    """
    return comm in ("python3", "python", "ros2")


def _alive(snap: Snapshot, entry: dict) -> bool:
    if entry.get("guard_cmdline"):
        return snap.cmdline_hits.get(entry["guard_cmdline"], 0) > 0
    guard = entry.get("guard", ())
    if not guard:
        return True     # 확인할 이름이 없다(컨테이너 유닛). 유닛 상태만 믿는다.
    return any(snap.procs.get(name, 0) > 0 for name in guard)


def _duplicated(snap: Snapshot, manifest: dict) -> list[str]:
    dup = sorted(name for name, count in snap.procs.items() if count > 1)
    for key, entry in manifest.items():
        pattern = entry.get("guard_cmdline")
        if pattern and snap.cmdline_hits.get(pattern, 0) > 1:
            dup.append(key)
    return dup


def _bundle(manifest: dict, bundle: str) -> dict:
    return {k: v for k, v in manifest.items() if v["bundle"] == bundle}


def drive_on(snap: Snapshot, manifest: dict) -> bool:
    """주행 묶음이 켜져 있거나 뜨는 중인가.

    target 은 소속 유닛이 다 뜰 때까지 inactive 로 남는다(systemd 가 target 의 Wants 에 After 를 붙인다).
    11-16 16:10 실기: 켜고 3 s 뒤 정지 요청에 target 만 보고 '이미 꺼져 있습니다'라고 답했다.
    """
    if _state(snap, DRIVE_TARGET) in ("active", "activating"):
        return True
    return any(_state(snap, e["unit"]) in ("active", "activating") for e in _bundle(manifest, "drive").values())


def derive_status(snap: Snapshot, manifest: dict) -> dict:
    """/vica/mode_status 에 실을 내용."""
    drive = _bundle(manifest, "drive")
    target = _state(snap, DRIVE_TARGET)

    failed = sorted(k for k, e in manifest.items() if _state(snap, e["unit"]) == "failed")
    absent = [k for k, e in manifest.items() if _state(snap, e["unit"]) == "active" and not _alive(snap, e)]
    # 막 뜬 유닛의 노드는 아직 올라오는 중일 수 있다.
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
        "can_up": snap.can_up,
        # 지난 정지에서 시한을 넘겨 강제 종료된 주행 유닛(systemd Result=timeout). 다시 켜면 지워진다.
        "stop_timeouts": sorted(k for k, e in drive.items()
                                if snap.units.get(e["unit"], ("", "", ""))[2] == "timeout"),
        # 감지기·음성 유닛 상태(active·inactive·activating·deactivating·failed)와 고른 음성.
        # 감지기는 앱에서 잠시 끌 수 있어 inactive 여도 오류가 아니다(사용자 11-16).
        "detector": _state(snap, manifest["detector"]["unit"]) if "detector" in manifest else "none",
        "voice": _state(snap, manifest["llm"]["unit"]) if "llm" in manifest else "none",
        "voice_mode": snap.voice_mode,
    }

    drive_states = {k: _state(snap, e["unit"]) for k, e in drive.items()}
    going_down = [k for k, st in drive_states.items() if st == "deactivating"]
    if drive_on(snap, manifest) or target == "deactivating" or going_down:
        status["mode"] = "drive"
        drive_failed = [k for k in failed if k in drive]
        drive_missing = [k for k in missing if k in drive]
        # 묶음 정지 때는 target 이 먼저 inactive 가 되고 멤버가 늦게 내려간다. target 이 active 인데
        # 멤버만 내려가는 중이면 그 유닛을 다시 띄우는 중(자동 재시도·nav2_restart)이다.
        if snap.stop_pending_sec is not None and drive_on(snap, manifest):
            status["state"] = "stopping"
            # 남은 초는 따로 싣는다 — detail 에 넣으면 매 주기 문구가 바뀌어 로그가 2 s 마다 쌓인다(11-16 실기).
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


def drive_stop_refusal(snap: Snapshot) -> str | None:
    """주행 묶음을 내리면 안 되는 이유. 움직임을 모르면(None) 막지 않는다 — Nav2 를 내리면 멈춘다."""
    if snap.robot_moving:
        return "주행 중입니다. 정지한 뒤 내립니다."
    return None


def should_auto_retry(snap: Snapshot) -> bool:
    """Nav2 기동이 중단되거나 멈췄으면 한 번만 자동으로 다시 띄운다(사용자 11-16).

    bond 4 s 실패 같은 기동 중단은 원래 방식에서도 14번 중 1번 났다. 다시 띄우면 대개 된다.
    두 번째도 안 되면 사람에게 넘긴다(무한 반복 금지). 움직이는 중에는 하지 않는다.
    Nav2 유닛 자체가 active 일 때만 — 개발하려고 Nav2 만 손으로 내리는 중에도 '중단' 줄이 남는데,
    그때 다시 띄우면 칸에서 띄운 Nav2 와 두 벌이 된다.
    """
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


def detector_refusal(snap: Snapshot, manifest: dict) -> str | None:
    """감지기 켜기·끄기를 거절할 이유. 주행 묶음 밖에서는 다루지 않는다."""
    if "detector" not in manifest:
        return "감지기 유닛이 없습니다(--units --stage 2c)."
    if not drive_on(snap, manifest):
        return "주행 모드가 아닙니다."
    return None


def voice_switch_refusal(snap: Snapshot) -> str | None:
    """음성을 바꾸면 음성 유닛을 다시 띄워 말이 끊긴다. 움직이는 중(안내 중 대화)에는 하지 않는다."""
    if snap.robot_moving:
        return "주행 중입니다. 정지한 뒤 바꿉니다."
    return None


def nav2_restart_refusal(snap: Snapshot) -> str | None:
    if _state(snap, DRIVE_TARGET) != "active":
        return "주행 모드가 아닙니다."
    if snap.robot_moving:
        return "주행 중입니다."
    return None
