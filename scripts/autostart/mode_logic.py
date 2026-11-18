"""모드 노드(vica_mode_node.py)의 판정 로직. ROS·systemd 를 부르지 않는다.

모드 노드는 상태를 저장하지 않는다. 매 초 사실을 모아 Snapshot 을 만들고, 여기 함수로
상태와 거부 사유를 유도한다. 노드가 다시 떠도 systemd·프로세스 표에서 다시 읽으므로 꼬이지 않는다.

    사실                       어디서
    유닛 상태                  systemctl --user show
    노드 프로세스 수           /proc/*/comm · cmdline (생성기가 쓴 units.json 의 guard)
    Nav2 기동 단계             저널(이번 vica-nav2 호출)의 lifecycle_manager_navigation·localization 줄
    safety 상태                /safety_state
    지도 그리기 상태           /vica/mapping_status
    움직이는 중·미션 상태      /vica/robot_state.is_moving · dialog_state
    주행 지도                  maps/CURRENT_MAP · 유닛 저널의 '[vica-map] <id>' 줄
"""
from __future__ import annotations

import collections
import json
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
    # /vica/robot_state.dialog_state(미션 상태). 미션이 없거나 오래됐으면 None
    dialog_state: str | None = None
    # maps/CURRENT_MAP 의 지도 id(다음에 켜는 유닛이 읽는 값). 없으면 ""
    current_map: str = ""
    # 지도를 쥐는 유닛(MAP_TRACKED) → 이번 기동이 읽은 지도 id(실행 스크립트의 '[vica-map]' 줄). 모르면 없다.
    running_maps: dict[str, str] = field(default_factory=dict)


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


def settle_nav2_phase(previous: str | None, lines: list[str] | None, age_sec: float | None) -> str:
    """같은 Nav2 호출에서 저널을 다시 읽은 결과로 단계를 정한다.

    `lines` 가 None 이면 저널을 못 읽은 것(시한 초과·오류)이다. 직전 판정을 그대로 두고, 없으면 starting.
    이미 waiting_pose·active 를 본 호출은 멈춤(stalled)으로 내려가지 않는다 — 저널이 지워지거나
    일부만 읽혀 '초기 위치' 줄이 안 보여도 진행은 이미 확인했다. 기동 실패(aborted)는 그대로 우선한다.

    11-17 21:33 사고: 주행 중 flutter 빌드로 저널이 약 10 s 멈춰 조회(5 s 시한)가 실패했고, 빈 목록을
    판정해 23분째 초기 위치를 기다리던 Nav2 를 '멈춤'으로 보고 자동 재시도로 껐다 켰다.
    """
    if lines is None:
        return previous or "starting"
    phase = nav2_phase(lines, age_sec)
    if phase == "stalled" and previous in ("waiting_pose", "active"):
        return previous
    return phase


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
    status = _derive_core(snap, manifest)
    status.update(map_fields(snap, manifest, status["state"]))
    return status


def _derive_core(snap: Snapshot, manifest: dict) -> dict:
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


class VoiceEventTracker:
    """음성 유닛 저널 줄에서 연결 알림을 가린다(사용자 11-16: 외부 API·로컬 API 연결 성공/실패).

    음성 코드는 고치지 않는다. 기댄 문구(vica-voice-llm, 2026-11-16):
      run_llm.sh                   '음성: online|offline'   — 유닛이 기동 때 찍는다(어느 모드인지)
      ros_node.py                  'LLM 워밍업 완료' · 'LLM 워밍업 실패' · 'Realtime 연결 예열 실패'
      langchain_intent_parser.py   '[LLM] 호출 실패(클라우드만, 폴백 꺼짐)' · '(로컬까지)' · '로컬 백엔드 준비 실패'
      llm_backend.py               '클라우드 실패(… → 로컬(…)로 대피' · '클라우드 건너뛰고 로컬' ·
                                   '인증 실패' · '→ 클라우드 복귀'
      vica_ollama_preload.sh       '[Ollama] <모델> 올리기 실패' (유닛 실행 스크립트, 11-17)
    성공은 따로 찍히지 않는다. 기동 뒤 실패 줄 없이 '워밍업 완료' 가 오면 성공으로 본다 — 워밍업 호출이
    실패해도 '완료' 는 찍힌다(11-16 17:36 로컬 기동: timed out 뒤 '워밍업 완료 (15.1초)').
    로컬 모드(⑬)는 Ollama 가 주 백엔드라 '클라우드만' 실패도 로컬 실패다.
    """

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
            # 음성 노드보다 먼저 모델을 올리다 실패(11-17). 첫 질문이 늦거나 실패할 수 있다.
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


# ---- 주행 지도 바꾸기(사용자 11-17) ------------------------------------------------
#
# 지도는 maps/CURRENT_MAP 한 곳이 정하고, 유닛은 켜질 때 한 번 읽는다(실행 스크립트 MAP_BLOCK).
# 그래서 파일을 쓰고 지도를 쥔 유닛을 다시 띄운다. 11-17 확인:
#   nav2     map:= · 같은 이름의 레일(_route.geojson)·금지구역(_keepout.yaml)
#   mission  map_id:= · ~/vica_data/destinations/<id>/ 목적지·홈·순찰 경로
#   llm      vica_voice.launch.py map_id:= → vica_llm·tts·wakeword 의 destinations.yaml 경로
#   posecheck /map 을 다시 받지만 크기·원점이 같으면 옛 거리표를 쓴다(_on_map 의 key) — 같이 다시 띄운다
# 손잡이·감지기·감시·safety 는 지도를 쓰지 않는다. 런타임 교체(map_server LoadMap)는 미션·음성·레일이
# 따로 쥔 지도가 어긋나서 쓰지 않는다.
MAP_TRACKED = ("nav2", "mission", "llm")
MAP_RESTART = ("nav2", "mission", "llm", "posecheck")
MAP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
_MAP_MARK = re.compile(r"\[vica-map\] (\S+)")
# 로봇이 멈춰 있고 주행이 끝나 대기 중이거나 일시정지 중일 때만 바꾼다(사용자 11-17).
# 일시정지한 안내·순찰은 미션을 다시 띄우면 취소된다(목적지·순찰 경로가 지도마다 따로다).
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
    if snap.mapping_state not in MAPPING_IDLE:
        return "지도 그리는 중엔 불가"
    if any(snap.procs.get(name, 0) for name in ("cartographer_no", "cartographer_oc")):
        return "지도 그리기 노드가 떠 있음"
    if not drive_on(snap, manifest) and state != "stopping":
        return None    # 주행이 꺼져 있으면 파일만 바꾼다. 다음에 켤 때 읽는다.
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
        # maps/ 에는 금지구역 마스크(<id>_keepout.yaml·pgm)도 지도와 같은 꼴로 있다.
        return "금지구역·레일 파일은 주행 지도가 아님"
    if map_problem:
        return map_problem
    return map_change_block(snap, manifest, state)


def map_restart_units(snap: Snapshot, manifest: dict, map_id: str) -> list[str]:
    """지도를 바꾼 뒤 다시 띄울 유닛. 주행이 꺼져 있거나, 떠 있는 유닛이 이미 그 지도면 없다.

    떠 있지 않은 유닛은 켜질 때 새 파일을 읽는다. failed 인 유닛도 다시 띄운다(새 지도로 살린다).
    """
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
        # 앱이 확인 창을 고른다 — 일시정지 중이면 '남은 안내·순찰 취소'를 알린다(목업 ④-2).
        "dialog_state": snap.dialog_state or "",
    }


# ---- rosbridge 감시(11-17) ---------------------------------------------------------
# 2026-11-17 14:39 rosbridge 안의 ROS 처리 스레드(Thread-1 spin)가 rclpy InvalidHandle 로 죽었다.
# 프로세스·웹소켓은 살아 있어 systemd 는 active, 앱은 '온라인'인데 토픽·서비스가 하나도 안 왔다.
# 모드 노드가 rosbridge 에 연결 하나를 계속 붙여 두고 자기 /vica/mode_status 가 돌아오는지 본다.
# 붙었다 끊기를 되풀이하지 않는다 — 그 죽음이 바로 클라이언트가 끊길 때 구독을 지우는 경합이다.
BRIDGE_APP_UNIT = "vica-app.service"
# 붙은 채 이만큼 아무것도 안 오면 죽은 것으로 본다. mode_status 는 2초마다(TICK_SEC) — 7번 놓침.
BRIDGE_SILENT_SEC = 15.0
# 그 사이 모드 노드 자신이 이만큼은 냈어야 한다(자기가 멈춘 것을 rosbridge 탓으로 돌리지 않게).
BRIDGE_MIN_OWN_PUBS = 4
# 재시작 뒤 판정을 쉬는 시간(11-17 실측: 재시작 → 9090 열림 17 s)과 재시작 사이 최소 간격.
BRIDGE_GRACE_SEC = 60.0
BRIDGE_COOLDOWN_SEC = 300.0


def bridge_message_is_ours(raw) -> bool:
    """rosbridge 가 보낸 한 줄이 /vica/mode_status 발행인가.

    rosbridge 는 JSON 의 '/' 를 '\\/' 로 적는다("topic":"\\/vica\\/mode_status"). 원문에서 문자열을 찾으면
    못 찾는다 — 11-17 첫 배포가 그래서 멀쩡한 rosbridge 를 다시 띄웠다. 풀어서 비교한다.
    """
    try:
        msg = json.loads(raw)
    except (TypeError, ValueError):
        return False
    return isinstance(msg, dict) and msg.get("op") == "publish" and msg.get("topic") == "/vica/mode_status"


class BridgeWatch:
    """rosbridge 가 데이터를 넘기는지 판정한다. 시각은 time.monotonic() 초. 스레드 안전하지 않다 — 호출 쪽이 잠근다.

    verdict(): "ok" | "wait"(판정할 근거가 아직 없음) | "hold"(죽었지만 지금은 재시작하지 않음) | "restart"
    """

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
            # 주행 중에는 vica-app(목적지·금지구역·경로 노드 포함)을 다시 띄우지 않는다. 멈추면 띄운다.
            self.hold_reason = "로봇이 움직이는 중"
            return "hold"
        if self.restarted_at is not None and now - self.restarted_at < BRIDGE_COOLDOWN_SEC:
            self.hold_reason = f"직전 재시작 뒤 {BRIDGE_COOLDOWN_SEC:.0f} s 안"
            return "hold"
        self.hold_reason = ""
        return "restart"
