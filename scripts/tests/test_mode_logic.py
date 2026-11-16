"""모드 노드의 판정 로직(mode_logic.py). ROS·systemd 없이 돈다.

모드 노드는 상태를 저장하지 않는다. 매 초 systemd·프로세스 표·저널·토픽에서 사실을 모아
여기 함수들로 상태를 유도한다 — 노드가 다시 떠도 꼬이지 않게.
"""
import unittest

from scripts.autostart import mode_logic as ml

MANIFEST = {
    "display": {"unit": "vica-display.service", "bundle": "base",
                "guard": ["robot_state_pub", "joint_state_pub"], "guard_cmdline": ""},
    "app": {"unit": "vica-app.service", "bundle": "base", "guard": ["rosbridge_webso"],
            "guard_cmdline": "^[^ ]*python3 [^ ]*rosbridge_server/rosbridge_websocket"},
    "motor": {"unit": "vica-motor.service", "bundle": "base",
              "guard": ["mdrobot_can_ke", "keyboard_knob"], "guard_cmdline": ""},
    "nav2": {"unit": "vica-nav2.service", "bundle": "drive",
             "guard": ["bt_navigator", "amcl", "ekf_node", "encoder_feedbac", "cartographer_no"],
             "guard_cmdline": ""},
    "mission": {"unit": "vica-mission.service", "bundle": "drive",
                "guard": ["vica_mission_ma", "mission_manager"], "guard_cmdline": ""},
}

SHOW = """\
Id=vica-drive.target
ActiveState=active
SubState=active
Result=success
InvocationID=

Id=vica-nav2.service
ActiveState=active
SubState=running
Result=success
InvocationID=2a7bbeb2412d40988548348501265d46

Id=vica-mission.service
ActiveState=failed
SubState=failed
Result=exit-code
InvocationID=
"""


def snapshot(**overrides):
    """모든 것이 정상인 주행 상태. 시험마다 한두 값만 바꾼다."""
    units = {
        "vica-base.target": ("active", "active", "success"),
        "vica-drive.target": ("active", "active", "success"),
        "vica-display.service": ("active", "running", "success"),
        "vica-app.service": ("active", "running", "success"),
        "vica-motor.service": ("active", "running", "success"),
        "vica-nav2.service": ("active", "running", "success"),
        "vica-mission.service": ("active", "running", "success"),
    }
    procs = {"robot_state_pub": 1, "joint_state_pub": 1, "keyboard_knob": 1,
             "bt_navigator": 1, "amcl": 1, "ekf_node": 1, "encoder_feedbac": 1,
             "mission_manager": 1}
    base = dict(
        units=units, procs=procs,
        cmdline_hits={"^[^ ]*python3 [^ ]*rosbridge_server/rosbridge_websocket": 1},
        nav2_phase="active", safety_state="READY_TO_GO", mapping_state="idle",
        robot_moving=False, can_up=True,
    )
    for key, value in overrides.items():
        if key in ("units", "procs", "cmdline_hits"):
            merged = dict(base[key])
            merged.update(value)
            base[key] = merged
        else:
            base[key] = value
    return ml.Snapshot(**base)


def idle_snapshot(**overrides):
    stopped = {u: ("inactive", "dead", "success")
               for u in ("vica-drive.target", "vica-nav2.service", "vica-mission.service")}
    procs = {n: 0 for n in ("bt_navigator", "amcl", "ekf_node", "encoder_feedbac", "mission_manager")}
    units = dict(stopped)
    units.update(overrides.pop("units", {}))
    procs.update(overrides.pop("procs", {}))
    overrides.setdefault("nav2_phase", "down")
    return snapshot(units=units, procs=procs, **overrides)


class ParseShow(unittest.TestCase):
    def test_reads_when_each_unit_became_active(self):
        text = SHOW.replace("InvocationID=2a7bbeb2412d40988548348501265d46",
                            "InvocationID=2a7bbeb2412d40988548348501265d46\n"
                            "ActiveEnterTimestampMonotonic=1314262724")
        _, _, entered = ml.parse_show(text)
        self.assertEqual(entered["vica-nav2.service"], 1314262724)
        self.assertNotIn("vica-mission.service", entered)

    def test_reads_each_unit_block(self):
        units, invocations, _ = ml.parse_show(SHOW)
        self.assertEqual(units["vica-drive.target"], ("active", "active", "success"))
        self.assertEqual(units["vica-mission.service"], ("failed", "failed", "exit-code"))
        self.assertEqual(invocations["vica-nav2.service"], "2a7bbeb2412d40988548348501265d46")
        self.assertNotIn("vica-mission.service", invocations)


class Nav2Phase(unittest.TestCase):
    ACTIVE = ("[lifecycle_manager-19] [INFO] [1794635454.68] [lifecycle_manager_navigation]: "
              "\x1b[34m\x1b[1mManaged nodes are active\x1b[0m\x1b[0m")
    ABORT = ("[lifecycle_manager-19] [ERROR] [1794634670.49] [lifecycle_manager_navigation]: "
             "Failed to bring up all requested nodes. Aborting bringup.")
    POSE = ("[amcl-10] [WARN] [1794635446.83] [amcl]: AMCL cannot publish a pose or update the "
            "transform. Please set the initial pose...")

    def test_active_line_is_found_through_colour_codes(self):
        self.assertEqual(ml.nav2_phase([self.POSE, self.ACTIVE]), "active")

    def test_abort_is_reported(self):
        self.assertEqual(ml.nav2_phase([self.POSE, self.ABORT]), "aborted")

    def test_waiting_for_initial_pose(self):
        self.assertEqual(ml.nav2_phase([self.POSE]), "waiting_pose")

    def test_nothing_yet_is_starting(self):
        self.assertEqual(ml.nav2_phase([]), "starting")

    def test_other_lifecycle_managers_do_not_count(self):
        # collision_monitor 관리자는 12초 재시도 때 늘 'Failed to bring up' 을 남긴다.
        other = self.ABORT.replace("lifecycle_manager_navigation", "lifecycle_manager_collision_monitor")
        local = self.ACTIVE.replace("lifecycle_manager_navigation", "lifecycle_manager_localization")
        self.assertEqual(ml.nav2_phase([self.POSE, other, local]), "waiting_pose")


class DeriveStatus(unittest.TestCase):
    def status(self, snap):
        return ml.derive_status(snap, MANIFEST)

    def test_everything_up_and_active_is_running(self):
        s = self.status(snapshot())
        self.assertEqual((s["mode"], s["state"]), ("drive", "running"))
        self.assertFalse(s["reset_needed"])

    def test_nav2_waiting_for_the_initial_pose(self):
        s = self.status(snapshot(nav2_phase="waiting_pose"))
        self.assertEqual(s["state"], "need_initial_pose")

    def test_nav2_bringup_abort_is_an_error_with_a_way_out(self):
        s = self.status(snapshot(nav2_phase="aborted", auto_retried=True))
        self.assertEqual(s["state"], "error")
        self.assertIn("nav2_restart", s["detail"])

    def test_failed_drive_unit_is_an_error(self):
        s = self.status(snapshot(units={"vica-mission.service": ("failed", "failed", "exit-code")}))
        self.assertEqual(s["state"], "error")
        self.assertIn("mission", s["failed_units"])

    def test_unit_alive_but_its_node_gone_is_an_error(self):
        # ros2 launch 는 자식 노드가 죽어도 살아 있다. 유닛은 active 로 보인다.
        s = self.status(snapshot(procs={"mission_manager": 0}))
        self.assertEqual(s["state"], "error")
        self.assertIn("mission", s["missing_nodes"])

    def test_nodes_not_up_yet_right_after_start_is_starting_not_error(self):
        # 2026-11-14 실기: 켠 직후 18 s 동안 미션·감시 노드가 아직 안 떠 error 로 보였다.
        snap = snapshot(procs={"mission_manager": 0}, nav2_phase="starting",
                        unit_age={"vica-mission.service": 10.0})
        s = self.status(snap)
        self.assertEqual(s["state"], "starting")
        self.assertEqual(s["missing_nodes"], [])

    def test_node_still_missing_after_the_grace_period_is_an_error(self):
        snap = snapshot(procs={"mission_manager": 0},
                        unit_age={"vica-mission.service": ml.STARTUP_GRACE_SEC + 1})
        self.assertIn("mission", self.status(snap)["missing_nodes"])

    def test_unit_with_nothing_to_check_is_not_reported_missing(self):
        # 컨테이너 유닛처럼 확인할 프로세스 이름이 없는 유닛은 유닛 상태만 믿는다.
        manifest = dict(MANIFEST, container={"unit": "vica-container.service", "bundle": "base",
                                              "guard": [], "guard_cmdline": ""})
        snap = snapshot(units={"vica-container.service": ("active", "running", "success")})
        self.assertNotIn("container", ml.derive_status(snap, manifest)["missing_nodes"])

    def test_node_counted_by_command_line(self):
        s = self.status(snapshot(cmdline_hits={
            "^[^ ]*python3 [^ ]*rosbridge_server/rosbridge_websocket": 0}))
        self.assertIn("app", s["missing_nodes"])

    def test_two_copies_are_reported(self):
        s = self.status(snapshot(procs={"ekf_node": 2}))
        self.assertIn("ekf_node", s["duplicated"])

    def test_stopping(self):
        s = self.status(snapshot(units={"vica-drive.target": ("deactivating", "stop", "success"),
                                        "vica-nav2.service": ("deactivating", "stop-sigterm", "success")}))
        self.assertEqual((s["mode"], s["state"]), ("drive", "stopping"))

    def test_drive_bundle_down_is_idle(self):
        s = self.status(idle_snapshot())
        self.assertEqual((s["mode"], s["state"]), ("idle", "idle"))
        self.assertEqual(s["missing_nodes"], [])

    def test_mapping_in_progress(self):
        s = self.status(idle_snapshot(mapping_state="mapping"))
        self.assertEqual((s["mode"], s["state"]), ("mapping", "mapping"))

    def test_supervisor_waiting_for_reset_is_flagged(self):
        # 래치 원인 없이 감독만 reset 대기로 떨어진 경우 — 자동 복구가 붙지 않는다.
        s = self.status(snapshot(safety_state="ESTOP_RELEASED_WAIT_RESET"))
        self.assertTrue(s["reset_needed"])
        self.assertEqual(s["safety"], "ESTOP_RELEASED_WAIT_RESET")

    def test_silent_safety_is_unknown_not_ready(self):
        s = self.status(snapshot(safety_state=None))
        self.assertEqual(s["safety"], "unknown")


class StartRefusal(unittest.TestCase):
    def refusal(self, snap):
        return ml.drive_start_refusal(snap, MANIFEST)

    def test_clean_idle_is_allowed(self):
        self.assertIsNone(self.refusal(idle_snapshot()))

    def test_already_running_is_not_an_error(self):
        self.assertIsNone(self.refusal(snapshot()))

    def test_refused_while_mapping(self):
        self.assertIn("지도", self.refusal(idle_snapshot(mapping_state="mapping")))

    def test_refused_when_cartographer_runs_even_without_status(self):
        self.assertIn("cartographer_no",
                      self.refusal(idle_snapshot(mapping_state=None, procs={"cartographer_no": 1})))

    def test_refused_when_nav2_was_started_from_a_pane(self):
        self.assertIn("bt_navigator", self.refusal(idle_snapshot(procs={"bt_navigator": 1})))

    def test_refused_when_a_base_unit_is_down(self):
        snap = idle_snapshot(units={"vica-motor.service": ("failed", "failed", "exit-code")})
        self.assertIn("motor", self.refusal(snap))

    def test_refused_when_can_is_down(self):
        self.assertIn("can1", self.refusal(idle_snapshot(can_up=False)))

    def test_refused_on_duplicates(self):
        self.assertIn("rosbridge", self.refusal(idle_snapshot(cmdline_hits={
            "^[^ ]*python3 [^ ]*rosbridge_server/rosbridge_websocket": 2})))


class StopRefusal(unittest.TestCase):
    def test_refused_while_the_robot_moves(self):
        self.assertIsNotNone(ml.drive_stop_refusal(snapshot(robot_moving=True)))

    def test_allowed_when_still(self):
        self.assertIsNone(ml.drive_stop_refusal(snapshot()))

    def test_unknown_motion_is_allowed(self):
        # 미션이 죽어 있으면 움직임을 모른다. 그래도 내릴 수 있어야 한다 — Nav2 를 내리면 멈춘다.
        self.assertIsNone(ml.drive_stop_refusal(snapshot(robot_moving=None)))


class CommandLineCandidates(unittest.TestCase):
    """명령줄을 읽을 프로세스만 고른다. 매 초 전부 읽으면 노드가 CPU 를 14~19 % 쓴다(11-14 실측)."""

    def test_python_processes_are_read(self):
        self.assertTrue(ml.needs_cmdline("python3"))
        self.assertTrue(ml.needs_cmdline("python"))

    def test_other_processes_are_skipped(self):
        for comm in ("bt_navigator", "bash", "rplidar_node", "mission_manager"):
            self.assertFalse(ml.needs_cmdline(comm), comm)

    def test_every_command_line_guard_starts_with_python3(self):
        # 명령줄 검사는 comm 이 python3 라 이름으로 못 찾는 프로세스용이다. 다른 것을 넣으면
        # needs_cmdline 이 그 프로세스를 건너뛰어 '노드 없음' 으로 잘못 보인다.
        from scripts import vica_terminator_layout as g
        for key, term in g.build_terms().items():
            if term.unit and term.unit.guard_cmdline:
                self.assertTrue(term.unit.guard_cmdline.startswith("^[^ ]*python3 "), key)


class AutoRetry(unittest.TestCase):
    """Nav2 기동이 중단되면 모드 노드가 Nav2 만 한 번 자동으로 다시 띄운다(사용자 11-16)."""

    def test_retries_once_after_an_abort(self):
        self.assertTrue(ml.should_auto_retry(snapshot(nav2_phase="aborted")))

    def test_not_twice(self):
        self.assertFalse(ml.should_auto_retry(snapshot(nav2_phase="aborted", auto_retried=True)))

    def test_not_while_moving(self):
        self.assertFalse(ml.should_auto_retry(snapshot(nav2_phase="aborted", robot_moving=True)))

    def test_not_when_drive_is_down_or_nav2_is_fine(self):
        self.assertFalse(ml.should_auto_retry(idle_snapshot(nav2_phase="aborted")))
        for phase in ("starting", "waiting_pose", "active"):
            self.assertFalse(ml.should_auto_retry(snapshot(nav2_phase=phase)), phase)

    def test_status_says_it_is_retrying_then_gives_up(self):
        first = ml.derive_status(snapshot(nav2_phase="aborted"), MANIFEST)
        self.assertEqual(first["state"], "starting")
        self.assertIn("자동", first["detail"])
        second = ml.derive_status(snapshot(nav2_phase="aborted", auto_retried=True), MANIFEST)
        self.assertEqual(second["state"], "error")
        self.assertIn("nav2_restart", second["detail"])


class Nav2RestartRefusal(unittest.TestCase):
    def test_allowed_when_drive_is_up_and_still(self):
        self.assertIsNone(ml.nav2_restart_refusal(snapshot(nav2_phase="aborted")))

    def test_refused_when_drive_is_down(self):
        self.assertIsNotNone(ml.nav2_restart_refusal(idle_snapshot()))

    def test_refused_while_the_robot_moves(self):
        self.assertIsNotNone(ml.nav2_restart_refusal(snapshot(robot_moving=True)))


if __name__ == "__main__":
    unittest.main()
