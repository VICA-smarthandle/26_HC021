"""모드 노드의 판정 로직(mode_logic.py). ROS·systemd 없이 돈다."""
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
    "detector": {"unit": "vica-detector.service", "bundle": "drive",
                 "guard": ["vica_detector"], "guard_cmdline": ""},
    "llm": {"unit": "vica-llm.service", "bundle": "drive", "guard": ["vica_llm_node"],
            "guard_cmdline": "^[^ ]*python3 [^ ]*ros2 launch launch/vica_voice\\.launch\\.py"},
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


class VoiceNodeRefusalTest(unittest.TestCase):
    """음성 노드(vica-llm) 켜기·끄기 거절 규칙(11-19 설계)."""

    def _with_check(self, state):
        units = dict(snapshot().units)
        units[ml.MIC_CHECK_UNIT] = (state, "stop-sigterm", "")
        return snapshot(units=units)

    def test_idle_driving_allows(self):
        self.assertIsNone(ml.voice_node_refusal(snapshot(), MANIFEST, on=False))
        self.assertIsNone(ml.voice_node_refusal(snapshot(), MANIFEST, on=True))

    def test_turning_off_within_a_minute_of_start_is_refused(self):
        young = snapshot(unit_age={"vica-llm.service": 18.2})
        self.assertEqual(ml.voice_node_refusal(young, MANIFEST, on=False),
                         "음성 노드가 켜지는 중입니다. 42초 뒤 다시 누르세요.")
        self.assertIsNone(ml.voice_node_refusal(young, MANIFEST, on=True))
        old = snapshot(unit_age={"vica-llm.service": ml.VOICE_OFF_GRACE_SEC})
        self.assertIsNone(ml.voice_node_refusal(old, MANIFEST, on=False))
        self.assertIsNone(ml.voice_node_refusal(snapshot(unit_age={}), MANIFEST, on=False))

    def test_moving_refusal_comes_before_starting_refusal(self):
        young_moving = snapshot(unit_age={"vica-llm.service": 5.0}, robot_moving=True)
        self.assertEqual(ml.voice_node_refusal(young_moving, MANIFEST, on=False), ml.MOVING_REFUSAL)

    def test_refuses_when_robot_state_moving(self):
        for on in (False, True):
            self.assertEqual(ml.voice_node_refusal(snapshot(robot_moving=True), MANIFEST, on=on),
                             ml.MOVING_REFUSAL)

    def test_refuses_when_only_cmd_vel_safe_moving(self):
        snap = snapshot(robot_moving=False, cmd_moving=True)
        self.assertTrue(ml.robot_in_motion(snap))
        self.assertEqual(ml.voice_node_refusal(snap, MANIFEST, on=False), ml.MOVING_REFUSAL)

    def test_refuses_outside_drive_mode(self):
        self.assertEqual(ml.voice_node_refusal(idle_snapshot(), MANIFEST, on=True), "주행 모드가 아닙니다.")

    def test_refuses_without_voice_unit(self):
        manifest = {k: v for k, v in MANIFEST.items() if k != "llm"}
        self.assertEqual(ml.voice_node_refusal(snapshot(), manifest, on=True), "음성 유닛이 없습니다.")

    def test_mic_check_deactivating_blocks_turning_on_only(self):
        snap = self._with_check("deactivating")
        self.assertTrue(ml.mic_check_active(snap))
        self.assertEqual(ml.voice_node_refusal(snap, MANIFEST, on=True),
                         "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요.")
        self.assertIsNone(ml.voice_node_refusal(snap, MANIFEST, on=False))

    def test_mic_check_inactive_is_not_active(self):
        self.assertFalse(ml.mic_check_active(snapshot()))
        self.assertFalse(ml.mic_check_active(self._with_check("failed")))

    def test_twist_moving_threshold(self):
        self.assertFalse(ml.twist_moving(0.0, 0.0))
        self.assertFalse(ml.twist_moving(0.0005, 0.0))
        self.assertTrue(ml.twist_moving(0.0, -0.2))


class ParseShow(unittest.TestCase):
    def test_reads_when_each_unit_became_active(self):
        text = SHOW.replace("InvocationID=2a7bbeb2412d40988548348501265d46",
                            "InvocationID=2a7bbeb2412d40988548348501265d46\n"
                            "ActiveEnterTimestampMonotonic=1314262724")
        _, _, entered = ml.parse_show(text)
        self.assertEqual(entered["vica-nav2.service"], 1314262724)
        self.assertNotIn("vica-mission.service", entered)

    def test_reads_when_each_unit_began_to_stop(self):
        text = SHOW.replace("InvocationID=2a7bbeb2412d40988548348501265d46",
                            "InvocationID=2a7bbeb2412d40988548348501265d46\n"
                            "ActiveExitTimestampMonotonic=1314999999")
        exited = ml.parse_exits(text)
        self.assertEqual(exited, {"vica-nav2.service": 1314999999})

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

    def test_localization_abort_counts_as_an_abort(self):
        loc = self.ABORT.replace("lifecycle_manager_navigation", "lifecycle_manager_localization")
        self.assertEqual(ml.nav2_phase([self.POSE, loc]), "aborted")

    def test_localization_abort_wins_over_navigation_active(self):
        loc = self.ABORT.replace("lifecycle_manager_navigation", "lifecycle_manager_localization")
        self.assertEqual(ml.nav2_phase([self.ACTIVE, loc]), "aborted")

    def test_no_progress_for_too_long_is_stalled(self):
        self.assertEqual(ml.nav2_phase([], age_sec=ml.NAV2_STALL_SEC), "stalled")
        self.assertEqual(ml.nav2_phase([], age_sec=ml.NAV2_STALL_SEC - 1), "starting")
        self.assertEqual(ml.nav2_phase([]), "starting")

    def test_waiting_for_the_pose_is_not_stalled(self):
        self.assertEqual(ml.nav2_phase([self.POSE], age_sec=600), "waiting_pose")
        self.assertEqual(ml.nav2_phase([self.ABORT], age_sec=600), "aborted")

    def test_failed_read_keeps_the_previous_phase(self):
        late = ml.NAV2_STALL_SEC * 15
        self.assertEqual(ml.settle_nav2_phase("waiting_pose", None, late), "waiting_pose")
        self.assertEqual(ml.settle_nav2_phase("active", None, late), "active")
        self.assertEqual(ml.settle_nav2_phase("starting", None, late), "starting")

    def test_failed_read_without_a_previous_phase_is_starting(self):
        self.assertEqual(ml.settle_nav2_phase(None, None, ml.NAV2_STALL_SEC * 15), "starting")

    def test_progress_seen_never_falls_back_to_stalled(self):
        late = ml.NAV2_STALL_SEC * 15
        self.assertEqual(ml.settle_nav2_phase("waiting_pose", [], late), "waiting_pose")
        self.assertEqual(ml.settle_nav2_phase("active", [], late), "active")

    def test_a_real_abort_still_wins(self):
        self.assertEqual(ml.settle_nav2_phase("waiting_pose", [self.POSE, self.ABORT], 600), "aborted")

    def test_settled_phase_matches_a_fresh_read_otherwise(self):
        self.assertEqual(ml.settle_nav2_phase(None, [], ml.NAV2_STALL_SEC), "stalled")
        self.assertEqual(ml.settle_nav2_phase("starting", [], ml.NAV2_STALL_SEC), "stalled")
        self.assertEqual(ml.settle_nav2_phase("starting", [self.POSE], 30), "waiting_pose")
        self.assertEqual(ml.settle_nav2_phase("waiting_pose", [self.POSE, self.ACTIVE], 30), "active")

    def test_other_lifecycle_managers_do_not_count(self):
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
        s = self.status(snapshot(procs={"mission_manager": 0}))
        self.assertEqual(s["state"], "error")
        self.assertIn("mission", s["missing_nodes"])

    def test_nodes_not_up_yet_right_after_start_is_starting_not_error(self):
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
        self.assertIsNone(ml.drive_stop_refusal(snapshot(robot_moving=None)))


class PaneNodes(unittest.TestCase):
    """유닛 밖(칸·터미널)에서 뜬 주행 노드(사용자 11-18 B안). 앱은 노드 목록(/rosapi/nodes) 대신 이것을 본다 —"""

    def test_none_while_units_run_them(self):
        self.assertEqual(ml.derive_status(snapshot(), MANIFEST)["pane_nodes"], [])

    def test_none_when_everything_is_off(self):
        self.assertEqual(ml.derive_status(idle_snapshot(), MANIFEST)["pane_nodes"], [])

    def test_nav2_from_a_terminal_is_listed(self):
        snap = idle_snapshot(procs={"bt_navigator": 1, "amcl": 1})
        self.assertEqual(ml.derive_status(snap, MANIFEST)["pane_nodes"], ["amcl", "bt_navigator"])

    def test_names_shared_with_mapping_are_left_out(self):
        snap = idle_snapshot(procs={"ekf_node": 1, "encoder_feedbac": 1, "cartographer_no": 1})
        self.assertEqual(ml.derive_status(snap, MANIFEST)["pane_nodes"], [])

    def test_units_still_stopping_are_not_counted(self):
        snap = idle_snapshot(units={"vica-nav2.service": ("deactivating", "stop-sigterm", "")},
                             procs={"bt_navigator": 1})
        self.assertEqual(ml.derive_status(snap, MANIFEST)["pane_nodes"], [])


class ShutdownRefusal(unittest.TestCase):
    """앱 '로봇 끄기'(사용자 11-18): 움직이는 중·지도 그리기 중이면 거절한다."""

    def test_refused_while_moving(self):
        self.assertIsNotNone(ml.shutdown_refusal(snapshot(robot_moving=True)))

    def test_refused_while_mapping(self):
        self.assertIn("지도 그리기", ml.shutdown_refusal(snapshot(mapping_state="mapping")))

    def test_allowed_when_still_and_idle(self):
        self.assertIsNone(ml.shutdown_refusal(snapshot()))
        self.assertIsNone(ml.shutdown_refusal(snapshot(robot_moving=None)))


class CommandLineCandidates(unittest.TestCase):
    """명령줄을 읽을 프로세스만 고른다. 매 초 전부 읽으면 노드가 CPU 를 14~19 % 쓴다(11-14 실측)."""

    def test_python_processes_are_read(self):
        self.assertTrue(ml.needs_cmdline("python3"))
        self.assertTrue(ml.needs_cmdline("python"))

    def test_other_processes_are_skipped(self):
        for comm in ("bt_navigator", "bash", "rplidar_node", "mission_manager"):
            self.assertFalse(ml.needs_cmdline(comm), comm)

    def test_every_command_line_guard_starts_with_python3(self):
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

    def test_retries_a_stalled_bringup_once(self):
        self.assertTrue(ml.should_auto_retry(snapshot(nav2_phase="stalled")))
        self.assertFalse(ml.should_auto_retry(snapshot(nav2_phase="stalled", auto_retried=True)))
        first = ml.derive_status(snapshot(nav2_phase="stalled"), MANIFEST)
        self.assertEqual(first["state"], "starting")
        self.assertIn("자동", first["detail"])
        second = ml.derive_status(snapshot(nav2_phase="stalled", auto_retried=True), MANIFEST)
        self.assertEqual(second["state"], "error")

    def test_not_while_nav2_is_being_stopped_by_hand(self):
        for st in ("deactivating", "inactive", "failed"):
            snap = snapshot(nav2_phase="aborted", units={"vica-nav2.service": (st, "x", "success")})
            self.assertFalse(ml.should_auto_retry(snap), st)

    def test_nav2_restarting_after_the_retry_is_still_starting(self):
        snap = snapshot(nav2_phase="starting", auto_retried=True,
                        units={"vica-nav2.service": ("deactivating", "stop-sigterm", "success")})
        self.assertEqual(ml.derive_status(snap, MANIFEST)["state"], "starting")

    def test_whole_bundle_going_down_is_stopping(self):
        snap = snapshot(auto_retried=True,
                        units={"vica-drive.target": ("inactive", "dead", "success"),
                               "vica-nav2.service": ("deactivating", "stop-sigterm", "success")})
        self.assertEqual(ml.derive_status(snap, MANIFEST)["state"], "stopping")

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


class DeferredStop(unittest.TestCase):
    """기동 중 정지는 Nav2 가 '초기 위치 대기'에 이를 때까지(최대 45 s) 미룬다(사용자 11-16)."""

    def test_stop_waits_while_nav2_is_still_coming_up(self):
        self.assertTrue(ml.stop_should_wait(snapshot(nav2_phase="starting"), MANIFEST))

    def test_stop_does_not_wait_once_nav2_is_up_or_has_failed(self):
        for phase in ("waiting_pose", "active", "aborted", "stalled", "down"):
            self.assertFalse(ml.stop_should_wait(snapshot(nav2_phase=phase), MANIFEST), phase)
        self.assertFalse(ml.stop_should_wait(idle_snapshot(nav2_phase="starting"), MANIFEST))

    def test_deferred_stop_goes_ahead_when_nav2_is_up_or_time_is_up(self):
        self.assertTrue(ml.deferred_stop_due(snapshot(nav2_phase="waiting_pose", stop_pending_sec=5), MANIFEST))
        self.assertFalse(ml.deferred_stop_due(snapshot(nav2_phase="starting", stop_pending_sec=5), MANIFEST))
        self.assertTrue(ml.deferred_stop_due(
            snapshot(nav2_phase="starting", stop_pending_sec=ml.STOP_DEFER_MAX_SEC), MANIFEST))

    def test_nothing_is_due_without_a_request(self):
        self.assertFalse(ml.deferred_stop_due(snapshot(nav2_phase="waiting_pose"), MANIFEST))

    def test_status_says_it_will_stop_after_startup(self):
        s = ml.derive_status(snapshot(nav2_phase="starting", stop_pending_sec=10), MANIFEST)
        self.assertEqual((s["mode"], s["state"]), ("drive", "stopping"))
        self.assertIn("다 뜬 뒤", s["detail"])
        self.assertEqual(s["stop_in_sec"], 35)
        later = ml.derive_status(snapshot(nav2_phase="starting", stop_pending_sec=20), MANIFEST)
        self.assertEqual(later["detail"], s["detail"])

    def test_no_auto_retry_while_a_stop_is_pending(self):
        self.assertFalse(ml.should_auto_retry(snapshot(nav2_phase="aborted", stop_pending_sec=3)))


class SlowStop(unittest.TestCase):
    """정지가 늦거나 강제 종료되면 상태에 드러낸다. 11-16 13:34~13:45 앞 부팅에서 정지가 매번 61~72 s"""

    STOPPING = {"vica-drive.target": ("inactive", "dead", "success"),
                "vica-nav2.service": ("deactivating", "stop-sigterm", "success")}

    def test_a_long_stop_is_flagged(self):
        s = ml.derive_status(snapshot(units=self.STOPPING, stopping_sec=31), MANIFEST)
        self.assertEqual(s["state"], "stopping")
        self.assertIn("늦어", s["detail"])

    def test_a_normal_stop_is_not_flagged(self):
        s = ml.derive_status(snapshot(units=self.STOPPING, stopping_sec=10), MANIFEST)
        self.assertEqual((s["state"], s["detail"]), ("stopping", ""))

    def test_a_unit_killed_on_the_last_stop_is_shown_when_idle(self):
        snap = idle_snapshot(units={"vica-nav2.service": ("inactive", "dead", "timeout")})
        s = ml.derive_status(snap, MANIFEST)
        self.assertEqual((s["mode"], s["state"]), ("idle", "idle"))
        self.assertEqual(s["stop_timeouts"], ["nav2"])
        self.assertIn("재부팅", s["detail"])

    def test_clean_idle_has_no_warning(self):
        s = ml.derive_status(idle_snapshot(), MANIFEST)
        self.assertEqual((s["stop_timeouts"], s["detail"]), ([], ""))


class DriveComingUp(unittest.TestCase):
    """주행 묶음 target 은 소속 유닛이 다 뜰 때까지 inactive 로 남는다(systemd 기본 순서)."""

    COMING = {"vica-drive.target": ("inactive", "dead", "success"),
              "vica-nav2.service": ("active", "running", "success"),
              "vica-mission.service": ("activating", "start-pre", "success")}

    def test_members_coming_up_count_as_drive_on(self):
        self.assertTrue(ml.drive_on(snapshot(units=self.COMING, nav2_phase="starting"), MANIFEST))
        self.assertFalse(ml.drive_on(idle_snapshot(), MANIFEST))

    def test_a_stop_while_coming_up_is_deferred(self):
        self.assertTrue(ml.stop_should_wait(snapshot(units=self.COMING, nav2_phase="starting"), MANIFEST))

    def test_status_is_drive_starting(self):
        s = ml.derive_status(snapshot(units=self.COMING, nav2_phase="starting"), MANIFEST)
        self.assertEqual((s["mode"], s["state"]), ("drive", "starting"))

    def test_status_while_a_deferred_stop_waits_on_coming_up(self):
        s = ml.derive_status(snapshot(units=self.COMING, nav2_phase="starting", stop_pending_sec=2), MANIFEST)
        self.assertEqual((s["mode"], s["state"]), ("drive", "stopping"))

    def test_start_is_not_repeated_while_coming_up(self):
        self.assertIsNone(ml.drive_start_refusal(snapshot(units=self.COMING), MANIFEST))


class DetectorAndVoice(unittest.TestCase):
    """감지기 잠시 끄기·음성 온라인/오프라인 고르기(사용자 11-16)."""

    def test_status_reports_detector_voice_and_voice_choice(self):
        snap = snapshot(units={"vica-detector.service": ("inactive", "dead", "success"),
                               "vica-llm.service": ("active", "running", "success")},
                        voice_mode="online")
        s = ml.derive_status(snap, MANIFEST)
        self.assertEqual((s["detector"], s["voice"], s["voice_mode"]), ("inactive", "active", "online"))

    def test_a_detector_turned_off_by_hand_is_not_an_error(self):
        snap = snapshot(units={"vica-detector.service": ("inactive", "dead", "success")})
        self.assertNotEqual(ml.derive_status(snap, MANIFEST)["state"], "error")

    def test_detector_can_be_toggled_only_while_driving(self):
        self.assertIsNone(ml.detector_refusal(snapshot(), MANIFEST))
        self.assertIsNotNone(ml.detector_refusal(idle_snapshot(), MANIFEST))

    def test_voice_is_not_switched_while_moving(self):
        self.assertIsNotNone(ml.voice_switch_refusal(snapshot(robot_moving=True)))
        self.assertIsNone(ml.voice_switch_refusal(snapshot()))
        self.assertIsNone(ml.voice_switch_refusal(idle_snapshot()))

    def test_voice_mode_file_reading(self):
        self.assertEqual(ml.read_voice_mode("online\n"), "online")
        self.assertEqual(ml.read_voice_mode("offline"), "offline")
        self.assertEqual(ml.read_voice_mode(""), "offline")
        self.assertEqual(ml.read_voice_mode("이상한값"), "offline")


class CmdlineScan(unittest.TestCase):
    def test_ros2_launch_frontends_are_scanned(self):
        self.assertTrue(ml.needs_cmdline("ros2"))
        self.assertTrue(ml.needs_cmdline("python3"))
        self.assertFalse(ml.needs_cmdline("controller_serv"))


class VoiceEvents(unittest.TestCase):
    """음성 연결 알림(사용자 11-16): 외부 API·로컬 API 연결 성공/실패를 앱에 알린다."""

    def run_lines(self, lines):
        tracker = ml.VoiceEventTracker()
        events = []
        for line in lines:
            events.extend(tracker.feed(line))
        return [kind for kind, _text in events]

    def test_online_warmup_without_failure_is_external_success(self):
        self.assertEqual(self.run_lines(["음성: online", "[INFO] [x] [vica_llm_intent_node]: LLM 워밍업 완료 (3.2초)"]),
                         ["external_ok"])

    def test_offline_warmup_without_failure_is_local_success(self):
        self.assertEqual(self.run_lines(["음성: offline", "... LLM 워밍업 완료 (9.0초)"]), ["local_ok"])

    def test_warmup_done_after_a_failed_call_is_not_a_success(self):
        self.assertEqual(self.run_lines([
            "음성: offline",
            "[vica_llm-1] [ERROR] [LLM] 호출 실패(클라우드만, 폴백 꺼짐): timed out",
            "... LLM 워밍업 완료 (15.1초)",
        ]), ["local_fail"])

    def test_cloud_failure_with_fallback_is_a_switch_to_local(self):
        self.assertEqual(self.run_lines([
            "음성: online",
            "[LLM] 클라우드 실패(timeout: timed out) → 로컬(midm2-mini)로 대피. 같은 발화 재처리",
        ]), ["external_fallback"])

    def test_failure_without_fallback_depends_on_the_mode(self):
        line = "[LLM] 호출 실패(클라우드만, 폴백 꺼짐): Connection error."
        self.assertEqual(self.run_lines(["음성: online", line]), ["external_fail"])
        self.assertEqual(self.run_lines(["음성: offline", line]), ["local_fail"])

    def test_preload_failure_is_a_local_failure(self):
        self.assertEqual(self.run_lines([
            "음성: offline",
            "[Ollama] 꺼져 있어 켭니다",
            "[Ollama] midm2-mini 올리기 실패 (180 s) — 첫 질문이 늦거나 실패할 수 있습니다",
            "... LLM 워밍업 완료 (15.1초)",
        ]), ["local_fail"])

    def test_preloaded_model_then_warmup_is_a_local_success(self):
        self.assertEqual(self.run_lines([
            "음성: offline",
            "[Ollama] midm2-mini 올림 (47 s)",
            "... LLM 워밍업 완료 (2.1초)",
        ]), ["local_ok"])

    def test_both_failing(self):
        self.assertEqual(self.run_lines(["음성: online", "[LLM] 호출 실패(로컬까지): refused"]), ["both_fail"])

    def test_realtime_and_recovery_and_auth(self):
        self.assertEqual(self.run_lines([
            "음성: online",
            "Realtime 연결 예열 실패(무시 가능): 401",
            "[LLM] Realtime 연결 오류 → 클라우드 건너뛰고 로컬(midm2-mini)로",
            "[LLM] 클라우드는 닿지만 인증 실패 — 키를 확인하세요. 복귀 보류",
            "[LLM] 주행 끝 → 클라우드 복귀",
        ]), ["realtime_fail", "external_fallback", "external_auth", "external_back"])

    def test_texts_say_external_or_local_api(self):
        tracker = ml.VoiceEventTracker()
        tracker.feed("음성: online")
        (kind, text), = tracker.feed("LLM 워밍업 완료 (1.0초)")
        self.assertEqual(text, "외부 API 연결 성공")
        tracker = ml.VoiceEventTracker()
        tracker.feed("음성: offline")
        (kind, text), = tracker.feed("[LLM] 호출 실패(클라우드만, 폴백 꺼짐): timed out")
        self.assertTrue(text.startswith("로컬 API 연결 실패"), text)

    def test_a_new_start_resets_the_failure_memory(self):
        self.assertEqual(self.run_lines([
            "음성: offline", "[LLM] 호출 실패(클라우드만, 폴백 꺼짐): timed out",
            "음성: online", "LLM 워밍업 완료 (2.0초)",
        ]), ["local_fail", "external_ok"])

    def test_unrelated_lines_are_ignored(self):
        self.assertEqual(self.run_lines(["음성: online", "[vica_tts-2] 고정 문장 워밍업 완료: 39건"]), [])


MAP_MANIFEST = dict(MANIFEST, posecheck={"unit": "vica-posecheck.service", "bundle": "drive",
                                         "guard": ["pose_bootstrap"], "guard_cmdline": ""})


def map_snapshot(**overrides):
    """주행이 떠 있고 멈춰 대기 중. nav2·mission·llm·posecheck 가 4층 복도 지도로 떠 있다."""
    units = {"vica-llm.service": ("active", "running", "success"),
             "vica-posecheck.service": ("active", "running", "success")}
    units.update(overrides.pop("units", {}))
    hits = {"^[^ ]*python3 [^ ]*ros2 launch launch/vica_voice\\.launch\\.py": 1}
    hits.update(overrides.pop("cmdline_hits", {}))
    procs = {"pose_bootstrap": 1}
    procs.update(overrides.pop("procs", {}))
    overrides.setdefault("dialog_state", "idle")
    overrides.setdefault("current_map", "map_1116_203022")
    overrides.setdefault("running_maps", {"nav2": "map_1116_203022", "mission": "map_1116_203022",
                                          "llm": "map_1116_203022"})
    return snapshot(units=units, cmdline_hits=hits, procs=procs, **overrides)


class MapChange(unittest.TestCase):
    """주행 지도 바꾸기(사용자 11-17): 멈춰 대기 중이거나 일시정지 중일 때만, 지도를 쥔 유닛만 다시 띄운다."""

    def refusal(self, snap, map_id="map_1103_142858", problem=None):
        state = ml.derive_status(snap, MAP_MANIFEST)["state"]
        return ml.map_change_refusal(snap, MAP_MANIFEST, state, map_id, problem)

    def test_waiting_or_paused_robot_may_change(self):
        for dialog in ("idle", "arrived", "failed", "paused", "paused_handle", "patrol_paused"):
            self.assertIsNone(self.refusal(map_snapshot(dialog_state=dialog)), dialog)

    def test_busy_robot_may_not_change_and_says_why(self):
        cases = {"navigating": "안내 중", "approaching": "사람 접근 중", "patrolling": "순찰 중",
                 "estopped": "비상정지 중", "grip_wait": "손잡이 잡기 대기 중", "waiting": "도착 뒤"}
        for dialog, word in cases.items():
            reason = self.refusal(map_snapshot(dialog_state=dialog))
            self.assertIn(word, reason or "", dialog)

    def test_moving_or_unknown_robot_may_not_change(self):
        self.assertIn("움직이는", self.refusal(map_snapshot(robot_moving=True)))
        self.assertIn("받지 못함", self.refusal(map_snapshot(dialog_state=None)))
        self.assertIn("받지 못함", self.refusal(map_snapshot(robot_moving=None)))

    def test_not_while_drive_is_starting_or_stopping(self):
        self.assertIn("켜거나 끄는", self.refusal(map_snapshot(nav2_phase="starting")))
        self.assertIn("켜거나 끄는", self.refusal(map_snapshot(stop_pending_sec=3.0)))

    def test_not_while_drawing_a_map(self):
        snap = idle_snapshot(mapping_state="mapping", dialog_state=None, robot_moving=None)
        self.assertIn("지도 그리는", self.refusal(snap))

    def test_drive_off_needs_no_robot_state(self):
        snap = idle_snapshot(dialog_state=None, robot_moving=None)
        self.assertIsNone(self.refusal(snap))
        self.assertEqual(ml.map_restart_units(snap, MAP_MANIFEST, "map_1103_142858"), [])

    def test_bad_name_or_missing_files_are_refused(self):
        self.assertIn("올바르지", self.refusal(map_snapshot(), "../etc"))
        self.assertIn("올바르지", self.refusal(map_snapshot(), ""))
        self.assertEqual(self.refusal(map_snapshot(), problem="지도 파일 없음"), "지도 파일 없음")
        self.assertIn("주행 지도가 아님", self.refusal(map_snapshot(), "map_1116_203022_keepout"))

    def test_restarts_every_unit_that_holds_the_map(self):
        units = ml.map_restart_units(map_snapshot(), MAP_MANIFEST, "map_1103_142858")
        self.assertEqual(units, ["vica-nav2.service", "vica-mission.service", "vica-llm.service",
                                 "vica-posecheck.service"])

    def test_handle_detector_and_safety_are_left_alone(self):
        manifest = dict(MAP_MANIFEST, handle={"unit": "vica-handle.service", "bundle": "drive",
                                              "guard": [], "guard_cmdline": ""})
        snap = map_snapshot(units={"vica-handle.service": ("active", "running", "success"),
                                   "vica-detector.service": ("active", "running", "success")})
        units = ml.map_restart_units(snap, manifest, "map_1103_142858")
        self.assertNotIn("vica-handle.service", units)
        self.assertNotIn("vica-detector.service", units)
        self.assertNotIn("vica-motor.service", units)

    def test_nothing_to_restart_when_units_already_run_that_map(self):
        snap = map_snapshot(current_map="map_1029_074730")
        self.assertEqual(ml.map_restart_units(snap, MAP_MANIFEST, "map_1116_203022"), [])

    def test_unknown_running_map_restarts(self):
        snap = map_snapshot(running_maps={})
        self.assertEqual(len(ml.map_restart_units(snap, MAP_MANIFEST, "map_1116_203022")), 4)

    def test_status_carries_map_fields(self):
        status = ml.derive_status(map_snapshot(), MAP_MANIFEST)
        self.assertEqual(status["current_map_id"], "map_1116_203022")
        self.assertEqual(status["running_maps"]["nav2"], "map_1116_203022")
        self.assertFalse(status["map_mismatch"])
        self.assertEqual(status["map_change_block"], "")
        busy = ml.derive_status(map_snapshot(dialog_state="navigating"), MAP_MANIFEST)
        self.assertIn("안내 중", busy["map_change_block"])
        self.assertEqual(busy["dialog_state"], "navigating")

    def test_file_changed_behind_the_drive_is_a_mismatch(self):
        status = ml.derive_status(map_snapshot(current_map="map_1029_074730"), MAP_MANIFEST)
        self.assertTrue(status["map_mismatch"])
        idle = ml.derive_status(idle_snapshot(current_map="map_1029_074730",
                                              running_maps={"nav2": "map_1116_203022"}), MAP_MANIFEST)
        self.assertFalse(idle["map_mismatch"])

    def test_reads_the_map_mark_from_the_journal(self):
        lines = ["\x1b[0;36m현재 지도\x1b[0m 4층 복도 (map_1116_203022 · maps/CURRENT_MAP)",
                 "[vica-map] map_1116_203022", "실행: ros2 launch …"]
        self.assertEqual(ml.parse_map_mark(lines), "map_1116_203022")
        self.assertEqual(ml.parse_map_mark(["현재 지도 x"]), "")


class BridgeWatchTest(unittest.TestCase):
    """rosbridge 감시(11-17). 처리 스레드가 죽어 소켓만 살아 있던 일을 다시 띄워 살린다."""

    def run_silent(self, w, start, secs, pub_every=2.0):
        t = start
        while t < start + secs:
            w.published(t)
            t += pub_every
        return start + secs

    def test_flowing_bridge_is_ok(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        for t in range(0, 60, 2):
            w.published(float(t))
            w.received(float(t) + 0.1)
        self.assertEqual(w.verdict(60.0, moving=False), "ok")

    def test_silent_bridge_while_we_publish_is_restarted(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        now = self.run_silent(w, 0.0, 16.0)
        self.assertEqual(w.verdict(now, moving=False), "restart")

    def test_not_judged_before_the_silent_window(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        now = self.run_silent(w, 0.0, 14.0)
        self.assertEqual(w.verdict(now, moving=False), "wait")

    def test_bridge_that_stopped_after_working_is_restarted(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        w.received(5.0)
        now = self.run_silent(w, 5.0, 16.0)
        self.assertEqual(w.verdict(now, moving=False), "restart")

    def test_our_own_silence_is_not_blamed_on_the_bridge(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        w.published(1.0)
        self.assertNotEqual(w.verdict(30.0, moving=False), "restart")

    def test_held_while_moving_then_restarted_when_stopped(self):
        w = ml.BridgeWatch()
        w.connected(0.0)
        now = self.run_silent(w, 0.0, 20.0)
        self.assertEqual(w.verdict(now, moving=True), "hold")
        self.assertIn("움직이는", w.hold_reason)
        self.assertEqual(w.verdict(now, moving=False), "restart")

    def test_grace_and_cooldown_after_restart(self):
        w = ml.BridgeWatch()
        w.restarted(100.0)
        w.connected(120.0)
        now = self.run_silent(w, 120.0, 30.0)
        self.assertEqual(w.verdict(now, moving=False), "wait")
        w.connected(170.0)
        now = self.run_silent(w, 170.0, 20.0)
        self.assertEqual(w.verdict(now, moving=False), "hold")
        w.connected(390.0)
        now = self.run_silent(w, 390.0, 20.0)
        self.assertEqual(w.verdict(now, moving=False), "restart")

    def test_disconnected_is_not_judged(self):
        w = ml.BridgeWatch()
        self.run_silent(w, 0.0, 30.0)
        self.assertEqual(w.verdict(30.0, moving=False), "wait")

    def test_reads_rosbridge_escaped_topic(self):
        raw = '{"op":"publish","topic":"\\/vica\\/mode_status","msg":{"data":"{}"}}'
        self.assertTrue(ml.bridge_message_is_ours(raw))
        self.assertTrue(ml.bridge_message_is_ours('{"op": "publish", "topic": "/vica/mode_status", "msg": {}}'))
        self.assertFalse(ml.bridge_message_is_ours('{"op":"publish","topic":"\\/robot_status","msg":{}}'))
        self.assertFalse(ml.bridge_message_is_ours('{"op":"status","msg":"x"}'))
        self.assertFalse(ml.bridge_message_is_ours("not json"))


class MicCheckTest(unittest.TestCase):
    """마이크 점검(앱 '마이크 점검', 11-19 설계) 시작·정지 거절 규칙과 실행 명령."""

    def _snap(self, base=snapshot, units=None, **overrides):
        merged = dict(units or {})
        return base(units=merged, **overrides)

    def _unit(self, name, state):
        return {name: (state, "running", "success")}

    def test_start_allowed_when_drive_off_and_voice_off(self):
        self.assertIsNone(ml.mic_check_start_refusal(idle_snapshot(), MANIFEST))

    def test_start_allowed_when_drive_on_but_voice_off_and_still(self):
        self.assertIsNone(ml.mic_check_start_refusal(snapshot(), MANIFEST))

    def test_start_refuses_when_already_running(self):
        for state in ("active", "activating", "deactivating"):
            snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, state))
            self.assertEqual(ml.mic_check_start_refusal(snap, MANIFEST), "이미 점검 중입니다.")

    def test_detector_on_refused_while_checking_but_off_allowed(self):
        snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, "active"))
        self.assertEqual(ml.detector_refusal(snap, MANIFEST, True),
                         "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요.")
        self.assertIsNone(ml.detector_refusal(snap, MANIFEST, False))
        self.assertIsNone(ml.detector_refusal(self._snap(), MANIFEST, True))

    def test_start_refuses_while_mapping(self):
        snap = idle_snapshot(mapping_state="running")
        self.assertEqual(ml.mic_check_start_refusal(snap, MANIFEST), "지도를 그리는 중입니다.")

    def test_start_refuses_when_cartographer_process(self):
        snap = idle_snapshot(procs={"cartographer_no": 1})
        self.assertEqual(ml.mic_check_start_refusal(snap, MANIFEST), "지도를 그리는 중입니다.")

    def test_start_refuses_when_moving(self):
        self.assertEqual(ml.mic_check_start_refusal(snapshot(robot_moving=True), MANIFEST), ml.MOVING_REFUSAL)
        self.assertEqual(ml.mic_check_start_refusal(snapshot(cmd_moving=True), MANIFEST), ml.MOVING_REFUSAL)

    def test_start_refuses_when_voice_unit_up(self):
        for state in ("active", "activating", "deactivating"):
            snap = self._snap(units=self._unit("vica-llm.service", state))
            self.assertEqual(ml.mic_check_start_refusal(snap, MANIFEST),
                             "음성 노드가 켜져 있습니다. 먼저 끄세요.")

    def test_start_refuses_when_voice_from_terminal(self):
        expect = "터미널에서 띄운 음성이 있습니다. 그 칸을 끈 뒤 다시 누르세요."
        self.assertEqual(ml.mic_check_start_refusal(snapshot(procs={"vica_llm_node": 1}), MANIFEST), expect)
        hit = MANIFEST["llm"]["guard_cmdline"]
        self.assertEqual(ml.mic_check_start_refusal(snapshot(cmdline_hits={hit: 1}), MANIFEST), expect)

    def test_start_order_already_before_mapping(self):
        snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, "active"), mapping_state="running")
        self.assertEqual(ml.mic_check_start_refusal(snap, MANIFEST), "이미 점검 중입니다.")

    def test_stop_refusal(self):
        self.assertEqual(ml.mic_check_stop_refusal(snapshot()), "점검 중이 아닙니다.")
        snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, "active"))
        self.assertIsNone(ml.mic_check_stop_refusal(snap))

    def test_side_stops_detector_only_when_drive_on_and_detector_up(self):
        for state in ("active", "activating"):
            snap = self._snap(units=self._unit("vica-detector.service", state))
            self.assertEqual(ml.mic_check_side_stops(snap, MANIFEST), ["vica-detector.service"])
        self.assertEqual(ml.mic_check_side_stops(snapshot(), MANIFEST), [])
        self.assertEqual(ml.mic_check_side_stops(idle_snapshot(), MANIFEST), [])

    def test_command(self):
        cmd = ml.mic_check_command("/v", "", None)
        self.assertEqual(cmd[:4], ["systemd-run", "--user", "--collect", f"--unit={ml.MIC_CHECK_UNIT}"])
        self.assertIn("--setenv=VICA_MIC_CHECK_LAUNCHER=mode", cmd)
        self.assertIn("--working-directory=/v", cmd)
        self.assertIn("/v/.venv/bin/python", cmd)
        self.assertEqual(cmd[cmd.index("--name"):], ["--name", "점검"])
        self.assertNotIn("--destinations", cmd)
        with_dest = ml.mic_check_command("/v", "map1", "a,b")
        self.assertEqual(with_dest[-4:], ["--name", "map1", "--destinations", "a,b"])

    def test_status_has_mic_check(self):
        self.assertEqual(ml.derive_status(snapshot(), MANIFEST)["mic_check"],
                         {"state": "inactive", "port": 8770})
        snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, "active"))
        self.assertEqual(ml.derive_status(snap, MANIFEST)["mic_check"]["state"], "active")

    def test_drive_start_refused_while_checking(self):
        snap = idle_snapshot(units=self._unit(ml.MIC_CHECK_UNIT, "active"))
        self.assertEqual(ml.drive_start_refusal(snap, MANIFEST),
                         "마이크 점검 중입니다. 점검을 끝낸 뒤 켜세요.")

    def test_map_change_blocked_while_checking(self):
        snap = self._snap(units=self._unit(ml.MIC_CHECK_UNIT, "deactivating"))
        self.assertEqual(ml.map_change_block(snap, MANIFEST, "running"),
                         "마이크 점검 중입니다. 점검을 끝낸 뒤 바꾸세요.")
