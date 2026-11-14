"""터미네이터 생성기의 systemd 유닛 출력(`--units`).

칸과 유닛은 같은 Term 정의에서 나온다. 여기서 지키는 것:

- 유닛을 더해도 칸의 rc 는 한 글자도 달라지지 않는다
- 실행 스크립트의 끝나는 값(0 정지 · 70 스스로 끝남 · 75 중복 거부)
- 터미널이 있어야 도는 명령(-it, sudo, history)이 유닛에 들어가지 않는다
- 바퀴가 도는 유닛은 단계를 올려야만 묶음에 들어온다
"""
import copy
import os
import re
import shutil
import signal
import stat
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from scripts import vica_terminator_layout as g

ROS_WS = g.DEFAULT_ROS_WS
MAP_ID = g.DEFAULT_MAP_ID


def unit_terms():
    return {key: term for key, term in g.build_terms().items() if term.unit}


def run_script(key, term):
    return g.render_run_script(key, term, ROS_WS, MAP_ID)


class RcIsUntouched(unittest.TestCase):
    def test_rc_does_not_depend_on_the_unit_field(self):
        for key, term in unit_terms().items():
            bare = copy.copy(term)
            bare.unit = None
            self.assertEqual(
                g.render_rc(term, ROS_WS, MAP_ID), g.render_rc(bare, ROS_WS, MAP_ID), key
            )

    def test_run_script_sets_the_same_ros_environment_as_the_rc(self):
        ros_block = g.ROS_BLOCK.format(ros_ws=ROS_WS, map_id=MAP_ID)
        term = g.build_terms()["display"]
        self.assertIn(ros_block, g.render_rc(term, ROS_WS, MAP_ID))
        self.assertIn(ros_block, run_script("display", term))


class RunScriptExitCodes(unittest.TestCase):
    """만든 스크립트를 실제로 돌려 끝나는 값을 본다. ROS 는 안 쓰는 임시 칸으로."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def script(self, command, guard=(), guard_cmdline=""):
        term = g.Term(
            title="시험 칸", note=(), command=command, ros=False, guard=guard,
            unit=g.Unit(stage="1a", guard_cmdline=guard_cmdline),
        )
        path = self.tmp / "run_test.sh"
        path.write_text(run_script("test", term), encoding="utf-8")
        path.chmod(path.stat().st_mode | stat.S_IEXEC)
        return path

    def test_command_ending_by_itself_is_reported_as_a_failure(self):
        # 멈추라고 한 적이 없는데 끝났다 — 0 으로 끝나도 유닛은 failed 로 보여야 한다.
        self.assertEqual(subprocess.run([str(self.script("true"))]).returncode, 70)

    def test_command_error_code_is_passed_through(self):
        self.assertEqual(subprocess.run([str(self.script("(exit 3)"))]).returncode, 3)

    def test_being_interrupted_is_a_clean_exit(self):
        proc = subprocess.Popen(
            [str(self.script("sleep 30"))], start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.addCleanup(lambda: proc.poll() is None and os.killpg(proc.pid, signal.SIGKILL))
        time.sleep(0.5)
        os.killpg(proc.pid, signal.SIGINT)   # systemd 는 묶음 전체에 SIGINT 를 보낸다
        self.assertEqual(proc.wait(timeout=5), 0)

    def test_duplicate_is_refused_with_75_and_the_command_does_not_run(self):
        dummy = self.tmp / "vica_dummy_g"
        shutil.copy("/bin/sleep", dummy)
        running = subprocess.Popen([str(dummy), "30"])
        self.addCleanup(lambda: (running.kill(), running.wait()))
        time.sleep(0.2)
        marker = self.tmp / "ran"
        proc = subprocess.run(
            [str(self.script(f"touch {marker}", guard=("vica_dummy_g",)))],
            capture_output=True,
        )
        self.assertEqual(proc.returncode, 75)
        self.assertFalse(marker.exists())

    def test_duplicate_found_by_command_line_is_refused_with_75(self):
        mark = "vica_marker_cmd_k3"
        running = subprocess.Popen(["bash", "-c", f"exec -a {mark} sleep 30"])
        self.addCleanup(lambda: (running.kill(), running.wait()))
        time.sleep(0.2)
        marker = self.tmp / "ran"
        proc = subprocess.run(
            [str(self.script(f"touch {marker}", guard_cmdline=mark))], capture_output=True
        )
        self.assertEqual(proc.returncode, 75)
        self.assertFalse(marker.exists())

    def test_a_shell_that_only_mentions_the_pattern_is_not_a_duplicate(self):
        # 패턴을 ^ 로 명령줄 맨 앞에 묶는다. 안 묶으면 그 글자를 담은 아무 셸이나 걸린다 —
        # 2026-11-14 실기에서 확인용 명령이 rosbridge 로 오인돼 유닛이 기동을 거부했다.
        mark = "vica_marker_cmd_k5"
        bystander = subprocess.Popen(["bash", "-c", f"sleep 30; : {mark} 30"])
        self.addCleanup(lambda: (bystander.kill(), bystander.wait()))
        time.sleep(0.2)
        marker = self.tmp / "ran"
        proc = subprocess.run(
            [str(self.script(f"touch {marker}", guard_cmdline=f"^{mark} 30"))],
            capture_output=True,
        )
        self.assertEqual(proc.returncode, 70)
        self.assertTrue(marker.exists())

    def test_command_line_guard_lets_the_command_run_when_nothing_matches(self):
        marker = self.tmp / "ran"
        proc = subprocess.run(
            [str(self.script(f"touch {marker}", guard_cmdline="vica_marker_absent_k4"))],
            capture_output=True,
        )
        self.assertEqual(proc.returncode, 70)
        self.assertTrue(marker.exists())


class RunScriptContent(unittest.TestCase):
    def test_every_script_parses(self):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        for key, term in unit_terms().items():
            path = tmp / f"run_{key}.sh"
            path.write_text(run_script(key, term), encoding="utf-8")
            proc = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, f"{key}: {proc.stderr}")

    def test_nothing_that_needs_a_terminal_or_a_password(self):
        for key, term in unit_terms().items():
            text = run_script(key, term)
            self.assertNotRegex(text, r"docker exec[^\n]* -it\b", key)
            # 실행되는 sudo 만 본다. 안내 문구(printf '… sudo apt install …')는 괜찮다.
            self.assertNotRegex(text, r"(?m)(^|[;&|])\s*sudo\b", key)
            self.assertNotIn("history -s", text, key)
            self.assertNotRegex(text, r"(?m)^vica_rs$", key)

    def test_does_not_read_bashrc(self):
        # 대화형이 아니면 ~/.bashrc 는 첫머리에서 끝난다. 기대면 조용히 환경이 빠진다.
        for key, term in unit_terms().items():
            self.assertNotIn('source "$HOME/.bashrc"', run_script(key, term), key)

    def test_sets_what_the_pane_got_from_bashrc(self):
        text = run_script("display", g.build_terms()["display"])
        self.assertIn("/usr/local/cuda-12.6/bin", text)
        self.assertIn("/usr/local/cuda-12.6/lib64", text)
        self.assertIn("en_US.UTF-8", text)

    def test_app_bridge_passes_the_map_just_like_the_pane(self):
        # supervisor_bringup 은 map_yaml 이 비면 `-p map_yaml:=` 를 넘기고, rcl 이 그것을
        # 해석하지 못해 vica_status_app_node 가 죽는다(2026-11-14 실기). launch 는 살아
        # 있어서 유닛은 active 로 보인다. 그 launch 가 고쳐지기 전에는 칸과 똑같이 넘긴다.
        self.assertIn("map_yaml:=$VICA_MAP_YAML", run_script("app", g.build_terms()["app"]))

    def test_python_launched_servers_are_guarded_by_command_line(self):
        # rosbridge 는 `#!/usr/bin/env python3` 로 떠서 comm 이 python3 다. 이름으로는 못 찾는다.
        terms = g.build_terms()
        self.assertIn(
            "vica_guard_cmdline '^[^ ]*python3 [^ ]*rosbridge_server/rosbridge_websocket' || exit 75",
            run_script("app", terms["app"]),
        )
        self.assertIn(
            "vica_guard_cmdline '^[^ ]*python3 -m http.server 5000' || exit 75",
            run_script("web", terms["web"]),
        )

    def test_command_line_guards_are_anchored_to_the_start(self):
        for key, term in unit_terms().items():
            if term.unit.guard_cmdline:
                self.assertTrue(term.unit.guard_cmdline.startswith("^"), key)

    def test_camera_runs_the_cloud_script_inside_the_container(self):
        text = run_script("d455", g.build_terms()["d455"])
        self.assertIn("docker exec vica_rs_container", text)
        self.assertIn("./run_d455_cloud.sh", text)


class UnitFile(unittest.TestCase):
    def setUp(self):
        self.terms = g.build_terms()
        self.rc_dir = Path("/x/vica-terminator")

    def unit(self, key):
        return g.render_unit(key, self.terms[key], self.rc_dir)

    def test_starts_the_generated_script_not_an_inline_shell(self):
        for key in unit_terms():
            text = self.unit(key)
            self.assertIn(f"ExecStart={self.rc_dir}/run_{key}.sh\n", text, key)
            self.assertNotIn("bash -c", text, key)

    def test_stops_like_ctrl_c_and_never_restarts_on_a_duplicate(self):
        for key in unit_terms():
            text = self.unit(key)
            for line in (
                "KillMode=control-group", "KillSignal=SIGINT",
                "RestartPreventExitStatus=75",
                "UnsetEnvironment=DISPLAY XAUTHORITY WAYLAND_DISPLAY",
            ):
                self.assertIn(line + "\n", text, key)

    def test_no_unit_is_bound_to_another(self):
        # Requires·BindsTo 로 묶으면 motor 재기동이 다른 유닛으로 번진다.
        for key in unit_terms():
            self.assertNotRegex(self.unit(key), r"(?m)^(Requires|BindsTo)=", key)

    def test_wheel_and_safety_units_are_never_restarted_automatically(self):
        for key in ("safety", "motor"):
            self.assertIn("Restart=no\n", self.unit(key), key)

    def test_motor_waits_for_safety_and_for_the_can_link(self):
        text = self.unit("motor")
        self.assertIn("After=vica-safety.service", text)
        self.assertRegex(text, r"ExecStartPre=\S+/bin/vica_wait\.sh canup can1 \d+")
        self.assertRegex(text, r"ExecStartPre=\S+/bin/vica_wait\.sh proc safety_supervis \d+")

    def test_safety_waits_for_the_can_interface_to_exist(self):
        # emergency_stop_node 는 CAN 소켓을 기동 때 한 번만 연다.
        self.assertRegex(
            self.unit("safety"),
            r"ExecStartPre=\S+/bin/vica_wait\.sh path /sys/class/net/can1 \d+",
        )

    def test_start_time_limit_covers_the_waits(self):
        text = self.unit("motor")
        waits = sum(int(n) for n in re.findall(r"vica_wait\.sh \S+ \S+ (\d+)", text))
        limit = int(re.search(r"TimeoutStartSec=(\d+)", text).group(1))
        self.assertGreater(limit, waits)

    def test_camera_stop_reaches_the_process_inside_the_container(self):
        text = self.unit("d455")
        pattern = re.escape('"' + self.terms["d455"].unit.stop_pattern + '"')
        self.assertIn("ExecStop=-/bin/kill -INT $MAINPID\n", text)
        self.assertRegex(text, rf"ExecStop=\S+/bin/vica_ctr_stop\.sh {pattern} \d+")
        self.assertRegex(text, rf"ExecStopPost=\S+/bin/vica_ctr_stop\.sh {pattern} \d+")

    def test_camera_stop_signals_the_node_itself_not_only_the_launch(self):
        # run_d455_cloud.sh 는 launch 를 `&` 로 띄운다. 터미널 없는 bash 는 그런 프로세스의
        # SIGINT 를 무시로 둔다 — launch 는 SIGINT 에 반응하지 않았고(SigIgn 0x2), SIGTERM 으로
        # 올리자 launch 만 죽고 노드가 고아로 남았다(2026-11-14 실기). 노드는 SIGINT 를 잡는다.
        pattern = self.terms["d455"].unit.stop_pattern
        launch = (
            "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch realsense2_camera"
            " rs_launch.py serial_no:='_114222251176' camera_name:=camera"
        )
        node = (
            "/workspaces/isaac_ros-dev/install/realsense2_camera/lib/realsense2_camera/"
            "realsense2_camera_node --ros-args --log-level info --ros-args -r __node:=camera"
        )
        self.assertRegex(launch, pattern)
        self.assertRegex(node, pattern)
        # 그 이름을 담기만 한 셸은 건드리지 않는다.
        self.assertNotRegex("/bin/bash -c pgrep -af realsense2_camera_node ", pattern)
        self.assertNotRegex("grep ros2 launch realsense2_camera", pattern)

    def test_stop_patterns_are_anchored_to_the_start(self):
        # 정지 패턴은 걸린 프로세스에 SIGINT → SIGTERM → SIGKILL 을 보낸다. 맨 앞에 묶지
        # 않으면 그 글자를 담은 다른 셸이 죽는다.
        for key, term in unit_terms().items():
            if term.unit.stop_pattern:
                self.assertTrue(term.unit.stop_pattern.startswith("^"), key)

    def test_guard_names_fit_in_a_comm(self):
        for key, term in unit_terms().items():
            for name in (*term.guard, *term.unit.guard):
                self.assertLessEqual(len(name), 15, f"{key}: {name}")


class Stages(unittest.TestCase):
    def keys(self, stage):
        return set(g.units_upto(g.build_terms(), stage))

    def test_first_stage_has_nothing_that_can_turn_a_wheel(self):
        self.assertEqual(self.keys("1a"), {"display", "lidar", "app", "web", "supervisor"})

    def test_second_stage_adds_the_camera_side(self):
        self.assertEqual(self.keys("1b") - self.keys("1a"), {"container", "d455", "imu"})

    def test_third_stage_adds_safety_and_motor(self):
        self.assertEqual(self.keys("1c") - self.keys("1b"), {"safety", "motor"})

    def test_target_wants_only_the_units_of_its_stage(self):
        target = g.render_target("base", sorted(self.keys("1b")))
        self.assertIn("vica-d455.service", target)
        self.assertNotIn("vica-motor.service", target)
        self.assertIn("WantedBy=default.target", target)

    def test_unknown_stage_is_rejected(self):
        with self.assertRaises(SystemExit):
            g.units_upto(g.build_terms(), "9z")


class DriveBundle(unittest.TestCase):
    """주행 묶음(2a). 부팅 때는 뜨지 않고, vica-drive.target 을 시작할 때만 뜬다."""

    DRIVE = {"nav2", "posecheck", "mission", "handle", "monitor"}

    def setUp(self):
        self.terms = g.build_terms()
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.rc_dir = self.tmp / "vica-terminator"
        self.sd_dir = self.tmp / "systemd-user"
        g.write_units(self.terms, "2a", ROS_WS, MAP_ID, self.rc_dir, self.sd_dir)

    def read(self, name):
        return (self.sd_dir / name).read_text(encoding="utf-8")

    def test_stage_2a_adds_the_drive_units(self):
        added = set(g.units_upto(self.terms, "2a")) - set(g.units_upto(self.terms, "1c"))
        self.assertEqual(added, self.DRIVE)

    def test_nothing_in_the_drive_bundle_starts_at_boot(self):
        base = self.read("vica-base.target")
        for key in self.DRIVE:
            self.assertNotIn(f"vica-{key}.service", base, key)
        self.assertNotIn("WantedBy", self.read("vica-drive.target"))

    def test_drive_target_pulls_its_units_and_needs_the_base_bundle(self):
        drive = self.read("vica-drive.target")
        for key in self.DRIVE:
            self.assertIn(f"vica-{key}.service", drive, key)
        self.assertIn("Requires=vica-base.target\n", drive)
        self.assertIn("After=vica-base.target\n", drive)

    def test_drive_units_stop_with_their_own_bundle_only(self):
        # 주행을 내릴 때 기본 묶음(motor 포함)은 건드리지 않는다.
        for key in self.DRIVE:
            text = self.read(f"vica-{key}.service")
            self.assertIn("PartOf=vica-drive.target\n", text, key)
            self.assertNotIn("PartOf=vica-base.target", text, key)
        self.assertIn("PartOf=vica-base.target\n", self.read("vica-motor.service"))

    def test_drive_units_start_after_the_base_bundle(self):
        for key in self.DRIVE:
            after = re.search(r"(?m)^After=(.*)$", self.read(f"vica-{key}.service")).group(1)
            self.assertIn("vica-base.target", after.split(), key)

    def test_units_that_move_the_robot_are_never_restarted_automatically(self):
        for key in ("nav2", "mission", "handle"):
            self.assertIn("Restart=no\n", self.read(f"vica-{key}.service"), key)

    def test_nav2_refuses_to_start_beside_another_odometry_source(self):
        # Nav2 와 SLAM 은 둘 다 EKF·encoder 를 띄운다. 겹치면 /odom 발행자가 둘이 된다.
        text = (self.rc_dir / "run_nav2.sh").read_text(encoding="utf-8")
        guard = re.search(r"(?m)^vica_guard (.*) \|\| exit 75$", text).group(1)
        for name in ("bt_navigator", "amcl", "ekf_node", "encoder_feedbac", "cartographer_no"):
            self.assertIn(f'"{name}"', guard)

    def test_mission_is_guarded_by_its_real_process_name(self):
        # 칸의 guard 'vica_mission_ma' 는 실제 이름과 다르다 — 노드 실행 파일이
        # mission_manager 라 comm 도 그렇다(2026-11-14 실기). 그 이름으로 막아야 두 벌이 안 뜬다.
        text = (self.rc_dir / "run_mission.sh").read_text(encoding="utf-8")
        guard = re.search(r"(?m)^vica_guard (.*) \|\| exit 75$", text).group(1)
        self.assertIn('"mission_manager"', guard)

    def test_nav2_gets_time_to_bring_its_nodes_down(self):
        stop = int(re.search(r"TimeoutStopSec=(\d+)", self.read("vica-nav2.service")).group(1))
        self.assertGreaterEqual(stop, 30)

    def test_mission_and_handle_wait_for_what_nav2_brings_up(self):
        mission = self.read("vica-mission.service")
        self.assertIn("vica-nav2.service", mission)
        self.assertRegex(mission, r"vica_wait\.sh proc bt_navigator \d+")
        self.assertRegex(self.read("vica-handle.service"), r"vica_wait\.sh proc ekf_node \d+")


class WriteUnits(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.rc_dir = self.tmp / "vica-terminator"
        self.sd_dir = self.tmp / "systemd-user"
        self.rc_dir.mkdir()

    def write(self, stage):
        return g.write_units(g.build_terms(), stage, ROS_WS, MAP_ID, self.rc_dir, self.sd_dir)

    def test_writes_scripts_units_target_and_helpers(self):
        self.write("1a")
        script = self.rc_dir / "run_display.sh"
        self.assertTrue(script.stat().st_mode & stat.S_IXUSR)
        self.assertTrue((self.sd_dir / "vica-display.service").is_file())
        self.assertTrue((self.sd_dir / "vica-base.target").is_file())
        for helper in ("vica_wait.sh", "vica_container.sh", "vica_ctr_stop.sh"):
            self.assertTrue((self.rc_dir / "bin" / helper).stat().st_mode & stat.S_IXUSR, helper)

    def test_pane_rc_files_are_left_exactly_as_they_were(self):
        # t_llm_local.rc 는 손으로 고친 판이다. 유닛을 만들다가 덮어쓰면 안 된다.
        rc = self.rc_dir / "t_llm_local.rc"
        rc.write_text("손으로 고친 내용\n", encoding="utf-8")
        self.write("1c")
        self.assertEqual(rc.read_text(encoding="utf-8"), "손으로 고친 내용\n")

    def test_rewriting_does_not_change_the_file_a_running_unit_is_reading(self):
        # bash 는 스크립트를 실행하면서 읽는다. 도는 중에 제자리에서 덮어쓰면 뒷부분을
        # 바뀐 파일에서 읽어 문법 오류로 끝난다(2026-11-14 실기). 새 파일로 바꿔 끼워야 한다.
        self.write("1a")
        script = self.rc_dir / "run_display.sh"
        with open(script, encoding="utf-8") as running:
            first_line = running.readline()
            script.write_text("#!/bin/bash\n# 옛 판이라고 치자\n", encoding="utf-8")
            with open(script, encoding="utf-8") as stale:
                self.write("1a")
                self.assertEqual(stale.read(), "#!/bin/bash\n# 옛 판이라고 치자\n")
            self.assertTrue(first_line.startswith("#!/bin/bash"))
        self.assertIn(g.GENERATED_MARK, script.read_text(encoding="utf-8"))

    def test_lowering_the_stage_removes_the_units_above_it(self):
        self.write("1c")
        self.assertTrue((self.sd_dir / "vica-motor.service").is_file())
        self.write("1a")
        self.assertFalse((self.sd_dir / "vica-motor.service").exists())
        self.assertFalse((self.rc_dir / "run_motor.sh").exists())

    def test_files_it_did_not_generate_are_kept(self):
        mine = self.sd_dir / "vica-custom.service"
        self.sd_dir.mkdir()
        mine.write_text("[Unit]\nDescription=사람이 만든 유닛\n", encoding="utf-8")
        self.write("1a")
        self.assertTrue(mine.is_file())


class CommandLine(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.saved = (g.RC_DIR, g.SYSTEMD_USER_DIR, g.TERMINATOR_CONFIG)
        g.RC_DIR = self.tmp / "vica-terminator"
        g.SYSTEMD_USER_DIR = self.tmp / "systemd-user"
        g.TERMINATOR_CONFIG = self.tmp / "terminator" / "config"
        self.addCleanup(self.restore)

    def restore(self):
        g.RC_DIR, g.SYSTEMD_USER_DIR, g.TERMINATOR_CONFIG = self.saved

    def test_units_need_an_explicit_stage(self):
        # 단계를 안 적으면 motor 까지 켜지는 일이 없게 한다.
        with self.assertRaises(SystemExit):
            g.main(["--units"])
        self.assertFalse(g.SYSTEMD_USER_DIR.exists())

    def test_units_do_not_write_pane_rc_or_terminator_config(self):
        self.assertEqual(g.main(["--units", "--stage", "1a"]), 0)
        self.assertTrue((g.SYSTEMD_USER_DIR / "vica-base.target").is_file())
        self.assertFalse(g.TERMINATOR_CONFIG.exists())
        self.assertEqual(list(g.RC_DIR.glob("t_*.rc")), [])

    def test_dry_run_writes_nothing(self):
        self.assertEqual(g.main(["--units", "--stage", "1c", "--dry-run"]), 0)
        self.assertFalse(g.SYSTEMD_USER_DIR.exists())
        self.assertFalse(g.RC_DIR.exists())


if __name__ == "__main__":
    unittest.main()
