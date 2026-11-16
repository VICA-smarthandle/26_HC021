"""터미네이터 생성기의 systemd 유닛 출력(`--units`).

칸과 유닛은 같은 Term 정의에서 나온다. 여기서 지키는 것:

- 유닛을 더해도 칸의 rc 는 한 글자도 달라지지 않는다
- 실행 스크립트의 끝나는 값(0 정지 · 70 스스로 끝남 · 75 중복 거부)
- 터미널이 있어야 도는 명령(-it, sudo, history)이 유닛에 들어가지 않는다
- 바퀴가 도는 유닛은 단계를 올려야만 묶음에 들어온다
"""
import copy
import json
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


class SafetyOverUdpOnly(unittest.TestCase):
    """safety·motor 는 FastDDS 공유메모리를 쓰지 않는다.

    2026-11-14 주행 묶음을 내릴 때 safety_supervisor_node 가 영구히 멈췄다. 스택:
    SharedMemTransport::push_discard 가 이미 끝난 Nav2 프로세스의 공유메모리 포트
    (/dev/shm/fastrtps_port9203, deleted) 잠금을 기다리며 writer 잠금을 쥐었고, 주 스레드가
    /cmd_vel_safe 발행에서 그 잠금을 기다렸다(~/vica_data/safety_deadlock_1114.txt).
    UDP 전송은 받는 쪽 잠금을 잡지 않는다.
    """

    UDP = 'export FASTRTPS_DEFAULT_PROFILES_FILE="$VICA_BIN/fastdds_udp_only.xml"'

    def setUp(self):
        self.terms = g.build_terms()

    def test_safety_and_motor_use_the_udp_only_profile(self):
        for key in ("safety", "motor"):
            self.assertIn(self.UDP, run_script(key, self.terms[key]), key)

    def test_no_other_unit_gives_up_shared_memory(self):
        # 카메라·Nav2 같은 큰 데이터는 공유메모리가 CPU 를 아낀다.
        for key, term in unit_terms().items():
            if key not in ("safety", "motor"):
                self.assertNotIn("FASTRTPS_DEFAULT_PROFILES_FILE", run_script(key, term), key)

    def test_profile_is_set_before_any_node_starts(self):
        text = run_script("safety", self.terms["safety"])
        self.assertLess(text.index(self.UDP), text.index("ros2 launch vica_safety"))

    def test_profile_has_only_udp_and_no_builtin_shared_memory(self):
        import xml.etree.ElementTree as ET
        ns = {"p": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(g.AUTOSTART_SRC / "fastdds_udp_only.xml").getroot()
        types = [t.text for t in root.findall(".//p:transport_descriptor/p:type", ns)]
        self.assertEqual(types, ["UDPv4"])
        participant = root.find(".//p:participant", ns)
        self.assertEqual(participant.get("is_default_profile"), "true")
        self.assertEqual(participant.find(".//p:useBuiltinTransports", ns).text, "false")

    def test_profile_is_copied_beside_the_scripts(self):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        g.write_units(self.terms, "1c", ROS_WS, MAP_ID, tmp / "rc", tmp / "sd")
        self.assertTrue((tmp / "rc" / "bin" / "fastdds_udp_only.xml").is_file())


class DiscoveryServer(unittest.TestCase):
    """--discovery-server: 노드들이 서로를 직접 찾지 않고 서버 하나를 거쳐 찾는다.

    2026-11-16 실측: 주행 묶음을 켤 때마다 발견 메시지가 82만~139만 건 버려졌고(수신 버퍼 4 MB 여도),
    그 회차에 map_server 가 'failed to send response ... client will not receive response' 로
    응답을 못 보내 Nav2 위치추정 관리자가 3분 멈췄다. 옵션이 없으면 지금과 똑같아야 한다.
    """

    SERVER = 'export ROS_DISCOVERY_SERVER="127.0.0.1:11811"'
    SUPER = 'export FASTRTPS_DEFAULT_PROFILES_FILE="$VICA_BIN/fastdds_super_client.xml"'

    def setUp(self):
        self.terms = g.build_terms()
        self.on = g.units_upto(self.terms, "2b", discovery=True)

    def script(self, key, discovery=True):
        return g.render_run_script(key, self.terms[key], ROS_WS, MAP_ID, discovery=discovery)

    def test_server_unit_exists_only_when_asked(self):
        self.assertIn("discovery", self.on)
        self.assertEqual(self.on["discovery"].unit.bundle, "base")
        self.assertNotIn("discovery", g.units_upto(self.terms, "2b"))

    def test_server_listens_on_localhost_11811(self):
        cmd = self.terms["discovery"].command
        self.assertIn("fast-discovery-server", cmd)
        self.assertIn("-i 0", cmd)
        self.assertIn("-l 127.0.0.1", cmd)
        self.assertIn("-p 11811", cmd)

    def test_every_client_points_at_the_server(self):
        for key in self.on:
            if key == "discovery":
                continue
            self.assertIn(self.SERVER, self.script(key), key)
        self.assertNotIn("ROS_DISCOVERY_SERVER", self.script("discovery"))

    def test_nothing_changes_without_the_option(self):
        for key in g.units_upto(self.terms, "2b"):
            self.assertNotIn("ROS_DISCOVERY_SERVER", self.script(key, discovery=False), key)
            self.assertNotIn("vica-discovery", g.render_unit(key, self.terms[key], Path("/x")), key)

    def test_container_process_gets_the_server_too(self):
        # docker exec 는 호스트 환경을 넘기지 않는다. 명령에 직접 실어야 한다.
        self.assertIn("docker exec -e ROS_DISCOVERY_SERVER=127.0.0.1:11811 vica_rs_container",
                      self.script("d455"))
        self.assertNotIn("docker exec -e", self.script("d455", discovery=False))

    def test_graph_watchers_see_every_node(self):
        # 서버는 같은 토픽을 쓰는 노드끼리만 짝짓는다. 전체 노드 목록이 필요한 rosapi(앱의
        # 중복 실행 감지)와 상태 감시 노드는 super client 여야 한다.
        for key in ("app", "monitor"):
            self.assertIn(self.SUPER, self.script(key), key)
        for key in ("display", "nav2", "mission", "mode"):
            self.assertNotIn("fastdds_super_client", self.script(key), key)
        # safety·motor 는 UDP 전용 프로파일을 그대로 쓴다(환경 변수로 서버를 받는다).
        self.assertIn("fastdds_udp_only.xml", self.script("safety"))
        self.assertIn(self.SERVER, self.script("safety"))

    def test_super_client_profile_points_at_server_zero(self):
        import xml.etree.ElementTree as ET
        ns = {"p": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(g.AUTOSTART_SRC / "fastdds_super_client.xml").getroot()
        self.assertEqual(root.find(".//p:discoveryProtocol", ns).text, "SUPER_CLIENT")
        server = root.find(".//p:RemoteServer", ns)
        self.assertEqual(server.get("prefix"), "44.53.00.5f.45.50.52.4f.53.49.4d.41")
        self.assertEqual(server.find(".//p:address", ns).text, "127.0.0.1")
        self.assertEqual(server.find(".//p:port", ns).text, "11811")

    def test_clients_start_after_the_server(self):
        unit = g.render_unit("display", self.terms["display"], Path("/x"), discovery=True)
        self.assertRegex(unit, r"(?m)^After=.*vica-discovery\.service")
        self.assertNotIn("After=vica-discovery", g.render_unit(
            "discovery", self.terms["discovery"], Path("/x"), discovery=True))

    def test_command_line_option(self):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        saved = (g.RC_DIR, g.SYSTEMD_USER_DIR)
        g.RC_DIR, g.SYSTEMD_USER_DIR = tmp / "rc", tmp / "sd"
        self.addCleanup(lambda: setattr(g, "RC_DIR", saved[0]) or setattr(g, "SYSTEMD_USER_DIR", saved[1]))
        self.assertEqual(g.main(["--units", "--stage", "2b", "--discovery-server"]), 0)
        self.assertTrue((tmp / "sd" / "vica-discovery.service").is_file())
        self.assertIn("ROS_DISCOVERY_SERVER", (tmp / "rc" / "run_nav2.sh").read_text())
        self.assertTrue((tmp / "rc" / "bin" / "fastdds_super_client.xml").is_file())
        # 옵션 없이 다시 만들면 서버 유닛이 사라진다(되돌리기).
        self.assertEqual(g.main(["--units", "--stage", "2b"]), 0)
        self.assertFalse((tmp / "sd" / "vica-discovery.service").exists())
        self.assertNotIn("ROS_DISCOVERY_SERVER", (tmp / "rc" / "run_nav2.sh").read_text())


class Nav2BondTimeout(unittest.TestCase):
    """유닛으로 띄우는 Nav2 는 lifecycle 관리자의 bond_timeout 을 4 s → 8 s 로 늘린다(사용자 11-16).

    2026-11-14·16 실기: controller_server 등이 bond 4.00 s 안에 연결되지 않아 기동이 중단됐다
    (원래 방식 14번 중 1번). 정상 회차에서도 연결에 5.9 s 걸린 적이 있다. 이 값은 nav2_params.yaml
    로는 안 바뀐다(nav2_bringup 이 관리자에게 파일을 넘기지 않음) — 감싸는 launch 로 얹는다.
    """

    def setUp(self):
        self.terms = g.build_terms()

    def test_unit_runs_the_wrapper_with_the_same_arguments(self):
        text = run_script("nav2", self.terms["nav2"])
        self.assertIn('ros2 launch "$VICA_BIN/nav2_bond.launch.py" map:=$VICA_MAP_YAML use_route:=true', text)
        # 칸은 그대로다.
        self.assertIn("ros2 launch vica_nav2 nav2_map_test.launch.py", self.terms["nav2"].command)

    def test_wrapper_is_copied_beside_the_scripts(self):
        self.assertIn("nav2_bond.launch.py", g.HELPER_SCRIPTS)
        self.assertTrue((g.AUTOSTART_SRC / "nav2_bond.launch.py").is_file())

    def test_wrapper_sets_bond_timeout_before_including_nav2(self):
        text = (g.AUTOSTART_SRC / "nav2_bond.launch.py").read_text(encoding="utf-8")
        self.assertIn("BOND_TIMEOUT_SEC = 8.0", text)
        self.assertIn('SetParameter(name="bond_timeout", value=BOND_TIMEOUT_SEC)', text)
        self.assertLess(text.index("SetParameter(name="), text.index("IncludeLaunchDescription("))
        self.assertIn('"nav2_map_test.launch.py"', text)


class ModeNodeUnit(unittest.TestCase):
    """모드 노드(2b). 부팅 때 뜨고, 앱이 부르면 주행 묶음을 켜고 끈다."""

    def setUp(self):
        self.terms = g.build_terms()
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.rc_dir = self.tmp / "vica-terminator"
        self.sd_dir = self.tmp / "systemd-user"
        g.write_units(self.terms, "2b", ROS_WS, MAP_ID, self.rc_dir, self.sd_dir)

    def test_stage_2b_adds_only_the_mode_node(self):
        added = set(g.units_upto(self.terms, "2b")) - set(g.units_upto(self.terms, "2a"))
        self.assertEqual(added, {"mode"})

    def test_mode_node_starts_at_boot_and_comes_back_if_it_dies(self):
        self.assertIn("vica-mode.service", (self.sd_dir / "vica-base.target").read_text())
        self.assertIn("Restart=on-failure\n", (self.sd_dir / "vica-mode.service").read_text())

    def test_mode_node_and_its_logic_are_copied_beside_the_scripts(self):
        for name in ("vica_mode_node.py", "mode_logic.py"):
            self.assertTrue((self.rc_dir / "bin" / name).is_file(), name)
        self.assertIn('"$VICA_BIN/vica_mode_node.py"', (self.rc_dir / "run_mode.sh").read_text())

    def test_manifest_lists_each_unit_with_the_names_that_prove_it_alive(self):
        manifest = json.loads((self.rc_dir / "units.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["nav2"]["unit"], "vica-nav2.service")
        self.assertEqual(manifest["nav2"]["bundle"], "drive")
        self.assertIn("ekf_node", manifest["nav2"]["guard"])
        self.assertIn("mission_manager", manifest["mission"]["guard"])
        self.assertTrue(manifest["app"]["guard_cmdline"].startswith("^"))
        self.assertEqual(manifest["motor"]["bundle"], "base")
        self.assertNotIn("teleop", manifest)

    def test_manifest_matches_what_the_mode_logic_reads(self):
        from scripts.autostart import mode_logic as ml
        manifest = json.loads((self.rc_dir / "units.json").read_text(encoding="utf-8"))
        idle = ml.Snapshot(
            units={e["unit"]: ("active", "running", "success")
                   for e in manifest.values() if e["bundle"] == "base"},
            procs={}, can_up=True,
        )
        # 기본 묶음 유닛이 다 떠 있어도 노드 프로세스가 없으면 그 사실이 보여야 한다.
        status = ml.derive_status(idle, manifest)
        self.assertIn("motor", status["missing_nodes"])


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
