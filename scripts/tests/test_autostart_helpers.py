"""유닛이 부르는 도우미 스크립트 — 대기, 컨테이너, 컨테이너 안 프로세스 정리.

`ip`·`docker` 는 PATH 앞에 둔 기록용 대역으로 바꾼다(진짜를 부르면 로봇 상태가 바뀐다).
프로세스·파일을 보는 부분은 진짜 프로세스와 임시 파일로 시험한다.
"""
import os
import shutil
import signal
import stat
import subprocess
import tempfile
import textwrap
import time
import unittest
from pathlib import Path

AUTOSTART = Path(__file__).resolve().parents[1] / "autostart"
WAIT = AUTOSTART / "vica_wait.sh"
CTR_STOP = AUTOSTART / "vica_ctr_stop.sh"
CONTAINER = AUTOSTART / "vica_container.sh"


class Sandbox(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.env = dict(os.environ, PATH=f"{self.tmp}:{os.environ['PATH']}")
        self.procs = []
        self.addCleanup(self._reap)

    def _reap(self):
        for proc in self.procs:
            if proc.poll() is None:
                proc.kill()
            proc.wait()

    def fake(self, name, body):
        path = self.tmp / name
        path.write_text("#!/bin/bash\n" + textwrap.dedent(body))
        path.chmod(path.stat().st_mode | stat.S_IEXEC)
        return path

    def spawn(self, args, **kwargs):
        proc = subprocess.Popen(args, **kwargs)
        self.procs.append(proc)
        return proc

    def run_sh(self, script, *args):
        return subprocess.run(
            ["bash", str(script), *map(str, args)],
            env=self.env, capture_output=True, text=True,
        )


class WaitForPath(Sandbox):
    def test_returns_at_once_when_the_path_exists(self):
        target = self.tmp / "dev"
        target.write_text("")
        started = time.monotonic()
        proc = self.run_sh(WAIT, "path", target, 5)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertLess(time.monotonic() - started, 1.0)

    def test_returns_when_the_path_appears_later(self):
        target = self.tmp / "late"
        self.spawn(["bash", "-c", f"sleep 0.6; touch {target}"])
        proc = self.run_sh(WAIT, "path", target, 5)
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_gives_up_after_the_time_limit(self):
        started = time.monotonic()
        proc = self.run_sh(WAIT, "path", self.tmp / "never", 1)
        self.assertNotEqual(proc.returncode, 0)
        self.assertGreaterEqual(time.monotonic() - started, 0.9)
        self.assertIn("never", proc.stderr)


class WaitForProcess(Sandbox):
    def test_finds_a_running_process_by_its_comm_name(self):
        binary = self.tmp / "vica_dummy_p"
        shutil.copy("/bin/sleep", binary)
        self.spawn([str(binary), "30"])
        time.sleep(0.2)
        self.assertEqual(self.run_sh(WAIT, "proc", "vica_dummy_p", 2).returncode, 0)

    def test_fails_when_no_such_process_shows_up(self):
        self.assertNotEqual(self.run_sh(WAIT, "proc", "vica_no_such_p", 1).returncode, 0)


class WaitForCanUp(Sandbox):
    def ip(self, flags):
        self.fake("ip", f"""\
            echo "6: can1: <{flags}> mtu 16 qdisc noop state X mode DEFAULT"
            """)

    def test_passes_when_the_link_is_up(self):
        self.ip("NOARP,UP,LOWER_UP,ECHO")
        self.assertEqual(self.run_sh(WAIT, "canup", "can1", 2).returncode, 0)

    def test_fails_when_the_link_stays_down(self):
        self.ip("NOARP,ECHO")
        self.assertNotEqual(self.run_sh(WAIT, "canup", "can1", 1).returncode, 0)


class WaitForContainer(Sandbox):
    def test_passes_when_docker_reports_the_container_running(self):
        self.fake("docker", 'echo true\n')
        self.assertEqual(self.run_sh(WAIT, "container", "vica_rs_container", 2).returncode, 0)

    def test_fails_when_the_container_is_not_running(self):
        self.fake("docker", 'echo false\n')
        self.assertNotEqual(self.run_sh(WAIT, "container", "vica_rs_container", 1).returncode, 0)


class WaitUsage(Sandbox):
    def test_unknown_kind_is_an_error_not_a_silent_pass(self):
        self.assertEqual(self.run_sh(WAIT, "nonsense", "x", 1).returncode, 2)


class ContainerProcessStop(Sandbox):
    MARK = "vica_marker_proc_qz7"

    def test_interrupts_the_process_matching_the_pattern(self):
        victim = self.spawn(["bash", "-c", f"exec -a {self.MARK} sleep 30"])
        time.sleep(0.2)
        proc = self.run_sh(CTR_STOP, self.MARK, 3)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(victim.wait(timeout=3), -signal.SIGINT)

    def test_does_not_signal_itself_though_its_own_command_line_matches(self):
        # 패턴이 인자로 들어가므로 이 스크립트의 명령줄도 패턴에 걸린다.
        self.spawn(["bash", "-c", f"exec -a {self.MARK} sleep 30"])
        time.sleep(0.2)
        proc = self.run_sh(CTR_STOP, self.MARK, 3)
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_escalates_when_the_process_ignores_the_interrupt(self):
        script = self.tmp / "stubborn.sh"
        script.write_text(f"trap '' INT\nexec -a {self.MARK} sleep 30\n")
        victim = self.spawn(["bash", str(script)])
        time.sleep(0.3)
        proc = self.run_sh(CTR_STOP, self.MARK, 1)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(victim.wait(timeout=5), -signal.SIGTERM)

    def test_nothing_to_stop_is_success(self):
        self.assertEqual(self.run_sh(CTR_STOP, "vica_marker_absent_q9", 1).returncode, 0)


class ContainerUnit(Sandbox):
    NAME = "vica_rs_container"

    def docker(self, running="", existing="", start_rc=0):
        self.log = self.tmp / "docker.log"
        self.log.write_text("")
        self.fake("docker", f"""\
            echo "$*" >> "{self.log}"
            case "$1 $2" in
              "ps -a") printf '%s\\n' {existing!r} ;;
              "ps --format") printf '%s\\n' {running!r} ;;
            esac
            [ "$1" = start ] && exit {start_rc}
            exit 0
            """)

    def calls(self):
        return [line for line in self.log.read_text().splitlines() if not line.startswith("ps ")]

    def test_attaches_to_a_container_that_is_already_running(self):
        self.docker(running=self.NAME, existing=self.NAME)
        self.run_sh(CONTAINER)
        self.assertEqual(self.calls(), [f"wait {self.NAME}"])

    def test_starts_an_exited_container_instead_of_recreating_it(self):
        self.docker(running="", existing=self.NAME)
        self.run_sh(CONTAINER)
        self.assertEqual(self.calls(), [f"start {self.NAME}", f"wait {self.NAME}"])

    def test_container_ending_by_itself_is_a_failure_so_the_unit_restarts(self):
        self.docker(running=self.NAME, existing=self.NAME)
        self.assertEqual(self.run_sh(CONTAINER).returncode, 70)

    def test_being_stopped_is_a_clean_exit(self):
        self.docker(running=self.NAME, existing=self.NAME)
        self.fake("docker", f"""\
            case "$1" in
              ps) echo {self.NAME} ;;
              wait) sleep 30 ;;
            esac
            """)
        proc = self.spawn(
            ["bash", str(CONTAINER)], env=self.env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.addCleanup(lambda: os.killpg(proc.pid, signal.SIGKILL) if proc.poll() is None else None)
        time.sleep(0.5)
        os.killpg(proc.pid, signal.SIGINT)   # systemd 는 묶음 전체에 SIGINT 를 보낸다
        self.assertEqual(proc.wait(timeout=5), 0)

    def test_creates_the_container_detached_when_there_is_none(self):
        self.docker(running="", existing="")
        self.run_sh(CONTAINER)
        run, wait = self.calls()
        self.assertTrue(run.startswith("run -d "), run)
        for needed in (
            f"--name {self.NAME}", "--net=host", "--ipc=host", "--runtime nvidia",
            "ROS_DOMAIN_ID=7", ":/workspaces/isaac_ros-dev",
            "vica_isaac_ros_realsense_2563:latest",
        ):
            self.assertIn(needed, run)
        self.assertNotIn("DISPLAY", run)
        self.assertEqual(wait, f"wait {self.NAME}")

    def test_reports_failure_when_the_container_will_not_start(self):
        self.docker(running="", existing=self.NAME, start_rc=1)
        proc = self.run_sh(CONTAINER)
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn(f"wait {self.NAME}", self.calls())


class StackStop(unittest.TestCase):
    """vica_stack_stop.sh 는 돌리면 로봇의 실제 프로세스를 내린다. 여기서는 돌리지 않고
    순서만 본다 — 유닛을 두고 프로세스만 죽이면 Restart 가 걸린 유닛이 되살아난다."""

    SCRIPT = AUTOSTART.parent / "vica_stack_stop.sh"

    def test_script_parses(self):
        proc = subprocess.run(["bash", "-n", str(self.SCRIPT)], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_units_are_stopped_before_any_process_is_signalled(self):
        text = self.SCRIPT.read_text(encoding="utf-8")
        stop = text.index("systemctl --user stop")
        self.assertLess(stop, text.index("send INT"))
        self.assertIn("vica-base.target", text[stop:text.index("\n", stop)])


if __name__ == "__main__":
    unittest.main()
