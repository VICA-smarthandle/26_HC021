"""부팅 때 can1 을 올리는 스크립트와 system 유닛.

진짜 `ip` 를 부르면 로봇의 CAN 을 건드리므로, PATH 앞에 기록만 하는 `ip` 를 둔다.
보는 것은 스크립트가 어떤 `ip` 명령을 내보내는가다.
"""
import os
import stat
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path

AUTOSTART = Path(__file__).resolve().parents[1] / "autostart"
CAN_UP = AUTOSTART / "vica_can_up.sh"
CAN_UNIT = AUTOSTART / "vica-can.service"
INSTALL = AUTOSTART / "install.sh"

DOWN = textwrap.dedent("""\
    6: can1: <NOARP,ECHO> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
        link/can  promiscuity 0 minmtu 0 maxmtu 0
        can state STOPPED (berr-counter tx 0 rx 0) restart-ms 0
    """)


def up(bitrate):
    return textwrap.dedent(f"""\
        6: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
            link/can  promiscuity 0 minmtu 0 maxmtu 0
            can <BERR-REPORTING> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
        	  bitrate {bitrate} sample-point 0.875
        """)


FAKE_IP = textwrap.dedent("""\
    #!/bin/bash
    echo "$*" >> "$IP_LOG"
    case "$*" in
      *"link show"*)
        [ -n "$IP_SHOW_FAIL" ] && { echo "Device \\"can1\\" does not exist." >&2; exit 1; }
        cat "$IP_SHOW_FILE"; exit 0 ;;
      *"type can"*) exit "${IP_TYPE_RC:-0}" ;;
    esac
    exit 0
    """)


class CanUpScript(unittest.TestCase):
    def run_script(self, show_text, **env_extra):
        tmp = Path(tempfile.mkdtemp())
        self.addCleanup(lambda: subprocess.run(["rm", "-rf", str(tmp)]))
        fake = tmp / "ip"
        fake.write_text(FAKE_IP)
        fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
        (tmp / "show.txt").write_text(show_text)
        log = tmp / "ip.log"
        log.write_text("")
        env = dict(os.environ)
        env.update(
            PATH=f"{tmp}:{env['PATH']}",
            IP_LOG=str(log),
            IP_SHOW_FILE=str(tmp / "show.txt"),
            **env_extra,
        )
        proc = subprocess.run(
            ["bash", str(CAN_UP), "can1"], env=env, capture_output=True, text=True
        )
        calls = [line for line in log.read_text().splitlines() if line]
        sets = [c for c in calls if c.startswith("link set")]
        return proc, sets

    def test_down_link_is_configured_then_brought_up(self):
        proc, sets = self.run_script(DOWN)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(
            sets,
            [
                "link set can1 type can bitrate 500000 berr-reporting on restart-ms 100",
                "link set can1 up",
            ],
        )

    def test_link_already_up_at_500000_is_left_alone(self):
        proc, sets = self.run_script(up(500000))
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(sets, [])

    def test_link_up_at_other_bitrate_is_refused_without_touching_it(self):
        proc, sets = self.run_script(up(1000000))
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(sets, [])
        self.assertIn("1000000", proc.stderr)

    def test_missing_interface_fails_without_touching_it(self):
        proc, sets = self.run_script(DOWN, IP_SHOW_FAIL="1")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(sets, [])

    def test_link_is_not_brought_up_when_bitrate_setting_fails(self):
        proc, sets = self.run_script(DOWN, IP_TYPE_RC="2")
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn("link set can1 up", sets)

    def test_never_takes_the_link_down(self):
        # 드라이버에 전원이 들어간 뒤 CAN 이 내려가면 전원을 다시 넣어야 복구된다.
        for text in (DOWN, up(500000), up(1000000)):
            _, sets = self.run_script(text)
            self.assertFalse([c for c in sets if "down" in c.split()], sets)


class CanUnit(unittest.TestCase):
    def setUp(self):
        self.text = CAN_UNIT.read_text(encoding="utf-8")

    def test_runs_once_when_the_can1_device_appears(self):
        device = "sys-subsystem-net-devices-can1.device"
        self.assertIn(f"BindsTo={device}", self.text)
        self.assertIn(f"After={device}", self.text)
        self.assertIn(f"WantedBy={device}", self.text)
        self.assertIn("Type=oneshot", self.text)
        self.assertIn("RemainAfterExit=yes", self.text)

    def test_stopping_the_unit_does_nothing_to_the_link(self):
        self.assertNotIn("ExecStop", self.text)

    def test_does_not_hold_up_boot_when_the_device_is_absent(self):
        self.assertNotIn("multi-user.target", self.text)


class Installer(unittest.TestCase):
    def test_script_parses(self):
        for path in (INSTALL, AUTOSTART / "uninstall.sh"):
            proc = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_can_unit_is_enabled_for_next_boot_not_started_now(self):
        # 설치하는 순간 링크 상태를 바꾸지 않는다. 다음 부팅부터 적용한다.
        text = INSTALL.read_text(encoding="utf-8")
        self.assertIn("systemctl enable vica-can.service", text)
        self.assertNotIn("--now", text)
        self.assertNotIn("systemctl start vica-can", text)

    def test_user_manager_may_raise_safety_priority(self):
        # safety·motor 실행 스크립트의 renice -10 은 사용자 관리자에 nice 한도가 있어야 된다
        # (2026-11-16 사용자 결정 B). user@.service 의 LimitNICE= 는 효과가 없었다 — 2026-11-16
        # 재부팅 뒤 systemctl show 는 30 인데 관리자 프로세스는 0. 관리자를 띄우는 PAM
        # (/usr/lib/pam.d/systemd-user 의 pam_limits)이 덮는다. 그래서 limits.d 에 둔다.
        text = INSTALL.read_text(encoding="utf-8")
        self.assertIn("/etc/security/limits.d/vica-nice.conf", text)
        line = next(l for l in text.splitlines() if "limits.d/vica-nice.conf" in l and "printf" in l)
        made = subprocess.run(["bash", "-c", "ROBOT_USER=ji_w; " + line.split(" > ")[0]],
                              capture_output=True, text=True).stdout
        self.assertEqual(made.split(), ["ji_w", "-", "nice", "-10"])
        self.assertNotIn("restart user@", text)
        # 효과 없던 앞 판의 drop-in 은 지운다(남아 있으면 systemctl show 가 30 이라 헷갈린다).
        self.assertIn("rm -f \"/etc/systemd/system/user@${ROBOT_UID}.service.d/vica-nice.conf\"", text)
        undo = (AUTOSTART / "uninstall.sh").read_text(encoding="utf-8")
        self.assertIn("/etc/security/limits.d/vica-nice.conf", undo)

    def test_installer_never_turns_autostart_on(self):
        # 자동 기동 켜기·끄기는 사용자 유닛(vica-base.target) 몫이다(11-14 사용자 "우선 꺼주세요").
        text = INSTALL.read_text(encoding="utf-8")
        self.assertNotIn("vica-base.target", text.replace("# ", "").split("설치 끝")[0])


if __name__ == "__main__":
    unittest.main()
