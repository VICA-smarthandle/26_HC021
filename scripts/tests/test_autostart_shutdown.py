"""앱 '로봇 끄기'(사용자 2026-11-18) — 끄는 스크립트와 종료 권한 설치 스크립트.

- 주행 묶음 → 기본 묶음 → sync → 시스템 종료 순서
- 묶음 정지가 실패해도 시스템 종료까지 간다(set -e 없음)
- 모드 노드가 부르는 스크립트가 생성기 도우미 목록에 들어 있다(bin 사본)
- 종료 권한은 polkit 0.105 .pkla 형식, 종료 동작 세 가지
"""
import re
import unittest
from pathlib import Path

from scripts import vica_terminator_layout as g

HERE = Path(__file__).resolve().parents[1] / "autostart"
SCRIPT = (HERE / "vica_shutdown.sh").read_text(encoding="utf-8")
RULE = (HERE / "install_poweroff_rule.sh").read_text(encoding="utf-8")


def code_lines(text):
    return [line for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]


class ShutdownScript(unittest.TestCase):
    def test_order_is_drive_base_sync_poweroff(self):
        body = "\n".join(code_lines(SCRIPT))
        steps = [body.index(s) for s in ("stop vica-drive.target", "stop vica-base.target", "\nsync", "systemctl poweroff")]
        self.assertEqual(steps, sorted(steps))

    def test_keeps_going_when_a_stop_fails(self):
        self.assertNotRegex("\n".join(code_lines(SCRIPT)), r"set -[a-z]*e")

    def test_never_kills_by_force(self):
        self.assertNotIn("kill -9", SCRIPT)
        self.assertNotIn("SIGKILL", SCRIPT)

    def test_mode_node_calls_the_bin_copy(self):
        self.assertIn("vica_shutdown.sh", g.HELPER_SCRIPTS)
        node = (HERE / "vica_mode_node.py").read_text(encoding="utf-8")
        self.assertIn('"/vica/mode/shutdown"', node)
        self.assertIn('HERE / "vica_shutdown.sh"', node)
        self.assertRegex(node, r'"systemd-run", "--user"')


class PoweroffRule(unittest.TestCase):
    def test_pkla_grants_poweroff_only(self):
        actions = re.search(r"^Action=(.*)$", RULE, re.M).group(1).split(";")
        self.assertEqual(sorted(actions), sorted([
            "org.freedesktop.login1.power-off",
            "org.freedesktop.login1.power-off-multiple-sessions",
            "org.freedesktop.login1.power-off-ignore-inhibit",
        ]))
        self.assertIn("ResultAny=yes", RULE)
        self.assertIn("/etc/polkit-1/localauthority/50-local.d/", RULE)


if __name__ == "__main__":
    unittest.main()
