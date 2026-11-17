"""대기(HOLD) 칸의 ↑+Enter 순간 중복 검사(2026-11-17).

칸을 열 때 한 번 하던 vica_guard 검사만으로는, 칸을 열어 둔 뒤 앱으로 주행을 켜고 그 칸에서
↑+Enter 를 누르면 두 벌이 떴다. history 에 넣는 명령 앞에도 같은 검사를 붙인다.

- guard 가 거부하면 명령은 한 글자도 돌지 않는다(명령 안의 `a && b || c` 도 마찬가지)
- 화면에 보여 주는 대기 명령은 예전 그대로다
- guard 가 없는 칸은 history 줄이 예전 그대로다
- 렌더한 rc 는 모두 bash 문법 검사를 통과한다
"""
import shlex
import subprocess
import unittest

from scripts import vica_terminator_layout as g

ROS_WS = g.DEFAULT_ROS_WS
MAP_ID = g.DEFAULT_MAP_ID


def history_command(rc: str) -> str:
    lines = [line for line in rc.splitlines() if line.startswith("history -s ")]
    assert len(lines) == 1, lines
    return shlex.split(lines[0])[2]


def run(command: str, guard_ok: bool) -> str:
    """vica_guard 를 흉내 내고 history 명령을 그대로 돌린다. 찍힌 것을 돌려준다."""
    script = f"vica_guard() {{ return {0 if guard_ok else 1}; }}\n{command}\n"
    return subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=10).stdout


class HoldGuardTest(unittest.TestCase):
    def setUp(self):
        self.terms = g.build_terms()

    def test_guarded_hold_panes_check_again_at_enter(self):
        guarded = [k for k, t in self.terms.items() if t.mode == g.HOLD and t.guard]
        self.assertIn("nav2", guarded)
        self.assertIn("mission", guarded)
        for key in guarded:
            term = self.terms[key]
            command = history_command(g.render_rc(term, ROS_WS, MAP_ID))
            names = " ".join(f'"{n}"' for n in term.guard)
            self.assertTrue(command.startswith(f"vica_guard {names} && {{ "), key)
            self.assertIn(term.command, command, key)

    def test_refused_guard_runs_nothing(self):
        for command in ("echo RAN", "false && echo A || echo RAN", "echo RAN; echo RAN2"):
            term = g.Term(title="t", note=(), command=command, mode=g.HOLD, guard=("bt_navigator",))
            line = history_command(g.render_rc(term, ROS_WS, MAP_ID))
            self.assertEqual(run(line, guard_ok=False), "", command)
            self.assertIn("RAN", run(line, guard_ok=True), command)

    def test_voice_dup_check_still_refuses_after_guard(self):
        # ⑫ 는 명령 안에 pgrep 검사(`a && echo [중복] || 실행`)가 있다. guard 를 통과해도 그 검사가 돈다.
        line = history_command(g.render_rc(self.terms["llm"], ROS_WS, MAP_ID))
        self.assertIn("pgrep -f", line)
        self.assertEqual(run(line.replace("ros2 launch", "echo RAN"), guard_ok=False), "")

    def test_shown_command_is_unchanged(self):
        term = self.terms["nav2"]
        rc = g.render_rc(term, ROS_WS, MAP_ID)
        self.assertIn(f"printf '  %s\\n\\n' {g.shell_quote(term.command)}", rc)

    def test_unguarded_hold_pane_is_unchanged(self):
        term = self.terms["teleop"]
        self.assertFalse(term.guard)
        self.assertEqual(history_command(g.render_rc(term, ROS_WS, MAP_ID)), term.command)

    def test_every_rc_parses(self):
        for key, term in self.terms.items():
            rc = g.render_rc(term, ROS_WS, MAP_ID)
            result = subprocess.run(["bash", "-n"], input=rc, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, f"{key}: {result.stderr}")


if __name__ == "__main__":
    unittest.main()
