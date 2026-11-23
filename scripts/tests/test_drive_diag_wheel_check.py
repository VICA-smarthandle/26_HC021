"""drive_diag.wheel_check — 직진 명령 rpm ↔ 엔코더(C5) 비교 · 한 바퀴 틱."""
import math
import os
import struct
import unittest
from types import SimpleNamespace

from scripts.drive_diag import wheel_check as wc


def cf(r, l):
    return bytes([0xCF, 1]) + struct.pack("<h", r) + bytes([1]) + struct.pack("<h", l) + bytes([5])


def c5(m, pos):
    return bytes([0xC5, m]) + struct.pack("<i", pos)


def wrap32(v):
    return (v + 0x80000000) % 0x100000000 - 0x80000000


def f1(knob):
    return bytes([0xF1, 0, 0x38, 0x58, 0x04, 0x05, knob, knob])


def simulate(plan, true_tpr, start=(17, -56), t0=1000.0, dt=0.05, tau=0.2, driver_uses_knob=False):
    """plan = [(초, R rpm, L rpm[, 가변저항 %]), ...] → candump -L 줄 목록."""
    lines = []
    t = t0
    speed = [0.0, 0.0]
    cont = [float(start[0]), float(start[1])]
    for step in plan:
        secs, r, l = step[:3]
        knob = step[3] if len(step) > 3 else 98
        gain = knob / 100.0 if driver_uses_knob else 1.0
        for _ in range(int(round(secs / dt))):
            lines.append(wc.format_line(t, "can1", 0x001, cf(r, l)))
            lines.append(wc.format_line(t + 0.0001, "can1", 0x701, f1(knob)))
            lines.append(wc.format_line(t + 0.0002, "can1", 0x701, bytes.fromhex("F10106010A660000")))
            for m, c in ((0, r), (1, l)):
                speed[m] += (c * gain - speed[m]) * (1 - math.exp(-dt / tau))
                cont[m] += speed[m] / 60.0 * true_tpr * dt
                lines.append(wc.format_line(t + 0.0003, "can1", 0x701, c5(m, wrap32(math.floor(cont[m])))))
            t += dt
    return lines


def trace_of(lines):
    tr = wc.Tracker()
    for line in lines:
        p = wc.parse_line(line)
        if p:
            tr.feed(*p)
    return tr


def args(**kw):
    base = dict(min_s=2.0, skip_s=0.7, tol_rpm=3, revs=None, revs_r=None, revs_l=None, tape_m=None, run=None,
                knob_min=55, knob_max=98, deadzone=5, max_linear=1.0)
    base.update(kw)
    return SimpleNamespace(**base)


class TestDecode(unittest.TestCase):
    def test_cf_positive_negative(self):
        self.assertEqual(wc.decode_cf(cf(44, 44)), (44, 44))
        self.assertEqual(wc.decode_cf(cf(-44, 10)), (-44, 10))
        self.assertEqual(wc.decode_cf(bytes.fromhex("CF01000001000005")), (0, 0))

    def test_c5_real_frames(self):
        self.assertEqual(wc.decode_c5(bytes.fromhex("C50011000000")), (0, 17))
        self.assertEqual(wc.decode_c5(bytes.fromhex("C501C8FFFFFF")), (1, -56))
        self.assertIsNone(wc.decode_c5(bytes.fromhex("F100181803036262")))

    def test_parse_line(self):
        self.assertEqual(wc.parse_line("(1791863761.413526) can1 001#CF01000001000005"),
                         (1791863761.413526, 0x001, bytes.fromhex("CF01000001000005")))
        self.assertIsNone(wc.parse_line("잡음"))
        line = wc.format_line(5.25, "can1", 0x701, c5(1, -3))
        self.assertEqual(wc.parse_line(line), (5.25, 0x701, c5(1, -3)))

    def test_int32_rollover(self):
        self.assertEqual(wc.int32_delta(-0x7FFFFFF0, 0x7FFFFFF0), 0x20)
        self.assertEqual(wc.int32_delta(0x7FFFFFF0, -0x7FFFFFF0), -0x20)


class TestRatio(unittest.TestCase):
    PLAN = [(3.0, 0, 0), (5.0, 44, 44), (3.0, 0, 0)]

    def _agg(self, true_tpr):
        tr = trace_of(simulate(self.PLAN, true_tpr))
        rows = wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2)
        self.assertEqual(len(rows), 1)
        return tr, rows, wc.straight_summary(rows)

    def test_matching_tpr_gives_ratio_one(self):
        _, rows, agg = self._agg(61.2)
        for m in (0, 1):
            self.assertAlmostEqual(agg[m]["ratio"], 1.0, delta=0.01)
            self.assertAlmostEqual(agg[m]["tpr_speed"], 61.2, delta=0.6)
            self.assertEqual(rows[0][m]["cmd"], 44)

    def test_true_605_gives_0988(self):
        _, _, agg = self._agg(60.5)
        for m in (0, 1):
            self.assertAlmostEqual(agg[m]["ratio"], 60.5 / 61.2, delta=0.01)
            self.assertAlmostEqual(agg[m]["tpr_speed"], 60.5, delta=0.6)

    def test_lag_excluded_by_skip(self):
        tr = trace_of(simulate(self.PLAN, 61.2))
        no_skip = wc.straight_summary(wc.segment_rows(tr, wc.steady_segments(tr.cmd, skip_s=0.0), 61.2))
        skip = wc.straight_summary(wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2))
        self.assertLess(no_skip[0]["ratio"], skip[0]["ratio"])

    def test_reverse_and_rotation(self):
        tr = trace_of(simulate([(1.0, 0, 0), (3.0, -30, -30), (1.0, 0, 0), (3.0, 10, -10), (1.0, 0, 0)], 61.2))
        rows = wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2)
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0][0]["ratio"], 1.0, delta=0.02)
        agg = wc.straight_summary(rows)
        self.assertEqual(agg[0]["n"], 1)
        runs = wc.motion_runs(tr)
        self.assertEqual([r.direction for r in runs], ["후진", "회전"])


class TestRunsAndMeasured(unittest.TestCase):
    def test_run_delta_and_revs(self):
        tr = trace_of(simulate([(3.0, 0, 0), (5.0, 44, 44), (3.0, 0, 0)], 60.5))
        runs = wc.motion_runs(tr)
        self.assertEqual(len(runs), 1)
        revs = 44 / 60 * 5
        for m in (0, 1):
            self.assertAlmostEqual(runs[0].delta[m], revs * 60.5, delta=2)
        lines = wc.compare_measured(runs[0].delta, 61.2, 0.065, {0: revs, 1: revs}, revs * 2 * math.pi * 0.065)
        self.assertEqual(len(lines), 3)
        self.assertIn("60.", lines[0])
        self.assertIn("다름", lines[0])
        self.assertIn("-1.", lines[2])

    def test_rollover_inside_run(self):
        tr = trace_of(simulate([(1.0, 0, 0), (4.0, 60, 60), (1.0, 0, 0)], 61.2,
                               start=(0x7FFFFFFF - 50, -0x80000000 + 30)))
        runs = wc.motion_runs(tr)
        for m in (0, 1):
            self.assertAlmostEqual(runs[0].delta[m], 60 / 60 * 4 * 61.2, delta=2)

    def test_summary_prints(self):
        import contextlib
        import io
        tr = trace_of(simulate([(1.0, 0, 0), (4.0, 44, 44), (2.0, 0, 0)], 61.2))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            wc.print_summary(tr, 61.2, 0.065, args(revs=2.93, tape_m=1.2))
        out = buf.getvalue()
        self.assertIn("직진 결론", out)
        self.assertIn("타이어 마크 R", out)
        self.assertIn("줄자", out)

    def test_live_line(self):
        tr = trace_of(simulate([(1.0, 0, 0), (3.0, 44, 44)], 61.2))
        line = wc.live_line(tr, tr.cmd[-1][0], tr.cmd[0][0], 61.2, 0.065, 1.0)
        self.assertIn("명령 R  +44 L  +44", line)
        self.assertNotIn("비율 R –", line)

    def test_load_config_reads_encoder_yaml(self):
        tpr, radius, src = wc.load_config()
        if "encoder.yaml" in src and "못 읽음" not in src:
            self.assertGreater(tpr, 0)
            self.assertGreater(radius, 0)
        tpr2, radius2, src2 = wc.load_config("/없는/경로.yaml")
        self.assertEqual((tpr2, radius2), (wc.DEFAULT_TPR, wc.DEFAULT_RADIUS))
        self.assertIn("못 읽음", src2)


class TestKnob(unittest.TestCase):
    def test_decode_f1_real_frames(self):
        self.assertEqual(wc.decode_f1(bytes.fromhex("F100385804056262")), 98)
        self.assertIsNone(wc.decode_f1(bytes.fromhex("F10106010A660000")))
        self.assertIsNone(wc.decode_f1(bytes.fromhex("C50011000000")))

    def test_knob_limit_matches_motor_node(self):
        self.assertEqual(wc.knob_limit(98), (100, 1.0))
        self.assertEqual(wc.knob_limit(77), (51, 0.51))
        self.assertEqual(wc.knob_limit(76), (49, 0.49))
        self.assertEqual(wc.knob_limit(57), (5, 0.0))
        self.assertEqual(wc.knob_limit(30), (0, 0.0))

    def _section(self, uses):
        import contextlib
        import io
        plan = [(2.0, 0, 0, 98), (5.0, 73, 73, 98), (2.0, 0, 0, 98), (5.0, 73, 73, 80), (2.0, 0, 0, 80)]
        tr = trace_of(simulate(plan, 60.5, driver_uses_knob=uses))
        rows = wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2)
        self.assertEqual([r["knob"][:2] for r in rows], [(98, 98), (80, 80)])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            wc.print_knob_section(tr, rows, args())
        return buf.getvalue()

    def test_driver_ignores_knob(self):
        out = self._section(False)
        self.assertIn("원값 98 % → 모터 노드 정규화 100 % → 직진 상한 1.00 m/s", out)
        self.assertIn("원인이 아님", out)

    def test_driver_uses_knob(self):
        out = self._section(True)
        self.assertIn("바꾸는 것으로 보임", out)

    def test_single_knob_position_hint(self):
        import contextlib
        import io
        tr = trace_of(simulate([(1.0, 0, 0), (4.0, 73, 73), (1.0, 0, 0), (4.0, 73, 73), (1.0, 0, 0)], 61.2))
        rows = wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2)
        self.assertEqual(len(rows), 2)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            wc.print_knob_section(tr, rows, args())
        self.assertIn("위치가 하나(98 %)뿐", buf.getvalue())

    def test_ignore_knob_display(self):
        import contextlib
        import io
        tr = trace_of(simulate([(1.0, 0, 0), (3.0, 44, 44, 0), (1.0, 0, 0, 0)], 61.2))
        line = wc.live_line(tr, tr.cmd[-1][0], tr.cmd[0][0], 61.2, 0.065, 1.0, (55, 98, 5, 1.0, True))
        self.assertIn("가변저항   0 % (모터 노드 무시, 상한 1.00 m/s)", line)
        rows = wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            wc.print_knob_section(tr, rows, args(ignore_knob=True))
        self.assertIn("ignore_knob:=true 라 속도에 안 씀", buf.getvalue())

    def test_live_line_shows_knob(self):
        tr = trace_of(simulate([(1.0, 0, 0), (3.0, 44, 44, 80)], 61.2))
        line = wc.live_line(tr, tr.cmd[-1][0], tr.cmd[0][0], 61.2, 0.065, 1.0)
        self.assertIn("가변저항  80 %→ 58 % 상한 0.58 m/s", line)


RUN92 = os.path.expanduser("~/vica_data/bags/1012_run92_map_1002_150946_motor_cpp_can1.log")


@unittest.skipUnless(os.path.exists(RUN92), "run92 CAN 기록 없음(젯슨 전용)")
class TestRealLog(unittest.TestCase):
    def test_run92_matches_preview(self):
        tr = wc.read_log(RUN92)
        agg = wc.straight_summary(wc.segment_rows(tr, wc.steady_segments(tr.cmd), 61.2))
        for m in (0, 1):
            self.assertIsNotNone(agg[m])
            self.assertTrue(0.975 <= agg[m]["ratio"] <= 0.995, agg[m])
            self.assertTrue(59.7 <= agg[m]["tpr_speed"] <= 60.9, agg[m])


if __name__ == "__main__":
    unittest.main()
