"""drive_diag.metrics — 주행 흔들림·치우침 판정 계산의 기본 성질."""
import math
import unittest

import numpy as np

from scripts.drive_diag import metrics as m

CIRC = 2 * math.pi * 0.065
RPM_STEP_W = CIRC / 60 / 0.37


class QuantizedW(unittest.TestCase):
    def test_straight_command_stays_zero(self):
        self.assertEqual(m.quantized_w(np.array([0.3]), np.array([0.0]))[0], 0.0)

    def test_tiny_turn_below_half_rpm_is_dropped(self):
        self.assertEqual(m.quantized_w(np.array([0.3]), np.array([0.005]))[0], 0.0)

    def test_one_rpm_split_gives_one_step(self):
        w = 0.016
        q = m.quantized_w(np.array([0.3]), np.array([w]))[0]
        self.assertAlmostEqual(q, RPM_STEP_W, places=6)


class StraightSegments(unittest.TestCase):
    def test_finds_only_the_straight_part(self):
        dt = 0.05
        t = np.arange(0, 30, dt)
        vx = np.where(t < 25, 0.4, 0.0)
        gz = np.where(t < 10, 0.5, 0.0)
        segs = m.straight_segments(t, vx, gz, min_len=4.0)
        self.assertEqual(len(segs), 1)
        a, b = segs[0]
        self.assertAlmostEqual(t[a], 10.5, delta=0.6)
        self.assertAlmostEqual(t[b], 24.5, delta=0.6)


class WobbleAmp(unittest.TestCase):
    def test_sine_heading_amplitude_in_degrees(self):
        dt = 0.05
        t = np.arange(0, 20, dt)
        yaw = np.radians(3.0) * np.sin(2 * np.pi * 0.4 * t) + 0.01 * t
        amp = m.wobble_amp_deg(yaw, dt)
        self.assertAlmostEqual(amp, 2 * 3.0 * math.sin(0.45 * math.pi), delta=0.4)


class BandRms(unittest.TestCase):
    def test_sine_energy_lands_in_its_band(self):
        dt = 0.025
        t = np.arange(0, 20, dt)
        x = 0.1 * np.sin(2 * np.pi * 0.5 * t)
        self.assertAlmostEqual(m.band_rms(x, dt, 0.2, 1.0), 0.1 / math.sqrt(2), delta=0.01)
        self.assertLess(m.band_rms(x, dt, 1.0, 3.0), 0.01)


class FitDelayGain(unittest.TestCase):
    def test_recovers_delay_and_gain(self):
        dt = 0.05
        rng = np.random.default_rng(1)
        cmd = np.convolve(rng.normal(0, 0.1, 2000), np.ones(10) / 10, 'same')
        y = 1.2 * np.roll(cmd, 4) + rng.normal(0, 0.002, cmd.size)
        lag, gain, resid = m.fit_delay_gain(cmd, y, dt)
        self.assertAlmostEqual(lag, 0.20, places=2)
        self.assertAlmostEqual(gain, 1.2, delta=0.05)
        self.assertLess(resid, 0.01)


class AmclLateral(unittest.TestCase):
    def test_left_jump_is_positive_and_forward_jump_is_not_lateral(self):
        t = np.arange(0, 10, 0.05)
        ekf = np.c_[t, np.zeros_like(t), 0.4 * t, np.full_like(t, math.pi / 2)]
        mo = np.array([[0.0, 0, 0, 0], [3.0, -0.01, 0, 0], [6.0, -0.01, 0.02, 0]])
        c = m.amcl_corrections(mo, ekf)
        self.assertEqual(len(c), 2)
        self.assertAlmostEqual(c[0]['side'], 0.01, places=6)
        self.assertAlmostEqual(c[0]['fwd'], 0.0, places=6)
        self.assertAlmostEqual(c[1]['side'], 0.0, places=6)
        self.assertAlmostEqual(c[1]['fwd'], 0.02, places=6)


class OpenLoop(unittest.TestCase):
    def test_exact_zero_turn_runs_only(self):
        rt = np.arange(0, 10, 0.05)
        v = np.full_like(rt, 0.3)
        w = np.where((rt > 2) & (rt < 6), 0.0, 0.02)
        segs = m.open_loop_segments(np.c_[rt, v, w], min_len=2.0)
        self.assertEqual(len(segs), 1)
        self.assertAlmostEqual(segs[0][0], 2.05, delta=0.06)
        self.assertAlmostEqual(segs[0][1], 5.95, delta=0.06)


class Verdicts(unittest.TestCase):
    def test_body_side_when_gyro_far_exceeds_command(self):
        self.assertEqual(m.verdict_c1(cmd_amp=2.4, gyro_amp=6.6)['who'], 'body')

    def test_command_side_when_command_itself_swings(self):
        self.assertEqual(m.verdict_c1(cmd_amp=4.0, gyro_amp=5.0)['who'], 'command')

    def test_normal_on_bare_floor_values(self):
        self.assertEqual(m.verdict_c1(cmd_amp=1.6, gyro_amp=2.4)['who'], 'normal')

    def test_robot_fixed_when_both_directions_same_sign(self):
        v = m.verdict_c5({'동': (+3.4, 17.0), '서': (+0.6, 11.0)})
        self.assertEqual(v['kind'], 'robot')

    def test_place_fixed_when_signs_flip(self):
        v = m.verdict_c5({'북': (+2.0, 10.0), '남': (-2.0, 10.0)})
        self.assertEqual(v['kind'], 'place')


if __name__ == '__main__':
    unittest.main()


def synthetic_run(dist_amp=0.0, bias=0.0, dur=60.0, v=0.4, heading=0.0):
    """동쪽(heading) 직진. 명령 w 는 0.4 Hz 작은 흔들림, 차체 = 정수 rpm 명령(0.2 s 지연) + 외란 + 편향."""
    dt = 0.05
    t = np.arange(0, dur, dt) + 1000.0
    cw = 0.03 * np.sin(2 * np.pi * 0.4 * t)
    wq = m.quantized_w(np.full_like(t, v), cw)
    rate = np.r_[np.zeros(4), wq[:-4]] + dist_amp * np.sin(2 * np.pi * 0.45 * t) + bias
    yaw = heading + np.r_[0.0, np.cumsum(rate)[:-1]] * dt
    x = np.cumsum(v * np.cos(yaw)) * dt
    y = np.cumsum(v * np.sin(yaw)) * dt
    odo = np.c_[t, x, y, yaw, np.full_like(t, v), rate, t + 0.01]
    mo_t = np.arange(t[0], t[-1], 0.1)
    return dict(wheel=odo, ekf=odo.copy(), imu=np.c_[t, rate, t + 0.01],
                mapodom=np.c_[mo_t, np.zeros((len(mo_t), 3)), mo_t + 0.01], amcl=np.zeros((0, 5)),
                cmd=np.c_[t + 0.01, np.full_like(t, v), cw], vcc=np.zeros((0, 2)))


class RunChecks(unittest.TestCase):
    def test_calm_run_is_normal(self):
        r = m.run_checks(synthetic_run(dist_amp=0.0))
        self.assertEqual(r['c1']['who'], 'normal')
        self.assertGreater(r['c1']['dist_m'], 15)

    def test_disturbed_body_is_flagged_body(self):
        r = m.run_checks(synthetic_run(dist_amp=0.15))
        self.assertEqual(r['c1']['who'], 'body')
        self.assertGreater(r['c2']['band']['0.2-1'], 0.05)

    def test_left_bias_shows_in_wheel_minus_rpm(self):
        r = m.run_checks(synthetic_run(bias=0.002))
        self.assertAlmostEqual(r['c4']['wheel_minus_rpm'], math.degrees(0.002 / 0.4), delta=0.08)
        self.assertAlmostEqual(r['c4']['gyro_minus_wheel'], 0.0, delta=0.02)

    def test_delay_is_recovered(self):
        r = m.run_checks(synthetic_run(dist_amp=0.0))
        self.assertAlmostEqual(r['c2']['delay_wheel'], 0.2, delta=0.06)

    def test_range_limits_the_segments(self):
        d = synthetic_run()
        t0 = d['wheel'][0, 0]
        r = m.run_checks(d, t0 + 10, t0 + 30)
        self.assertLess(r['c1']['dist_m'], 8.5)
