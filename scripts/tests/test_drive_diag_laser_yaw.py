"""drive_diag.laser_yaw — 라이다 장착각(yaw) 측정 계산.

방법 1: 차체를 벽과 평행하게 두고 정지 스캔에서 벽 직선의 각도 → 라이다 yaw = −벽 각도.
방법 2: 명령 w=0 전진·후진의 진행 방향(라이다 좌표계) → 장착각 = −(전진+후진)/2, 미끄럼 = (전진−후진)/2.
루트에서:  python3 -m pytest scripts/tests/test_drive_diag_laser_yaw.py
"""
import math
import unittest

import numpy as np

from scripts.drive_diag import laser_yaw as ly

# 복도 1.3 m 폭(y = ±0.65), 길이 12 m, 2 m 마다 문틀 돌출(관측성을 위해)
WALLS = [((-1.0, 0.65), (11.0, 0.65)), ((-1.0, -0.65), (11.0, -0.65)), ((11.0, -0.65), (11.0, 0.65)),
         ((-1.0, -0.65), (-1.0, 0.65))]
for k in range(0, 11, 2):
    WALLS.append(((k, 0.65), (k, 0.55)))
    WALLS.append(((k + 0.4, -0.65), (k + 0.4, -0.55)))


def raycast(px, py, heading, n=720, rmax=11.0, noise=0.005, rng=None):
    """센서 자세(px, py, heading)에서 본 스캔 (ranges, angle_min, angle_inc)."""
    ang = -math.pi + np.arange(n) * (2 * math.pi / n)
    r = np.full(n, np.inf)
    for (ax, ay), (bx, by) in WALLS:
        ex, ey = bx - ax, by - ay
        for i, a in enumerate(ang):
            dx, dy = math.cos(heading + a), math.sin(heading + a)
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((ax - px) * ey - (ay - py) * ex) / den
            s = ((ax - px) * dy - (ay - py) * dx) / den
            if t > 0 and 0 <= s <= 1 and t < r[i]:
                r[i] = t
    if rng is not None:
        r = r + rng.normal(0, noise, n)
    r[r > rmax] = np.inf
    return r, -math.pi, 2 * math.pi / n


def drive(laser_yaw, crab, direction, rng, step=0.5, n=8, y0=0.0):
    """차체 heading 0 으로 x 방향 직진(direction +1 전진, −1 후진). crab: 옆 미끄럼(rad, 세계 +y 쪽).

    반환: 0.5 m 간격 키 스캔 점들(라이다 좌표계) 과 바퀴 거리.
    """
    keys = []
    x = 1.0 if direction > 0 else 1.0 + step * (n - 1)
    y = y0
    for k in range(n):
        r, amin, ainc = raycast(x, y, laser_yaw, rng=rng)
        keys.append(ly.scan_points(r, amin, ainc))
        x += direction * step
        y += step * math.tan(crab)
    return keys, step


class ScanPoints(unittest.TestCase):
    def test_drops_out_of_range_beams(self):
        r = np.array([0.1, 1.0, np.inf, 20.0])
        p = ly.scan_points(r, 0.0, math.pi / 2)
        self.assertEqual(p.shape, (1, 2))
        np.testing.assert_allclose(p[0], [0.0, 1.0], atol=1e-9)


class WallFit(unittest.TestCase):
    def test_recovers_wall_angle_and_yaw_sign(self):
        rng = np.random.default_rng(0)
        # 차체는 벽과 평행, 라이다가 −2° (정면이 오른쪽으로) 돌아감 → 라이다 좌표계에서 벽은 +2° 로 보임
        r, amin, ainc = raycast(1.0, 0.0, math.radians(-2.0), rng=rng)
        res = ly.fit_wall(ly.scan_points(r, amin, ainc), sector_deg=(30, 150))
        self.assertAlmostEqual(math.degrees(res['angle']), 2.0, delta=0.1)
        self.assertAlmostEqual(res['yaw_deg'], -2.0, delta=0.1)
        self.assertGreater(res['inliers'], 100)


class MotionDirection(unittest.TestCase):
    def test_mount_yaw_seen_same_forward_and_backward(self):
        rng = np.random.default_rng(1)
        f = ly.motion_direction(*drive(math.radians(-2.0), 0.0, +1, rng), direction=+1)
        b = ly.motion_direction(*drive(math.radians(-2.0), 0.0, -1, rng), direction=-1)
        self.assertAlmostEqual(math.degrees(f['phi']), 2.0, delta=0.2)
        self.assertAlmostEqual(math.degrees(b['phi']), 2.0, delta=0.2)

    def test_slip_flips_between_forward_and_backward(self):
        rng = np.random.default_rng(2)
        crab = math.radians(1.0)
        f = ly.motion_direction(*drive(0.0, crab, +1, rng), direction=+1)
        b = ly.motion_direction(*drive(0.0, crab, -1, rng), direction=-1)
        self.assertAlmostEqual(math.degrees(f['phi']), 1.0, delta=0.2)
        self.assertAlmostEqual(math.degrees(b['phi']), -1.0, delta=0.2)


class Combine(unittest.TestCase):
    def test_mount_and_slip_separate(self):
        res = ly.combine([math.radians(3.0)] * 3, [math.radians(1.0)] * 3)
        self.assertAlmostEqual(res['yaw_deg'], -2.0, places=6)
        self.assertAlmostEqual(res['slip_deg'], 1.0, places=6)

    def test_needs_both_directions(self):
        self.assertIsNone(ly.combine([0.03], [])['yaw_deg'])


class DriveRuns(unittest.TestCase):
    def test_splits_forward_and_backward_open_loop_runs(self):
        rt = np.arange(0, 30, 0.05)
        v = np.where(rt < 10, 0.3, np.where(rt < 15, 0.0, -0.3))
        w = np.zeros_like(rt)
        runs = ly.drive_runs(np.c_[rt, v, w], min_len=3.0)
        self.assertEqual([r[2] for r in runs], [+1, -1])


if __name__ == '__main__':
    unittest.main()


class TfYaw(unittest.TestCase):
    def test_laser_yaw_through_chain(self):
        entries = [('base_footprint', 'base_link', 0.0), ('base_link', 'laser_frame', -0.0346),
                   ('base_link', 'camera_link', 0.0)]
        self.assertAlmostEqual(ly.laser_tf_yaw(entries), -0.0346, places=6)

    def test_missing_laser_frame_is_none(self):
        self.assertIsNone(ly.laser_tf_yaw([('base_footprint', 'base_link', 0.0)]))
