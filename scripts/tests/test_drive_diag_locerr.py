"""drive_diag.locerr — x·y·θ 오차 계산.

① 위치추정 오차 = AMCL 자세 − 스캔이 지도에 가장 잘 맞는 자세(좁은 지도 대조)
② 오도메트리 오차 = EKF·바퀴 이동 − 스캔-스캔 ICP 이동(구간 시작 기준)
루트에서:  python3 -m pytest scripts/tests/test_drive_diag_locerr.py
"""
import math
import tempfile
import unittest

import numpy as np

from scripts.drive_diag import locerr as le
from scripts.tests import drive_diag_sim as sim

SENSOR = (0.031, 0.0, math.radians(-2.0))  # base_link → laser_frame (VICA 실측과 같은 꼴)


def laser_pose(base):
    return le.compose(base, SENSOR)


class LoadMap(unittest.TestCase):
    def test_image_top_row_becomes_last_grid_row(self):
        data = np.zeros((10, 20), dtype=np.int8)
        data[7, 5] = 100  # 0행 = origin_y 쪽
        with tempfile.TemporaryDirectory() as d:
            yml = sim.write_map(d, data, -1.0, -0.5, 0.1)
            grid, meta = le.load_map(yml)
        self.assertEqual(grid.data.shape, (10, 20))
        self.assertEqual(int(grid.data[7, 5]), 100)
        self.assertEqual(int(grid.data[2, 5]), 0)
        self.assertAlmostEqual(grid.origin_x, -1.0)
        self.assertAlmostEqual(meta['resolution'], 0.1)


class Frames(unittest.TestCase):
    def test_world_offset_in_robot_frame(self):
        e = le.pose_error((-0.05, 1.0, math.pi / 2), (0.0, 1.0, math.pi / 2))
        self.assertAlmostEqual(e['lat'], 0.05, places=9)  # 북쪽을 보는 로봇에게 서쪽 = 왼쪽
        self.assertAlmostEqual(e['fwd'], 0.0, places=9)

    def test_compose_inverse_round_trip(self):
        a, b = (1.0, 2.0, 0.3), (-0.4, 0.7, -1.1)
        r = le.relative(a, le.compose(a, b))
        np.testing.assert_allclose(r, b, atol=1e-12)


class LocalMatch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        data, ox, oy = sim.rasterize(res=0.05)
        cls.grid = le.ps.MapGrid(data, 0.05, ox, oy)
        cls.field = le.ps.build_likelihood_field(cls.grid)

    def test_finds_true_pose_and_reports_amcl_offset(self):
        rng = np.random.default_rng(0)
        true = (2.0, 0.05, 0.0)
        lp = laser_pose(true)
        beams = le.beams_from_scan(*sim.raycast(lp[0], lp[1], lp[2], rng=rng))
        amcl = (2.0, 0.11, math.radians(1.0))  # AMCL 이 6 cm 왼쪽·1° 왼쪽으로 틀렸다
        m = le.local_match(self.field, self.grid, beams, amcl, SENSOR)
        self.assertFalse(m.edge)
        self.assertAlmostEqual(m.x, true[0], delta=0.015)
        self.assertAlmostEqual(m.y, true[1], delta=0.01)
        self.assertAlmostEqual(math.degrees(m.yaw), 0.0, delta=0.15)
        e = le.pose_error(amcl, (m.x, m.y, m.yaw))
        self.assertAlmostEqual(e['lat'], 0.06, delta=0.01)
        self.assertAlmostEqual(math.degrees(e['dyaw']), 1.0, delta=0.15)
        self.assertGreater(m.score, m.amcl_score)

    def test_edge_flag_when_error_exceeds_search_window(self):
        rng = np.random.default_rng(1)
        true = (3.0, 0.0, 0.0)
        lp = laser_pose(true)
        beams = le.beams_from_scan(*sim.raycast(lp[0], lp[1], lp[2], rng=rng))
        m = le.local_match(self.field, self.grid, beams, (3.0, 0.30, 0.0), SENSOR)
        self.assertTrue(m.edge)


class LidarOdometry(unittest.TestCase):
    def _drive(self, rng, dist=3.0, step=0.25, yaw_rate_per_m=0.0):
        """참 경로(곡률 yaw_rate_per_m)를 따라 키 스캔과 참 자세."""
        poses, x, y, th = [], 1.0, 0.0, 0.0
        for _ in range(int(dist / step) + 1):
            poses.append((x, y, th))
            x += step * math.cos(th + yaw_rate_per_m * step / 2)
            y += step * math.sin(th + yaw_rate_per_m * step / 2)
            th += yaw_rate_per_m * step
        scans = []
        for p in poses:
            lp = laser_pose(p)
            r, amin, ainc = sim.raycast(lp[0], lp[1], lp[2], rng=rng)
            scans.append(le.scan_points(r, amin, ainc))
        return poses, scans

    def test_chain_recovers_base_motion_with_mounted_laser_yaw(self):
        rng = np.random.default_rng(2)
        poses, scans = self._drive(rng)
        guess = [le.relative(a, b) for a, b in zip(poses[:-1], poses[1:])]
        chain, inl = le.lidar_chain(scans, guess, SENSOR)
        end = chain[-1]
        self.assertAlmostEqual(end[0], 3.0, delta=0.03)
        self.assertAlmostEqual(end[1], 0.0, delta=0.01)  # 장착각을 빼고 나면 옆 이동 없음
        self.assertAlmostEqual(math.degrees(end[2]), 0.0, delta=0.2)
        self.assertGreater(min(inl), 0.2)

    def test_odom_error_shows_ekf_heading_drift(self):
        rng = np.random.default_rng(3)
        poses, scans = self._drive(rng)
        guess = [le.relative(a, b) for a, b in zip(poses[:-1], poses[1:])]
        chain, _ = le.lidar_chain(scans, guess, SENSOR)
        t_ref = np.arange(len(chain), dtype=float)
        # EKF: 같은 거리를 가지만 방향이 1 m 당 +0.3° 씩 왼쪽으로 샌다고 믿음
        k = math.radians(0.3)
        ekf, x, y, th = [], 0.0, 0.0, 0.0
        for i in range(len(chain)):
            ekf.append((x, y, th))
            x += 0.25 * math.cos(th + k * 0.125)
            y += 0.25 * math.sin(th + k * 0.125)
            th += k * 0.25
        err = le.odom_error(t_ref, np.array(chain), t_ref, np.array(ekf), t0=0.0)
        self.assertAlmostEqual(math.degrees(err['dyaw'][-1]), 0.9, delta=0.2)
        self.assertAlmostEqual(err['lat'][-1], 0.0236, delta=0.012)  # L²κ/2 = 9·0.00524/2
        self.assertAlmostEqual(err['fwd'][-1], 0.0, delta=0.03)


class TfShift(unittest.TestCase):
    def test_future_dated_map_odom_is_detected(self):
        rt = np.arange(0, 10, 0.08)
        s = le.tf_time_shift(rt + 1.0 - 0.003, rt, rt - 0.010, rt)
        self.assertAlmostEqual(s, 1.0, places=6)

    def test_no_shift_when_stamps_match(self):
        rt = np.arange(0, 10, 0.08)
        self.assertEqual(le.tf_time_shift(rt - 0.01, rt, rt - 0.01, rt), 0.0)


if __name__ == '__main__':
    unittest.main()


class Keyframes(unittest.TestCase):
    def test_every_quarter_metre_or_three_degrees(self):
        P = [(0.05 * i, 0.0, 0.0) for i in range(21)]                       # 1 m 직진
        P += [(1.0, 0.0, math.radians(0.5 * i)) for i in range(1, 19)]     # 제자리 9°
        idx = le.select_keyframes(P, step=0.25, yaw_step=math.radians(3.0))
        self.assertEqual(idx[:5], [0, 5, 10, 15, 20])
        self.assertEqual(len(idx), 5 + 3)


class StaticTf(unittest.TestCase):
    def test_chain_to_laser(self):
        entries = [('base_footprint', 'base_link', (0.0, 0.0, 0.0)),
                   ('base_link', 'laser_frame', (0.031, 0.0, -0.0346)),
                   ('base_link', 'camera_link', (-0.379, 0.0, 0.0))]
        s = le.static_pose(entries, 'base_footprint', 'laser_frame')
        np.testing.assert_allclose(s, (0.031, 0.0, -0.0346), atol=1e-12)

    def test_missing_chain_is_none(self):
        self.assertIsNone(le.static_pose([('a', 'b', (0, 0, 0))], 'base_footprint', 'laser_frame'))


class TfShiftExact(unittest.TestCase):
    def test_snaps_to_scan_stamps_despite_record_latency(self):
        scan = np.arange(0, 20, 0.0833)
        mo_stamp = scan[::3] + 1.0               # AMCL: 스캔 시각 + transform_tolerance
        mo_rt = scan[::3] + 0.09                 # 기록은 처리 지연 뒤
        od = np.arange(0, 20, 0.033)
        s = le.tf_time_shift(mo_stamp, mo_rt, od - 0.01, od, scan_stamps=scan)
        self.assertAlmostEqual(s, 1.0, places=6)


class OdomErrorStartAfterLongStop(unittest.TestCase):
    def test_start_inside_long_keyframe_gap_is_not_interpolated_in_time(self):
        # 0~49 s 정지 뒤 0.25 m/s 로 출발. 라이다 키프레임은 0 s 와 출발 뒤 0.25 m 마다만 있다.
        t_ref = np.array([0.0, 50.0, 51.0, 52.0])
        ref = np.array([[0.0, 0, 0], [0.25, 0, 0], [0.5, 0, 0], [0.75, 0, 0]])
        t_est = np.arange(0, 52.01, 0.05)
        est = np.c_[np.clip(0.25 * (t_est - 49.0), 0, None), np.zeros_like(t_est), np.zeros_like(t_est)]
        e = le.odom_error(t_ref, ref, t_est, est, t0=48.0)   # EKF 가 참과 같으니 오차 0 이어야
        self.assertLess(abs(e['fwd'][-1]), 0.01)
        self.assertLess(abs(e['lat'][-1]), 0.01)
