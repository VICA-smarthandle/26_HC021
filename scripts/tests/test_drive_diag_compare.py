"""drive_diag.compare — 주행 bag 비교(AMCL 보정량·① 위치 오차·지도 정합·흔들림)."""
import math
import os
import tempfile
import unittest

import numpy as np

from scripts.drive_diag import compare as cp


def fake_z(init_t=(), side_cm=1.0, yaw_deg=0.2):
    t = np.arange(0.0, 20.0, 0.05)
    v = np.where(t < 10.0, 0.3, -0.3)
    x = np.where(t < 10.0, 0.3 * t, 3.0 - 0.3 * (t - 10.0))
    odom = np.column_stack([x, np.zeros_like(t), np.zeros_like(t)])
    mo_t = np.arange(0.5, 20.0, 1.0)
    mo = np.zeros((len(mo_t), 3))
    mo[:, 1] = side_cm / 100 * np.arange(len(mo_t))
    mo[:, 2] = np.radians(np.where(np.arange(len(mo_t)) % 2 == 0, 0.0, yaw_deg))
    return dict(vx_t=t, vx=v, imu_t=t, gz=np.zeros_like(t), odom_t=t, odom=odom,
                mo_t=mo_t, mo=mo, init_t=np.array(init_t, dtype=float))


class TestAmclStats(unittest.TestCase):
    def test_side_by_travel_direction(self):
        a = cp.amcl_stats(fake_z(yaw_deg=0.0))
        self.assertAlmostEqual(a['dist'], 4.83, delta=0.1)
        self.assertEqual(a['n'], 16)
        self.assertAlmostEqual(a['side_net'], 0.0, delta=0.01)
        self.assertAlmostEqual(a['side_p95'], 1.0, delta=0.01)
        self.assertAlmostEqual(a['yaw_gross'], 0.0, places=6)

    def test_direction_filter(self):
        z = fake_z(yaw_deg=0.0)
        f, b = cp.amcl_stats(z, 1), cp.amcl_stats(z, -1)
        self.assertEqual((f['n'], b['n']), (8, 8))
        self.assertAlmostEqual(f['side_net'], 8 / f['dist'], delta=0.01)
        self.assertAlmostEqual(b['side_net'], -8 / b['dist'], delta=0.01)
        self.assertAlmostEqual(f['dist'] + b['dist'], cp.amcl_stats(z)['dist'], delta=1e-9)

    def test_turn_neighbourhood_excluded(self):
        z = fake_z(yaw_deg=0.0)
        z['gz'] = np.where((z['imu_t'] >= 5.0) & (z['imu_t'] < 7.0), 0.5, 0.0)
        self.assertEqual(cp.amcl_stats(z)['n'], 3 + 8)

    def test_curved_path_skipped(self):
        z = fake_z(yaw_deg=0.0)
        t = z['odom_t']
        z['odom'][:, 1] = np.where(t > 10.0, 0.30 * np.sin(np.pi * np.clip((t - 11.0) / 8.0, 0, 1)), 0.0)
        keep, skipped = cp.straight_segments(z)
        self.assertEqual([s['sign'] for s in keep], [1])
        self.assertEqual([s['sign'] for s in skipped], [-1])
        self.assertGreater(skipped[0]['width'], 20.0)
        self.assertEqual(cp.amcl_stats(z)['n'], 8)
        self.assertEqual(cp.amcl_stats(z, curve_cm=0)['n'], 16)

    def test_yaw_noise_vs_net(self):
        a = cp.amcl_stats(fake_z(side_cm=0.0))
        self.assertAlmostEqual(a['yaw_gross'], a['n'] * 0.2 / a['dist'], delta=0.01)
        self.assertLess(abs(a['yaw_net']), 0.05)
        self.assertAlmostEqual(a['yaw_p95'], 0.2, delta=0.001)
        self.assertGreater(a['side_p95'], 0.5)

    def test_initialpose_excluded(self):
        a = cp.amcl_stats(fake_z(init_t=[4.0]))
        self.assertEqual(a['n'], 16 - 8)

    def test_old_cache_without_mo(self):
        z = fake_z()
        del z['mo_t']
        self.assertIsNone(cp.amcl_stats(z))
        self.assertIn('dist', cp.bag_stats(z))


class TestLocAndVerdict(unittest.TestCase):
    def test_loc_stats_straight_only(self):
        z = fake_z()
        m_t = np.arange(0.0, 20.0, 0.5)
        z.update(m_t=m_t, m_edge=np.zeros_like(m_t), m_lat=np.full_like(m_t, 0.03),
                 m_dyaw=np.full_like(m_t, math.radians(0.5)), m_score_amcl=np.full_like(m_t, 70.0),
                 m_score_best=np.full_like(m_t, 80.0))
        L = cp.loc_stats(z)
        self.assertAlmostEqual(L['loc_lat_p95'], 3.0, places=6)
        self.assertAlmostEqual(L['loc_yaw_p95'], 0.5, places=6)
        self.assertEqual((L['score_amcl'], L['score_best']), (70.0, 80.0))

    def test_verdict(self):
        self.assertIn('줄었음', cp.verdict(1.0, 2.0, -1))
        self.assertIn('좋아짐', cp.verdict(1.0, 2.0, -1))
        self.assertIn('나빠짐', cp.verdict(60.0, 80.0, +1))
        self.assertIn('비슷', cp.verdict(1.9, 2.0, -1))
        self.assertIn('좋아짐', cp.verdict(-0.1, 0.5, 0))
        self.assertEqual(cp.verdict(None, 1.0, -1), '')


class TestResolve(unittest.TestCase):
    def test_map_from_bag_name_longest(self):
        with tempfile.TemporaryDirectory() as d:
            for n in ('map_1102', 'map_1102_170853', 'map_1102_170853_keepout'):
                open(os.path.join(d, n + '.yaml'), 'w').close()
            got = cp.map_from_bag_name('/x/1102_173000_map_1102_170853_nav', d)
            self.assertEqual(os.path.basename(got), 'map_1102_170853.yaml')
            self.assertIsNone(cp.map_from_bag_name('/x/1101_170326_laser_yaw_drive', d))


if __name__ == '__main__':
    unittest.main()
