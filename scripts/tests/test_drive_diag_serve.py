"""drive_diag.serve — 화면에 넘길 JSON 만들기(HTTP 없이)."""
import math
import os
import tempfile
import unittest

import numpy as np

from scripts.drive_diag import serve


def fake_cache(folder):
    t = np.arange(100.0, 130.0, 0.5)
    n = len(t)
    odom_t = np.arange(100.0, 130.0, 0.05)
    odom = np.c_[0.3 * (odom_t - 100), np.zeros_like(odom_t), np.zeros_like(odom_t)]
    wheel = odom.copy()
    wheel[:, 2] = math.radians(-0.3) * odom[:, 0]  # 바퀴는 오른쪽으로 돈다고 믿음
    lid_t = np.arange(100.0, 130.0, 1.0)
    lid = np.c_[0.3 * (lid_t - 100), np.zeros_like(lid_t), np.zeros_like(lid_t)]
    p = os.path.join(folder, 'x.npz')
    np.savez(p, m_t=t, m_amcl_x=np.zeros(n), m_amcl_y=np.zeros(n), m_amcl_yaw=np.zeros(n),
             m_best_x=np.zeros(n), m_best_y=np.full(n, -0.02), m_best_yaw=np.zeros(n),
             m_score_amcl=np.full(n, 70.0), m_score_best=np.full(n, 75.0), m_edge=np.zeros(n),
             m_fwd=np.zeros(n), m_lat=np.full(n, 0.02), m_dyaw=np.zeros(n), m_beams=np.full(n, 180.0),
             path_t=odom_t, path=odom, odom_t=odom_t, odom=odom, wheel_t=odom_t, wheel=wheel,
             lid_t=lid_t, lid=lid, lid_inl=np.full(len(lid_t), 0.5), vx_t=odom_t, vx=np.full(len(odom_t), 0.3),
             imu_t=odom_t, gz=np.zeros(len(odom_t)), cmd_t=odom_t, cmd_v=np.full(len(odom_t), 0.3),
             cmd_w=np.zeros(len(odom_t)), sensor=np.array([0.031, 0.0, -0.0346]), off=0.01, tf_shift=1.0,
             map_meta=np.array('{"resolution": 0.05, "origin": [-1, -1], "width": 10, "height": 10}'),
             scan_t=t, scan_r=np.ones((n, 8), np.float32), scan_amin=np.full(n, -math.pi),
             scan_ainc=np.full(n, math.pi / 4), bag=np.array('/b/x'), map_yaml=np.array('/m/x.yaml'))
    return p


class Payloads(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.z = np.load(fake_cache(self.tmp.name))

    def tearDown(self):
        self.tmp.cleanup()

    def test_result_times_are_relative_to_start_and_errors_in_cm_deg(self):
        r = serve.build_result(self.z)
        self.assertAlmostEqual(r['t0'], 100.0)
        self.assertAlmostEqual(r['match']['t'][0], 0.0)
        self.assertAlmostEqual(r['match']['lat'][0], 2.0)        # cm
        self.assertEqual(len(r['path']['x']), len(r['path']['t']))
        self.assertTrue(r['has_match'])

    def test_odom_error_from_selection_start(self):
        e = serve.build_odomerr(self.z, 10.0, 20.0)              # 상대 시각(초)
        self.assertAlmostEqual(e['t'][0], 10.0)
        self.assertAlmostEqual(e['ekf']['dyaw'][-1], 0.0, places=3)
        # 바퀴: 3 m 동안 −0.3°/m → −0.9°
        self.assertAlmostEqual(e['wheel']['dyaw'][-1], -0.9, delta=0.05)
        self.assertAlmostEqual(e['dist'], 3.0, delta=0.05)

    def test_scan_points_in_map_frame(self):
        s = serve.build_scan(self.z, 0.2)
        self.assertEqual(len(s['amcl']['x']), 8)
        self.assertAlmostEqual(s['t'], 0.0)



class EvalEndpoints(unittest.TestCase):
    def test_app_segments_cached_and_json_safe(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            p = fake_cache(d)
            app = serve.App(d, d, d)
            r = app.segments(os.path.basename(p))
            json.dumps(r, ensure_ascii=False)          # 화면으로 보낼 수 있어야 한다
            self.assertIs(app.segments(os.path.basename(p)), r)
            self.assertIn('thresholds', r)


if __name__ == '__main__':
    unittest.main()
