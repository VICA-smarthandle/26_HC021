"""drive_diag.evaluate — 구간 자동 분할과 판정(좋음·주의·나쁨)·문장."""
import math
import os
import tempfile
import unittest

import numpy as np

from scripts.drive_diag import evaluate as ev
from scripts.tests.test_drive_diag_serve import fake_cache


class Grade(unittest.TestCase):
    def test_boundaries_inclusive(self):
        self.assertEqual(ev.grade(5.0, 5.0, 10.0), 'good')
        self.assertEqual(ev.grade(7.0, 5.0, 10.0), 'warn')
        self.assertEqual(ev.grade(10.1, 5.0, 10.0), 'bad')

    def test_higher_is_better(self):
        self.assertEqual(ev.grade(80.0, 75.0, 65.0, higher_better=True), 'good')
        self.assertEqual(ev.grade(70.0, 75.0, 65.0, higher_better=True), 'warn')
        self.assertEqual(ev.grade(60.0, 75.0, 65.0, higher_better=True), 'bad')

    def test_worst_of(self):
        self.assertEqual(ev.worst(['good', 'warn', 'good']), 'warn')
        self.assertEqual(ev.worst(['good', None]), 'good')


class Segments(unittest.TestCase):
    def test_stop_forward_rotate_backward(self):
        t = np.arange(0, 40, 0.05)
        vx = np.select([(t >= 5) & (t < 15), (t >= 25) & (t < 35)], [0.3, -0.3], 0.0)
        gz = np.where((t >= 18) & (t < 22), 0.5, 0.0)
        segs = ev.segment_motion(t, vx, t, gz)
        self.assertEqual([s['kind'] for s in segs], ['정지', '전진', '정지', '회전', '정지', '후진', '정지'])
        fwd = segs[1]
        self.assertAlmostEqual(fwd['t0'], 5.0, delta=0.6)
        self.assertAlmostEqual(fwd['t1'], 15.0, delta=0.6)

    def test_short_blips_merge_into_neighbour(self):
        t = np.arange(0, 20, 0.05)
        vx = np.where((t >= 2) & (t < 18), 0.3, 0.0)
        vx[(t >= 9.0) & (t < 9.4)] = 0.0  # 0.4 s 멈칫
        segs = ev.segment_motion(t, vx, t, np.zeros_like(t))
        self.assertEqual([s['kind'] for s in segs], ['정지', '전진', '정지'])


class EvaluateRange(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.z = dict(np.load(fake_cache(self.tmp.name)))

    def tearDown(self):
        self.tmp.cleanup()

    def test_wheel_heading_drift_is_bad_and_ekf_good(self):
        r = ev.evaluate_range(self.z, 0.0, 29.0)
        items = {i['key']: i for i in r['items']}
        self.assertEqual(items['odom_ekf']['grade'], 'good')
        self.assertEqual(items['odom_wheel']['grade'], 'bad')        # −0.3°/m 넘음 경계: 8.7 m 에 −2.6°
        self.assertEqual(items['loc']['grade'], 'good')              # 옆 2 cm
        self.assertEqual(r['grade'], 'bad')
        self.assertTrue(any('바퀴' in s for s in [i['text'] for i in r['items']]))

    def test_low_score_marks_localisation_untrusted(self):
        self.z['m_score_best'] = np.full_like(self.z['m_score_best'], 60.0)
        r = ev.evaluate_range(self.z, 0.0, 29.0)
        items = {i['key']: i for i in r['items']}
        self.assertEqual(items['score']['grade'], 'bad')
        self.assertIn('신뢰 낮음', items['loc']['text'])


class Solutions(unittest.TestCase):
    def _items(self, **g):
        base = dict(score=('good', dict(score=79.0, score_amcl=78.0)),
                    loc=('good', dict(lat_p95=3.0, yaw_p95=0.8, lat_mean=-1.0, yaw_mean=0.1, n=50, edge=0)),
                    odom_ekf=('good', dict(yaw_slope=0.02, lat_slope=1.5, dist=11.0, yaw=0.2, lat=2.0)),
                    odom_wheel=('good', dict(yaw_slope=-0.05, lat_slope=-0.5, dist=11.0, yaw=-0.5, lat=-5.0)),
                    wobble=('good', dict(amp=1.0)))
        base.update(g)
        return [dict(key=k, grade=v[0], values=v[1], label=k, text='') for k, v in base.items()]

    def test_all_good_has_no_action(self):
        self.assertEqual(ev.solutions(self._items()), [])

    def test_wheel_drift_with_good_ekf_points_to_mechanics(self):
        sol = ev.solutions(self._items(odom_wheel=('bad', dict(yaw_slope=-0.35, lat_slope=-1.0, dist=11.0, yaw=-3.8, lat=-30.0))))
        self.assertEqual(len(sol), 1)
        self.assertEqual(sol[0]['grade'], 'bad')
        self.assertTrue(any('캐스터' in a for a in sol[0]['actions']))
        self.assertIn('-0.35', sol[0]['why'])

    def test_low_map_score_says_remap_first(self):
        sol = ev.solutions(self._items(score=('bad', dict(score=60.0, score_amcl=58.0)),
                                       loc=('bad', dict(lat_p95=12.0, yaw_p95=4.0, lat_mean=-2.0, yaw_mean=0.3, n=50, edge=3))))
        keys = [s['key'] for s in sol]
        self.assertEqual(keys[0], 'map')                     # 지도 문제를 먼저
        self.assertTrue(any('지도' in a for a in sol[0]['actions']))
        self.assertNotIn('loc', keys)                        # 지도가 나쁘면 ① 은 따로 처방하지 않는다

    def test_loc_bad_with_good_score_is_amcl_problem(self):
        sol = ev.solutions(self._items(loc=('bad', dict(lat_p95=12.0, yaw_p95=4.0, lat_mean=7.0, yaw_mean=-2.6, n=40, edge=0))))
        self.assertEqual(sol[0]['key'], 'loc')
        self.assertTrue(any('초기 위치' in a for a in sol[0]['actions']))

    def test_wobble_bad_lists_floor_order(self):
        sol = ev.solutions(self._items(wobble=('bad', dict(amp=6.6))))
        self.assertEqual(sol[0]['key'], 'wobble')
        self.assertTrue(sol[0]['actions'][0].startswith('캐스터'))


if __name__ == '__main__':
    unittest.main()
