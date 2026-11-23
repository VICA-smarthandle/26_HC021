"""drive_diag.map_scale — 지도 길이 보정(그림 다시 뽑기·레일 좌표 배율)."""
import json
import os
import tempfile
import unittest

import numpy as np

from scripts.drive_diag import map_scale as ms


def room(w=200, h=120, wall=1):
    """빈칸 방 + 1 칸 두께 벽, 바깥은 미지."""
    a = np.full((h, w), ms.UNKNOWN, np.uint8)
    a[10:h - 10, 10:w - 10] = ms.FREE
    a[10:10 + wall, 10:w - 10] = ms.OCC
    a[h - 10 - wall:h - 10, 10:w - 10] = ms.OCC
    a[10:h - 10, 10:10 + wall] = ms.OCC
    a[10:h - 10, w - 10 - wall:w - 10] = ms.OCC
    return a


def wall_x(a, origin, res, row):
    """한 줄에서 점유 칸들의 x(m)."""
    h = a.shape[0]
    cols = np.where(a[row] == ms.OCC)[0]
    return origin[0] + (cols + 0.5) * res, origin[1] + (h - 1 - row + 0.5) * res


class TestScaleGrid(unittest.TestCase):
    def test_shrink_keeps_thin_walls_and_scales_about_origin(self):
        a = room()
        origin, res, s = [-3.0, -2.0], 0.05, 0.9645
        b, o2 = ms.scale_grid(a, origin, res, s)
        self.assertEqual(o2, [origin[0] * s, origin[1] * s])
        self.assertEqual(b.shape, (int(np.ceil(120 * s)), int(np.ceil(200 * s))))
        mid = b.shape[0] // 2
        xs, _ = wall_x(b, o2, res, mid)
        x0, _ = wall_x(a, origin, res, 60)
        self.assertAlmostEqual(xs.min(), x0.min() * s, delta=0.05)
        self.assertAlmostEqual(xs.max(), x0.max() * s, delta=0.05)
        self.assertAlmostEqual((b == ms.OCC).sum() / (a == ms.OCC).sum(), s, delta=0.03)
        self.assertEqual(set(np.unique(b).tolist()), {ms.OCC, ms.FREE, ms.UNKNOWN})

    def test_mask_value_passes_through(self):
        a = np.full((50, 60), 128, np.uint8)
        b, _ = ms.scale_grid(a, [0.0, 0.0], 0.05, 0.9645)
        self.assertEqual(np.unique(b).tolist(), [128])

    def test_enlarge(self):
        a = room(100, 80)
        b, o2 = ms.scale_grid(a, [-1.0, -1.0], 0.05, 1.05)
        self.assertEqual(b.shape, (int(np.ceil(80 * 1.05)), int(np.ceil(100 * 1.05))))
        self.assertTrue((b == ms.OCC).any())


class TestJson(unittest.TestCase):
    def test_route_and_edit(self):
        geo = {'type': 'FeatureCollection', 'features': [
            {'type': 'Feature', 'properties': {'id': 1, 'frame': 'map'},
             'geometry': {'type': 'Point', 'coordinates': [10.25, -0.355]}},
            {'type': 'Feature', 'properties': {'id': 9}, 'geometry': {'type': 'LineString',
                                                                     'coordinates': [[0, 0], [2.0, 4.0]]}}]}
        out = ms.scale_json(geo, 0.5)
        self.assertEqual(out['features'][0]['geometry']['coordinates'], [5.125, -0.1775])
        self.assertEqual(out['features'][1]['geometry']['coordinates'], [[0, 0], [1.0, 2.0]])
        self.assertEqual(out['features'][0]['properties'], {'id': 1, 'frame': 'map'})
        edit = {'sketch': {'nodes': [{'id': 1, 'x': 10.25, 'y': -0.355}], 'edges': [[1, 2]]}}
        e2 = ms.scale_json(edit, 0.5)
        self.assertEqual(e2['sketch']['nodes'][0], {'id': 1, 'x': 5.125, 'y': -0.1775})
        self.assertEqual(e2['sketch']['edges'], [[1, 2]])


class TestBuild(unittest.TestCase):
    def test_build_writes_all_files(self):
        with tempfile.TemporaryDirectory() as d:
            maps, out = os.path.join(d, 'maps'), os.path.join(d, 'out')
            os.makedirs(maps)
            ms.write_pgm(os.path.join(maps, 'm.pgm'), room())
            with open(os.path.join(maps, 'm.yaml'), 'w') as f:
                f.write('image: m.pgm\nmode: trinary\nresolution: 0.05\norigin: [-3.0, -2.0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
            json.dump({'sketch': {'nodes': [{'id': 1, 'x': 2.0, 'y': 1.0}]}}, open(os.path.join(maps, 'm_route_edit.json'), 'w'))
            json.dump({'map_id': 'm'}, open(os.path.join(maps, 'm.meta.json'), 'w'))
            done = ms.build('m', 0.9645, out, maps)
            self.assertIn('m.pgm', done)
            import yaml
            y = yaml.safe_load(open(os.path.join(out, 'm.yaml')))
            self.assertEqual(y['resolution'], 0.05)
            self.assertAlmostEqual(y['origin'][0], -3.0 * 0.9645, places=4)
            self.assertEqual(json.load(open(os.path.join(out, 'm_route_edit.json')))['sketch']['nodes'][0]['x'], 1.929)
            self.assertEqual(json.load(open(os.path.join(out, 'm.meta.json')))['scale_history'][0]['scale'], 0.9645)


class TestDestinations(unittest.TestCase):
    def test_pose_and_wait_spot_scaled(self):
        import yaml
        with tempfile.TemporaryDirectory() as d:
            root = os.path.join(d, 'dest'); os.makedirs(os.path.join(root, 'm'))
            doc = {'schema_version': 1, 'map_id': 'm', 'destinations': [
                {'id': 'a', 'name': '도착', 'floor': 4, 'pose': {'frame_id': 'map', 'x': 10.25, 'y': -0.355, 'yaw': 0.0},
                 'wait_spot': {'x': 2.0, 'y': 4.0, 'yaw': 90.0, 'side': 'right'}}]}
            yaml.safe_dump(doc, open(os.path.join(root, 'm', 'destinations.yaml'), 'w'), allow_unicode=True)
            done = ms.build_destinations('m', 0.9645, os.path.join(d, 'out'), root)
            self.assertEqual(done, ['destinations.yaml'])
            out = yaml.safe_load(open(os.path.join(d, 'out', 'destinations', 'destinations.yaml')))
            dd = out['destinations'][0]
            self.assertEqual((dd['pose']['x'], dd['pose']['y']), (9.8861, -0.3424))
            self.assertEqual((dd['wait_spot']['x'], dd['wait_spot']['y'], dd['wait_spot']['yaw']), (1.929, 3.858, 90.0))
            self.assertEqual((dd['floor'], dd['name']), (4, '도착'))


if __name__ == '__main__':
    unittest.main()
