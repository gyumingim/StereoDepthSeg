import json
import tempfile
from pathlib import Path
import unittest

import cv2
import numpy as np

from lib_rgbd_viewer import RGBDViewer


def snapshot():
    h, w = 60, 80
    z = np.full((h, w), 1.234, np.float32)
    y, x = np.mgrid[:h, :w]
    points = np.stack([(x-w/2)*z/100, (y-h/2)*z/100, z], -1).astype(np.float32)
    valid = np.ones((h, w), bool); valid[0, 0] = False
    z[0, 0] = np.nan; points[0, 0] = np.nan
    rgb = np.full((h, w, 3), [10, 20, 200], np.uint8)
    return dict(rgb=rgb, depth=z, valid=valid, points=points, depth_color=rgb.copy(),
                depth_range=(.3, 3.), K_rect=np.eye(3), R1=np.eye(3),
                timestamp_ns=[100, 200], skew_ms=.0001, scale_status='metric',
                imu={'gyroscope': {'values': [0., .1, .2], 'dt_ms': 1.}})


class ViewerTests(unittest.TestCase):
    def test_metric_sample_and_invalid_pixels(self):
        v = RGBDViewer(); v.set_snapshot(snapshot())
        self.assertIsNone(v.sample(0, 0))
        self.assertIsNone(v.sample(-1, 10))
        hit = v.sample(40, 30)
        self.assertAlmostEqual(hit['rectified_z_m'], 1.234, places=5)
        self.assertAlmostEqual(hit['range_m'], 1.234, places=5)

    def test_depth_export_units_invalid_color_alignment_and_metadata(self):
        v = RGBDViewer(); s = snapshot(); v.set_snapshot(s)
        with tempfile.TemporaryDirectory() as d:
            self.assertTrue(v.save(d))
            d = Path(d)
            z = cv2.imread(str(d/'depth_mm.png'), cv2.IMREAD_UNCHANGED)
            self.assertEqual(z.dtype, np.uint16)
            self.assertEqual(z[0, 0], 0)
            self.assertEqual(z[30, 40], 1234)
            np.testing.assert_array_equal(cv2.imread(str(d/'rgb.png')), s['rgb'])
            data = np.load(d/'rgbd.npz')
            self.assertAlmostEqual(float(data['depth_m'][30, 40]), 1.234, places=5)
            meta = json.loads((d/'frame.json').read_text())
            self.assertEqual(meta['timestamp_ns'], [100, 200])
            self.assertEqual(meta['depth_unit_m'], .001)
            self.assertIn('200 20 10', (d/'scene.ply').read_text())
            self.assertTrue((d/'scene.html').exists())

    def test_freeze_keeps_rgb_depth_and_imu_from_same_snapshot(self):
        v = RGBDViewer(); a, b = snapshot(), snapshot()
        b['timestamp_ns'] = [300, 400]
        v.set_snapshot(a); v.key(ord('f')); v.set_snapshot(b)
        self.assertIs(v.snapshot, a)
        v.key(ord('f')); v.set_snapshot(b)
        self.assertIs(v.snapshot, b)

    def test_mouse_measure_orbit_zoom_and_pan(self):
        v = RGBDViewer(); v.set_snapshot(snapshot())
        v.mouse(cv2.EVENT_LBUTTONDOWN, 960, 225, 0)
        self.assertEqual(v.pixel, (40, 30))
        yaw = v.yaw
        v.mouse(cv2.EVENT_LBUTTONDOWN, 300, 600, 0)
        v.mouse(cv2.EVENT_MOUSEMOVE, 320, 620, cv2.EVENT_FLAG_LBUTTON)
        self.assertNotEqual(v.yaw, yaw)
        v.mouse(cv2.EVENT_MOUSEMOVE, 330, 630, cv2.EVENT_FLAG_SHIFTKEY)
        np.testing.assert_array_equal(v.pan, [10, 10])
        v.mouse(cv2.EVENT_LBUTTONUP, 330, 630, 0)
        v.mouse(cv2.EVENT_MOUSEWHEEL, 300, 600, 120 << 16)
        self.assertGreater(v.zoom, 150)
        v.key(ord('r')); self.assertEqual(v.zoom, 150)

    def test_empty_and_invalid_cloud_render(self):
        v = RGBDViewer()
        self.assertEqual(v.render().shape, (878, 1280, 3))
        s = snapshot(); s['valid'][:] = False
        v.set_snapshot(s)
        self.assertEqual(len(v.cloud()[0]), 0)
        self.assertEqual(v.render().shape, (878, 1280, 3))

    def test_cloud_range_and_cache(self):
        v = RGBDViewer(); s = snapshot(); s['depth'][1, 1] = 4.
        v.set_snapshot(s)
        self.assertEqual(len(v.cloud()[0]), 60*80-2)
        v.render(); cached = v._cache
        v.key(255); v.render()
        self.assertIs(v._cache, cached)

if __name__ == '__main__':
    unittest.main()
