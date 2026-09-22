"""Phone RGB-D/IMU inspector. GUI calls stay on the main thread.

The published snapshot is one completed stereo inference, never mixed with newer
RGB or IMU. Points use original left-camera coordinates; depth is rectified Z.
"""
import json
from pathlib import Path
import time

import cv2
import numpy as np


class RGBDViewer:
    TITLE = 'Phone RGB-D | RGB / Depth / 3D / IMU'
    W, H = 640, 420

    def __init__(self):
        self.snapshot = None
        self.yaw, self.pitch, self.zoom = -.25, -.15, 150.
        self.pan = np.zeros(2)
        self.drag = None
        self.pixel = None
        self.frozen = False
        self.opened = False
        self.depth_colors = False
        self.received_at = None
        self._cache = None
        self._dirty = True

    def set_snapshot(self, snapshot):
        if snapshot is not None and not self.frozen and snapshot is not self.snapshot:
            self.snapshot = snapshot
            self.received_at = time.monotonic()
            self._dirty = True

    def sample(self, x, y):
        s = self.snapshot
        if s is None or not (0 <= y < s['depth'].shape[0] and 0 <= x < s['depth'].shape[1]):
            return None
        if not s['valid'][y, x] or not np.isfinite(s['points'][y, x]).all():
            return None
        return {'rectified_z_m': float(s['depth'][y, x]),
                'range_m': float(np.linalg.norm(s['points'][y, x])),
                'xyz_left_m': s['points'][y, x].tolist()}

    def mouse(self, event, x, y, flags, _=None):
        if event == cv2.EVENT_LBUTTONDOWN:
            if 0 <= x < self.W and self.H <= y < 2*self.H:
                self.drag = (x, y)
            elif 30 <= y < self.H and self.snapshot is not None:
                h, w = self.snapshot['depth'].shape
                self.pixel = (min(w-1, int((x % self.W)*w/self.W)),
                              min(h-1, int((y-30)*h/(self.H-30))))
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag = None
        elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
            dx, dy = x-self.drag[0], y-self.drag[1]
            if flags & cv2.EVENT_FLAG_SHIFTKEY:
                self.pan += [dx, dy]
            else:
                self.yaw += dx*.008; self.pitch = float(np.clip(self.pitch+dy*.008, -1.5, 1.5))
            self.drag = (x, y)
        elif event == cv2.EVENT_MOUSEWHEEL and x < self.W and y >= self.H:
            delta = (flags >> 16) & 0xffff
            if delta >= 0x8000:
                delta -= 0x10000
            self.zoom = float(np.clip(self.zoom * (1.15 if delta > 0 else 1/1.15), 10, 3000))
        self._dirty = True

    def key(self, key):
        if key == ord('f'):
            self.frozen = not self.frozen
        elif key == ord('r'):
            self.yaw, self.pitch, self.zoom = -.25, -.15, 150.
            self.pan[:] = 0
        elif key == ord('c'):
            self.depth_colors = not self.depth_colors
        else:
            return
        self._dirty = True

    def cloud(self):
        s = self.snapshot
        if s is None:
            return np.empty((0, 3)), np.empty((0, 3), np.uint8)
        lo, hi = s['depth_range']
        ok = s['valid'] & np.isfinite(s['points']).all(axis=2)
        ok &= (s['depth'] >= lo) & (s['depth'] <= hi)
        p = s['points'][ok]
        colors = s['depth_color'][ok] if self.depth_colors else s['rgb'][ok]
        step = max(1, (len(p)+49999)//50000)
        return p[::step], colors[::step]

    def _cloud_panel(self):
        canvas = np.full((self.H, self.W, 3), (25, 22, 19), np.uint8)
        p, c = self.cloud()
        if len(p):
            center = np.median(p, axis=0)
            R = cv2.Rodrigues(np.array([self.pitch, self.yaw, 0.]))[0]
            rotated = (p-center) @ R.T
            uv = rotated[:, :2]*self.zoom + [self.W/2, self.H/2] + self.pan
            uv = np.clip(uv, -100000, 100000).astype(int)
            good = (uv[:, 0] >= 0) & (uv[:, 0] < self.W) & (uv[:, 1] >= 32) & (uv[:, 1] < self.H-28)
            # Farthest first; near points overwrite each projected pixel deterministically.
            ix = np.where(good)[0]; ix = ix[np.argsort(-rotated[ix, 2])]
            coords = uv[ix, 1]*self.W+uv[ix, 0]
            _, near = np.unique(coords[::-1], return_index=True)
            ix = ix[len(ix)-1-near]
            canvas[uv[ix, 1], uv[ix, 0]] = c[ix]
        self._text(canvas, '3D scene | '+('depth color' if self.depth_colors else 'RGB color'), 22)
        self._text(canvas, 'Drag orbit | Wheel zoom | Shift-drag pan | R reset', self.H-10, .45)
        return canvas

    @staticmethod
    def _text(image, text, y, size=.55, color=(215, 220, 225)):
        cv2.putText(image, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)

    def render(self):
        if not self._dirty and self._cache is not None:
            result = self._cache.copy()
        else:
            result = np.full((2*self.H+38, 2*self.W, 3), 20, np.uint8)
            s = self.snapshot
            if s is None:
                self._text(result, 'Waiting for calibrated RGB-D inference...', 40)
            else:
                for i, (im, title) in enumerate([(s.get('rgb_overlay', s['rgb']), 'RGB | object detection'),
                                                (s['depth_color'], 'DEPTH | near red - far blue')]):
                    x = i*self.W
                    result[30:self.H, x:x+self.W] = cv2.resize(im, (self.W, self.H-30))
                    panel = result[:self.H, x:x+self.W]
                    self._text(panel, title, 22)
                    if self.pixel is not None:
                        px, py = self.pixel; h, w = s['depth'].shape
                        cv2.drawMarker(panel, (int(px*self.W/w), 30+int(py*(self.H-30)/h)),
                                       (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
                result[self.H:2*self.H, :self.W] = self._cloud_panel()
                info = result[self.H:2*self.H, self.W:]
                self._text(info, 'IMU / Frame information', 25, .65)
                y = 58
                for name, label, unit in [('gyroscope', 'Gyro', 'rad/s'),
                    ('linear_acceleration', 'Linear accel (gravity removed)', 'm/s^2'),
                    ('gravity', 'Gravity', 'm/s^2')]:
                    v = s['imu'].get(name)
                    self._text(info, label+' ['+unit+']', y); y += 22
                    text = ('  '.join(f'{q:+.3f}' for q in v['values'][:3])+
                            f"   dt={v.get('dt_ms', '?')}ms") if v else 'unavailable'
                    self._text(info, text, y, .5); y += 28
                self._text(info, f"Stereo skew {s['skew_ms']:+.2f}ms | valid {100*s['valid'].mean():.1f}%", y); y += 26
                self._text(info, f"Depth range {s['depth_range'][0]:g} - {s['depth_range'][1]:g}m | {s['scale_status']}", y); y += 26
                if self.pixel is not None:
                    hit = self.sample(*self.pixel)
                    self._text(info, f'Pixel {self.pixel}: '+('invalid depth' if hit is None else f"Z {hit['rectified_z_m']:.3f}m | range {hit['range_m']:.3f}m"), y, .5)
                    y += 24
                    if hit:
                        self._text(info, 'XYZ left: '+', '.join(f'{v:+.3f}' for v in hit['xyz_left_m'])+' m', y, .5)
                orientation = s['imu'].get('game_rotation_vector')
                if orientation and len(orientation.get('values', [])) >= 3:
                    from lib_phone_motion import R_world_from_device
                    R = R_world_from_device(orientation['values'])
                    origin = np.array([548, 337])
                    for axis, color, label in zip(R.T, [(90, 90, 255), (90, 255, 90), (255, 170, 90)], 'XYZ'):
                        end = origin + np.array([axis[0]+.4*axis[2], -axis[1]-.3*axis[2]])*40
                        cv2.arrowedLine(info, tuple(origin), tuple(end.astype(int)), color, 2, tipLength=.2)
                        cv2.putText(info, label, tuple(end.astype(int)), 0, .45, color, 1)
                    cv2.putText(info, 'Device orientation', (470, 390), 0, .4, (200, 200, 200), 1)
                self._text(info, 'IMU axes: Android device | XYZ: original left camera', self.H-15, .45)
            self._cache = result.copy(); self._dirty = False
        age = '' if self.received_at is None else f' | display age {time.monotonic()-self.received_at:.1f}s'
        self._text(result, ('FROZEN' if self.frozen else 'LIVE')+age+' | F freeze | C cloud color | E export | Q quit | Click RGB/depth to measure', 2*self.H+25, .5)
        return result

    def show(self):
        if not self.opened:
            cv2.namedWindow(self.TITLE, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.TITLE, self.mouse)
            self.opened = True
        cv2.imshow(self.TITLE, self.render())

    def save(self, directory):
        s = self.snapshot
        if s is None:
            return False
        directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
        depth = np.where(s['valid'] & np.isfinite(s['depth']), s['depth'], 0)
        mm = np.where((depth > 0) & (depth <= 65.535), np.rint(depth*1000), 0).astype(np.uint16)
        cv2.imwrite(str(directory/'rgb.png'), s['rgb'])
        cv2.imwrite(str(directory/'depth_mm.png'), mm)
        cv2.imwrite(str(directory/'depth_color.png'), s['depth_color'])
        cv2.imwrite(str(directory/'viewer.png'), self.render())
        np.savez_compressed(directory/'rgbd.npz', depth_m=depth.astype(np.float32),
                            valid=s['valid'], points_left_m=s['points'], K_rect=s['K_rect'], R1=s['R1'])
        # Export geometry/colors of the frame, independent of current orbit and C toggle.
        from lib_pointcloud_export import write_ply
        from lib_pointcloud_view import write_viewer
        lo, hi = s['depth_range']
        ok = s['valid'] & np.isfinite(s['points']).all(axis=2) & (depth >= lo) & (depth <= hi)
        points, colors = s['points'][ok], s['rgb'][ok][:, ::-1]
        write_ply(directory/'scene.ply', points, colors)
        write_viewer(directory/'scene.html', points, colors, {'objects': [], 'trajectory': []})
        meta = {k: s[k] for k in ('timestamp_ns', 'skew_ms', 'depth_range', 'scale_status', 'imu')}
        meta.update(source='phone stereo FFS', depth_unit_m=.001, invalid_depth=0,
                    depth_frame='rectified left Z', color_frame='rectified left, pixel aligned to depth',
                    points_frame='original left camera: x right, y down, z forward; meters',
                    imu_frame='Android device axes; linear_acceleration excludes gravity',
                    K_rect=s['K_rect'].tolist(), R1=s['R1'].tolist())
        (directory/'frame.json').write_text(json.dumps(meta, indent=2, allow_nan=False))
        return True
