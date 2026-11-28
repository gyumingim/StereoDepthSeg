"""kfsel 2026-11-11: keyframe selection rules that the phone can run live (causal, small look-ahead window only).

Inputs per VIO-fed frame, in time order: frame index, frame time t_ns, live VIO pose T_world_cam0 (status tracking),
optionally the image (for sharpness) and the gyro (imu.csv type 4, rad/s).

  interval : every --kf-step-th VIO-fed frame (the old rule, kf_ba.py --kf-from-vio --kf-step 10).
  motion   : a keyframe is due when the camera has moved >= dist_m or rotated >= rot_deg since the LAST KEYFRAME (VIO pose).
             The due frame opens a window of `win` VIO-fed frames (the due frame + win-1 following ones, ~17 ms each at 60 fps);
             when the window is full the best frame in it becomes the keyframe (blur score below) and the next keyframe is measured
             from that frame's pose. The very first tracking frame opens the first window.
  blur score (which frame of the window):
    none : the due frame (win is ignored)
    lap  : highest variance of the Laplacian of the grey image downscaled to 480x270 (INTER_AREA)
    gyro : lowest mean |gyro| (rad/s) over the exposure [t_mid - exp/2 - skew/2, t_mid + exp/2 + skew/2]
  max_gyro (rad/s, 0 = off): frames above it are not taken as keyframe while a better one is in the window (only a ranking key:
    the window still always emits one keyframe so coverage never stalls).
"""
import numpy as np
import cv2


def rot_deg(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def lap_var(img):
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if g.shape[1] > 480:
        g = cv2.resize(g, (480, int(round(g.shape[0] * 480 / g.shape[1]))), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(g, cv2.CV_32F).var())


class Gyro:
    """mean |omega| over a time window from imu.csv (type 4 rows = gyro rad/s)."""

    def __init__(self, imu_csv):
        t, w = [], []
        with open(imu_csv) as f:
            next(f)
            for line in f:
                p = line.split(",")
                if p[0] == "4":
                    t.append(int(p[1])); w.append((float(p[2]), float(p[3]), float(p[4])))
        o = np.argsort(np.array(t, np.int64), kind="stable")
        self.t = np.array(t, np.int64)[o]
        self.w = np.linalg.norm(np.array(w, np.float64), axis=1)[o]

    def rate(self, t0, t1):
        a, b = np.searchsorted(self.t, [t0, t1])
        if b <= a:                                   # no sample inside: nearest one
            k = int(np.clip(np.searchsorted(self.t, (t0 + t1) // 2), 0, len(self.t) - 1))
            return float(self.w[k])
        return float(self.w[a:b].mean())


class MotionSelector:
    """feed() VIO-fed frames in time order; it returns a list of (frame, info) keyframes decided by that call.
    Holds at most `win` frames (the phone needs the same small buffer)."""

    def __init__(self, dist_m=0.03, rot_deg_=4.0, win=4, blur="lap", max_gyro=0.0):
        self.d, self.r, self.win, self.blur, self.max_gyro = dist_m, rot_deg_, (1 if blur == "none" else max(1, win)), blur, max_gyro
        self.last = None                             # pose of the last keyframe
        self.buf = []                                # open window: (frame, T, score dict, payload)

    def due(self, T):
        if self.last is None:
            return True
        return (np.linalg.norm(T[:3, 3] - self.last[:3, 3]) >= self.d) or (rot_deg(self.last[:3, :3], T[:3, :3]) >= self.r)

    def wants_image(self, T):
        """True when this frame will be in a window (so the caller decodes / scores it)."""
        return bool(self.buf) or self.due(T)

    def feed(self, fi, T, lap=None, gyro=None, payload=None):
        if not self.buf and not self.due(T):
            return []
        self.buf.append((fi, T, dict(lap=lap, gyro=gyro), payload))
        if len(self.buf) >= self.win:
            return [self._close()]
        return []

    def flush(self):
        return [self._close()] if self.buf else []

    def _key(self, c):
        s = c[2]
        over = 1 if (self.max_gyro > 0 and s["gyro"] is not None and s["gyro"] > self.max_gyro) else 0
        if self.blur == "lap":
            return (over, -s["lap"])
        if self.blur == "gyro":
            return (over, s["gyro"])
        return (over, 0)

    def _close(self):
        k = min(range(len(self.buf)), key=lambda j: (self._key(self.buf[j]), j))
        fi, T, s, pay = self.buf[k]
        info = dict(frame=fi, pos_in_window=k, window=[b[0] for b in self.buf], lap=s["lap"], gyro=s["gyro"],
                    lap_window=[b[2]["lap"] for b in self.buf], gyro_window=[b[2]["gyro"] for b in self.buf])
        if self.last is not None:
            info["moved_m"] = float(np.linalg.norm(T[:3, 3] - self.last[:3, 3])); info["rot_deg"] = rot_deg(self.last[:3, :3], T[:3, :3])
        self.last = T
        self.buf = []
        return fi, info, pay
