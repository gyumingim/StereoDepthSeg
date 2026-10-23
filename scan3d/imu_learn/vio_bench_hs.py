#!/usr/bin/env python3
"""VIO testbed on the 240 fps board recordings: feed (subsampled, undistorted, resized) frames + raw IMU into the PC
OpenVINS bridge (ov_bridge) and score its camera trajectory against the board ground truth of the same recording.

Variants (same backend, only one thing changes at a time — the 2026 'same-backend control' advice):
  --fps 30/60/120/240     camera rate fed to the VIO (frame nearest to each 1/fps time step of t_mid — by TIME, not by
                          frame index, so dropped frames do not make the rate uneven; delivered rate/gaps reported)
  --size 640x360 ...      VIO image size (pinhole after undistortion, aspect 16:9)
  --npts / --clones       OpenVINS feature count / sliding-window clones
  --noise cfg|meas|allan  IMU noise densities: the app config (cfg), measured from this recording (meas) or the Allan
                          values of the long still recording (allan, runs/gyro_bias_stability.json)
  --imu-calib RESULT.json apply a learned physical calibration (train_imu result 'calib': G, bg, A, ba) to the IMU
  --dropout T:D,...       camera off for D s starting T s after the first frame (IMU keeps running)
  --imu-raw               use the uncalibrated IMU (Android types 16/35, values 0..2) instead of 1/4
  --init-cfg strict|strict-dyn|loose
                          OpenVINS initialization values. strict (default, 2026-10-20) = make_ov_config.init_params
                          (the live app path, --init static): still = 0.3 deg disparity, 0.3 m/s^2, no dynamic init;
                          strict-dyn = make_ov_config --init dynamic values; loose = the OLD bench (before 2026-10-20):
                          template config/mono (15 px, 1.0 m/s^2) + init_dyn_use true + 15 px * W/640 (for comparison).
  --start-off S           first VIO frame S seconds later (IMU feed unchanged) = another initialization; --max-secs
                          counts from the first VIO frame
  --cfg-over k=v,...      extra OpenVINS config overrides (key must exist in the template; e.g. zupt_chi2_multipler=1)
  --still-hold W:G:A      external IMU-only stillness detector (2026-10-21 set-down mitigation): if over the trailing
                          W s of IMU (up to the frame time) max |gyro| < G rad/s and std |acc| < A m/s^2, the frame is
                          replaced by the frame latched when the still began (identical image -> disparity 0 -> OpenVINS
                          ZUPT passes). Releases on the first frame whose window fails. Off by default (output unchanged).
                          Optional 4th+ fields (2026-10-21 H tuning, docs/h_tune_plan.md; none = old behaviour):
                          m = armed only after the first fed frame whose window is NOT still (skips the start still /
                          initialization), mD = only after D s of continuous not-still fed frames, tS = armed only
                          from clip second S on (clip = from the first VIO frame).
                          iD (2026-10-21 arm-after-init test, docs/h_arm_init_plan.md) = armed only for frames later
                          than the bridge's first 'tracking' pose (= OpenVINS initialized) + D s. Until then the bench
                          waits for the bridge's pose of the previous fed frame before each decision (deterministic;
                          bridge sources unchanged), so the arming frame does not depend on bridge speed.
  --focus-comp [--focus-k 0.0080] [--drop-lens-moving]
                          (2026-10-22, off by default) per-frame focus from results_cam5.csv; the frame is rescaled
                          about the principal point by 1/(1 + k (D - D_ref)) (D_ref = lens file focus) so the VIO sees
                          the lens-file intrinsics; --drop-lens-moving drops frames during / 40 ms after lens motion
  --focus-comp --focus-table config/lens/hs_1920x1080_focus_table_sweep.json
                          (2026-10-22, off by default) instead of the k scale, undistort each frame with the table's
                          per-focus K/distortion, blended (pixel maps, linear in dpt) at the frame's focus. _sweep =
                          f2.22 lens x same-pose image scale per step (recommended); hs_1920x1080_focus_table.json =
                          calibrations at 0.37/1.0/3.33 dpt (not recommended: docs/focus_calib_20261022.md)
  --gyro-rebias / --no-gyro-rebias
                          re-estimate the raw gyro bias causally at every still >= 4 s (imu_learn/gyro_rebias.py,
                          docs/gyro_rebias_apply.md). ON by default with --imu-raw since 2026-10-22 (user decision), also
                          for run(a) callers whose Namespace has no gyro_rebias (or None); --no-gyro-rebias /
                          a.gyro_rebias = False reproduces older runs (all runners written before then pass it).
  --frame-cache           (2026-10-23, off by default) read the fed images from recordings/<rec>/cache/ (imu_learn/
                          frame_cache.py: decoded once, after gray + undistort/resize = bit-identical bridge input);
                          a run missing any frame decodes as usual and adds them. Ignored with --focus-comp.
  --gt auto|spl|base|rs   scoring GT: auto = imu_gt_spl if present else imu_gt (both camera-only); rs = imu_gt_rs
                          (rolling shutter undone with the gyro -> partly self-scoring, comparison only)
Time: frames are sent on the IMU clock (t_mid - dt from align.json) and OpenVINS timeshift_cam_imu = 0, so the offset is
applied exactly once; the online estimate (dt_cam_imu_*_ms in the result) should stay near 0.
Metrics: ATE (SE3-aligned position RMSE, cm), scale (Sim3), RPE over 1 s, rotation error, drift at the end of each
dropout, OpenVINS time per frame (ms, PC) and the share of frames tracked.
  ../venv_ffs/bin/python imu_learn/vio_bench_hs.py recordings/hs_... --fps 60 --size 640x360 [--out runs/vio/NAME.json]
"""
import argparse
import csv
import json
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(SCAN3D))
sys.path.insert(0, str(HERE))
import lens_calib  # noqa: E402
import make_ov_config  # noqa: E402  (init_params: single source of the OpenVINS init thresholds)
from imu_gt import read_imu  # noqa: E402

BRIDGE = SCAN3D / "ov_bridge" / "build" / "ov_bridge"
TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
HDR = struct.Struct("<III")
MAGIC = 0x4F564252


def lens_path(rec, meta):
    own = rec / "imu_gt" / "lens.json"
    fd = meta.get("focus_dpt_request") or -1
    shared = SCAN3D / "config" / "lens" / f"hs_{meta['width']}x{meta['height']}_f{fd:.2f}.json"
    return own if own.exists() else shared


def lens_for(rec, meta):
    return lens_calib.load(lens_path(rec, meta))[:2]


class FocusComp:
    """--focus-comp / --drop-lens-moving (2026-10-22, docs/focus_scale_20261022.md; off by default = output unchanged).
    The image grows by k per dioptre of focus (fx, fy x (1 + k (D - D_ref)), measured k = 0.0080, linear 0.37-3.33 dpt;
    principal point and distortion assumed unchanged = not measured). Per frame the focus D of the nearest capture
    result (results_cam5.csv sensor_ts_ns vs the frame's physical_ts_ns, same sensor clock) gives s = 1 + k (D - D_ref);
    the frame is rescaled about the principal point by 1/s so the VIO always sees the lens-file intrinsics (D_ref = the
    lens file's focus_dpt). Done inside the undistort map (sample the raw frame at c + s (x - c)) = one interpolation.
    Frames whose largest pixel shift (|s - 1| x half-diagonal) is below tol_px keep the original map (bit-identical).
    Lens moving: the nearest result has lens_state 1, or a lens_state 1 result lies at most drop_ms before the frame.
    --focus-table FILE (2026-10-22, docs/focus_calib_20261022.md; off by default): instead of the linear k scale, each
    frame is undistorted with the intrinsics (K, distortion) of fixed focus nodes (config/lens/
    hs_1920x1080_focus_table_sweep.json), interpolated at the frame's focus D: per node the raw-frame sample map for the same
    output (Knew, size) is built, and the map at D is the linear blend (in dpt) of the two bracketing node maps
    (= linear interpolation of the projection, not of the distortion coefficients, which are ill-conditioned for the
    rational model). D outside the nodes is clamped to the end node. Frames whose map differs from the lens-file map
    by less than tol_px everywhere keep the original map."""

    def __init__(self, rec, rows, K, dist, Knew, raw_size, size, d_ref, k, tol_px, drop_ms, comp, drop, table=None):
        res = list(csv.DictReader(open(rec / "results_cam5.csv")))
        rt = np.array([int(r["sensor_ts_ns"]) for r in res], np.int64)
        col = "lens_state" if res and "lens_state" in res[0] else "lens_moving"   # older app: lens_moving only
        o = np.argsort(rt, kind="stable")                    # results are written out of order (and sometimes twice)
        self.rt = rt[o]
        self.fd = np.array([float(r["focus_dpt"]) for r in res])[o]
        self.mv = np.array([int(float(r.get(col) or 0)) for r in res])[o] == 1
        self.tf = np.array([int(r["physical_ts_ns"]) for r in rows], np.int64)
        self.d_ref, self.k, self.tol_px, self.drop_ns = float(d_ref), float(k), float(tol_px), int(drop_ms * 1e6)
        self.comp, self.drop, self.col = comp, drop, col
        self.half_diag = 0.5 * float(np.hypot(*raw_size))       # raw frame half-diagonal (largest-shift test)
        self.mx, self.my = cv2.initUndistortRectifyMap(K, dist, None, Knew, size, cv2.CV_32FC1)
        self.cx, self.cy = float(K[0, 2]), float(K[1, 2])
        self.maps = {}
        self.n_rescaled = self.n_dropped = 0
        self.s_used, self.d_seen = [], []
        self.table = None
        if table:
            tj = json.load(open(table))
            nodes = sorted(tj["entries"], key=lambda e: float(e["focus_dpt"]))
            self.table, self.tab_d = str(table), np.array([float(e["focus_dpt"]) for e in nodes])
            self.tab_maps = [cv2.initUndistortRectifyMap(np.asarray(e["K"], float), np.asarray(e["dist"], float), None,
                                                         Knew, size, cv2.CV_32FC1) for e in nodes]
            self.n_clamped, self.tab_shift = 0, []

    def table_map(self, D):
        """Raw-frame sample maps (float32) at focus D: linear blend of the two bracketing node maps (clamped)."""
        d = self.tab_d
        if len(d) == 1 or D <= d[0] or D >= d[-1]:
            m = self.tab_maps[0 if len(d) == 1 or D <= d[0] else -1]
            return m[0], m[1], len(d) > 1 and (D < d[0] or D > d[-1])
        i = int(np.clip(np.searchsorted(d, D) - 1, 0, len(d) - 2))
        w = float((D - d[i]) / (d[i + 1] - d[i]))
        (ax, ay), (bx, by) = self.tab_maps[i], self.tab_maps[i + 1]
        return ((1 - w) * ax + w * bx).astype(np.float32), ((1 - w) * ay + w * by).astype(np.float32), False

    def map_for_table(self, fi):
        D = float(self.fd[self.nearest(fi)])
        self.d_seen.append(D)
        key = round(D, 5)                                    # focus values are a few discrete steps
        if key not in self.maps:
            if len(self.maps) > 256:
                self.maps.clear()
            mx, my, clamped = self.table_map(D)
            shift = float(np.max(np.hypot(mx - self.mx, my - self.my)))
            self.maps[key] = (None if shift < self.tol_px else cv2.convertMaps(mx, my, cv2.CV_16SC2), clamped, shift)
        mp, clamped, shift = self.maps[key]
        self.n_clamped += int(clamped)
        self.tab_shift.append(shift)
        if mp is not None:
            self.n_rescaled += 1
        return mp

    def nearest(self, fi):
        t = self.tf[fi]
        j = int(np.clip(np.searchsorted(self.rt, t), 1, len(self.rt) - 1))
        return j - 1 if abs(self.rt[j - 1] - t) <= abs(self.rt[j] - t) else j

    def moving(self, fi):
        t = self.tf[fi]
        if self.mv[self.nearest(fi)]:
            return True
        j0, j1 = np.searchsorted(self.rt, t - self.drop_ns), np.searchsorted(self.rt, t, side="right")
        return bool(self.mv[j0:j1].any())

    def scale(self, fi):
        return 1.0 + self.k * (self.fd[self.nearest(fi)] - self.d_ref)

    def map_for(self, fi):
        """None = keep the original map; else (m1, m2) sampling the raw frame at c + s (x - c)."""
        if self.table is not None:
            return self.map_for_table(fi)
        s = self.scale(fi)
        self.d_seen.append(float(self.fd[self.nearest(fi)]))
        if abs(s - 1.0) * self.half_diag < self.tol_px:
            return None
        key = float(round(float(s), 7))                      # 1e-7 x 1100 px = 1e-4 px: same map within that
        if key not in self.maps:
            if len(self.maps) > 256:
                self.maps.clear()
            self.maps[key] = cv2.convertMaps((self.cx + key * (self.mx - self.cx)).astype(np.float32),
                                             (self.cy + key * (self.my - self.cy)).astype(np.float32), cv2.CV_16SC2)
        self.n_rescaled += 1
        self.s_used.append(key)
        return self.maps[key]

    def summary(self):
        img = 1.0 / np.asarray(self.s_used) if self.s_used else None
        d = np.asarray(self.d_seen)
        tab = {}
        if self.table is not None:
            sh = np.asarray(self.tab_shift)
            tab = dict(focus_table=self.table, focus_table_dpt=[round(float(x), 4) for x in self.tab_d],
                       frames_focus_clamped=self.n_clamped,
                       table_map_shift_px_max=round(float(sh.max()), 4) if len(sh) else None)
        return dict(focus_comp=self.comp, focus_k=self.k, focus_d_ref=round(self.d_ref, 4), focus_tol_px=self.tol_px, **tab,
                    focus_col_moving=self.col, frames_rescaled=self.n_rescaled,
                    image_scale_min=round(float(img.min()), 6) if img is not None else None,
                    image_scale_max=round(float(img.max()), 6) if img is not None else None,
                    focus_fed_min=round(float(d.min()), 4) if len(d) else None,
                    focus_fed_max=round(float(d.max()), 4) if len(d) else None,
                    drop_lens_moving=self.drop, drop_lens_moving_ms=self.drop_ns / 1e6,
                    frames_dropped_lens_moving=self.n_dropped)


ALLAN_JSON = SCAN3D / "runs" / "gyro_bias_stability.json"


def allan_noise(path=ALLAN_JSON):
    """Allan-deviation noise of the 43 min still recording (static_imu_20261020_095113, raw types 16/35): white noise =
    mean over axes of N, random walk = mean over the axes where the Allan fit found K (rate random walk). Units in the
    JSON are deg/s/sqrt(Hz), deg/s/sqrt(s), m/s^2/sqrt(Hz), m/s^3/sqrt(Hz) -> OpenVINS rad units. These are the sensor
    floor (not inflated); secondary noise setting only (overnight 2026-10-20 plan)."""
    al = json.load(open(path))["allan"]

    def mean(sensor, key):
        v = [ax[key] for ax in al[sensor].values() if isinstance(ax, dict) and ax.get(key) is not None]
        return float(np.mean(v))
    d2r = np.pi / 180
    return dict(gyroscope_noise_density=mean("gyro_raw", "N") * d2r, gyroscope_random_walk=mean("gyro_raw", "K") * d2r,
                accelerometer_noise_density=mean("accel_raw", "N"), accelerometer_random_walk=mean("accel_raw", "K"))


def measured_noise(acc, gyr):
    """White-noise density from sample-to-sample differences in the quietest 10 % of the recording (sigma·sqrt(dt)),
    random walk kept at 1/10 of the density (not observable from minutes of hand-held data; estimate)."""
    dt = np.median(np.diff(gyr[:, 0])) / 1e9
    w = np.linalg.norm(gyr[:, 1:4], axis=1)
    quiet = w < np.quantile(w, 0.10)
    sg = np.std(np.diff(gyr[:, 1:4], axis=0)[quiet[1:]], axis=0).mean() / np.sqrt(2)
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    sa = np.std(np.diff(a_i, axis=0)[quiet[1:]], axis=0).mean() / np.sqrt(2)
    nd_g, nd_a = sg * np.sqrt(dt), sa * np.sqrt(dt)
    return dict(gyroscope_noise_density=nd_g, gyroscope_random_walk=nd_g / 10,
                accelerometer_noise_density=nd_a, accelerometer_random_walk=nd_a / 10)


def init_values(init_cfg, K, W):
    """OpenVINS init keys for the config. strict/strict-dyn come from make_ov_config.init_params (the app path), with
    the disparity threshold converted by THIS camera's focal length; loose = the old bench values (template + overrides)."""
    if init_cfg == "loose":
        return {"init_dyn_use": "true", "init_max_disparity": f"{15.0 * W / 640:.1f}"}   # rest from the template
    ip = make_ov_config.init_params(float(K[0, 0]), dynamic=(init_cfg == "strict-dyn"))
    return {"init_window_time": f"{ip['init_window_time']}", "init_imu_thresh": f"{ip['init_imu_thresh']}",
            "init_max_disparity": f"{ip['init_max_disparity']:.2f}",
            "init_dyn_use": "true" if ip["init_dyn_use"] else "false"}


def write_config(d, K, size, fps, R_ci, p_CinI, timeshift, noise, npts, clones, init_cfg="strict", init_over=None):
    W, H = size
    txt = TEMPLATE.read_text()
    rep = {"track_frequency:": f"track_frequency: {float(fps)}", "num_pts:": f"num_pts: {int(npts)}",
           "max_clones:": f"max_clones: {int(clones)}",
           "verbosity:": 'verbosity: "WARNING"', "record_timing_information:": "record_timing_information: false"}
    iv = init_values(init_cfg, K, W)
    iv.update(init_over or {})                                   # --init-window / --init-disp (2026-10-21 init window test)
    rep.update({k + ":": f"{k}: {v}" for k, v in iv.items()})
    missing = [k for k in rep if not any(line.strip().startswith(k) for line in txt.splitlines())]
    if missing:
        sys.exit(f"template {TEMPLATE} lacks keys {missing}")
    out = []
    for line in txt.splitlines():
        key = next((k for k in rep if line.strip().startswith(k)), None)
        out.append(rep[key] if key else line)
    (d / "estimator_config.yaml").write_text("\n".join(out) + "\n")
    R_CtoI = np.asarray(R_ci).T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_CtoI, p_CinI
    rows = "\n".join("    - [" + ", ".join(f"{v:.12g}" for v in r) + "]" for r in T)
    (d / "kalibr_imucam_chain.yaml").write_text(
        "%YAML:1.0\ncam0:\n  T_imu_cam:\n" + rows + "\n  cam_overlaps: []\n  camera_model: pinhole\n"
        "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n  distortion_model: radtan\n"
        f"  intrinsics: [{K[0, 0]:.6f}, {K[1, 1]:.6f}, {K[0, 2]:.6f}, {K[1, 2]:.6f}]\n  resolution: [{W}, {H}]\n"
        f"  rostopic: /cam0/image_raw\n  timeshift_cam_imu: {timeshift:.6f}\n")
    imu = (SCAN3D / "config" / "mono" / "kalibr_imu_chain.yaml").read_text().splitlines()
    out = []
    for line in imu:
        k = line.strip().split(":")[0]
        out.append(f"  {k}: {noise[k]:.4e}" if k in noise else line)
    (d / "kalibr_imu_chain.yaml").write_text("\n".join(out) + "\n")
    used = {}                                                    # what OpenVINS will actually read (recorded in results)
    for line in (d / "estimator_config.yaml").read_text().splitlines():
        body = line.split("#")[0]
        k = body.split(":")[0].strip()
        if ":" in body and (k.startswith("init_") or k.startswith("zupt_") or k in ("max_clones", "try_zupt")):
            used[k] = body.split(":", 1)[1].strip()
    return used


def umeyama(src, dst, scale=False):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S, D = src - mu_s, dst - mu_d
    U, s, Vt = np.linalg.svd(D.T @ S / len(src))
    Sg = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        Sg[2, 2] = -1
    R = U @ Sg @ Vt
    c = (s * np.diag(Sg)).sum() / S.var(0).sum() if scale else 1.0
    return R, c, mu_d - c * R @ mu_s


def gap_stats(t_ns, period_ns):
    """Time-gap statistics of a frame time series: gaps = intervals > 1.5 x the nominal period (frame_idx and the
    high-speed burst frame_number cannot show drops; only the time stamps can)."""
    t = np.asarray(t_ns, np.int64)
    if len(t) < 2:
        return dict(n=int(len(t)))
    d = np.diff(t)
    gap = d > 1.5 * period_ns
    return dict(n=int(len(t)), secs=round(float((t[-1] - t[0]) / 1e9), 2),
                fps=round(float((len(t) - 1) / ((t[-1] - t[0]) / 1e9)), 2), nominal_fps=round(1e9 / period_ns, 2),
                gaps=int(gap.sum()), missing_est=int(np.maximum(np.round(d[gap] / period_ns) - 1, 0).sum()),
                max_gap_ms=round(float(d.max() / 1e6), 2), p99_ms=round(float(np.percentile(d, 99) / 1e6), 2))


class StillHold:
    """--still-hold W:G:A[:m][:tS] decision per fed frame (causal: IMU samples up to the frame time only). still = over
    the trailing W s of IMU max |gyro| < G rad/s and std |acc| < A m/s^2 (the 2026-10-21 rule, unchanged). Arming
    (none given = armed from the start = old behaviour): m[D] = after D s (default 0) of continuous not-still fed
    frames; tS = from clip second S on. step() returns True if this frame is to be replaced by the latched frame."""

    def __init__(self, spec, t_imu, g_i, a_i):
        f = spec.split(":")
        hw, hg, ha = (float(v) for v in f[:3])
        opts = f[3:]
        num = (lambda x: x.replace(".", "", 1).isdigit())
        bad = [o for o in opts if not (o == "m" or (o[:1] in "mti" and num(o[1:])))]
        io = [o for o in opts if o[:1] == "i"]
        if bad or (io and len(opts) > 1):
            sys.exit(f"--still-hold: unknown option(s) {bad} (iD cannot be combined)")
        mo = [o for o in opts if o[:1] == "m"]
        self.arm_motion = bool(mo)
        self.arm_motion_s = float(mo[0][1:] or 0) if mo else 0.0
        self.move_t0 = None
        self.arm_t = next((float(o[1:]) for o in opts if o[:1] == "t"), None)
        self.arm_init = float(io[0][1:]) if io else None     # iD: armed for frames > init + D s (init_ns from the bridge)
        self.init_ns = None
        self.armed = not (self.arm_motion or self.arm_t is not None or self.arm_init is not None)
        self.t_imu = t_imu
        nwin = max(2, int(round(hw * 1e9 / float(np.median(np.diff(t_imu))))))
        sw = np.lib.stride_tricks.sliding_window_view
        gmax, astd = np.full(len(t_imu), np.inf), np.full(len(t_imu), np.inf)
        gmax[nwin - 1:] = sw(np.linalg.norm(g_i, axis=1), nwin).max(1)
        astd[nwin - 1:] = sw(np.linalg.norm(a_i, axis=1), nwin).std(1)
        self.ok = (gmax < hg) & (astd < ha)

    def needs_init(self):
        """True while the iD arming still waits for the bridge's first tracking pose."""
        return self.arm_init is not None and self.init_ns is None

    def step(self, tci, clip_s):
        jh = int(np.searchsorted(self.t_imu, tci, side="right")) - 1
        still = bool(jh >= 0 and self.ok[jh])
        if not self.armed and self.arm_init is not None:
            if self.init_ns is not None and tci > self.init_ns and tci >= self.init_ns + self.arm_init * 1e9 - 1:
                self.armed = True
        elif not self.armed:
            if still:
                self.move_t0 = None
            elif self.move_t0 is None:
                self.move_t0 = clip_s
            if (self.arm_motion and not still and clip_s - self.move_t0 >= self.arm_motion_s - 1e-9) or \
                    (self.arm_t is not None and clip_s >= self.arm_t - 1e-9):
                self.armed = True
        return still and self.armed


def pick_by_time(t_frames, t_start, t_end, fps, period_ns):
    """Indices of the frames nearest to the time grid t_start + n/fps: each within one camera period of its grid
    step (and half a VIO period), at least 0.75 VIO period after the previous pick. A grid step inside a long drop
    gets no frame instead of shifting the whole later sequence, and no short (< 0.75/fps) intervals appear after a drop."""
    step = 1e9 / fps
    grid = t_start + np.arange(0, int((t_end - t_start) / step) + 1) * step
    j = np.clip(np.searchsorted(t_frames, grid), 1, len(t_frames) - 1)
    j = np.where(np.abs(t_frames[j - 1] - grid) <= np.abs(t_frames[j] - grid), j - 1, j)
    tol = min(step / 2, 1.01 * period_ns)
    ok = (np.abs(t_frames[j] - grid) <= tol) & (t_frames[j] >= t_start) & (t_frames[j] <= t_end)
    out, last = [], None
    for i in np.unique(j[ok]):
        if last is None or t_frames[i] - last >= 0.75 * step:
            out.append(i)
            last = t_frames[i]
    return np.asarray(out, int)


def pick_gt(rec, mode):
    """Scoring GT dir. auto: camera-only spline GT (imu_gt_spl) if present, else camera-only imu_gt."""
    names = {"spl": "imu_gt_spl", "base": "imu_gt", "rs": "imu_gt_rs"}
    if mode == "auto":
        d = rec / "imu_gt_spl"
        d = d if (d / "gt.npz").exists() else rec / "imu_gt"
    else:
        d = rec / names.get(mode, mode)
    if not (d / "gt.npz").exists():
        sys.exit(f"GT not found: {d}/gt.npz")
    return d


def undistort_maps(rec, meta, size):
    """The default undistort maps (lens file, fed size): m1, m2 (CV_16SC2), Knew, (W, H). Also used by frame_cache.py."""
    K, dist = lens_for(rec, meta)
    Wf, Hf = int(meta["width"]), int(meta["height"])
    W, H = (int(v) for v in size.split("x"))
    Knew, _ = cv2.getOptimalNewCameraMatrix(K, dist, (Wf, Hf), 0, (W, H))
    m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, Knew, (W, H), cv2.CV_16SC2)
    return m1, m2, Knew, (W, H)


def fed_selection(rec, meta, rows, fps, start_off, max_secs):
    """Clip window and fed frame indices (by time, pick_by_time). Also used by frame_cache.py (same selection)."""
    t_frames = np.array([int(r["t_mid_ns"]) for r in rows])
    t_start = t_frames[0]
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        t_start = max(t_start, json.load(open(trim))["start_ns"])
    t_start = t_start + int(start_off * 1e9)                 # IMU feed still starts 2 s before the (later) first frame
    if max_secs:
        t_end = t_start + int(max_secs * 1e9)
    else:
        t_end = t_frames[-1]
    t_end = min(t_end, t_frames[-1])
    period = 1e9 / float(meta.get("save_fps") or 240)
    sel = pick_by_time(t_frames, t_start, t_end, fps, period)
    return dict(t_frames=t_frames, t_start=t_start, t_end=t_end, period=period, sel=sel)


def run(a):
    rec = Path(a.rec)
    meta = json.load(open(rec / "meta.json"))
    K, dist = lens_for(rec, meta)
    Wf, Hf = int(meta["width"]), int(meta["height"])
    m1, m2, Knew, (W, H) = undistort_maps(rec, meta, a.size)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_ns, R_ci = int(al["dt_ns"]), np.asarray(al["R_ci"])
    start_off = float(getattr(a, "start_off", 0.0) or 0.0)
    fs = fed_selection(rec, meta, rows, a.fps, start_off, a.max_secs)
    t_frames, t_start, t_end, period, sel = fs["t_frames"], fs["t_start"], fs["t_end"], fs["period"], fs["sel"]
    try:
        acc, gyr = read_imu(rec / "imu.csv", raw=a.imu_raw)
    except ValueError as e:
        sys.exit(str(e))
    print(f"IMU: {'raw types 16/35 (v0..2)' if a.imu_raw else 'Android-calibrated types 1/4'}", file=sys.stderr)
    # cfg = the app's values (make_ov_config.IMU_NOISE, same numbers as before 2026-10-20, now imported)
    if a.noise == "meas":
        noise = {k: v * a.noise_scale for k, v in measured_noise(acc, gyr).items()}
    elif a.noise == "allan":
        noise = {k: v * a.noise_scale for k, v in allan_noise().items()}
    else:
        noise = dict(make_ov_config.IMU_NOISE)
    a_i = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
    g_i = gyr[:, 1:4].copy()
    rebias_log = None
    rb = getattr(a, "gyro_rebias", None)                      # None / missing = default: ON with raw IMU (2026-10-22)
    if rb is None:
        rb = bool(a.imu_raw)
    if rb:                                                     # causal per-still gyro bias (imu_learn/gyro_rebias.py)
        if not a.imu_raw:
            sys.exit("--gyro-rebias needs --imu-raw")
        from gyro_rebias import correct_array
        g_i, _, rebias_log = correct_array(gyr[:, 0].astype(np.int64), g_i)
    p_CinI = np.zeros(3)
    if a.imu_calib:
        cal = json.load(open(a.imu_calib))["calib"]
        g_i = g_i @ np.asarray(cal["G"]).T + np.asarray(cal["bg"])
        a_i = a_i @ np.asarray(cal["A"]).T + np.asarray(cal["ba"])
        p_CinI = np.asarray(cal.get("r", [0, 0, 0]))
    t_imu = gyr[:, 0].astype(np.int64)
    drop = []
    for s in (a.dropout.split(",") if a.dropout else []):
        t0, d = (float(v) for v in s.split(":"))
        drop.append((t_start + int(t0 * 1e9), t_start + int((t0 + d) * 1e9)))
    k = max(1, int(round(float(meta.get("save_fps") or 240) / a.fps)))
    old_idx = np.nonzero((t_frames >= t_start) & (t_frames <= t_end) & (np.arange(len(t_frames)) % k == 0))[0]
    frames = dict(rec_all=gap_stats(t_frames, period),
                  rec_window=gap_stats(t_frames[(t_frames >= t_start) & (t_frames <= t_end)], period),
                  fed=gap_stats(t_frames[sel], 1e9 / a.fps),
                  index_every_k_would_be=gap_stats(t_frames[old_idx], 1e9 / a.fps))
    print(f"frames: recording {frames['rec_all']}\n  fed by time {frames['fed']}", file=sys.stderr)
    selected = np.zeros(len(t_frames), bool)
    selected[sel] = True
    fc = None
    if getattr(a, "focus_comp", False) or getattr(a, "drop_lens_moving", False):
        lj = lens_calib.load(lens_path(rec, meta))[2]
        d_ref = lj.get("focus_dpt")
        if d_ref is None or d_ref <= 0:
            d_ref = meta.get("focus_dpt_request")
        if a.focus_comp and (d_ref is None or d_ref <= 0):
            sys.exit("--focus-comp: no reference focus (lens file focus_dpt / meta focus_dpt_request)")
        fc = FocusComp(rec, rows, K, dist, Knew, (Wf, Hf), (W, H), d_ref or 0.0, float(getattr(a, "focus_k", 0.0080)),
                       float(getattr(a, "focus_tol_px", 0.05)), 40.0, bool(a.focus_comp),
                       bool(getattr(a, "drop_lens_moving", False)), table=getattr(a, "focus_table", None))
        print(f"focus: comp={fc.comp} D_ref={fc.d_ref:.4f} k={fc.k} drop_lens_moving={fc.drop}"
              + (f" table={fc.table} nodes={fc.tab_d.tolist()}" if fc.table else ""), file=sys.stderr)
    tmp = Path(tempfile.mkdtemp(prefix="vio_", dir=str(SCAN3D / "runs")))
    # OpenVINS: t_imu = t_cam + timeshift_cam_imu; ours: IMU time + dt = camera time. The frames below are already sent
    # on the IMU clock (tci = tc - dt), so the shift is applied there and timeshift must be 0 (before 2026-10-20 it was
    # also set to -dt here = applied twice, OpenVINS then started 2 ms off and had to estimate it away online).
    init_cfg = getattr(a, "init_cfg", None)
    if init_cfg is None:                                     # callers built before 2026-10-20 (collapse_run, ablation)
        init_cfg = "strict"
        print("init_cfg not given -> strict (bench default since 2026-10-20; old runs used 'loose')", file=sys.stderr)
    init_over = {}
    if getattr(a, "init_window", None):
        init_over["init_window_time"] = f"{float(a.init_window)}"
    if getattr(a, "init_disp", None):
        init_over["init_max_disparity"] = f"{float(a.init_disp):.2f}"
    for kv in (getattr(a, "cfg_over", "") or "").split(","):    # 2026-10-21 set-down mitigation (default: none)
        if kv.strip():
            k_, v_ = kv.split("=", 1)
            init_over[k_.strip()] = v_.strip()
    cfg_used = write_config(tmp, Knew, (W, H), a.fps, R_ci, p_CinI, 0.0, noise, a.npts, a.clones, init_cfg, init_over)
    hold_ok = StillHold(a.still_hold, t_imu, g_i, a_i) if getattr(a, "still_hold", None) else None
    held, hold_log, n_held = None, [], 0
    print(f"init: {init_cfg} {cfg_used}", file=sys.stderr)
    p = subprocess.Popen([str(BRIDGE), str(tmp / "estimator_config.yaml"), "--stdio", "--max-speed", "8",
                          "--lost-frames", "1000000", "--startup-lost-frames", "1000000"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    poses = []
    bst = {"last_t": -1, "init_t": None}                         # bridge progress (read only by the iD still-hold arming)

    def reader():
        for line in p.stdout:
            try:
                m = json.loads(line)
            except Exception:
                continue
            if m.get("type") == "pose":
                poses.append(m)
                if m.get("status") == "tracking" and bst["init_t"] is None:
                    bst["init_t"] = int(m["t_ns"])
                bst["last_t"] = max(bst["last_t"], int(m.get("t_ns") or -1))
        bst["last_t"] = 1 << 62                                  # bridge closed: stop any wait
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    nthr = int(getattr(a, "decode_threads", 0) or 0)            # 0 = OpenCV default (all cores); frames identical
    fcache, fwriter, cache_hit = None, None, False               # --frame-cache (2026-10-23, off by default)
    if getattr(a, "frame_cache", False):
        if fc is not None and fc.comp:
            print("frame cache: not used with --focus-comp (per-frame maps)", file=sys.stderr)
        else:
            from frame_cache import FrameCache
            fcache = FrameCache(rec, (W, H), m1, m2)
            need = [fi for fi in sel if not any(d0 <= t_frames[fi] < d1 for d0, d1 in drop)
                    and not (fc is not None and fc.drop and fc.moving(fi))]   # = the frames the loop retrieves
            cache_hit = fcache.has_all(need)
            if not cache_hit:
                fwriter = fcache.writer()
            print(f"frame cache: {'hit' if cache_hit else 'miss'} {fcache.info()}", file=sys.stderr)
    cap = None if cache_hit else (
        cv2.VideoCapture(str(rec / "cam5.mp4"), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, nthr]) if nthr
        else cv2.VideoCapture(str(rec / "cam5.mp4")))
    imu_lead = float(getattr(a, "imu_lead", 2.0))               # s of IMU before the first frame (default 2, unchanged)
    ii = int(np.searchsorted(t_imu, t_start - dt_ns - int(imu_lead * 1e9)))
    n_fed = 0
    last_fed_t, hold_wait_timeouts, hold_wait_s = None, 0, 0.0
    t_wall = time.time()
    crashed = None
    try:
        for fi in range(len(rows)):
            if cache_hit:                                        # same stop as the decoder's failed grab()
                if fcache.eof is not None and fi >= fcache.eof:
                    break
            elif not cap.grab():
                if fwriter is not None:
                    fwriter.eof(fi)
                break
            tc = t_frames[fi]
            if tc > t_end:                                       # (before 2026-10-20 checked after 'selected' -> decoded
                break                                            #  the rest of the video for nothing; output unchanged)
            if not selected[fi]:
                continue
            tci = tc - dt_ns                                     # camera time on the IMU clock
            while ii < len(t_imu) and t_imu[ii] <= tci + 5_000_000:
                p.stdin.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
                ii += 1
            if any(d0 <= tc < d1 for d0, d1 in drop):
                continue
            if fc is not None and fc.drop and fc.moving(fi):
                fc.n_dropped += 1
                continue
            if cache_hit:
                g = fcache.get(fi)
            else:
                _, f = cap.retrieve()
                mp = fc.map_for(fi) if fc is not None and fc.comp else None
                g = cv2.remap(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), *(mp or (m1, m2)), cv2.INTER_LINEAR)
                if fwriter is not None:                          # only without --focus-comp (mp is None)
                    fwriter.add(fi, g)
            if hold_ok is not None and hold_ok.needs_init() and last_fed_t is not None:
                tw = time.time()                                 # iD: wait for the bridge's pose of the previous frame
                while bst["last_t"] < last_fed_t and bst["init_t"] is None and time.time() - tw < 5.0:
                    time.sleep(0.0005)
                if bst["last_t"] < last_fed_t and bst["init_t"] is None:
                    hold_wait_timeouts += 1
                hold_wait_s += time.time() - tw
                if bst["init_t"] is not None:
                    hold_ok.init_ns = bst["init_t"]
            if hold_ok is not None:                              # only IMU samples up to this frame time (causal)
                if hold_ok.step(tci, (tc - t_start) / 1e9):
                    if held is None:
                        held = g
                        hold_log.append([int(tci), None])
                    g = held
                    n_held += 1
                elif held is not None:
                    held = None
                    hold_log[-1][1] = int(tci)
            body = struct.pack("<qIIB3x", int(tci), W, H, 1) + np.ascontiguousarray(g).tobytes()
            p.stdin.write(HDR.pack(MAGIC, 2, len(body)) + body)
            n_fed += 1
            last_fed_t = int(tci)
        for _ in range(200):                                     # flush: a bit more IMU so the last frame is processed
            if ii >= len(t_imu):
                break
            p.stdin.write(HDR.pack(MAGIC, 1, 56) + struct.pack("<q3d3d", int(t_imu[ii]), *g_i[ii], *a_i[ii]))
            ii += 1
        p.stdin.write(HDR.pack(MAGIC, 4, 0))
        p.stdin.close()
    except BrokenPipeError:                                      # ov_bridge died (e.g. diverged) -> score what we got
        crashed = f"ov_bridge exited (rc {p.poll()}) after {n_fed} frames"
        print(crashed, file=sys.stderr)
    finally:
        n_cached_new = fwriter.close() if fwriter is not None else 0
    p.wait(timeout=120)
    th.join(timeout=10)
    wall = time.time() - t_wall
    res = evaluate(rec, a, poses, n_fed, drop, dt_ns, wall, noise, Knew, frames)
    res.update(init_cfg=init_cfg, config_used=cfg_used, start_off=start_off, imu_lead_s=imu_lead, **code_version())
    if getattr(a, "cfg_over", None):
        res["cfg_over"] = a.cfg_over
    if fc is not None:
        res.update(fc.summary())
    if fcache is not None:                                       # key only with --frame-cache (default result unchanged)
        res["frame_cache"] = dict(hit=cache_hit, key=fcache.key, dir=str(fcache.dir),
                                  frames_written=n_cached_new)
    res["gyro_rebias"] = rebias_log is not None
    if rebias_log is not None:
        res.update(gyro_rebias_events=[[round((t - int(gyr[0, 0])) / 1e9, 2), ev, round(L, 1),
                                                          [round(float(np.degrees(x)) * 1e3, 1) for x in v]]
                                                         for t, ev, L, v in rebias_log])
    if hold_ok is not None:
        res.update(still_hold=a.still_hold, frames_held=n_held, hold_segments_imu_ns=hold_log)
        if hold_ok.arm_init is not None:
            res.update(hold_init_ns=hold_ok.init_ns, hold_wait_timeouts=hold_wait_timeouts,
                       hold_wait_s=round(hold_wait_s, 2))
    res["lens_file"] = str(lens_path(rec, meta).resolve().relative_to(SCAN3D))
    res["lens_blob"] = subprocess.run(["git", "-C", str(SCAN3D), "hash-object", res["lens_file"]],
                                      capture_output=True, text=True).stdout.strip()
    tr_t = [m["t_ns"] for m in poses if m.get("status") == "tracking"]
    t0c, t1c = int(t_frames[sel[0]] - dt_ns), int(t_frames[sel[-1]] - dt_ns)   # clip = fed frames, IMU clock
    res["clip_s"] = round((t1c - t0c) / 1e9, 3)
    res["t_init_clip_s"] = round((tr_t[0] - t0c) / 1e9, 3) if tr_t else None
    res["covered_frac"] = round(max(0.0, (t1c - tr_t[0]) / max(1, t1c - t0c)), 3) if tr_t else 0.0
    if crashed:
        res["bridge_crash"] = crashed
    return res


def code_version():
    """git HEAD + blobs of the bench and make_ov_config (blob = exact file content, also when uncommitted)."""
    def git(*c):
        return subprocess.run(["git", "-C", str(SCAN3D), *c], capture_output=True, text=True).stdout.strip()
    return dict(git_head=git("rev-parse", "--short", "HEAD"), bench_blob=git("hash-object", "imu_learn/vio_bench_hs.py"),
                imu_gt_blob=git("hash-object", "imu_learn/imu_gt.py"),   # read_imu lives here (2026-10-21 verdict 11)
                make_ov_config_blob=git("hash-object", "make_ov_config.py"),
                template_blobs={f: git("hash-object", f"config/mono/{f}") for f in
                                ("estimator_config.yaml", "kalibr_imu_chain.yaml")},
                bench_dirty=bool(git("status", "--porcelain", "imu_learn/vio_bench_hs.py", "make_ov_config.py",
                                     "imu_learn/imu_gt.py")))


def evaluate(rec, a, poses, n_fed, drop, dt_ns, wall, noise, Knew, frames):
    gt_dir = pick_gt(rec, a.gt)
    print(f"scoring GT: {gt_dir.name} (mode {a.gt})", file=sys.stderr)
    g = dict(np.load(gt_dir / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t = g["t_ns"][ok]
    gt_p, gt_R = g["p_bc"][ok], g["R_bc"][ok]
    tr = [m for m in poses if m.get("status") == "tracking" and m.get("T_world_cam0")]
    ms = np.array([m["ms"] for m in poses if "ms" in m])
    res = dict(rec=rec.name, fps=a.fps, size=a.size, npts=a.npts, clones=a.clones, noise=a.noise,
               imu_calib=bool(a.imu_calib), dropout=a.dropout, frames_fed=n_fed, poses=len(poses),
               tracked=len(tr), tracked_frac=round(len(tr) / max(1, n_fed), 3),
               ms_med=round(float(np.median(ms)), 2) if len(ms) else None,
               ms_p90=round(float(np.percentile(ms, 90)), 2) if len(ms) else None, wall_s=round(wall, 1),
               gt=gt_dir.name, gt_mode=a.gt, gt_uses_imu=gt_dir.name in ("imu_gt_rs", "imu_gt_spl_imu"),
               imu_source="raw_16_35" if a.imu_raw else "calibrated_1_4", noise_scale=a.noise_scale,
               timeshift_cam_imu=0.0, align_dt_ms=round(dt_ns / 1e6, 4), frames=frames,
               noise_used={k: float(v) for k, v in noise.items()})
    dts = np.array([m["dt_cam_imu"] for m in poses if m.get("dt_cam_imu") is not None], float) * 1e3
    if len(dts):   # OpenVINS online camera-IMU time offset (ms); ~0 means the offset we applied was right
        res.update(dt_cam_imu_init_ms=round(float(dts[0]), 4), dt_cam_imu_5s_ms=round(float(dts[min(len(dts) - 1, int(5 * a.fps))]), 4),
                   dt_cam_imu_final_ms=round(float(dts[-1]), 4), dt_cam_imu_med_ms=round(float(np.median(dts)), 4))
    if len(tr) < 50:
        res["error"] = "too few tracked poses"
        return res
    # VIO camera time (t_ns fed = camera time on the IMU clock) -> GT time (camera clock): + dt
    vt = np.array([m["t_ns"] for m in tr]) + dt_ns
    T = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4) for m in tr])
    # nearest GT within 2 ms (RS GT is stamped at t_mid exactly; base GT within ±1.5 ms of it)
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    if m.sum() < 50:
        res["error"] = "too few matched poses"
        return res
    P, Pg = T[m, :3, 3], gt_p[j[m]]
    Rv, Rg = T[m, :3, :3], gt_R[j[m]]
    tt = vt[m]
    indrop = np.array([any(d0 <= t < d1 + 1 for d0, d1 in drop) for t in tt]) if drop else np.zeros(len(tt), bool)
    R, _, t = umeyama(P[~indrop], Pg[~indrop])
    Rs, c, ts = umeyama(P[~indrop], Pg[~indrop], scale=True)
    e = np.linalg.norm((P @ R.T + t) - Pg, axis=1)
    res.update(matched=int(m.sum()), ate_cm=round(float(np.sqrt(np.mean(e[~indrop] ** 2))) * 100, 2),
               ate_sim3_cm=round(float(np.sqrt(np.mean(np.linalg.norm((c * P @ Rs.T + ts) - Pg, axis=1)[~indrop] ** 2))) * 100, 2),
               scale=round(float(c), 4))
    # rotation error after aligning orientations by themselves (a 40 cm position track fixes the frame only to a few °)
    U, _, Vt = np.linalg.svd(np.einsum("nij,nkj->ik", Rg[~indrop], Rv[~indrop]))
    Rrot = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    res["frame_rot_diff_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(Rrot.T @ R) - 1) / 2, -1, 1)))), 2)
    Ra = np.einsum("ij,njk->nik", Rrot, Rv)
    ang = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Ra, Rg) - 1) / 2, -1, 1)))
    res["rot_med_deg"] = round(float(np.median(ang[~indrop])), 3)
    # RPE over 1 s: relative displacement error
    rp = []
    for i0 in range(0, len(tt), 5):
        i1 = np.searchsorted(tt, tt[i0] + 1_000_000_000)
        if i1 >= len(tt) or tt[i1] - tt[i0] > 1.2e9 or indrop[i0] or indrop[i1]:
            continue
        dv = R @ (P[i1] - P[i0])
        dg = Pg[i1] - Pg[i0]
        rp.append(np.linalg.norm(dv - dg))
    res["rpe1s_cm"] = round(float(np.median(rp)) * 100, 2) if rp else None
    drifts = []
    for d0, d1 in drop:
        w = (tt >= d0) & (tt <= d1 + 300_000_000)
        if w.any():
            drifts.append(round(float(e[w].max()) * 100, 1))
    if drop:
        res["dropout_max_err_cm"] = drifts
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--fps", type=float, default=60)
    ap.add_argument("--size", default="640x360")
    ap.add_argument("--npts", type=int, default=200)
    ap.add_argument("--clones", type=int, default=11)
    ap.add_argument("--noise", default="cfg", choices=["cfg", "meas", "allan"])
    ap.add_argument("--noise-scale", type=float, default=1.0,
                    help="multiply measured noise (quiet-sample white noise underestimates real errors; Kalibr practice inflates)")
    ap.add_argument("--imu-calib", default=None)
    ap.add_argument("--init-cfg", default="strict", choices=["strict", "strict-dyn", "loose"],
                    help="OpenVINS init values: strict = make_ov_config (app), loose = old bench template (comparison)")
    ap.add_argument("--start-off", type=float, default=0.0, help="first VIO frame this many s later (other init)")
    ap.add_argument("--imu-lead", type=float, default=2.0,
                    help="IMU fed from this many s before the first frame (C5 retest 2026-10-21 uses 0)")
    ap.add_argument("--init-window", type=float, default=None, help="override init_window_time s (default: from --init-cfg)")
    ap.add_argument("--init-disp", type=float, default=None, help="override init_max_disparity px (default: from --init-cfg)")
    ap.add_argument("--decode-threads", type=int, default=0, help="video decode threads (0 = OpenCV default)")
    ap.add_argument("--dropout", default="")
    ap.add_argument("--max-secs", type=float, default=0)
    ap.add_argument("--imu-raw", action="store_true", help="uncalibrated IMU (types 16/35); new recordings only")
    ap.add_argument("--cfg-over", default="", help="k=v,... OpenVINS config overrides (keys must be in the template)")
    ap.add_argument("--still-hold", default=None, help="W:G:A IMU-only stillness -> hold the latched frame (off)")
    ap.add_argument("--gt", default="auto",
                    help="auto (imu_gt_spl if present else imu_gt, camera-only) | spl | base | rs | <dir name>")
    ap.add_argument("--focus-comp", action="store_true",
                    help="rescale each frame about the principal point by 1/(1+k(D-D_ref)) from results_cam5 focus (off)")
    ap.add_argument("--focus-k", type=float, default=0.0080, help="image scale per dioptre (docs/focus_scale_20261022.md)")
    ap.add_argument("--focus-tol-px", type=float, default=0.05,
                    help="--focus-comp: skip frames whose largest pixel shift is below this (keeps them bit-identical)")
    ap.add_argument("--drop-lens-moving", action="store_true",
                    help="drop frames whose nearest result has lens_state 1 or within 40 ms after one (off)")
    ap.add_argument("--focus-table", default=None,
                    help="--focus-comp: undistort each frame with K/distortion interpolated at its focus from this table "
                         "(config/lens/hs_1920x1080_focus_table_sweep.json) instead of the linear --focus-k scale (off)")
    ap.add_argument("--gyro-rebias", action="store_true",
                    help="subtract a causal per-still gyro bias (imu_learn/gyro_rebias.py); default ON with --imu-raw")
    ap.add_argument("--no-gyro-rebias", action="store_true", help="raw gyro as is (runs before 2026-10-22)")
    ap.add_argument("--frame-cache", action="store_true",
                    help="read the fed frames from recordings/<rec>/cache/ (imu_learn/frame_cache.py) when complete, else "
                         "decode as usual and add the missing frames; bit-identical images; not with --focus-comp (off)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.gyro_rebias and a.no_gyro_rebias:
        sys.exit("--gyro-rebias and --no-gyro-rebias")
    a.gyro_rebias = True if a.gyro_rebias else (False if a.no_gyro_rebias else None)   # None = default (ON with --imu-raw)
    if a.focus_table and not a.focus_comp:
        sys.exit("--focus-table needs --focus-comp")
    r = run(a)
    print(json.dumps(r, ensure_ascii=False))
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        json.dump(r, open(a.out, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
