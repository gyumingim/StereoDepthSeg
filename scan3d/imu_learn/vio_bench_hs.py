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


def write_config(d, K, size, fps, R_ci, p_CinI, timeshift, noise, npts, clones, init_cfg="strict"):
    W, H = size
    txt = TEMPLATE.read_text()
    rep = {"track_frequency:": f"track_frequency: {float(fps)}", "num_pts:": f"num_pts: {int(npts)}",
           "max_clones:": f"max_clones: {int(clones)}",
           "verbosity:": 'verbosity: "WARNING"', "record_timing_information:": "record_timing_information: false"}
    rep.update({k + ":": f"{k}: {v}" for k, v in init_values(init_cfg, K, W).items()})
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
        if ":" in body and (k.startswith("init_") or k in ("max_clones", "try_zupt", "zupt_max_disparity")):
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


def run(a):
    rec = Path(a.rec)
    meta = json.load(open(rec / "meta.json"))
    K, dist = lens_for(rec, meta)
    Wf, Hf = int(meta["width"]), int(meta["height"])
    W, H = (int(v) for v in a.size.split("x"))
    Knew, _ = cv2.getOptimalNewCameraMatrix(K, dist, (Wf, Hf), 0, (W, H))
    m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, Knew, (W, H), cv2.CV_16SC2)
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    t_frames = np.array([int(r["t_mid_ns"]) for r in rows])
    al = json.load(open(rec / "imu_gt" / "align.json"))
    dt_ns, R_ci = int(al["dt_ns"]), np.asarray(al["R_ci"])
    t_start = t_frames[0]
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        t_start = max(t_start, json.load(open(trim))["start_ns"])
    start_off = float(getattr(a, "start_off", 0.0) or 0.0)
    t_start = t_start + int(start_off * 1e9)                 # IMU feed still starts 2 s before the (later) first frame
    if a.max_secs:
        t_end = t_start + int(a.max_secs * 1e9)
    else:
        t_end = t_frames[-1]
    t_end = min(t_end, t_frames[-1])
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
    period = 1e9 / float(meta.get("save_fps") or 240)
    sel = pick_by_time(t_frames, t_start, t_end, a.fps, period)
    k = max(1, int(round(float(meta.get("save_fps") or 240) / a.fps)))
    old_idx = np.nonzero((t_frames >= t_start) & (t_frames <= t_end) & (np.arange(len(t_frames)) % k == 0))[0]
    frames = dict(rec_all=gap_stats(t_frames, period),
                  rec_window=gap_stats(t_frames[(t_frames >= t_start) & (t_frames <= t_end)], period),
                  fed=gap_stats(t_frames[sel], 1e9 / a.fps),
                  index_every_k_would_be=gap_stats(t_frames[old_idx], 1e9 / a.fps))
    print(f"frames: recording {frames['rec_all']}\n  fed by time {frames['fed']}", file=sys.stderr)
    selected = np.zeros(len(t_frames), bool)
    selected[sel] = True
    tmp = Path(tempfile.mkdtemp(prefix="vio_", dir=str(SCAN3D / "runs")))
    # OpenVINS: t_imu = t_cam + timeshift_cam_imu; ours: IMU time + dt = camera time. The frames below are already sent
    # on the IMU clock (tci = tc - dt), so the shift is applied there and timeshift must be 0 (before 2026-10-20 it was
    # also set to -dt here = applied twice, OpenVINS then started 2 ms off and had to estimate it away online).
    init_cfg = getattr(a, "init_cfg", None)
    if init_cfg is None:                                     # callers built before 2026-10-20 (collapse_run, ablation)
        init_cfg = "strict"
        print("init_cfg not given -> strict (bench default since 2026-10-20; old runs used 'loose')", file=sys.stderr)
    cfg_used = write_config(tmp, Knew, (W, H), a.fps, R_ci, p_CinI, 0.0, noise, a.npts, a.clones, init_cfg)
    print(f"init: {init_cfg} {cfg_used}", file=sys.stderr)
    p = subprocess.Popen([str(BRIDGE), str(tmp / "estimator_config.yaml"), "--stdio", "--max-speed", "8",
                          "--lost-frames", "1000000", "--startup-lost-frames", "1000000"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    poses = []

    def reader():
        for line in p.stdout:
            try:
                m = json.loads(line)
            except Exception:
                continue
            if m.get("type") == "pose":
                poses.append(m)
    th = threading.Thread(target=reader, daemon=True)
    th.start()
    nthr = int(getattr(a, "decode_threads", 0) or 0)            # 0 = OpenCV default (all cores); frames identical
    cap = (cv2.VideoCapture(str(rec / "cam5.mp4"), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, nthr]) if nthr
           else cv2.VideoCapture(str(rec / "cam5.mp4")))
    ii = int(np.searchsorted(t_imu, t_start - dt_ns - 2_000_000_000))
    n_fed = 0
    t_wall = time.time()
    crashed = None
    try:
        for fi in range(len(rows)):
            ok_r = cap.grab()
            if not ok_r:
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
            _, f = cap.retrieve()
            g = cv2.remap(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), m1, m2, cv2.INTER_LINEAR)
            body = struct.pack("<qIIB3x", int(tci), W, H, 1) + np.ascontiguousarray(g).tobytes()
            p.stdin.write(HDR.pack(MAGIC, 2, len(body)) + body)
            n_fed += 1
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
    p.wait(timeout=120)
    th.join(timeout=10)
    wall = time.time() - t_wall
    res = evaluate(rec, a, poses, n_fed, drop, dt_ns, wall, noise, Knew, frames)
    res.update(init_cfg=init_cfg, config_used=cfg_used, start_off=start_off, **code_version())
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
    ap.add_argument("--decode-threads", type=int, default=0, help="video decode threads (0 = OpenCV default)")
    ap.add_argument("--dropout", default="")
    ap.add_argument("--max-secs", type=float, default=0)
    ap.add_argument("--imu-raw", action="store_true", help="uncalibrated IMU (types 16/35); new recordings only")
    ap.add_argument("--gt", default="auto",
                    help="auto (imu_gt_spl if present else imu_gt, camera-only) | spl | base | rs | <dir name>")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    r = run(a)
    print(json.dumps(r, ensure_ascii=False))
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        json.dump(r, open(a.out, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
