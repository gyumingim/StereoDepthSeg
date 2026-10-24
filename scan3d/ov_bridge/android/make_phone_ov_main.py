#!/usr/bin/env python3
"""Config sets for live OpenVINS on the phone with the CURRENT BASELINE (arlat 0.6.7, mode ov, ov_cam main).

  venv_ffs/bin/python scan3d/ov_bridge/android/make_phone_ov_main.py [OUT_DIR] [--focus-dpt 0.5] [--lens-file F]

Baseline = docs/final_baseline_20261023.md, built with the same function as the bench (imu_learn/vio_bench_hs.py
write_config, template config/mono, as imu_learn/final_baseline_run.py): try_zupt false, strict init with window 1.0 s
and disparity 15 px, num_opencv_threads 2, cfg IMU noise (make_ov_config.IMU_NOISE), 640x360 undistorted pinhole,
num_pts 200, clones 11, lever arm config/imu_cam_lever.json, lens config/lens/hs_1920x1080_f2.22_cal20261022.json.

Differences from a bench run (the phone has no per-recording GT alignment):
  - R_ci = chordal mean of imu_gt/align.json R_ci over the 15 baseline recordings (spread printed, ~0.1-0.3 deg),
    p_CinI = -R_ci^T r_cm (lever), timeshift_cam_imu = -median(align dt_ns) (bench: frames shifted by each recording's
    dt, timeshift 0). calib_cam_extrinsics / timeoffset stay on (online), as in the bench.
  - Focus 0.5 dpt (app default since 0.6.6) while the lens file is the 2.22 dpt calibration: unless --lens-file gives a
    0.5 dpt calibration, fx, fy are scaled by s = 1 + k (D - D_ref) (k = 0.0080 / dpt, docs/focus_scale_20261022.md;
    principal point and distortion unchanged = assumption, same as the bench FocusComp). --no-focus-scale turns it off.
    (For reference the 0.37 dpt node of hs_1920x1080_focus_cal_20261022.json interpolated to 0.5 dpt gives fx ~0.5 %
    lower than this law; printed below. Timing does not depend on it, accuracy does.)

OUT_DIR (default scan3d/config/phone_ov_main; *.bin not committed, regenerate) gets
  remap_1920x1080.bin   raw 1920x1080 Y -> 640x360 undistorted (= bench maps: getOptimalNewCameraMatrix alpha 0 +
                        initUndistortRectifyMap CV_16SC2, stored as exact float32 -> the bridge restores them bit for bit)
  remap_640x360.bin     ISP-scaled 640x360 Y -> the same 640x360 undistorted image (K/3 with the half-pixel shift;
                        assumes the 640x360 YUV stream = the 1080p frame scaled by 1/3, NOT verified -> fallback only)
  <variant>/estimator_config.yaml + kalibr_imu_chain.yaml + kalibr_imucam_chain.yaml for
      base60 (60 fps, 200 pts, 11 clones = baseline), n150, n100, c8, n100c8, fps30, fps30n100c8
  variants.json         every value used, inputs, sha256 of the lens file, creation time.
Remap files use the v2 format (int32 w, h, float32 mapx, mapy, int32 src_w, src_h) -> ov_bridge built 2026-10-23+.
Push: adb push OUT_DIR/. /sdcard/Android/data/com.scan3d.arlat/files/ov/  (docs/phone_session_60fps_plan.md).
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SCAN3D = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SCAN3D / "imu_learn"))
sys.path.insert(0, str(SCAN3D))
import vio_bench_hs as vb  # noqa: E402
import make_ov_config  # noqa: E402
import lens_calib  # noqa: E402
from phone60_run import RAW, OLD  # noqa: E402  (the 15 baseline recordings)

PHONE_DIR = "/storage/emulated/0/Android/data/com.scan3d.arlat/files/ov"
LENS = SCAN3D / "config" / "lens" / "hs_1920x1080_f2.22_cal20261022.json"
LEVER = SCAN3D / "config" / "imu_cam_lever.json"
FOCUS_CAL = SCAN3D / "config" / "lens" / "hs_1920x1080_focus_cal_20261022.json"
RAW_SIZE = (1920, 1080)
VIO_SIZE = (640, 360)
# name: (fps, num_pts, max_clones)
VARIANTS = {"base60": (60, 200, 11), "n150": (60, 150, 11), "n100": (60, 100, 11), "c8": (60, 200, 8),
            "n100c8": (60, 100, 8), "fps30": (30, 200, 11), "fps30n100c8": (30, 100, 8)}
# final_baseline_run.py: init_window=1.0, init_disp=15.0, cfg_over="num_opencv_threads=2,try_zupt=false", init_cfg strict
INIT_OVER = {"init_window_time": "1.0", "init_max_disparity": "15.00", "num_opencv_threads": "2", "try_zupt": "false"}
# feature-init limits = F1, adopted 2026-10-24 (docs/fi_limits_result.md; OpenVINS defaults 40 / 10000)
FI_OVER = {"fi_max_baseline": "200.0", "fi_max_cond_number": "1000000.0"}


def mean_rotation(Rs):
    U, _, Vt = np.linalg.svd(sum(Rs))
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    ang = [float(np.degrees(np.arccos(np.clip((np.trace(R.T @ x) - 1) / 2, -1, 1)))) for x in Rs]
    return R, ang


def write_remap(path, K_src, dist, src_size, Knew):
    # The bench's fixed-point maps (CV_16SC2 + 1/32 px table index) written back as exact float32 multiples of 1/32, so
    # the bridge's cv::convertMaps(float -> CV_16SC2) restores them bit for bit. Float maps made directly differ in
    # 0.2 % of the entries -> 0.02 % of the pixels by 1-2 levels, which already changes an OpenVINS run (checked
    # 2026-10-23 on 133820 o0: window ATE 1.43 -> 2.17 cm).
    m1, m2 = cv2.initUndistortRectifyMap(K_src, dist, None, Knew, VIO_SIZE, cv2.CV_16SC2)
    mx = m1[..., 0].astype(np.float32) + (m2 & 31).astype(np.float32) / 32
    my = m1[..., 1].astype(np.float32) + (m2 >> 5).astype(np.float32) / 32
    c1, c2 = cv2.convertMaps(mx, my, cv2.CV_16SC2)
    if not (np.array_equal(c1, m1) and np.array_equal(c2, m2)):
        sys.exit(f"{path}: float maps do not round-trip to the bench's fixed-point maps")
    with open(path, "wb") as f:
        f.write(np.array(VIO_SIZE, np.int32).tobytes())
        f.write(np.ascontiguousarray(mx, np.float32).tobytes())
        f.write(np.ascontiguousarray(my, np.float32).tobytes())
        f.write(np.array(src_size, np.int32).tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", nargs="?", default=str(SCAN3D / "config" / "phone_ov_main"))
    ap.add_argument("--lens-file", default=str(LENS))
    ap.add_argument("--focus-dpt", type=float, default=0.5, help="focus the phone runs at (app default 0.5)")
    ap.add_argument("--focus-k", type=float, default=0.0080, help="image scale per dioptre (docs/focus_scale_20261022.md)")
    ap.add_argument("--no-focus-scale", action="store_true", help="lens file already calibrated at --focus-dpt")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    K, dist, lj = lens_calib.load(a.lens_file)
    d_ref = float(lj.get("focus_dpt") or 0)
    s = 1.0
    if not a.no_focus_scale and d_ref > 0 and abs(d_ref - a.focus_dpt) > 1e-3:
        s = 1.0 + a.focus_k * (a.focus_dpt - d_ref)
    K = K.copy()
    K[0, 0] *= s
    K[1, 1] *= s
    Knew, _ = cv2.getOptimalNewCameraMatrix(K, dist, RAW_SIZE, 0, VIO_SIZE)
    write_remap(out / "remap_1920x1080.bin", K, dist, RAW_SIZE, Knew)
    K3 = K.copy()
    K3[0, 0] /= 3
    K3[1, 1] /= 3
    K3[0, 2] = (K[0, 2] + 0.5) / 3 - 0.5
    K3[1, 2] = (K[1, 2] + 0.5) / 3 - 0.5
    write_remap(out / "remap_640x360.bin", K3, dist, VIO_SIZE, Knew)

    # reference: the 0.37 dpt node of the joint focus fit, linearly interpolated (fx) to the run focus
    fx_interp = None
    try:
        fc = json.load(open(FOCUS_CAL))["focus"]
        nodes = sorted((float(n["focus_dpt"]), float(n["K"][0][0])) for n in fc)
        fx_interp = float(np.interp(a.focus_dpt, [n[0] for n in nodes], [n[1] for n in nodes]))
    except Exception as e:  # noqa: BLE001
        print(f"focus table reference not available: {e}", file=sys.stderr)

    recs = RAW + OLD
    Rs, dts = [], []
    for r in recs:
        al = json.load(open(SCAN3D / "recordings" / r / "imu_gt" / "align.json"))
        Rs.append(np.asarray(al["R_ci"], float))
        dts.append(int(al["dt_ns"]))
    R_ci, spread = mean_rotation(Rs)
    r_cm = np.asarray(json.load(open(LEVER))["r_cm"], float)
    p_CinI = -R_ci.T @ (r_cm / 100.0)
    timeshift = -float(np.median(dts)) * 1e-9            # OpenVINS t_imu = t_cam + timeshift; bench: t_imu = t_cam - dt

    vb.TEMPLATE = SCAN3D / "config" / "mono" / "estimator_config.yaml"
    noise = dict(make_ov_config.IMU_NOISE)
    used = {}
    for name, (fps, npts, clones) in VARIANTS.items():
        d = out / name
        d.mkdir(exist_ok=True)
        used[name] = vb.write_config(d, Knew, VIO_SIZE, fps, R_ci, p_CinI, timeshift, noise, npts, clones, "strict", {**INIT_OVER, **FI_OVER})
        cfg = d / "estimator_config.yaml"
        lines = []
        for line in cfg.read_text().splitlines():
            t = line.strip()
            if t.startswith("record_timing_information:"):
                line = "record_timing_information: true"
            for k, fn in (("record_timing_filepath:", "ov_timing.txt"), ("filepath_est:", "ov_estimate.txt"),
                          ("filepath_std:", "ov_estimate_std.txt"), ("filepath_gt:", "ov_groundtruth.txt")):
                if t.startswith(k):        # the app rewrites these to its run dir; never a PC path on the phone
                    line = f'{k} "{PHONE_DIR}/{name}/{fn}"'
            lines.append(line)
        cfg.write_text("\n".join(lines) + "\n")
        chk = {k: v for k, v in (ln.split("#")[0].split(":", 1) for ln in cfg.read_text().splitlines()
                                 if ":" in ln.split("#")[0] and not ln.startswith(" "))}
        used[name].update({k: chk[k].strip() for k in ("track_frequency", "num_pts", "max_clones", "num_opencv_threads",
                                                         "try_zupt", "record_timing_information", "use_stereo",
                                                         "calib_cam_intrinsics", "calib_cam_extrinsics", *FI_OVER)})

    info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), script="scan3d/ov_bridge/android/make_phone_ov_main.py",
                lens_file=str(Path(a.lens_file).resolve()), lens_sha256=hashlib.sha256(Path(a.lens_file).read_bytes()).hexdigest(),
                lens_focus_dpt=d_ref, run_focus_dpt=a.focus_dpt, focus_scale=s, focus_k=a.focus_k,
                K_raw_used=K.tolist(), fx_focus_table_interp=fx_interp, dist=dist.tolist(), Knew_640x360=Knew.tolist(),
                K_640x360_isp_assumed=K3.tolist(), R_ci=R_ci.tolist(), R_ci_spread_deg=spread, recordings=recs,
                lever_r_cm=r_cm.tolist(), p_CinI_m=p_CinI.tolist(), dt_ns=dts, timeshift_cam_imu_s=timeshift,
                imu_noise=noise, template=str(vb.TEMPLATE), init_over=INIT_OVER, fi_over=FI_OVER, variants=VARIANTS, cfg_used=used,
                remaps={"remap_1920x1080.bin": "src 1920x1080 -> 640x360", "remap_640x360.bin": "src 640x360 (ISP) -> 640x360"})
    (out / "variants.json").write_text(json.dumps(info, indent=1))
    print(f"lens {a.lens_file} D_ref {d_ref} -> run {a.focus_dpt} dpt, scale {s:.5f}: fx {K[0, 0]:.2f} "
          f"(focus-table interp {fx_interp if fx_interp is None else round(fx_interp, 2)})")
    print(f"Knew 640x360: fx {Knew[0, 0]:.3f} fy {Knew[1, 1]:.3f} cx {Knew[0, 2]:.3f} cy {Knew[1, 2]:.3f}")
    print(f"R_ci mean of {len(recs)} (spread deg max {max(spread):.3f}); p_CinI cm {np.round(p_CinI * 100, 3)}; "
          f"timeshift {timeshift * 1e3:.3f} ms")
    print(f"-> {out}: {', '.join(VARIANTS)}")


if __name__ == "__main__":
    main()
