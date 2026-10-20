#!/usr/bin/env python3
"""Overnight VIO 2026-10-20 (docs/overnight_vio_prereg.md): per recording, the initial still period inside the VIO clip
and the calibration JSONs the C experiment feeds to the bench (vio_bench_hs --imu-calib, applied as g*G^T + bg,
a*A^T + ba, lever r).

Still (fixed before any VIO result): 0.5 s windows (step 0.1 s) on the IMU clock; still = max-axis gyro std < 1.0 deg/s
AND |a| std < 0.1 m/s^2 (measured still ~0.1 deg/s / 0.006 m/s^2, motion > 2 deg/s). The still period = from the clip
start (first VIO frame at start offset 0 = trim start, on the IMU clock) to the start of the first non-still window.

Outputs in OUT (default runs/overnight_vio/inputs):
  <rec>_still.json        clip start, motion start (s after clip start), still length, gyro mean over the still
                          (Android-calibrated 1/4 and raw 16/35), battery temperature at the still
  <rec>_bias_cal.json     bias-only calibration for the calibrated IMU: G = A = I, bg = -mean gyro(still), ba = r = 0
  <rec>_bias_raw.json     same for the raw IMU
  old_phys_mean.json      physical calibration learned on the OLD monitor recordings (runs/on_mid_s{0,1,2}_<rec>/result.json,
                          midpoint-fixed, 3 seeds x 4 leave-one-out folds): element-wise mean of G, bg, A, ba, r
  --bias-from REC         take the still bias of another recording (same session, e.g. 221925 <- 221903)

  ../venv_ffs/bin/python imu_learn/c5_still.py recordings/hs_20261020_214138 [--bias-from recordings/hs_...]
"""
import argparse
import csv
import glob
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402

SCAN3D = HERE.parent
WIN_S, STEP_S, G_STD_DPS, A_STD = 0.5, 0.1, 1.0, 0.1


def clip_start_imu(rec):
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    t0 = int(rows[0]["t_mid_ns"])
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        t0 = max(t0, json.load(open(trim))["start_ns"])
    al = rec / "imu_gt" / "align.json"
    dt = int(json.load(open(al))["dt_ns"]) if al.exists() else 0
    return t0 - dt, dt, (int(rows[-1]["t_mid_ns"]) - dt)


def still_info(rec):
    t0, dt, t_last = clip_start_imu(rec)
    acc, gyr = read_imu(rec / "imu.csv", raw=True)
    accc, gyrc = read_imu(rec / "imu.csv", raw=False)
    t = gyr[:, 0]
    g = np.degrees(gyr[:, 1:4])
    an = np.linalg.norm(np.stack([np.interp(t, acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1), axis=1)
    motion = None
    s = t0
    while s + WIN_S * 1e9 < t[-1]:
        m = (t >= s) & (t < s + WIN_S * 1e9)
        if m.sum() > 20 and (g[m].std(0).max() >= G_STD_DPS or an[m].std() >= A_STD):
            motion = s
            break
        s += STEP_S * 1e9
    still_end = motion if motion is not None else t[-1]
    sel = (t >= t0) & (t < still_end - 0.5e9)
    selc = (gyrc[:, 0] >= t0) & (gyrc[:, 0] < still_end - 0.5e9)
    still_s = (still_end - t0) / 1e9
    temp = None
    tf = rec / "temps.csv"
    if tf.exists():
        try:
            vv = [float(r["battery_dC"]) / 10 for r in csv.DictReader(open(tf)) if r.get("battery_dC")]
            temp = dict(first_c=vv[0], last_c=vv[-1]) if vv else None
        except (KeyError, ValueError):
            temp = None
    return dict(rec=rec.name, clip_start_imu_ns=int(t0), align_dt_ns=dt, video_end_after_clip_s=round((t_last - t0) / 1e9, 2),
                motion_start_s=None if motion is None else round((motion - t0) / 1e9, 2), still_s=round(still_s, 2),
                has_still=bool(still_s >= 4.0), n_still_samples=int(sel.sum()),
                gyro_mean_raw_rad_s=gyr[sel, 1:4].mean(0).tolist() if sel.sum() > 100 else None,
                gyro_mean_cal_rad_s=gyrc[selc, 1:4].mean(0).tolist() if selc.sum() > 100 else None,
                thresholds=dict(win_s=WIN_S, step_s=STEP_S, gyro_std_dps=G_STD_DPS, acc_std=A_STD), battery=temp)


def bias_calib(mean_g, src):
    return dict(calib=dict(G=np.eye(3).tolist(), A=np.eye(3).tolist(), bg=(-np.asarray(mean_g)).tolist(), ba=[0.0] * 3,
                           r=[0.0] * 3), source=src)


def old_phys(out):
    files = sorted(glob.glob(str(SCAN3D / "runs" / "on_mid_s[012]_hs_20261019_*" / "result.json")))
    cals = [json.load(open(f))["calib"] for f in files]
    mean = {k: np.mean([np.asarray(c[k], float) for c in cals], 0).tolist() for k in ("G", "bg", "A", "ba", "r")}
    spread = {k: float(np.max(np.std([np.asarray(c[k], float) for c in cals], 0))) for k in ("G", "bg", "A", "ba", "r")}
    d = dict(calib=mean, source="element-wise mean of the midpoint-fixed physical calibrations trained on the OLD "
             "monitor recordings (Android-calibrated 1/4 IMU)", files=[str(Path(f).relative_to(SCAN3D)) for f in files],
             file_sha256={str(Path(f).relative_to(SCAN3D)): hashlib.sha256(open(f, "rb").read()).hexdigest() for f in files},
             max_std_across_files=spread)
    json.dump(d, open(out / "old_phys_mean.json", "w"), indent=1)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="*")
    ap.add_argument("--bias-from", default=None)
    ap.add_argument("--out", default=str(SCAN3D / "runs" / "overnight_vio" / "inputs"))
    ap.add_argument("--old-phys", action="store_true", help="(re)write old_phys_mean.json")
    o = ap.parse_args()
    out = Path(o.out)
    out.mkdir(parents=True, exist_ok=True)
    if o.old_phys or not (out / "old_phys_mean.json").exists():
        d = old_phys(out)
        print(f"old_phys_mean.json from {len(d['files'])} files, max std {d['max_std_across_files']}")
    for r in o.recs:
        rec = Path(r.rstrip("/"))
        info = still_info(rec)
        src_rec = rec
        if o.bias_from:
            src = still_info(Path(o.bias_from.rstrip("/")))
            info["bias_from"] = src["rec"]
            info["bias_source_still_s"] = src["still_s"]
            info["gyro_mean_raw_rad_s"], info["gyro_mean_cal_rad_s"] = src["gyro_mean_raw_rad_s"], src["gyro_mean_cal_rad_s"]
            src_rec = Path(o.bias_from)
        json.dump(info, open(out / f"{rec.name}_still.json", "w"), indent=1)
        for kind in ("cal", "raw"):
            mg = info[f"gyro_mean_{kind}_rad_s"]
            if mg is not None:
                json.dump(bias_calib(mg, f"still gyro mean of {src_rec.name} ({kind} IMU)"),
                          open(out / f"{rec.name}_bias_{kind}.json", "w"), indent=1)
        print(json.dumps({k: info[k] for k in ("rec", "still_s", "motion_start_s", "has_still", "video_end_after_clip_s")}),
              "bg_cal_dps", None if info["gyro_mean_cal_rad_s"] is None else np.round(np.degrees(info["gyro_mean_cal_rad_s"]), 3).tolist(),
              "bg_raw_dps", None if info["gyro_mean_raw_rad_s"] is None else np.round(np.degrees(info["gyro_mean_raw_rad_s"]), 3).tolist())


if __name__ == "__main__":
    main()
