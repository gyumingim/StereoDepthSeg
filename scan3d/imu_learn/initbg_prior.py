#!/usr/bin/env python3
"""Gyro bias prior per recording, leave-one-recording-out (docs/initbg_plan.md, 2026-10-23). Read-only analysis.

Still windows = same labelling as imu_learn/initgyro_thresh.py (0.5 s windows, GT camera rate over a 0.1 s baseline,
GT coverage >= 80 %, GT rate p90 < 0.02 rad/s), step 0.25 s. Gyro = the stream the bench feeds (raw 16/35 for new
recordings, Android-calibrated 1/4 for 131326/154320; rebias off), so each window mean = that recording's gyro bias.
Per recording: per-axis median of its still-window means. Prior for recording R = per-axis median over the per-recording
medians of the OTHER recordings with the SAME IMU source (raw vs calibrated differ: the OS removes its bias estimate).
213955 (drop) is a valid source (only its scoring is excluded). Recordings with no still window get a prior but give none.

  ../venv_ffs/bin/python imu_learn/initbg_prior.py   # -> runs/initbg/prior.json
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402
from phone60_run import RAW, OLD  # noqa: E402

OUT = SCAN3D / "runs" / "initbg"
STEP, WIN = 0.25, 0.5


def still_means(rec):
    R = SCAN3D / "recordings" / rec
    acc, gyr = read_imu(R / "imu.csv", raw=rec not in OLD)
    ti = gyr[:, 0].astype(np.int64)
    t = (ti - ti[0]) / 1e9
    w = gyr[:, 1:4]
    dt_ns = int(json.load(open(R / "imu_gt" / "align.json"))["dt_ns"])
    g = np.load(R / "imu_gt_spl" / "gt.npz")
    ok = g["ok"].astype(bool)
    tg = (g["t_ns"][ok] - dt_ns - ti[0]) / 1e9
    Rg = g["R_bc"][ok]
    o = np.argsort(tg)
    tg, Rg = tg[o], Rg[o]
    fps_gt = 1.0 / np.median(np.diff(tg))
    lag = max(1, int(round(0.1 * fps_gt)))
    d = tg[lag:] - tg[:-lag]
    ang = 2 * np.arcsin(np.clip(np.linalg.norm(Rg[:-lag] - Rg[lag:], axis=(1, 2)) / (2 * np.sqrt(2)), 0, 1))
    rate = ang / np.maximum(d, 1e-6)
    tm = 0.5 * (tg[:-lag] + tg[lag:])
    good = (d > 0) & (d < 0.15)
    tm, rate = tm[good], rate[good]
    out = []
    for te in np.arange(1.0, t[-1], STEP):
        mg = (tm > te - WIN) & (tm <= te)
        if mg.sum() < 0.8 * WIN * fps_gt or np.percentile(rate[mg], 90) >= 0.02:
            continue
        m = (t > te - WIN) & (t <= te)
        if m.sum() >= 10:
            out.append(w[m].mean(0))
    return np.array(out).reshape(-1, 3)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    per = {}
    for rec in RAW + OLD:
        S = still_means(rec)
        per[rec] = dict(src="calibrated" if rec in OLD else "raw", n_still=int(len(S)),
                        median=[round(float(x), 6) for x in np.median(S, 0)] if len(S) else None,
                        iqr=[round(float(x), 6) for x in (np.percentile(S, 75, 0) - np.percentile(S, 25, 0))] if len(S) else None)
        print(rec, per[rec], flush=True)
    prior = {}
    for rec in RAW + OLD:
        others = [per[r]["median"] for r in per if r != rec and per[r]["src"] == per[rec]["src"] and per[r]["median"]]
        p = np.median(np.array(others), 0) if others else np.zeros(3)
        own = per[rec]["median"]
        prior[rec[-6:]] = dict(prior=[round(float(x), 6) for x in p], n_sources=len(others), own_median=own,
                               own_minus_prior_norm=round(float(np.linalg.norm(np.array(own) - p)), 6) if own else None)
    res = dict(made=time.strftime("%F %T"), method=__doc__.split("\n\n")[0], per_rec=per, prior=prior)
    json.dump(res, open(OUT / "prior.json", "w"), indent=1)
    for k, v in prior.items():
        print(k, v)


if __name__ == "__main__":
    main()
