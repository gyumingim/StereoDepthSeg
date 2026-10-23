#!/usr/bin/env python3
"""Threshold for the static-init gyro gate (docs/initgyro_plan.md, 2026-10-23). Read-only analysis, run BEFORE the comparison.

Gate statistic (same as tools/ov_initgyro_patch.py): over a 0.5 s bias window (= older half of init_window_time 1.0),
g = max over five 0.1 s sub-windows of |mean(gyro vector)|, on the gyro stream the bench feeds (raw types 16/35 for new
recordings, Android-calibrated 1/4 for 131326/154320; gyro_rebias off, imu_calib none) -> bias included.

Windows: every 0.05 s over each recording. Label from GT (imu_gt_spl/gt.npz board pose, spline) camera rotation rate in
the same window (IMU clock = GT camera time - align dt; rate = rotation angle over a 0.1 s baseline, chordal formula;
still GT rate is ~0.001 rad/s, bimodal against motion):
  still  : >= 80 % GT coverage, GT rate p90 < 0.02 rad/s
  moving : >= 80 % GT coverage, GT rate median > 0.10 rad/s
'eligible' = stock static-init accel condition on that window (vector sample std < init_imu_thresh 0.3) = the windows the
gate actually acts on. Also: g at each V4 o0 init (bias window ending 0.5 s before... see init_window()).

  ../venv_ffs/bin/python imu_learn/initgyro_thresh.py   # -> runs/initgyro/thresh.json
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402
from phone60_run import RAW, OLD, DROP  # noqa: E402

OUT = SCAN3D / "runs" / "initgyro"
STEP, WIN, SUB = 0.05, 0.5, 0.1


def so3_log_norm(R):
    c = np.clip((np.trace(R, axis1=-2, axis2=-1) - 1) / 2, -1, 1)
    return np.arccos(c)


def gate_stat(t, w, t_end):
    """max over 0.1 s sub-windows of |mean gyro| in (t_end - 0.5, t_end]; also mean |gyro| and accel std done elsewhere."""
    best = 0.0
    for k in range(int(round(WIN / SUB))):
        lo, hi = t_end - WIN + k * SUB, t_end - WIN + (k + 1) * SUB
        m = (t > lo) & (t <= hi)
        if m.sum():
            best = max(best, float(np.linalg.norm(w[m].mean(0))))
    return best


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows, inits = [], {}
    for rec in RAW + OLD:
        R = SCAN3D / "recordings" / rec
        acc, gyr = read_imu(R / "imu.csv", raw=rec not in OLD)
        ti = gyr[:, 0].astype(np.int64)
        t = (ti - ti[0]) / 1e9
        w = gyr[:, 1:4]
        a = np.stack([np.interp(gyr[:, 0], acc[:, 0], acc[:, 1 + j]) for j in range(3)], 1)
        dt_ns = int(json.load(open(R / "imu_gt" / "align.json"))["dt_ns"])
        g = np.load(R / "imu_gt_spl" / "gt.npz")
        ok = g["ok"].astype(bool)
        tg = (g["t_ns"][ok] - dt_ns - ti[0]) / 1e9
        Rg = g["R_bc"][ok]
        o = np.argsort(tg)
        tg, Rg = tg[o], Rg[o]
        # rate over a 0.1 s baseline; angle from the chordal distance (arccos of the trace loses ~1e-4 rad to the
        # non-orthonormality of the stored R, = 0.02 rad/s at a 4 ms baseline)
        fps_gt = 1.0 / np.median(np.diff(tg))
        lag = max(1, int(round(0.1 * fps_gt)))
        d = tg[lag:] - tg[:-lag]
        ang = 2 * np.arcsin(np.clip(np.linalg.norm(Rg[:-lag] - Rg[lag:], axis=(1, 2)) / (2 * np.sqrt(2)), 0, 1))
        rate = ang / np.maximum(d, 1e-6)
        tm = 0.5 * (tg[:-lag] + tg[lag:])
        good = (d > 0) & (d < 0.15)
        tm, rate = tm[good], rate[good]
        # V4 o0 init (first tracking pose, IMU clock)
        P = [json.loads(x) for x in gzip.open(SCAN3D / "runs" / "phone60" / f"{rec[-6:]}_o0_V4.poses.jsonl.gz", "rt")]
        tr = [p["t_ns"] for p in P if p.get("status") == "tracking"]
        ends = np.arange(1.0, t[-1], STEP)
        for te in ends:
            m = (t > te - WIN) & (t <= te)
            if m.sum() < 10:
                continue
            aw = a[m]
            astd = float(np.sqrt(((aw - aw.mean(0)) ** 2).sum(1).sum() / (len(aw) - 1)))
            gs = gate_stat(t[m], w[m], te)
            mabs = float(np.linalg.norm(w[m], axis=1).mean())
            mg = (tm > te - WIN) & (tm <= te)
            cov = mg.sum() / (WIN * fps_gt)
            lab = None
            if cov >= 0.8:
                r = rate[mg]
                if np.percentile(r, 90) < 0.02:
                    lab = "still"
                elif np.median(r) > 0.10:
                    lab = "moving"
            rows.append(dict(rec=rec[-6:], te=round(float(te), 2), g=gs, mabs=mabs, astd=astd, lab=lab,
                             gt_med=float(np.median(rate[mg])) if mg.sum() else None))
        if tr:
            # bias window = older half of the 1.0 s init window; the newest IMU at init is ~ the init frame (+ queue);
            # report g for bias-window ends in [t_init - 0.75, t_init - 0.35] (alignment unknown by ~0.1 s)
            t0 = (tr[0] - ti[0]) / 1e9
            cand = [gate_stat(t[(t > e - WIN) & (t <= e)], w[(t > e - WIN) & (t <= e)], e)
                    for e in np.arange(t0 - 0.75, t0 - 0.35 + 1e-9, 0.05)]
            inits[rec[-6:]] = dict(t_init_s=round(t0, 2), g_min=round(min(cand), 4), g_max=round(max(cand), 4))
        print(rec, len(rows), inits.get(rec[-6:]), flush=True)

    def q(v):
        v = np.asarray(v)
        return dict(n=int(len(v)), **{f"p{p}": round(float(np.percentile(v, p)), 4) for p in (1, 5, 50, 95, 99)},
                    max=round(float(v.max()), 4) if len(v) else None) if len(v) else dict(n=0)
    res = dict(made=time.strftime("%F %T"), stat="max over 0.1 s sub-windows of |mean gyro| in a 0.5 s window",
               labels=dict(still="GT rate p90 < 0.02 rad/s", moving="GT rate median > 0.10 rad/s", coverage=0.8),
               excl_drop=DROP[-6:], dist={}, per_rec={}, v4_o0_init=inits)
    use = [r for r in rows if r["rec"] != DROP[-6:]]
    for lab in ("still", "moving"):
        for elig in (False, True):
            v = [r["g"] for r in use if r["lab"] == lab and (r["astd"] < 0.3 or not elig)]
            res["dist"][f"{lab}{'_eligible' if elig else ''}"] = q(v)
    for th in (0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.15):
        st = [r for r in use if r["lab"] == "still"]
        mv = [r for r in use if r["lab"] == "moving" and r["astd"] < 0.3]
        res.setdefault("by_threshold", {})[str(th)] = dict(
            still_blocked_frac=round(sum(r["g"] > th for r in st) / max(1, len(st)), 4),
            moving_eligible_passed_frac=round(sum(r["g"] <= th for r in mv) / max(1, len(mv)), 4))
    for rec in sorted({r["rec"] for r in rows}):
        st = [r["g"] for r in rows if r["rec"] == rec and r["lab"] == "still"]
        res["per_rec"][rec] = dict(n_still=len(st), still_g_p50=round(float(np.median(st)), 4) if st else None,
                                   still_g_p99=round(float(np.percentile(st, 99)), 4) if st else None,
                                   still_g_min=round(float(min(st)), 4) if st else None)
    json.dump(res, open(OUT / "thresh.json", "w"), indent=1)
    np.save(OUT / "thresh_rows.npy", np.array([(r["rec"], r["te"], r["g"], r["mabs"], r["astd"], r["lab"] or "")
                                                for r in rows], dtype=object), allow_pickle=True)
    print(json.dumps({k: res[k] for k in ("dist", "by_threshold", "per_rec", "v4_o0_init")}, indent=1))


if __name__ == "__main__":
    main()
