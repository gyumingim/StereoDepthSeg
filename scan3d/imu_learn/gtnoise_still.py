#!/usr/bin/env python3
"""GT speed noise in IMU-certainly-still periods vs the 224323 end-still held interval (question: is the 3' share
check, <= 0.5 % of held GT samples > 2 cm/s, reachable at all given GT noise?). Analysis only, no VIO.

  ../venv_ffs/bin/python imu_learn/gtnoise_still.py   ->  runs/gtnoise/gtnoise_still.json

Speed estimator = z2_regress.gt_speed exactly (+-V_HALF central difference, GT_TOL), generalised to any GT folder
(asserted identical to zr.gt_speed for imu_gt_spl). Held-interval count = h_tune.held_motion exactly (asserted).
Certainly still (per GT sample, IMU clock t = t_gt - align_dt): in the +-WIN s IMU window around the sample the
per-axis std of gyro and accel are all below GYR_STD / ACC_STD and max |gyro - median| < GYR_DEV.
Time axis in the report = s since the first IMU sample (same origin as docs/setdown_failure.html).
"""
import gzip
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
import z2_regress as zr  # noqa: E402
import h_tune as ht  # noqa: E402
import c5_overnight_analyse as an  # noqa: E402
from imu_gt import read_imu  # noqa: E402

OUT = Path("runs/gtnoise")
RECS = list(zr.NEW)                                 # the six usable new recordings (raw IMU + spl/rs GT)
EXTRA = list(zr.OLD)                                # older two (no raw IMU -> calibrated IMU), reported separately
ALIGN = {"hs_20261020_214138": -2.1716, "hs_20261020_221052": -2.5828, "hs_20261020_221925": -2.1632,
         "hs_20261020_222747": -1.8186, "hs_20261020_223815": -2.7697, "hs_20261020_224323": -2.1382,
         "hs_20261019_131326": -2.0289, "hs_20261019_154320": -2.2895}   # align_dt_ms from the run jsons
WIN = 1.0                                           # s, half window of the IMU stillness test
LEVELS = {"tight": (0.003, 0.03, 0.01), "base": (0.005, 0.05, 0.02), "loose": (0.01, 0.1, 0.05)}
#          (max per-axis gyro std rad/s, max per-axis accel std m/s^2, max |gyro - median| rad/s)
HELD_RUN = "runs/h_arm_init/F_224323_Hw15m05_o0"
NAMES = ("imu_gt_spl", "imu_gt_rs")


def gt_speed_any(rec, name):
    """Copy of z2_regress.gt_speed with the GT folder as a parameter (same V_HALF, GT_TOL, an.gt filter)."""
    V_HALF, GT_TOL = zr.V_HALF, zr.GT_TOL
    t, p, _ = an.gt(rec, name)
    ts = t / 1e9
    v = np.full(len(t), np.nan)
    ja = np.clip(np.searchsorted(ts, ts + V_HALF), 1, len(ts) - 1)
    jb = np.clip(np.searchsorted(ts, ts - V_HALF), 1, len(ts) - 1)
    ja = np.where(np.abs(ts[ja - 1] - (ts + V_HALF)) < np.abs(ts[ja] - (ts + V_HALF)), ja - 1, ja)
    jb = np.where(np.abs(ts[jb - 1] - (ts - V_HALF)) < np.abs(ts[jb] - (ts - V_HALF)), jb - 1, jb)
    ok = (np.abs(ts[ja] - ts - V_HALF) < GT_TOL) & (np.abs(ts[jb] - ts + V_HALF) < GT_TOL)
    v[ok] = np.linalg.norm(p[ja[ok]] - p[jb[ok]], axis=1) / (ts[ja[ok]] - ts[jb[ok]])
    return t, v


def ncorners(rec, name, t):
    g = np.load(SCAN3D / "recordings" / rec / name / "gt.npz", allow_pickle=True)
    m = dict(zip(g["t_ns"].tolist(), g["n_corners"].tolist()))
    return np.array([m.get(int(x), -1) for x in t])


def imu_stats(ti, x, tq, half):
    """Per query time: max over axes of std, and max |x - median| (norm) inside [tq-half, tq+half] (seconds)."""
    a = np.searchsorted(ti, tq - half)
    b = np.searchsorted(ti, tq + half)
    c1 = np.vstack([np.zeros((1, 3)), np.cumsum(x, 0)])
    c2 = np.vstack([np.zeros((1, 3)), np.cumsum(x * x, 0)])
    n = np.maximum(b - a, 1)[:, None].astype(float)
    mu = (c1[b] - c1[a]) / n
    sd = np.sqrt(np.maximum((c2[b] - c2[a]) / n - mu * mu, 0)).max(1)
    dev = np.full(len(tq), np.inf)
    for k, (i, j) in enumerate(zip(a, b)):
        if j - i > 10:
            dev[k] = np.linalg.norm(x[i:j] - np.median(x[i:j], 0), axis=1).max()
    sd[(b - a) <= 10] = np.inf
    return sd, dev


def summ(v):
    v = v[np.isfinite(v)]
    if not len(v):
        return dict(n=0, fast=0)
    return dict(n=int(len(v)), fast=int((v > zr.V_MOVE).sum()), share=round(float((v > zr.V_MOVE).mean()), 5),
                vmax_cms=round(float(v.max()) * 100, 3), p50_cms=round(float(np.median(v)) * 100, 3),
                p95_cms=round(float(np.percentile(v, 95)) * 100, 3), p99_cms=round(float(np.percentile(v, 99)) * 100, 3))


def runs_of(mask):
    d = np.diff(np.r_[0, mask.astype(int), 0])
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


def still_mask(s, lv):
    gs, as_, gd = LEVELS[lv]
    return (s["gsd"] < gs) & (s["asd"] < as_) & (s["gdev"] < gd)


def analyse_rec(rec, raw):
    acc, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=raw)
    t0 = min(acc[0, 0], gyr[0, 0])
    tg_s, ta_s = (gyr[:, 0] - t0) / 1e9, (acc[:, 0] - t0) / 1e9
    dt = ALIGN[rec] * 1e6
    out = dict(rec=rec, imu="raw_16_35" if raw else "calibrated_1_4", t_imu_end_s=round(float(tg_s[-1]), 2))
    sp = {}
    for name in NAMES:
        t, v = gt_speed_any(rec, name)
        if name == "imu_gt_spl":
            t2, v2 = zr.gt_speed(rec)
            assert np.array_equal(t, t2) and np.array_equal(np.isnan(v), np.isnan(v2))
            assert np.allclose(v[~np.isnan(v)], v2[~np.isnan(v2)])
        tq = (t - dt - t0) / 1e9
        gsd, gdev = imu_stats(tg_s, gyr[:, 1:4], tq, WIN)
        asd, _ = imu_stats(ta_s, acc[:, 1:4], tq, WIN)
        sp[name] = dict(t=t, tq=tq, v=v, gsd=gsd, gdev=gdev, asd=asd, nc=ncorners(rec, name, t))
    out["gt_range_s"] = [round(float(sp["imu_gt_spl"]["tq"][0]), 2), round(float(sp["imu_gt_spl"]["tq"][-1]), 2)]
    for lv in LEVELS:
        out[lv] = {}
        for name, s in sp.items():
            still = still_mask(s, lv)
            fin = still & np.isfinite(s["v"])
            r = summ(s["v"][fin])
            r["still_spans_s"] = [[round(float(s["tq"][i]), 1), round(float(s["tq"][j]), 1)]
                                  for i, j in runs_of(still) if s["tq"][j] - s["tq"][i] > 1.0]
            fast = fin & (s["v"] > zr.V_MOVE)
            r["fast_t_s"] = [round(float(x), 2) for x in s["tq"][fast]][:80]
            r["fast_nc"] = s["nc"][fast].tolist()[:80]
            r["ncorners_med"] = float(np.median(s["nc"][fin])) if fin.any() else None
            if fin.any():          # noise vs number of detected corners (few corners = noisier GT)
                r["by_corners"] = {}
                for lo, hi in ((0, 20), (20, 40), (40, 80), (80, 10 ** 6)):
                    m = fin & (s["nc"] >= lo) & (s["nc"] < hi)
                    if m.any():
                        r["by_corners"][f"{lo}-{hi}"] = summ(s["v"][m])
            out[lv][name] = r
    return out, sp, (t0, tg_s, ta_s, gyr, acc, dt)


def held(sp, imu):
    """224323 end-still held interval of the Hw15m05/i* runs: GT counts (h_tune.held_motion reproduced) + IMU."""
    t0, tg_s, ta_s, gyr, acc, dt = imu
    d = json.load(open(HELD_RUN + ".json"))
    ref = ht.held_motion(HELD_RUN)
    t_last = max(m["t_ns"] for m in map(json.loads, gzip.open(HELD_RUN + ".poses.jsonl.gz", "rt")) if "t_ns" in m)
    (s, e), = d["hold_segments_imu_ns"]
    e = t_last if e is None else e
    res = dict(run=HELD_RUN, held_ref=ref, seg_imu_s=[round((s - t0) / 1e9, 2), round((e - t0) / 1e9, 2)])
    for name, x in sp.items():
        m = (x["t"] >= s + dt) & (x["t"] < e + dt) & np.isfinite(x["v"])
        r = summ(x["v"][m])
        if name == "imu_gt_spl":
            assert r["n"] == ref["held_gt_n"] and r["fast"] == ref["held_gt_fast"], (r, ref)
        fast = m & (x["v"] > zr.V_MOVE)
        st = still_mask(x, "base")
        r["gt_t_range_s"] = [round(float(x["tq"][m].min()), 2), round(float(x["tq"][m].max()), 2)] if m.any() else None
        r["ncorners_med"] = float(np.median(x["nc"][m])) if m.any() else None
        r["imu_still_base_share"] = round(float(st[m].mean()), 3) if m.any() else None
        r["held_and_still_base"] = summ(x["v"][m & st])
        r["held_not_still_base"] = summ(x["v"][m & ~st])
        r["imu_at_fast"] = [dict(t=round(float(x["tq"][i]), 2), v_cms=round(float(x["v"][i]) * 100, 2),
                                 gyr_sd=round(float(x["gsd"][i]), 4), acc_sd=round(float(x["asd"][i]), 4),
                                 gyr_dev=round(float(x["gdev"][i]), 4), nc=int(x["nc"][i]))
                            for i in np.where(fast)[0]]
        bins = {}
        for i in np.where(m)[0]:
            bins.setdefault(int(np.floor(x["tq"][i])), []).append(i)
        r["bins_1s"] = {k: dict(n=len(ii), fast=int((x["v"][ii] > zr.V_MOVE).sum()),
                                vmax_cms=round(float(x["v"][ii].max()) * 100, 2),
                                nc_med=float(np.median(x["nc"][ii]))) for k, ii in sorted(bins.items())}
        res[name] = r
    a, b = (s - t0) / 1e9, (e - t0) / 1e9
    rows = []
    for k in np.arange(np.floor(a), np.ceil(b)):
        g = gyr[(tg_s >= max(k, a)) & (tg_s < min(k + 1, b)), 1:4]
        q = acc[(ta_s >= max(k, a)) & (ta_s < min(k + 1, b)), 1:4]
        if len(g) < 10:
            continue
        rows.append(dict(t=int(k), gyr_sd=round(float(g.std(0).max()), 4),
                         gyr_dev=round(float(np.linalg.norm(g - np.median(g, 0), axis=1).max()), 4),
                         acc_sd=round(float(q.std(0).max()), 4),
                         acc_dev=round(float(np.linalg.norm(q - np.median(q, 0), axis=1).max()), 4)))
    res["imu_bins_1s"] = rows
    return res


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), levels=LEVELS, win_half_s=WIN,
               v_move=zr.V_MOVE, v_half=zr.V_HALF, gt_tol=zr.GT_TOL, recs={}, extra={})
    for rec in RECS + EXTRA:
        raw = rec in RECS
        r, sp, imu = analyse_rec(rec, raw)
        (res["recs"] if raw else res["extra"])[rec] = r
        if rec == "hs_20261020_224323":
            res["held_224323"] = held(sp, imu)
        print(rec, {lv: {n: (r[lv][n].get("fast"), r[lv][n].get("n"), r[lv][n].get("vmax_cms")) for n in r[lv]}
                    for lv in LEVELS}, flush=True)
    for grp in ("recs", "extra"):
        tot = {}
        for lv in LEVELS:
            tot[lv] = {}
            for name in NAMES:
                rr = [x[lv][name] for x in res[grp].values() if x[lv][name].get("n")]
                n = sum(x["n"] for x in rr)
                f = sum(x["fast"] for x in rr)
                tot[lv][name] = dict(n=n, fast=f, share=round(f / n, 5) if n else None,
                                     vmax_cms=max([x["vmax_cms"] for x in rr] or [None]))
        res[f"total_{grp}"] = tot
    json.dump(res, open(OUT / "gtnoise_still.json", "w"), indent=1, default=float)
    print(json.dumps({g: res[f"total_{g}"] for g in ("recs", "extra")}, indent=1))
    h = res["held_224323"]
    print("held", h["seg_imu_s"], {n: {k: h[n][k] for k in ("n", "fast", "share", "vmax_cms", "gt_t_range_s",
                                                           "ncorners_med", "imu_still_base_share")} for n in NAMES})


if __name__ == "__main__":
    main()
