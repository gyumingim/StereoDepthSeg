#!/usr/bin/env python3
"""224323 end-still held interval: is the > 2 cm/s GT speed real motion or GT noise? Analysis only, no VIO.

  ../venv_ffs/bin/python imu_learn/gtnoise_held.py   ->  runs/gtnoise/gtnoise_held.json

Looks at (1) the GT position trace (imu_gt_spl, imu_gt_rs; same filter as c5_overnight_analyse.gt) in the held
interval: net displacement between the still part before the hand (118.8-130.5 s) and after (135.8-137 s), and
per 0.1 s cluster means around the fast samples; (2) GT angular rate (same +-0.1 s difference on R_bc) vs the
bias-removed gyro rate; (3) an IMU bound on translational speed change: |integral of (acc - local mean)| over
every 0.2 s window (the GT speed window) inside 133-137 s.
Time axis = s since the first IMU sample (docs/setdown_failure.html).
"""
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
os.chdir(SCAN3D)
sys.path.insert(0, str(HERE))
import z2_regress as zr  # noqa: E402
import c5_overnight_analyse as an  # noqa: E402
from imu_gt import read_imu  # noqa: E402
from gtnoise_still import gt_speed_any, ALIGN  # noqa: E402

REC = "hs_20261020_224323"
OUT = Path("runs/gtnoise")
PRE, POST = (118.8, 130.5), (135.8, 137.0)


def so3_angle(R):
    return np.arccos(np.clip((np.einsum("...ii", R) - 1) / 2, -1, 1))


def gate_eval():
    """Possible GT-quality gate for 3': keep a speed sample only if the sample and both +-V_HALF neighbours have
    >= NC_MIN corners and the neighbours have the same corner count (proxy: same visible corner subset).
    Evaluated on the certainly-still samples (gtnoise_still 'base') of all new recordings and on the held interval."""
    import gzip
    import h_tune as ht
    import gtnoise_still as gs
    out = {}
    for nc_min in (0, 12, 15):
        for same in (False, True):
            key = f"nc>={nc_min}{' same' if same else ''}"
            tot = dict(still_n=0, still_fast=0, still_vmax=0.0, held_n=0, held_fast=0, held_vmax=0.0)
            for rec in gs.RECS:
                acc, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=True)
                t0 = min(acc[0, 0], gyr[0, 0])
                dt = ALIGN[rec] * 1e6
                g = np.load(SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
                ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
                t, nc = g["t_ns"][ok], g["n_corners"][ok]
                t2, v = gt_speed_any(rec, "imu_gt_spl")
                ts = t / 1e9
                ja = np.clip(np.searchsorted(ts, ts + zr.V_HALF), 1, len(ts) - 1)
                jb = np.clip(np.searchsorted(ts, ts - zr.V_HALF), 1, len(ts) - 1)
                ja = np.where(np.abs(ts[ja - 1] - (ts + zr.V_HALF)) < np.abs(ts[ja] - (ts + zr.V_HALF)), ja - 1, ja)
                jb = np.where(np.abs(ts[jb - 1] - (ts - zr.V_HALF)) < np.abs(ts[jb] - (ts - zr.V_HALF)), jb - 1, jb)
                keep = np.isfinite(v) & (np.minimum(nc, np.minimum(nc[ja], nc[jb])) >= nc_min)
                if same:
                    keep &= nc[ja] == nc[jb]
                tq = (t - dt - t0) / 1e9
                tg_s, ta_s = (gyr[:, 0] - t0) / 1e9, (acc[:, 0] - t0) / 1e9
                gsd, gdev = gs.imu_stats(tg_s, gyr[:, 1:4], tq, gs.WIN)
                asd, _ = gs.imu_stats(ta_s, acc[:, 1:4], tq, gs.WIN)
                st = gs.still_mask(dict(gsd=gsd, asd=asd, gdev=gdev), "base")
                held = np.zeros(len(t), bool)
                if rec == REC:
                    d = json.load(open(gs.HELD_RUN + ".json"))
                    t_last = max(m["t_ns"] for m in map(json.loads, gzip.open(gs.HELD_RUN + ".poses.jsonl.gz", "rt"))
                                 if "t_ns" in m)
                    (s, e), = d["hold_segments_imu_ns"]
                    e = t_last if e is None else e
                    held = (t >= s + dt) & (t < e + dt)
                    st &= ~held
                for lab, m in (("still", st & keep), ("held", held & keep)):
                    tot[f"{lab}_n"] += int(m.sum())
                    tot[f"{lab}_fast"] += int((v[m] > zr.V_MOVE).sum())
                    tot[f"{lab}_vmax"] = max(tot[f"{lab}_vmax"], round(float(v[m].max(initial=0)) * 100, 3))
            out[key] = tot
            print("gate", key, tot, flush=True)
    return out


def main():
    acc, gyr = read_imu(Path("recordings") / REC / "imu.csv", raw=True)
    t0 = min(acc[0, 0], gyr[0, 0])
    tg, ta = (gyr[:, 0] - t0) / 1e9, (acc[:, 0] - t0) / 1e9
    dt = ALIGN[REC] * 1e6
    res = dict(rec=REC)
    still = (tg > 122) & (tg < 130)
    bias = np.median(gyr[still, 1:4], 0)
    res["gyro_bias_still_122_130"] = bias.round(5).tolist()
    for name in ("imu_gt_spl", "imu_gt_rs"):
        g = np.load(SCAN3D / "recordings" / REC / name / "gt.npz", allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        t, p, R, nc, rms = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok], g["n_corners"][ok], g["rms_px"][ok]
        tq = (t - dt - t0) / 1e9
        t2, v = gt_speed_any(REC, name)
        assert np.array_equal(t, t2)
        r = {}
        for lab, (a, b) in (("pre", PRE), ("post", POST)):
            m = (tq >= a) & (tq < b)
            r[lab] = dict(n=int(m.sum()), p_mean_mm=(p[m].mean(0) * 1000).round(2).tolist() if m.any() else None,
                          p_std_mm=(p[m].std(0) * 1000).round(2).tolist() if m.any() else None,
                          nc_med=float(np.median(nc[m])) if m.any() else None,
                          dist_m=round(float(np.linalg.norm(p[m], axis=1).mean()), 3) if m.any() else None)
        if r["pre"]["n"] and r["post"]["n"]:
            r["net_disp_mm"] = round(float(np.linalg.norm(np.subtract(r["post"]["p_mean_mm"], r["pre"]["p_mean_mm"]))), 2)
            m1, m2 = (tq >= PRE[0]) & (tq < PRE[1]), (tq >= POST[0]) & (tq < POST[1])
            ang = so3_angle(np.einsum("ji,jk->ik", R[m1].mean(0), R[m2].mean(0)))
            r["net_rot_deg"] = round(float(np.degrees(ang)), 3)
        # 0.1 s clusters 133-137 s
        cl = []
        for k in np.arange(133.0, 137.0, 0.1):
            m = (tq >= k) & (tq < k + 0.1)
            if m.any():
                cl.append(dict(t=round(float(k), 1), n=int(m.sum()), p_mm=(p[m].mean(0) * 1000).round(2).tolist(),
                               p_sd_mm=round(float(np.linalg.norm(p[m].std(0)) * 1000), 2), nc=int(np.median(nc[m])),
                               rms=round(float(np.median(rms[m])), 3),
                               vmax_cms=round(float(np.nanmax(v[m])) * 100, 2) if np.isfinite(v[m]).any() else None))
        r["clusters_133_137"] = cl
        # GT angular rate vs gyro at the GT samples with finite speed in 133-137
        ts = t / 1e9
        ja = np.clip(np.searchsorted(ts, ts + zr.V_HALF), 1, len(ts) - 1)
        jb = np.clip(np.searchsorted(ts, ts - zr.V_HALF), 1, len(ts) - 1)
        ja = np.where(np.abs(ts[ja - 1] - (ts + zr.V_HALF)) < np.abs(ts[ja] - (ts + zr.V_HALF)), ja - 1, ja)
        jb = np.where(np.abs(ts[jb - 1] - (ts - zr.V_HALF)) < np.abs(ts[jb] - (ts - zr.V_HALF)), jb - 1, jb)
        m = (tq >= 133) & (tq < 137) & np.isfinite(v)
        w_gt = so3_angle(np.einsum("nji,njk->nik", R[jb[m]], R[ja[m]])) / (ts[ja[m]] - ts[jb[m]])
        w_imu = []
        for i in np.where(m)[0]:
            a, b = tq[jb[i]], tq[ja[i]]
            s = (tg >= a) & (tg < b)
            w_imu.append(np.linalg.norm((gyr[s, 1:4] - bias).mean(0)))
        w_imu = np.array(w_imu)
        fast = v[m] > zr.V_MOVE
        r["rate_133_137"] = dict(n=int(m.sum()), gt_rate_med=round(float(np.median(w_gt)), 4),
                                 gt_rate_max=round(float(w_gt.max()), 4), imu_rate_med=round(float(np.median(w_imu)), 4),
                                 imu_rate_max=round(float(w_imu.max()), 4),
                                 fast_gt_rate=w_gt[fast].round(4).tolist(), fast_imu_rate=w_imu[fast].round(4).tolist(),
                                 fast_dist_m=np.linalg.norm(p[m][fast], axis=1).round(3).tolist())
        res[name] = r
    # IMU translational speed-change bound over 0.2 s windows (local mean removed over +-1 s; tilt change ignored)
    rows = []
    for c in np.arange(133.0, 137.0, 0.05):
        loc = (ta >= c - 1) & (ta < c + 1)
        w = (ta >= c - 0.1) & (ta < c + 0.1)
        a = acc[w, 1:4] - acc[loc, 1:4].mean(0)
        dv = np.abs(np.cumsum(a * np.r_[np.diff(ta[w]), 0][:, None], 0)).max(0)
        rows.append((round(float(c), 2), round(float(np.linalg.norm(dv)) * 100, 3)))
    res["imu_dv_0p2s_cms"] = rows
    res["imu_dv_0p2s_cms_max"] = max(x[1] for x in rows)
    rows2 = []
    for c in np.arange(122.0, 130.0, 0.05):     # same bound in the quiet part (noise floor of the bound)
        loc = (ta >= c - 1) & (ta < c + 1)
        w = (ta >= c - 0.1) & (ta < c + 0.1)
        a = acc[w, 1:4] - acc[loc, 1:4].mean(0)
        dv = np.abs(np.cumsum(a * np.r_[np.diff(ta[w]), 0][:, None], 0)).max(0)
        rows2.append(float(np.linalg.norm(dv)) * 100)
    res["imu_dv_0p2s_cms_quiet_122_130"] = dict(med=round(float(np.median(rows2)), 3), max=round(float(max(rows2)), 3))
    for a, c in ((130.5, 135.8), (133.0, 135.3)):        # gyro rotation (bias from 122-130 s) vs GT net rotation
        m = (tg >= a) & (tg < c)
        th = np.cumsum((gyr[m, 1:4] - bias) * np.r_[np.diff(tg[m]), 0][:, None], 0)
        res[f"gyro_rot_deg_{a}_{c}"] = round(float(np.degrees(np.linalg.norm(th[-1]))), 3)
    res["gate"] = gate_eval()
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(res, open(OUT / "gtnoise_held.json", "w"), indent=1, default=float)
    for name in ("imu_gt_spl", "imu_gt_rs"):
        r = res[name]
        print(name, "pre", r["pre"], "post", r["post"], "net", r.get("net_disp_mm"), "mm", r.get("net_rot_deg"), "deg")
        print("  rate", {k: v for k, v in r["rate_133_137"].items()})
        for c in r["clusters_133_137"]:
            print("   ", c)
    print("imu dv max 133-137", res["imu_dv_0p2s_cms_max"], "quiet", res["imu_dv_0p2s_cms_quiet_122_130"])
    print([x for x in res["imu_dv_0p2s_cms"] if x[1] > 0.5])


if __name__ == "__main__":
    main()
