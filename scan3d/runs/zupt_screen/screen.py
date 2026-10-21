#!/usr/bin/env python3
"""zupt_screen step 2 (post-hoc, exploratory, analysis only): first-order screen of zupt_max_disparity thresholds.
Per recording: unique frames (union of all non-held runs, 60 fps 640x360 npts 200), median disparity across runs.
Labels: GT speed (z2_regress.gt_speed, +-0.1 s central diff, > 2 cm/s), GT quality (corners >= 12 at the sample),
IMU still (i3_confirm.still_runs on raw 16 gyro, or Android type 4 for recordings before 2026-10-20)."""
import sys, json, pickle, time
from pathlib import Path
from collections import defaultdict
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "imu_learn"))
import os
os.chdir(ROOT)
import z2_regress as zr
import i3_confirm as ic
from imu_gt import read_imu

THRS = [0.5, 0.75, 1.0, 1.5]
V_MOVE = 0.02
NF_MIN = 20          # OpenVINS: passed = disp < thr && num_features > 20
BIN_NS = 4_000_000   # frame identity across runs: t rounded to 4 ms (60 fps -> 16.7 ms apart)

D = pickle.load(open(ROOT / "runs/zupt_screen/frames.pkl", "rb"))
by = defaultdict(list)
for r in D["runs"]:
    if not r["held"]:
        by[r["rec"]].append(r)

out = dict(made=time.strftime("%F %T"), input="runs/zupt_screen/frames.pkl made " + D["made"], thresholds=THRS,
           note="post-hoc exploratory first-order screen; disparity from logged (real) filter path", recs={})
per_frame = {}
for rec in sorted(by):
    runs = by[rec]
    # union of frames, median disparity across runs
    acc = defaultdict(list)
    for r in runs:
        k = np.round(r["t"] / BIN_NS).astype(np.int64)
        for kk, dd, nf in zip(k, r["disp"], r["nf"]):
            acc[kk].append((dd, nf))
    keys = np.array(sorted(acc))
    disp = np.array([np.median([x[0] for x in acc[k]]) for k in keys])
    dspread = np.array([(max(x[0] for x in acc[k]) - min(x[0] for x in acc[k])) for k in keys])
    nruns = np.array([len(acc[k]) for k in keys])
    nf = np.array([np.median([x[1] for x in acc[k]]) for k in keys])
    t = keys * BIN_NS
    # IMU still
    try:
        _, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=True); imu_src = "raw16"
    except ValueError:
        _, gyr = read_imu(Path("recordings") / rec / "imu.csv", raw=False); imu_src = "android4 (가정: 보정 자이로)"
    gt_ = gyr[:, 0].astype(np.int64)
    sr = ic.still_runs(gt_, gyr[:, 1:4])
    still = np.zeros(len(t), bool)
    for a, b in sr:
        still |= (t >= a) & (t <= b)
    # gyro |w| over +-25 ms (bias = median gyro inside stills)
    bias = np.median(np.concatenate([gyr[(gt_ >= a) & (gt_ <= b), 1:4] for a, b in sr]), 0) if sr else np.zeros(3)
    i0 = np.searchsorted(gt_, t - 25_000_000); i1 = np.searchsorted(gt_, t + 25_000_000)
    wn = np.array([np.linalg.norm((gyr[a:b, 1:4] - bias).mean(0)) if b > a else np.nan for a, b in zip(i0, i1)])
    # GT speed
    al = np.median([r["align_dt_ms"] for r in runs if r["align_dt_ms"] is not None])
    vgt = np.full(len(t), np.nan); cq = np.zeros(len(t), bool)
    gtdir = Path("recordings") / rec / "imu_gt_spl" / "gt.npz"
    if gtdir.exists():
        gt_t, gt_v = zr.gt_speed(rec)
        g = np.load(gtdir, allow_pickle=True)
        ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
        nc = g["n_corners"][ok]
        tq = t + int(al * 1e6)
        j = np.clip(np.searchsorted(gt_t, tq), 1, len(gt_t) - 1)
        j = np.where(np.abs(gt_t[j - 1] - tq) < np.abs(gt_t[j] - tq), j - 1, j)
        m = np.abs(gt_t[j] - tq) < 10_000_000
        vgt[m] = gt_v[j[m]]
        # quality: sample and its +-0.1 s neighbours have >= 12 corners
        tsec = gt_t / 1e9
        ja = np.clip(np.searchsorted(tsec, tsec + 0.1), 0, len(tsec) - 1)
        jb = np.clip(np.searchsorted(tsec, tsec - 0.1), 0, len(tsec) - 1)
        q = (nc >= 12) & (nc[ja] >= 12) & (nc[jb] >= 12)
        cq[m] = q[j[m]]
    gmove = np.isfinite(vgt) & (vgt > V_MOVE)
    gmove_q = gmove & cq
    gstill_q = np.isfinite(vgt) & (vgt <= V_MOVE) & cq
    nfok = nf > NF_MIN
    R = dict(n_runs=len(runs), n_frames=int(len(t)), imu=imu_src, still_runs_s=len(sr),
             frac_imu_still=round(float(still.mean()), 3), gt_frames=int(np.isfinite(vgt).sum()),
             disp_spread_med=round(float(np.median(dspread[nruns > 1])), 4) if (nruns > 1).any() else None,
             disp_spread_p95=round(float(np.percentile(dspread[nruns > 1], 95)), 4) if (nruns > 1).any() else None,
             still_disp_pct=[round(float(x), 3) for x in np.percentile(disp[still], [10, 50, 90])] if still.any() else None,
             thr={})
    for th in THRS:
        p = (disp < th) & nfok
        new = p & ~((disp < 0.5) & nfok)
        R["thr"][str(th)] = dict(
            pass_all=int(p.sum()),
            pass_gtmove=int((p & gmove).sum()), pass_gtmove_q=int((p & gmove_q).sum()),
            pass_imumove=int((p & ~still).sum()),
            new_gtmove=int((new & gmove).sum()), new_gtmove_q=int((new & gmove_q).sum()),
            new_imumove=int((new & ~still).sum()),
            new_imumove_and_gtmove=int((new & ~still & gmove).sum()),
            new_still_gain=int((new & still).sum()), still_pass=int((p & still).sum()), still_n=int(still.sum()),
            new_gtstill_q=int((new & gstill_q).sum()),
            new_imumove_w_med=round(float(np.nanmedian(wn[new & ~still])), 4) if (new & ~still).any() else None,
            new_gtmove_v_max=round(float(np.nanmax(vgt[new & gmove])), 4) if (new & gmove).any() else None)
    out["recs"][rec] = R
    per_frame[rec] = dict(t=t, disp=disp, nf=nf, still=still, vgt=vgt, cq=cq, wn=wn, nruns=nruns, still_runs=sr)
    print(rec, json.dumps(R, ensure_ascii=False))

json.dump(out, open(ROOT / "runs/zupt_screen/screen.json", "w"), indent=1, ensure_ascii=False)
pickle.dump(per_frame, open(ROOT / "runs/zupt_screen/per_frame.pkl", "wb"))
