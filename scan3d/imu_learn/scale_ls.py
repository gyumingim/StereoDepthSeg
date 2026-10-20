#!/usr/bin/env python3
"""GT scale from the accelerometer (integral form, linear least squares) — is the board square size right?

Per window of --win s (non-overlapping):  s * p_cam(t) = p0_w + v0_w*tau + g*tau^2/2 + DoubleInt(R_bc R_ci f)
unknowns: s and gravity g (3, board frame) shared by all windows, p0_w/v0_w per window (eliminated by projecting every
window onto the orthogonal complement of [1, tau], so the global solve has 4 unknowns). Bootstrap over windows → 95 % CI.
s > 1 → GT positions too small → true square = square_mm * s (before the method-bias correction).
  - accelerometer integrated on its own 500 Hz samples (frame drops of up to ~70 ms do not matter), board orientation
    slerped from the per-frame GT, dt / R_ci / IMU type (raw or calibrated) from <rec>/imu_gt/align.json (batch_sync)
  - IMU–camera lever arm and accelerometer bias are ignored → method bias; measure it on a recording whose square is known
    (monitor board, hs_20261019_163345: same tool, same settings) and divide by it (--ref-s).
Overnight 2026-10-20 runs/overnight_tmp/scale_ls.py (first 7.5 s only) → this file (whole recording, any board).
  ../venv_ffs/bin/python imu_learn/scale_ls.py recordings/hs_... [--gt-dir imu_gt_rs] [--ref-s 1.022] [--out x.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu, provenance  # noqa: E402


def windows_eq(rec, gt_dir="imu_gt_rs", win=1.0, max_gap_ms=30.0, min_ok=0.8, max_dist=1.5, smooth=5, raw=None,
               start_s=0.0, end_s=0.0):
    rec = Path(rec)
    g = np.load(rec / gt_dir / "gt.npz", allow_pickle=True)
    al = json.load(open(rec / "imu_gt" / "align.json"))
    raw = bool(al.get("imu_raw", False)) if raw is None else raw
    acc, _ = read_imu(rec / "imu.csv", raw=raw)
    t = g["t_ns"].astype(np.int64)
    ok = g["ok"].astype(bool) & (t > 0) & (np.linalg.norm(g["p_bc"], axis=1) < max_dist)
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        ok &= t >= json.load(open(trim))["start_ns"]
    to, Ro, Po = t[ok], g["R_bc"][ok], g["p_bc"][ok]
    keep = np.r_[True, np.diff(to) > 0]
    to, Ro, Po = to[keep], Ro[keep], Po[keep]
    if smooth > 1:                                       # light moving average over consecutive ok frames
        k = np.ones(smooth) / smooth
        Ps = np.stack([np.convolve(np.pad(Po[:, j], smooth // 2, mode="edge"), k, mode="valid") for j in range(3)], 1)
    else:
        Ps = Po
    sl = Slerp((to - to[0]) / 1e9, Rot.from_matrix(Ro))
    R_ci = np.asarray(al["R_ci"])
    ta = acc[:, 0].astype(np.int64) + int(al["dt_ns"])           # IMU samples on the camera clock
    lo = to[0] + int(start_s * 1e9)
    hi = (to[0] + int(end_s * 1e9)) if end_s else to[-1]
    nrm, info = [], []
    w0 = lo
    while w0 + win * 1e9 <= hi:
        w1 = w0 + int(win * 1e9)
        fi = np.where((to >= w0) & (to < w1))[0]
        tall = t[(t >= w0) & (t < w1) & (t > 0)]
        if len(fi) >= 20 and len(tall) and len(fi) >= min_ok * len(tall):
            gaps = np.diff(np.r_[w0, to[fi], w1])
            if gaps.max() < max_gap_ms * 1e6:
                ia = np.where((ta >= to[fi[0]]) & (ta <= to[fi[-1]]))[0]
                if len(ia) > 50:
                    tt = (ta[ia] - to[fi[0]]) / 1e9                       # seconds since the window's first frame
                    R = sl((ta[ia] - to[0]) / 1e9).as_matrix()
                    ab = np.einsum("nij,jk,nk->ni", R, R_ci, acc[ia, 1:4])
                    v = np.vstack([np.zeros(3), np.cumsum((ab[1:] + ab[:-1]) / 2 * np.diff(tt)[:, None], 0)])
                    D = np.vstack([np.zeros(3), np.cumsum((v[1:] + v[:-1]) / 2 * np.diff(tt)[:, None], 0)])
                    tf = (to[fi] - to[fi[0]]) / 1e9
                    Df = np.stack([np.interp(tf, tt, D[:, j]) for j in range(3)], 1)
                    B = np.c_[np.ones_like(tf), tf]                          # per-window nuisance p0 + v0 tau
                    Q = np.eye(len(tf)) - B @ np.linalg.pinv(B)
                    pc, h, Dr = Q @ Ps[fi], Q @ (0.5 * tf ** 2), Q @ Df
                    # rows per axis j:  s*pc_j - g_j*h = D_j   → x = [s, gx, gy, gz]
                    A = np.zeros((3 * len(tf), 4))
                    for j in range(3):
                        A[j::3, 0] = pc[:, j]
                        A[j::3, 1 + j] = -h
                    b = Dr.reshape(-1)
                    nrm.append((A.T @ A, A.T @ b, b @ b, len(b)))
                    lin = ab - ab.mean(0)
                    info.append(dict(t0=(w0 - to[0]) / 1e9, acc_dyn_rms=float(np.sqrt((lin ** 2).sum(1).mean())),
                                     path_m=float(np.linalg.norm(np.diff(Po[fi], axis=0), axis=1).sum())))
        w0 = w1
    return nrm, info, raw, float(np.asarray(g["square_mm"])) if "square_mm" in g else None


def solve(nrm, sel):
    AtA = sum(nrm[i][0] for i in sel)
    Atb = sum(nrm[i][1] for i in sel)
    x = np.linalg.solve(AtA, Atb)
    btb = sum(nrm[i][2] for i in sel)
    n = sum(nrm[i][3] for i in sel)
    res = max(btb - 2 * x @ Atb + x @ AtA @ x, 0) / n
    return x, float(np.sqrt(res))


def run(rec, gt_dir="imu_gt_rs", win=1.0, nboot=200, ref_s=None, raw=None, start_s=0.0, end_s=0.0):
    nrm, info, raw, sq = windows_eq(rec, gt_dir, win, raw=raw, start_s=start_s, end_s=end_s)
    W = len(nrm)
    if W < 5:
        return dict(rec=Path(rec).name, gt_dir=gt_dir, windows=W, error="too few windows")
    x, res = solve(nrm, range(W))
    rng = np.random.default_rng(0)
    bs = np.array([solve(nrm, rng.integers(0, W, W))[0][0] for _ in range(nboot)])
    out = dict(rec=Path(rec).name, gt_dir=gt_dir, win_s=win, windows=W, imu=("raw35" if raw else "cal1"),
               s=round(float(x[0]), 4), s_ci95=[round(float(np.percentile(bs, 2.5)), 4), round(float(np.percentile(bs, 97.5)), 4)],
               s_boot_std=round(float(bs.std()), 4), g_norm=round(float(np.linalg.norm(x[1:])), 3), res_cm=round(res * 100, 3),
               acc_dyn_rms_med=round(float(np.median([i["acc_dyn_rms"] for i in info])), 3),
               gt_square_mm=sq, start_s=start_s, end_s=end_s)
    if sq:
        out["square_mm_raw"] = round(sq * x[0], 2)
        if ref_s:
            out.update(ref_s=ref_s, s_corr=round(float(x[0]) / ref_s, 4),
                       square_mm_est=round(sq * x[0] / ref_s, 2),
                       square_mm_ci95=[round(sq * c / ref_s, 2) for c in out["s_ci95"]])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--gt-dir", default="imu_gt_rs")
    ap.add_argument("--win", type=float, default=1.0)
    ap.add_argument("--ref-s", type=float, default=None, help="method bias s measured on a known-square recording")
    ap.add_argument("--raw", choices=["auto", "raw", "cal"], default="auto", help="IMU type (auto = align.json imu_raw)")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=0.0, help="seconds after the first GT frame (0 = to the end)")
    ap.add_argument("--out", default=None, help="json (list of results)")
    a = ap.parse_args()
    raw = None if a.raw == "auto" else a.raw == "raw"
    res = []
    for r in a.recs:
        o = run(r, a.gt_dir, a.win, ref_s=a.ref_s, raw=raw, start_s=a.start, end_s=a.end)
        o["provenance"] = provenance([__file__])
        res.append(o)
        print(f"RESULT {o['rec']} " + json.dumps({k: v for k, v in o.items() if k not in ("rec", "provenance")}), flush=True)
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
