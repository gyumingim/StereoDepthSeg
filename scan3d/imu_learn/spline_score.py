#!/usr/bin/env python3
"""Fair scores for choosing the spline knot spacing (and comparing with the per-frame GT) — NOT the training residual.

(a) held-out corners: every 5th frame's corners are left out of the fit; reprojection error (px) of those corners at
    their own row times. Spline: evaluated directly. Per-frame GT (imu_gt_rs): its own pose for that frame was fitted
    to these very corners, so it is interpolated (slerp / lerp) from the neighbouring kept frames at each corner's row
    time. Smoothed per-frame GT (gt_smooth, what train_imu --gt-smooth uses): smoothed on the kept frames, interpolated.
(b) gyro consistency: rotation of the camera over 0.2 s from the GT vs the integrated gyroscope (align.json dt, R_ci,
    gyro bias knots), median angle (deg), all windows and fast ones (|ω| > 1 rad/s). Fitted on ALL frames.
  ../venv_ffs/bin/python imu_learn/spline_score.py recordings/hs_A@10:30 recordings/hs_B@10:30 ... [--knots ...]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation as Rot, Slerp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import spline_gt as S  # noqa: E402
from gt_smooth import smooth_gt  # noqa: E402
from imu_gt import read_imu  # noqa: E402


def interp_pose(tk, Rk, pk, t):
    """slerp / lerp of poses (tk sorted, s) at times t (clipped)."""
    t = np.clip(t, tk[0], tk[-1])
    R = Slerp(tk, Rot.from_matrix(Rk))(t).as_matrix()
    p = np.stack([np.interp(t, tk, pk[:, j]) for j in range(3)], 1)
    return R, p


def reproj_np(D, R, p, X, obs):
    xc = np.einsum("nji,nj->ni", R, X - p)
    return D["f"] * np.linalg.norm(xc[:, :2] / xc[:, 2:3] - obs, axis=1)


def stats(e):
    return dict(median=round(float(np.median(e)), 4), rms=round(float(np.sqrt(np.mean(e ** 2))), 4),
                p90=round(float(np.quantile(e, 0.9)), 4), n=int(len(e)))


def perframe(D, holdout):
    """kept per-frame GT poses (ok, near, deduplicated, not held out) as seconds since T0."""
    g = D["g"]
    ok = g["ok"].astype(bool) & (g["t_ns"] > 0) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    ok &= (g["t_ns"] >= D["lo"] - 1e9) & (g["t_ns"] <= D["hi"] + 1e9)
    if holdout:
        ok &= np.arange(len(ok)) % holdout != 0
    tg, Rg, pg = g["t_ns"][ok], g["R_bc"][ok], g["p_bc"][ok]
    k = np.r_[True, np.diff(tg) > 10_000]
    return tg[k], Rg[k], pg[k]


def smoothed(tg, Rg, pg):
    keep, Rs, ps = smooth_gt(tg, Rg, pg)
    return tg[keep], Rs[keep], ps[keep]


class Gyro:
    def __init__(self, rec, T0, lo, hi, hz=4000):
        al = json.load(open(rec / "imu_gt" / "align.json"))
        _, gyr = read_imu(rec / "imu.csv")
        self.R_ci = np.asarray(al["R_ci"])
        tg = (gyr[:, 0] + int(al["dt_ns"]) - T0) / 1e9
        bk = np.asarray(al.get("gyro_bias_knots", [[0, 0, 0]]))
        ks = float(al.get("knot_s", 20.0))
        tb0 = (max(gyr[0, 0], 0) + int(al["dt_ns"]) - T0) / 1e9      # batch_sync knots start at the first IMU sample
        self.t = np.arange((lo - T0) / 1e9 - 0.3, (hi - T0) / 1e9 + 0.3, 1 / hz)
        w = np.stack([np.interp(self.t, tg, gyr[:, 1 + j]) for j in range(3)], 1)
        tb = tb0 + np.arange(len(bk)) * ks
        b = np.stack([np.interp(self.t, tb, bk[:, j]) for j in range(3)], 1) if len(bk) > 1 else np.repeat(bk, len(self.t), 0)
        w = w - b
        self.w = w
        E = Rot.from_rotvec(w / hz)
        q = np.empty((len(self.t), 4))
        cur = Rot.identity()
        Es = E.as_quat()
        for i in range(len(self.t)):                                 # Q_i = product of steps before sample i
            q[i] = cur.as_quat()
            cur = cur * Rot.from_quat(Es[i])
        self.Q = Rot.from_quat(q)
        self.hz = hz

    def rel_cam(self, ta, tb):
        ia = np.clip(np.round((ta - self.t[0]) * self.hz).astype(int), 0, len(self.t) - 1)
        ib = np.clip(np.round((tb - self.t[0]) * self.hz).astype(int), 0, len(self.t) - 1)
        Qi = (self.Q[ia].inv() * self.Q[ib]).as_matrix()
        return np.einsum("ij,njk,lk->nil", self.R_ci, Qi, self.R_ci)

    def omega_cam(self, t):
        i = np.clip(np.round((t - self.t[0]) * self.hz).astype(int), 0, len(self.t) - 1)
        return self.w[i] @ self.R_ci.T


def gyro_score(G, t_frames, pose_fn, win=0.2, step=6):
    """t_frames: per-frame GT times (s), pose_fn(t) → R (N,3,3). Windows start at every `step`-th frame."""
    ta = t_frames[::step]
    tb = ta + win
    m = tb <= t_frames[-1]
    ta, tb = ta[m], tb[m]
    j = np.searchsorted(t_frames, tb)                             # end at an actual frame time (≤ 1.5× window)
    j = np.clip(j, 0, len(t_frames) - 1)
    tb = t_frames[j]
    m = (tb - ta > 0.8 * win) & (tb - ta < 1.5 * win)
    ta, tb = ta[m], tb[m]
    Ra, Rb = pose_fn(ta), pose_fn(tb)
    dR = np.einsum("nji,njk->nik", Ra, Rb)
    big = np.degrees(np.linalg.norm(Rot.from_matrix(dR).as_rotvec(), axis=1)) >= 2
    Qc = G.rel_cam(ta, tb)
    e = np.degrees(np.linalg.norm(Rot.from_matrix(np.einsum("nji,njk->nik", dR, Qc)).as_rotvec(), axis=1))
    fast = np.linalg.norm(G.omega_cam(ta), axis=1) > 1.0
    return dict(gyro_deg=round(float(np.median(e[big])), 4), gyro_fast_deg=round(float(np.median(e[big & fast])), 4),
                n=int(big.sum()), n_fast=int((big & fast).sum()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+", help="folder@start:secs")
    ap.add_argument("--knots", type=float, nargs="+", default=[0.1, 0.05, 0.025, 0.0125])
    ap.add_argument("--holdout", type=int, default=5)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--out", default=str(HERE.parent / "runs" / "spline_score.json"))
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    res = {}
    for spec in a.recs:
        rec, rng = spec.split("@")
        st, secs = (float(v) for v in rng.split(":"))
        name = Path(rec).name
        R = res[name] = dict(start=st, secs=secs, holdout={}, gyro={})
        t0 = time.time()
        Dh = S.load(rec, "imu_gt_rs", st, secs, holdout=a.holdout)
        Da = S.load(rec, "imu_gt_rs", st, secs)
        ho = Dh["ho"]
        print(f"{name} {st}+{secs} s: fit corners {len(Dh['fi'])}, held-out corners {len(ho['fi'])} "
              f"in {len(np.unique(ho['fi']))} frames", flush=True)
        # per-frame GT interpolated from neighbours, and the smoothed version
        tg, Rg, pg = perframe(Dh, a.holdout)
        tgs = (tg - Dh["T0"]) / 1e9
        Ri, pi = interp_pose(tgs, Rg, pg, ho["t"])
        R["holdout"]["perframe_interp"] = stats(reproj_np(Dh, Ri, pi, ho["X"], ho["obs"]))
        ts_, Rs_, ps_ = smoothed(tg, Rg, pg)
        Ri, pi = interp_pose((ts_ - Dh["T0"]) / 1e9, Rs_, ps_, ho["t"])
        R["holdout"]["smooth_interp"] = stats(reproj_np(Dh, Ri, pi, ho["X"], ho["obs"]))
        # gyro consistency
        G = Gyro(Path(rec), Da["T0"], Da["lo"], Da["hi"])
        tga, Rga, pga = perframe(Da, 0)
        tgas = (tga - Da["T0"]) / 1e9
        sel = (tgas >= (Da["lo"] - Da["T0"]) / 1e9) & (tgas <= (Da["hi"] - Da["T0"]) / 1e9)
        tf = tgas[sel]
        R["gyro"]["perframe"] = gyro_score(G, tf, lambda t: interp_pose(tgas, Rga, pga, t)[0])
        tss, Rss, pss = smoothed(tga, Rga, pga)
        R["gyro"]["smooth"] = gyro_score(G, tf, lambda t: interp_pose((tss - Da["T0"]) / 1e9, Rss, pss, t)[0])
        for k in a.knots:
            Fh = S.fit(Dh, k, False, a.iters, dev, log=lambda s: None)
            Rh, ph = S.eval_pose(Fh, ho["t"], dev)
            R["holdout"][f"spline_{k}"] = stats(reproj_np(Dh, Rh, ph, ho["X"], ho["obs"]))
            R["holdout"][f"spline_{k}"]["fit_median"] = round(float(np.median(Fh["corner_err"])), 4)
            Fa = S.fit(Da, k, False, a.iters, dev, log=lambda s: None)
            R["gyro"][f"spline_{k}"] = gyro_score(G, tf, lambda t: S.eval_pose(Fa, t, dev)[0])
            # position difference vs per-frame GT at the frame times (size of the change of ruler)
            Rp, pp = S.eval_pose(Fa, tf, dev)
            R["gyro"][f"spline_{k}"]["dpos_mm_median"] = round(float(np.median(np.linalg.norm(pp - pga[sel], axis=1)) * 1000), 3)
            print(f"  knot {k}: held-out {R['holdout'][f'spline_{k}']}  gyro {R['gyro'][f'spline_{k}']}", flush=True)
        print(f"  per-frame interp {R['holdout']['perframe_interp']}  smooth interp {R['holdout']['smooth_interp']}")
        print(f"  gyro per-frame {R['gyro']['perframe']}  smooth {R['gyro']['smooth']}  ({time.time() - t0:.0f} s)", flush=True)
        json.dump(res, open(a.out, "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
