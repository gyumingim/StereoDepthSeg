#!/usr/bin/env python3
"""Why is the VIO ATE floor 0.5-1.2 cm?  GT error vs VIO error (2026-10-22, docs/ate_floor_20261022.md).

  still : static-gravity check of the raw accelerometer (type 35): |a| in stills vs local g (Seoul 9.7995 m/s^2)
  scale : IMU-vs-board GT scale (same equations as scale_ls.py) + variants: per-board-axis scale, lever arm,
          time shift, motion terciles
  vio   : ATE decomposition of finished VIO runs (SE3, Sim3, GT x s_IMU, per-axis scale, frequency split,
          local 10 s Sim3, GT variant rs vs spl)
Read-only on recordings and runs; writes runs/ate_floor/*.json.
  ../venv_ffs/bin/python imu_learn/ate_floor.py still|scale|vio
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402

OUT = SCAN3D / "runs" / "ate_floor"
G_SEOUL = 9.7995   # Somigliana at 37.5 N, ~50 m (estimate +-0.0005)
LAPTOP = ["hs_20261021_213341", "hs_20261021_213955", "hs_20261021_214550", "hs_20261021_215348"]
MONITOR = ["hs_20261022_111605", "hs_20261022_111836"]
MON19 = ["hs_20261019_114000", "hs_20261019_115620", "hs_20261019_125714", "hs_20261019_131326",
         "hs_20261019_132803", "hs_20261019_145816", "hs_20261019_151358", "hs_20261019_152631",
         "hs_20261019_154320", "hs_20261019_161035", "hs_20261019_163345"]


# ----------------------------------------------------------------------------------------------------------- still
def stills(acc, gyr, win=1.0, wthr=0.03, athr=0.03):
    t = acc[:, 0]
    out = []
    t0 = t[0]
    gi = np.searchsorted(gyr[:, 0], t)
    while t0 + win * 1e9 < t[-1]:
        m = (t >= t0) & (t < t0 + win * 1e9)
        if m.sum() > 50:
            gw = gyr[np.clip(gi[m], 0, len(gyr) - 1), 1:4]
            gw = gw - np.median(gw, 0)                         # raw gyro has a bias; still = no change
            a = acc[m, 1:4]
            if np.abs(gw).max() < 0.1 and np.sqrt((gw ** 2).sum(1).mean()) < wthr and a.std(0).max() < athr:
                out.append(dict(t=float((t0 - t[0]) / 1e9), a=a.mean(0).tolist(), n=int(m.sum())))
        t0 += win * 1e9
    return out


def cmd_still():
    recs = sorted(p for p in (SCAN3D / "recordings").iterdir() if p.name.startswith(("hs_2026102", "static_imu")))
    res = []
    for r in recs:
        if not (r / "imu.csv").exists():
            continue
        try:
            acc, gyr = read_imu(r / "imu.csv", raw=True)
            kind = "raw35"
        except ValueError:
            acc, gyr = read_imu(r / "imu.csv", raw=False)
            kind = "cal1"
        S = stills(acc, gyr)
        for s in S:
            a = np.array(s["a"])
            ax = int(np.argmax(np.abs(a)))
            s.update(rec=r.name, imu=kind, norm=float(np.linalg.norm(a)), face=("+-"[int(a[ax] < 0)] + "xyz"[ax]))
        # merge consecutive still seconds of the same face into one entry (one pose)
        poses = []
        for s in S:
            if poses and poses[-1]["face"] == s["face"] and abs(s["t"] - poses[-1]["t1"] - 1) < 1e-6 and \
                    np.linalg.norm(np.array(s["a"]) - np.array(poses[-1]["a_last"])) < 0.05:
                p = poses[-1]
                p["t1"] = s["t"]; p["n"] += 1; p["a_sum"] += np.array(s["a"]); p["a_last"] = s["a"]
            else:
                poses.append(dict(rec=r.name, imu=s["imu"], face=s["face"], t0=s["t"], t1=s["t"], n=1,
                                  a_sum=np.array(s["a"]), a_last=s["a"]))
        for p in poses:
            a = p.pop("a_sum") / p["n"]
            p.pop("a_last")
            p.update(a=[round(float(x), 4) for x in a], norm=round(float(np.linalg.norm(a)), 4))
            res.append(p)
        print(f"{r.name}: {kind}, still seconds {len(S)}, poses {len(poses)}", flush=True)
    # least-squares per-axis gain + bias from all raw poses (needs >= 2 faces per axis to separate gain and bias)
    raw = [p for p in res if p["imu"] == "raw35" and p["n"] >= 2]
    A = np.array([p["a"] for p in raw])
    W = np.array([min(p["n"], 30) for p in raw], float)
    fit = None
    if len(A) >= 6:
        from scipy.optimize import least_squares

        def f(x):
            k, b = x[:3], x[3:]
            return np.sqrt(W) * (np.linalg.norm((A - b) * k, axis=1) - G_SEOUL)
        sol = least_squares(f, np.r_[np.ones(3), np.zeros(3)])
        J = sol.jac
        cov = np.linalg.pinv(J.T @ J) * (sol.fun ** 2).sum() / max(1, len(A) - 6)
        fit = dict(gain=[round(float(x), 5) for x in sol.x[:3]], bias=[round(float(x), 4) for x in sol.x[3:]],
                   gain_sd=[round(float(np.sqrt(cov[i, i])), 5) for i in range(3)],
                   res_rms=round(float(np.sqrt(np.mean((sol.fun / np.sqrt(W)) ** 2))), 4), poses=len(A),
                   faces=sorted(set(p["face"] for p in raw)))
    by_face = {}
    for p in raw:
        by_face.setdefault(p["face"], []).append(p["norm"])
    summ = {f: dict(poses=len(v), norm_med=round(float(np.median(v)), 4), norm_min=round(min(v), 4), norm_max=round(max(v), 4),
                    gain_vs_g=round(float(np.median(v)) / G_SEOUL, 4)) for f, v in sorted(by_face.items())}
    out = dict(made=time.strftime("%F %T"), g_local=G_SEOUL, by_face=summ, fit_gain_bias=fit, poses=res)
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(OUT / "still.json", "w"), indent=1)
    print(json.dumps(dict(by_face=summ, fit=fit), indent=1))


# ----------------------------------------------------------------------------------------------------------- scale
def scale_windows(rec, gt_dir="imu_gt_spl", win=1.0, max_gap_ms=30.0, min_ok=0.8, max_dist=1.5, smooth=5,
                  shift_ns=0, raw=None, q2=False):
    """Same window equations as scale_ls.windows_eq but returns per-window projected pieces:
    pc (Q p_gt, n x 3), h (Q tau^2/2), Dr (Q double-integral, n x 3), RL (Q R_bc(t), n x 3 x 3 for lever arm)."""
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
    k = np.ones(smooth) / smooth
    Ps = np.stack([np.convolve(np.pad(Po[:, j], smooth // 2, mode="edge"), k, mode="valid") for j in range(3)], 1)
    sl = Slerp((to - to[0]) / 1e9, Rot.from_matrix(Ro))
    R_ci = np.asarray(al["R_ci"])
    ta = acc[:, 0].astype(np.int64) + int(al["dt_ns"]) + int(shift_ns)
    W = []
    w0 = to[0]
    while w0 + win * 1e9 <= to[-1]:
        w1 = w0 + int(win * 1e9)
        fi = np.where((to >= w0) & (to < w1))[0]
        tall = t[(t >= w0) & (t < w1) & (t > 0)]
        if len(fi) >= 20 and len(tall) and len(fi) >= min_ok * len(tall):
            gaps = np.diff(np.r_[w0, to[fi], w1])
            if gaps.max() < max_gap_ms * 1e6:
                ia = np.where((ta >= to[fi[0]]) & (ta <= to[fi[-1]]))[0]
                if len(ia) > 50:
                    tt = (ta[ia] - to[fi[0]]) / 1e9
                    R = sl((ta[ia] - to[0]) / 1e9).as_matrix()
                    ab = np.einsum("nij,jk,nk->ni", R, R_ci, acc[ia, 1:4])
                    v = np.vstack([np.zeros(3), np.cumsum((ab[1:] + ab[:-1]) / 2 * np.diff(tt)[:, None], 0)])
                    D = np.vstack([np.zeros(3), np.cumsum((v[1:] + v[:-1]) / 2 * np.diff(tt)[:, None], 0)])
                    tf = (to[fi] - to[fi[0]]) / 1e9
                    Df = np.stack([np.interp(tf, tt, D[:, j]) for j in range(3)], 1)
                    B = np.c_[np.ones_like(tf), tf, tf ** 2] if q2 else np.c_[np.ones_like(tf), tf]
                    Q = np.eye(len(tf)) - B @ np.linalg.pinv(B)   # q2: also a constant acceleration per window
                    lin = ab - ab.mean(0)
                    om = Rot.from_matrix(Ro[fi]).as_rotvec()
                    rel = Rot.from_matrix(Ro[fi[0]]).inv() * Rot.from_matrix(Ro[fi])
                    W.append(dict(pc=Q @ Ps[fi], h=Q @ (0.5 * tf ** 2), Dr=Q @ Df,
                                  RL=np.einsum("ab,bij->aij", Q, Ro[fi]),
                                  acc_dyn=float(np.sqrt((lin ** 2).sum(1).mean())),
                                  path=float(np.linalg.norm(np.diff(Po[fi], axis=0), axis=1).sum()),
                                  dist=float(np.median(np.linalg.norm(Po[fi], axis=1))),
                                  rot=float(np.max(np.linalg.norm(rel.as_rotvec(), axis=1))),
                                  t0=float((w0 - to[0]) / 1e9)))
        w0 = w1
    return W, float(np.asarray(g["square_mm"])) if "square_mm" in g else None


def fit(W, mode="iso", sel=None):
    """mode iso: [s, g3]; axes: [sx, sy, sz, g3] (board axes); lever: [s, g3, r3] (r = IMU offset in camera
    frame, p_imu = p_cam + R_bc r).  Returns x, residual rms (m)."""
    sel = range(len(W)) if sel is None else sel
    AtA = Atb = None
    btb = n = 0
    for i in sel:
        w = W[i]
        m = len(w["h"])
        if mode == "iso":
            A = np.zeros((3 * m, 4))
            for j in range(3):
                A[j::3, 0] = w["pc"][:, j]
                A[j::3, 1 + j] = -w["h"]
        elif mode == "axes":
            A = np.zeros((3 * m, 6))
            for j in range(3):
                A[j::3, j] = w["pc"][:, j]
                A[j::3, 3 + j] = -w["h"]
        elif mode in ("q2", "q2lever"):          # windows built with q2=True: gravity/bias per window removed
            A = np.zeros((3 * m, 4 if mode == "q2lever" else 1))
            for j in range(3):
                A[j::3, 0] = w["pc"][:, j]
                if mode == "q2lever":
                    A[j::3, 1:4] = w["RL"][:, j, :]
        else:
            A = np.zeros((3 * m, 7))
            for j in range(3):
                A[j::3, 0] = w["pc"][:, j]
                A[j::3, 1 + j] = -w["h"]
                A[j::3, 4:7] = w["RL"][:, j, :]
        b = w["Dr"].reshape(-1)
        AtA = A.T @ A if AtA is None else AtA + A.T @ A
        Atb = A.T @ b if Atb is None else Atb + A.T @ b
        btb += b @ b
        n += len(b)
    x = np.linalg.solve(AtA, Atb)
    res = max(btb - 2 * x @ Atb + x @ AtA @ x, 0) / n
    return x, float(np.sqrt(res))


def boot(W, mode, idx, nb=200):
    rng = np.random.default_rng(0)
    return np.array([fit(W, mode, rng.integers(0, len(W), len(W)))[0][idx] for _ in range(nb)])


def cmd_scale():
    recs = LAPTOP + MONITOR + MON19
    out = []
    for name in recs:
        rec = SCAN3D / "recordings" / name
        gd = "imu_gt_spl" if (rec / "imu_gt_spl" / "gt.npz").exists() else "imu_gt_rs"
        if not (rec / gd / "gt.npz").exists():
            print(name, "no GT", flush=True)
            continue
        W, sq = scale_windows(rec, gd)
        if len(W) < 8:
            print(name, "few windows", len(W), flush=True)
            continue
        r = dict(rec=name, gt=gd, windows=len(W), square_mm=sq)
        x, res = fit(W, "iso")
        bs = boot(W, "iso", 0)
        r.update(s=round(x[0], 4), s_ci=[round(np.percentile(bs, 2.5), 4), round(np.percentile(bs, 97.5), 4)],
                 g_norm=round(float(np.linalg.norm(x[1:4])), 3), res_cm=round(res * 100, 3))
        x, res = fit(W, "axes")
        r.update(s_axes=[round(v, 4) for v in x[:3]], res_axes_cm=round(res * 100, 3))
        bz = boot(W, "axes", 2, 100)
        bx = boot(W, "axes", 0, 100)
        r.update(sz_ci=[round(np.percentile(bz, 2.5), 4), round(np.percentile(bz, 97.5), 4)],
                 sx_ci=[round(np.percentile(bx, 2.5), 4), round(np.percentile(bx, 97.5), 4)])
        x, res = fit(W, "lever")
        bl = boot(W, "lever", 0, 100)
        r.update(s_lever=round(x[0], 4), s_lever_ci=[round(np.percentile(bl, 2.5), 4), round(np.percentile(bl, 97.5), 4)],
                 lever_cm=[round(v * 100, 2) for v in x[4:7]], res_lever_cm=round(res * 100, 3))
        # time shift of the IMU (ms) -> s, residual
        sh = {}
        for d in (-6, -3, 3, 6):
            Wd, _ = scale_windows(rec, gd, shift_ns=d * 1_000_000)
            xd, rd = fit(Wd, "iso")
            sh[str(d)] = [round(xd[0], 4), round(rd * 100, 4)]
        r["shift_ms"] = sh
        # motion / distance terciles (same windows)
        for key in ("acc_dyn", "dist", "rot"):
            v = np.array([w[key] for w in W])
            q = np.quantile(v, [1 / 3, 2 / 3])
            parts = [np.where(v <= q[0])[0], np.where((v > q[0]) & (v <= q[1]))[0], np.where(v > q[1])[0]]
            r[f"terc_{key}"] = [dict(med=round(float(np.median(v[p])), 3), s=round(fit(W, "iso", p)[0][0], 4)) for p in parts]
        r["acc_dyn_med"] = round(float(np.median([w["acc_dyn"] for w in W])), 3)
        r["rot_med"] = round(float(np.median([w["rot"] for w in W])), 3)
        r["dist_med"] = round(float(np.median([w["dist"] for w in W])), 3)
        out.append(r)
        print(json.dumps(r), flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(dict(made=time.strftime("%F %T"), results=out), open(OUT / "scale.json", "w"), indent=1)


def cmd_scale2():
    """per-window constant acceleration removed (insensitive to accel bias and to the gravity value), with and
    without the lever arm"""
    out = []
    for name in LAPTOP + MONITOR + MON19:
        rec = SCAN3D / "recordings" / name
        gd = "imu_gt_spl" if (rec / "imu_gt_spl" / "gt.npz").exists() else "imu_gt_rs"
        if not (rec / gd / "gt.npz").exists():
            continue
        W, _ = scale_windows(rec, gd, q2=True)
        if len(W) < 8:
            continue
        r = dict(rec=name, windows=len(W))
        x, res = fit(W, "q2")
        b = boot(W, "q2", 0, 100)
        r.update(s_q2=round(x[0], 4), s_q2_ci=[round(np.percentile(b, 2.5), 4), round(np.percentile(b, 97.5), 4)], res_q2_cm=round(res * 100, 3))
        x, res = fit(W, "q2lever")
        b = boot(W, "q2lever", 0, 100)
        r.update(s_q2lever=round(x[0], 4), s_q2lever_ci=[round(np.percentile(b, 2.5), 4), round(np.percentile(b, 97.5), 4)],
                 lever_q2_cm=[round(v * 100, 2) for v in x[1:4]], res_q2lever_cm=round(res * 100, 3))
        out.append(r)
        print(json.dumps(r), flush=True)
    json.dump(dict(made=time.strftime("%F %T"), results=out), open(OUT / "scale2.json", "w"), indent=1)


# ------------------------------------------------------------------------------------------------------------- vio
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


def rms(e):
    return float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))


def lowpass(t, x, fc):
    """zero-phase moving average with window 1/fc on a (possibly irregular) time grid via interpolation."""
    tg = np.arange(t[0], t[-1], 1 / 240)
    xg = np.stack([np.interp(tg, t, x[:, j]) for j in range(3)], 1)
    n = max(1, int(round(240 / fc)))
    k = np.ones(n) / n
    yg = np.stack([np.convolve(np.pad(xg[:, j], n // 2, mode="edge"), k, mode="same")[n // 2:n // 2 + len(tg)]
                   for j in range(3)], 1)
    return np.stack([np.interp(t, tg, yg[:, j]) for j in range(3)], 1)


def match(rec, runj, poses, gt_dir):
    g = dict(np.load(rec / gt_dir / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p, gt_R = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]
    tr = [m for m in poses if m.get("status") == "tracking" and m.get("T_world_cam0")]
    dt_ns = int(round(runj["align_dt_ms"] * 1e6))
    vt = np.array([m["t_ns"] for m in tr]) + dt_ns
    T = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4) for m in tr])
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    m = np.abs(gt_t[j] - vt) < 2_000_000
    return dict(t=(vt[m] - vt[m][0]) / 1e9, P=T[m, :3, 3], Pg=gt_p[j[m]], Rg=gt_R[j[m]], Rv=T[m, :3, :3],
                gt_t=gt_t, gt_p=gt_p, vt=vt[m])


def decompose(M, s_imu=None):
    P, Pg, t = M["P"], M["Pg"], M["t"]
    R, _, tt = umeyama(P, Pg)
    e = P @ R.T + tt - Pg
    Rs, c, ts = umeyama(P, Pg, scale=True)
    es = c * P @ Rs.T + ts - Pg
    o = dict(n=len(P), dur_s=round(float(t[-1] - t[0]), 1), ate_se3=round(rms(e) * 100, 3), ate_sim3=round(rms(es) * 100, 3),
             vio_scale=round(float(c), 4), gt_extent_rms_cm=round(rms(Pg - Pg.mean(0)) * 100, 2))
    # error split by axis of the board frame (z = board normal ~ camera depth)
    o["se3_axis_rms_cm"] = [round(float(np.sqrt(np.mean(e[:, k] ** 2))) * 100, 3) for k in range(3)]
    # GT corrected by the IMU scale check (GT x s_imu), SE3
    if s_imu:
        R2, _, t2 = umeyama(P, Pg * s_imu)
        o["ate_se3_gt_x_simu"] = round(rms(P @ R2.T + t2 - Pg * s_imu) * 100, 3)
    # Sim3 + per-board-axis scale (affine diag): Pg ~ D * (c R P + t), linear LS for D after Sim3
    Pa = c * P @ Rs.T + ts
    Dg = np.array([np.dot(Pa[:, k] - Pa[:, k].mean(), Pg[:, k] - Pg[:, k].mean()) /
                   np.dot(Pa[:, k] - Pa[:, k].mean(), Pa[:, k] - Pa[:, k].mean()) for k in range(3)])
    ea = (Pa - Pa.mean(0)) * Dg + Pg.mean(0) - Pg
    o.update(axis_scale_after_sim3=[round(float(v), 4) for v in Dg], ate_axis_scaled=round(rms(ea) * 100, 3))
    # frequency split of the Sim3 error: < 0.2 Hz (drift), 0.2-2 Hz, > 2 Hz (jitter)
    lo = lowpass(t, es, 0.2)
    mid = lowpass(t, es, 2.0) - lo
    hi = es - lo - mid
    o["sim3_err_band_cm"] = dict(below_0p2Hz=round(rms(lo) * 100, 3), b0p2_2Hz=round(rms(mid) * 100, 3),
                                 above_2Hz=round(rms(hi) * 100, 3))
    # local: Sim3 per 10 s segment (removes slow drift / scale drift), RMS over all points
    le, lc = [], []
    for a in np.arange(t[0], t[-1] - 5, 10.0):
        w = (t >= a) & (t < a + 10)
        if w.sum() > 100:
            Rl, cl, tl = umeyama(P[w], Pg[w], scale=True)
            le.append(c * 0 + (cl * P[w] @ Rl.T + tl - Pg[w]))
            lc.append(cl)
    if le:
        o["ate_local10s_sim3"] = round(rms(np.concatenate(le)) * 100, 3)
        o["local_scale_p10_p90"] = [round(float(np.percentile(lc, 10)), 4), round(float(np.percentile(lc, 90)), 4)]
    # time offset scan: shift VIO time vs GT (interpolated GT), +-10 ms
    best = None
    for d in np.arange(-10, 10.5, 1.0):
        tq = M["vt"] + int(d * 1e6)
        Pq = np.stack([np.interp(tq, M["gt_t"], M["gt_p"][:, k]) for k in range(3)], 1)
        Rq, cq, tqq = umeyama(P, Pq, scale=True)
        v = rms(cq * P @ Rq.T + tqq - Pq)
        if best is None or v < best[1]:
            best = (float(d), v)
    o["time_shift_best_ms"] = best[0]
    o["ate_sim3_best_shift"] = round(best[1] * 100, 3)
    # GT high-frequency jitter seen in GT alone (GT minus its own 2 Hz low-pass), same samples
    gl = lowpass(t, Pg, 8.0)
    o["gt_minus_8Hz_lowpass_cm"] = round(rms(Pg - gl) * 100, 3)
    return o, (R, tt, c, Rs, ts)


def cmd_vio():
    sc = {}
    for f in sorted((SCAN3D / "runs" / "overnight_gt").glob("*_scale_spl.json")):
        d = json.load(open(f))[0]
        if d.get("s"):
            sc[d["rec"]] = d["s"]
    rows = []
    for d in ("i3_confirm", "focus_vio"):
        for jf in sorted((SCAN3D / "runs" / d).glob("*_o?.json")):
            j = json.load(open(jf))
            if j.get("error") or not j.get("ate_cm"):
                continue
            pf = jf.with_suffix("").with_suffix(".poses.jsonl.gz")
            pf = jf.parent / (jf.stem + ".poses.jsonl.gz")
            if not pf.exists():
                continue
            poses = [json.loads(l) for l in gzip.open(pf, "rt") if l.strip()]
            rec = SCAN3D / "recordings" / j["rec"]
            M = match(rec, j, poses, "imu_gt_spl")
            o, _ = decompose(M, sc.get(j["rec"]))
            o.update(run=f"{d}/{jf.stem}", rec=j["rec"], bench_ate=j["ate_cm"], bench_sim3=j["ate_sim3_cm"],
                     bench_scale=j["scale"], s_imu=sc.get(j["rec"]))
            # same VIO vs the other camera-only GT variant (per-frame RS PnP) -> GT method sensitivity
            if (rec / "imu_gt_rs" / "gt.npz").exists():
                M2 = match(rec, j, poses, "imu_gt_rs")
                o2, _ = decompose(M2, None)
                o.update(ate_se3_vs_rs=o2["ate_se3"], ate_sim3_vs_rs=o2["ate_sim3"])
                # GT-vs-GT difference on common times (spl vs rs), after nothing (same board frame)
                gs = np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)
                gr = np.load(rec / "imu_gt_rs" / "gt.npz", allow_pickle=True)
                ts_, ok_s = gs["t_ns"], gs["ok"].astype(bool)
                tr_, ok_r = gr["t_ns"], gr["ok"].astype(bool)
                common, ia, ib = np.intersect1d(ts_[ok_s], tr_[ok_r], return_indices=True)
                if len(common) > 100:
                    dd = gs["p_bc"][ok_s][ia] - gr["p_bc"][ok_r][ib]
                    o["gt_spl_minus_rs_rms_cm"] = round(rms(dd - dd.mean(0)) * 100, 3)
            rows.append(o)
            print(json.dumps(o), flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    json.dump(dict(made=time.strftime("%F %T"), scale_check=sc, runs=rows), open(OUT / "vio.json", "w"), indent=1)


def pose_feats(M, sq_mm):
    """pose-only regressors (repeatable GT errors depend on where the board is in the image, not on time)"""
    Pg, Rg = M["Pg"], M["Rg"]
    c = np.array([4.5, 2.5, 0.0]) * sq_mm / 1000                    # board centre (9 x 5 squares)
    d = np.einsum("nji,nj->ni", Rg, c - Pg)                       # board centre in camera frame (R_bc^T (c - p))
    u, v = d[:, 0] / d[:, 2], d[:, 1] / d[:, 2]
    ax, ay = Rg[:, 0, 2], Rg[:, 1, 2]                              # optical axis in board frame (tilt)
    dist = np.linalg.norm(d, axis=1)
    F = np.c_[np.ones(len(u)), dist, ax, ay, u, v, u * u, v * v, u * v, dist * u, dist * v]
    return F


def r2_of(F, e, tr, te):
    beta = np.linalg.lstsq(F[tr], e[tr], rcond=None)[0]
    r = e[te] - F[te] @ beta
    return 1 - (r ** 2).sum() / ((e[te] - e[te].mean(0)) ** 2).sum(), float(np.sqrt((r ** 2).sum(1).mean()))


def cmd_regress():
    """Is the low-frequency Sim3 error a function of pose (repeatable GT error) or of time (VIO drift)?
    Train on alternating 10 s blocks, test on the others (pose model generalises only if the error is repeatable)."""
    rows = []
    for d, stem in [("i3_confirm", f"{r}_{v}_o{o}") for r in ("213341", "215348") for v in ("base", "i3") for o in range(3)] +                    [("focus_vio", f"111836_base_o{o}") for o in range(3)] + [("focus_vio", "111605_base_o0"), ("focus_vio", "111605_base_o1")]:
        jf = SCAN3D / "runs" / d / f"{stem}.json"
        j = json.load(open(jf))
        poses = [json.loads(l) for l in gzip.open(jf.parent / f"{stem}.poses.jsonl.gz", "rt") if l.strip()]
        rec = SCAN3D / "recordings" / j["rec"]
        sq = float(np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True)["square_mm"])
        M = match(rec, j, poses, "imu_gt_spl")
        P, Pg, t = M["P"], M["Pg"], M["t"]
        Rs, c, ts = umeyama(P, Pg, scale=True)
        e = c * P @ Rs.T + ts - Pg
        F = pose_feats(M, sq)
        knots = np.arange(t[0], t[-1] + 10, 10.0)
        T = np.stack([np.clip(1 - np.abs(t - k) / 10.0, 0, None) for k in knots], 1)     # hat functions, 10 s
        blk = (np.floor((t - t[0]) / 10).astype(int) % 2 == 0)
        o = dict(run=f"{d}/{stem}", ate_sim3=round(rms(e) * 100, 3))
        for name, X in (("pose", F), ("time10s", T)):
            r_in = r2_of(X, e, np.ones(len(t), bool), np.ones(len(t), bool))
            o[f"{name}_r2_in"] = round(float(r_in[0]), 3)
            o[f"{name}_left_cm_in"] = round(r_in[1] * 100, 3)
        a = r2_of(F, e, blk, ~blk)
        b = r2_of(F, e, ~blk, blk)
        o["pose_r2_crossblock"] = round(float((a[0] + b[0]) / 2), 3)
        r_j = r2_of(np.c_[F, T], e, np.ones(len(t), bool), np.ones(len(t), bool))
        o["pose+time_left_cm"] = round(r_j[1] * 100, 3)
        rows.append(o)
        print(json.dumps(o), flush=True)
    json.dump(dict(made=time.strftime("%F %T"), runs=rows), open(OUT / "regress.json", "w"), indent=1)


if __name__ == "__main__":
    {"still": cmd_still, "scale": cmd_scale, "scale2": cmd_scale2, "regress": cmd_regress, "vio": cmd_vio}[sys.argv[1]]()
