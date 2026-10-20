#!/usr/bin/env python3
"""Check that the learned physical IMU calibration is applied the right way round (2026-10-20, calib_vio_check).

Part A (synthetic): train_imu.imu_on_grid must put the IMU on the step midpoint. Known smooth rotation + acceleration,
  irregular 500 Hz samples; integrate the way train_imu.integrate_all does (R_{k+1} = R_k exp(w_k DT)) with the old
  (left, grid[k]) and new (midpoint) sampling, compare with the exact trajectory and fit the effective time lag.
Part B (real data): from the spline GT (imu_gt_spl, camera only) compute the true angular velocity and specific force in
  IMU axes (R_bi = R_bc R_ci, IMU position = camera position - R_bi r, as in train_imu.window_loss) and compare with the
  recorded IMU corrected in several ways. The application used by vio_bench_hs.py (:184-186, g @ G.T + bg,
  a @ A.T + ba, p_CinI = r) must give the smallest residual; inverse / sign-flipped / transposed variants must be worse.
  The calibration is the held-out fold one (runs/on_spl_<rec>/result.json, trained without <rec>).

  ../venv_ffs/bin/python imu_learn/calib_apply_test.py [--recs hs_20261019_145816 ...] [--out runs/vio_calibcheck/apply_test.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
import train_imu as T  # noqa: E402
from imu_gt import read_imu  # noqa: E402


# ------------------------------------------------------------------ part A
def synth():
    rng = np.random.default_rng(0)
    DT = T.DT
    f = np.array([0.7, 1.3, 2.1])
    amp = np.array([2.0, 1.5, 2.5])                       # rad/s, hand shake level

    def w(t):
        return amp * np.sin(2 * np.pi * f * t[:, None])

    # exact orientation by fine integration (100 kHz, midpoint rule), stored every 0.1 ms
    h = 1e-5
    tf = np.arange(0, 6.0, h)
    E = T.so3_exp_batch(w(tf + h / 2) * h)
    Rcur = np.eye(3)
    ex_t, ex_R = [0.0], [Rcur]
    for i in range(len(tf)):
        Rcur = Rcur @ E[i]
        if (i + 1) % 10 == 0:
            ex_t.append((i + 1) * h)
            ex_R.append(Rcur)
    ex_t = np.array(ex_t)
    from scipy.spatial.transform import Slerp
    slerp = Slerp(ex_t, Rot.from_matrix(np.array(ex_R)))
    # IMU samples: 499 Hz with 0.1 ms jitter, on IMU clock = camera clock - dt (dt = 0 here)
    ti = np.cumsum(np.full(3000, 1 / 499.0) + rng.normal(0, 1e-4, 3000))
    ti = ti[ti < 5.9]
    gyr = np.c_[(ti * 1e9).astype(np.int64), w(ti)]
    acc = gyr.copy()
    grid = np.arange(int(0.5e9), int(5.0e9), int(1e9 * DT))
    out = {}
    for name, shift in (("left_old", 0.0), ("midpoint_new", None)):
        if shift is None:
            g, _ = T.imu_on_grid(acc, gyr, 0, grid)
        else:
            g = np.stack([np.interp(grid, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1)
        R = [slerp(grid[0] / 1e9)]
        for k in range(len(grid) - 1):
            R.append(R[-1] * Rot.from_rotvec(g[k] * DT))
        R = Rot.concatenate(R)
        t = grid / 1e9
        err = (slerp(t).inv() * R).magnitude()
        # effective lag: shift the exact trajectory in time, find the best match of the RELATIVE rotation over 0.2 s
        lags = np.arange(-3e-3, 3.01e-3, 1e-4)
        k = int(0.2 / DT)
        sc = []
        for L in lags:
            tt = np.clip(t - L, ex_t[0], ex_t[-1])
            Rx = slerp(tt)
            rel_e = (Rx[:-k].inv() * Rx[k:])
            rel_i = (R[:-k].inv() * R[k:])
            sc.append(np.mean((rel_e.inv() * rel_i).magnitude() ** 2))
        out[name] = dict(rot_err_deg_rms_4s=round(float(np.degrees(np.sqrt(np.mean(err ** 2)))), 4),
                         lag_ms=round(float(lags[int(np.argmin(sc))] * 1e3), 2))
    return out


# ------------------------------------------------------------------ part B
def gt_motion(rec, gt_dir="imu_gt_spl"):
    g = np.load(rec / gt_dir / "gt.npz", allow_pickle=True)
    al = json.load(open(rec / "imu_gt" / "align.json"))
    R_ci = np.asarray(al["R_ci"])
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < T.MAX_DIST_M)
    t = g["t_ns"][ok]
    k = np.r_[True, np.diff(t) > 10_000]
    t, Rbc, p = t[k], g["R_bc"][ok][k], g["p_bc"][ok][k]
    return t, Rbc @ R_ci, p, al["dt_ns"]


def chunks(t, max_gap_ns=12_000_000, min_len_s=1.0):
    br = np.r_[0, np.nonzero(np.diff(t) > max_gap_ns)[0] + 1, len(t)]
    return [(a, b) for a, b in zip(br[:-1], br[1:]) if (t[b - 1] - t[a]) / 1e9 > min_len_s]


def box(x, n):
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, j], k, mode="same") for j in range(x.shape[1])], 1)


def eval_rec(rec, cal, rate=1000, lp_s=0.04):
    t, Rbi, pc, dt = gt_motion(rec)
    acc, gyr = read_imu(rec / "imu.csv")
    G, bg, A, ba, r = (np.asarray(cal[k]) for k in ("G", "bg", "A", "ba", "r"))
    nlp = int(lp_s * rate)
    W, F_raw, Ws, Rs, PDD = [], [], [], [], {}
    rows = []
    for a, b in chunks(t):
        tt = t[a:b] / 1e9
        q = Rot.from_matrix(Rbi[a:b])
        # angular velocity in IMU axes at frame midpoints, then interpolated
        rv = (q[:-1].inv() * q[1:]).as_rotvec() / np.diff(tt)[:, None]
        tm = 0.5 * (tt[1:] + tt[:-1])
        te = np.arange(tt[0] + 0.1, tt[-1] - 0.1, 1 / rate)
        if len(te) < 3 * nlp:
            continue
        w_gt = np.stack([np.interp(te, tm, rv[:, j]) for j in range(3)], 1)
        from scipy.spatial.transform import Slerp
        Re = Slerp(tt, q)(te)
        cs_p = CubicSpline(tt, pc[a:b])
        pdd_c = cs_p(te, 2)
        # IMU position = camera - R r -> its acceleration = pdd_c - d2/dt2(R r): R r splined per component
        Rr = np.einsum("nij,j->ni", Rbi[a:b], r)
        pdd_lever = CubicSpline(tt, Rr)(te, 2)
        ti = (te * 1e9).astype(np.int64)
        g_m = np.stack([np.interp(ti, gyr[:, 0] + dt, gyr[:, 1 + j]) for j in range(3)], 1)
        a_m = np.stack([np.interp(ti, acc[:, 0] + dt, acc[:, 1 + j]) for j in range(3)], 1)
        sl = slice(nlp, -nlp)
        rows.append(dict(w_gt=box(w_gt, nlp)[sl], R=Re.as_matrix()[sl], pdd0=box(pdd_c, nlp)[sl],
                         pddL=box(pdd_lever, nlp)[sl], g_m=box(g_m, nlp)[sl], a_m=box(a_m, nlp)[sl]))
    cat = {k: np.concatenate([r_[k] for r_ in rows]) for k in rows[0]}
    Gi, Ai = np.linalg.inv(G), np.linalg.inv(A)
    gyro_vars = {
        "raw": cat["g_m"],
        "bench (G x + bg)": cat["g_m"] @ G.T + bg,
        "bias only (x + bg)": cat["g_m"] + bg,
        "bias sign flipped (G x - bg)": cat["g_m"] @ G.T - bg,
        "inverse (G^-1 (x - bg))": (cat["g_m"] - bg) @ Gi.T,
        "transposed (G^T x + bg)": cat["g_m"] @ G + bg,
    }
    acc_vars = {
        "raw": cat["a_m"],
        "bench (A x + ba)": cat["a_m"] @ A.T + ba,
        "bias only (x + ba)": cat["a_m"] + ba,
        "bias sign flipped (A x - ba)": cat["a_m"] @ A.T - ba,
        "inverse (A^-1 (x - ba))": (cat["a_m"] - ba) @ Ai.T,
        "transposed (A^T x + ba)": cat["a_m"] @ A + ba,
    }
    res = dict(rec=rec.name, n=len(cat["w_gt"]), lp_s=lp_s, gyro={}, acc={})
    for k, v in gyro_vars.items():
        e = v - cat["w_gt"]
        res["gyro"][k] = dict(rms_mrad_s=round(float(np.sqrt(np.mean(e ** 2))) * 1e3, 3),
                              mean_mrad_s=np.round(e.mean(0) * 1e3, 3).tolist())
    # direct least-squares fit  true ≈ M · measured + b  (noise is in the GT side, so M is unbiased): which of
    # G, G^-1, G^T does the data's M look like? (distances in units of 1e-3, off-identity part only)
    def lsq(x, y):
        X = np.c_[x, np.ones(len(x))]
        sol = np.linalg.lstsq(X, y, rcond=None)[0]
        return sol[:3].T, sol[3]

    def dist(M, C):
        return round(float(np.linalg.norm(M - C)) * 1e3, 2)
    Mg, bg_fit = lsq(cat["g_m"], cat["w_gt"])
    res["gyro_lsq"] = dict(M=np.round(Mg, 5).tolist(), b=np.round(bg_fit, 5).tolist(),
                           dist_e3={"I (no calib)": dist(Mg, np.eye(3)), "G (bench)": dist(Mg, G), "G^-1": dist(Mg, Gi),
                                    "G^T": dist(Mg, G.T)})
    pdd = cat["pdd0"] - cat["pddL"]                                   # lever +r
    for it in range(3):                                                # alternate: g_b <-> M, b in IMU axes
        Rf = np.einsum("nij,nj->ni", cat["R"], cat["a_m"] @ (Ma.T if it else np.eye(3)) + (ba_fit if it else 0))
        gb = (pdd - Rf).mean(0)
        f_gt = np.einsum("nji,nj->ni", cat["R"], pdd - gb)
        Ma, ba_fit = lsq(cat["a_m"], f_gt)
    res["acc_lsq"] = dict(M=np.round(Ma, 5).tolist(), b=np.round(ba_fit, 4).tolist(),
                          dist_e3={"I (no calib)": dist(Ma, np.eye(3)), "A (bench)": dist(Ma, A), "A^-1": dist(Ma, Ai),
                                   "A^T": dist(Ma, A.T)})
    for lever_name, sgn in (("lever 0", 0.0), ("lever +r (bench)", 1.0), ("lever -r", -1.0)):
        pdd = cat["pdd0"] - sgn * cat["pddL"]
        for k, v in acc_vars.items():
            # world-frame model: pdd = R f + g_b ; g_b fitted (3 values) per variant
            Rf = np.einsum("nij,nj->ni", cat["R"], v)
            gb = (pdd - Rf).mean(0)
            e_w = pdd - Rf - gb
            e = np.einsum("nji,nj->ni", cat["R"], e_w)            # back in IMU axes
            res["acc"][f"{lever_name} | {k}"] = dict(rms_mm_s2=round(float(np.sqrt(np.mean(e ** 2))) * 1e3, 2),
                                                     g_norm=round(float(np.linalg.norm(gb)), 4))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recs", nargs="+", default=["hs_20261019_131326", "hs_20261019_145816", "hs_20261019_154320",
                                                     "hs_20261019_161035"])
    ap.add_argument("--calib-tag", default="spl", help="runs/on_<tag>_<rec>/result.json (held-out fold)")
    ap.add_argument("--out", default=str(SCAN3D / "runs" / "vio_calibcheck" / "apply_test.json"))
    a = ap.parse_args()
    out = dict(synthetic=synth())
    print("synthetic integration timing:", out["synthetic"], flush=True)
    out["real"] = []
    for n in a.recs:
        cf = SCAN3D / "runs" / f"on_{a.calib_tag}_{n}" / "result.json"
        cal = json.load(open(cf))["calib"]
        r = eval_rec(SCAN3D / "recordings" / n, cal)
        r["calib_file"] = str(cf.relative_to(SCAN3D))
        out["real"].append(r)
        print(n, "n", r["n"])
        for k, v in r["gyro"].items():
            print(f"  gyro {k:32s} rms {v['rms_mrad_s']:8.3f} mrad/s  mean {v['mean_mrad_s']}")
        for k, v in r["acc"].items():
            if "bench" in k or "raw" in k:
                print(f"  acc  {k:50s} rms {v['rms_mm_s2']:8.2f} mm/s2  |g| {v['g_norm']}")
        print("  gyro LSQ M vs:", r["gyro_lsq"]["dist_e3"], "b", r["gyro_lsq"]["b"], "(calib bg", np.round(cal["bg"], 5).tolist(), ")")
        print("  acc  LSQ M vs:", r["acc_lsq"]["dist_e3"], "b", r["acc_lsq"]["b"], "(calib ba", np.round(cal["ba"], 4).tolist(), ")")
        sys.stdout.flush()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(out, open(a.out, "w"), indent=1, ensure_ascii=False)
    print("->", a.out)


if __name__ == "__main__":
    main()
