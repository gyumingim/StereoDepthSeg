#!/usr/bin/env python3
"""Standard continuous-time (spline) ground truth — docs/spline_super_easy.html, spline_std_easy.html, spline_gt_result.html.

One smooth trajectory per recording, fitted directly to every observed board corner at the moment ITS image row was
read (rolling shutter exact), instead of one PnP pose per photo (Kalibr / Furgale 2013 style):
  position  p(t) = uniform cubic B-spline (knots every --knot s) — camera centre in the board frame
  rotation  R(t) = cumulative cubic B-spline on SO(3) (Kim 1995, Lovegrove 2013, Sommer 2020):
                   R(t) = C_i · exp(b̃1 Ω_i) · exp(b̃2 Ω_i+1) · exp(b̃3 Ω_i+2),  Ω_j = log(C_jᵀ C_j+1)
                   (C = control rotations, initialised from the per-frame GT); angular velocity / acceleration
                   analytic (Sommer et al. 2020 recursion) — no finite differences of noisy per-frame poses.
Residuals (all in units of their noise, summed, divided by the number of corners)
  corners   (proj(R(t_row), p(t_row), X_k) − x_obs) / σ_px in pixels (observations undistorted once), Huber at 2σ
  prior     weak: acceleration ~ N(0, 30 m/s²), angular acceleration ~ N(0, 300 rad/s²) at every control point
            (keeps gaps without photos well-posed; negligible where corners exist)
  --imu     + gyro  (R_ciᵀ ω_cam + b_g − ω_meas) / σ_g + accel (R_biᵀ(p̈_imu − g) + b_a − f_meas) / σ_a,
            with b_g(t), b_a(t) (linear knots every 10 s), gravity direction, lever r (camera = IMU + R_bi r, IMU axes —
            same convention as train_imu.Calib), δR_ci as extra unknowns — ONLY for calibration values / training
            targets, never for evaluation GT (the IMU would grade itself).
Frames with any corners count; output frames need ≥ 4 corners and a spline reprojection RMS ≤ 1.5 px.
Output = gt.npz in the same format as imu_gt (so train_imu --gt-dir works).
  ../venv_ffs/bin/python imu_learn/spline_gt.py recordings/hs_... [--imu] [--secs 20 --start 10] [--out-dir imu_gt_spl]
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from scipy.spatial.transform import Rotation as Rot, Slerp

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import aruco_scale  # noqa: E402
from imu_gt import read_imu  # noqa: E402

BOARD = HERE.parent / "config" / "charuco" / "board_monitor_6x6.json"
G0 = 9.80665


# ---------------------------------------------------------------- SO(3)
def hat(w):
    K = torch.zeros(*w.shape[:-1], 3, 3, device=w.device, dtype=w.dtype)
    K[..., 0, 1], K[..., 0, 2], K[..., 1, 0] = -w[..., 2], w[..., 1], w[..., 2]
    K[..., 1, 2], K[..., 2, 0], K[..., 2, 1] = -w[..., 0], -w[..., 1], w[..., 0]
    return K


def so3_exp(w):
    th2 = (w * w).sum(-1, keepdim=True)
    th = th2.clamp_min(1e-12).sqrt()
    small = th2 < 1e-8
    a = torch.where(small, 1 - th2 / 6, torch.sin(th) / th)[..., None]
    b = torch.where(small, 0.5 - th2 / 24, (1 - torch.cos(th)) / th2.clamp_min(1e-12))[..., None]
    K = hat(w)
    return torch.eye(3, device=w.device, dtype=w.dtype) + a * K + b * (K @ K)


def so3_log(R):
    v = torch.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0], R[..., 1, 0] - R[..., 0, 1]], -1) / 2
    s = v.norm(dim=-1)
    c = (R.diagonal(dim1=-2, dim2=-1).sum(-1) - 1) / 2
    th = torch.atan2(s, c)
    f = torch.where(s < 1e-6, 1 + th * th / 6, th / s.clamp_min(1e-12))
    return v * f[..., None]


# ---------------------------------------------------------------- B-splines (uniform cubic)
def bspline_w(u):
    """u in [0,1) → weights of the 4 control points and their 1st/2nd derivatives (per unit u)."""
    u2, u3 = u * u, u * u * u
    w = torch.stack([(1 - u) ** 3, 3 * u3 - 6 * u2 + 4, -3 * u3 + 3 * u2 + 3 * u + 1, u3], -1) / 6
    d1 = torch.stack([-3 * (1 - u) ** 2, 9 * u2 - 12 * u, -9 * u2 + 6 * u + 3, 3 * u2], -1) / 6
    d2 = torch.stack([6 * (1 - u), 18 * u - 12, -18 * u + 6, 6 * u], -1) / 6
    return w, d1, d2


def cum_w(u):
    """cumulative basis b̃1..3 (control differences 1..3) and its 1st/2nd derivatives per unit u."""
    u2, u3 = u * u, u * u * u
    b = torch.stack([(5 + 3 * u - 3 * u2 + u3), (1 + 3 * u + 3 * u2 - 2 * u3), u3], -1) / 6
    d1 = torch.stack([(3 - 6 * u + 3 * u2), (3 + 6 * u - 6 * u2), 3 * u2], -1) / 6
    d2 = torch.stack([(-6 + 6 * u), (6 - 12 * u), 6 * u], -1) / 6
    return b, d1, d2


class Spline:
    """time axis of both splines: segment i ∈ [t0 + i·k, t0 + (i+1)·k) uses control points i..i+3;
    control j is centred at t0 + (j−1)·k."""
    def __init__(self, t0, t1, knot, dev):
        self.t0, self.k = t0, knot
        self.n = int(np.ceil((t1 - t0) / knot)) + 3
        self.dev = dev

    def ctrl_times(self):
        return self.t0 + (np.arange(self.n) - 1) * self.k

    def index(self, t):                                  # t: tensor seconds
        x = (t - self.t0) / self.k
        i = torch.clamp(x.floor().long(), 0, self.n - 4)
        return i, (x - i).clamp(0, 1)

    def eval(self, c, t, order=0):
        i, u = self.index(t)
        w = bspline_w(u)[order] / (self.k ** order)
        idx = i[:, None] + torch.arange(4, device=self.dev)[None]
        return (w[..., None] * c[idx]).sum(1)

    def rot(self, C, t, deriv=False):
        """C (n,3,3) control rotations → R(t) (N,3,3) [, body angular velocity ω (N,3), angular acceleration α (N,3)]."""
        i, u = self.index(t)
        Om = so3_log(C[:-1].transpose(-1, -2) @ C[1:])          # (n−1,3)
        b, d1, d2 = cum_w(u)
        R = C[i]
        w = torch.zeros(len(t), 3, device=t.device, dtype=C.dtype)
        a = torch.zeros_like(w)
        for j in range(3):
            O = Om[i + j]
            A = so3_exp(b[:, j:j + 1] * O)
            R = R @ A
            if deriv:
                AT = A.transpose(-1, -2)
                w_rot = (AT @ w[..., None])[..., 0]
                a = (AT @ a[..., None])[..., 0] + d2[:, j:j + 1] * O - d1[:, j:j + 1] * torch.cross(O, w_rot, dim=-1)
                w = w_rot + d1[:, j:j + 1] * O
        if deriv:
            return R, w / self.k, a / self.k ** 2
        return R


# ---------------------------------------------------------------- data
def load(rec, gt_dir, start_s, secs, holdout=0, board_override=None):
    """holdout = n > 0: frames with frame index % n == 0 are left out of the fit (their corners go to D['ho'])."""
    rec = Path(rec)
    g = dict(np.load(rec / gt_dir / "gt.npz", allow_pickle=True))
    if "corners" not in g:
        raise SystemExit(f"{rec.name}/{gt_dir}: no per-frame corners (run imu_gt.py --rs --out-dir imu_gt_rs --keep-corners)")
    rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
    tmid = np.array([int(r["t_mid_ns"]) for r in rows])
    skew = np.array([int(r["rolling_shutter_skew_ns"]) for r in rows])
    meta = json.load(open(rec / "meta.json"))
    H = int(meta["height"])
    K, dist = g["K"], g["dist"]
    # board: --board > the board recorded in the per-frame GT (imu_gt.py, 2026-10-20+) > BOARD (monitor, older GTs)
    bpath = Path(board_override) if board_override else (Path(str(g["board_json"])) if "board_json" in g else BOARD)
    board = aruco_scale.Board(aruco_scale.load_board_spec(str(bpath)))
    T0 = tmid[0]
    lo = T0 + int(start_s * 1e9)
    trim = rec / "imu_gt" / "trim.json"
    if trim.exists():
        lo = max(lo, json.load(open(trim))["start_ns"])
    hi = (lo + int(secs * 1e9)) if secs else tmid[-1]
    fi, X, obs, tr = [], [], [], []
    n = min(len(g["corners"]), len(tmid))
    for i in range(n):
        if not (lo <= tmid[i] <= hi):
            continue
        c, ids = np.asarray(g["corners"][i], np.float64).reshape(-1, 2), np.asarray(g["corner_ids"][i], int).reshape(-1)
        if len(ids) == 0:
            continue
        nrm = cv2.undistortPoints(c.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
        fi.append(np.full(len(ids), i))
        X.append(board.obj[ids])
        obs.append(nrm)
        tr.append(tmid[i] + (c[:, 1] / H - 0.5) * skew[i])            # each corner's own row time
    fi, X, obs, tr = np.concatenate(fi), np.concatenate(X), np.concatenate(obs), np.concatenate(tr)
    t = (tr - T0) / 1e9
    ho = None
    if holdout:
        m = fi % holdout == 0
        ho = dict(fi=fi[m], X=X[m], obs=obs[m], t=t[m])
        fi, X, obs, t = fi[~m], X[~m], obs[~m], t[~m]
    # reference poses for initialisation: per-frame GT (ok, near), held-out frames excluded
    ok = g["ok"].astype(bool) & (g["t_ns"] > 0) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    ok &= (g["t_ns"] >= lo - 1e9) & (g["t_ns"] <= hi + 1e9)
    if holdout:
        ok &= np.arange(len(ok)) % holdout != 0
    tg, Rg, pg = g["t_ns"][ok], g["R_bc"][ok], g["p_bc"][ok]
    k = np.r_[True, np.diff(tg) > 10_000]
    tg, Rg, pg = tg[k], Rg[k], pg[k]
    return dict(rec=rec, T0=int(T0), lo=int(lo), hi=int(hi), K=K, f=float((K[0, 0] + K[1, 1]) / 2), fi=fi, X=X, obs=obs,
                t=t, tg=(tg - T0) / 1e9, Rg=Rg, pg=pg, tmid=tmid, skew=skew, H=H, n_frames=n, g=g, ho=ho, board_json=bpath)


def slerp_at(tg, Rg, t):
    t = np.clip(t, tg[0], tg[-1])
    return Slerp(tg, Rot.from_matrix(Rg))(t).as_matrix()


# ---------------------------------------------------------------- fit
def project(D, R, p, X, obs):
    xc = ((X - p)[:, None, :] @ R)[:, 0, :]                       # Rᵀ (X − p)
    z = xc[:, 2].clamp_min(1e-3)
    return D["f"] * (xc[:, :2] / z[:, None] - obs)


def fit(D, knot=0.05, imu=False, iters=300, dev="cuda", log=print, sig_px=0.5, sig_acc=30.0, sig_alpha=300.0,
        I=None, imu_w=1.0, sg=0.02, sa=0.1, init=None, chunk=400_000):
    t_lo, t_hi = (D["lo"] - D["T0"]) / 1e9, (D["hi"] - D["T0"]) / 1e9
    sp = Spline(t_lo - 0.02, t_hi + 0.02, knot, dev)
    f32 = lambda a: torch.tensor(np.asarray(a), dtype=torch.float32, device=dev)
    t = f32(D["t"])
    X, obs = f32(D["X"]), f32(D["obs"])
    Nc = len(t)
    if init is not None:                                          # warm start from a camera-only fit (same knots)
        cp0, C0 = init["cp"].clone(), init["C"].clone()
    else:
        # position control points: least squares of the spline to the per-frame GT positions (+ tiny ridge)
        import scipy.sparse as sps
        from scipy.sparse.linalg import spsolve
        tk = torch.tensor(D["tg"], dtype=torch.float64)
        i, u = sp.index(tk)
        Wb = bspline_w(u)[0].numpy()
        cols = (i[:, None] + torch.arange(4)[None]).numpy()
        A = sps.csr_matrix((Wb.ravel(), (np.repeat(np.arange(len(tk)), 4), cols.ravel())), shape=(len(tk), sp.n))
        p_lin = np.stack([np.interp(sp.ctrl_times(), D["tg"], D["pg"][:, j]) for j in range(3)], 1)
        lam = 1e-3                                                # sparse (banded) normal equations: dense A was n_frames × n_ctrl
        cp0 = f32(spsolve((A.T @ A + lam * sps.identity(sp.n)).tocsc(), A.T @ D["pg"] + lam * p_lin))
        C0 = f32(slerp_at(D["tg"], D["Rg"], sp.ctrl_times()))     # control rotations ← per-frame GT at control times
    cp = cp0.contiguous().clone().requires_grad_(True)
    dphi = torch.zeros(sp.n, 3, device=dev, requires_grad=True)
    params = [cp, dphi]
    extra = {}
    if imu:
        I = I or imu_data(D, dev)
        nb = I["nb"]
        extra = dict(bg=torch.zeros(nb, 3, device=dev, requires_grad=True), ba=torch.zeros(nb, 3, device=dev, requires_grad=True),
                     gdir=torch.tensor(I["g0"] / G0, dtype=torch.float32, device=dev, requires_grad=True),
                     r=torch.zeros(3, device=dev, requires_grad=True), dci=torch.zeros(3, device=dev, requires_grad=True))
        params += list(extra.values())
    ctrl = lambda: C0 @ so3_exp(dphi)

    def corner_res(sl=slice(None)):
        return project(D, sp.rot(ctrl(), t[sl]), sp.eval(cp, t[sl]), X[sl], obs[sl])

    def corner_loss(sl=slice(None)):
        e = corner_res(sl).norm(dim=1) / sig_px
        return torch.where(e < 2.0, 0.5 * e ** 2, 2.0 * e - 2.0).sum()  # Huber at 2σ

    def loss_fn(parts=False, corners=True):
        Lc = corner_loss() if corners else torch.zeros((), device=dev)
        C = ctrl()
        d2p = (cp[2:] - 2 * cp[1:-1] + cp[:-2]) / knot ** 2 / sig_acc
        Om = so3_log(C[:-1].transpose(-1, -2) @ C[1:])
        d2r = (Om[1:] - Om[:-1]) / knot ** 2 / sig_alpha
        Lp = 0.5 * (d2p ** 2).sum() + 0.5 * (d2r ** 2).sum()
        Li = imu_loss(sp, cp, C, extra, I, sg, sa) * imu_w if imu else torch.zeros((), device=dev)
        if parts:
            return Lc.item() / Nc, Lp.item() / Nc, Li.item() / Nc
        return (Lc + Lp + Li) / Nc
    opt = torch.optim.LBFGS(params, lr=1, max_iter=20, history_size=30, line_search_fn="strong_wolfe")
    t_start = time.time()
    for it in range(iters // 20):
        def closure():                                            # corners in chunks: bounded GPU memory
            opt.zero_grad()
            L = loss_fn(corners=False)
            L.backward()
            tot = L.detach()
            for c0 in range(0, Nc, chunk):
                Lk = corner_loss(slice(c0, c0 + chunk)) / Nc
                Lk.backward()
                tot = tot + Lk.detach()
            return tot
        L = opt.step(closure)
        if it % 3 == 0 or it == iters // 20 - 1:
            with torch.no_grad():
                e = corner_res().norm(dim=1)
                pc = loss_fn(parts=True)
            msg = (f"  it {(it + 1) * 20:4d}: loss {L.item():.4f} (corners {pc[0]:.4f} prior {pc[1]:.4f} imu {pc[2]:.4f})"
                   f"  corner px median {e.median().item():.3f}  p90 {e.quantile(0.9).item():.3f}")
            if imu:
                msg += f"  r {np.round(extra['r'].detach().cpu().numpy() * 100, 2).tolist()} cm"
            log(msg + f"  ({time.time() - t_start:.0f} s)")
    with torch.no_grad():
        e = corner_res().norm(dim=1).cpu().numpy()
    out = dict(sp=sp, cp=cp.detach(), C=ctrl().detach(), corner_err=e)
    if imu:
        ex = {k: v.detach().cpu().numpy() for k, v in extra.items()}
        gd = ex.pop("gdir")
        ex["g"] = gd / np.linalg.norm(gd) * G0
        out["extra"] = {k: v.tolist() for k, v in ex.items()}
        with torch.no_grad():
            out["imu_res"] = imu_loss(sp, cp, ctrl(), extra, I, sg, sa, per_sample=True)
    return out


def imu_data(D, dev, rate=200, bias_knot=10.0):
    rec = D["rec"]
    al = json.load(open(rec / "imu_gt" / "align.json"))
    acc, gyr = read_imu(rec / "imu.csv", raw=bool(al.get("imu_raw", False)))
    dt = int(al["dt_ns"])
    tg = (gyr[:, 0] + dt - D["T0"]) / 1e9                         # IMU samples on the camera clock, seconds
    lo, hi = (D["lo"] - D["T0"]) / 1e9, (D["hi"] - D["T0"]) / 1e9
    ts = np.arange(lo + 0.05, hi - 0.05, 1 / rate)
    w = np.stack([np.interp(ts, tg, gyr[:, 1 + j]) for j in range(3)], 1)
    a = np.stack([np.interp(ts, (acc[:, 0] + dt - D["T0"]) / 1e9, acc[:, 1 + j]) for j in range(3)], 1)
    # only where the board is seen (spline well determined): IMU samples within 50 ms of a fitted corner
    tc = np.unique(np.round(D["t"], 3))
    j = np.searchsorted(tc, ts).clip(1, len(tc) - 1)
    near = np.minimum(np.abs(tc[j] - ts), np.abs(tc[j - 1] - ts)) < 0.05
    ts, w, a = ts[near], w[near], a[near]
    R_ci = np.asarray(al["R_ci"])
    Rref = slerp_at(D["tg"], D["Rg"], ts)
    g0 = -(Rref @ R_ci @ a[..., None])[..., 0].mean(0)            # rough gravity in the board frame
    g0 = g0 / np.linalg.norm(g0) * G0
    nb = int(np.ceil((hi - lo) / bias_knot)) + 1
    f32 = lambda x: torch.tensor(np.asarray(x), dtype=torch.float32, device=dev)
    return dict(t=f32(ts), w=f32(w), a=f32(a), R_ci=f32(R_ci), g0=g0, nb=nb, b_lo=lo, b_knot=bias_knot)


def bias_at(B, I):
    u = ((I["t"] - I["b_lo"]) / I["b_knot"]).clamp(0, B.shape[0] - 1.000001)
    k0 = u.floor().long()
    f = (u - k0)[:, None]
    return (1 - f) * B[k0] + f * B[(k0 + 1).clamp_max(B.shape[0] - 1)]


def imu_pred(sp, cp, C, ex, I):
    """predicted gyro (rad/s) and accelerometer (m/s²) in IMU axes at the IMU sample times."""
    t = I["t"]
    R, wc, ac = sp.rot(C, t, deriv=True)                         # camera → board, ω, α in camera axes
    R_ci = I["R_ci"] @ so3_exp(ex["dci"][None])[0]
    wi = wc @ R_ci                                               # R_ciᵀ ω  (IMU axes)
    ai = ac @ R_ci
    Rbi = R @ R_ci                                               # IMU axes → board
    r = ex["r"]
    # camera = IMU + R_bi r  →  p̈_imu = p̈_cam − R_bi (α × r + ω × (ω × r))
    lev = torch.cross(ai, r.expand_as(ai), dim=-1) + torch.cross(wi, torch.cross(wi, r.expand_as(wi), dim=-1), dim=-1)
    a_imu = sp.eval(cp, t, order=2) - (Rbi @ lev[..., None])[..., 0]
    g = ex["gdir"] / ex["gdir"].norm() * G0
    f = ((a_imu - g)[:, None, :] @ Rbi)[:, 0, :]                  # Rbiᵀ (a − g)
    return wi + bias_at(ex["bg"], I), f + bias_at(ex["ba"], I)


def imu_loss(sp, cp, C, ex, I, sg=0.02, sa=0.1, per_sample=False):
    wp, fp = imu_pred(sp, cp, C, ex, I)
    if per_sample:
        return dict(gyro_rms=float(((wp - I["w"]) ** 2).sum(-1).mean().sqrt()), acc_rms=float(((fp - I["a"]) ** 2).sum(-1).mean().sqrt()))
    rg, ra = (wp - I["w"]) / sg, (fp - I["a"]) / sa
    return 0.5 * ((rg ** 2).sum() + (ra ** 2).sum())


def eval_pose(F, tt, dev="cuda"):
    """spline pose (R camera→board, p) at times tt (s since T0, numpy)."""
    sp = F["sp"]
    with torch.no_grad():
        x = torch.tensor(tt, dtype=torch.float32, device=dev)
        return sp.rot(F["C"], x).cpu().numpy().astype(np.float64), sp.eval(F["cp"], x).cpu().numpy().astype(np.float64)


def export(D, F, out_dir, min_corners=4, max_rms=1.5):
    g = D["g"]
    n = D["n_frames"]
    tm = (D["tmid"][:n] - D["T0"]) / 1e9
    inwin = (D["tmid"][:n] >= D["lo"]) & (D["tmid"][:n] <= D["hi"])
    cnt = np.bincount(D["fi"], minlength=n)
    rms = np.full(n, np.nan)
    s = np.bincount(D["fi"], weights=F["corner_err"] ** 2, minlength=n)
    m = cnt > 0
    rms[m] = np.sqrt(s[m] / cnt[m])
    R, p = eval_pose(F, tm[inwin], F["cp"].device)
    R_all = np.tile(np.eye(3), (n, 1, 1))
    p_all = np.zeros((n, 3))
    R_all[inwin], p_all[inwin] = R, p
    ok = inwin & (cnt >= min_corners) & (rms <= max_rms)
    out = dict(t_ns=D["tmid"][:n].astype(np.int64), R_bc=R_all, p_bc=p_all, rms_px=rms, n_corners=cnt, ok=ok,
               K=g["K"], dist=g["dist"], spline=True, knot=F["sp"].k)
    from imu_gt import board_info, write_board_info
    bi = board_info(D["board_json"])
    out.update(board_json=bi["board_json"], square_mm=bi["square_mm"])
    if "extra" in F:
        out["imu_params"] = json.dumps(F["extra"])
    od = D["rec"] / out_dir
    od.mkdir(exist_ok=True)
    np.savez(od / "gt.npz", **out)
    ce = np.asarray(F["corner_err"])
    write_board_info(od, D["board_json"], dict(
        rec=D["rec"].name, spline=True, knot=F["sp"].k, imu_terms="extra" in F, frames=int(n), ok=int(ok.sum()),
        corner_px_median=float(np.median(ce)), corner_px_p90=float(np.percentile(ce, 90)),
        window_s=[(D["lo"] - D["T0"]) / 1e9, (D["hi"] - D["T0"]) / 1e9]), files=[__file__])
    if "extra" in F:
        json.dump(dict(F["extra"], imu_res=F.get("imu_res"), knot=F["sp"].k), open(od / "imu_params.json", "w"), indent=1)
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--gt-dir", default="imu_gt_rs")
    ap.add_argument("--out-dir", default=None, help="default imu_gt_spl (camera only) / imu_gt_spl_imu (--imu)")
    ap.add_argument("--imu", action="store_true", help="camera-only fit first, then add the IMU terms (warm start)")
    ap.add_argument("--knot", type=float, default=0.025)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--start", type=float, default=0)
    ap.add_argument("--secs", type=float, default=0, help="0 = whole recording")
    ap.add_argument("--board", default=None, help="보드 json (기본: 정답 gt.npz 에 적힌 보드, 없으면 board_monitor_6x6.json)")
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    t0 = time.time()
    D = load(a.rec, a.gt_dir, a.start, a.secs, board_override=a.board)
    print(f"{D['rec'].name}: corners {len(D['fi'])} in {len(np.unique(D['fi']))} frames, "
          f"{(D['hi'] - D['lo']) / 1e9:.0f} s, knot {a.knot}, loaded {time.time() - t0:.0f} s", flush=True)
    log = lambda s: print(s, flush=True)
    F = fit(D, a.knot, False, a.iters, dev, log=log)
    if a.imu:
        log("  + IMU terms")
        F = fit(D, a.knot, True, a.iters, dev, log=log, init=F)
    out_dir = a.out_dir or ("imu_gt_spl_imu" if a.imu else "imu_gt_spl")
    ok = export(D, F, out_dir)
    base = D["g"]["ok"].astype(bool)
    print(f"→ {out_dir}: frames ok {ok.sum()} (per-frame GT {base.sum()}), corner px median {np.median(F['corner_err']):.3f}"
          + (f", r {np.round(np.array(F['extra']['r']) * 100, 2).tolist()} cm, imu res {F['imu_res']}" if a.imu else "")
          + f"  total {time.time() - t0:.0f} s, GPU peak {torch.cuda.max_memory_allocated() / 1e9:.1f} GB", flush=True)


if __name__ == "__main__":
    main()
