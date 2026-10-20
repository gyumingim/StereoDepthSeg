"""Does the S24 IMU (LSM6DSV) bias depend on temperature HISTORY (low-pass / lag) rather than only current temperature?

usage (WSL, scan3d/):  ../venv_ffs/bin/python tools/imu_shell/temp_history_bias.py [recordings/static_imu_X]
reads the static heat/cool recording, writes runs/temp_history_bias.json, docs/temp_history_bias.html,
appends runs/temp_history_progress.log.  Reuses loaders of analyze_static.py.
"""
import json
import math
import os
import sys
import time
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.signal import lfilter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_static as AS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CUTS = [[1959, 1996], [2035, 2038.5], [2479.5, 2482]]
SRCS = ["battery", "AP", "BAT", "CP", "PA", "SKIN", "USB"]
TAUS = [0, 5, 10, 15, 20, 30, 45, 60, 90, 120, 150, 180, 240, 300, 360, 420, 480, 540, 600]
LAGS = list(range(-120, 121, 10))       # +L = lead: sensor follows T(t+L) (reacts before the source); -L = lag
CACHE = Path(os.environ.get("TH_CACHE", "/tmp/temp_history_1s.npz"))
SENS = [("gyro", "x"), ("gyro", "y"), ("gyro", "z"), ("accel", "x"), ("accel", "y"), ("accel", "z")]
UNIT = {"gyro": "mdeg/s", "accel": "mg"}


def log(msg):
    with open(ROOT / "runs/temp_history_progress.log", "a") as f:
        f.write(time.strftime("%F %T ") + msg + "\n")
    print(msg, flush=True)


# ------------------------------------------------------------------------------------------------- data
def load(rec):
    if CACHE.exists():
        z = np.load(CACHE, allow_pickle=True)
        return {k: z[k] for k in z.files}
    d = AS.read_merged(rec)
    ts0 = min(d[k][0, 0] for k in d)
    T = (max(d[k][-1, 0] for k in d) - ts0) * 1e-9
    out = {"T": np.array(T)}
    for nm, typ, sc in (("gyro", 16, AS.R2D * 1e3), ("accel", 35, 1e3 / 9.80665)):
        a = d[typ]
        t = (a[:, 0] - ts0) * 1e-9
        idx = t.astype(int)
        nb = int(T) + 1
        cnt = np.bincount(idx, minlength=nb).astype(float)
        m = np.stack([np.bincount(idx, weights=a[:, 1 + j] * sc, minlength=nb) for j in range(3)], 1)
        with np.errstate(all="ignore"):
            out[nm] = m / cnt[:, None]
        out[nm + "_n"] = cnt
    tp = AS.read_temps(rec, ts0)
    for k in SRCS + ["t", "cam_on"]:
        out["tp_" + k] = tp[k]
    out["cams"] = np.array(AS.cam_intervals(tp, pad=0.0))
    np.savez(CACHE, **out)
    return out


def temp_1s(tt, vv, n):
    """1 s series of a temperature source: linear interpolation of the ~14 s samples (edge-held)."""
    ok = np.isfinite(vv)
    return np.interp(np.arange(n) + 0.5, tt[ok], vv[ok])


def lowpass(x, tau):
    if tau <= 0:
        return x.copy()
    a = math.exp(-1.0 / tau)
    y, _ = lfilter([1 - a], [1, -a], x, zi=[a * x[0]])
    return y


def shift(x, L):
    """value at t+L (edge held). L>0 lead."""
    n = len(x)
    i = np.clip(np.arange(n) + L, 0, n - 1)
    return x[i]


# ------------------------------------------------------------------------------------------------- fitting
def ols(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    return beta


def rms(e):
    return float(np.sqrt(np.mean(e ** 2)))


class Blocks:
    def __init__(self, D, blk):
        n = len(D["gyro"])
        self.n1 = n
        self.blk = blk
        tg = np.arange(n) + 0.5
        bad = np.zeros(n, bool)
        for a, b in CUTS:
            bad |= (tg >= a - 2) & (tg < b + 2)
        nb = n // blk
        self.t = (np.arange(nb) + 0.5) * blk
        Y, ok = [], np.ones(nb, bool)
        for nm in ("gyro", "accel"):
            v = D[nm][:nb * blk].copy()
            c = D[nm + "_n"][:nb * blk].copy()
            v[bad[:nb * blk]] = np.nan
            c[bad[:nb * blk]] = 0
            w = np.where(np.isfinite(v[:, 0]), c, 0.0)
            vv = np.nan_to_num(v)
            W = w.reshape(nb, blk)
            S = (vv * w[:, None]).reshape(nb, blk, 3).sum(1)
            Wsum = W.sum(1)
            cov = (W > 0).mean(1)
            ok &= cov >= 0.6
            with np.errstate(all="ignore"):
                Y.append(S / Wsum[:, None])
        self.Y = np.concatenate(Y, 1)             # nb x 6  (gyro xyz mdeg/s, accel xyz mg)
        self.ok = ok & np.all(np.isfinite(self.Y), 1)
        cams = D["cams"]
        self.cam = np.zeros(nb, bool)
        for a, b in cams:
            self.cam |= (self.t > a) & (self.t < b)
        # phases
        h1a, h1b = cams[0]; h2a, h2b = cams[1]
        self.phase = np.full(nb, "room", dtype=object)
        self.phase[(self.t >= h1a) & (self.t < h1b)] = "heat1"
        self.phase[(self.t >= h1b) & (self.t < h2a)] = "cool1"
        self.phase[(self.t >= h2a) & (self.t < h2b)] = "heat2"
        self.t_mid = h2b + (self.t[self.ok].max() - h2b) / 2
        self.phase[(self.t >= h2b) & (self.t < self.t_mid)] = "cool2a"
        self.phase[self.t >= self.t_mid] = "cool2b"
        self.heating = np.isin(self.phase, ["heat1", "heat2"])
        self.cooling = np.isin(self.phase, ["cool1", "cool2a", "cool2b"])

    def avg(self, x1s):
        nb = len(self.t)
        return x1s[:nb * self.blk].reshape(nb, self.blk).mean(1)


def build_features(D, B):
    """F[src][(tau, L)] = block-mean of shifted+low-passed 1 s temperature; also dT/dt (block, K/min)."""
    n = B.n1
    raw = {s: temp_1s(D["tp_t"], D["tp_" + s], n) for s in SRCS}
    F, dT = {}, {}
    for s in SRCS:
        F[s] = {}
        for L in LAGS:
            xs = shift(raw[s], L)
            for tau in TAUS:
                F[s][(tau, L)] = B.avg(lowpass(xs, tau))
        sm = lowpass(raw[s][::-1], 30)[::-1]
        sm = lowpass(sm, 30)                       # zero-phase 30 s smoothing, then slope
        dT[s] = B.avg(np.gradient(sm) * 60.0)
    return raw, F, dT


def design(model, F, dT, p):
    """returns block design matrix (nb x k) for model with hyper-params p."""
    if model == "M1":
        x = F[p["src"]][(0, 0)]
        return np.c_[np.ones_like(x), x]
    if model == "M2":
        x = F[p["src"]][(p["tau"], p["lag"])]
        return np.c_[np.ones_like(x), x]
    if model == "M3":
        x = F[p["src"]][(0, 0)]
        return np.c_[np.ones_like(x), x, dT[p["src"]]]
    if model == "M4":
        x = F[p["src"]][(0, 0)]
        xc = x - 30.0
        return np.c_[np.ones_like(x), xc, xc ** 2]
    if model == "M5":
        x1 = F[p["s1"]][(p.get("tau1", 0), 0)]
        x2 = F[p["s2"]][(p.get("tau2", 0), 0)]
        return np.c_[np.ones_like(x1), x1, x2]
    raise ValueError(model)


def candidates(model, srcs=SRCS):
    if model in ("M1", "M3", "M4"):
        return [{"src": s} for s in srcs]
    if model == "M2":
        return [{"src": s, "tau": t, "lag": L} for s in srcs for t in TAUS for L in LAGS]
    if model == "M5":
        return [{"s1": a, "s2": b} for a, b in combinations(srcs, 2)]
    if model == "M5f":   # two sources, each low-passed (no lag)
        T5 = [0, 15, 30, 60, 120, 240, 480]
        return [{"s1": a, "s2": b, "tau1": t1, "tau2": t2} for a, b in combinations(srcs, 2) for t1 in T5 for t2 in T5]


def select_fit(model, F, dT, y, tr, cands):
    """hyper-params chosen by train RMS only (honest); returns (p, beta, train_rms)."""
    best = None
    for p in cands:
        X = design("M5" if model == "M5f" else model, F, dT, p)
        b = ols(X[tr], y[tr])
        r = rms(y[tr] - X[tr] @ b)
        if best is None or r < best[2]:
            best = (p, b, r)
    return best


def evaluate(F, dT, B, ycol, tr, te, cand_cache, per_src=False):
    y = B.Y[:, ycol]
    res = {}
    for model in ("M0", "M1", "M2", "M3", "M4", "M5", "M5f"):
        if model == "M0":
            m = y[tr].mean()
            res[model] = {"test_rms": rms(y[te] - m), "train_rms": rms(y[tr] - m)}
            continue
        p, b, rtr = select_fit(model, F, dT, y, tr, cand_cache[model])
        X = design("M5" if model == "M5f" else model, F, dT, p)
        res[model] = {"p": p, "beta": b.tolist(), "train_rms": rtr, "test_rms": rms(y[te] - X[te] @ b)}
    if per_src:
        res["per_src"] = {}
        for s in SRCS:
            r = {}
            for model in ("M1", "M2", "M3", "M4"):
                p, b, rtr = select_fit(model, F, dT, y, tr, candidates(model, [s]))
                X = design(model, F, dT, p)
                r[model] = {"p": p, "train_rms": rtr, "test_rms": rms(y[te] - X[te] @ b), "beta": b.tolist()}
            res["per_src"][s] = r
    return res


def hysteresis(y, x, B, ok):
    """fit line on all ok blocks; hysteresis = mean residual (cooling) - mean residual (heating); also overall RMS."""
    X = np.c_[np.ones(ok.sum()), x[ok]]
    b = ols(X, y[ok])
    e = y - (b[0] + b[1] * x)
    h, c = ok & B.heating, ok & B.cooling
    return {"slope": float(b[1]), "rms_all": rms(e[ok]),
            "cool_minus_heat": float(e[c].mean() - e[h].mean()), "rms_heat": rms(e[h]), "rms_cool": rms(e[c])}


# ------------------------------------------------------------------------------------------------- main
def run(blk, D, outj):
    B = Blocks(D, blk)
    raw, F, dT = build_features(D, B)
    ok = B.ok
    noise = {}
    room = ok & (B.phase == "room")
    for j, (s, ax) in enumerate(SENS):
        d = np.diff(B.Y[room, j])
        noise[f"{s}_{ax}"] = float(np.std(d) / math.sqrt(2))
    log(f"blk={blk}s nblocks ok={ok.sum()} (heat {np.sum(ok & B.heating)}, cool {np.sum(ok & B.cooling)}), t_mid={B.t_mid:.0f}s")
    cc = {m: candidates(m) for m in ("M1", "M2", "M3", "M4", "M5", "M5f")}
    splits = {"chrono": (ok & (B.t < B.t_mid), ok & (B.t >= B.t_mid))}
    phases = ["room", "heat1", "cool1", "heat2", "cool2a", "cool2b"]
    out = {"block_s": blk, "n_blocks": int(ok.sum()), "t_mid_s": float(B.t_mid), "noise_block_white_est": noise, "sensors": {}}
    for j, (s, ax) in enumerate(SENS):
        key = f"{s}_{ax}"
        r = {}
        tr, te = splits["chrono"]
        r["chrono"] = evaluate(F, dT, B, j, tr, te, cc, per_src=(blk == 10))
        # leave-one-segment-out: pooled test residuals
        loso = {}
        for ph in phases:
            te = ok & (B.phase == ph)
            tr = ok & ~te
            if te.sum() < 3:
                continue
            e = evaluate(F, dT, B, j, tr, te, cc)
            for m, v in e.items():
                loso.setdefault(m, {"per_phase": {}, "sq": 0.0, "n": 0})
                loso[m]["per_phase"][ph] = {"test_rms": v["test_rms"], "p": v.get("p")}
                loso[m]["sq"] += v["test_rms"] ** 2 * te.sum()
                loso[m]["n"] += int(te.sum())
        for m in loso:
            loso[m]["pooled_test_rms"] = math.sqrt(loso[m]["sq"] / loso[m]["n"])
            del loso[m]["sq"]
        r["loso"] = loso
        # all-data fit of M2 per source: tau / lag surface minimum
        y = B.Y[:, j]
        allfit = {}
        for src in SRCS:
            best = None
            surf = np.zeros((len(TAUS), len(LAGS)))
            for a, tau in enumerate(TAUS):
                for c, L in enumerate(LAGS):
                    x = F[src][(tau, L)]
                    X = np.c_[np.ones(ok.sum()), x[ok]]
                    b = ols(X, y[ok])
                    surf[a, c] = rms(y[ok] - X @ b)
                    if best is None or surf[a, c] < best[0]:
                        best = (surf[a, c], tau, L, b)
            r1 = surf[0, LAGS.index(0)]
            allfit[src] = {"tau": best[1], "lag": best[2], "rms": float(best[0]), "rms_M1": float(r1),
                           "slope_per_C": float(best[3][1]),
                           "tau_profile_lag0": surf[:, LAGS.index(0)].tolist(),
                           "hyst_M1": hysteresis(y, F[src][(0, 0)], B, ok),
                           "hyst_M2": hysteresis(y, F[src][(best[1], best[2])], B, ok)}
        r["allfit_M2"] = allfit
        out["sensors"][key] = r
        c = r["chrono"]; l = r["loso"]
        log(f"  {key}: chrono test " + " ".join(f"{m}={c[m]['test_rms']:.2f}" for m in ("M0", "M1", "M2", "M3", "M4", "M5", "M5f"))
            + " | LOSO " + " ".join(f"{m}={l[m]['pooled_test_rms']:.2f}" for m in l)
            + f" | M2 chrono p={c['M2']['p']}")
    return out, B, F, raw, dT


def main():
    rec = ROOT / (sys.argv[1] if len(sys.argv) > 1 else "recordings/static_imu_20261020_095113")
    log(f"start {rec.name}")
    D = load(rec)
    log(f"loaded T={float(D['T']):.0f}s, temps n={len(D['tp_t'])}, dt median={np.median(np.diff(D['tp_t'])):.1f}s")
    res = {"recording": rec.name, "created": time.strftime("%F %T"), "cuts": CUTS, "taus": TAUS, "lags": LAGS,
           "camera_intervals_s": D["cams"].tolist(),
           "temp_sample_dt_s": {"median": float(np.median(np.diff(D["tp_t"]))), "max": float(np.max(np.diff(D["tp_t"])))},
           "notes": "gyro=type16 uncal v0..v2 (mdeg/s), accel=type35 uncal v0..v2 (mg); Earth rate in intercept; "
                    "hyper-params (source, tau, lag) chosen on train only; lag>0 = lead (sensor follows T(t+L))."}
    out10, B, F, raw, dT = run(10, D, res)
    out5, *_ = run(5, D, res)
    res["blk10"] = out10
    res["blk5"] = out5
    (ROOT / "runs/temp_history_bias.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    np.savez("/tmp/temp_history_plotdata.npz", t=B.t, Y=B.Y, ok=B.ok, heating=B.heating, cooling=B.cooling, cam=B.cam,
             phase=B.phase.astype(str), **{f"F_{s}_{t}_{L}": F[s][(t, L)] for s in SRCS for t in TAUS for L in LAGS if L in (0,) or t in (0,) or True},
             **{f"raw_{s}": raw[s] for s in SRCS}, **{f"dT_{s}": dT[s] for s in SRCS})
    log("json written; run temp_history_report.py for html")


if __name__ == "__main__":
    main()
