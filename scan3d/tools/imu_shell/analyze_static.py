"""Gyro/accel bias stability of the Galaxy S24 IMU (LSM6DSV) from a static IMU-only recording + existing hs recordings.

usage (WSL, scan3d/):
  ../venv_ffs/bin/python tools/imu_shell/analyze_static.py [recordings/static_imu_X] [--preview]
reads optional runs/static_imu_cuts.json  ([[t0,t1],...] seconds from start; if missing -> auto-detected movement cuts)
writes runs/gyro_bias_stability.json, docs/gyro_bias_stability.html, docs/static_imu_raw.html
"""
import base64
import glob
import io
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
D2R, R2D = math.pi / 180, 180 / math.pi
LAT_DEG = 36.0                      # assumed (Korea)
EARTH_DPS = 15.041 / 3600           # deg/s


def log(msg):
    with open(ROOT / "runs/gyro_bias_progress.log", "a") as f:
        f.write(time.strftime("%F %T ") + msg + "\n")
    print(msg, flush=True)


def read_csv(path):
    out = {}
    with open(path) as f:
        next(f)
        for ln in f:
            p = ln.rstrip().split(",")
            if len(p) < 5:
                continue
            out.setdefault(int(p[0]), []).append([float(x) for x in p[1:]])
    res = {}
    for k, v in out.items():
        n = min(len(r) for r in v)
        a = np.array([r[:n] for r in v])
        a = a[np.argsort(a[:, 0], kind="stable")]
        res[k] = a
    return res


def to_grid(t, x, dt, t0, t1):
    """bin-average samples onto a uniform grid of width dt (NaN for empty bins)."""
    nb = int((t1 - t0) / dt)
    idx = ((t - t0) / dt).astype(int)
    m = (idx >= 0) & (idx < nb)
    idx = idx[m]
    cnt = np.bincount(idx, minlength=nb).astype(float)
    out = np.full((nb, x.shape[1]), np.nan)
    for j in range(x.shape[1]):
        s = np.bincount(idx, weights=x[m, j], minlength=nb)
        with np.errstate(invalid="ignore", divide="ignore"):
            out[:, j] = s / cnt
    return out


def fill_nan(y):
    y = y.copy()
    for j in range(y.shape[1]):
        c = y[:, j]
        bad = np.isnan(c)
        if bad.any() and (~bad).any():
            c[bad] = np.interp(np.flatnonzero(bad), np.flatnonzero(~bad), c[~bad])
    return y


def adev(y, dt, taus, min_frac=0.5):
    """overlapping Allan deviation of rate series y (uniform grid dt, NaN = no sample in that bin; gyro has ~1 s gaps).
    Window averages = sum/count of available bins; a pair is used when both windows have >= min_frac of bins."""
    ok = ~np.isnan(y)
    cs = np.concatenate([[0], np.cumsum(np.where(ok, y, 0.0))])
    cn = np.concatenate([[0], np.cumsum(ok)])
    N = len(y)
    out_t, out_s, used = [], [], set()
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= N or m in used:
            continue
        used.add(m)
        s1, n1 = cs[m:-m] - cs[:-2 * m], cn[m:-m] - cn[:-2 * m]
        s2, n2 = cs[2 * m:] - cs[m:-m], cn[2 * m:] - cn[m:-m]
        v = (n1 >= min_frac * m) & (n2 >= min_frac * m) & (n1 > 0) & (n2 > 0)
        if v.sum() < 10:
            continue
        d = s2[v] / n2[v] - s1[v] / n1[v]
        out_t.append(m * dt)
        out_s.append(math.sqrt(0.5 * np.mean(d * d)))
    return np.array(out_t), np.array(out_s)


def allan_params(t, s):
    ls, lt = np.log10(s), np.log10(t)
    slope = np.gradient(ls, lt)
    # white noise N: sigma*sqrt(tau) where slope ~ -1/2
    w = (slope < -0.35) & (slope > -0.65) & (t <= 10)
    N = float(np.median(s[w] * np.sqrt(t[w]))) if w.any() else float(s[np.argmin(abs(t - 1))] * 1.0)
    i = int(np.argmin(s))
    B = float(s[i] / 0.664)
    tauB = float(t[i])
    # rate random walk K: sigma*sqrt(3/tau) where slope ~ +1/2 (after the minimum)
    r = (slope > 0.3) & (t > tauB)
    K = float(np.median(s[r] * np.sqrt(3 / t[r]))) if r.sum() >= 3 else None
    return dict(N=N, B=B, tau_B=tauB, sigma_min=float(s[i]), K=K)


def auto_cuts(tg, gy, ac, dt):
    """movement detection on 0.5 s windows: gyro std or accel std or mean-gyro jump above robust thresholds."""
    w = int(round(0.5 / dt))
    nw = len(tg) // w
    G = gy[:nw * w].reshape(nw, w, 3)
    A = ac[:nw * w].reshape(nw, w, 3)
    with np.errstate(all="ignore"):
        gs = np.nanstd(G, axis=1).max(1)
    as_ = np.nanstd(A, axis=1).max(1)
    gm = np.nanmean(G, axis=1)
    gdev = np.linalg.norm(gm - np.nanmedian(gm, axis=0), axis=1)
    am = np.nanmean(A, axis=1)
    adev_ = np.linalg.norm(am - np.nanmedian(am, axis=0), axis=1)

    def thr(x, k):
        med = np.nanmedian(x)
        mad = np.nanmedian(abs(x - med)) * 1.4826
        return med + k * max(mad, 1e-12)
    cov = np.mean(~np.isnan(G[:, :, 0]), axis=1)
    gs = np.where(cov >= 0.3, np.nan_to_num(gs), 0.0); gdev = np.where(cov >= 0.3, np.nan_to_num(gdev), 0.0)
    flags = {"gyro_std": gs > thr(gs, 10), "accel_std": as_ > thr(as_, 10), "gyro_mean": gdev > thr(gdev, 12), "accel_mean": adev_ > thr(adev_, 12)}
    auto_cuts.why = {k: (np.flatnonzero(v) * w * dt).round(1).tolist()[:20] for k, v in flags.items()}
    bad = flags["gyro_std"] | flags["accel_std"] | flags["gyro_mean"] | flags["accel_mean"]
    cuts = []
    i = 0
    while i < nw:
        if bad[i]:
            j = i
            while j + 1 < nw and bad[j + 1]:
                j += 1
            a, b = max(0, (i - 2) * w * dt), min(len(tg) * dt, (j + 3) * w * dt)
            if cuts and a <= cuts[-1][1]:
                cuts[-1][1] = b
            else:
                cuts.append([a, b])
            i = j + 1
        else:
            i += 1
    return [[round(a, 1), round(b, 1)] for a, b in cuts]


def png(fig):
    b = io.BytesIO()
    fig.savefig(b, format="png", dpi=110, bbox_inches="tight")
    plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(b.getvalue()).decode()


def quat_R(q):
    x, y, z = q[:3]
    w = q[3] if len(q) > 3 else math.sqrt(max(0, 1 - x * x - y * y - z * z))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------------------------------------- static recording
def read_merged(rec):
    """imu.csv (+ imu2.csv from the 2nd overlapping logger): per type, sorted, duplicate timestamps dropped."""
    parts = [read_csv(f) for f in (rec / "imu.csv", rec / "imu2.csv") if f.exists()]
    d = {}
    for k in set().union(*parts):
        arrs = [p[k] for p in parts if k in p]
        n = min(a.shape[1] for a in arrs)
        a = np.concatenate([x[:, :n] for x in arrs])
        a = a[np.argsort(a[:, 0], kind="stable")]
        a = a[np.concatenate([[True], np.diff(a[:, 0]) > 0])]
        d[k] = a
    return d


def _num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return np.nan


def read_temps(rec, ts0):
    """temps.csv (run_static) + temps_all.csv (run_thermal) -> dict of arrays, t = s from IMU start; cam_on."""
    rows = []
    for fn in ("temps.csv", "temps_all.csv"):
        f = rec / fn
        if not f.exists():
            continue
        lines = f.read_text().splitlines()
        cols = lines[0].split(",")
        for ln in lines[1:]:
            v = ln.split(",")
            if len(v) == len(cols):
                rows.append(dict(zip(cols, v)))
    if not rows:
        return None
    rows.sort(key=lambda r: _num(r["phone_uptime_s"]))
    out = {"t": np.array([_num(r["phone_uptime_s"]) for r in rows]) - ts0 * 1e-9}
    out["battery"] = np.array([_num(r["battery_dC"]) / 10 for r in rows])
    for k in ("AP", "BAT", "CP", "PA", "SKIN", "USB"):
        out[k] = np.array([_num(r.get(k, "")) for r in rows])
    out["cam_on"] = np.array([_num(r.get("cam_on", "0")) for r in rows])
    out["thermal_status"] = np.array([_num(r.get("thermal_status", "0")) for r in rows])
    out["pc_unix"] = np.array([_num(r["pc_unix"]) for r in rows])
    return out


def cam_intervals(tp, pad=5.0):
    """camera-on intervals from the cam_on column (sample times), padded."""
    if tp is None:
        return []
    t, c = tp["t"], np.nan_to_num(tp["cam_on"])
    iv, i = [], 0
    while i < len(t):
        if c[i] > 0:
            j = i
            while j + 1 < len(t) and c[j + 1] > 0:
                j += 1
            a = t[i - 1] if i > 0 else t[i]
            b = t[j + 1] if j + 1 < len(t) else t[j]
            iv.append([float(a - pad), float(b + pad)])
            i = j + 1
        else:
            i += 1
    return iv


def analyze_static(rec, preview=False):
    d = read_merged(rec)
    ts0 = min(d[k][0, 0] for k in d)
    meta = {}
    lo = rec / "logger_out.txt"
    wall0 = None
    if lo.exists():
        for ln in lo.read_text().splitlines():
            if ln.startswith("START"):
                p = ln.split()
                wall0 = int(p[4]) / 1000 - (int(p[2]) - ts0) * 1e-9
    gyr = d.get(16)          # uncalibrated gyro (raw, rad/s) — primary
    gyc = d.get(4)           # calibrated gyro (Android online bias removed)
    acc = d.get(35, d.get(1))
    accc = d.get(1)
    t_end = max(d[k][-1, 0] for k in d)
    T = (t_end - ts0) * 1e-9
    rates = {k: float((len(v) - 1) / ((v[-1, 0] - v[0, 0]) * 1e-9)) for k, v in d.items()}
    gaps = {k: float(np.max(np.diff(v[:, 0])) * 1e-9) for k, v in d.items()}
    cover = {}
    for k, v in d.items():
        dtt = np.diff(v[:, 0]) * 1e-9
        cover[k] = {"gaps_over_50ms": int((dtt > 0.05).sum()), "time_in_gaps_frac": float(dtt[dtt > 0.05].sum() / max(1e-9, dtt.sum()))}
    dt = 0.01
    tg = lambda a: (a[:, 0] - ts0) * 1e-9
    G = to_grid(tg(gyr), gyr[:, 1:4] * R2D, dt, 0, T)            # deg/s raw
    Gc = to_grid(tg(gyc), gyc[:, 1:4] * R2D, dt, 0, T)
    A = to_grid(tg(acc), acc[:, 1:4], dt, 0, T)
    Ac = to_grid(tg(accc), accc[:, 1:4], dt, 0, T)
    tgrid = (np.arange(len(G)) + 0.5) * dt
    android_bias = gyr[:, 4:7] * R2D if gyr.shape[1] >= 7 else None

    tp = read_temps(rec, ts0)
    cams = cam_intervals(tp)
    static_end = cams[0][0] if cams else T
    cam_mask = np.zeros(len(tgrid), bool)
    for a_, b_ in cams:
        cam_mask |= (tgrid >= a_) & (tgrid < b_ + 30)      # +30 s settle after camera off
    cuts_file = ROOT / "runs/static_imu_cuts.json"
    Gm, Am = G.copy(), A.copy()
    Gm[cam_mask] = np.nan
    Am[cam_mask] = np.nan
    auto = auto_cuts(tgrid, Gm, Am, dt)
    if cuts_file.exists():
        cuts = json.loads(cuts_file.read_text())
        cut_src = "user (runs/static_imu_cuts.json)"
    else:
        cuts = auto
        cut_src = "auto (provisional)"
    keep_all = np.ones(len(tgrid), bool)
    for a, b in cuts:
        keep_all &= ~((tgrid >= a) & (tgrid < b))
    keep = keep_all & ~cam_mask & (tgrid < static_end)     # pure static part (before heating) for Allan / drift
    # longest clean continuous segment for Allan
    edges = np.flatnonzero(np.diff(np.concatenate([[0], keep.astype(int), [0]])))
    segs = list(zip(edges[::2], edges[1::2]))
    s0, s1 = max(segs, key=lambda s: s[1] - s[0])
    log(f"static: T={T:.0f}s rates={ {k: round(v) for k, v in rates.items()} } cuts={cuts} ({cut_src}) "
        f"longest clean {tgrid[s0]:.0f}-{tgrid[s1 - 1]:.0f}s")

    taus = np.logspace(-2, math.log10(600), 60)
    allan = {}
    curves = {}
    for name, Y, unit in (("gyro_raw", G, "deg/s"), ("gyro_cal", Gc, "deg/s"), ("accel_raw", A, "m/s2"), ("accel_cal", Ac, "m/s2")):
        Yc = Y[s0:s1].copy()
        allan[name] = {}
        for j, ax in enumerate("xyz"):
            t_, s_ = adev(Yc[:, j], dt, taus)
            p = allan_params(t_, s_)
            curves[(name, ax)] = (t_, s_)
            if name.startswith("gyro"):
                p.update(ARW_deg_per_sqrt_h=p["N"] * 60, ARW_deg_s_per_sqrtHz=p["N"],
                         BI_deg_s=p["B"], BI_deg_h=p["B"] * 3600,
                         RRW_deg_s_per_sqrt_s=p["K"])
            else:
                p.update(VRW_m_s_per_sqrt_h=p["N"] * 60, VRW_ug_per_sqrtHz=p["N"] / 9.81 * 1e6,
                         BI_m_s2=p["B"], BI_mg=p["B"] / 9.81 * 1e3, RRW=p["K"])
            allan[name][ax] = p

    # orientation / Earth rate
    orient = {}
    am = np.nanmean(A[keep], axis=0)
    g_dir = am / np.linalg.norm(am)
    orient["accel_mean_m_s2"] = am.tolist()
    orient["tilt_deg_from_flat"] = float(math.degrees(math.acos(abs(g_dir[2]))))
    mag_f = rec / "mag.csv"
    earth = None
    if mag_f.exists():
        m = read_csv(mag_f)
        B = m[2][:, 1:4].mean(0)
        orient["mag_uT"] = B.tolist()
        orient["mag_norm_uT"] = float(np.linalg.norm(B))
        up = am / np.linalg.norm(am)                   # accel at rest points up (specific force)
        E = np.cross(B, up); E /= np.linalg.norm(E)
        Nn = np.cross(up, E)
        Rwd = np.vstack([E, Nn, up])                   # rows = world ENU axes in device coords
        if 11 in m:
            q = m[11][:, 1:5].mean(0); q /= np.linalg.norm(q)
            Rrv = quat_R(q)                            # device -> world (ENU)
            orient["rotvec_vs_magaccel_deg"] = float(R2D * math.acos(np.clip((np.trace(Rrv @ Rwd) - 1) / 2, -1, 1)))
        # heading of device +y (top edge) projected on horizontal
        yw = Rwd @ np.array([0, 1, 0.])
        orient["heading_top_edge_deg_from_N"] = float((R2D * math.atan2(yw[0], yw[1])) % 360)
        zw = Rwd @ np.array([0, 0, 1.])
        orient["screen_faces"] = "up" if zw[2] > 0 else "down"
        lat = LAT_DEG * D2R
        Om_w = EARTH_DPS * np.array([0, math.cos(lat), math.sin(lat)])
        earth = Rwd.T @ Om_w                            # device axes, deg/s
        orient["earth_rate_device_deg_s"] = earth.tolist()
        orient["lat_assumed_deg"] = LAT_DEG
        # sensitivity: 10 deg heading error
        c = math.cos(10 * D2R); s = math.sin(10 * D2R)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        orient["earth_rate_err_10deg_heading_deg_s"] = float(np.linalg.norm(Rwd.T @ (Rz @ Om_w) - earth))

    # 10 s blocks
    blk = int(round(10 / dt))
    nb = len(G) // blk
    bt, bm, bmc, bma, bacc = [], [], [], [], []
    for i in range(nb):
        sl = slice(i * blk, (i + 1) * blk)
        if keep[sl].mean() < 0.9:
            continue
        bt.append(tgrid[sl].mean())
        bm.append(np.nanmean(G[sl][keep[sl]], 0))
        bmc.append(np.nanmean(Gc[sl][keep[sl]], 0))
        bacc.append(np.nanmean(A[sl][keep[sl]], 0))
    bt, bm, bmc, bacc = map(np.array, (bt, bm, bmc, bacc))

    # temperatures
    temps = None if tp is None else {k: [None if not np.isfinite(x) else float(x) for x in v] for k, v in tp.items()}
    thermal = thermal_analysis(tgrid, G, A, keep_all, cam_mask, tp, earth, dt) if tp is not None else None
    thermal_cal = thermal_analysis(tgrid, Gc, Ac, keep_all, cam_mask, tp, earth, dt) if tp is not None else None
    if thermal_cal:
        for k in ("T_blocks", "dTdt_blocks"):
            thermal_cal.pop(k, None)
    android = android_cal(d, ts0, cams)
    tcorr = thermal["per_source"] if thermal else {}

    # start-estimate drift
    def est_err(win_est, lags=(60, 180, 300), ref_win=10.0):
        out = {}
        ok = keep
        Gf = G
        def avg(a, b):
            i0, i1 = int(a / dt), int(b / dt)
            if i0 < 0 or i1 > len(Gf):
                return None
            m = ok[i0:i1]
            if m.mean() < 0.9:
                return None
            return np.nanmean(Gf[i0:i1][m[:len(Gf[i0:i1])]], 0)
        # literal start = first clean second
        st = tgrid[np.argmax(keep)]
        b0 = avg(st, st + win_est)
        lit = {}
        for L in lags:
            b1 = avg(st + L - ref_win / 2, st + L + ref_win / 2)
            lit[str(L)] = None if (b0 is None or b1 is None) else ((b1 - b0) * 1e3).tolist()
        # all start points (every 5 s)
        stat = {}
        for L in lags:
            errs = []
            for s in np.arange(0, T - L - ref_win, 5.0):
                a = avg(s, s + win_est)
                b = avg(s + L - ref_win / 2, s + L + ref_win / 2)
                if a is not None and b is not None:
                    errs.append((b - a) * 1e3)
            errs = np.array(errs)
            if len(errs):
                nrm = np.linalg.norm(errs, axis=1)
                stat[str(L)] = {"n": int(len(errs)), "rms_per_axis_mdeg_s": np.sqrt((errs ** 2).mean(0)).tolist(),
                                "p95_norm_mdeg_s": float(np.percentile(nrm, 95)), "median_norm_mdeg_s": float(np.median(nrm))}
        return {"literal_start_t": float(st), "literal_mdeg_s": lit, "all_starts": stat}

    drift = {"2s": est_err(2.0), "5s": est_err(5.0)}
    # noise floor of a W-second mean from ARW: N/sqrt(W)
    floor = {f"{w}s": [allan["gyro_raw"][ax]["N"] / math.sqrt(w) * 1e3 for ax in "xyz"] for w in (2, 5, 10)}

    res = dict(recording=str(rec.relative_to(ROOT)), duration_s=T, wall_start=wall0, rates_hz=rates, max_gap_s=gaps, coverage=cover,
               cuts=cuts, cuts_source=cut_src, auto_cuts=auto, auto_cut_reasons_t=auto_cuts.why,
               allan_segment_s=[float(tgrid[s0]), float(tgrid[s1 - 1])],
               allan=allan, orientation=orient,
               mean_gyro_raw_mdeg_s=(np.nanmean(G[keep], 0) * 1e3).tolist(),
               mean_gyro_cal_mdeg_s=(np.nanmean(Gc[keep], 0) * 1e3).tolist(),
               mean_gyro_raw_minus_earth_mdeg_s=((np.nanmean(G[keep], 0) - (earth if earth is not None else 0)) * 1e3).tolist(),
               android_bias_estimate_mdeg_s_first_last=None if android_bias is None else
               [(android_bias[0] * 1e3).tolist(), (android_bias[-1] * 1e3).tolist()],
               block10s={"t": bt.tolist(), "gyro_raw_mdeg_s": (bm * 1e3).tolist(), "gyro_cal_mdeg_s": (bmc * 1e3).tolist(),
                         "accel_raw_m_s2": bacc.tolist()},
               block_spread_mdeg_s={"raw_pp": (np.ptp(bm, 0) * 1e3).tolist() if len(bm) else None,
                                    "raw_std": (bm.std(0) * 1e3).tolist() if len(bm) else None},
               temperature=temps, temp_correlation=tcorr, thermal=thermal, thermal_cal=thermal_cal, android_cal=android, camera_intervals_s=cams, static_end_s=float(static_end), start_estimate_error=drift, mean_noise_floor_mdeg_s=floor)
    return res, dict(G=G, Gc=Gc, A=A, tgrid=tgrid, curves=curves, keep=keep, wall0=wall0, cams=cams, thermal=thermal)


def android_cal(d, ts0, cams):
    """Android's online bias estimate = v3..v5 of the uncalibrated streams (16 gyro, 35 accel) = raw - calibrated.
    Returns step changes (time, before, after, jump) and a down-sampled series; checks type4 - type16 consistency."""
    out = {}
    for k, nm, sc, unit in ((16, "gyro", R2D * 1e3, "mdeg/s"), (35, "accel", 1e3 / 9.81, "mg")):
        a = d.get(k)
        if a is None or a.shape[1] < 7:
            continue
        t = (a[:, 0] - ts0) * 1e-9
        b = a[:, 4:7] * sc
        ch = np.flatnonzero(np.any(np.abs(np.diff(b, axis=0)) > 1e-9, axis=1)) + 1
        steps = []
        for i in ch:
            cam = any(c0 <= t[i] <= c1 for c0, c1 in cams)
            steps.append({"t": round(float(t[i]), 2), "before": b[i - 1].round(2).tolist(), "after": b[i].round(2).tolist(),
                          "jump": (b[i] - b[i - 1]).round(2).tolist(), "jump_norm": float(np.linalg.norm(b[i] - b[i - 1])), "during_camera": cam})
        idx = np.unique(np.concatenate([[0], ch, ch - 1, [len(t) - 1]]).clip(0, len(t) - 1))
        out[nm] = {"unit": unit, "n_changes": int(len(ch)), "first": b[0].tolist(), "last": b[-1].tolist(),
                   "range": np.ptp(b, 0).tolist(), "steps": steps[:300], "series_t": t[idx].round(2).tolist(), "series": b[idx].round(3).tolist()}
    # consistency: type4 - type16 raw should equal -estimate
    if 4 in d and 16 in d:
        g4, g16 = d[4], d[16]
        n = min(len(g4), len(g16))
        same_t = np.mean(g4[:n, 0] == g16[:n, 0])
        if same_t > 0.99:
            diff = (g4[:n, 1:4] - g16[:n, 1:4]) * R2D * 1e3
            est = -g16[:n, 4:7] * R2D * 1e3 if g16.shape[1] >= 7 else None
            out["check_type4_minus_type16_vs_minus_estimate_max_mdeg_s"] = None if est is None else float(np.max(np.abs(diff - est)))
    return out


def thermal_analysis(tgrid, G, A, keep, cam_mask, tp, earth, dt, blk_s=30.0):
    """30 s block means of Earth-corrected raw gyro (mdeg/s) and accel (mg) vs each temperature source (camera-off blocks,
    +30 s settle): slope, r, heating / cooling branch slopes (by temperature trend), hysteresis = mean branch offset."""
    e = np.zeros(3) if earth is None else np.asarray(earth)
    nb = int(round(blk_s / dt))
    rows = []
    for i in range(len(tgrid) // nb):
        sl = slice(i * nb, (i + 1) * nb)
        cam = cam_mask[sl].mean() > 0.5
        k = keep[sl] & ~cam_mask[sl]
        if k.mean() < 0.8 and not cam:
            continue
        sel = np.ones(nb, bool) if cam else k
        with np.errstate(all="ignore"):
            gm = (np.nanmean(G[sl][sel], 0) - e) * 1e3
            am = np.nanmean(A[sl][sel], 0) / 9.81 * 1e3
            gstd = np.nanmean(np.nanstd(G[sl].reshape(-1, 100, 3), axis=1), 0) * 1e3
        rows.append((tgrid[sl].mean(), cam, gm, am, gstd))
    if len(rows) < 5:
        return None
    bt = np.array([r[0] for r in rows]); camb = np.array([r[1] for r in rows])
    bg = np.array([r[2] for r in rows]); ba = np.array([r[3] for r in rows]); bs = np.array([r[4] for r in rows])
    out = {"block_s": blk_s, "t": bt.tolist(), "cam": camb.tolist(), "gyro_mdeg_s_earth_corr": bg.tolist(),
           "accel_mg": ba.tolist(), "gyro_noise_1s_std_mdeg_s": bs.tolist(), "per_source": {}}
    with np.errstate(all="ignore"):
        out["noise_cam_on_vs_off_mdeg_s"] = {"on": np.nanmean(bs[camb], 0).tolist() if camb.any() else None,
                                             "off": np.nanmean(bs[~camb], 0).tolist()}
    off = ~camb & np.all(np.isfinite(bg), 1) & np.all(np.isfinite(ba), 1)
    best = None
    for src in ("battery", "AP", "BAT", "CP", "PA", "SKIN", "USB"):
        tv = tp.get(src)
        if tv is None:
            continue
        ok = np.isfinite(tv)
        if ok.sum() < 5:
            continue
        tt, vv = tp["t"][ok], tv[ok]
        Tb = np.interp(bt, tt, vv)
        sm = np.convolve(np.pad(vv, 3, mode="edge"), np.ones(7) / 7, mode="valid")
        dT = np.interp(bt, tt, np.gradient(sm, tt))
        res = {"min": float(np.nanmin(Tb[off])), "max": float(np.nanmax(Tb[off])), "range_C": float(np.ptp(Tb[off]))}
        rs = []
        for lab, Y in (("gyro_raw_mdeg_s", bg), ("accel_raw_mg", ba)):
            for j, ax in enumerate("xyz"):
                x, y = Tb[off], Y[off, j]
                if np.ptp(x) < 0.3:
                    res[f"{lab}_{ax}"] = {"slope_per_C": float("nan"), "r": float("nan")}
                    continue
                k1, c1 = np.polyfit(x, y, 1)
                r = float(np.corrcoef(x, y)[0, 1])
                item = {"slope_per_C": float(k1), "r": r, "resid_std": float(np.std(y - (k1 * x + c1)))}
                h, c_ = off & (dT > 0.002), off & (dT < -0.002)
                for nm, m in (("heating", h), ("cooling", c_)):
                    if m.sum() >= 4 and np.ptp(Tb[m]) > 0.3:
                        kk, _ = np.polyfit(Tb[m], Y[m, j], 1)
                        item[nm] = {"slope_per_C": float(kk), "n": int(m.sum())}
                if h.sum() >= 4 and c_.sum() >= 4:
                    item["hysteresis_cooling_minus_heating"] = float(np.mean(Y[c_, j] - (k1 * Tb[c_] + c1)) - np.mean(Y[h, j] - (k1 * Tb[h] + c1)))
                res[f"{lab}_{ax}"] = item
                if lab.startswith("gyro"):
                    rs.append(abs(r))
        res["mean_abs_r_gyro"] = float(np.mean(rs)) if rs else None
        out["per_source"][src] = res
        out.setdefault("T_blocks", {})[src] = Tb.tolist()
        out.setdefault("dTdt_blocks", {})[src] = dT.tolist()
        if rs and (best is None or np.mean(rs) > best[1]):
            best = (src, float(np.mean(rs)))
    out["best_source"] = best[0] if best else None
    return out


def six_face(rec, min_s=10.0):
    """6-face static test: still blocks (>= min_s, low accel std), grouped by dominant gravity axis (+-x, y, z).
    accel bias b = (f+ + f-)/2, scale = (f+ - f-)/(2 g) per axis; gyro mean per block."""
    d = read_merged(rec)
    acc = d.get(35, d.get(1)); gyr = d.get(16, d.get(4))
    ts0 = min(acc[0, 0], gyr[0, 0]); T = (max(acc[-1, 0], gyr[-1, 0]) - ts0) * 1e-9; dt = 0.01
    A = to_grid((acc[:, 0] - ts0) * 1e-9, acc[:, 1:4], dt, 0, T)
    G = to_grid((gyr[:, 0] - ts0) * 1e-9, gyr[:, 1:4] * R2D, dt, 0, T)
    w = 50; nw = len(A) // w
    with np.errstate(all="ignore"):
        Aw = A[:nw * w].reshape(nw, w, 3); Gw = G[:nw * w].reshape(nw, w, 3)
        still = (np.nanstd(Aw, 1).max(1) < 0.05) & (np.linalg.norm(np.nanmean(Gw, 1), axis=1) < 1.0)
    blocks, i = [], 0
    while i < nw:
        if still[i]:
            j = i
            while j + 1 < nw and still[j + 1]:
                j += 1
            if (j - i + 1) * w * dt >= min_s:
                a, b = (i + 1) * w, j * w
                with np.errstate(all="ignore"):
                    f = np.nanmean(A[a:b], 0); g = np.nanmean(G[a:b], 0)
                ax = int(np.argmax(abs(f))); sg = "+" if f[ax] > 0 else "-"
                blocks.append({"t0": a * dt, "t1": b * dt, "face": sg + "xyz"[ax], "f_m_s2": f.tolist(), "gyro_mdeg_s": (g * 1e3).tolist()})
            i = j + 1
        else:
            i += 1
    faces = {}
    for bl in blocks:
        faces.setdefault(bl["face"], []).append(np.array(bl["f_m_s2"]))
    res = {"blocks": blocks, "faces_found": sorted(faces), "axes": {}, "g_assumed": 9.80}
    for j, ax in enumerate("xyz"):
        if "+" + ax in faces and "-" + ax in faces:
            fp = np.mean(faces["+" + ax], 0)[j]; fm = np.mean(faces["-" + ax], 0)[j]
            res["axes"][ax] = {"bias_m_s2": float((fp + fm) / 2), "bias_mg": float((fp + fm) / 2 / 9.81 * 1e3),
                               "scale": float((fp - fm) / (2 * 9.80)), "scale_err_ppm": float(((fp - fm) / (2 * 9.80) - 1) * 1e6)}
    return res


# --------------------------------------------------------------------------------------------- hs recordings
def analyze_hs(noise_std_gyro_dps=None):
    out = []
    for rd in sorted(glob.glob(str(ROOT / "recordings/hs_2026101*_*"))):
        rd = Path(rd)
        f = rd / "imu.csv"
        if not f.exists():
            continue
        d = read_csv(f)
        if 4 not in d or 1 not in d:
            continue
        ts0 = min(d[4][0, 0], d[1][0, 0])
        T = (max(d[4][-1, 0], d[1][-1, 0]) - ts0) * 1e-9
        dt = 0.01
        G = fill_nan(to_grid((d[4][:, 0] - ts0) * 1e-9, d[4][:, 1:4] * R2D, dt, 0, T))
        A = fill_nan(to_grid((d[1][:, 0] - ts0) * 1e-9, d[1][:, 1:4], dt, 0, T))
        w = 50                                  # 0.5 s windows
        nw = len(G) // w
        Gw = G[:nw * w].reshape(nw, w, 3)
        Aw = A[:nw * w].reshape(nw, w, 3)
        gstd = Gw.std(1).max(1)
        gmn = np.linalg.norm(Gw.mean(1), axis=1)
        astd = Aw.std(1).max(1)
        lim_g = 3 * noise_std_gyro_dps if noise_std_gyro_dps else 0.15
        still = (gstd < lim_g) & (gmn < 0.3) & (astd < 0.06)
        segs = []
        i = 0
        while i < nw:
            if still[i]:
                j = i
                while j + 1 < nw and still[j + 1]:
                    j += 1
                if (j - i + 1) * w * dt >= 1.0:
                    a, b = i * w, (j + 1) * w
                    segs.append({"t0": round(a * dt, 1), "t1": round(b * dt, 1),
                                 "gyro_mean_mdeg_s": (G[a:b].mean(0) * 1e3).round(1).tolist(),
                                 "gyro_std_mdeg_s": (G[a:b].std(0) * 1e3).round(1).tolist(),
                                 "mean_noise_mdeg_s": None})
                i = j + 1
            else:
                i += 1
        meta = json.loads((rd / "meta.json").read_text()) if (rd / "meta.json").exists() else {}
        r = {"name": rd.name, "duration_s": round(T, 1), "battery_c_end": meta.get("battery_c_end"),
             "fps": meta.get("fps"), "segments": segs,
             "quietest_0p5s_window": {"gyro_std_p1_dps": float(np.percentile(gstd, 1)), "gyro_mean_norm_p1_dps": float(np.percentile(gmn, 1)),
                                      "accel_std_p1_m_s2": float(np.percentile(astd, 1))}}
        if len(segs) >= 2:
            m = np.array([s["gyro_mean_mdeg_s"] for s in segs])
            r["first_vs_last_mdeg_s"] = (m[-1] - m[0]).round(1).tolist()
            r["max_spread_mdeg_s"] = np.ptp(m, 0).round(1).tolist()
            tt = np.array([(s["t0"] + s["t1"]) / 2 for s in segs])
            if np.ptp(tt) > 30:
                r["trend_mdeg_s_per_min"] = [float(np.polyfit(tt / 60, m[:, j], 1)[0]) for j in range(3)]
        out.append(r)
        log(f"hs {rd.name}: {len(segs)} still segments")
    return out


# --------------------------------------------------------------------------------------------- raw page
def env(t, y, n=1500):
    k = max(1, len(t) // n)
    m = (len(t) // k) * k
    tt = t[:m].reshape(-1, k).mean(1)
    Y = y[:m].reshape(-1, k)
    with np.errstate(all="ignore"):
        return tt, np.nanmin(Y, 1), np.nanmax(Y, 1), np.nanmean(Y, 1)


def raw_page(res, arr, path):
    t = arr["tgrid"]
    G, A = arr["G"], arr["A"]
    series = {}
    for nm, Y in (("gx", G[:, 0]), ("gy", G[:, 1]), ("gz", G[:, 2]), ("ax", A[:, 0]), ("ay", A[:, 1]), ("az", A[:, 2]),
                  ("wn", np.linalg.norm(G - np.nanmedian(G, 0), axis=1)), ("an", np.linalg.norm(A, axis=1))):
        tt, lo, hi, mn = env(t, Y)
        r = lambda v: [None if not np.isfinite(x) else round(float(x), 5) for x in v]
        series[nm] = {"t": [round(float(x), 2) for x in tt], "lo": r(lo), "hi": r(hi), "mean": r(mn)}
    data = {"series": series, "temps": res["temperature"], "cuts": res["cuts"], "auto_cuts": res["auto_cuts"],
            "cuts_source": res["cuts_source"], "wall0": res["wall_start"], "cams": res.get("camera_intervals_s", []), "duration": res["duration_s"],
            "rec": res["recording"]}
    html = RAW_HTML.replace("__DATA__", json.dumps(data))
    Path(path).write_text(html, encoding="utf-8")


RAW_HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>정지 IMU 원자료</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.35.2/plotly.min.js"></script>
<style>
:root{--bg:#fff;--fg:#1a1a1a;--mut:#666;--line:#ddd}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#15171a;--fg:#e8e8e8;--mut:#9a9a9a;--line:#333}}
body{background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif;margin:0 auto;max-width:1200px;padding:12px 16px}
table{border-collapse:collapse;margin:8px 0}td,th{border:1px solid var(--line);padding:3px 8px;text-align:right}
.mut{color:var(--mut)} #plot{width:100%;height:1100px}
</style></head><body>
<h1>정지 IMU 원자료 (폰을 책상에 둔 녹화)</h1>
<p id="info" class="mut"></p>
<p>그래프를 끌어서 확대, 두 번 눌러 되돌림. 네 그래프의 시간 축은 같이 움직입니다. 연한 띠 = 구간 안 최솟값~최댓값(짧은 충격도 보이게), 진한 선 = 평균.
<b style="color:#d33">빨간 칸</b> = 움직임 의심 구간(자동 검출), <b style="color:#2a6fdb">파란 칸</b> = 가열용 240 fps 카메라 켠 구간(움직임 검출에서 뺌).</p>
<div id="plot"></div>
<h2>움직임 의심 구간</h2><table id="cuts"><tr><th>#</th><th>시작 s</th><th>끝 s</th><th>길이 s</th><th>시작 시각</th></tr></table>
<p class="mut">자를 구간을 알려 주면 <code>runs/static_imu_cuts.json</code> 에 [[시작,끝],...] (초) 로 적고 분석을 다시 돌립니다.</p>
<script>
const D=__DATA__;
const clock=s=>D.wall0?new Date((D.wall0+s)*1000).toLocaleTimeString('ko-KR',{hour12:false}):'';
document.getElementById('info').textContent=`${D.rec} · 길이 ${D.duration.toFixed(0)} s · 시작 ${clock(0)} · 자르기: ${D.cuts_source}`;
const col={x:'#2a6fdb',y:'#e07b00',z:'#1a9e5a',n:'#7a4fd0'};
const tr=[];
function band(s,c,name,ax,leg){
 const S=D.series[s];
 tr.push({x:S.t,y:S.lo,mode:'lines',line:{width:0},showlegend:false,hoverinfo:'skip',xaxis:'x',yaxis:ax});
 tr.push({x:S.t,y:S.hi,mode:'lines',line:{width:0},fill:'tonexty',fillcolor:c+'40',showlegend:false,hoverinfo:'skip',xaxis:'x',yaxis:ax});
 tr.push({x:S.t,y:S.mean,mode:'lines',line:{width:1.2,color:c},name:name,xaxis:'x',yaxis:ax,
   customdata:S.t.map(clock),hovertemplate:'%{x:.1f} s (%{customdata})<br>%{y:.4f}<extra>'+name+'</extra>'});
}
band('gx',col.x,'자이로 x °/s','y');band('gy',col.y,'자이로 y °/s','y');band('gz',col.z,'자이로 z °/s','y');
band('ax',col.x,'가속도 x m/s²','y2');band('ay',col.y,'가속도 y m/s²','y2');band('az',col.z,'가속도 z m/s²','y2');
band('wn',col.n,'|ω−중앙값| °/s','y3');band('an','#888888','|a| m/s²','y3');
if(D.temps){for(const [k,c] of [['battery','#d33'],['AP','#2a6fdb'],['SKIN','#1a9e5a'],['BAT','#e07b00'],['CP','#7a4fd0']]){
  if(!D.temps[k])continue; tr.push({x:D.temps.t,y:D.temps[k],mode:'lines+markers',marker:{size:3},line:{color:c},name:'온도 '+k+' °C',xaxis:'x',yaxis:'y4',
   customdata:D.temps.t.map(clock),hovertemplate:'%{x:.0f} s (%{customdata})<br>%{y:.1f} °C<extra>'+k+'</extra>'});}}
const shapes=D.auto_cuts.map(c=>({type:'rect',xref:'x',yref:'paper',x0:c[0],x1:c[1],y0:0,y1:1,fillcolor:'rgba(220,40,40,0.18)',line:{width:0}}))
 .concat((D.cams||[]).map(c=>({type:'rect',xref:'x',yref:'paper',x0:c[0],x1:c[1],y0:0,y1:1,fillcolor:'rgba(40,110,220,0.12)',line:{width:0}})));
const dark=matchMedia('(prefers-color-scheme: dark)').matches, fg=dark?'#ddd':'#222', gr=dark?'#333':'#e5e5e5';
const ax=t=>({title:{text:t},gridcolor:gr,zeroline:false});
Plotly.newPlot('plot',tr,{grid:{rows:4,columns:1,pattern:'coupled'},paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{color:fg},
 xaxis:{title:{text:'시작부터 초'},gridcolor:gr},yaxis:ax('자이로 °/s'),yaxis2:ax('가속도 m/s²'),yaxis3:ax('크기'),yaxis4:ax('온도 °C'),
 shapes,legend:{orientation:'h',y:1.06},margin:{l:60,r:10,t:30,b:40},hovermode:'x unified'},{responsive:true});
const tb=document.getElementById('cuts');
D.auto_cuts.forEach((c,i)=>{const r=tb.insertRow();[i+1,c[0].toFixed(1),c[1].toFixed(1),(c[1]-c[0]).toFixed(1),clock(c[0])].forEach(v=>r.insertCell().textContent=v);});
if(!D.auto_cuts.length){tb.insertRow().insertCell().textContent='검출된 움직임 없음';}
</script></body></html>"""


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    preview = "--preview" in sys.argv
    if "--six-face" in sys.argv:
        rec = Path(args[0])
        rec = rec if rec.is_absolute() else ROOT / rec
        r = six_face(rec)
        (ROOT / "runs/six_face_result.json").write_text(json.dumps(r, indent=1))
        print(json.dumps(r["axes"], indent=1), r["faces_found"])
        return
    rec = Path(args[0]) if args else ROOT / (ROOT / "runs/static_imu_current.txt").read_text().strip()
    if not rec.is_absolute():
        rec = ROOT / rec
    res, arr = analyze_static(rec, preview)
    raw_page(res, arr, ROOT / "docs/static_imu_raw.html")
    log("wrote docs/static_imu_raw.html")
    if preview:
        print(json.dumps({k: res[k] for k in ("auto_cuts", "auto_cut_reasons_t")}, indent=1))
        print({ax: {k: v for k, v in res["allan"]["gyro_raw"][ax].items() if k in ("ARW_deg_per_sqrt_h", "BI_deg_s", "tau_B")} for ax in "xyz"})
        return
    noise = float(np.nanstd(arr["G"][arr["keep"]], 0).max())
    hs = analyze_hs(noise)
    res["hs_recordings"] = hs
    res["hs_still_threshold_gyro_std_dps"] = 3 * noise
    out = ROOT / "runs/gyro_bias_stability.json"
    out.write_text(json.dumps(res, indent=1))
    import report_static
    report_static.write(res, arr, ROOT / "docs/gyro_bias_stability.html")
    log("wrote runs/gyro_bias_stability.json + docs/gyro_bias_stability.html")


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).parent))
    main()
