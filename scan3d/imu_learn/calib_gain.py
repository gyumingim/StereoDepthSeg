#!/usr/bin/env python3
"""Gyro bias calibration gain (docs/imu_calib_gain_plan.md, pre-registered 958c3ae).
Gyro-only orientation integration error vs spline GT (imu_gt_spl, vision only) over 10 s windows starting at every
GT-valid frame after the calibration still. Conditions A raw / B type4 / C start-still bias / D causal re-estimate at
every still / E oracle constant bias.
  ../venv_ffs/bin/python imu_learn/calib_gain.py            # -> runs/imu_calib_gain/result.json
"""
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
sys.path.insert(0, str(HERE))
from i3_confirm import still_runs  # noqa: E402
from imu_gt import read_imu  # noqa: E402

RECS = ["hs_20261021_213341", "hs_20261021_214550", "hs_20261021_215348",
        "hs_20261020_214138", "hs_20261020_221052", "hs_20261020_221903", "hs_20261020_221925",
        "hs_20261020_222747", "hs_20261020_223815", "hs_20261020_224323"]
WIN_S, END_TOL_S, MIN_WIN = 10.0, 0.010, 100
TRIM_S, TRIM_MIN_LEN = 2.0, 4.0
GATE = np.radians(0.100)                                     # D2 only (post-hoc): 100 mdeg/s
OUT = SCAN3D / "runs" / "imu_calib_gain"


def read_type4(path):
    rows = []
    for line in open(path):
        if line.startswith("4,"):
            p = line.strip().split(",")
            try:
                rows.append([int(p[1])] + [float(x) for x in p[2:5]])
            except (ValueError, IndexError):
                pass
    return np.asarray(rows)


def still_mean(t, w, a, b):
    if (b - a) / 1e9 >= TRIM_MIN_LEN:
        a = a + int(TRIM_S * 1e9)
    m = (t >= a) & (t <= b)
    return w[m].mean(0)


def qmul(p, q):  # scalar-last quaternions, Hamilton product p*q
    x1, y1, z1, w1 = p[..., 0], p[..., 1], p[..., 2], p[..., 3]
    x2, y2, z2, w2 = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], -1)


def integrate(t, w, bias):
    """Q_k = orientation of IMU at t_k relative to t_0 (body-frame right products). w, bias: (n,3) rad/s."""
    dt = np.diff(t) / 1e9
    inc = Rot.from_rotvec((w[:-1] - bias[:-1]) * dt[:, None]).as_quat()
    cum = np.vstack([[0, 0, 0, 1.0], inc])          # cum[k] = inc_0..inc_{k-1} after scan
    s = 1
    while s < len(cum):                                # Hillis-Steele inclusive prefix product
        new = cum.copy()
        new[s:] = qmul(cum[:-s], cum[s:])
        cum = new / np.linalg.norm(new, axis=1, keepdims=True)
        s *= 2
    return cum


def orient_at(t, q, w, bias, tq):
    """Orientation at arbitrary IMU times tq (Rotation)."""
    k = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, len(t) - 1)
    part = Rot.from_rotvec((w[k] - bias[k]) * ((tq - t[k]) / 1e9)[:, None])
    return Rot.from_quat(q[k]) * part


def run_rec(rec):
    R = SCAN3D / "recordings" / rec
    acc, g16 = read_imu(R / "imu.csv", raw=True)
    acc1, _ = read_imu(R / "imu.csv", raw=False)
    g4 = read_type4(R / "imu.csv")
    al = json.load(open(R / "imu_gt" / "align.json"))
    R_ci, dt_ns = np.asarray(al["R_ci"]), int(al["dt_ns"])
    gt = np.load(R / "imu_gt_spl" / "gt.npz")
    ok = gt["ok"]
    tc, Rbc = gt["t_ns"][ok].astype(np.int64), gt["R_bc"][ok]
    ti = tc - dt_ns                                     # GT frame times on the IMU clock

    def clean(g):
        g = g[np.argsort(g[:, 0], kind="stable")]
        keep = np.r_[True, np.diff(g[:, 0]) > 0]
        return g[keep][:, 0].astype(np.int64), g[keep][:, 1:4]
    t16, w16 = clean(g16)
    t4, w4 = clean(g4)
    runs = still_runs(t16, w16)
    info = dict(rec=rec, n_gt_ok=int(ok.sum()), stills_s=[[round((a - ti[0]) / 1e9, 2), round((b - ti[0]) / 1e9, 2)]
                                                          for a, b in runs],
                max_gap_ms_16=float(np.diff(t16).max() / 1e6), max_gap_ms_4=float(np.diff(t4).max() / 1e6))
    if not runs:
        info["excluded"] = "no IMU still (no calibration still)"
        return info
    sb = [still_mean(t16, w16, a, b) for a, b in runs]
    info["still_bias_mdps"] = [np.round(np.degrees(b) * 1e3, 1).tolist() for b in sb]
    cal_end = runs[0][1]

    # bias time series per condition (on t16 samples)
    zero = np.zeros_like(w16)
    bC = np.tile(sb[0], (len(t16), 1))
    ends = np.array([b for _, b in runs])
    idx = np.searchsorted(ends, t16, side="right") - 1      # latest still that has ended at t
    bD = np.array(sb)[np.clip(idx, 0, None)]                 # before the first still ends: = start still (= C)
    # D2 (POST-HOC, exploratory, not pre-registered): skip a still whose mean differs from the last accepted one by
    # > GATE on any axis (slow rotation mistaken for still, e.g. 222747 7.8-11.1 s y -47 mdeg/s)
    acc_b, gated = [sb[0]], [False]
    for v in sb[1:]:
        bad = bool(np.any(np.abs(v - acc_b[-1]) > GATE))
        gated.append(bad)
        acc_b.append(acc_b[-1] if bad else v)
    info["D2_gated_stills"] = gated
    bD2 = np.array(acc_b)[np.clip(idx, 0, None)]
    # E oracle: mean(w_raw - R_ci^T w_gt)
    d = np.diff(ti) / 1e9
    m = (d > 0) & (d < 0.02)
    Rr = Rot.from_matrix(Rbc)
    wc = (Rr[:-1].inv() * Rr[1:]).as_rotvec()[m] / d[m][:, None]
    tm = ((ti[:-1] + ti[1:]) // 2)[m]
    wi_pred = wc @ R_ci                                       # R_ci^T w_cam, row form
    wraw = np.stack([np.interp(tm, t16, w16[:, j]) for j in range(3)], 1)
    bE_v = (wraw - wi_pred).mean(0)
    info["oracle_bias_mdps"] = np.round(np.degrees(bE_v) * 1e3, 1).tolist()
    bE = np.tile(bE_v, (len(t16), 1))
    conds = {"A": (t16, w16, zero), "B": (t4, w4, np.zeros_like(w4)), "C": (t16, w16, bC), "D": (t16, w16, bD),
             "E": (t16, w16, bE), "D2": (t16, w16, bD2)}

    # windows
    i0 = np.nonzero(ti > cal_end)[0]
    tgt = ti[i0] + int(WIN_S * 1e9)
    j = np.clip(np.searchsorted(ti, tgt), 1, len(ti) - 1)
    j = np.where(np.abs(ti[j - 1] - tgt) < np.abs(ti[j] - tgt), j - 1, j)
    good = (np.abs(ti[j] - tgt) <= END_TOL_S * 1e9) & (tgt <= min(t16[-1], t4[-1]))
    i0, j = i0[good], j[good]
    info["n_windows"] = int(len(i0))
    if len(i0) < MIN_WIN:
        info["excluded"] = f"windows {len(i0)} < {MIN_WIN}"
        return info
    Rgt = Rr[i0].inv() * Rr[j]
    span = (ti[j] - ti[i0]) / 1e9
    # gravity in board frame from calibration still (acc type 1)
    a0, a1 = runs[0]
    if (a1 - a0) / 1e9 >= TRIM_MIN_LEN:
        a0 += int(TRIM_S * 1e9)
    am = acc1[(acc1[:, 0] >= a0) & (acc1[:, 0] <= a1), 1:4].mean(0)
    fs = (ti >= a0) & (ti <= a1)
    gb = (Rbc[fs] @ (R_ci @ am)).mean(0) if fs.any() else Rbc[0] @ (R_ci @ am)
    gb /= np.linalg.norm(gb)
    Rci = Rot.from_matrix(R_ci)
    res = {}
    for c, (t, w, b) in conds.items():
        q = integrate(t, w, b)
        Q0, Q1 = orient_at(t, q, w, b, ti[i0]), orient_at(t, q, w, b, ti[j])
        Rest = Rci * (Q0.inv() * Q1) * Rci.inv()
        err = np.degrees((Rgt.inv() * Rest).magnitude())
        phi = (Rr[i0] * Rest * Rr[j].inv()).as_rotvec()
        yaw = np.degrees(np.abs(phi @ gb)) / span * 60
        res[c] = dict(median=float(np.median(err)), p90=float(np.percentile(err, 90)), mean=float(err.mean()),
                      yaw_dpm_median=float(np.median(yaw)), yaw_dpm_p90=float(np.percentile(yaw, 90)))
        info.setdefault("_err", {})[c] = err
        info.setdefault("_t0", (ti[i0] - ti[0]) / 1e9)
    info["cond"] = res
    return info


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    head = subprocess.run(["git", "-C", str(SCAN3D), "rev-parse", "--short", "HEAD"], capture_output=True,
                          text=True).stdout.strip()
    out = dict(made=time.strftime("%F %T"), git_head=head, plan="docs/imu_calib_gain_plan.md (958c3ae)",
               inputs="recordings/<rec>/{imu.csv, imu_gt/align.json (R_ci, dt_ns), imu_gt_spl/gt.npz}", recs=[])
    curves = {}
    for rec in RECS:
        r = run_rec(rec)
        if "_err" in r:
            e, t0 = r.pop("_err"), r.pop("_t0")
            curves[rec] = dict(t0=t0, **e)
        out["recs"].append(r)
        print(rec, r.get("excluded", ""), {c: round(v["median"], 3) for c, v in r.get("cond", {}).items()}, flush=True)
    used = [r for r in out["recs"] if "cond" in r]
    summ = {}
    for c in ["A", "B", "C", "D", "E", "D2"]:
        meds = [r["cond"][c]["median"] for r in used]
        summ[c] = dict(median_of_medians=float(np.median(meds)), p90_median=float(np.median([r["cond"][c]["p90"] for r in used])),
                       yaw_dpm_median=float(np.median([r["cond"][c]["yaw_dpm_median"] for r in used])))
    wins = [r["rec"] for r in used if r["cond"]["D"]["median"] <= 0.8 * r["cond"]["B"]["median"]]
    wins_c = [r["rec"] for r in used if r["cond"]["D"]["median"] <= 0.8 * r["cond"]["C"]["median"]]
    out["summary"] = summ
    out["decision"] = dict(rule="D median >= 20% lower than B on >= 3 recordings", recs_D_beats_B=wins,
                           D_better_than_B=len(wins) >= 3, recs_D_beats_C_20pct=wins_c,
                           reduction_B_to_D_pct=100 * (1 - summ["D"]["median_of_medians"] / summ["B"]["median_of_medians"]))
    json.dump(out, open(OUT / "result.json", "w"), indent=1)
    np.savez_compressed(OUT / "curves.npz", **{f"{rec}|{k}": v for rec, d in curves.items() for k, v in d.items()})
    print(json.dumps(out["summary"], indent=1), json.dumps(out["decision"], indent=1))


if __name__ == "__main__":
    main()
