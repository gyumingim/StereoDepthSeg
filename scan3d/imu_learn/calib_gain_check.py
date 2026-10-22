#!/usr/bin/env python3
"""Independent re-computation of runs/imu_calib_gain/result.json (A, B, C, D medians) with a different code path:
own CSV parser, per-window sequential rotation-matrix integration (Rodrigues, no prefix scan, no shared helpers except
still_runs), every 40th window start. Pass = every median within 3 % (or 0.03 deg) of result.json.
  ../venv_ffs/bin/python imu_learn/calib_gain_check.py   -> runs/imu_calib_gain/check.json
"""
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
S = HERE.parent
sys.path.insert(0, str(HERE))
from i3_confirm import still_runs  # noqa: E402

EVERY = 40


def rod(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def load(path, typ):
    out = {}
    with open(path) as f:
        for r in csv.reader(f):
            if len(r) >= 5 and r[0] == str(typ):
                try:
                    out[int(r[1])] = [float(r[2]), float(r[3]), float(r[4])]
                except ValueError:
                    pass
    t = np.array(sorted(out), dtype=np.int64)
    return t, np.array([out[k] for k in t])


def rod_b(v):
    th = np.linalg.norm(v, axis=1)
    k = v / np.maximum(th, 1e-15)[:, None]
    K = np.zeros((len(v), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 2] = -k[:, 2], k[:, 1], -k[:, 0]
    K[:, 1, 0], K[:, 2, 0], K[:, 2, 1] = k[:, 2], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], (1 - np.cos(th))[:, None, None]
    return np.eye(3) + s * K + c * (K @ K)


def integ(t, w, b, ta, tb):
    """Rotations of IMU from ta[n] to tb[n] (batched over windows), sequential sample-by-sample products of
    piecewise-constant w - b; each step covers [max(t_i, ta), min(t_i+1, tb)]."""
    i = np.searchsorted(t, ta, side="right") - 1
    R = np.tile(np.eye(3), (len(ta), 1, 1))
    cur = ta.astype(np.int64).copy()
    done = np.zeros(len(ta), bool)
    tn = np.r_[t, np.iinfo(np.int64).max]
    while not done.all():
        nxt = tn[np.minimum(i + 1, len(t))]
        end = np.minimum(nxt, tb)
        h = np.where(done, 0, end - cur) / 1e9
        ii = np.minimum(i, len(t) - 1)
        R = R @ rod_b((w[ii] - b[ii]) * h[:, None])
        done |= end >= tb
        cur, i = end, i + 1
    return R


def ang(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def main():
    res = json.load(open(S / "runs/imu_calib_gain/result.json"))
    out, ok_all = dict(made=time.strftime("%F %T"), every=EVERY, recs={}), True
    for r in res["recs"]:
        if "cond" not in r:
            continue
        rec = r["rec"]
        P = S / "recordings" / rec
        t16, w16 = load(P / "imu.csv", 16)
        t4, w4 = load(P / "imu.csv", 4)
        al = json.load(open(P / "imu_gt/align.json"))
        Rci, dt = np.array(al["R_ci"]), int(al["dt_ns"])
        g = np.load(P / "imu_gt_spl/gt.npz")
        tc, Rb = g["t_ns"][g["ok"]] - dt, g["R_bc"][g["ok"]]
        runs = still_runs(t16, w16)

        def smean(a, b):
            a2 = a + 2_000_000_000 if (b - a) >= 4_000_000_000 else a
            m = (t16 >= a2) & (t16 <= b)
            return w16[m].mean(0)
        sb = [smean(a, b) for a, b in runs]
        ends = np.array([b for _, b in runs])
        bD = np.array([sb[max(np.searchsorted(ends, x, side="right") - 1, 0)] for x in t16])
        B = {"A": (t16, w16, np.zeros_like(w16)), "B": (t4, w4, np.zeros_like(w4)),
             "C": (t16, w16, np.tile(sb[0], (len(t16), 1))), "D": (t16, w16, bD)}
        starts = np.nonzero(tc > runs[0][1])[0][::EVERY]
        errs = {c: [] for c in B}
        I, J = [], []
        for i in starts:
            tgt = tc[i] + 10_000_000_000
            j = int(np.argmin(np.abs(tc - tgt)))
            if abs(tc[j] - tgt) > 10_000_000 or tgt > min(t16[-1], t4[-1]):
                continue
            I.append(i)
            J.append(j)
        I, J = np.array(I), np.array(J)
        for c, (t, w, b) in B.items():
            Ri = integ(t, w, b, tc[I], tc[J])
            for n in range(len(I)):
                Rg = Rb[I[n]].T @ Rb[J[n]]
                errs[c].append(ang(Rg.T @ (Rci @ Ri[n] @ Rci.T)))
        d = {}
        for c in B:
            m = float(np.median(errs[c]))
            ref = r["cond"][c]["median"]
            ok = abs(m - ref) <= max(0.03, 0.03 * ref)
            ok_all &= ok
            d[c] = dict(check=round(m, 3), result=round(ref, 3), n=len(errs[c]), ok=ok)
        out["recs"][rec] = d
        print(rec, d, flush=True)
    out["PASS"] = bool(ok_all)
    json.dump(out, open(S / "runs/imu_calib_gain/check.json", "w"), indent=1)
    print("PASS" if ok_all else "FAIL")


if __name__ == "__main__":
    main()
