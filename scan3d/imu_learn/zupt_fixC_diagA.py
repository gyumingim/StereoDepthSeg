#!/usr/bin/env python3
"""Diagnosis of fix A regressions from existing logs only (runs/zupt_fixA, no new VIO runs).

For each cell: base vs A init time, first tracking pose, ZUPT streaks (count/length, first 20 s), the first time the two
runs' frame-invariant motion differs (1-s displacement norm |p(t+1)-p(t)| and speed |v|), and the 1-s displacement error
vs GT over time (frame-free, so no alignment needed) to locate where A's error grows.

  ../venv_ffs/bin/python imu_learn/zupt_fixC_diagA.py > runs/zupt_fixC/diag_A.json
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an  # noqa: E402  (parser only; OUT = runs/zupt_fixA)

CELLS = sys.argv[1:] or ["Z_O_154320_o0", "I3_214550_o1", "I3_213955_o1", "I3_214550_o2", "BR_133820_o0"]


def load(name):
    pre = an.OUT / name
    j = json.load(open(str(pre) + ".json"))
    P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
    fr = an.parse(str(pre) + ".stderr.log.gz")
    return j, P, fr


def gt_center(rec, j, P0):
    f = an.SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz"
    g = np.load(f, allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    c = g["p_bc"][ok].reshape(-1, 3)  # GT fix 2026-10-23: p_bc is already the camera centre (was -R_bc^T p_bc)
    off = P0["t_ns"] - P0["t"] * 1e9 + j["align_dt_ms"] * 1e6
    return (g["t_ns"][ok] - off) / 1e9, c


def interp(t, T, X):
    return np.stack([np.interp(t, T, X[:, k]) for k in range(X.shape[1])], 1)


def streaks(fr):
    out, s = [], None
    for f in fr:
        if f["zacc"]:
            s = [f["t"], f["t"], 1] if s is None else [s[0], f["t"], s[2] + 1]
        elif s is not None and f["slam"] is not None:
            out.append(s)
            s = None
    if s:
        out.append(s)
    return out


def one(cell):
    r = {"cell": cell}
    D = {c: load(f"{cell}_{c}") for c in ("base", "A")}
    tr = {}
    for c, (j, P, fr) in D.items():
        T = [p for p in P if p.get("status") == "tracking" and p.get("p")]
        t = np.array([p["t"] for p in T])
        p = np.array([p["p"] for p in T])
        v = np.array([p.get("v") or [np.nan] * 3 for p in T])
        tr[c] = (t, p, v)
        st = streaks(fr)
        L = np.array([b - a for a, b, k in st]) if st else np.zeros(0)
        up = an.updates(fr)
        r[c] = dict(ate=j.get("ate_cm"), t_init=j.get("t_init_clip_s"), first_track=float(t[0]) if len(t) else None,
                    n_zupt=sum(f["zacc"] for f in fr), n_streak=len(st),
                    streak_len_p50=float(np.median(L)) if len(L) else None, streak_len_max=float(L.max()) if len(L) else None,
                    n_streak_ge03=int((L >= 0.3).sum()), n_streak_ge072=int((L >= 0.72).sum()),
                    zupt_first20=sum(1 for f in fr if f["zacc"] and f["t"] < 20),
                    streaks_first20=[[round(a, 2), round(b, 2), k] for a, b, k in st if a < 20][:12],
                    first_update=up[0]["t"] if up else None,
                    wipeouts=an.wipeouts(up), all_reject=an.all_reject(up))
    # frame-invariant divergence base vs A: 1-s displacement norm and speed
    tb, pb, vb = tr["base"]
    ta, pa, va = tr["A"]
    t0, t1 = max(tb[0], ta[0]), min(tb[-1], ta[-1]) - 1
    grid = np.arange(t0, t1, 0.1)

    def d1(t, p):
        return np.linalg.norm(interp(grid + 1, t, p) - interp(grid, t, p), axis=1)
    db, da = d1(tb, pb), d1(ta, pa)
    sb, sa = np.linalg.norm(interp(grid, tb, vb), axis=1), np.linalg.norm(interp(grid, ta, va), axis=1)
    first = lambda m: round(float(grid[np.argmax(m)]), 2) if m.any() else None  # noqa: E731
    r["diverge_disp1s_gt_1cm"] = first(np.abs(db - da) > 0.01)
    r["diverge_disp1s_gt_5cm"] = first(np.abs(db - da) > 0.05)
    r["diverge_speed_gt_5cms"] = first(np.abs(sb - sa) > 0.05)
    # 1-s displacement error vs GT (frame-free), per condition, binned 10 s
    j, P, _ = D["base"]
    rec = Path(j["rec"]).name
    try:
        gt, gc = gt_center(rec, j, P[0])
        m = (grid > gt[0]) & (grid + 1 < gt[-1])
        g = grid[m]
        dg = np.linalg.norm(interp(g + 1, gt, gc) - interp(g, gt, gc), axis=1)
        eb, ea = np.abs(db[m] - dg), np.abs(da[m] - dg)
        bins = {}
        for lo in range(0, int(g[-1]) + 1, 10):
            k = (g >= lo) & (g < lo + 10)
            if k.any():
                bins[f"{lo}-{lo + 10}"] = [round(float(np.mean(eb[k])) * 100, 2), round(float(np.mean(ea[k])) * 100, 2)]
        r["disp1s_err_cm_base_A_by10s"] = bins
        r["first_A_disp1s_err_gt_5cm"] = first(ea > 0.05)
        r["first_base_disp1s_err_gt_5cm"] = first(eb > 0.05)
    except FileNotFoundError:
        r["gt"] = None
    return r


if __name__ == "__main__":
    print(json.dumps([one(c) for c in CELLS], indent=1))
