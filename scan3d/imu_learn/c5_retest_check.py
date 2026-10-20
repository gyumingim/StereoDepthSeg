#!/usr/bin/env python3
"""C5 retest self-check (docs/c5_retest_plan.md): recompute every run's labels straight from the raw files
(<name>.json + poses.jsonl.gz + GT npz) with independent code (own matching / alignment, no analyse_run), then the
main table (failures per lead x S x config, both rules) and compare it with runs/c5_retest/summary.json.

  ../venv_ffs/bin/python imu_learn/c5_retest_check.py runs/c5_retest
"""
import gzip
import json
import re
import sys
from collections import Counter
from pathlib import Path

import numpy as np

SCAN3D = Path(__file__).resolve().parent.parent
NAME = re.compile(r"^(L2_)?(\d{6})_S(\d)_(loose|strict|strictdyn)_(dm025|d0|dp025)$")


def align(P, Q, with_scale):
    """least squares Q ~ s R P + t (Horn/Umeyama via SVD), returns rmse, s"""
    mp, mq = P.mean(0), Q.mean(0)
    X, Y = P - mp, Q - mq
    U, S, Vt = np.linalg.svd(Y.T @ X)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (X ** 2).sum() if with_scale else 1.0
    e = Q - (s * X @ R.T + mq)
    return float(np.sqrt((e ** 2).sum(1).mean())), float(s)


def label(d, poses, gt_t, gt_p, t_from=None, t_to=None, min_n=50):
    T = [(m["t_ns"], np.array(m["T_world_cam0"]).reshape(4, 4)[:3, 3]) for m in poses
         if m.get("status") == "tracking" and m.get("T_world_cam0") and np.all(np.isfinite(m["T_world_cam0"]))]
    if not T:
        return "noinit"
    t = np.array([x[0] for x in T]) + int(d["align_dt_ms"] * 1e6)
    P = np.array([x[1] for x in T])
    t_init = T[0][0] + int(d["align_dt_ms"] * 1e6)
    lo = t_init if t_from is None else t_init + t_from
    hi = np.inf if t_to is None else t_init + t_to
    sel = (t >= lo) & (t <= hi)
    t, P = t[sel], P[sel]
    k = np.searchsorted(gt_t, t)
    k = np.clip(k, 1, len(gt_t) - 1)
    k = np.where(np.abs(gt_t[k - 1] - t) < np.abs(gt_t[k] - t), k - 1, k)
    ok = np.abs(gt_t[k] - t) < 2e6
    if ok.sum() < min_n:
        return "unlabelled"
    ate, _ = align(P[ok], gt_p[k[ok]], False)
    _, s = align(P[ok], gt_p[k[ok]], True)
    bad = (s < 0.5 or s > 2) or not (ate * 100 <= 10)
    return "collapsed" if bad else "ok"


def main():
    d0 = Path(sys.argv[1])
    gts, outs = {}, {}
    for f in sorted(d0.glob("*.json")):
        m = NAME.match(f.stem)
        if not m:
            continue
        d = json.load(open(f))
        rec = d["rec"]
        if rec not in gts:
            g = np.load(SCAN3D / "recordings" / rec / (d.get("gt") or "imu_gt_spl") / "gt.npz", allow_pickle=True)
            ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
            gts[rec] = (g["t_ns"][ok], g["p_bc"][ok])
        poses = [json.loads(x) for x in gzip.open(str(f)[:-5] + ".poses.jsonl.gz", "rt")]
        full = label(d, poses, *gts[rec])
        win = label(d, poses, *gts[rec], t_from=0, t_to=10e9, min_n=30)
        if win == "unlabelled" and full != "noinit":
            win = label(d, poses, *gts[rec], t_from=0, t_to=5e9, min_n=30)
        outs[f.stem] = dict(lead="L2" if m[1] else "L0", S=int(m[3]), cfg=m[4], win=win, full=full)
    summ = json.load(open(d0 / "summary.json"))
    sr = {r["name"]: r for r in summ["runs"]}
    diff = [(n, o["win"], sr[n]["out_win"], o["full"], sr[n]["out_full"]) for n, o in outs.items()
            if n not in sr or o["win"] != sr[n]["out_win"] or o["full"] != sr[n]["out_full"]]
    tab = Counter()
    for o in outs.values():
        for lab in ("win", "full"):
            tab[(o["lead"], o["S"], o["cfg"], lab)] += o[lab] != "ok"
    mism = []
    for key, v in summ["cells"].items():
        lead, S, c = key.split("|")
        for lab in ("win", "full"):
            mine = tab[(lead, int(S[1:]), c, lab)]
            if mine != v[f"fail_{lab}"]:
                mism.append((key, lab, mine, v[f"fail_{lab}"]))
    print(json.dumps(dict(n_runs_check=len(outs), n_runs_summary=len(sr), run_label_differences=diff,
                          table_mismatches=mism,
                          table={f"{k[0]}|S{k[1]}|{k[2]}|{k[3]}": v for k, v in sorted(tab.items())}), indent=1))


if __name__ == "__main__":
    main()
