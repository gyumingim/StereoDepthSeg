#!/usr/bin/env python3
"""Independent checker for Part 2 (docs/imu_calib_gain_p2_plan.md). Does not import calib_gain*.py.
 (1) corrected imu.csv: every non-type-16 line byte-identical; type-16 t and v3..5 identical; v0..2 difference equals an
     independently recomputed causal per-still bias (own CSV parse, own trim/mean/assignment; still_runs shared) within 1e-9
 (2) ATE from poses with own SE3 Umeyama on poses matched to imu_gt_spl within 2 ms; compare with bench ate_cm; pairwise
     direction U vs D
 (3) yaw drift by a different formula (ZYX yaw angle atan2 of M M0^T, unwrapped, slope) vs p2_summary.json
 (4) verdict recomputed.
  ../venv_ffs/bin/python imu_learn/calib_gain_p2_check.py   -> runs/imu_calib_gain/p2_check.json
"""
import gzip
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
S = HERE.parent
sys.path.insert(0, str(HERE))
from i3_confirm import still_runs  # noqa: E402

RECS = ["hs_20261021_213341", "hs_20261021_214550", "hs_20261021_215348"]


def check_csv(rec):
    a = open(S / "recordings" / rec / "imu.csv").read().splitlines()
    b = open(S / "runs/imu_calib_gain/recs" / rec / "imu.csv").read().splitlines()
    assert len(a) == len(b), "line count"
    t, v, dv, other_bad = [], [], [], 0
    for la, lb in zip(a, b):
        if not la.startswith("16,"):
            other_bad += la != lb
            continue
        pa, pb = la.split(","), lb.split(",")
        try:
            ta = int(pa[1])
            va = [float(x) for x in pa[2:5]]
            vb = [float(x) for x in pb[2:5]]
        except (ValueError, IndexError):
            other_bad += la != lb
            continue
        other_bad += (pa[:2] + pa[5:]) != (pb[:2] + pb[5:])
        t.append(ta)
        v.append(va)
        dv.append(np.subtract(va, vb))
    t, v, dv = np.array(t, np.int64), np.array(v), np.array(dv)
    o = np.argsort(t, kind="stable")
    ts, vs = t[o], v[o]
    keep = np.r_[True, np.diff(ts) > 0]
    ts, vs = ts[keep], vs[keep]
    runs = still_runs(ts, vs)
    means = []
    for s, e in runs:
        s2 = s + 2_000_000_000 if e - s >= 4_000_000_000 else s
        means.append(vs[(ts >= s2) & (ts <= e)].mean(0))
    exp = np.empty((len(t), 3))
    for n, tt in enumerate(t):
        k = 0
        for q, (_, e) in enumerate(runs):
            if e <= tt:
                k = q
        exp[n] = means[k]
    err = float(np.abs(dv - exp).max())
    return dict(lines=len(a), other_lines_changed=int(other_bad), max_bias_err=err, ok=other_bad == 0 and err < 1e-9)


def load(prefix):
    d = json.load(open(str(prefix) + ".json"))
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt")]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    return d, tr


def own_metrics(prefix, rec):
    d, tr = load(prefix)
    if len(tr) < 50:
        return None
    g = np.load(S / "recordings" / rec / "imu_gt_spl/gt.npz")
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt, gp, gR = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]
    dt = int(round(d["align_dt_ms"] * 1e6))
    vt = np.array([m["t_ns"] for m in tr], np.int64) + dt
    T = np.array([np.asarray(m["T_world_cam0"], float).reshape(4, 4) for m in tr])
    idx = np.array([int(np.argmin(np.abs(gt - x))) for x in vt])
    m = np.abs(gt[idx] - vt) < 2_000_000
    P, Q = T[m, :3, 3], gp[idx[m]]
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((Q - mq).T @ (P - mp))
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt
    e = Q - ((P - mp) @ R.T + mq)
    ate = float(np.sqrt((e ** 2).sum(1).mean())) * 100
    M = T[m, :3, :3] @ np.transpose(gR[idx[m]], (0, 2, 1))
    D = M @ M[0].T
    yaw = np.degrees(np.unwrap(np.arctan2(D[:, 1, 0], D[:, 0, 0])))
    tm = (vt[m] - vt[m][0]) / 60e9
    A = np.vstack([tm, np.ones_like(tm)]).T
    slope = np.linalg.lstsq(A, yaw, rcond=None)[0][0]
    return dict(ate=ate, bench_ate=d.get("ate_cm"), yaw_dpm=abs(float(slope)))


def main():
    out = dict(made=time.strftime("%F %T"), csv={}, runs={})
    for rec in RECS:
        out["csv"][rec] = check_csv(rec)
        print(rec, out["csv"][rec], flush=True)
    summ = json.load(open(S / "runs/imu_calib_gain/p2_summary.json"))
    sy = {p["rec"] + str(p["k"]) + c: p[c].get("yaw_dpm") for p in summ["pairs"] for c in ("U", "D")}
    ate_ok, yaw_ok, pairs = True, True, []
    for rec in RECS:
        for k in range(3):
            r = {}
            for c in ("U", "D"):
                pre = S / "runs/imu_calib_gain/p2" / f"{rec[-6:]}_{c}_o{k}"
                if not Path(str(pre) + ".json").exists():
                    continue
                mm = own_metrics(pre, rec)
                if mm is None:
                    continue
                mm["summary_yaw"] = sy.get(rec + str(k) + c)
                mm["ate_match"] = mm["bench_ate"] is not None and abs(mm["ate"] - mm["bench_ate"]) <= max(0.05, 0.02 * mm["bench_ate"])
                mm["yaw_match"] = mm["summary_yaw"] is not None and abs(mm["yaw_dpm"] - mm["summary_yaw"]) <= max(0.1, 0.15 * mm["summary_yaw"])
                ate_ok &= mm["ate_match"]
                yaw_ok &= mm["yaw_match"]
                r[c] = mm
            out["runs"][f"{rec[-6:]}_o{k}"] = r
            if "U" in r and "D" in r:
                pairs.append((r["U"]["ate"], r["D"]["ate"]))
            print(rec[-6:], k, {c: {kk: (round(v, 3) if isinstance(v, float) else v) for kk, v in x.items()} for c, x in r.items()}, flush=True)
    wins = sum(d < u for u, d in pairs)
    losses = sum(d > u for u, d in pairs)
    mu, md = (float(np.median([u for u, _ in pairs])), float(np.median([d for _, d in pairs]))) if pairs else (None, None)
    verdict = ("D better" if len(pairs) == 9 and wins >= 6 and md <= 0.8 * mu else
               "D worse" if len(pairs) == 9 and losses >= 6 and mu <= 0.8 * md else "no clear difference")
    out.update(own_wins=wins, own_losses=losses, own_median_U=mu, own_median_D=md, own_verdict=verdict,
               summary_verdict=summ["verdict"], ate_all_match=bool(ate_ok), yaw_all_match=bool(yaw_ok),
               csv_all_ok=all(v["ok"] for v in out["csv"].values()))
    out["PASS"] = bool(out["csv_all_ok"] and ate_ok and yaw_ok and verdict == summ["verdict"])
    json.dump(out, open(S / "runs/imu_calib_gain/p2_check.json", "w"), indent=1)
    print({k: v for k, v in out.items() if k not in ("csv", "runs")})


if __name__ == "__main__":
    main()
