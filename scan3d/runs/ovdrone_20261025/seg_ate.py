#!/usr/bin/env python3
"""Per-segment accuracy vs rotation rate for the rolling-shutter option (2026-10-25, ovdrone).

Input = ate_run.py outputs (OUT/<cond>/<rec6>_o<off>_<cond>.poses.jsonl.gz) of the 42 HS cells (213955 excluded).
Segments = 2.0 s windows, step 0.5 s, over the GT-matched part of each run. Segment error = SE3 (Umeyama, no scale) RMSE
of the camera position (T_world_cam0) vs the board GT camera position (imu_gt_spl_cal1022 p_bc, pose t_ns + align dt,
nearest GT within 2 ms) using the GT samples matched in BOTH conditions (paired). Rotation rate of a segment = mean |gyro|
(imu.csv type 4, device gyro) over it. Segments where either condition's error > 10 cm (divergence) are dropped and
counted. Reported: paired difference (cond - ref) by rotation-rate bin, and the fastest 3 segments (non-overlapping)
per recording x offset.

  python3 seg_ate.py REF COND [COND...]  -> OUT/seg_<conds>.json + .csv, table on stdout
"""
import csv
import gzip
import json
import os
import sys
from pathlib import Path

import numpy as np

MAIN = Path("/home/system/scanner/StereoDepthSeg/scan3d")
OUT = Path(os.environ.get("OVOPT_ATE_OUT", "/tmp/ovdrone/ate"))
sys.path.insert(0, str(MAIN / "imu_learn"))
from phone60_run import RAW, OLD, DROP  # noqa: E402

WIN = float(os.environ.get("SEG_WIN", "2.0"))
STEP, MINM, DIV = 0.5, 30, 0.10


def umeyama_rmse(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    X, Y = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(Y.T @ X / len(src))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    e = (R @ X.T).T - Y
    return float(np.sqrt((e ** 2).sum(1).mean()))


def load_gt(rec):
    R = MAIN / "recordings" / rec
    g = np.load(R / "imu_gt_spl_cal1022" / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    dt = int(json.load(open(R / "imu_gt" / "align.json"))["dt_ns"])
    return g["t_ns"][ok].astype(np.int64), g["p_bc"][ok], dt


def load_gyro(rec):
    t, w = [], []
    with open(MAIN / "recordings" / rec / "imu.csv") as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            if row[0] == "4":
                t.append(int(row[1]))
                w.append([float(row[2]), float(row[3]), float(row[4])])
    return np.array(t, np.int64), np.linalg.norm(np.array(w), axis=1)


def matched(cond, rec, off, gt_t, gt_p, dt):
    f = OUT / cond / f"{rec[-6:]}_o{off}_{cond}.poses.jsonl.gz"
    if not f.exists():
        return None
    m = {}
    for line in gzip.open(f, "rt"):
        if not line.strip():
            continue
        p = json.loads(line)
        if p.get("type") != "pose" or p.get("status") != "tracking" or not p.get("T_world_cam0"):
            continue
        tg = int(p["t_ns"]) + dt
        i = np.searchsorted(gt_t, tg)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(gt_t) and abs(int(gt_t[j]) - tg) < 2_000_000 and (best is None or abs(int(gt_t[j]) - tg) < abs(int(gt_t[best]) - tg)):
                best = j
        if best is not None:
            T = p["T_world_cam0"]
            m[int(gt_t[best])] = np.array([T[3], T[7], T[11]], float)
    return m


def main():
    ref, conds = sys.argv[1], sys.argv[2:]
    rows, dropped = [], {c: 0 for c in conds}
    for rec in RAW + OLD:
        if rec == DROP:
            continue
        gt_t, gt_p, dt = load_gt(rec)
        gp = dict(zip(gt_t.tolist(), gt_p))
        gy_t, gy_w = load_gyro(rec)
        for off in (0, 1, 2):
            mr = matched(ref, rec, off, gt_t, gt_p, dt)
            if mr is None:
                continue
            mc = {c: matched(c, rec, off, gt_t, gt_p, dt) for c in conds}
            ts = np.array(sorted(mr), np.int64)
            if len(ts) < MINM:
                continue
            t0 = ts[0]
            while t0 + WIN * 1e9 <= ts[-1]:
                sel = ts[(ts >= t0) & (ts < t0 + WIN * 1e9)]
                gw = gy_w[(gy_t >= t0 - dt) & (gy_t < t0 - dt + WIN * 1e9)]
                row = dict(rec=rec, off=off, t0_s=round((t0 - ts[0]) / 1e9, 2), gyro=float(gw.mean()) if len(gw) else float("nan"))
                okrow = True
                for c in conds:
                    if mc[c] is None:
                        okrow = False
                        break
                    s2 = [t for t in sel.tolist() if t in mc[c]]
                    if len(s2) < MINM:
                        okrow = False
                        break
                    G = np.array([gp[t] for t in s2])
                    er = umeyama_rmse(np.array([mr[t] for t in s2]), G)
                    ec = umeyama_rmse(np.array([mc[c][t] for t in s2]), G)
                    if er > DIV or ec > DIV:
                        dropped[c] += 1
                        okrow = False
                        break
                    row["ref"] = er
                    row[c] = ec
                    row["n"] = len(s2)
                if okrow:
                    rows.append(row)
                t0 += int(STEP * 1e9)
    g = np.array([r["gyro"] for r in rows])
    qs = np.quantile(g, [0.5, 0.9, 0.95])
    bins = [("all", lambda x: True), ("gyro < p50", lambda x: x < qs[0]), ("p50-p90", lambda x: qs[0] <= x < qs[1]),
            ("p90-p95", lambda x: qs[1] <= x < qs[2]), ("top 5 %", lambda x: x >= qs[2])]
    summ = dict(made=__import__("time").strftime("%F %T"), ref=ref, conds=conds, out=str(OUT), win_s=WIN, step_s=STEP,
                n_segments=len(rows), dropped_divergent=dropped, gyro_quantiles_rad_s=[round(float(q), 3) for q in qs], bins={})
    print(f"segments {len(rows)} (2 s, step 0.5 s, overlapping), dropped (>10 cm) {dropped}; gyro p50/p90/p95 {qs.round(3)} rad/s")
    for c in conds:
        for name, f in bins:
            rr = [r for r in rows if f(r["gyro"])]
            if not rr:
                continue
            d = np.array([r[c] - r["ref"] for r in rr]) * 100
            e_ref = np.array([r["ref"] for r in rr]) * 100
            s = dict(n=len(rr), gyro_mean=round(float(np.mean([r["gyro"] for r in rr])), 3),
                     ref_err_cm_median=round(float(np.median(e_ref)), 4),
                     diff_cm_median=round(float(np.median(d)), 4), diff_cm_mean=round(float(d.mean()), 4),
                     rel_diff_median_pct=round(float(np.median(d / e_ref)) * 100, 2), frac_better=round(float((d < 0).mean()), 3))
            summ["bins"][f"{c} {name}"] = s
            print(f"{c:6s} {name:11s} n {s['n']:5d} gyro {s['gyro_mean']:.2f} ref {s['ref_err_cm_median']:.3f} cm  diff med {s['diff_cm_median']:+.4f} "
                  f"mean {s['diff_cm_mean']:+.4f} cm ({s['rel_diff_median_pct']:+.1f} %) better {s['frac_better']:.2f}")
    # fastest 3 non-overlapping segments per cell
    fast = []
    by = {}
    for r in rows:
        by.setdefault((r["rec"], r["off"]), []).append(r)
    for k, rr in by.items():
        rr = sorted(rr, key=lambda r: -r["gyro"])
        pick = []
        for r in rr:
            if all(abs(r["t0_s"] - p["t0_s"]) >= WIN for p in pick):
                pick.append(r)
            if len(pick) == 3:
                break
        fast += pick
    summ["fastest3_per_cell"] = {}
    for c in conds:
        d = np.array([r[c] - r["ref"] for r in fast]) * 100
        e_ref = np.array([r["ref"] for r in fast]) * 100
        summ["fastest3_per_cell"][c] = dict(n=len(fast), gyro_mean=round(float(np.mean([r["gyro"] for r in fast])), 3),
                                            ref_err_cm_median=round(float(np.median(e_ref)), 4),
                                            diff_cm_median=round(float(np.median(d)), 4), diff_cm_mean=round(float(d.mean()), 4),
                                            frac_better=round(float((d < 0).mean()), 3))
        print(f"{c:6s} fastest-3-per-cell n {len(fast)} gyro {summ['fastest3_per_cell'][c]['gyro_mean']:.2f} ref {np.median(e_ref):.3f} cm diff med "
              f"{np.median(d):+.4f} mean {d.mean():+.4f} better {(d < 0).mean():.2f}")
    tag = "_".join([ref] + conds) + f"_w{WIN:g}"
    json.dump(summ, open(OUT / f"seg_{tag}.json", "w"), indent=1)
    with open(OUT / f"seg_{tag}.csv", "w") as f:
        w = csv.DictWriter(f, fieldnames=["rec", "off", "t0_s", "gyro", "n", "ref"] + conds)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print("->", OUT / f"seg_{tag}.json")


if __name__ == "__main__":
    main()
