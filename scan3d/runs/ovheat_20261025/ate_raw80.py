# ATE + lag of bridge runs on raw80 (hs_20261021_213341, first 80 s; same method as the phone replay study's
# scripts/analyze.py: camera position T_world_cam0 vs board GT imu_gt_spl_cal1022, pose t_ns + align dt = GT time,
# nearest GT < 2 ms, SE3 Umeyama RMSE; Sim3 scale) + feed lag from the --timing-log (rx_boot_ns - t_ns, minus its min).
#   python3 ate_raw80.py [--speed S] RUN_PREFIX [...]   (RUN_PREFIX.jsonl, optional RUN_PREFIX.tl.csv; S = feeder speed)
import csv
import json
import sys
from pathlib import Path

import numpy as np

REC = Path("/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261021_213341")


def umeyama(src, dst, scale=False):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    X, Y = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(Y.T @ X / len(src))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    c = np.trace(np.diag(S) @ D) / X.var(0).sum() if scale else 1.0
    return R, c, mu_d - c * R @ mu_s


g = dict(np.load(REC / "imu_gt_spl_cal1022" / "gt.npz", allow_pickle=True))
ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
DT = int(json.load(open(REC / "imu_gt" / "align.json"))["dt_ns"])
GT_T, GT_P = g["t_ns"][ok], g["p_bc"][ok]

args = sys.argv[1:]
SPEED = 1.0
if args and args[0] == "--speed":
    SPEED = float(args[1])
    args = args[2:]
for pre in args:
    P = [json.loads(x) for x in open(pre + ".jsonl") if '"pose"' in x]
    tr = [m for m in P if m.get("status") == "tracking" and m.get("T_world_cam0")]
    vt = np.array([m["t_ns"] for m in tr]) + DT
    X = np.array([np.asarray(m["T_world_cam0"]).reshape(4, 4)[:3, 3] for m in tr])
    j = np.clip(np.searchsorted(GT_T, vt), 1, len(GT_T) - 1)
    j = np.where(np.abs(GT_T[j - 1] - vt) < np.abs(GT_T[j] - vt), j - 1, j)
    m = np.abs(GT_T[j] - vt) < 2_000_000
    A, B = X[m], GT_P[j[m]]
    R, _, t = umeyama(A, B)
    e = np.linalg.norm(A @ R.T + t - B, axis=1)
    Rs, c, ts = umeyama(A, B, True)
    out = dict(run=pre, poses=len(P), tracked=len(tr), matched=int(m.sum()), ate_cm=round(float(np.sqrt((e ** 2).mean())) * 100, 3),
               max_err_cm=round(float(e.max()) * 100, 2), scale=round(float(c), 4),
               late_dropped=P[-1].get("late_dropped") if P else None)
    tl = Path(pre + ".tl.csv")
    if tl.exists():
        rows = list(csv.DictReader(open(tl)))
        tn = np.array([int(r["t_ns"]) for r in rows], dtype=np.int64)
        rx = np.array([int(r["rx_boot_ns"]) for r in rows], dtype=np.int64)
        fe = np.array([float(r["feed_ms"]) + float(r["remap_ms"]) + float(r["wait_ms"]) for r in rows])
        lag = ((rx - rx[0]) - (tn - tn[0]) / SPEED) / 1e6   # receive delay vs the paced schedule (speed factor)
        lag = lag - lag.min() + fe
        out.update(frames_fed=len(rows), lag_p50_ms=round(float(np.percentile(lag, 50)), 1),
                   lag_p95_ms=round(float(np.percentile(lag, 95)), 1), lag_max_ms=round(float(lag.max()), 1),
                   proc_p50_ms=round(float(np.percentile(fe, 50)), 2), proc_p95_ms=round(float(np.percentile(fe, 95)), 2))
    print(json.dumps(out))
