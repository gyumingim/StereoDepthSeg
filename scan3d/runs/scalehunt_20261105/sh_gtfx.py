#!/usr/bin/env python3
"""scalehunt GT-side sensitivity (README 3): 165506 GT re-solved with fx,fy x (1+e).

Board inner corners (OpenCV ChArUco convention, 5x3 squares, 60.4 mm) are projected with the GT pose and the GT lens
(K, dist from gt.npz), then solvePnP (ITERATIVE, start = GT pose) with K' = K with fx, fy scaled -> new camera positions.
Writes OUT_DIR/gt_fx{e}/gt.npz (same keys used by sh_score: t_ns, ok, p_bc, K) and prints k vs B poses.
  python sh_gtfx.py REC OUT_DIR
"""
import json, sys
from pathlib import Path
import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sh_score import load_poses, rulers, ROOT  # noqa: E402

rec, outd = Path(sys.argv[1]), Path(sys.argv[2])
g = dict(np.load(rec / "imu_gt_spl" / "gt.npz", allow_pickle=True))
sq = float(g["square_mm"]) / 1000
X = np.array([[(i + 1) * sq, (j + 1) * sq, 0.0] for j in range(2) for i in range(4)])
K, dist = g["K"], g["dist"]
res = {}
vt, P = load_poses(str(ROOT / "runs/B" / "165506s.poses.jsonl.gz"))
for e in (-0.01, 0.0, 0.01):
    K2 = K.copy(); K2[0, 0] *= 1 + e; K2[1, 1] *= 1 + e
    p_new = g["p_bc"].copy()
    for i in np.nonzero(g["ok"])[0]:
        Rbc, pbc = g["R_bc"][i], g["p_bc"][i]
        Rcb = Rbc.T; tcb = -Rcb @ pbc                     # board -> camera
        rv, _ = cv2.Rodrigues(Rcb)
        uv, _ = cv2.projectPoints(X, rv, tcb, K, dist)
        ok, rv2, t2 = cv2.solvePnP(X, uv, K2, dist, rv.copy(), tcb.reshape(3, 1).copy(), True, cv2.SOLVEPNP_ITERATIVE)
        R2, _ = cv2.Rodrigues(rv2)
        p_new[i] = (-R2.T @ t2).ravel()
    d = outd / f"gt_fx{e:+.2f}"; d.mkdir(parents=True, exist_ok=True)
    np.savez(d / "gt.npz", t_ns=g["t_ns"], ok=g["ok"], p_bc=p_new, K=K2, R_bc=g["R_bc"])
    ok = g["ok"].astype(bool) & (np.linalg.norm(p_new, axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], p_new[ok]
    j = np.clip(np.searchsorted(gt_t, vt), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - vt) < np.abs(gt_t[j] - vt), j - 1, j)
    mm = np.abs(gt_t[j] - vt) < 2_000_000
    r = rulers(P[mm], gt_p[j[mm]])
    dist_ratio = float(np.median(np.linalg.norm(p_new[ok], axis=1) / np.linalg.norm(g["p_bc"][ok], axis=1)))
    res[f"{e:+.2f}"] = dict(dist_ratio=round(dist_ratio, 5), **r)
    print(e, json.dumps(res[f"{e:+.2f}"]))
json.dump(res, open(outd / "gtfx_165506.json", "w"), indent=1)
