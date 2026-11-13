#!/usr/bin/env python3
"""fixes 11-13 unit 15: where are the vertices the wide-only guard removed? (CPU, no GT; plausibility only)
Floor plane = least-squares plane through BASE mesh vertices near the lowest dense z level (iterated, 1.5 cm band).
Sets: A = NONE vertices > 2 cm from GUARD (removed by the guard), K = GUARD vertices > 2 cm from BASE (wide-pair gain kept),
N = NONE vertices > 2 cm from BASE (wide-pair gain without guard). Per set: count, height above floor (below -3 cm = impossible
for a room = phantom sign), on-floor share (|h| < 1.5 cm), camera depth (far_eval rule) bins, small-component share.
Usage: u15_guard_where.py BASE_DIR NONE_DIR GUARD_DIR OUT_JSON"""
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import far_eval as fe  # noqa: E402


def load(d):
    X, C, F = fe.read_mesh(Path(d) / "mesh.ply")[:3]
    return np.asarray(X, np.float64), np.asarray(F)


def cam_depth(X, P):
    best = np.full(len(X), np.inf)
    f, cx, cy = float(P["f"]), float(P["cx"]), float(P["cy"]); W, H = (int(v) for v in P["size"])
    for T in P["T"]:
        pc = (X - T[:3, 3]) @ T[:3, :3]
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pc[:, 0] / z * f + cx; v = pc[:, 1] / z * f + cy
        inn = (z > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        best = np.where(inn, np.minimum(best, z), best)
    return best


def floor_plane(X):
    z = X[:, 2]
    h, e = np.histogram(z, bins=400)
    lo = np.argmax(h[: len(h) // 2])                  # densest level in the lower half = floor (z up)
    z0 = 0.5 * (e[lo] + e[lo + 1])
    m = np.abs(z - z0) < 0.03
    for _ in range(5):
        A = np.c_[X[m, 0], X[m, 1], np.ones(m.sum())]
        c, *_ = np.linalg.lstsq(A, X[m, 2], rcond=None)
        r = X[:, 2] - (X[:, 0] * c[0] + X[:, 1] * c[1] + c[2])
        m = np.abs(r) < 0.015
    nrm = np.sqrt(1 + c[0] ** 2 + c[1] ** 2)
    return c, float(np.degrees(np.arccos(1 / nrm))), float(np.sqrt(np.mean(r[m] ** 2)))


def comp_size(X, F):          # far_eval rule (vertices welded at 0.1 mm)
    return fe.comp_size(X, F)[0]


def main():
    bd, nd, gd, out = sys.argv[1:5]
    Xb, _ = load(bd); Xn, Fn = load(nd); Xg, Fg = load(gd)
    P = np.load(Path(bd) / "kf_poses.npz")
    c, tilt, rms = floor_plane(Xb)
    hgt = lambda X: (X[:, 2] - (X[:, 0] * c[0] + X[:, 1] * c[1] + c[2])) / np.sqrt(1 + c[0] ** 2 + c[1] ** 2)
    tb, tg = cKDTree(Xb), cKDTree(Xg)
    dn_g, _ = tg.query(Xn, workers=-1); dn_b, _ = tb.query(Xn, workers=-1); dg_b, _ = tb.query(Xg, workers=-1)
    szn, szg = comp_size(Xn, Fn), comp_size(Xg, Fg)
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), base=bd, none=nd, guard=gd,
               floor=dict(coef=[float(x) for x in c], tilt_deg=tilt, inlier_rms_mm=rms * 1000))
    for name, X, sel, sz in (("A_removed_by_guard", Xn, dn_g > 0.02, szn), ("N_gain_no_guard", Xn, dn_b > 0.02, szn),
                             ("K_gain_with_guard", Xg, dg_b > 0.02, szg)):
        Y = X[sel]; h = hgt(Y); cd = cam_depth(Y, P); s = sz[sel]
        res[name] = dict(n=int(sel.sum()), below_floor_3cm=int((h < -0.03).sum()), on_floor_1p5cm=int((np.abs(h) < 0.015).sum()),
                         above_floor_10cm=int((h > 0.10).sum()), h_median_mm=float(np.median(h) * 1000) if len(h) else None,
                         cam_lt_1p2=int((cd < 1.2).sum()), cam_1p2_1p6=int(((cd >= 1.2) & (cd < 1.6)).sum()), cam_gt_1p6=int((cd >= 1.6).sum()),
                         in_small_comp_lt5000=int((s < 5000).sum()), in_small_comp_lt500=int((s < 500).sum()))
        print(name, json.dumps(res[name]), flush=True)
    res["below_floor_3cm_all"] = dict(base=int((hgt(Xb) < -0.03).sum()), none=int((hgt(Xn) < -0.03).sum()), guard=int((hgt(Xg) < -0.03).sum()))
    print("floor", json.dumps(res["floor"]), "below", json.dumps(res["below_floor_3cm_all"]))
    Path(out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
