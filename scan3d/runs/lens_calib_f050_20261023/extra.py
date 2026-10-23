"""Extra checks on results/calib_all.json:
1. How far the fitted models disagree OUTSIDE the covered image area (ray mapping difference vs k1_prior, px),
   on a 33x19 grid over the whole image and on the image corners.
2. Fixed lenses scored on all views (per-view PnP, lens fixed): 2.22 node scaled by the focus law (HS) and by the
   law + std60 -0.39 % FOV, vs the fitted models.
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/extra.py → results/extra.json
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calib  # noqa: E402

W, H = 1920, 1080


def mapping_diff(KA, dA, KB, dB, pts):
    rays = cv2.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float64), KA, dA)
    xyz = np.c_[rays.reshape(-1, 2), np.ones(len(pts))]
    p, _ = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), KB, dB)
    return np.linalg.norm(p.reshape(-1, 2) - pts, axis=1)


def main():
    R = json.load(open(HERE / "results" / "calib_all.json"))
    gx, gy = np.meshgrid(np.linspace(0, W - 1, 33), np.linspace(0, H - 1, 19))
    grid = np.c_[gx.ravel(), gy.ravel()]
    corners = np.array([[0, 0], [W - 1, 0], [0, H - 1], [W - 1, H - 1]], float)
    views, clusters, members = calib.build()
    covered = np.concatenate([v["img"].reshape(-1, 2) for v in views])
    lenses = {m: (np.array(R["models"][m]["K"]), np.array(R["models"][m]["dist"])) for m in R["models"]}
    K22, d22 = calib.ref_lens()
    s_hs = 1 + 0.008 * (0.5 - 2.22)
    for nm, s in [("ref222_law_hs", s_hs), ("ref222_law_std60", s_hs * (1 - 0.0039))]:
        K = K22.copy()
        K[0, 0] *= s
        K[1, 1] *= s
        lenses[nm] = (K, d22.copy())
    out = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), mapping_vs_k1_prior={}, fixed_lens_scores={})
    KB, dB = lenses["k1_prior"]
    for m, (K, d) in lenses.items():
        g = mapping_diff(KB, dB, K, d, grid)
        c = mapping_diff(KB, dB, K, d, corners)
        cv_ = mapping_diff(KB, dB, K, d, covered)
        out["mapping_vs_k1_prior"][m] = dict(covered_p50=float(np.median(cv_)), covered_max=float(cv_.max()),
                                             grid_max=float(g.max()), corners=[round(float(x), 1) for x in c])
    for m, (K, d) in lenses.items():
        e = [calib.view_err(v, K, d)[0] for v in views]
        out["fixed_lens_scores"][m] = dict(fx=float(K[0, 0]), median=float(np.median(e)), mean=float(np.mean(e)))
    for k, v in out.items():
        print(k, json.dumps(v, indent=0) if isinstance(v, dict) else v)
    json.dump(out, open(HERE / "results" / "extra.json", "w"), indent=1)


if __name__ == "__main__":
    main()
