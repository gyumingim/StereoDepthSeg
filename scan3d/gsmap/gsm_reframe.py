#!/usr/bin/env python3
"""gsmap 2026-11-25: move a dump made against a lensonly stand-in GT (= raw VIO poses + 1000 m in z, NOT a real GT) back to the
VIO world and turn it from z-up to the viewer convention y-down: X_new = Rw (X_vio - [0, 0, 1000]), Rw = [[1,0,0],[0,0,-1],[0,1,0]].
Keeps the original as cams_lensonly.npz.  python gsm_reframe.py DUMP_DIR"""
import sys, shutil
from pathlib import Path
import numpy as np
d = Path(sys.argv[1]); src = d / "cams_lensonly.npz"
if not src.exists():
    shutil.copy(d / "cams.npz", src)
c = dict(np.load(src))
W = np.eye(4); W[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]; S = np.eye(4); S[2, 3] = -1000.0
for k in ("T", "Tgt"):
    c[k] = np.einsum("ij,njk->nik", W @ S, c[k])
c["reframe"] = np.array("X_new = Rw (X_lensonly - [0,0,1000]); Rw = [[1,0,0],[0,0,-1],[0,1,0]] (VIO z-up -> y-down)")
np.savez(d / "cams.npz", **c)
T = c["T"]; print("cam pos range", T[:, :3, 3].min(0).round(2), T[:, :3, 3].max(0).round(2), "cam image-down axis mean", T[:, :3, 1].mean(0).round(2))
