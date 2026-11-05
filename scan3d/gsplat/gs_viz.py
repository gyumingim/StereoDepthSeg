#!/usr/bin/env python3
"""gsplat 2026-11-05: picture of the board-plane error of each mesh (vertices over the board, colour = z from the plane)
and of the mesh depth error of one keyframe. Agg backend only (no window).
  venv/bin/python scan3d/gsplat/gs_viz.py scan3d/runs/gsplat_20261105"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

O = Path(sys.argv[1])
runs = [(f"{m}_{r}", O / ("tsdf/" + r if m == "tsdf" else f"gs/{m}_{r}") / "mesh.ply", r)
        for r in ("ba", "ba_fs") for m in ("tsdf", "gsA", "gsB")]
runs = [x for x in runs if x[1].exists()]
fig, ax = plt.subplots(2, len(runs), figsize=(4.6 * len(runs), 7.5), squeeze=False)
for k, (name, p, r) in enumerate(runs):
    cams = np.load(O / "dump" / r / "cams.npz")
    BW, BH = float(cams["BW"]), float(cams["BH"])
    V = np.asarray(o3d.io.read_triangle_mesh(str(p)).vertices)
    mb = (V[:, 0] > 0.02) & (V[:, 0] < BW - 0.02) & (V[:, 1] > 0.02) & (V[:, 1] < BH - 0.02) & (np.abs(V[:, 2]) < 0.10)
    z = V[mb, 2] * 100
    s = ax[0, k].scatter(V[mb, 0] * 100, V[mb, 1] * 100, c=z, s=2, cmap="coolwarm", vmin=-1, vmax=1)
    ax[0, k].set_aspect("equal"); ax[0, k].set_title(f"{name}\nboard z med {np.median(z):+.2f}\nRMS {np.sqrt(np.mean((z - np.median(z)) ** 2)):.2f} cm", fontsize=9)
    ax[0, k].set_xlabel("x cm"); ax[0, k].set_ylabel("y cm")
    ax[1, k].hist(np.clip(z, -3, 3), bins=120, range=(-3, 3), color="C0")
    ax[1, k].set_xlabel("vertex z from board plane (cm)"); ax[1, k].set_yscale("log")
fig.colorbar(s, ax=ax[0, :].tolist(), shrink=0.7, label="z (cm)")
fig.suptitle("165506 board: mesh vertices over the board (TSDF = FFS depth, gsA = 2DGS photo only, gsB = 2DGS + FFS depth)")
fig.savefig(O / "board_mesh_compare.png", dpi=90)
print(O / "board_mesh_compare.png")
