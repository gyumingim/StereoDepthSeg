#!/usr/bin/env python3
"""scanplan: quick PNG of TSDF meshes (board frame: x right, y down on the board, z away from camera). Usage: render_mesh.py out.png title1=mesh1.ply ..."""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tsdf_map

out = sys.argv[1]
items = [a.split("=", 1) for a in sys.argv[2:]]
fig, ax = plt.subplots(2, len(items), figsize=(5.2 * len(items), 8.4), squeeze=False)
for k, (title, p) in enumerate(items):
    V, C, F = tsdf_map.read_mesh_ply(p)
    V = np.asarray(V); C = np.asarray(C)[:, :3] / 255.0
    o = np.argsort(V[:, 2])[::-1]          # far first, near last (camera looks +z)
    ax[0, k].scatter(V[o, 0], V[o, 1], c=C[o], s=1.2, lw=0)
    ax[0, k].add_patch(plt.Rectangle((0, 0), 0.302, 0.1812, fill=False, ec="red", lw=1.2))
    ax[0, k].set_aspect("equal"); ax[0, k].invert_yaxis(); ax[0, k].set_title(f"{title}: front (m)")
    sel = (V[:, 1] > 0.02) & (V[:, 1] < 0.16)
    ax[1, k].scatter(V[sel, 0], V[sel, 2] * 100, c=C[sel], s=2, lw=0)
    ax[1, k].axhline(0, color="red", lw=0.8); ax[1, k].set_ylim(-6, 6); ax[1, k].set_xlim(-0.1, 0.4)
    ax[1, k].set_title(f"{title}: board band side view, z (cm)"); ax[1, k].set_xlabel("x (m)")
fig.tight_layout(); fig.savefig(out, dpi=90)
print("wrote", out)
