#!/usr/bin/env python3
"""plots for plane_diag.json -> OUT/plot_<roi>.png"""
import json, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2/diag_plane_20261127"
J = json.loads((OUT / "plane_diag.json").read_text())
COL = dict(a="#9aa5b1", b="#2b6cb0", d="#d69e2e", e="#c53030", f="#b7791f", g="#9b2c2c")
LAB = dict(a="a single pair", b="b kf fused", d="d 2DGS", e="e mesh", f="d* FFS px", g="e* FFS px")
for name, R in J["rois"].items():
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    for st in "abde":
        if st not in R["stages"] or "maps" not in R["stages"][st]:
            continue
        L = R["stages"][st]["maps"]; x = [s["idx"] for s in L]; y = [s["offset_cm"] for s in L]
        ax[0].scatter(x, y, s=8 if st == "a" else 14, c=COL[st], label=LAB[st], alpha=0.6 if st == "a" else 0.9)
    ax[0].axhline(0, c="k", lw=0.5); ax[0].set_xlabel("keyframe idx"); ax[0].set_ylabel("plane offset vs global plane (cm)")
    ax[0].set_title(f"{name}: per-map plane offset"); ax[0].legend(fontsize=8)
    u0 = float(np.median(np.load(OUT / f"pooled_{name}_b.npy")[:, 0]))   # common origin for all stages
    for st in "bdeg":
        f = OUT / f"pooled_{name}_{st}.npy"
        if not f.exists():
            continue
        P = np.load(f); u = P[:, 0] - u0; r = P[:, 2] * 100
        bins = np.arange(np.percentile(u, 1), np.percentile(u, 99), 0.02); idx = np.digitize(u, bins)
        med = [np.median(r[idx == i]) if (idx == i).sum() > 30 else np.nan for i in range(1, len(bins))]
        ax[1].plot(bins[:-1] * 100, med, c=COL[st], label=LAB[st])
    ax[1].axhline(0, c="k", lw=0.5); ax[1].set_xlabel("position u on plane (cm)"); ax[1].set_ylabel("median residual (cm), 2 cm bins")
    ax[1].set_title("pooled profile (waves)"); ax[1].legend(fontsize=8)
    sts = [s for s in "abdefg" if s in R["stages"] and "bands_combined" in R["stages"][s]["summary"]]
    w = 0.25; xs = np.arange(len(sts))
    for k, (key, lab) in enumerate([("long_gt20_cm", ">20 cm"), ("mid_5_20_cm", "5–20 cm"), ("short_lt5_cm", "<5 cm")]):
        ax[2].bar(xs + (k - 1) * w, [R["stages"][s]["summary"]["bands_combined"][key] for s in sts], w, label=lab)
    ax[2].set_xticks(xs); ax[2].set_xticklabels([LAB[s] for s in sts], fontsize=8); ax[2].set_ylabel("RMS (cm), 1 cm cells")
    ax[2].set_title("wavelength bands (pooled, global plane)"); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUT / f"plot_{name}.png", dpi=90); plt.close(fig)
print("ok")
