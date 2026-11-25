#!/usr/bin/env python3
"""gsmap 2026-11-25: collect numbers + media of the gsA/gsB/gsC runs into a folder (table.json, grid stills) for the HTML report.

  python scan3d/gsmap/gsm_report.py RUN_ROOT OUT_DIR
"""
import json, shutil, sys
from pathlib import Path

import cv2
import numpy as np


def main():
    R, O = Path(sys.argv[1]), Path(sys.argv[2]); O.mkdir(parents=True, exist_ok=True)
    V = [v for v in ["gsA", "gsB", "gsC", "gsD", "gsE"] if (R / v / "summary.json").exists()]
    lab = dict(gsA="A FFS only", gsB="B FFS+DA3", gsC="C FFS+MoGe-2", gsD="D FFS+DA3 checked", gsE="E FFS+DA3 no fill loss")
    tab = {}
    for v in V:
        s = json.loads((R / v / "summary.json").read_text())
        g = np.loadtxt(R / f"{v}.gpu.txt", delimiter=",", ndmin=2) if (R / f"{v}.gpu.txt").exists() else None
        tab[v] = {k: s[k] for k in ["test_psnr", "test_ssim", "test_lpips", "train_psnr", "n_init", "init_fill_share", "n_gauss",
                                     "train_s", "runtime_s", "gpu_max_alloc_gb", "gpu_max_reserved_gb", "test_depth_vs_ffs", "floaters", "made"]}
        import torch
        ck = torch.load(R / v / "gaussians.pt", weights_only=False); pp = ck["params"]
        op = torch.sigmoid(pp["opacities"]).numpy(); lum = (pp["sh0"][:, 0].numpy() * 0.28209479177387814 + 0.5).mean(1)
        tab[v]["dark_opaque"] = int(((op > 0.3) & (lum < 0.1)).sum()); tab[v]["opaque"] = int((op > 0.3).sum())
        tab[v]["ply_mb"] = round((R / v / "splat.ply").stat().st_size / 2 ** 20, 1)
        tab[v]["nvsmi_max_mb"] = float(g[:, 0].max()) if g is not None and len(g) else None
        tab[v]["per_test"] = [dict(kf=e["kf"], psnr=e["psnr"], ssim=e["ssim"], lpips=e["lpips"]) for e in s["per_view"] if e["test"]]
        for f in ["chase.mp4", "slice.png"]:
            if (R / v / f).exists():
                shutil.copy(R / v / f, O / f"{v}_{f}")
    tab["align"] = {k: {kk: vv for kk, vv in m.items() if kk != "per_kf"}
                    for k, m in json.loads((R / "fuse" / "align_stats.json").read_text())["models"].items()}
    tab["depth_ai"] = {m: json.loads((R / f"depth_{m}" / "meta.json").read_text()) for m in ["da3", "moge"]}
    (O / "table.json").write_text(json.dumps(tab, indent=1))
    # grid stills: original | A | B | C at held-out keyframes
    test = [e["kf"] for e in tab[V[0]]["per_test"]]
    pick = [test[i] for i in np.linspace(0, len(test) - 1, 4).astype(int)]
    rows = []
    for k in pick:
        t = [cv2.imread(str(R / "dump_ba" / "img" / f"{k:05d}.png"))] + [cv2.imread(str(R / v / "test" / f"{k:05d}.jpg")) for v in V]
        names = [f"original kf {k} (held out)"] + [lab[v] for v in V]
        row = []
        for im, n in zip(t, names):
            im = cv2.resize(im, (480, 270)); cv2.putText(im, n, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2); row.append(im)
        rows.append(np.hstack(row))
    rows = [cv2.resize(r, (1920, int(r.shape[0] * 1920 / r.shape[1]))) for r in rows]
    cv2.imwrite(str(O / "heldout_grid.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
    # chase frames grid (same frame index across variants)
    rows = []
    for j in [120, 400, 640]:
        row = []
        for v in V:
            cap = cv2.VideoCapture(str(R / v / "chase.mp4")); cap.set(1, j); ok, f = cap.read()
            f = cv2.resize(f if ok else np.zeros((540, 960, 3), np.uint8), (480, 270))
            cv2.putText(f, lab[v], (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2); row.append(f)
        rows.append(np.hstack(row))
    cv2.imwrite(str(O / "chase_grid.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
    shutil.copy(R / "figs" / "depth_compare.jpg", O / "depth_compare.jpg")
    print(json.dumps({v: {k: tab[v][k] for k in ["test_psnr", "test_ssim", "test_lpips", "train_s", "gpu_max_alloc_gb", "floaters", "dark_opaque",
                                                   "test_depth_vs_ffs", "n_gauss"]} for v in V}, indent=1))


if __name__ == "__main__":
    main()
