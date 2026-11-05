#!/usr/bin/env python3
"""gsplat 2026-11-06 (post-hoc control, added after the results): fuse the dumped FFS keyframe depths with the SAME fusion
settings as the 2DGS mesh (tsdf_map 5 mm, sigma_max 0.08, sigma_ok 0.02, min_obs 2, per-pixel sigma fixed 0.01 m) to separate
"fusion settings" from "depth source". Only the 93 keyframes that have FFS depth exist.
  venv/bin/python scan3d/gsplat/tsdf_ctrl.py DUMP --out OUT"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--out", required=True)
    ap.add_argument("--sigma", type=float, default=0.01)
    ap.add_argument("--depth-dir", default="", help="fuse these depth npz instead of the FFS ones (same keyframes)")
    a = ap.parse_args()
    import torch
    import tsdf_map
    t0 = time.time()
    d, out = Path(a.dump), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = np.load(d / "cams.npz"); kf, T, Kr = cams["kf"], cams["T"], cams["Kr"]
    tsdf = tsdf_map.TSDFMap(voxel_m=float(cams["voxel"]), z_max=float(cams["zmax"]), sigma_max=0.08, sigma_ok=0.02, min_obs=2)
    n = 0
    for c, i in enumerate(kf):
        p = d / "ffs" / f"{i:05d}.npz"
        if not p.exists():
            continue
        if a.depth_dir:                      # e.g. GS rendered depth, restricted to the same 93 FFS keyframes
            p = Path(a.depth_dir) / f"{i:05d}.npz"
        Z = np.load(p)["Z"].astype(np.float32); V = np.isfinite(Z)
        tsdf.integrate(Z, V, cv2.imread(str(d / "img" / f"{i:05d}.png")), float(Kr[0, 0]), float(Kr[0, 2]), float(Kr[1, 2]), 0.08, T[c],
                       sigma=np.where(V, a.sigma, np.inf).astype(np.float32))
        n += 1
    tsdf.update_mesh(force=True)
    Vm, Cm, Fm = tsdf.mesh_arrays()
    tsdf_map.write_mesh_ply(out / "mesh.ply", Vm, Cm, Fm)
    s = dict(made=time.strftime("%F %T"), dump=str(d), depth_dir=a.depth_dir or "ffs", sigma=a.sigma, n_kf_fused=n, mesh_vertices=int(len(Vm)),
             runtime_s=round(time.time() - t0, 1), gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2))
    (out / "summary.json").write_text(json.dumps(s, indent=1)); print(s)


if __name__ == "__main__":
    import gpu_guard
    o = sys.argv[sys.argv.index("--out") + 1]
    with gpu_guard.job("tsdf_ctrl " + o, out_dir=o, torch_cap=True):
        main()
