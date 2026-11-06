#!/usr/bin/env python3
"""mvggt: MASt3R-SfM (official sparse_global_alignment, demo defaults; README 3) on the dumped undistorted keyframes.
Saves raw outputs: cam2w (4x4), K (at the 512x288 model grid), dense depth + conf (get_dense_pts3d(clean_depth=True)).
usage: mast3r_run.py FRAMES_DIR --out raw_mast3r.npz [--win 8]
"""
import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WEIGHTS = Path.home() / "scanner" / "mvggt_env" / "weights" / "MASt3R_metric"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("--out", required=True)
    ap.add_argument("--win", type=int, default=8)
    ap.add_argument("--keep-cache", action="store_true")
    ap.add_argument("--every", type=int, default=1, help="use every k-th dumped keyframe (README 6: 356-frame GA OOM at 0.6)")
    a = ap.parse_args()
    import torch
    from mast3r.model import AsymmetricMASt3R
    from mast3r.cloud_opt.sparse_ga import sparse_global_alignment
    # scipy >= 1.18 no longer exposes scipy.cluster.hierarchy.distance (sparse_ga uses sch.distance.squareform);
    # same function, compatibility only (11-06 first 165506 run failed here after the pair forward)
    import scipy.cluster.hierarchy as _sch, scipy.spatial.distance as _ssd
    if not hasattr(_sch, "distance"):
        _sch.distance = _ssd
    from dust3r.utils.image import load_images
    from dust3r.image_pairs import make_pairs
    t0 = time.time()
    fr = np.load(Path(a.frames) / "frames.npz")
    tn = fr["t_ns"][::a.every]; Kr = fr["K"]; W0, H0 = [int(x) for x in fr["size"]]
    paths = [str(Path(a.frames) / "img" / f"{t}.png") for t in tn]
    model = AsymmetricMASt3R.from_pretrained(str(WEIGHTS)).cuda().eval()
    imgs = load_images(paths, size=512, verbose=False)
    h, w = [int(x) for x in imgs[0]["true_shape"][0]]
    sx, sy = w / W0, h / H0
    assert abs(sx - sy) < 1e-6, (w, h, W0, H0)     # 960x540 -> 512x288, same ratio, no crop
    Km = Kr.copy(); Km[0] *= sx; Km[1] *= sy
    pairs = make_pairs(imgs, scene_graph=f"swin-{a.win}-noncyclic", prefilter=None, symmetrize=True)
    cache = Path(a.out).parent / "mast3r_cache"
    cache.mkdir(parents=True, exist_ok=True)
    init = {p: dict(intrinsics=torch.tensor(Km, dtype=torch.float32, device="cuda")) for p in paths}
    t1 = time.time()
    scene = sparse_global_alignment(paths, pairs, str(cache), model, lr1=0.07, niter1=300, lr2=0.01, niter2=300, device="cuda",
                                    opt_depth=True, shared_intrinsics=True, matching_conf_thr=0.0, init=init, opt_pp=False)
    t_ga = time.time() - t1
    pts3d, depthmaps, confs = scene.get_dense_pts3d(clean_depth=True)
    cam2w = scene.get_im_poses().detach().float().cpu().numpy()
    K = np.stack([k.detach().float().cpu().numpy() for k in scene.intrinsics])
    D = np.stack([d.detach().float().cpu().numpy().reshape(h, w) for d in depthmaps]).astype(np.float32)
    C = np.stack([c.detach().float().cpu().numpy().reshape(h, w) for c in confs]).astype(np.float32)
    meta = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), frames=a.frames, weights=str(WEIGHTS), n_frames=len(paths), every=a.every,
                n_pairs=len(pairs), scene_graph=f"swin-{a.win}-noncyclic", model_hw=[h, w], K_init=Km.tolist(),
                focal_final_median=float(np.median(K[:, 0, 0])), focal_init=float(Km[0, 0]),
                ga_s=round(t_ga, 1), runtime_s=round(time.time() - t0, 1),
                gpu_peak_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                gpu_peak_reserved_gb=round(torch.cuda.max_memory_reserved() / 2 ** 30, 2), torch=torch.__version__)
    np.savez(a.out, cam2w=cam2w, K=K, depth=D, conf=C, t_ns=tn, meta=json.dumps(meta))
    Path(a.out).with_suffix(".json").write_text(json.dumps(meta, indent=1))
    if not a.keep_cache:
        shutil.rmtree(cache, ignore_errors=True)
    print("DONE", json.dumps(meta))


if __name__ == "__main__":
    sys.path.insert(0, str(HERE.parent / "scanA"))
    import gpu_guard
    od = str(Path(sys.argv[sys.argv.index("--out") + 1]).parent)
    with gpu_guard.job("mvggt mast3r " + od, out_dir=od, torch_cap=True):
        main()
