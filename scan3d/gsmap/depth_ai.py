#!/usr/bin/env python3
"""gsmap 2026-11-25: AI depth per keyframe for the gsplat dump (960x540 undistorted keyframes + BA poses).

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/depth_ai.py DUMP --model da3|moge --out OUTDIR

da3  = Depth Anything 3 (DA3-LARGE-1.1), multi-view, pose-conditioned: our BA poses (w2c) + Kr are given;
       DA3 aligns its output depth to the scale of the input poses (align_to_input_ext_scale=True). Views are run in
       overlapping chunks (--chunk, default all at once if it fits), process_res 504 -> depth resized to 960x540.
moge = MoGe-2 (moge-2-vitl-normal), single image, fov_x from Kr, metric depth output.
Output: OUTDIR/<kf>.npz with Z (float32 m, NaN = invalid), C (float16 confidence or mask) + OUTDIR/meta.json.
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "gsplat"))


def run_da3(imgs, T, Kr, a):
    import torch
    from depth_anything_3.api import DepthAnything3
    m = DepthAnything3.from_pretrained("depth-anything/DA3-LARGE-1.1").to("cuda").eval()
    n = len(imgs)
    ext = np.linalg.inv(T).astype(np.float32)
    ixt = np.repeat(Kr[None].astype(np.float32), n, 0)
    Zs, Cs = [None] * n, [None] * n
    chunk = a.chunk if a.chunk > 0 else n
    step = max(1, chunk - a.overlap)
    starts = list(range(0, max(1, n - a.overlap), step)) if chunk < n else [0]
    for s in starts:
        ids = list(range(s, min(n, s + chunk)))
        t = time.time()
        with torch.no_grad():
            p = m.inference([imgs[i][:, :, ::-1].copy() for i in ids], extrinsics=ext[ids], intrinsics=ixt[ids],
                            align_to_input_ext_scale=True, process_res=a.res)
        print(f"da3 chunk {ids[0]}..{ids[-1]} ({len(ids)} views) {time.time() - t:.1f}s depth {p.depth.shape} "
              f"gpu {torch.cuda.max_memory_allocated() / 2**30:.2f} GB", flush=True)
        for j, i in enumerate(ids):
            if Zs[i] is not None and j < a.overlap // 2:   # keep the earlier chunk's value for the first half of the overlap
                continue
            Zs[i] = p.depth[j].astype(np.float32)
            Cs[i] = p.conf[j].astype(np.float32) if p.conf is not None else np.ones_like(Zs[i])
    return Zs, Cs, dict(model="depth-anything/DA3-LARGE-1.1", process_res=a.res, chunk=chunk, overlap=a.overlap,
                        pose_conditioned=True, align_to_input_ext_scale=True)


def run_moge(imgs, T, Kr, a):
    import torch
    from moge.model.v2 import MoGeModel
    m = MoGeModel.from_pretrained("Ruicheng/moge-2-vitl-normal").to("cuda").eval()
    W = imgs[0].shape[1]
    fov_x = float(np.degrees(2 * np.arctan(W / 2 / Kr[0, 0])))
    Zs, Cs = [], []; NORMALS = {}
    for k, im in enumerate(imgs):
        x = torch.from_numpy(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)).float().div(255).permute(2, 0, 1).cuda()
        with torch.no_grad():
            o = m.infer(x, fov_x=fov_x)
        Z = o["depth"].float().cpu().numpy(); M = o["mask"].cpu().numpy().astype(bool)
        Zs.append(np.where(M & np.isfinite(Z), Z, np.nan).astype(np.float32)); Cs.append(M.astype(np.float32))
        if "normal" in o:   # camera-frame normals (MoGe: x right, y down, z forward), for the E7 normal prior
            NORMALS[k] = o["normal"].float().cpu().numpy().astype(np.float16)
        if k % 20 == 0:
            print(f"moge {k}/{len(imgs)} gpu {torch.cuda.max_memory_allocated() / 2**30:.2f} GB", flush=True)
    globals()["NORMALS_OUT"] = NORMALS
    return Zs, Cs, dict(model="Ruicheng/moge-2-vitl-normal", fov_x_deg=fov_x, metric=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--model", required=True, choices=["da3", "moge"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--res", type=int, default=504)
    ap.add_argument("--chunk", type=int, default=0); ap.add_argument("--overlap", type=int, default=8)
    a = ap.parse_args()
    import torch
    t0 = time.time()
    dump, out = Path(a.dump), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = np.load(dump / "cams.npz")
    kf, T, Kr = cams["kf"], cams["T"], cams["Kr"].astype(np.float64)
    W, H = [int(v) for v in cams["size"]]
    imgs = [cv2.imread(str(dump / "img" / f"{i:05d}.png")) for i in kf]
    Zs, Cs, meta = (run_da3 if a.model == "da3" else run_moge)(imgs, T, Kr, a)
    for i, Z, C in zip(kf, Zs, Cs):
        if Z.shape != (H, W):
            Z = cv2.resize(Z, (W, H), interpolation=cv2.INTER_NEAREST); C = cv2.resize(C, (W, H), interpolation=cv2.INTER_NEAREST)
        extra = {}
        if "NORMALS_OUT" in globals() and list(kf).index(i) in NORMALS_OUT:
            extra["N"] = NORMALS_OUT[list(kf).index(i)]
        np.savez_compressed(out / f"{i:05d}.npz", Z=Z.astype(np.float32), C=C.astype(np.float16), **extra)
    meta.update(made=time.strftime("%F %T"), dump=str(dump), n_kf=int(len(kf)), runtime_s=round(time.time() - t0, 1),
                gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2))
    (out / "meta.json").write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta), flush=True)


if __name__ == "__main__":
    import gpu_guard
    o = sys.argv[sys.argv.index("--out") + 1]
    with gpu_guard.job("gsmap depth_ai " + o, out_dir=o, torch_cap=True):
        main()
