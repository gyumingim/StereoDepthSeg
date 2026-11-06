#!/usr/bin/env python3
"""mvggt: VGGT-1B on the dumped undistorted keyframes, consecutive chunks of C frames with O overlap (README 3).
Saves the raw outputs per chunk (no alignment here): extrinsic (w2c, OpenCV), intrinsic (at the 518x294 model grid), depth, depth_conf.
usage: vggt_run.py FRAMES_DIR --out raw_vggt.npz [--chunk 64 --overlap 8] [--probe]  (probe = first chunk only, memory test)
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WEIGHTS = Path.home() / "scanner" / "mvggt_env" / "weights" / "VGGT-1B" / "model.safetensors"


def chunks(n, C, O):
    if n <= C:
        return [(0, n)]
    out, s = [], 0
    while True:
        e = min(s + C, n)
        if e == n:
            out.append((max(0, n - C), n))
            break
        out.append((s, e))
        s = e - O
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("--out", required=True)
    ap.add_argument("--chunk", type=int, default=64)
    ap.add_argument("--overlap", type=int, default=8)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--fp32-weights", action="store_true", help="old (11-06 first probes): fp32 weights; default = bf16 aggregator weights")
    a = ap.parse_args()
    import torch
    from safetensors.torch import load_file
    from vggt.models.vggt import VGGT
    from vggt.utils.load_fn import load_and_preprocess_images
    from vggt.utils.pose_enc import pose_encoding_to_extri_intri
    t0 = time.time()
    fr = np.load(Path(a.frames) / "frames.npz")
    tn = fr["t_ns"]
    paths = [str(Path(a.frames) / "img" / f"{t}.png") for t in tn]
    model = VGGT(enable_point=False, enable_track=False)
    sd = load_file(str(WEIGHTS))
    miss, unexp = model.load_state_dict(sd, strict=False)
    assert not miss, miss[:5]
    assert all(k.startswith(("point_head.", "track_head.")) for k in unexp), [k for k in unexp if not k.startswith(("point_head.", "track_head."))][:5]
    del sd
    if not a.fp32_weights:          # README 5 (11-06 amendment): aggregator weights bf16 (heads stay fp32, run outside autocast as in the official forward)
        model.aggregator.to(torch.bfloat16)
    model = model.cuda().eval()
    dtype = torch.bfloat16
    ch = chunks(len(paths), a.chunk, a.overlap)
    if a.probe:
        ch = ch[:1]
    E, I, D, Cf, cid, fidx, stats = [], [], [], [], [], [], []
    for k, (s, e) in enumerate(ch):
        torch.cuda.reset_peak_memory_stats()
        t1 = time.time()
        imgs = load_and_preprocess_images(paths[s:e], mode="crop").cuda()
        with torch.no_grad(), torch.autocast("cuda", dtype=dtype):
            pred = model(imgs[None])
        ext, intr = pose_encoding_to_extri_intri(pred["pose_enc"], imgs.shape[-2:])
        E.append(ext[0].float().cpu().numpy()); I.append(intr[0].float().cpu().numpy())
        D.append(pred["depth"][0, ..., 0].float().cpu().numpy().astype(np.float32))
        Cf.append(pred["depth_conf"][0].float().cpu().numpy().astype(np.float32))
        cid.append(np.full(e - s, k)); fidx.append(np.arange(s, e))
        st = dict(chunk=k, s=s, e=e, n=e - s, sec=round(time.time() - t1, 2),
                  gpu_peak_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                  gpu_peak_reserved_gb=round(torch.cuda.max_memory_reserved() / 2 ** 30, 2), hw=list(imgs.shape[-2:]))
        stats.append(st)
        print(json.dumps(st), flush=True)
        del pred, imgs
    meta = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), frames=a.frames, weights=str(WEIGHTS), chunk=a.chunk, overlap=a.overlap, weights_dtype="fp32" if a.fp32_weights else "aggregator bf16",
                n_frames=len(paths), n_chunks=len(ch), chunks=ch, stats=stats, runtime_s=round(time.time() - t0, 1),
                unexpected_keys_heads=sorted({k.split(".")[0] for k in unexp}), torch=torch.__version__)
    if not a.probe:
        np.savez(a.out, extrinsic=np.concatenate(E), intrinsic=np.concatenate(I), depth=np.concatenate(D),
                 conf=np.concatenate(Cf), chunk=np.concatenate(cid), fidx=np.concatenate(fidx), t_ns=tn,
                 meta=json.dumps(meta))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).with_suffix(".json").write_text(json.dumps(meta, indent=1))
    print("DONE", json.dumps(meta)[:400])


if __name__ == "__main__":
    sys.path.insert(0, str(HERE.parent / "scanA"))
    import gpu_guard
    with gpu_guard.job("mvggt vggt " + str(Path(sys.argv[sys.argv.index("--out") + 1]).parent), out_dir=str(Path(sys.argv[sys.argv.index("--out") + 1]).parent), torch_cap=True):
        main()
