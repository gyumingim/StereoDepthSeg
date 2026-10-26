#!/usr/bin/env python3
"""ovloop 2026-10-26: vpr_desc.py on many keyframe dumps with one model load (same preprocessing / model / output).
  venv_ffs/bin/python vpr_batch.py --model boq|salad --kfroot OUT/kf --outdir OUT/vpr/boq [--fp16] [--size HxW]
Each sub-directory of --kfroot without '_' after the cell key (= B30 dumps, e.g. 133820_o0, s165506_o1, l142148_o0)
-> outdir/<dir>.ovpr. Writes outdir/batch_info.json (made time, inputs, per-dir counts, ms/img)."""
import argparse
import json
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vpr_desc as V  # noqa: E402
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import torch.nn.functional as F  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--model", required=True, choices=["boq", "salad"])
ap.add_argument("--kfroot", required=True)
ap.add_argument("--outdir", required=True)
ap.add_argument("--size", default=None)
ap.add_argument("--batch", type=int, default=8)
ap.add_argument("--fp16", action="store_true")
a = ap.parse_args()
dev = torch.device("cuda")
model, mode, size, wfile = (V.build_boq if a.model == "boq" else V.build_salad)()
model = model.eval().to(dev)
if a.size:
    size = tuple(int(v) for v in a.size.lower().split("x"))
os.makedirs(a.outdir, exist_ok=True)
dirs = sorted(d for d in os.listdir(a.kfroot) if os.path.isdir(os.path.join(a.kfroot, d)) and d.count("_") == 1)
info = dict(made=time.strftime("%F %T"), model=a.model, size=list(size), mode=mode, fp16=a.fp16, weights=wfile,
            kfroot=os.path.abspath(a.kfroot), dirs={})
gpu_t, n_all = 0.0, 0
for d in dirs:
    frames = V.list_frames(os.path.join(a.kfroot, d))
    if not frames:
        info["dirs"][d] = 0
        continue
    out = []
    for i in range(0, len(frames), a.batch):
        imgs = [cv2.imread(p, cv2.IMREAD_GRAYSCALE) for _, p in frames[i:i + a.batch]]
        torch.cuda.synchronize()
        ts = time.perf_counter()
        x = V.preprocess(imgs, size, mode, dev)
        with torch.no_grad(), torch.autocast("cuda", dtype=torch.float16, enabled=a.fp16):
            y = model(x)
        y = F.normalize(y.float(), p=2, dim=1)
        torch.cuda.synchronize()
        gpu_t += time.perf_counter() - ts
        out.append(y.cpu().numpy().astype("<f4"))
    D = np.concatenate(out, 0)
    with open(os.path.join(a.outdir, d + ".ovpr"), "wb") as f:
        f.write(b"OVPR")
        f.write(struct.pack("<I", D.shape[1]))
        for (t, _), v in zip(frames, D):
            f.write(struct.pack("<q", t))
            f.write(v.tobytes())
    info["dirs"][d] = len(frames)
    n_all += len(frames)
info["n_images"] = n_all
info["ms_per_image_gpu"] = round(1000 * gpu_t / max(n_all, 1), 2)
json.dump(info, open(os.path.join(a.outdir, "batch_info.json"), "w"), indent=1)
print(a.model, len(dirs), "dirs", n_all, "images", info["ms_per_image_gpu"], "ms/img")
