#!/usr/bin/env python3
"""ovxfeat (supervisor 21:03 #4): phone backend outputs vs PC ORT CPU on the same real frame (142610 #600, 640x352):
top-k keypoint overlap (same XFeat post-processing: softmax heatmap, NMS 5, threshold 0.05, score = K * H, top 1024;
a keypoint 'matches' if the other set has one within 1 px) and descriptor cosine at the reference keypoints.
  python3 acc_compare.py ACCDIR"""
import glob
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F

XF_SRC = "/home/system/scanner/tmp_ovxfeat/xfeat_src"
sys.path.insert(0, XF_SRC)
from modules.xfeat import XFeat  # noqa: E402
from modules.interpolator import InterpolateSparse2d  # noqa: E402

A = sys.argv[1]
H, W = 352, 640
h, w = H // 8, W // 8
xf = XFeat(weights=None)
xf.dev = torch.device("cpu")


def load_nchw(prefix):
    f = np.fromfile(f"{prefix}_feats.f32", np.float32).reshape(1, 64, h, w)
    k = np.fromfile(f"{prefix}_keypoints.f32", np.float32).reshape(1, 65, h, w)
    hm = np.fromfile(f"{prefix}_heatmap.f32", np.float32).reshape(1, 1, h, w)
    return f, k, hm


def load_tfl(fn):
    # benchmark_model --output_filepath: raw bytes of the outputs one after the other (NHWC); order = output tensor order
    raw = np.fromfile(fn, np.float32)
    sizes = {64: h * w * 64, 65: h * w * 65, 1: h * w}
    out = {}
    off = 0
    for c in order_tfl:
        out[c] = raw[off:off + sizes[c]].reshape(1, h, w, c).transpose(0, 3, 1, 2)
        off += sizes[c]
    assert off == raw.size, (off, raw.size)
    return out[64], out[65], out[1]


def post(f, k, hm, topk=1024):
    M1 = F.normalize(torch.from_numpy(f), dim=1)
    K1h = xf.get_kpts_heatmap(torch.from_numpy(k))
    mk = xf.NMS(K1h, threshold=0.05, kernel_size=5)
    sc = (InterpolateSparse2d("nearest")(K1h, mk, H, W) * InterpolateSparse2d("bilinear")(torch.from_numpy(hm), mk, H, W)).squeeze(-1)
    idx = torch.argsort(-sc)[0, :topk]
    kp = mk[0, idx].float()
    return kp, M1


ref = post(*load_nchw(f"{A}/ref_640x352"))
order_tfl = [64, 65, 1]
cands = {os.path.basename(p)[:-10]: p[:-10] for p in glob.glob(f"{A}/phone/o_ort_*_640_feats.f32")}
res = {}
for name, pre in sorted(cands.items()):
    res[name] = post(*load_nchw(pre))
for fn in sorted(glob.glob(f"{A}/phone/o_tfl_*_640.bin")):
    raw = np.fromfile(fn, np.float32)
    best = None
    import itertools
    for perm in itertools.permutations([64, 65, 1]):
        order_tfl = list(perm)
        try:
            out = post(*load_tfl(fn))
        except AssertionError:
            continue
        d = torch.cdist(out[0], ref[0]).min(1).values
        ov = float((d <= 1.0).float().mean())
        if best is None or ov > best[0]:
            best = (ov, out, perm)
    if best:
        res[os.path.basename(fn)[:-4] + f" (order {best[2]})"] = best[1]
bil = InterpolateSparse2d("bicubic")
for name, (kp, M1) in res.items():
    d = torch.cdist(kp, ref[0]).min(1).values
    ov = float((d <= 1.0).float().mean())
    dr = F.normalize(bil(ref[1], ref[0][None], H, W)[0], dim=-1)
    dp = F.normalize(bil(M1, ref[0][None], H, W)[0], dim=-1)
    cos = (dr * dp).sum(-1)
    print(f"{name:40s} top-1024 overlap (<=1 px) {ov * 100:.1f} %  desc cos at ref kps: median {float(cos.median()):.4f} p5 {float(cos.quantile(0.05)):.4f} min {float(cos.min()):.4f}")
