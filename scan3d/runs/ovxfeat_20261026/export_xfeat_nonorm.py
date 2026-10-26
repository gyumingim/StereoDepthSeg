#!/usr/bin/env python3
"""ovxfeat: XFeat backbone ONNX without the input InstanceNorm (onnx2tf cannot convert it); the per-image
mean / std normalisation (InstanceNorm2d(1), eps 1e-5, no affine) is done on the CPU before the network (a few us).
Checks: nonorm(normalised x) == full model(x).  python3 export_xfeat_nonorm.py OUTDIR"""
import os
import sys

import numpy as np
import torch
import torch.nn as nn

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
from modules.model import XFeatModel  # noqa: E402

OUT = sys.argv[1]
torch.set_grad_enabled(False)
m = XFeatModel().eval()
m.load_state_dict(torch.load(os.path.join(XF_SRC, "weights", "xfeat.pt"), map_location="cpu"))
full = XFeatModel().eval()
full.load_state_dict(m.state_dict())
m.norm = nn.Identity()
# _unfold2d(x, 8) on the 1-channel image = 8x8 space-to-depth (channel i*8+j); as a fixed one-hot stride-8 conv it is
# exact and converts / runs on any NPU (the unfold/reshape/permute chain does not convert with onnx2tf)
s2d = nn.Conv2d(1, 64, 8, stride=8, bias=False)
w = torch.zeros(64, 1, 8, 8)
for i in range(8):
    for j in range(8):
        w[i * 8 + j, 0, i, j] = 1.0
s2d.weight = nn.Parameter(w, requires_grad=False)
m.s2d = s2d
m._unfold2d = lambda x, ws=2: m.s2d(x)
lines = []
for (H, W) in [(256, 480), (352, 640), (544, 960)]:
    x = torch.rand(1, 1, H, W)
    xn = (x - x.mean()) / torch.sqrt(x.var(unbiased=False) + 1e-5)
    ref = full(x)
    got = m(xn)
    err = max(float((a - b).abs().max()) for a, b in zip(ref, got))
    fn = os.path.join(OUT, f"xfeatnn_{W}x{H}.onnx")
    torch.onnx.export(m, (xn,), fn, input_names=["image_norm"], output_names=["feats", "keypoints", "heatmap"], opset_version=17,
                      dynamo=False)
    lines.append(f"{fn} nonorm(norm x) vs full(x) max err {err:.2e}")
    print(lines[-1])
open(os.path.join(OUT, "export_nonorm_info.txt"), "w").write("\n".join(lines) + "\n")
