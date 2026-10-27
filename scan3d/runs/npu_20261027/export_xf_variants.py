#!/usr/bin/env python3
"""npu: XFeat nonorm (as ovxfeat export_xfeat_nonorm.py: s2d as one-hot conv, no InstanceNorm) with int8-friendlier
keypoint output: 'sm' = softmax over the 65 channels inside the graph (probabilities 0..1 quantise far better than logits
-25..8); 'cl' = logits clamped to >= -8.   python3 export_xf_variants.py OUTDIR"""
import os, sys
import torch, torch.nn as nn
XF_SRC = "/home/system/scanner/tmp_ovxfeat/xfeat_src"
sys.path.insert(0, XF_SRC)
from modules.model import XFeatModel
OUT = sys.argv[1]
torch.set_grad_enabled(False)
m = XFeatModel().eval()
m.load_state_dict(torch.load(os.path.join(XF_SRC, "weights", "xfeat.pt"), map_location="cpu"))
m.norm = nn.Identity()
s2d = nn.Conv2d(1, 64, 8, stride=8, bias=False)
w = torch.zeros(64, 1, 8, 8)
for i in range(8):
    for j in range(8):
        w[i * 8 + j, 0, i, j] = 1.0
s2d.weight = nn.Parameter(w, requires_grad=False)
m.s2d = s2d
m._unfold2d = lambda x, ws=2: m.s2d(x)
class V(nn.Module):
    def __init__(s, m, mode):
        super().__init__(); s.m, s.mode = m, mode
    def forward(s, x):
        f, k, h = s.m(x)
        k = torch.softmax(k, 1) if s.mode == "sm" else torch.clamp(k, min=-8.0)
        return f, k, h
for mode in ("sm", "cl"):
    x = torch.rand(1, 1, 352, 640)
    torch.onnx.export(V(m, mode).eval(), (x,), os.path.join(OUT, f"xfeat{mode}_640x352.onnx"), input_names=["image_norm"],
                      output_names=["feats", "keypoints", "heatmap"], opset_version=17, dynamo=False)
    print(mode, "ok")
