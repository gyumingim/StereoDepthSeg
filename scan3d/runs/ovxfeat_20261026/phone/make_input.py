#!/usr/bin/env python3
"""ovxfeat: real white-wall frame (hand loop 142610, frame 600) -> gray 960x540 -> resize 640x352 (and 960x544),
InstanceNorm-normalised (for xfeatnn_*) and raw /255 (for xfeat_* with the norm inside) raw float32 files for the phone
accuracy check, plus the PC ORT CPU reference outputs.  python3 make_input.py OUTDIR"""
import os
import sys

import cv2
import numpy as np
import onnxruntime as ort

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
cap = cv2.VideoCapture("/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_142610_std60/cam5.mp4")
cap.set(cv2.CAP_PROP_POS_FRAMES, 600)
ok, fr = cap.read()
assert ok
g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
g = cv2.resize(g, (960, 540), interpolation=cv2.INTER_AREA)
cv2.imwrite(os.path.join(OUT, "frame_142610_600.png"), g)
O = "/home/system/scanner/tmp_ovxfeat/onnx"
for (W, H) in [(640, 352), (960, 544)]:
    x = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32)[None, None] / 255.0
    xn = (x - x.mean()) / np.sqrt(x.var() + 1e-5)
    x.astype(np.float32).tofile(os.path.join(OUT, f"in_{W}x{H}.f32"))
    xn.astype(np.float32).tofile(os.path.join(OUT, f"in_{W}x{H}_norm.f32"))
    s = ort.InferenceSession(f"{O}/xfeat_{W}x{H}.onnx", providers=["CPUExecutionProvider"])
    for name, arr in zip(["feats", "keypoints", "heatmap"], s.run(None, {"image": x})):
        arr.astype(np.float32).tofile(os.path.join(OUT, f"ref_{W}x{H}_{name}.f32"))
    print(W, H, "written", float(x.mean()))
