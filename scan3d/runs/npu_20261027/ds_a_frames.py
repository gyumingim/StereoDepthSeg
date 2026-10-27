#!/usr/bin/env python3
"""npu downstream A (venv_ffs): second frame of each eval pair (eval frame + 30) -> data/eval2_640x352_norm.npy"""
import cv2, numpy as np
D = "/home/system/scanner/tmp_npu/data"; R = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
names = open(f"{D}/eval_list.txt").read().split("eval:\n")[1].split()
out = []
for n in names:
    rec, fi = n.split("#")
    cap = cv2.VideoCapture(f"{R}/{rec}/cam5.mp4"); cap.set(cv2.CAP_PROP_POS_FRAMES, int(fi) + 30)
    ok, fr = cap.read(); assert ok
    g = cv2.resize(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), (960, 540), interpolation=cv2.INTER_AREA)
    x = cv2.resize(g, (640, 352), interpolation=cv2.INTER_AREA).astype(np.float32)[None] / 255.0
    out.append(((x - x.mean()) / np.sqrt(x.var() + 1e-5)).astype(np.float32))
np.save(f"{D}/eval2_640x352_norm.npy", np.stack(out)); print(len(out))
