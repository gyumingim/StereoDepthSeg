#!/usr/bin/env python3
"""npu 2026-10-27: PC fp32 references (ORT CPU, xfeatnn_640x352.onnx on InstanceNorm-ed input) for
 (a) XFeat accuracy: eval frames -> ref outputs (feats/keypoints/heatmap) saved to D/ref_xf_eval.npz
 (b) LighterGlue int8 calibration inputs: pairs of consecutive calib frames (same recording, 6 per rec -> 5 pairs x 8)
 (c) LighterGlue eval pairs: each eval frame + the frame 30 frames (0.5 s) later of the same recording
Keypoints: XFeat post (softmax heatmap, NMS 5, thr 1e-4, score K*H, top N), desc = bicubic sample + L2 norm,
kpts normalised like kornia normalize_keypoints with image size (640, 352).   python3 prep_feats.py D"""
import os, sys
import cv2
import numpy as np
import onnxruntime as ort
import torch
import torch.nn.functional as F
XF_SRC = "/home/system/scanner/tmp_ovxfeat/xfeat_src"
sys.path.insert(0, XF_SRC)
from modules.xfeat import XFeat
from modules.interpolator import InterpolateSparse2d
D = sys.argv[1]
H, W = 352, 640
xf = XFeat(weights=None); xf.dev = torch.device("cpu")
S = ort.InferenceSession("/home/system/scanner/tmp_ovxfeat/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
def norm(x):
    return ((x - x.mean()) / np.sqrt(x.var() + 1e-5)).astype(np.float32)
def run(xn):
    return S.run(None, {"image_norm": xn[None]})
def kpdesc(f, k, hm, N, thr=1e-4):
    M1 = F.normalize(torch.from_numpy(f), dim=1)
    K1h = xf.get_kpts_heatmap(torch.from_numpy(k))
    mk = xf.NMS(K1h, threshold=thr, kernel_size=5)
    sc = (InterpolateSparse2d("nearest")(K1h, mk, H, W) * InterpolateSparse2d("bilinear")(torch.from_numpy(hm), mk, H, W)).squeeze(-1)
    idx = torch.argsort(-sc)[0, :N]
    kp = mk[0, idx].float()
    assert kp.shape[0] == N, kp.shape
    de = F.normalize(InterpolateSparse2d("bicubic")(M1, kp[None], H, W), dim=-1)[0]
    kn = (kp - torch.tensor([W / 2, H / 2])) / (max(W, H) / 2)
    return kp.numpy(), kn.numpy().astype(np.float32), de.numpy().astype(np.float32)
cal = np.load(f"{D}/calib_640x352_norm.npy")
ev = np.load(f"{D}/eval_640x352_norm.npy")
names = open(f"{D}/eval_list.txt").read().split("eval:\n")[1].split()
# (a)
outs = [run(x) for x in ev]
np.savez(f"{D}/ref_xf_eval.npz", feats=np.concatenate([o[0] for o in outs]), keypoints=np.concatenate([o[1] for o in outs]),
         heatmap=np.concatenate([o[2] for o in outs]))
# (b)
for N in (512, 1024):
    K0, K1, D0, D1 = [], [], [], []
    for r in range(8):
        fo = [run(cal[r * 6 + j]) for j in range(6)]
        for j in range(5):
            a, b = kpdesc(*fo[j], N), kpdesc(*fo[j + 1], N)
            K0.append(a[1]); D0.append(a[2]); K1.append(b[1]); D1.append(b[2])
    for nm, arr in zip(("kpts0", "kpts1", "desc0", "desc1"), (K0, K1, D0, D1)):
        np.save(f"{D}/lgcal{N}_{nm}.npy", np.stack(arr))
    print("calib pairs", N, len(K0))
# (c)
R = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
ev2 = []
for n in names:
    rec, fi = n.split("#")
    cap = cv2.VideoCapture(f"{R}/{rec}/cam5.mp4"); cap.set(cv2.CAP_PROP_POS_FRAMES, int(fi) + 30)
    ok, fr = cap.read(); assert ok
    g = cv2.resize(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), (960, 540), interpolation=cv2.INTER_AREA)
    ev2.append(norm(cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32)[None] / 255.0))
for N in (512, 1024):
    K0, K1, D0, D1 = [], [], [], []
    for i in range(len(ev)):
        a, b = kpdesc(*outs[i], N), kpdesc(*run(ev2[i]), N)
        K0.append(a[1]); D0.append(a[2]); K1.append(b[1]); D1.append(b[2])
    for nm, arr in zip(("kpts0", "kpts1", "desc0", "desc1"), (K0, K1, D0, D1)):
        np.save(f"{D}/lgev{N}_{nm}.npy", np.stack(arr))
    print("eval pairs", N, len(K0))
