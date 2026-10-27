#!/usr/bin/env python3
"""npu 2026-10-27: calibration (int8) + evaluation frames from our own std60 recordings (cam5.mp4, gray, 960x540 ->
INTER_AREA resize W x H, /255). Calibration = 48 frames (8 recordings x 6), evaluation = 12 DIFFERENT recordings x 1
frame (+ the old white-wall frame 142610#600). Disjoint recordings. Writes OUT/{calib,eval}_{W}x{H}[_norm].npy (NCHW,
_norm = per-image InstanceNorm as in xfeatnn_*) and eval_list.txt.   python3 make_calib.py OUT"""
import glob, os, sys
import cv2
import numpy as np
OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
R = "/home/system/scanner/StereoDepthSeg/scan3d/recordings"
recs = sorted(d for d in glob.glob(f"{R}/hs_*_std60") if os.path.exists(f"{d}/cam5.mp4"))
rng = np.random.RandomState(27)
perm = rng.permutation(len(recs))
cal_recs = [recs[i] for i in perm[:8]]
ev_recs = [recs[i] for i in perm[8:20] if "142610" not in recs[i]]
def grab(rec, idxs):
    cap = cv2.VideoCapture(f"{rec}/cam5.mp4")
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out = []
    for f in idxs:
        fi = int(f * (n - 1))
        cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
        ok, fr = cap.read()
        if ok:
            out.append((f"{os.path.basename(rec)}#{fi}", cv2.resize(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), (960, 540), interpolation=cv2.INTER_AREA)))
    return out
cal = [g for r in cal_recs for g in grab(r, np.linspace(0.1, 0.9, 6))]
ev = [g for r in ev_recs for g in grab(r, [0.5])]
ev += grab(f"{R}/hs_20261024_142610_std60", [600 / max(1, int(cv2.VideoCapture(f'{R}/hs_20261024_142610_std60/cam5.mp4').get(cv2.CAP_PROP_FRAME_COUNT)) - 1)])
for W, H in [(640, 352)]:
    for nm, S in [("calib", cal), ("eval", ev)]:
        x = np.stack([cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32)[None] / 255.0 for _, g in S])
        mu = x.mean(axis=(1, 2, 3), keepdims=True); sd = np.sqrt(x.var(axis=(1, 2, 3), keepdims=True) + 1e-5)
        np.save(f"{OUT}/{nm}_{W}x{H}.npy", x)
        np.save(f"{OUT}/{nm}_{W}x{H}_norm.npy", ((x - mu) / sd).astype(np.float32))
        print(nm, W, H, x.shape)
open(f"{OUT}/eval_list.txt", "w").write("calib:\n" + "\n".join(n for n, _ in cal) + "\neval:\n" + "\n".join(n for n, _ in ev) + "\n")
