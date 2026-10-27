#!/usr/bin/env python3
"""nncrun: phone keypoint files from pipe=1 (per frame: int32 n, n x (x,y) int32, n x 64 float desc) vs PC fp32 reference
top-1024 (acc_lib.topk on ref_eval_fp32.bin): overlap (1 px) and desc cos at matched points (nearest ref point <= 1 px).
  python acc_kp.py KP.bin [KP2.bin ...]  -> prints JSON per file"""
import json, sys
import numpy as np
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
from acc_lib import topk
ref = np.fromfile("/home/system/scanner/tmp_npu/nncrun_data/ref_eval_fp32.bin", np.float32).reshape(-1, 130, 44, 80)
sig = lambda x: 1 / (1 + np.exp(-x))
out = {}
for fn in sys.argv[1:]:
    b = open(fn, "rb").read(); o = 0; ov = []; cs = []
    for i in range(len(ref)):
        n = int(np.frombuffer(b, np.int32, 1, o)[0]); o += 4
        xy = np.frombuffer(b, np.int32, 2 * n, o).reshape(n, 2).astype(np.float32); o += 8 * n
        de = np.frombuffer(b, np.float32, 64 * n, o).reshape(n, 64); o += 256 * n
        a = topk(ref[i, :64], ref[i, 64:129], sig(ref[i, 129]))
        d = np.sqrt(((a[:, None] - xy[None]) ** 2).sum(-1)); j = d.argmin(1); ok = d.min(1) <= 1.0
        ov.append(ok.mean())
        f = ref[i, :64]; f = f / np.linalg.norm(f, axis=0, keepdims=True)
        gx, gy = (a[ok, 0] // 8).astype(int), (a[ok, 1] // 8).astype(int)
        cs.append(np.median((f[:, gy, gx].T * de[j[ok]]).sum(1)))
    assert o == len(b), (o, len(b))
    out[fn.split("/")[-1]] = {"overlap_med": float(np.median(ov)), "overlap_min": float(min(ov)), "cos_med": float(np.median(cs))}
print(json.dumps(out, indent=1))
