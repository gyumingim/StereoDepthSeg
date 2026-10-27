#!/usr/bin/env python3
"""npu: phone GPU outputs (benchmark_model --output_filepath raw f32, outputs concatenated in output order) vs PC fp32
ORT references on the same inputs.  XFeat (frames 0/5/12 of eval set): top-1024 overlap (1 px) + desc cos;
LighterGlue tanh-GELU on GPU vs PC fp32 lgc (exact erf GELU = kornia): mutual-match (>0.1) agreement.
  python3 acc_phone.py OUTDIR(with xf_gpu_I.bin, lg512_gpu_I.bin, lg1024_gpu_I.bin) OUT.json   (venv_tf python)"""
import json, os, sys, time
import numpy as np, onnxruntime as ort
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acc_lib import xf_compare, lg_compare
B = "/home/system/scanner/tmp_npu"; A = sys.argv[1]
res = {"made": time.strftime("%F %T"), "inputs": "eval frames 0,5,12 / lgev pairs 0,5,12", "xfeat_gpu": {}, "lg512_gpu": {}, "lg1024_gpu": {}}
ev = np.load(f"{B}/data/eval_640x352_norm.npy")
S = ort.InferenceSession(f"{B}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
h, w = 44, 80
for i in (0, 5, 12):
    fn = f"{A}/xf_gpu_{i}.bin"
    if not os.path.exists(fn):
        continue
    raw = np.fromfile(fn, np.float32)
    n64, n65 = h * w * 64, h * w * 65
    if raw.size != n64 + n65 + h * w:
        res["xfeat_gpu"][i] = {"error": f"size {raw.size}"}; continue
    f = raw[:n64].reshape(h, w, 64).transpose(2, 0, 1); k = raw[n64:n64 + n65].reshape(h, w, 65).transpose(2, 0, 1); hm = raw[n64 + n65:].reshape(h, w)
    r = [o[0] for o in S.run(None, {"image_norm": ev[i:i + 1]})]
    q = xf_compare((r[0], r[1], r[2][0]), (f, k, hm))
    res["xfeat_gpu"][i] = {"overlap": q[0], "cos_med": q[1], "cos_min": q[4], "max_abs_feat_err": float(np.abs(f - r[0]).max())}
for N in (512, 1024):
    L = {k: np.load(f"{B}/data/lgev{N}_{k}.npy") for k in ("kpts0", "kpts1", "desc0", "desc1")}
    ref_onnx = f"{B}/onnx/lgc_{N}.onnx"
    S2 = ort.InferenceSession(ref_onnx, providers=["CPUExecutionProvider"])
    for i in (0, 5, 12):
        fn = f"{A}/lg{N}_gpu_{i}.bin"
        if not os.path.exists(fn):
            continue
        raw = np.fromfile(fn, np.float32)
        if raw.size != N * N + 2 * N:
            res[f"lg{N}_gpu"][i] = {"error": f"size {raw.size}"}; continue
        P = raw[:N * N].reshape(N, N); za = raw[N * N:N * N + N]; zb = raw[N * N + N:]
        r = S2.run(None, {k: L[k][i:i + 1] for k in L})
        z0r = r[1][0].reshape(-1)
        if np.abs(za - z0r).mean() > np.abs(zb - z0r).mean():
            za, zb = zb, za
        a, nr, nt = lg_compare((r[0][0], r[1][0], r[2][0]), (P, za, zb))
        res[f"lg{N}_gpu"][i] = {"agree": a, "ref_matches": nr, "gpu_matches": nt, "P_max_abs_err": float(np.abs(P - r[0][0]).max())}
print(json.dumps(res, indent=1))
json.dump(res, open(sys.argv[2], "w"), indent=1)
