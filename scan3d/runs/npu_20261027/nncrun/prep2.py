#!/usr/bin/env python3
"""nncrun 2026-10-27 (2nd batch): phone inputs for c3/c4 (input range folded into the model: u=(x-lo)/(hi-lo)) and a3 (1ch, raw x).
x = 13 eval frames, per-image normalised gray (eval_640x352_norm.npy). Also checks each ONNX fp32 on its own input == the
original fp32 reference (ref_eval_fp32.bin from a2 ONNX on x), so the fold itself is exact.
  python prep2.py OUTDIR"""
import json, sys, time
import numpy as np, onnxruntime as ort
R = "/mnt/c/Users/a3162/scan3d_handoff/npu_results"
D = "/home/system/scanner/tmp_npu/nncrun_data"
O = sys.argv[1]
ev = np.load("/home/system/scanner/tmp_npu/data/eval_640x352_norm.npy").astype(np.float32)  # (13,1,352,640)
ref = np.fromfile(f"{D}/ref_eval_fp32.bin", np.float32).reshape(-1, 130, 44, 80)
snr = lambda r, t: float(10 * np.log10((r.astype(np.float64) ** 2).sum() / max(1e-30, ((r - t).astype(np.float64) ** 2).sum())))
M = {"c3": ("c3_xfeat3ch_fold_m5p11_e2400/xfeat_3ch_cat1out_fold_m5p11.onnx", -5.0, 11.0, 3),
     "c4": ("c4_xfeat3ch_fold_m3p8_e2400/xfeat_3ch_cat1out_fold_m3p8.onnx", -3.0, 8.0, 3),
     "a3": ("a3_xfeat1ch_cat1out_e2400/xfeat_backbone_noadd_op13_cat1out.onnx", None, None, 1)}
info = {"made": time.strftime("%F %T"), "x_range": [float(ev.min()), float(ev.max())]}
for k, (p, lo, hi, ch) in M.items():
    x = np.repeat(ev, ch, axis=1)
    u = x if lo is None else (x - lo) / (hi - lo)
    u = u.astype(np.float32)
    u.tofile(f"{O}/in_{k}.bin")
    S = ort.InferenceSession(f"{R}/{p}", providers=["CPUExecutionProvider"])
    nm = S.get_inputs()[0].name
    o = np.concatenate([S.run(None, {nm: u[i:i + 1]})[0] for i in range(len(u))])
    info[k] = {"onnx": p, "input": nm, "lo": lo, "hi": hi, "u_range": [float(u.min()), float(u.max())],
               "share_u_lt0": float((u < 0).mean()), "share_u_gt1": float((u > 1).mean()),
               "pc_fp32_onnx_vs_ref_snr_db": snr(ref, o)}
print(json.dumps(info, indent=1))
json.dump(info, open(f"{O}/prep2_info.json", "w"), indent=1)
