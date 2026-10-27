#!/usr/bin/env python3
"""nncrun 2026-10-27: phone NPU (.nnc via ENN) outputs vs PC fp32 ONNX on the same 13 eval frames.
Output layout (compile log): one float32 NCHW tensor 1x130x44x80 = feats 0-63, keypoint logits 64-128, heat LOGIT 129
(sigmoid applied here; prep_ref.py checked sigmoid(ch129) == original heatmap output, max err 1.5e-6).
Metrics: per-frame SNR, XFeat top-1024 overlap (1 px) + desc cos (acc_lib.xf_compare, same as npu GPU checks).
Extra (PC only, to split the loss): fp32 ONNX with the input clipped to [-1, 1] (the .nnc input quantizer range:
scale 0.0078469 zp 0, from random calibration) vs fp32.
  python acc_nnc.py PHONE_OUT_EVAL.bin OUT.json"""
import json, os, sys, time
import numpy as np, onnxruntime as ort
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
from acc_lib import xf_compare
D = "/home/system/scanner/tmp_npu/nncrun_data"
R = "/mnt/c/Users/a3162/scan3d_handoff/npu_results/a2_xfeat3ch_cat1out_e2400/xfeat_backbone_3ch_noadd_op13_cat1out.onnx"
ph = np.fromfile(sys.argv[1], np.float32).reshape(-1, 130, 44, 80)
ref = np.fromfile(f"{D}/ref_eval_fp32.bin", np.float32).reshape(-1, 130, 44, 80)
assert len(ph) == len(ref), (ph.shape, ref.shape)
ev3 = np.fromfile(f"{D}/eval3ch.bin", np.float32).reshape(-1, 3, 352, 640)
S = ort.InferenceSession(R, providers=["CPUExecutionProvider"])
clip = np.concatenate([S.run(None, {"image": np.clip(ev3[i:i + 1], -1.0, 1.0)})[0] for i in range(len(ev3))])
sig = lambda x: 1 / (1 + np.exp(-x))
snr = lambda r, t: float(10 * np.log10((r.astype(np.float64) ** 2).sum() / max(1e-30, ((r - t).astype(np.float64) ** 2).sum())))
split = lambda o: (o[:64], o[64:129], sig(o[129]))
rows = []
for i in range(len(ph)):
    q = xf_compare(split(ref[i]), split(ph[i]))
    c = xf_compare(split(ref[i]), split(clip[i]))
    rows.append({"frame": i, "snr_all_db": snr(ref[i], ph[i]), "snr_feats_db": snr(ref[i][:64], ph[i][:64]),
                 "snr_kpts_db": snr(ref[i][64:129], ph[i][64:129]), "snr_heat_db": snr(ref[i][129], ph[i][129]),
                 "overlap": q[0], "cos_med": q[1], "cos_min": q[4], "n_ref": q[2], "n_phone": q[3],
                 "pc_clip_overlap": c[0], "pc_clip_cos_med": c[1], "pc_clip_snr_db": snr(ref[i], clip[i])})
med = lambda k: float(np.median([r[k] for r in rows]))
summ = {k: med(k) for k in ("snr_all_db", "snr_feats_db", "snr_kpts_db", "snr_heat_db", "overlap", "cos_med",
                             "pc_clip_overlap", "pc_clip_cos_med", "pc_clip_snr_db")}
summ["overlap_min"] = float(min(r["overlap"] for r in rows)); summ["overlap_max"] = float(max(r["overlap"] for r in rows))
res = {"made": time.strftime("%F %T"), "phone_out": sys.argv[1], "ref": "PC onnxruntime CPU fp32 " + R,
       "inputs": f"{D}/eval3ch.bin (13 eval frames, eval_640x352_norm.npy x3 ch)", "median": summ, "frames": rows}
print(json.dumps(summ, indent=1))
json.dump(res, open(sys.argv[2], "w"), indent=1)
