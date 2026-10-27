#!/usr/bin/env python3
"""nncrun 2026-10-27: phone inputs + PC fp32 references for the AI Studio .nnc (a2_xfeat3ch_cat1out_e2400).
Inputs (float32 NCHW 1x3x352x640, raw bytes, frames concatenated):
  golden_in.bin = AI Studio input_data.bin (as given)
  eval3ch.bin   = 13 eval frames, per-image normalised gray (eval_640x352_norm.npy) copied to 3 channels
                  (the 3ch variant starts with a 1x1 conv with weights 1/3 each, so 3 equal channels == the gray input)
References (float32 NCHW 1x130x44x80): PC onnxruntime CPU fp32 of the cat1out ONNX on the same inputs.
Also checks cat1out ONNX == original xfeatnn_640x352.onnx (feats/kpts/heat order and heat activation)."""
import json, os, sys, time
import numpy as np, onnxruntime as ort
R = "/mnt/c/Users/a3162/scan3d_handoff/npu_results/a2_xfeat3ch_cat1out_e2400"
NM = "xfeat_backbone_3ch_noadd_op13_cat1out"
B = "/home/system/scanner/tmp_npu"
O = sys.argv[1]
os.makedirs(O, exist_ok=True)
gin = np.fromfile(f"{R}/{NM}-input_data.bin", np.float32).reshape(1, 3, 352, 640)
gold = np.fromfile(f"{R}/{NM}-golden_data.bin", np.float32).reshape(1, 130, 44, 80)
ev = np.load(f"{B}/data/eval_640x352_norm.npy").astype(np.float32)  # (13,1,352,640)
ev3 = np.repeat(ev, 3, axis=1)
gin.tofile(f"{O}/golden_in.bin")
ev3.tofile(f"{O}/eval3ch.bin")
S = ort.InferenceSession(f"{R}/{NM}.onnx", providers=["CPUExecutionProvider"])
iname = S.get_inputs()[0].name
ref_g = S.run(None, {iname: gin})[0]
ref_e = np.concatenate([S.run(None, {iname: ev3[i:i + 1]})[0] for i in range(len(ev3))])
ref_g.astype(np.float32).tofile(f"{O}/ref_golden_fp32.bin")
ref_e.astype(np.float32).tofile(f"{O}/ref_eval_fp32.bin")
snr = lambda r, t: float(10 * np.log10((r.astype(np.float64) ** 2).sum() / max(1e-30, ((r - t).astype(np.float64) ** 2).sum())))
S0 = ort.InferenceSession(f"{B}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
o0 = S0.run(None, {"image_norm": ev[:1]})
names0 = [o.name for o in S0.get_outputs()]
c = ref_e[0]
chk = {"orig_outputs": names0, "orig_shapes": [list(x.shape) for x in o0],
       "feats_maxabs": float(np.abs(c[:64] - o0[0][0]).max()), "kpts_maxabs": float(np.abs(c[64:129] - o0[1][0]).max()),
       "heat_raw_vs_orig_maxabs": float(np.abs(c[129] - o0[2][0].reshape(44, 80)).max()),
       "sigmoid_heat_vs_orig_maxabs": float(np.abs(1 / (1 + np.exp(-c[129])) - o0[2][0].reshape(44, 80)).max())}
info = {"made": time.strftime("%F %T"), "onnx": f"{R}/{NM}.onnx", "onnx_input": iname,
        "golden_in_stats": [float(gin.min()), float(gin.max()), float(gin.mean()), float(gin.std())],
        "eval_in_stats": [float(ev.min()), float(ev.max()), float(ev.mean()), float(ev.std())],
        "eval_share_abs_gt_1": float((np.abs(ev) > 1.0).mean()),
        "pc_onnx_vs_aistudio_golden_snr_db": snr(gold, ref_g), "orig_check": chk}
print(json.dumps(info, indent=1))
json.dump(info, open(f"{O}/prep_info.json", "w"), indent=1)
