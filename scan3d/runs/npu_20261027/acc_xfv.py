#!/usr/bin/env python3
"""npu: PC accuracy of XFeat int8 variants (sm = softmax in graph, cl = clamped logits) vs fp32 ONNX, 13 eval frames."""
import sys, os, glob, json, numpy as np, onnxruntime as ort, tensorflow as tf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acc_lib import xf_compare
B = "/home/system/scanner/tmp_npu"
ev = np.load(f"{B}/data/eval_640x352_norm.npy")
S = ort.InferenceSession(f"{B}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
refs = []
for x in ev:
    r = [o[0] for o in S.run(None, {"image_norm": x[None]})]; refs.append((r[0], r[1], r[2][0]))
res = {}
for fn in sorted(glob.glob(f"{B}/tflite/xfeat[sc][ml]_640x352/*quant*.tflite")):
    nm = os.path.basename(fn)[:-7]
    it = tf.lite.Interpreter(fn, num_threads=8); it.allocate_tensors()
    di = it.get_input_details()[0]
    rows = []
    for x, r in zip(ev, refs):
        xi = np.transpose(x[None], (0, 2, 3, 1))
        if di["dtype"] != np.float32:
            s, z = di["quantization"]; xi = np.clip(np.round(xi / s + z), -128, 127).astype(di["dtype"])
        it.set_tensor(di["index"], xi); it.invoke()
        outs = {}
        for d in it.get_output_details():
            o = it.get_tensor(d["index"])
            if o.dtype != np.float32:
                s, z = d["quantization"]; o = (o.astype(np.float32) - z) * s
            outs[o.shape[-1]] = o[0].transpose(2, 0, 1)
        k = outs[65]
        if nm.startswith("xfeatsm"):
            k = np.log(np.maximum(k, 1e-6))
        rows.append(xf_compare(r, (outs[64], k, outs[1][0])))
    res[nm] = {"overlap_mean": float(np.mean([q[0] for q in rows])), "overlap_min": float(np.min([q[0] for q in rows])),
               "cos_med_mean": float(np.mean([q[1] for q in rows])), "cos_min": float(np.min([q[4] for q in rows]))}
    print(nm, res[nm], flush=True)
json.dump(res, open("acc_pc_xfv.json", "w"), indent=1)
