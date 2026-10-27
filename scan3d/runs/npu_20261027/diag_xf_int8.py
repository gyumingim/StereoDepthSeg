#!/usr/bin/env python3
"""npu: which XFeat output loses accuracy under int8 (swap one output at a time int8 -> fp32 ref) + output value ranges."""
import sys, os, numpy as np, onnxruntime as ort, tensorflow as tf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acc_lib import xf_compare
B = "/home/system/scanner/tmp_npu"
ev = np.load(f"{B}/data/eval_640x352_norm.npy")
S = ort.InferenceSession(f"{B}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
it = tf.lite.Interpreter(f"{B}/tflite/xfeatnn_640x352/xfeatnn_640x352_integer_quant.tflite", num_threads=8); it.allocate_tensors()
print([ (d["name"], d["shape"], d["dtype"].__name__) for d in it.get_output_details()])
acc = {k: [] for k in ("all", "feats_only", "kp_only", "hm_only")}
for x in ev:
    r = [o[0] for o in S.run(None, {"image_norm": x[None]})]; r = (r[0], r[1], r[2][0])
    it.set_tensor(it.get_input_details()[0]["index"], np.transpose(x[None], (0, 2, 3, 1)))
    it.invoke()
    byc = {o.shape[-1]: o[0].transpose(2, 0, 1) for o in (it.get_tensor(d["index"]) for d in it.get_output_details())}
    q = (byc[64], byc[65], byc[1][0])
    acc["all"].append(xf_compare(r, q)[0])
    acc["feats_only"].append(xf_compare(r, (q[0], r[1], r[2]))[0])
    acc["kp_only"].append(xf_compare(r, (r[0], q[1], r[2]))[0])
    acc["hm_only"].append(xf_compare(r, (r[0], r[1], q[2]))[0])
print({k: round(float(np.mean(v)), 3) for k, v in acc.items()})
print("ref kp logits range", float(r[1].min()), float(r[1].max()), "q kp unique", len(np.unique(q[1])), "hm range", float(r[2].min()), float(r[2].max()), "q hm unique", len(np.unique(q[2])))
