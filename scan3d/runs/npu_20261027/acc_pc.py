#!/usr/bin/env python3
"""npu: PC accuracy of the converted TFLite models (TF CPU reference kernels) vs PC fp32 ONNX (ORT CPU) on our 13 eval
frames / pairs.  venv_tf python.  python3 acc_pc.py OUT.json"""
import json, sys, time, glob, os
import numpy as np
import onnxruntime as ort
import tensorflow as tf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from acc_lib import xf_compare, lg_compare
B = "/home/system/scanner/tmp_npu"
res = {"made": time.strftime("%F %T"), "inputs": "eval_640x352_norm.npy (13 frames), lgev512_* (13 pairs)", "xfeat": {}, "lg512": {}}
ev = np.load(f"{B}/data/eval_640x352_norm.npy")
S = ort.InferenceSession(f"{B}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
refs = [[o[0] for o in S.run(None, {"image_norm": x[None]})] for x in ev]
refs = [(r[0], r[1], r[2][0]) for r in refs]


def tfl_run(fn, feeds):
    it = tf.lite.Interpreter(fn, num_threads=8); it.allocate_tensors()
    for d in it.get_input_details():
        x = feeds[d["name"].split(":")[0]] if d["name"].split(":")[0] in feeds else None
        if x is None:
            x = [v for k, v in feeds.items() if k in d["name"]][0]
        if d["dtype"] != np.float32:
            s, z = d["quantization"]; x = np.clip(np.round(x / s + z), -128, 127).astype(d["dtype"])
        it.set_tensor(d["index"], x.reshape(d["shape"]).astype(d["dtype"]))
    it.invoke()
    outs = []
    for d in it.get_output_details():
        o = it.get_tensor(d["index"])
        if o.dtype != np.float32:
            s, z = d["quantization"]; o = (o.astype(np.float32) - z) * s
        outs.append(o)
    return outs


for fn in sorted(glob.glob(f"{B}/tflite/xfeatnn_640x352/*.tflite")):
    nm = os.path.basename(fn)[:-7]
    rows = []
    for x, r in zip(ev, refs):
        outs = tfl_run(fn, {"image_norm": np.transpose(x[None], (0, 2, 3, 1))})
        byc = {o.shape[-1]: o[0].transpose(2, 0, 1) for o in outs}
        rows.append(xf_compare(r, (byc[64], byc[65], byc[1][0])))
    a = np.array([r[0] for r in rows]); c = np.array([r[1] for r in rows])
    res["xfeat"][nm] = {"overlap_mean": float(a.mean()), "overlap_min": float(a.min()), "cos_med_mean": float(c.mean()), "cos_min": float(min(r[4] for r in rows))}
    print(nm, res["xfeat"][nm], flush=True)
L = {k: np.load(f"{B}/data/lgev512_{k}.npy") for k in ("kpts0", "kpts1", "desc0", "desc1")}
S2 = ort.InferenceSession(f"{B}/onnx/lgc_512.onnx", providers=["CPUExecutionProvider"])
lref = [S2.run(None, {k: L[k][i:i + 1] for k in L}) for i in range(len(L["kpts0"]))]
for fn in sorted(glob.glob(f"{B}/tflite/lgc_512/*.tflite")):
    nm = os.path.basename(fn)[:-7]
    rows = []
    for i, r in enumerate(lref):
        try:
            outs = tfl_run(fn, {k: L[k][i:i + 1] for k in L})
        except Exception as e:
            res["lg512"][nm] = {"error": str(e)[:200]}; print(nm, "ERROR", str(e)[:120]); break
        Pm = [o for o in outs if o.shape[-1] == 512][0]
        zs = [o for o in outs if o.shape[-1] == 1]
        # z0/z1 order: identify by closeness to reference
        if np.abs(zs[0] - r[1]).mean() > np.abs(zs[1] - r[1]).mean():
            zs = zs[::-1]
        rows.append(lg_compare((r[0][0], r[1][0], r[2][0]), (Pm[0], zs[0][0], zs[1][0])))
    if not rows:
        continue
    a = np.array([r[0] for r in rows])
    res["lg512"][nm] = {"agree_mean": float(a.mean()), "agree_min": float(a.min()), "ref_matches_mean": float(np.mean([r[1] for r in rows])),
                         "test_matches_mean": float(np.mean([r[2] for r in rows]))}
    print(nm, res["lg512"][nm], flush=True)
json.dump(res, open(sys.argv[1], "w"), indent=1)
