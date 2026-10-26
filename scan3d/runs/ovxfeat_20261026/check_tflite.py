#!/usr/bin/env python3
"""ovxfeat: TFLite (onnx2tf, NHWC) vs ONNX outputs on the same random normalised image (venv_tf python).
  python3 check_tflite.py NAME   (e.g. xfeatnn_640x352)"""
import sys

import numpy as np
import onnxruntime as ort
import tensorflow as tf

name = sys.argv[1]
B = "/home/system/scanner/tmp_ovxfeat"
s = ort.InferenceSession(f"{B}/onnx/{name}.onnx", providers=["CPUExecutionProvider"])
i = s.get_inputs()[0]
x = np.random.RandomState(0).randn(*i.shape).astype(np.float32)
ref = s.run(None, {i.name: x})
for prec in ("float32", "float16"):
    it = tf.lite.Interpreter(f"{B}/tflite/{name}/{name}_{prec}.tflite")
    it.allocate_tensors()
    di = it.get_input_details()[0]
    it.set_tensor(di["index"], np.transpose(x, (0, 2, 3, 1)) if len(x.shape) == 4 else x)
    it.invoke()
    outs = [it.get_tensor(d["index"]) for d in it.get_output_details()]
    for r in ref:
        rr = np.transpose(r, (0, 2, 3, 1)) if r.ndim == 4 else r
        cand = [o for o in outs if o.shape == rr.shape]
        err = min(float(np.abs(o.astype(np.float32) - rr).max()) for o in cand) if cand else None
        print(name, prec, rr.shape, "max|tflite-onnx|", err, "ref max", float(np.abs(rr).max()))
