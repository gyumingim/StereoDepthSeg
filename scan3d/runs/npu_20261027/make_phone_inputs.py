#!/usr/bin/env python3
"""npu: raw float32 input files for the phone GPU accuracy check (benchmark_model --input_layer_value_files) +
tensor names/order of the TFLite models.  venv_tf.  python3 make_phone_inputs.py OUTDIR"""
import os, sys, numpy as np, tensorflow as tf
B = "/home/system/scanner/tmp_npu"; O = sys.argv[1]; os.makedirs(O, exist_ok=True)
ev = np.load(f"{B}/data/eval_640x352_norm.npy")
for i in (0, 5, 12):
    np.transpose(ev[i:i + 1], (0, 2, 3, 1)).astype(np.float32).tofile(f"{O}/xin_{i}.f32")
for N in (512, 1024):
    L = {k: np.load(f"{B}/data/lgev{N}_{k}.npy") for k in ("kpts0", "kpts1", "desc0", "desc1")}
    for i in (0, 5, 12):
        for k, v in L.items():
            v[i:i + 1].astype(np.float32).tofile(f"{O}/lg{N}_{i}_{k}.f32")
for fn in (f"{B}/tflite/xfeatnn_640x352/xfeatnn_640x352_float32.tflite", f"{B}/tflite/lgct_512/lgct_512_float32.tflite", f"{B}/tflite/lgct_1024/lgct_1024_float32.tflite"):
    it = tf.lite.Interpreter(fn)
    print(os.path.basename(fn), "IN", [(d["name"], list(d["shape"])) for d in it.get_input_details()], "OUT", [(d["name"], list(d["shape"])) for d in it.get_output_details()])
