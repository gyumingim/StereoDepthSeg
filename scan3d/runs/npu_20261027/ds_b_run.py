#!/usr/bin/env python3
"""npu downstream B (venv_tf): run XFeat variants (fp32 ONNX ref + TFLite files given) on eval + eval2 frames, save NCHW
outputs to tmp_npu/ds/<name>.npz (keypoint channel always as LOGITS: softmax variant -> log).  python3 ds_b_run.py TFLITE..."""
import os, sys, numpy as np, onnxruntime as ort, tensorflow as tf
D = "/home/system/scanner/tmp_npu"; os.makedirs(f"{D}/ds", exist_ok=True)
X = np.concatenate([np.load(f"{D}/data/eval_640x352_norm.npy"), np.load(f"{D}/data/eval2_640x352_norm.npy")])
S = ort.InferenceSession(f"{D}/onnx/xfeatnn_640x352.onnx", providers=["CPUExecutionProvider"])
o = [S.run(None, {"image_norm": x[None]}) for x in X]
np.savez(f"{D}/ds/ref_fp32.npz", f=np.concatenate([a[0] for a in o]), k=np.concatenate([a[1] for a in o]), h=np.concatenate([a[2] for a in o]))
for fn in sys.argv[1:]:
    nm = os.path.basename(fn)[:-7]
    it = tf.lite.Interpreter(fn, num_threads=8); it.allocate_tensors(); di = it.get_input_details()[0]
    F_, K_, H_ = [], [], []
    for x in X:
        xi = np.transpose(x[None], (0, 2, 3, 1))
        if di["dtype"] != np.float32:
            s, z = di["quantization"]; xi = np.clip(np.round(xi / s + z), -128, 127).astype(di["dtype"])
        it.set_tensor(di["index"], xi); it.invoke()
        outs = {}
        for d in it.get_output_details():
            t = it.get_tensor(d["index"])
            if t.dtype != np.float32:
                s, z = d["quantization"]; t = (t.astype(np.float32) - z) * s
            outs[t.shape[-1]] = t.transpose(0, 3, 1, 2)
        k = outs[65]
        if nm.startswith("xfeatsm"):
            k = np.log(np.maximum(k, 1e-6))
        F_.append(outs[64]); K_.append(k); H_.append(outs[1])
    np.savez(f"{D}/ds/{nm}.npz", f=np.concatenate(F_), k=np.concatenate(K_), h=np.concatenate(H_)); print(nm, flush=True)
