#!/usr/bin/env python3
"""npu: probe set 2 (supervisor 20:41 / coordinator): why ENN rejects int8 conv / FC.  Variants per op:
  i8pt   = int8, PER-TENSOR weights (converter._experimental_disable_per_channel), int8 io
  i8ptf  = int8 per-tensor, float io (QUANTIZE/DEQUANTIZE at the ends)
  u8     = legacy uint8 asymmetric (TENSOR_QUANT8_ASYMM; TF1 converter, fake-quant ranges, inference_type uint8)
  python3 make_probes2.py OUTDIR"""
import os, sys
import numpy as np
import tensorflow as tf
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
R = np.random.RandomState(1)
def W(*s):
    return R.randn(*s).astype(np.float32) * 0.1
OPS = {}
w33, b64, w11, w8, wfc, b96 = W(3, 3, 64, 64), W(64), W(1, 1, 64, 64), W(8, 8, 1, 64), W(96, 96), W(96)
OPS["conv3"] = ([(1, 44, 80, 64)], lambda x: tf.nn.relu(tf.nn.conv2d(x, w33, 1, "SAME") + b64))
OPS["conv1"] = ([(1, 44, 80, 64)], lambda x: tf.nn.conv2d(x, w11, 1, "VALID") + b64)
OPS["s2d8"] = ([(1, 352, 640, 1)], lambda x: tf.nn.conv2d(x, w8, 8, "VALID"))
OPS["fc2d"] = ([(512, 96)], lambda x: tf.matmul(x, wfc) + b96)
OPS["add"] = ([(1, 44, 80, 64), (1, 44, 80, 64)], lambda a, b: a + b)
OPS["pad"] = ([(1, 44, 80, 64)], lambda x: tf.pad(x, [[0, 0], [1, 1], [1, 1], [0, 0]]))
OPS["resize"] = ([(1, 22, 40, 64)], lambda x: tf.image.resize(x, (44, 80), "bilinear"))
OPS["sigmoid"] = ([(1, 44, 80, 1)], lambda x: tf.sigmoid(x))
for name, (shapes, fn) in OPS.items():
    f = tf.function(fn, input_signature=[tf.TensorSpec(s, tf.float32) for s in shapes]).get_concrete_function()
    def rep():
        r = np.random.RandomState(0)
        for _ in range(8):
            yield [r.randn(*s).astype(np.float32) for s in shapes]
    for q in ("i8pt", "i8ptf"):
        c = tf.lite.TFLiteConverter.from_concrete_functions([f])
        c.optimizations = [tf.lite.Optimize.DEFAULT]; c.representative_dataset = rep
        c.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        c._experimental_disable_per_channel = True
        if q == "i8pt":
            c.inference_input_type = tf.int8; c.inference_output_type = tf.int8
        open(f"{OUT}/q_{name}_{q}.tflite", "wb").write(c.convert())
    # legacy uint8: TF1 graph with fake-quant nodes on inputs and output, inference_type uint8
    try:
        g = tf.Graph()
        with g.as_default():
            ins = [tf.compat.v1.placeholder(tf.float32, s, name=f"in{i}") for i, s in enumerate(shapes)]
            fq = [tf.quantization.fake_quant_with_min_max_args(t, -4.0, 4.0) for t in ins]
            y = tf.quantization.fake_quant_with_min_max_args(fn(*fq), -8.0, 8.0, name="out")
            with tf.compat.v1.Session(graph=g) as s:
                c = tf.compat.v1.lite.TFLiteConverter.from_session(s, ins, [y])
                c.inference_type = tf.uint8
                c.quantized_input_stats = {f"in{i}": (128.0, 127.0 / 4.0) for i in range(len(ins))}
                c.default_ranges_stats = (-8.0, 8.0)
                open(f"{OUT}/q_{name}_u8.tflite", "wb").write(c.convert())
    except Exception as e:
        print(name, "u8 FAILED", str(e)[:200])
    print(name, flush=True)
