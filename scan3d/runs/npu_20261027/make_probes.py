#!/usr/bin/env python3
"""npu: single-op TFLite probe models (full int8 incl. int8 in/out, and float32) to find which ops / shapes the Samsung
ENN NNAPI driver accepts.  venv_tf python.  python3 make_probes.py OUTDIR"""
import os, sys
import numpy as np
import tensorflow as tf
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
R = np.random.RandomState(1)
W = lambda *s: tf.constant(R.randn(*s).astype(np.float32) * 0.1)
def mk(name, shapes, fn):
    f = tf.function(fn, input_signature=[tf.TensorSpec(s, tf.float32) for s in shapes]).get_concrete_function()
    def rep():
        r = np.random.RandomState(0)
        for _ in range(8):
            yield [r.randn(*s).astype(np.float32) for s in shapes]
    for q in ("i8", "f32"):
        c = tf.lite.TFLiteConverter.from_concrete_functions([f])
        if q == "i8":
            c.optimizations = [tf.lite.Optimize.DEFAULT]; c.representative_dataset = rep
            c.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
            c.inference_input_type = tf.int8; c.inference_output_type = tf.int8
        open(f"{OUT}/p_{name}_{q}.tflite", "wb").write(c.convert())
    print(name, flush=True)
w33, b64 = W(3, 3, 64, 64), W(64)
mk("conv3", [(1, 44, 80, 64)], lambda x: tf.nn.relu(tf.nn.conv2d(x, w33, 1, "SAME") + b64))
w11 = W(1, 1, 64, 64)
mk("conv1", [(1, 44, 80, 64)], lambda x: tf.nn.conv2d(x, w11, 1, "VALID") + b64)
w3s = W(3, 3, 32, 64)
mk("conv3s2", [(1, 88, 160, 32)], lambda x: tf.nn.conv2d(x, w3s, 2, "SAME") + b64)
w8 = W(8, 8, 1, 64)
mk("s2d8", [(1, 352, 640, 1)], lambda x: tf.nn.conv2d(x, w8, 8, "VALID"))
w3c = W(3, 3, 1, 4)
mk("conv3c1", [(1, 352, 640, 1)], lambda x: tf.nn.conv2d(x, w3c, 1, "SAME"))
mk("avgpool4", [(1, 352, 640, 1)], lambda x: tf.nn.avg_pool2d(x, 4, 4, "VALID"))
mk("pad", [(1, 44, 80, 64)], lambda x: tf.pad(x, [[0, 0], [1, 1], [1, 1], [0, 0]]))
mk("add", [(1, 44, 80, 64), (1, 44, 80, 64)], lambda a, b: a + b)
mk("resize", [(1, 22, 40, 64)], lambda x: tf.image.resize(x, (44, 80), "bilinear"))
mk("sigmoid", [(1, 44, 80, 1)], lambda x: tf.sigmoid(x))
wfc, b96 = W(96, 96), W(96)
mk("fc", [(1, 512, 96)], lambda x: tf.matmul(x, wfc) + b96)
mk("fc2d", [(512, 96)], lambda x: tf.matmul(x, wfc) + b96)
mk("bmm", [(1, 512, 96), (1, 512, 96)], lambda a, b: tf.matmul(a, b, transpose_b=True))
mk("softmax", [(1, 512, 512)], lambda x: tf.nn.softmax(x, -1))
mk("gelu_tanh", [(1, 512, 192)], lambda x: tf.nn.gelu(x, approximate=True))
mk("mean", [(1, 512, 192)], lambda x: tf.reduce_mean(x, -1, keepdims=True))
