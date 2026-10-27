#!/usr/bin/env python3
"""npu: list op types (+count) and tensor dtypes of TFLite files.  python3 tfl_ops.py FILE..."""
import sys, collections
import tensorflow as tf
for fn in sys.argv[1:]:
    it = tf.lite.Interpreter(fn)
    ops = collections.Counter(d["op_name"] for d in it._get_ops_details())
    dt = collections.Counter(str(t["dtype"].__name__) for t in it.get_tensor_details())
    print(fn.split("/")[-1], dict(ops), "dtypes", dict(dt))
