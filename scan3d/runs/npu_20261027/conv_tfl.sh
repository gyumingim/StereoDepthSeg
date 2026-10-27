#!/bin/bash
# npu: ONNX -> TFLite with onnx2tf (venv_tf: TF 2.17.1, onnx2tf 1.26.3).  conv_tfl.sh NAME [extra onnx2tf args]
B=/home/system/scanner/tmp_npu
n=$1; shift
mkdir -p $B/tflite && rm -rf $B/tflite/$n
timeout 1800 /home/system/scanner/tmp_ovxfeat/venv_tf/bin/onnx2tf -i $B/onnx/$n.onnx -o $B/tflite/$n "$@" > $B/tflite/$n.log 2>&1
echo "exit $?"
sed 's/\x1b\[[0-9;]*m//g' $B/tflite/$n.log | grep -E "ERROR|Error:|onnx_op_name" | head -6
ls -la $B/tflite/$n/*.tflite 2>/dev/null
