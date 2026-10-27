"""nncrun: PC fp32 of the xpnmsr ONNX (relu-approximated NMS mask in the model) on the 13 eval frames -> kp file in the
phone pipe format (score>0 top-1024 raster tie-break, desc at the nearest cell + L2) for acc_official.py / acc_kp.py.
venv_tf python: python pc_xpnmsr.py OUT_KP.bin OUT_RAW.bin"""
import sys, numpy as np, onnxruntime as ort
S = ort.InferenceSession("/mnt/c/Users/a3162/scan3d_handoff/npu_results/lab_xfeat_post/xpnmsr/xfeat3ch_postnms_relu.onnx", providers=["CPUExecutionProvider"])
ev = np.repeat(np.load("/home/system/scanner/tmp_npu/data/eval_640x352_norm.npy").astype(np.float32), 3, axis=1)
fk = open(sys.argv[1], "wb"); raws = []
for i in range(len(ev)):
    o = S.run(None, {S.get_inputs()[0].name: ev[i:i + 1]})[0][0]; raws.append(o)
    c, cy, cx = np.nonzero(o[64:] > 0); s = o[64:][c, cy, cx]; y, x = cy * 8 + c // 8, cx * 8 + c % 8
    od = np.lexsort((y * 640 + x, -s))[:1024]; x, y = x[od], y[od]
    f = o[:64][:, y // 8, x // 8].T; f = f / np.maximum(np.linalg.norm(f, axis=1, keepdims=True), 1e-6)
    fk.write(np.int32(len(x)).tobytes()); fk.write(np.stack([x, y], 1).astype(np.int32).tobytes()); fk.write(f.astype(np.float32).tobytes())
fk.close(); np.stack(raws).astype(np.float32).tofile(sys.argv[2])
