"""npuopt 10-28: XFeat NPU graph (3ch / 1ch, noadd, no sigmoid, cat1out) at 960x544 = the 640x352 Lab graph with
only the spatial dims changed (all ops are Conv/Relu/AvgPool/Concat/Resize with constant scales -> size independent).
Check: same output as official XFeatModel (InstanceNorm inside) on random + real input. Windows python (onnx, ORT, torch)."""
import onnx, numpy as np, onnxruntime as ort, sys, json, time, os
from onnx import shape_inference
SRC = {3: r"C:\Users\a3162\scan3d_handoff\npu_upload\probe\xfeat_backbone_3ch_noadd_op13_cat1out.onnx",
       1: r"C:\Users\a3162\scan3d_handoff\npu_upload\probe\xfeat_backbone_noadd_op13_cat1out.onnx"}
OUT = r"C:\Users\a3162\scan3d_handoff\npu_upload\xf960"
H, W = 544, 960
info = {"made": time.strftime("%F %T"), "src": SRC, "H": H, "W": W}
for ch, src in SRC.items():
    m = onnx.load(src); g = m.graph
    d = g.input[0].type.tensor_type.shape.dim; assert d[1].dim_value == ch
    d[2].dim_value, d[3].dim_value = H, W
    o = g.output[0].type.tensor_type.shape.dim; o[2].dim_value, o[3].dim_value = H // 8, W // 8
    del g.value_info[:]
    m = shape_inference.infer_shapes(m, strict_mode=True)
    onnx.checker.check_model(m, full_check=True)
    fn = os.path.join(OUT, f"xf960_{ch}ch_noadd_op13_cat1out.onnx"); onnx.save(m, fn)
    info[f"{ch}ch"] = fn
    print(fn, os.path.getsize(fn))
json.dump(info, open(os.path.join(OUT, "mk960_info.json"), "w"), indent=1)
