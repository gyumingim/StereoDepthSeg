"""npuopt 10-28 evening: 960x544 XFeat graph + xpsoft post nodes (Slice/Softmax/Sigmoid/Concat), single in / single out 1x129x68x120.
Also runs ORT fp32 on the 13 live-path frames and saves raw out129 for the C++/torch check. Windows python."""
import onnx, numpy as np, onnxruntime as ort, json, time, os, hashlib
from onnx import helper as h, TensorProto as TP, numpy_helper as nh
SRC = r"C:\Users\a3162\scan3d_handoff\npu_upload\xf960"
OUT = r"C:\Users\a3162\scan3d_handoff\npu_upload\xf960soft"
IN = r"C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\team\npuopt\data\in960_norm.npy"
h8, w8 = 68, 120
def const(n, a): return h.make_node("Constant", [], [n], value=nh.from_array(np.array(a, np.int64), n))
def sl(n, s, a, b): return [const(n+"_s",[a]), const(n+"_e",[b]), const(n+"_ax",[1]), h.make_node("Slice",[s,n+"_s",n+"_e",n+"_ax"],[n])]
info = {"made": time.strftime("%F %T"), "in": IN}
ev = None
try: ev = np.load(IN)
except Exception as e: info["in_err"] = str(e)
for ch in (3, 1):
    src = os.path.join(SRC, f"xf960_{ch}ch_noadd_op13_cat1out.onnx")
    m = onnx.load(src); g = m.graph; o = g.output[0].name
    nodes = sl("p_feats", o, 0, 64) + sl("p_kl", o, 64, 129) + sl("p_hl", o, 129, 130)
    nodes += [h.make_node("Softmax", ["p_kl"], ["p_soft"], axis=1)] + sl("p_prob", "p_soft", 0, 64)
    nodes += [h.make_node("Sigmoid", ["p_hl"], ["p_heat"]), h.make_node("Concat", ["p_feats", "p_prob", "p_heat"], ["out129"], axis=1)]
    g.node.extend(nodes); del g.output[:]; g.output.extend([h.make_tensor_value_info("out129", TP.FLOAT, [1, 129, h8, w8])])
    onnx.checker.check_model(m, full_check=True)
    assert len(g.input) == 1 and len(g.output) == 1
    fn = os.path.join(OUT, f"xf960_{ch}ch_postsoft.onnx"); onnx.save(m, fn)
    md5 = hashlib.md5(open(fn, "rb").read()).hexdigest()
    S = ort.InferenceSession(fn, providers=["CPUExecutionProvider"]); B = ort.InferenceSession(src, providers=["CPUExecutionProvider"])
    nin = S.get_inputs()[0].name
    r = {"src": src, "file": fn, "md5": md5, "size": os.path.getsize(fn), "input": [nin, S.get_inputs()[0].shape], "output": [S.get_outputs()[0].name, S.get_outputs()[0].shape]}
    if ev is not None:
        x = np.repeat(ev, ch, axis=1).astype(np.float32) if ch == 3 else ev.astype(np.float32)
        outs, md = [], 0.0
        for i in range(len(x)):
            y = S.run(None, {nin: x[i:i+1]})[0][0]; b = B.run(None, {nin: x[i:i+1]})[0][0]
            ex = np.exp(b[64:129] - b[64:129].max(0)); p = (ex / ex.sum(0))[:64]
            md = max(md, float(np.abs(y[:64] - b[:64]).max()), float(np.abs(y[64:128] - p).max()), float(np.abs(y[128] - 1/(1+np.exp(-b[129]))).max()))
            outs.append(y)
        np.stack(outs).astype(np.float32).tofile(os.path.join(OUT, f"onnx_out129_{ch}ch_fp32.bin"))
        r["frames"] = len(x); r["maxdiff_vs_base_plus_numpy_post"] = md
    info[f"{ch}ch"] = r; print(ch, r)
json.dump(info, open(os.path.join(OUT, "mk960soft_info.json"), "w"), indent=1)
