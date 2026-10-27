#!/usr/bin/env python3
"""nncrun 2026-10-28: XFeat ONNX variants with part of the post-processing inside the model (for NPU; compile later).
Base: lab_realcalib_xfeat3ch/xfeat_backbone_3ch_noadd_op13_cat1out.onnx (input image 1x3x352x640 = x, output 1x130x44x80).
 A  xfeat3ch_postnms : out 1x128x44x80 = [feats 0-63 | score cells 64-127]. score = K * sigmoid(heat) where K is a 5x5
    local max (MaxPool 5, pad 2) and K > 0.05, else 0; K = DepthToSpace(8, DCR)(softmax65(kpts)[:64]); score cell layout =
    SpaceToDepth(8) so channel c of cell (cy,cx) = pixel (cy*8 + c//8, cx*8 + c%8). CPU left: top-1024 over score > 0,
    64-d desc at the cell + L2.
 B  xfeat3ch_postsoft: out 1x129x44x80 = [feats 0-63 | softmax probs 64-127 (dustbin dropped) | sigmoid(heat) 128].
    CPU left: 8x8 shuffle, NMS, top-k, desc (no exp on CPU).
Check on PC (ORT fp32, 13 eval frames): CPU remainder of A / B vs acc_lib.topk on the base model output must give the
same top-1024 keypoints (set equality and order), desc identical.
  python make_post_onnx.py OUTDIR"""
import json, sys, time
import numpy as np, onnx, onnxruntime as ort
from onnx import helper as h, TensorProto as TP, numpy_helper as nh
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
from acc_lib import topk
BASE = "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lab_realcalib_xfeat3ch/xfeat_backbone_3ch_noadd_op13_cat1out.onnx"
O = sys.argv[1]
H, W, h8, w8 = 352, 640, 44, 80


def const(name, arr, dt=np.int64):
    return h.make_node("Constant", [], [name], value=nh.from_array(np.array(arr, dt), name))


def sl(name, src, a, b):
    return [const(name + "_s", [a]), const(name + "_e", [b]), const(name + "_ax", [1]),
            h.make_node("Slice", [src, name + "_s", name + "_e", name + "_ax"], [name])]


def build(kind):
    m = onnx.load(BASE)
    g = m.graph
    src = g.output[0].name
    nodes = []
    nodes += sl("p_feats", src, 0, 64) + sl("p_kl", src, 64, 129) + sl("p_hl", src, 129, 130)
    nodes += [h.make_node("Softmax", ["p_kl"], ["p_soft"], axis=1)] + sl("p_prob", "p_soft", 0, 64)
    nodes += [h.make_node("Sigmoid", ["p_hl"], ["p_heat"])]
    if kind == "B":
        nodes += [h.make_node("Concat", ["p_feats", "p_prob", "p_heat"], ["out129"], axis=1)]
        outs = [h.make_tensor_value_info("out129", TP.FLOAT, [1, 129, h8, w8])]
    else:
        nodes += [h.make_node("DepthToSpace", ["p_prob"], ["p_K"], blocksize=8, mode="DCR"),
                  h.make_node("MaxPool", ["p_K"], ["p_mp"], kernel_shape=[5, 5], pads=[2, 2, 2, 2], strides=[1, 1]),
                  h.make_node("Equal", ["p_K", "p_mp"], ["p_eq"]),
                  const("p_thr", 0.05, np.float32), h.make_node("Greater", ["p_K", "p_thr"], ["p_gt"]),
                  h.make_node("And", ["p_eq", "p_gt"], ["p_keep"]),
                  h.make_node("Cast", ["p_keep"], ["p_keepf"], to=TP.FLOAT),
                  h.make_node("SpaceToDepth", ["p_K"], ["p_Kc"], blocksize=8),
                  h.make_node("SpaceToDepth", ["p_keepf"], ["p_keepc"], blocksize=8),
                  h.make_node("Mul", ["p_Kc", "p_heat"], ["p_sc0"]),
                  h.make_node("Mul", ["p_sc0", "p_keepc"], ["p_score"]),
                  h.make_node("Concat", ["p_feats", "p_score"], ["out128"], axis=1)]
        outs = [h.make_tensor_value_info("out128", TP.FLOAT, [1, 128, h8, w8])]
    g.node.extend(nodes)
    del g.output[:]
    g.output.extend(outs)
    onnx.checker.check_model(m)
    return m


def cpu_A(o, n=1024):  # o (128,44,80)
    sc = o[64:]  # (64,44,80) cell layout
    c, cy, cx = np.nonzero(sc > 0)
    s = sc[c, cy, cx]
    y, x = cy * 8 + c // 8, cx * 8 + c % 8
    idx = y * W + x
    order = np.lexsort((idx, -s))[:n]  # score desc, raster index asc (= numpy stable argsort on raster list)
    return np.stack([x[order], y[order]], 1).astype(np.float32)


def cpu_B(o, n=1024, thr=0.05):
    P = o[64:128]
    K = P.reshape(8, 8, h8, w8).transpose(2, 0, 3, 1).reshape(H, W)
    from numpy.lib.stride_tricks import sliding_window_view
    mx = sliding_window_view(np.pad(K, 2, constant_values=-1), (5, 5)).max((-1, -2))
    ys, xs = np.nonzero((K == mx) & (K > thr))
    s = K[ys, xs] * o[128][ys // 8, xs // 8]
    od = np.argsort(-s, kind="stable")[:n]
    return np.stack([xs[od], ys[od]], 1).astype(np.float32)


ev = np.repeat(np.load("/home/system/scanner/tmp_npu/data/eval_640x352_norm.npy").astype(np.float32), 3, axis=1)
Sb = ort.InferenceSession(BASE, providers=["CPUExecutionProvider"])
sig = lambda z: 1 / (1 + np.exp(-z))
res = {"made": time.strftime("%F %T"), "base": BASE, "frames": len(ev)}
for kind, nm, fn in (("A", "xfeat3ch_postnms", cpu_A), ("B", "xfeat3ch_postsoft", cpu_B)):
    m = build(kind)
    p = f"{O}/{nm}.onnx"
    onnx.save(m, p)
    S = ort.InferenceSession(p, providers=["CPUExecutionProvider"])
    same_set, same_order, desc_eq, nk = [], [], [], []
    for i in range(len(ev)):
        b = Sb.run(None, {"image": ev[i:i + 1]})[0][0]
        o = S.run(None, {"image": ev[i:i + 1]})[0][0]
        ref = topk(b[:64], b[64:129], sig(b[129]))
        got = fn(o)
        same_set.append(set(map(tuple, ref.astype(int))) == set(map(tuple, got.astype(int))))
        same_order.append(bool(len(ref) == len(got) and (ref == got).all()))
        desc_eq.append(bool(np.array_equal(o[:64], b[:64])))
        nk.append(len(got))
    ops = sorted({n.op_type for n in m.graph.node[-40:]})
    res[nm] = {"file": p, "out": [o.name for o in m.graph.output], "shape": list(S.run(None, {"image": ev[:1]})[0].shape),
               "same_set_frames": int(sum(same_set)), "same_order_frames": int(sum(same_order)),
               "feats_identical_frames": int(sum(desc_eq)), "n_kp": nk, "post_ops": ops}
print(json.dumps(res, indent=1))
json.dump(res, open(f"{O}/post_onnx_check.json", "w"), indent=1)
