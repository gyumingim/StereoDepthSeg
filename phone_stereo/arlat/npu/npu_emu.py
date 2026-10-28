#!/usr/bin/env python3
"""appnpu 2026-10-28: PC stand-in for the phone NPU models, used ONLY by the PC test build xfnpu_pc (protocol / pre-post /
bridge-integration check; torch fp32, not the int16/fp16 NNC on the phone).
  pc:xfsoft  in 1x1x544x960 float (normalised image)  -> out 129 x 68 x 120 (feats | softmax probs 64 | sigmoid heat)
             = official XFeatModel (verlab accelerated_features, Apache-2.0) + softmax / drop dustbin
  pc:xfoff   same input -> out 130 (feats | 65 logits | heat logit)
  pc:lg512   in 1x512x512 (lgnpu pack) -> out 512 x 1026 (A | BT | z0 | z1) = lgnpu.LGNpu(tanh, 6 layers), same graph as
             the L6_a16mc ONNX before quantisation
stdin/stdout: 'L' u32 len path -> u32 in_bytes u32 out_bytes ; 'X' i32 model + in bytes -> out bytes ; 'Q' exit"""
import os, struct, sys, importlib.util
import numpy as np
import torch
import torch.nn.functional as F

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
torch.set_grad_enabled(False)
torch.set_num_threads(int(os.environ.get("EMU_THREADS", "4")))
LGNPU = os.environ.get("LGNPU_PY", "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu/lgnpu.py")
fin, fout = sys.stdin.buffer, sys.stdout.buffer


def rd(n):
    b = fin.read(n)
    if len(b) != n:
        raise EOFError
    return b


models = []
xfnet = None
while True:
    try:
        c = rd(1)
    except EOFError:
        break
    if c == b"Q":
        break
    if c == b"L":
        n, = struct.unpack("<I", rd(4))
        p = rd(n).decode()
        if p in ("pc:xfsoft", "pc:xfoff"):
            if xfnet is None:
                from modules.xfeat import XFeat
                xfnet = XFeat(weights=os.path.join(XF_SRC, "weights", "xfeat.pt"), top_k=512).net.cpu().eval()
            models.append((p, (1, 1, 544, 960), 129 if p == "pc:xfsoft" else 130))
            fout.write(struct.pack("<II", 544 * 960 * 4, (129 if p == "pc:xfsoft" else 130) * 68 * 120 * 4))
        elif p == "pc:lg512":
            spec = importlib.util.spec_from_file_location("lgnpu", LGNPU)
            lgm = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(lgm)
            net = lgm.load_net()
            models.append((p, lgm.LGNpu(net, "tanh", 6).eval(), None))
            fout.write(struct.pack("<II", 512 * 512 * 4, 512 * 1026 * 4))
        else:
            fout.write(struct.pack("<II", 0, 0))
        fout.flush()
    elif c == b"X":
        m, = struct.unpack("<i", rd(4))
        kind, a, b = models[m]
        if kind.startswith("pc:xf"):
            x = torch.from_numpy(np.frombuffer(rd(544 * 960 * 4), np.float32).reshape(1, 1, 544, 960).copy())
            f, k, h = xfnet(x)
            if kind == "pc:xfsoft":
                o = torch.cat([f, F.softmax(k, 1)[:, :64], h], 1)
            else:
                o = torch.cat([f, k, torch.logit(h.clamp(1e-7, 1 - 1e-7))], 1)
            fout.write(o.numpy().astype(np.float32).tobytes())
        else:
            x = torch.from_numpy(np.frombuffer(rd(512 * 512 * 4), np.float32).reshape(1, 512, 512).copy())
            fout.write(a(x).numpy().astype(np.float32).tobytes())
        fout.flush()
