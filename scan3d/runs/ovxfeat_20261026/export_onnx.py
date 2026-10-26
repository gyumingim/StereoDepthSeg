#!/usr/bin/env python3
"""ovxfeat 2026-10-26: export XFeat backbone and LighterGlue (fixed shapes) to ONNX for the phone benchmark.

XFeat: XFeatModel(x 1x1xHxW float) -> feats 1x64xH/8xW/8, kpt logits 1x65xH/8xW/8, heatmap 1x1xH/8xW/8
       (NMS / top-k / descriptor sampling are cheap post-processing, done outside the network on the phone).
LighterGlue (fixed N points per image, no pruning / early stop = all 6 layers on all points):
       (kpts0 1xNx2 normalised like kornia normalize_keypoints, kpts1, desc0 1xNx64, desc1) ->
       m0 (best column per row), m1 (best row per column), s0 = exp(max row log-assignment)  (mutual check + threshold outside)
  python3 export_onnx.py OUTDIR
"""
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn

XF_SRC = os.environ.get("XF_SRC", "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, XF_SRC)
import warnings  # noqa: E402
warnings.filterwarnings("ignore")
from modules.model import XFeatModel  # noqa: E402

OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
torch.set_grad_enabled(False)


class LGFixed(nn.Module):
    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, k0, k1, d0, d1):
        n = self.net
        d0 = n.input_proj(d0)
        d1 = n.input_proj(d1)
        e0 = n.posenc(k0)
        e1 = n.posenc(k1)
        for i in range(n.conf.n_layers):
            d0, d1 = n.transformers[i](d0, d1, e0, e1)
        scores, _ = n.log_assignment[n.conf.n_layers - 1](d0, d1)
        s = scores[:, :-1, :-1]
        mx0 = s.max(2)
        m1 = s.argmax(1)
        return mx0.indices, m1, mx0.values.exp()


def main():
    import onnx
    import onnxruntime as ort
    info = []
    m = XFeatModel().eval()
    m.load_state_dict(torch.load(os.path.join(XF_SRC, "weights", "xfeat.pt"), map_location="cpu"))
    for (H, W) in ([] if os.environ.get("SKIP_XF") else [(256, 480), (352, 640), (544, 960)]):
        x = torch.rand(1, 1, H, W)
        fn = os.path.join(OUT, f"xfeat_{W}x{H}.onnx")
        torch.onnx.export(m, (x,), fn, input_names=["image"], output_names=["feats", "keypoints", "heatmap"], opset_version=17,
                          dynamo=False)
        ref = m(x)
        s = ort.InferenceSession(fn, providers=["CPUExecutionProvider"])
        out = s.run(None, {"image": x.numpy()})
        err = max(float(np.abs(a - b.numpy()).max()) for a, b in zip(out, ref))
        info.append(f"{fn} max|onnx-torch| {err:.2e} size {os.path.getsize(fn)/1e6:.2f} MB")
        print(info[-1], flush=True)
    sys.path.insert(0, XF_SRC)
    from modules.lighterglue import LighterGlue
    lg = LighterGlue(weights=os.path.join(XF_SRC, "weights", "xfeat-lighterglue.pt"))
    net = lg.net.cpu().eval()
    for att in net.modules():  # SDPA path (no flash-attn on CPU); keep math identical
        if hasattr(att, "enable_flash"):
            att.enable_flash = True
    w = LGFixed(net).eval()
    for N in [256, 512, 1024, 2048, 4096]:
        k0 = torch.rand(1, N, 2) * 2 - 1
        k1 = torch.rand(1, N, 2) * 2 - 1
        d0 = torch.nn.functional.normalize(torch.randn(1, N, 64), dim=-1)
        d1 = torch.nn.functional.normalize(torch.randn(1, N, 64), dim=-1)
        fn = os.path.join(OUT, f"lighterglue_{N}.onnx")
        torch.onnx.export(w, (k0, k1, d0, d1), fn, input_names=["kpts0", "kpts1", "desc0", "desc1"],
                          output_names=["m0", "m1", "s0"], opset_version=18, dynamo=True)
        ref = w(k0, k1, d0, d1)
        s = ort.InferenceSession(fn, providers=["CPUExecutionProvider"])
        t0 = time.perf_counter()
        out = s.run(None, {"kpts0": k0.numpy(), "kpts1": k1.numpy(), "desc0": d0.numpy(), "desc1": d1.numpy()})
        dt = (time.perf_counter() - t0) * 1000
        same = float((out[0] == ref[0].numpy()).mean())
        info.append(f"{fn} m0 agree {same:.4f} s0 max err {float(np.abs(out[2]-ref[2].numpy()).max()):.2e} "
                    f"PC ORT CPU {dt:.0f} ms size {os.path.getsize(fn)/1e6:.2f} MB")
        print(info[-1], flush=True)
    open(os.path.join(OUT, "export_info.txt"), "w").write(time.strftime("%F %T") + "\n" + "\n".join(info) + "\n")


if __name__ == "__main__":
    main()
