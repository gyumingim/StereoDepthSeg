#!/usr/bin/env python3
"""ovxfeat: mutual-NN matcher as a fixed-shape ONNX graph (desc0 1xNx64, desc1 1xNx64 -> best j per i, best i per j,
best cosine per i) for the phone benchmark (the mutual check + cos >= 0.82 is a few comparisons outside).
  python3 export_nn.py OUTDIR"""
import os
import sys

import numpy as np
import torch
import torch.nn as nn

OUT = sys.argv[1]


class NN(nn.Module):
    def forward(self, d0, d1):
        S = torch.matmul(d0, d1.transpose(1, 2))
        mx = S.max(2)
        return mx.indices, S.argmax(1), mx.values


def main():
    import onnxruntime as ort
    lines = []
    for N in [int(x) for x in os.environ.get("NN_SIZES", "512,1024,2048,4096").split(",")]:
        d0 = torch.nn.functional.normalize(torch.randn(1, N, 64), dim=-1)
        d1 = torch.nn.functional.normalize(torch.randn(1, N, 64), dim=-1)
        fn = os.path.join(OUT, f"nnmatch_{N}.onnx")
        torch.onnx.export(NN(), (d0, d1), fn, input_names=["desc0", "desc1"], output_names=["m0", "m1", "c0"], opset_version=17,
                          dynamo=False)
        ref = NN()(d0, d1)
        o = ort.InferenceSession(fn, providers=["CPUExecutionProvider"]).run(None, {"desc0": d0.numpy(), "desc1": d1.numpy()})
        lines.append(f"{fn} m0 agree {float((o[0] == ref[0].numpy()).mean()):.4f}")
        print(lines[-1])
    open(os.path.join(OUT, "export_nn_info.txt"), "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
