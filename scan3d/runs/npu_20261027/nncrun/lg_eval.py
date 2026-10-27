#!/usr/bin/env python3
"""nncrun 2026-10-28: LighterGlue NPU (n3t256_a16) phone results vs PC fp32 original LighterGlue (6 layers, same 256 pts).
Inputs: phone exec output oe_lg.bin [13,256,514] f32 (from lg_x13.bin), phone pipeline matches ml_lg_r1.bin (C++ pack +
post), lg_ref.npz (lg_prep.py). Also PC fp32 of the same cut model (lgnpu.LGNpu tanh, 3 layers) to split cut vs NPU error.
Per pair: agreement = |ref & test| / |ref|, match counts, F-RANSAC (cv2 FM_RANSAC 1 px, 0.999, seed 0) inlier counts.
venv_ffs python:  python lg_eval.py OE_LG.bin ML_LG.bin OUT.json"""
import json, sys, time
import numpy as np, torch, cv2
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
sys.path.insert(0, "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu")
from lg_clean import load_net  # noqa
import lgnpu  # noqa
torch.set_grad_enabled(False)
D = "/home/system/scanner/tmp_npu/nncrun_lg"
N = 256
R = np.load(f"{D}/lg_ref.npz")
ph = np.fromfile(sys.argv[1], np.float32).reshape(-1, N, 2 * N + 2)
b = open(sys.argv[2], "rb").read(); o = 0; pipe = []
for i in range(len(ph)):
    n = int(np.frombuffer(b, np.int32, 1, o)[0]); o += 4
    pipe.append({tuple(p) for p in np.frombuffer(b, np.int32, 2 * n, o).reshape(n, 2)}); o += 8 * n
x = np.fromfile(f"{D}/lg_x13.bin", np.float32).reshape(-1, 1, N, 512)
net = load_net()
cut = lgnpu.LGNpu(net, "tanh", 3).eval()


def inl(m, i):
    if len(m) < 8:
        return 0
    m = np.array(sorted(m))
    cv2.setRNGSeed(0)
    F, mask = cv2.findFundamentalMat(R["k0"][i][m[:, 0]], R["k1"][i][m[:, 1]], cv2.FM_RANSAC, 1.0, 0.999)
    return int(mask.sum()) if mask is not None else 0


rows = []
for i in range(len(ph)):
    ref = {tuple(p) for p in R[f"m{i}"]}
    npu = lgnpu.post(ph[i:i + 1], N)
    pcc = lgnpu.post(cut(torch.from_numpy(x[i])).numpy(), N)
    ag = lambda t: len(ref & t) / max(1, len(ref))
    rows.append({"pair": i, "ref_n": len(ref), "npu_n": len(npu), "pipe_n": len(pipe[i]), "pc_cut_n": len(pcc),
                 "agree_npu": ag(npu), "agree_pipe": ag(pipe[i]), "agree_pc_cut": ag(pcc), "pipe_eq_npu": pipe[i] == npu,
                 "inl_ref": inl(ref, i), "inl_npu": inl(npu, i), "inl_pc_cut": inl(pcc, i)})
med = lambda k: float(np.median([r[k] for r in rows]))
summ = {k: med(k) for k in ("ref_n", "npu_n", "pc_cut_n", "agree_npu", "agree_pc_cut", "inl_ref", "inl_npu", "inl_pc_cut")}
summ["agree_npu_min"] = float(min(r["agree_npu"] for r in rows))
summ["pipe_eq_npu_pairs"] = int(sum(r["pipe_eq_npu"] for r in rows))
summ["inl_ratio_npu_over_ref_med"] = float(np.median([r["inl_npu"] / max(1, r["inl_ref"]) for r in rows]))
res = {"made": time.strftime("%F %T"), "ref": "LGClean fp32 6 layers exact GELU, 256 pts (== kornia LightGlue)",
       "median": summ, "pairs": rows}
print(json.dumps(summ, indent=1))
json.dump(res, open(sys.argv[3], "w"), indent=1)
