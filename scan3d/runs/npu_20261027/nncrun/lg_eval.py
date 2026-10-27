#!/usr/bin/env python3
"""nncrun 2026-10-28: LighterGlue NPU phone results vs PC fp32 original LighterGlue (6 layers, same N points).
Inputs: phone exec output OE [13,N,2N+2] f32 (from lg_x13[_nN].bin), phone pipeline matches ML (C++ pack + post),
lg_ref[_nN].npz (lg_prep.py). Also PC fp32 of the same cut model (lgnpu.LGNpu tanh, NL layers; LGLN=native env for the
native-LayerNorm builds) to split cut vs NPU error.
Per pair: agreement = |ref & test| / |ref|, match counts, F-RANSAC (cv2 FM_RANSAC 1 px, 0.999, seed 0) inlier counts,
NPU-only matches (not in ref) and the share of them consistent with the ref F (Sampson < 1 px) - inlier counts alone
grow with the number of matches and are not a quality gain.
venv_ffs python:  python lg_eval.py OE.bin ML.bin OUT.json [N=256] [NL=3]"""
import json, sys, time
import numpy as np, torch, cv2
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/npu_20261027")
sys.path.insert(0, "/mnt/c/Users/a3162/scan3d_handoff/npu_results/lgnpu")
from lg_clean import load_net  # noqa
import lgnpu  # noqa
torch.set_grad_enabled(False)
D = "/home/system/scanner/tmp_npu/nncrun_lg"
N = int(sys.argv[4]) if len(sys.argv) > 4 else 256
NL = int(sys.argv[5]) if len(sys.argv) > 5 else 3
SUF = "" if N == 256 else f"_n{N}"
R = np.load(f"{D}/lg_ref{SUF}.npz")
ph = np.fromfile(sys.argv[1], np.float32).reshape(-1, N, 2 * N + 2)
b = open(sys.argv[2], "rb").read(); o = 0; pipe = []
for i in range(len(ph)):
    n = int(np.frombuffer(b, np.int32, 1, o)[0]); o += 4
    pipe.append({tuple(p) for p in np.frombuffer(b, np.int32, 2 * n, o).reshape(n, 2)}); o += 8 * n
x = np.fromfile(f"{D}/lg_x13{SUF}.bin", np.float32).reshape(-1, 1, N, 512)
net = load_net()
cut = lgnpu.LGNpu(net, "tanh", NL).eval()


def fit(m, i):
    if len(m) < 8:
        return 0, None
    m = np.array(sorted(m))
    cv2.setRNGSeed(0)
    F, mask = cv2.findFundamentalMat(R["k0"][i][m[:, 0]], R["k1"][i][m[:, 1]], cv2.FM_RANSAC, 1.0, 0.999)
    return (int(mask.sum()) if mask is not None else 0), F


def sampson_ok(m, i, F, thr=1.0):
    if F is None or len(m) == 0:
        return float("nan")
    m = np.array(sorted(m))
    a = np.c_[R["k0"][i][m[:, 0]], np.ones(len(m))]; bb = np.c_[R["k1"][i][m[:, 1]], np.ones(len(m))]
    Fa, Fb = a @ F.T, bb @ F
    num = (bb * Fa).sum(1) ** 2
    den = Fa[:, 0] ** 2 + Fa[:, 1] ** 2 + Fb[:, 0] ** 2 + Fb[:, 1] ** 2
    return float((np.sqrt(num / den) < thr).mean())


rows = []
for i in range(len(ph)):
    ref = {tuple(p) for p in R[f"m{i}"]}
    npu = lgnpu.post(ph[i:i + 1], N)
    pcc = lgnpu.post(cut(torch.from_numpy(x[i])).numpy(), N)
    ag = lambda t: len(ref & t) / max(1, len(ref))
    ir, Fr = fit(ref, i)
    rows.append({"pair": i, "ref_n": len(ref), "npu_n": len(npu), "pipe_n": len(pipe[i]), "pc_cut_n": len(pcc),
                 "agree_npu": ag(npu), "agree_pipe": ag(pipe[i]), "agree_pc_cut": ag(pcc), "pipe_eq_npu": pipe[i] == npu,
                 "inl_ref": ir, "inl_npu": fit(npu, i)[0], "inl_pc_cut": fit(pcc, i)[0],
                 "npu_only_n": len(npu - ref), "ref_missed_n": len(ref - npu),
                 "npu_only_ok_refF": sampson_ok(npu - ref, i, Fr), "ref_ok_refF": sampson_ok(ref, i, Fr)})
med = lambda k: float(np.median([r[k] for r in rows]))
summ = {k: med(k) for k in ("ref_n", "npu_n", "pc_cut_n", "agree_npu", "agree_pc_cut", "inl_ref", "inl_npu", "inl_pc_cut")}
summ["agree_npu_mean"] = float(np.mean([r["agree_npu"] for r in rows]))
summ["agree_pc_cut_mean"] = float(np.mean([r["agree_pc_cut"] for r in rows]))
summ["agree_npu_min"] = float(min(r["agree_npu"] for r in rows))
summ["npu_only_total"] = int(sum(r["npu_only_n"] for r in rows))
summ["ref_missed_total"] = int(sum(r["ref_missed_n"] for r in rows))
summ["npu_only_ok_refF_pooled"] = float(np.nansum([r["npu_only_ok_refF"] * r["npu_only_n"] for r in rows if r["npu_only_n"]])
                                        / max(1, summ["npu_only_total"]))
summ["ref_ok_refF_med"] = med("ref_ok_refF")
summ["pipe_eq_npu_pairs"] = int(sum(r["pipe_eq_npu"] for r in rows))
res = {"made": time.strftime("%F %T"), "N": N, "NL_cut": NL,
       "ref": f"LGClean fp32 6 layers exact GELU, {N} pts (== kornia LightGlue, no pruning)", "median": summ, "pairs": rows}
print(json.dumps(summ, indent=1))
json.dump(res, open(sys.argv[3], "w"), indent=1)
