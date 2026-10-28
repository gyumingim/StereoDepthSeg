#!/usr/bin/env python3
"""npuopt 10-28 (verifier fixes): official XFeat post at 960x544 (torch, y>=540 dropped) vs
 (a) C++ xfpost_off.h keypoint file on the same PC fp32 raw outputs (13 live-path eval frames)
 (b) Lab fp16 simulator golden vs PC fp32 on the golden (random) input, both through the official post (pre-registered >= 99 %).
venv_ffs:  python score960.py OUT.json CPPKP.bin"""
import json, sys, time
import numpy as np, torch, torch.nn.functional as F
sys.path.insert(0, "/home/system/scanner/tmp_ovxfeat/xfeat_src")
from modules.xfeat import XFeat
from modules.interpolator import InterpolateSparse2d
torch.set_grad_enabled(False)
xf = XFeat(weights=None); xf.dev = torch.device("cpu")
near, bil, bic = InterpolateSparse2d("nearest"), InterpolateSparse2d("bilinear"), InterpolateSparse2d("bicubic")
H, W, T = 544, 960, "/home/system/scanner/tmp_npu/xf960"


def off(o, ymax, n=1024):
    t = torch.from_numpy(o[None].copy())
    Kh = xf.get_kpts_heatmap(t[:, 64:129]); Hm = torch.sigmoid(t[:, 129:130]); M = t[:, :64]
    mk = xf.NMS(Kh, threshold=0.05, kernel_size=5)
    sc = (near(Kh, mk, H, W) * bil(Hm, mk, H, W)).squeeze(-1)
    sc[torch.all(mk == 0, dim=-1)] = -1
    sc[mk[..., 1] >= ymax] = -1
    idx = torch.argsort(-sc)[:, :n]
    mk = torch.stack([torch.gather(mk[..., 0], -1, idx), torch.gather(mk[..., 1], -1, idx)], -1)
    s = torch.gather(sc, -1, idx)
    de = F.normalize(bic(F.normalize(M, dim=1), mk, H, W), dim=-1)
    v = s[0] > 0
    return mk[0][v].float().numpy(), de[0][v].numpy()


def cmp(a, da, b, db):
    d = np.sqrt(((a[:, None] - b[None]) ** 2).sum(-1)); j = d.argmin(1); ok = d.min(1) <= 1.0
    return float(ok.mean()), float(np.median((da[ok] * db[j[ok]]).sum(1)))


ref = np.fromfile(f"{T}/ref960_raw130_fp32.bin", np.float32).reshape(-1, 130, 68, 120)
bb = open(sys.argv[2], "rb").read(); o = 0; ov, cs, nn = [], [], []
for i in range(len(ref)):
    n = int(np.frombuffer(bb, np.int32, 1, o)[0]); o += 4
    xy = np.frombuffer(bb, np.int32, 2 * n, o).reshape(n, 2).astype(np.float32); o += 8 * n
    de = np.frombuffer(bb, np.float32, 64 * n, o).reshape(n, 64); o += 256 * n
    a, da = off(ref[i], 540); q = cmp(a, da, xy, de); ov.append(q[0]); cs.append(q[1]); nn.append((len(a), n))
res = {"made": time.strftime("%F %T"), "cpp_vs_official_960": {"overlap_med": float(np.median(ov)), "overlap_min": min(ov),
       "cos_med": float(np.median(cs)), "n_ref_cpp": nn}, "golden": {}}
for b in ("x960fpmc", "x960f1mc"):
    g = np.fromfile(f"{T}/gold_sim_{b}.bin", np.float32).reshape(130, 68, 120)
    p = np.fromfile(f"{T}/gold_pc_{b}.bin", np.float32).reshape(130, 68, 120)
    a, da = off(p, 544); c, dc = off(g, 544); q = cmp(a, da, c, dc)
    res["golden"][b] = {"overlap": q[0], "cos_med": q[1], "n_pc": len(a), "n_sim": len(c), "input": "Lab random db uniform [0,1) (not a real image)"}
print(json.dumps(res, indent=1)); json.dump(res, open(sys.argv[1], "w"), indent=1)
