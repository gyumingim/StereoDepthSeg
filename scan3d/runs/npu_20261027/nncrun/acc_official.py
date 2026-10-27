#!/usr/bin/env python3
"""nncrun 2026-10-28: XFeat accuracy against the OFFICIAL XFeat post-processing (modules/xfeat.py detectAndCompute:
get_kpts_heatmap, NMS thr 0.05 k5, score = nearest K * bilinear H, top 1024, bicubic desc + L2) on the PC fp32 reference.
 (a) phone raw output -> same official post (layout 130: feats|kpt logits|heat logit; soft129: feats|probs|sigmoid heat;
     nms128: score is made inside the model -> official post not applicable, skipped)
 (b) phone C++ keypoint file (pipe=1 out: n, xy, desc) -> what the phone pipeline really outputs
Metric: overlap = share of reference top-1024 points with a test point within 1 px; cos = median cosine of the
reference desc and the desc of the nearest test point (matched ones only).
venv_ffs python:  python acc_official.py OUT.json [raw:LAYOUT:FILE | kp:FILE] ...   (frames: 13 eval)"""
import json, sys, time
import numpy as np, torch, torch.nn.functional as F
sys.path.insert(0, "/home/system/scanner/tmp_ovxfeat/xfeat_src")
from modules.xfeat import XFeat
from modules.interpolator import InterpolateSparse2d
torch.set_grad_enabled(False)
xf = XFeat(weights=None); xf.dev = torch.device("cpu")
H, W = 352, 640
ref = np.fromfile("/home/system/scanner/tmp_npu/nncrun_data/ref_eval_fp32.bin", np.float32).reshape(-1, 130, 44, 80)
near, bil, bic = InterpolateSparse2d("nearest"), InterpolateSparse2d("bilinear"), InterpolateSparse2d("bicubic")


def official(M, Kh, Hm, n=1024):
    mk = xf.NMS(Kh, threshold=0.05, kernel_size=5)
    sc = (near(Kh, mk, H, W) * bil(Hm, mk, H, W)).squeeze(-1)
    sc[torch.all(mk == 0, dim=-1)] = -1
    idx = torch.argsort(-sc)[:, :n]
    mk = torch.stack([torch.gather(mk[..., 0], -1, idx), torch.gather(mk[..., 1], -1, idx)], -1)
    s = torch.gather(sc, -1, idx)
    de = F.normalize(bic(F.normalize(M, dim=1), mk, H, W), dim=-1)
    v = s[0] > 0
    return mk[0][v].float().numpy(), de[0][v].numpy()


def from130(o):
    t = torch.from_numpy(o[None].copy())
    return official(t[:, :64], xf.get_kpts_heatmap(t[:, 64:129]), torch.sigmoid(t[:, 129:130]))


def fromsoft(o):
    t = torch.from_numpy(o[None].copy())
    p = t[:, 64:128]
    Kh = p.permute(0, 2, 3, 1).reshape(1, 44, 80, 8, 8).permute(0, 1, 3, 2, 4).reshape(1, 1, H, W)  # = get_kpts_heatmap w/o softmax
    return official(t[:, :64], Kh, t[:, 128:129])


def cmp(a, da, b, db):
    d = np.sqrt(((a[:, None] - b[None]) ** 2).sum(-1)); j = d.argmin(1); ok = d.min(1) <= 1.0
    return float(ok.mean()), float(np.median((da[ok] * db[j[ok]]).sum(1))) if ok.any() else float("nan")


R = [from130(r) for r in ref]
res = {"made": time.strftime("%F %T"), "ref": "PC ORT fp32 original XFeat + official post", "items": {}}
for spec in sys.argv[2:]:
    kind, rest = spec.split(":", 1)
    ov, cs = [], []
    if kind == "raw":
        lay, fn = rest.split(":", 1)
        if lay == "nms128":
            res["items"][spec] = "official post not applicable (NMS inside the model)"; continue
        C = {"130": 130, "soft129": 129}[lay]
        ph = np.fromfile(fn, np.float32).reshape(-1, C, 44, 80)
        for i in range(len(ref)):
            a, da = R[i]; b, db = (from130 if C == 130 else fromsoft)(ph[i])
            q = cmp(a, da, b, db); ov.append(q[0]); cs.append(q[1])
    else:
        bb = open(rest, "rb").read(); o = 0
        for i in range(len(ref)):
            n = int(np.frombuffer(bb, np.int32, 1, o)[0]); o += 4
            xy = np.frombuffer(bb, np.int32, 2 * n, o).reshape(n, 2).astype(np.float32); o += 8 * n
            de = np.frombuffer(bb, np.float32, 64 * n, o).reshape(n, 64); o += 256 * n
            q = cmp(R[i][0], R[i][1], xy, de); ov.append(q[0]); cs.append(q[1])
    res["items"][spec] = {"overlap_med": float(np.median(ov)), "overlap_min": float(min(ov)), "cos_med": float(np.median(cs))}
print(json.dumps(res["items"], indent=1))
json.dump(res, open(sys.argv[1], "w"), indent=1)
