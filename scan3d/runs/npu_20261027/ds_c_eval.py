#!/usr/bin/env python3
"""npu downstream C (venv_ffs): for each tmp_npu/ds/*.npz: XFeat post (as prep_feats: NMS 5, thr 1e-4, top 512, bicubic
desc) on both frames of each of the 13 eval pairs -> fp32 LighterGlue (lgc_512.onnx, ORT CPU; == kornia) mutual matches
(score > 0.1) -> RANSAC F-matrix inliers (cv2 USAC_MAGSAC... plain RANSAC 1 px, conf 0.999, seed 0).
Reports per variant: mean #matches, mean #inliers, inlier ratio vs ref_fp32.   python3 ds_c_eval.py OUT.json"""
import glob, json, os, sys, time
import cv2, numpy as np, onnxruntime as ort, torch, torch.nn.functional as F
sys.path.insert(0, "/home/system/scanner/tmp_ovxfeat/xfeat_src")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from modules.xfeat import XFeat
from modules.interpolator import InterpolateSparse2d
from acc_lib import lg_post
H, W, N = 352, 640, 512
xf = XFeat(weights=None); xf.dev = torch.device("cpu")
LG = ort.InferenceSession("/home/system/scanner/tmp_npu/onnx/lgc_512.onnx", providers=["CPUExecutionProvider"])
def kd(f, k, h):
    f, k, h = (torch.from_numpy(a[None]) for a in (f, k, h))
    K1h = xf.get_kpts_heatmap(k); mk = xf.NMS(K1h, threshold=1e-4, kernel_size=5)
    sc = (InterpolateSparse2d("nearest")(K1h, mk, H, W) * InterpolateSparse2d("bilinear")(h, mk, H, W)).squeeze(-1)
    kp = mk[0, torch.argsort(-sc)[0, :N]].float()
    de = F.normalize(InterpolateSparse2d("bicubic")(F.normalize(f, dim=1), kp[None], H, W), dim=-1)[0]
    return kp.numpy(), ((kp - torch.tensor([W / 2, H / 2])) / (W / 2)).numpy().astype(np.float32), de.numpy().astype(np.float32)
res = {"made": time.strftime("%F %T"), "pairs": 13, "variants": {}}
for fn in sorted(glob.glob("/home/system/scanner/tmp_npu/ds/*.npz")):
    nm = os.path.basename(fn)[:-4]; z = np.load(fn); n = len(z["f"]) // 2
    M, I = [], []
    for i in range(n):
        a, b = kd(z["f"][i], z["k"][i], z["h"][i]), kd(z["f"][n + i], z["k"][n + i], z["h"][n + i])
        if len(a[0]) < N or len(b[0]) < N:
            M.append(0); I.append(0); continue
        P, z0, z1 = LG.run(None, {"kpts0": a[1][None], "kpts1": b[1][None], "desc0": a[2][None], "desc1": b[2][None]})
        m = sorted(lg_post(P[0], z0[0], z1[0]))
        M.append(len(m))
        if len(m) >= 8:
            p0 = np.float32([a[0][i0] for i0, _ in m]); p1 = np.float32([b[0][j] for _, j in m])
            cv2.setRNGSeed(0)
            _, mask = cv2.findFundamentalMat(p0, p1, cv2.FM_RANSAC, 1.0, 0.999)
            I.append(int(mask.sum()) if mask is not None else 0)
        else:
            I.append(0)
    res["variants"][nm] = {"matches_mean": float(np.mean(M)), "inliers_mean": float(np.mean(I)), "inliers_min": int(np.min(I)), "per_pair_inliers": I}
    print(nm, res["variants"][nm]["matches_mean"], res["variants"][nm]["inliers_mean"], I, flush=True)
json.dump(res, open(sys.argv[1], "w"), indent=1)
