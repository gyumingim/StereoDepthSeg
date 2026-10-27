#!/usr/bin/env python3
"""lgopt 2026-10-27: PC relative timing of LighterGlue variants on real keyframe pairs (dev recording 134550, read only).
Run alone (no other jobs). Points = nq OpenVINS-like queries (150 random-in-image, same per frame) + XFeat top-k, as in xf_server.
Variants: base (= PC xfopt: no pruning, no early exit), prune (width 0.95 always = phone CPU behaviour), prune+exit (depth 0.95),
NN (mutual NN cos >= 0.82). Devices: cuda, cpu (4 threads). -> lg_bench.json. Phone ms = measured LG phone p50
(N=512 66 ms, N=1024 195 ms; earlier measurement) x (cpu variant / cpu base) = ESTIMATE."""
import json, os, sys, time
import numpy as np
import cv2
import torch
import torch.nn.functional as F
XF_SRC = "/home/system/scanner/tmp_ovxfeat/xfeat_src"
sys.path.insert(0, XF_SRC)
import warnings; warnings.filterwarnings("ignore")
from modules.xfeat import XFeat
from modules.lighterglue import LighterGlue
from modules.interpolator import InterpolateSparse2d
REC = "/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261022_134550/cam5.mp4"
PHONE = {512: 66.0, 1024: 195.0}
torch.backends.cuda.matmul.allow_tf32 = False


def frames(idx):
    cap = cv2.VideoCapture(REC)
    out, i, want = {}, 0, set(idx)
    while len(out) < len(want):
        ok, f = cap.read()
        if not ok:
            break
        if i in want:
            g = cv2.cvtColor(cv2.resize(f, (960, 540), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
            out[i] = g
        i += 1
    return out


def feats(xf, bil, img, topk, dev, rng):
    x = torch.from_numpy(img).to(dev).float()[None, None] / 255.0
    x = F.interpolate(x, (360, 640), mode="bilinear", align_corners=False, antialias=True)
    d = xf.detectAndCompute(x, top_k=topk)[0]
    kp, de = d["keypoints"], d["descriptors"]
    xp, rh, rw = xf.preprocess_tensor(x)
    M1, _, _ = xf.net(xp)
    M1 = F.normalize(M1, dim=1)
    q = torch.from_numpy(rng.uniform([0, 0], [960, 540], (150, 2)).astype(np.float32)).to(dev)
    qx = torch.stack([q[:, 0] / 1.5 / rw, q[:, 1] / 1.5 / rh], -1)[None]
    dq = F.normalize(bil(M1, qx, H=xp.shape[-2], W=xp.shape[-1])[0], dim=-1)
    return torch.cat([q, kp * 1.5], 0), torch.cat([dq, de], 0)


def main():
    pairs = [(f, f + 15) for f in range(300, 3300, 300)] + [(f, f + 1500) for f in range(300, 2300, 200)]
    imgs = frames(sorted({a for p in pairs for a in p}))
    pairs = [p for p in pairs if p[0] in imgs and p[1] in imgs]
    res = dict(made=time.strftime("%F %T"), rec=REC, n_pairs=len(pairs), phone_base_ms=PHONE, rows=[])
    for devn in ("cuda", "cpu"):
        dev = torch.device(devn)
        torch.set_num_threads(4)
        xf = XFeat(weights=os.path.join(XF_SRC, "weights", "xfeat.pt"), top_k=1024, detection_threshold=0.05)
        xf.dev = dev; xf.net = xf.net.to(dev).eval()
        lg = LighterGlue(weights=os.path.join(XF_SRC, "weights", "xfeat-lighterglue.pt"))
        lg.net = lg.net.to(dev).eval()
        tab0 = dict(lg.net.pruning_keypoint_thresholds)
        bil = InterpolateSparse2d("bicubic")
        for topk in (512, 1024):
            rng = np.random.default_rng(0)
            with torch.inference_mode():
                F_ = {i: feats(xf, bil, imgs[i], topk, dev, rng) for i in imgs}
            for var, (w, dpt, pm) in dict(base=(0.95, -1, None), prune=(0.95, -1, -1), prune_exit=(0.95, 0.95, -1),
                                          exit=(0.95, 0.95, None), nn=(None, None, None)).items():
                ts, ns = [], []
                for rep in range(2):
                    for a, b in pairs:
                        (pa, da), (pb, db) = F_[a], F_[b]
                        if devn == "cuda":
                            torch.cuda.synchronize()
                        t0 = time.perf_counter()
                        with torch.inference_mode():
                            if var == "nn":
                                S = da @ db.T
                                m12 = S.argmax(1); m21 = S.argmax(0)
                                i0 = torch.arange(len(m12), device=dev)
                                ok = (m21[m12] == i0) & (S.max(1).values >= 0.82)
                                n = int(ok.sum())
                            else:
                                lg.net.conf.width_confidence = w
                                lg.net.conf.depth_confidence = dpt
                                lg.net.pruning_keypoint_thresholds = dict(tab0) if pm is None else {k: pm for k in tab0}
                                out = lg({"keypoints0": pa[None], "keypoints1": pb[None], "descriptors0": da[None], "descriptors1": db[None],
                                          "image_size0": torch.tensor([[960, 540]], device=dev).float(),
                                          "image_size1": torch.tensor([[960, 540]], device=dev).float()}, min_conf=0.1)
                                n = int(out["matches"][0].shape[0])
                        if devn == "cuda":
                            torch.cuda.synchronize()
                        if rep:          # first pass = warm-up
                            ts.append((time.perf_counter() - t0) * 1000)
                            ns.append(n)
                r = dict(dev=devn, topk=topk, var=var, ms_p50=round(float(np.median(ts)), 2), ms_p90=round(float(np.percentile(ts, 90)), 2),
                         matches_med_near=int(np.median(ns[:len(ns) // 2])), matches_med_far=int(np.median(ns[len(ns) // 2:])))
                res["rows"].append(r)
                print(r, flush=True)
    base = {(r["dev"], r["topk"]): r["ms_p50"] for r in res["rows"] if r["var"] == "base"}
    for r in res["rows"]:
        r["rel_base"] = round(r["ms_p50"] / base[(r["dev"], r["topk"])], 3)
        if r["dev"] == "cpu":
            r["phone_ms_est"] = round(PHONE[r["topk"]] * r["rel_base"], 1)
    json.dump(res, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lg_bench.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
