#!/usr/bin/env python3
"""gsmap 2026-11-27 W9: full-res (1920x1080) texture detail ruler = audit9b logic (true-detail slope: regression of the render's
high-pass on the photo's high-pass in textured regions) + audit9 fake-gradient-in-flat-regions + a chroma error, for N GLBs on the SAME mesh.
Frames = held-out partner frames, every 6th (the 27 of audit9b), from w8/rjudge2/photo_perframe.npz. Poses = the map's own BA set.
  python gsm_w9detail.py OUT.json POSES(bap|ba2) NAME=GLB [NAME=GLB ...]      first NAME = baseline; diffs vs every earlier map
Mask = ray hit & valid lens area, eroded 15 px (audit9b). tex = photo gradient >= p80 (dilated 5 px), tex5 = >= p95, flat = 9x9 mean photo
gradient <= p50 (audit9). slope_tex = true-detail amplitude (must not drop); flat_grad / flat_hf_rms = fake texture in flat regions (lower = better);
chroma_err = mean |a,b| Lab error vs photo on the mask (hue shift). n = 1 recording; se = std of the per-frame paired diff / sqrt(n)."""
import json, sys, time
from pathlib import Path
import cv2, numpy as np
cv2.setNumThreads(8)
G = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d"
sys.path.insert(0, str(G / "gsmap"))
from gsm_texture import load_frames  # noqa: E402
from gsm_eval2 import load_mesh  # noqa: E402
import open3d as o3d  # noqa: E402

V2 = G / "runs/gsmap_20261125_121801_v2"
PO = dict(bap=V2 / "ba_partners", ba2=V2 / "ba_partners2")
hp = lambda x, s=2.0: x - cv2.GaussianBlur(x, (0, 0), s)
grad = lambda x: np.hypot(cv2.Sobel(x, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(x, cv2.CV_32F, 0, 1, ksize=3)) / 8
gry = lambda x: cv2.cvtColor(x.astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)


def main():
    out, poses = Path(sys.argv[1]), sys.argv[2]; specs = [s.split("=", 1) for s in sys.argv[3:]]
    assert not out.exists(), "never overwrite"
    t0 = time.time()
    z = np.load(V2 / "w8/rjudge2/photo_perframe.npz", allow_pickle=True)
    part = [int(f) for f, k in zip(z["frames"], z["kind"]) if k == "partner"]; sel0 = part[::6]
    pa = np.load(PO[poses] / "poses_all.npz"); TF = {int(f): pa["T"][k] for k, f in enumerate(pa["frames"]) if pa["src"][k] == "ba"}
    sel = [f for f in sel0 if f in TF]
    dump = PO[poses] / "run/dump_ba"
    imgs, Kr1, valid = load_frames(dump, V2 / "rec", sel, "lensonly")
    W, H = 1920, 1080; uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    ray = np.stack([(uu - Kr1[0, 2]) / Kr1[0, 0], (vv - Kr1[1, 2]) / Kr1[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    PH = []
    for im in imgs:
        ph = gry(im); gp = grad(ph); PH.append((im.astype(np.float32), ph, gp, hp(ph), cv2.cvtColor(im, cv2.COLOR_BGR2LAB).astype(np.float32)))
    R = {}
    for nm, g in specs:
        V, F, UV, tex = load_mesh(g); Ah, Aw = tex.shape[:2]
        sc = o3d.t.geometry.RaycastingScene(nthreads=8); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
        rows = []
        for i, f in enumerate(sel):
            Tc = TF[f]; r = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], ray.shape), ray @ Tc[:3, :3].T], 1).astype(np.float32)), nthreads=8)
            pid = r["primitive_ids"].numpy(); hit = pid != 4294967295; bu = r["primitive_uvs"].numpy(); q = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
            uvp = b0[:, None] * UV[F[q, 0]] + b1[:, None] * UV[F[q, 1]] + b2[:, None] * UV[F[q, 2]]
            im = np.zeros((H * W, 3), np.uint8)
            im[hit] = tex[np.clip((uvp[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uvp[:, 0] * Aw).astype(int), 0, Aw - 1)]; im = im.reshape(H, W, 3)
            ph3, ph, gp, hpp, lab = PH[i]
            m = cv2.erode((hit.reshape(H, W) & valid).astype(np.uint8), np.ones((15, 15), np.uint8)).astype(bool)
            tx = m & (cv2.dilate((gp >= np.percentile(gp[m], 80)).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0)
            tx5 = m & (cv2.dilate((gp >= np.percentile(gp[m], 95)).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0)
            fl = m & (cv2.blur(gp, (9, 9)) <= np.percentile(gp[m], 50))
            gr = gry(im); h_ = hp(gr); gg = grad(gr); o = {}
            for tag, t_ in (("tex", tx), ("tex5", tx5)):
                a_ = hpp[t_] - hpp[t_].mean(); b_ = h_[t_] - h_[t_].mean(); sl = float((a_ * b_).sum() / (a_ * a_).sum())
                o[f"slope_{tag}"] = sl; o[f"resid_rms_{tag}"] = float(np.sqrt(((b_ - sl * a_) ** 2).mean()))
                o[f"grad_ratio_{tag}"] = float(gg[t_].mean() / gp[t_].mean())
            o["flat_grad"] = float(gg[fl].mean()); o["flat_hf_rms"] = float(np.sqrt((h_[fl] ** 2).mean()))
            o["psnr1920"] = float(-10 * np.log10((((im.astype(np.float32) - ph3) / 255) ** 2)[m].mean()))
            lr = cv2.cvtColor(im, cv2.COLOR_BGR2LAB).astype(np.float32)
            o["chroma_err"] = float(np.abs(lr[..., 1:] - lab[..., 1:])[m].mean())
            o["mask_px"] = int(m.sum()); rows.append(o)
        R[nm] = rows; del sc, tex; print(nm, round(time.time() - t0), flush=True)
    keys = list(R[specs[0][0]][0])
    A = {nm: {k: np.array([r_[k] for r_ in R[nm]]) for k in keys} for nm in R}
    res = dict(made=time.strftime("%F %T"), script="gsm_w9detail.py", poses=str(PO[poses] / "poses_all.npz"), images_dump=str(dump),
               maps={n: g for n, g in specs}, n_frames=len(sel), frames=sel, n_recordings=1,
               mean={nm: {k: round(float(v.mean()), 4) for k, v in A[nm].items()} for nm in A}, diff={})
    names = [n for n, _ in specs]
    for bi, b in enumerate(names):
        for m_ in names[bi + 1:]:
            e = {}
            for k in keys:
                if k == "mask_px":
                    continue
                d = A[m_][k] - A[b][k]
                e[k] = dict(mean=round(float(d.mean()), 4), se=round(float(d.std(ddof=1) / np.sqrt(len(d))), 4),
                            rel_pct=round(float(100 * d.mean() / A[b][k].mean()), 2), n_pos=int((d > 0).sum()))
            res["diff"][f"{m_}-{b}"] = e
    out.write_text(json.dumps(res, indent=1)); print(json.dumps(res["mean"]))


if __name__ == "__main__":
    main()
