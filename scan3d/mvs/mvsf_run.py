#!/usr/bin/env python3
"""mvs 11-12 method B: MVSFormer++ (Cao et al. ICLR 2024, github.com/maybeLx/MVSFormerPlusPlus, Apache-2.0 code; weights =
authors' OneDrive 'tnt_ckpt/model_best.pth' (BlendedMVS fine-tuned, recommended for Tanks&Temples), no separate licence stated)
on the mvs_prep.py keyframes / fixed BA poses / views.json source views. Code is NOT copied into this repo: --repo points at the clone.

Phase 1 (GPU): per keyframe ref + N src views, images edge-padded to a multiple of 64 in height (540 -> 576, 1080 -> 1088; cy shifted,
  like the repo's T&T loader pads 1080 -> 1088), ImageNet normalisation, depth hypotheses dmin..dmax (views.json, 192 values; model uses
  inverse-depth sampling), bf16 autocast, tmp 5,5,5,1 (repo defaults) -> raw depth + photometric confidence (cropped back) -> OUT/raw/.
Phase 2: repo 'dpcd' dynamic consistency filter (misc/fusion.get_reproj_dynamic + vis_filter_dynamic, dist_base 4, rel_diff_base 1300,
  conf > --conf) with the same source views -> filtered + averaged depth OUT/depth/NAME.npy (0 = invalid) and own fused point cloud
  OUT/fused.ply (all kept pixels, colour from the image).
Attention: flash-attn / xformers not installed -> the repo's own fallback (torch scaled_dot_product_attention with the same
  entropy-invariance softmax scale).
Usage: PYTHONPATH=third_party/mvs_site venv_ffs/bin/python mvsf_run.py PREP OUT --size 960 --repo DIR --ckpt FILE [--n-src 10]"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prep"); ap.add_argument("out")
    ap.add_argument("--size", type=int, default=960)
    ap.add_argument("--repo", default=str(Path.home() / "scanner/third_party/MVSFormerPlusPlus"))
    ap.add_argument("--ckpt", default=str(Path.home() / "scanner/third_party/mvsformerpp_weights/tnt_ckpt/model_best.pth"))
    ap.add_argument("--n-src", type=int, default=10)
    ap.add_argument("--ndepth", type=int, default=192)
    ap.add_argument("--conf", type=float, default=0.5, help="dpcd photometric confidence threshold (repo custom-data example 0.5)")
    ap.add_argument("--skip-infer", action="store_true")
    ap.add_argument("--limit", type=int, default=0, help="only the first N keyframes (smoke test)")
    a = ap.parse_args()
    sys.path.insert(0, a.repo)
    import torch
    import torch.nn.functional as F
    P, out = Path(a.prep), Path(a.out)
    (out / "raw").mkdir(parents=True, exist_ok=True); (out / "depth").mkdir(exist_ok=True)
    C = np.load(P / "cams.npz")
    names = [str(x) for x in C["names"]]
    if a.limit:
        names = names[:a.limit]
    idx = {n: i for i, n in enumerate(str(x) for x in C["names"])}
    K0 = C[f"K{a.size}"].astype(np.float64)
    T = C["T"]
    V = json.loads((P / "views.json").read_text())
    W0, H0 = a.size, a.size * 9 // 16
    Hp = (H0 + 63) // 64 * 64
    pad = (Hp - H0) // 2
    info = json.loads((out / "mvsf_run.json").read_text()) if (out / "mvsf_run.json").exists() else {}
    info.update(made=time.strftime("%Y-%m-%d %H:%M:%S"), prep=str(P), size=a.size, padded=[W0, Hp], pad_top=pad, n_src=a.n_src,
                ndepth=a.ndepth, ckpt=a.ckpt, conf=a.conf, torch=torch.__version__)
    dev = torch.device("cuda")
    mean = np.array([0.485, 0.456, 0.406], np.float32); std = np.array([0.229, 0.224, 0.225], np.float32)

    def load_img(n):
        im = cv2.cvtColor(cv2.imread(str(P / f"img{a.size}" / n)), cv2.COLOR_BGR2RGB)
        im = np.pad(im, ((pad, Hp - H0 - pad), (0, 0), (0, 0)), mode="edge")
        return ((im.astype(np.float32) / 255.0 - mean) / std).transpose(2, 0, 1)

    def cam(n, Kp):
        Tcw = np.linalg.inv(T[idx[n]])
        m = np.zeros((2, 4, 4), np.float32); m[0] = Tcw; m[1, :3, :3] = Kp; m[1, 3, 3] = 1.0
        return m

    Kp = K0.copy(); Kp[1, 2] += pad
    if not a.skip_infer:
        from utils import init_model
        cfg = json.loads(Path(a.ckpt).with_name("config.json").read_text())
        model = init_model(cfg)
        sd = torch.load(a.ckpt, map_location="cpu", weights_only=False)["state_dict"]
        sd = {(k[7:] if k.startswith("module.") else k): v for k, v in sd.items() if "pe_dict" not in k}
        model.load_state_dict(sd, strict=True)
        model = model.to(dev).eval()
        torch.cuda.reset_peak_memory_stats()
        t0 = time.time(); times = []
        with torch.no_grad():
            for k, n in enumerate(names):
                src = V[n]["src"][:a.n_src]
                if len(src) < a.n_src:
                    src = src + [src[0]] * (a.n_src - len(src))
                vs = [n] + src
                imgs = torch.from_numpy(np.stack([load_img(x) for x in vs]))[None].to(dev)
                pm = np.stack([cam(x, Kp) for x in vs])
                pms = {}
                for st, s in (("stage1", 0.125), ("stage2", 0.25), ("stage3", 0.5), ("stage4", 1.0)):
                    q = pm.copy(); q[:, 1, :2, :] *= s; pms[st] = torch.from_numpy(q)[None].to(dev)
                dv = torch.from_numpy(np.linspace(V[n]["dmin"], V[n]["dmax"], a.ndepth, dtype=np.float32))[None].to(dev)
                torch.cuda.synchronize(); ts = time.time()
                with torch.autocast("cuda", dtype=torch.bfloat16):
                    o = model.forward(imgs, pms, dv, tmp=[5., 5., 5., 1.])
                torch.cuda.synchronize(); times.append(time.time() - ts)
                d = o["refined_depth"][0].float().cpu().numpy()[pad:pad + H0]
                c = o["photometric_confidence"][0].float().cpu().numpy()[pad:pad + H0]
                np.save(out / "raw" / (Path(n).stem + "_d.npy"), d.astype(np.float32))
                np.save(out / "raw" / (Path(n).stem + "_c.npy"), (c * 255).clip(0, 255).astype(np.uint8))
                if k % 25 == 0:
                    print(f"[infer] {k}/{len(names)} {n} {times[-1]:.2f}s d p50 {np.median(d):.2f} conf p50 {np.median(c):.2f}", flush=True)
        info.update(infer_s=round(time.time() - t0, 1), infer_model_s=round(sum(times), 1), infer_per_kf_s=round(float(np.median(times)), 3),
                    infer_gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2))
        del model; torch.cuda.empty_cache()

    # ---- phase 2: dpcd filter (repo functions)
    import misc.fusion as fusion
    t1 = time.time()
    K4 = np.eye(4, dtype=np.float32); K4[:3, :3] = K0
    cams = {}

    def cam0(n):
        if n not in cams:
            m = np.zeros((2, 4, 4), np.float32); m[0] = np.linalg.inv(T[idx[n]]); m[1] = K4; cams[n] = m
        return cams[n]

    def rd(n):
        s = Path(n).stem
        return np.load(out / "raw" / f"{s}_d.npy"), np.load(out / "raw" / f"{s}_c.npy").astype(np.float32) / 255.0

    pts, cols, kept = [], [], []
    with torch.no_grad():
        for n in names:
            src = [x for x in V[n]["src"][:a.n_src] if x in idx and (out / "raw" / (Path(x).stem + "_d.npy")).exists()]
            d0, c0 = rd(n)
            if not src:
                np.save(out / "depth" / (Path(n).stem + ".npy"), np.zeros_like(d0)); kept.append(0.0); continue
            sd_ = np.stack([rd(x)[0] for x in src])[:, None]
            ref_d = torch.from_numpy(d0)[None, None].to(dev)
            src_d = torch.from_numpy(sd_)[None].to(dev)
            ref_c = torch.from_numpy(cam0(n))[None].to(dev)
            src_c = torch.from_numpy(np.stack([cam0(x) for x in src]))[None].to(dev)
            prob_mask = torch.from_numpy(c0 > a.conf)[None, None].to(dev)
            reproj = fusion.get_reproj_dynamic(ref_d, src_d, ref_c, src_c)
            vis_masks, vis_mask = fusion.vis_filter_dynamic(ref_d, reproj, dist_base=4, rel_diff_base=1300)
            rdp = reproj[:, :, -1]
            rdp[~vis_mask.squeeze(2)] = 0
            sums = vis_masks.sum(dim=1); s1 = vis_mask.sum(dim=1)
            dav = (torch.sum(rdp, dim=1, keepdim=True) + ref_d) / (s1 + 1)
            vr = len(src) + 1
            geo = s1 >= vr
            for i in range(2, vr):
                geo = torch.logical_or(geo, sums[:, i - 2] >= i)
            m = torch.minimum(prob_mask, geo)
            D = torch.where(m, dav, torch.zeros_like(dav))[0, 0].cpu().numpy().astype(np.float32)
            np.save(out / "depth" / (Path(n).stem + ".npy"), D)
            kept.append(float((D > 0).mean()))
            # own point cloud (camera -> world)
            vv, uu = np.nonzero(D > 0)
            z = D[vv, uu]
            Xc = np.stack([(uu - K0[0, 2]) / K0[0, 0] * z, (vv - K0[1, 2]) / K0[1, 1] * z, z], 1)
            Tw = T[idx[n]]
            pts.append((Xc @ Tw[:3, :3].T + Tw[:3, 3]).astype(np.float32))
            bgr = cv2.imread(str(P / f"img{a.size}" / n))
            cols.append(bgr[vv, uu][:, ::-1])
    Xa = np.concatenate(pts) if pts else np.zeros((0, 3), np.float32)
    Ca = np.concatenate(cols) if cols else np.zeros((0, 3), np.uint8)
    n_raw = len(Xa)
    if len(Xa):                         # 5 mm voxel thinning (first point per voxel) so the file stays viewable
        kq = np.floor(Xa.astype(np.float64) / 0.005).astype(np.int64)
        kq -= kq.min(0)
        key = (kq[:, 0] * (kq[:, 1].max() + 1) + kq[:, 1]) * (kq[:, 2].max() + 1) + kq[:, 2]
        _, first = np.unique(key, return_index=True)
        Xa, Ca = Xa[first], Ca[first]
    info["fused_points_raw"] = int(n_raw)
    hdr = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(Xa)}\nproperty float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode()
    rec = np.empty(len(Xa), [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = Xa[:, 0], Xa[:, 1], Xa[:, 2]
    rec["red"], rec["green"], rec["blue"] = Ca[:, 0], Ca[:, 1], Ca[:, 2]
    with open(out / "fused.ply", "wb") as f:
        f.write(hdr); f.write(rec.tobytes())
    info.update(filter_s=round(time.time() - t1, 1), fused_points=int(len(Xa)), kept_frac_median=float(np.median(kept)) if kept else None,
                n_kf=len(names))
    (out / "mvsf_run.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
