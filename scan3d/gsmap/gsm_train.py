#!/usr/bin/env python3
"""gsmap 2026-11-25: 3D Gaussian splatting (gsplat.rasterization, 3DGS) on the keyframes, init + depth loss from fused depth.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_train.py DUMP --fused FUSEDIR --out OUT [--steps 20000]

Input: gsplat dump (134 keyframes 960x540 undistorted, BA poses T = cam->world, Kr) + fused depth dir from align_fuse.py
(Z, F: 1 = FFS, 2 = AI fill). Hold-out: every 8th keyframe (index % 8 == 0) is never used (no photo, no depth, no init).
Init: back-project train keyframes (stride 3 px), voxel 1 cm, colour = mean. Loss: 0.8 L1 + 0.2 D-SSIM
+ depth_w * weighted L1(expected depth, Z) with weight 1 on FFS pixels, --fill-w (default 0.1) on filled pixels.
Densification = gsplat DefaultStrategy (3DGS), refine 500..steps/2, opacity reset every 3000.
Eval: held-out PSNR / SSIM / LPIPS(alex) on valid pixels (+ *_full incl. the black border), render depth vs FFS on held-out views, free-space (floater) check vs FFS.
Saves OUT/splat.ply (INRIA 3DGS layout, loads in common splat viewers), OUT/gaussians.pt (params + cams, to refine later),
OUT/summary.json, OUT/train.log.
"""
import argparse, json, math, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "gsplat"))
C0 = 0.28209479177387814


def ssim(x, y, C1=0.01 ** 2, C2=0.03 ** 2):
    import torch
    import torch.nn.functional as Fn
    g = torch.exp(-((torch.arange(11, device=x.device) - 5.0) ** 2) / (2 * 1.5 ** 2)); g = g / g.sum()
    w = (g[:, None] * g[None, :])[None, None].repeat(x.shape[1], 1, 1, 1)
    conv = lambda t: Fn.conv2d(t, w, padding=5, groups=x.shape[1])  # noqa: E731
    mx, my = conv(x), conv(y)
    sxx, syy, sxy = conv(x * x) - mx * mx, conv(y * y) - my * my, conv(x * y) - mx * my
    return (((2 * mx * my + C1) * (2 * sxy + C2)) / ((mx * mx + my * my + C1) * (sxx + syy + C2))).mean()


def write_ply(path, p):
    """INRIA 3DGS ply: x y z nx ny nz f_dc_0..2 f_rest_0..(3*(K-1)-1) opacity scale_0..2 rot_0..3 (logit / log / wxyz)."""
    means = p["means"].numpy(); n = len(means)
    sh0 = p["sh0"].numpy().reshape(n, -1)                       # (n, 3)
    shN = p["shN"].numpy().transpose(0, 2, 1).reshape(n, -1)    # channel-major like the reference exporter
    cols = [means, np.zeros((n, 3), np.float32), sh0, shN, p["opacities"].numpy()[:, None], p["scales"].numpy(), p["quats"].numpy()]
    names = ["x", "y", "z", "nx", "ny", "nz"] + [f"f_dc_{i}" for i in range(3)] + [f"f_rest_{i}" for i in range(shN.shape[1])] \
        + ["opacity"] + [f"scale_{i}" for i in range(3)] + [f"rot_{i}" for i in range(4)]
    data = np.concatenate(cols, 1).astype(np.float32)
    hdr = "ply\nformat binary_little_endian 1.0\nelement vertex %d\n" % n + "".join(f"property float {k}\n" for k in names) + "end_header\n"
    with open(path, "wb") as f:
        f.write(hdr.encode()); f.write(data.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("--fused", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--steps", type=int, default=20000); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--depth-w", type=float, default=0.1); ap.add_argument("--fill-w", type=float, default=0.1)
    ap.add_argument("--hold", type=int, default=8); ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--voxel", type=float, default=0.01); ap.add_argument("--max-gs", type=int, default=3_000_000)
    ap.add_argument("--mask-border", type=int, default=1, help="1 = ignore the black undistortion border (photo, depth, init, eval)")
    a = ap.parse_args()
    import torch
    from gsplat import rasterization
    from gsplat.strategy import DefaultStrategy
    t0 = time.time()
    torch.manual_seed(a.seed); np.random.seed(a.seed); rng = np.random.default_rng(a.seed)
    dev = "cuda"
    dump, fused, out = Path(a.dump), Path(a.fused), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = np.load(dump / "cams.npz")
    kf, T, Kr = cams["kf"], cams["T"], cams["Kr"].astype(np.float64)
    W, H = [int(v) for v in cams["size"]]
    n = len(kf)
    test = np.arange(n)[np.arange(n) % a.hold == 0]; train = np.setdiff1d(np.arange(n), test)
    imgs = [cv2.imread(str(dump / "img" / f"{i:05d}.png")) for i in kf]
    fz = [np.load(fused / f"{i:05d}.npz") for i in kf]
    Zf = [d["Z"] for d in fz]; Ff = [d["F"] for d in fz]
    ffs = [np.where(F == 1, Z, np.nan) for Z, F in zip(Zf, Ff)]   # FFS reference for the checks (same in every variant)
    # valid-pixel mask of the undistorted images (black border where the source image has no pixel), eroded 3 px.
    # Added 11-25 after v1: without it the photo loss fits black Gaussians to the border and AI fill puts points there.
    if a.mask_border:
        mx, my = cv2.initUndistortRectifyMap(cams["K"], cams["dist"], None, Kr, (W, H), cv2.CV_32FC1)
        vm_ = ((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8)
        vmask = cv2.erode(vm_, np.ones((7, 7), np.uint8)).astype(bool)
    else:
        vmask = np.ones((H, W), bool)
    Ff = [np.where(vmask, F, 0).astype(np.uint8) for F in Ff]
    Zf = [np.where(F > 0, Z, np.nan) for Z, F in zip(Zf, Ff)]
    mt = torch.from_numpy(vmask).float().to("cuda")[None, ..., None]   # (1, H, W, 1)
    print(f"valid pixel mask: {vmask.mean():.4f} (mask_border={a.mask_border})", flush=True)
    print(f"keyframes {n} ({W}x{H}) train {len(train)} test {len(test)}; train px FFS {np.mean([(Ff[c] == 1).mean() for c in train]):.3f} "
          f"fill {np.mean([(Ff[c] == 2).mean() for c in train]):.3f}", flush=True)
    gt = torch.stack([torch.from_numpy(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) for im in imgs]).float().div(255).to(dev)
    viewmats = torch.from_numpy(np.linalg.inv(T)).float().to(dev)
    Ks = torch.from_numpy(Kr).float().to(dev)
    fx, fy, cx, cy = Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2]

    # ---- init from fused depth of train views
    P, Cc, Fl = [], [], []
    vv, uu = np.mgrid[0:H:a.stride, 0:W:a.stride]
    for c in train:
        z = Zf[c][vv, uu]; m = np.isfinite(z)
        if not m.any():
            continue
        u, v, z = uu[m], vv[m], z[m]
        X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1)
        P.append(X @ T[c][:3, :3].T + T[c][:3, 3]); Cc.append(imgs[c][v, u, ::-1] / 255.0); Fl.append(Ff[c][v, u])
    P = np.concatenate(P); Cc = np.concatenate(Cc); Fl = np.concatenate(Fl)
    key = np.floor(P / a.voxel).astype(np.int64)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True); inv = inv.ravel()
    Pm = np.zeros((len(cnt), 3)); Cm = np.zeros((len(cnt), 3)); Fm = np.zeros(len(cnt))
    np.add.at(Pm, inv, P); np.add.at(Cm, inv, Cc); np.add.at(Fm, inv, (Fl == 2).astype(float))
    P = (Pm / cnt[:, None]).astype(np.float32); Cc = (Cm / cnt[:, None]).astype(np.float32); fillfrac_init = float((Fm / cnt > 0.5).mean())
    N = len(P)
    cen = T[:, :3, 3]
    # init scale = mean distance to 3 nearest neighbours (as 3DGS), via a KD-tree
    from scipy.spatial import cKDTree
    d, _ = cKDTree(P).query(P, k=4)
    sc0 = np.log(np.clip(d[:, 1:].mean(1), 1e-4, 0.05))
    scene_scale = float(np.linalg.norm(cen - cen.mean(0), axis=1).max() * 1.1)
    print(f"init: {N} points (voxel {a.voxel} m, fill share {fillfrac_init:.3f}), scene_scale {scene_scale:.3f} m", flush=True)
    sh_deg = 3
    params = torch.nn.ParameterDict(dict(
        means=torch.nn.Parameter(torch.from_numpy(P)),
        scales=torch.nn.Parameter(torch.from_numpy(np.repeat(sc0[:, None], 3, 1).astype(np.float32))),
        quats=torch.nn.Parameter(torch.rand(N, 4)),
        opacities=torch.nn.Parameter(torch.logit(torch.full((N,), 0.1))),
        sh0=torch.nn.Parameter(torch.from_numpy((Cc - 0.5) / C0)[:, None, :]),
        shN=torch.nn.Parameter(torch.zeros(N, (sh_deg + 1) ** 2 - 1, 3)),
    )).to(dev)
    lrs = dict(means=1.6e-4 * scene_scale, scales=5e-3, quats=1e-3, opacities=5e-2, sh0=2.5e-3, shN=2.5e-3 / 20)
    opts = {k: torch.optim.Adam([{"params": params[k], "lr": lr, "name": k}], eps=1e-15) for k, lr in lrs.items()}
    sched = torch.optim.lr_scheduler.ExponentialLR(opts["means"], gamma=0.01 ** (1.0 / a.steps))
    strat = DefaultStrategy(verbose=False, refine_start_iter=500, refine_stop_iter=a.steps // 2, reset_every=3000, refine_every=100)
    strat.check_sanity(params, opts); sstate = strat.initialize_state(scene_scale=scene_scale)
    zt = [torch.from_numpy(np.nan_to_num(z, nan=0.0)).to(dev) for z in Zf]
    wt = [torch.from_numpy(np.where(F == 1, 1.0, np.where(F == 2, a.fill_w, 0.0)).astype(np.float32)).to(dev) for F in Ff]

    def render(ids, deg, mode="RGB+ED", vm=None, K=None):
        vm = viewmats[ids] if vm is None else vm
        K = Ks[None].expand(len(vm), 3, 3) if K is None else K
        return rasterization(params["means"], torch.nn.functional.normalize(params["quats"], dim=-1), torch.exp(params["scales"]),
                             torch.sigmoid(params["opacities"]), torch.cat([params["sh0"], params["shN"]], 1), vm, K, W, H,
                             sh_degree=deg, render_mode=mode, near_plane=0.02, far_plane=20.0, packed=False)

    log = open(out / "train.log", "w"); hist = []; stopped = False
    for step in range(a.steps):
        deg = min(sh_deg, step // 1000)
        c = int(train[rng.integers(len(train))])
        rc, ra, info = render(torch.tensor([c], device=dev), deg)
        rgb, ed = rc[..., :3], rc[0, ..., 3]
        if not stopped:
            strat.step_pre_backward(params, opts, sstate, step, info)
        rgb = rgb * mt + gt[c:c + 1] * (1 - mt)   # invalid border pixels: no photo gradient
        l1 = (rgb - gt[c:c + 1]).abs().mean()
        ss = 1 - ssim(rgb.permute(0, 3, 1, 2), gt[c:c + 1].permute(0, 3, 1, 2))
        loss = 0.8 * l1 + 0.2 * ss
        dpl = torch.zeros((), device=dev)
        if a.depth_w > 0:
            w = wt[c]
            if w.sum() > 0:
                dpl = (w * (ed - zt[c]).abs()).sum() / (w > 0).sum()
                loss = loss + a.depth_w * dpl
        loss.backward()
        if not stopped:
            strat.step_post_backward(params, opts, sstate, step, info, packed=False)
            if len(params["means"]) > a.max_gs:
                stopped = True; print(f"step {step}: > max-gs, densification stopped", flush=True)
        for o in opts.values():
            o.step(); o.zero_grad(set_to_none=True)
        sched.step()
        if step % 500 == 0 or step == a.steps - 1:
            s_ = dict(step=step, n=len(params["means"]), l1=round(float(l1), 4), dssim=round(float(ss), 4), depth=round(float(dpl), 4),
                      gpu_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2), t=round(time.time() - t0, 1))
            hist.append(s_); print(json.dumps(s_), flush=True); log.write(json.dumps(s_) + "\n"); log.flush()
    t_train = time.time() - t0

    # ---- save (before eval so a crash in eval keeps the map)
    pc = {k: v.detach().cpu() for k, v in params.items()}
    torch.save(dict(params=pc, sh_degree=sh_deg, T=T, Kr=Kr, size=(W, H), kf=kf, test=test, args=vars(a)), out / "gaussians.pt")
    write_ply(out / "splat.ply", pc)

    # ---- eval
    import lpips
    lp = lpips.LPIPS(net="alex", verbose=False).to(dev)
    ev, depth_ev = [], []
    (out / "test").mkdir(exist_ok=True)
    with torch.no_grad():
        for c in list(test) + list(train):
            rc, ra, _ = render(torch.tensor([int(c)], device=dev), sh_deg)
            rgb = rc[0, ..., :3].clamp(0, 1); ed = rc[0, ..., 3].cpu().numpy(); al = ra[0, ..., 0].cpu().numpy()
            g = gt[int(c)]
            mse_full = ((rgb - g) ** 2).mean()
            m0 = mt[0]
            mse = (m0 * (rgb - g) ** 2).sum() / (m0.sum() * 3)   # = mse_full when mask_border 0
            e = dict(c=int(c), kf=int(kf[c]), test=bool(c in test), psnr=float(-10 * torch.log10(mse)),
                     psnr_full=float(-10 * torch.log10(mse_full)))
            if c in test:
                x, y = rgb.permute(2, 0, 1)[None], g.permute(2, 0, 1)[None]
                e["ssim_full"] = float(ssim(x, y)); e["lpips_full"] = float(lp(x * 2 - 1, y * 2 - 1))
                xm = (rgb * m0 + g * (1 - m0)).permute(2, 0, 1)[None]
                e["ssim"] = float(ssim(xm, y)); e["lpips"] = float(lp(xm * 2 - 1, y * 2 - 1))
                cv2.imwrite(str(out / "test" / f"{kf[c]:05d}.jpg"), cv2.cvtColor((rgb.cpu().numpy() * 255).astype(np.uint8), cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, 92])
                np.savez_compressed(out / "test" / f"{kf[c]:05d}_depth.npz", ED=ed.astype(np.float16), A=al.astype(np.float16))
                r = ffs[c]; m = np.isfinite(r) & (al > 0.5)
                if m.sum() > 1000:
                    dz = np.abs(ed[m] - r[m])
                    depth_ev.append(dict(kf=int(kf[c]), med_cm=float(np.median(dz) * 100), p90_cm=float(np.percentile(dz, 90) * 100),
                                         gt3cm=float((dz > 0.03).mean()), n=int(m.sum())))
            ev.append(e)
        # free-space (floater) check: Gaussian centres seen clearly in front of an FFS surface (> max(5 cm, 5 %))
        op = torch.sigmoid(params["opacities"]).cpu().numpy(); M = params["means"].cpu().numpy()
        sel = op > 0.2
        Ms = M[sel]; viol = np.zeros(len(Ms), int); seen = np.zeros(len(Ms), int)
        for c in range(n):
            r = ffs[c]
            if not np.isfinite(r).any():
                continue
            Tw = np.linalg.inv(T[c]); Xc = Ms @ Tw[:3, :3].T + Tw[:3, 3]
            z = Xc[:, 2]; ok = z > 0.05
            u = np.round(fx * Xc[:, 0] / np.where(ok, z, 1) + cx).astype(int); v = np.round(fy * Xc[:, 1] / np.where(ok, z, 1) + cy).astype(int)
            ok &= (u >= 0) & (u < W) & (v >= 0) & (v < H)
            zr = np.full(len(Ms), np.nan); zr[ok] = r[v[ok], u[ok]]
            ok &= np.isfinite(zr)
            seen += ok
            viol += ok & (z < zr - np.maximum(0.05, 0.05 * zr))
        floater = (viol >= 2) & (viol > 0.5 * seen)
    tr = [e for e in ev if not e["test"]]; te = [e for e in ev if e["test"]]
    summ = dict(made=time.strftime("%F %T"), dump=str(dump), fused=str(fused), args=vars(a), n_kf=n, n_train=len(train), n_test=len(test),
                n_init=N, init_fill_share=fillfrac_init, n_gauss=int(len(M)),
                test_psnr=float(np.mean([e["psnr"] for e in te])), test_ssim=float(np.mean([e["ssim"] for e in te])),
                test_lpips=float(np.mean([e["lpips"] for e in te])), train_psnr=float(np.mean([e["psnr"] for e in tr])),
                test_psnr_full=float(np.mean([e["psnr_full"] for e in te])), test_ssim_full=float(np.mean([e["ssim_full"] for e in te])),
                test_lpips_full=float(np.mean([e["lpips_full"] for e in te])), valid_px=float(vmask.mean()),
                test_depth_vs_ffs=dict(med_cm=float(np.median([d["med_cm"] for d in depth_ev])) if depth_ev else None,
                                       p90_cm=float(np.median([d["p90_cm"] for d in depth_ev])) if depth_ev else None,
                                       frac_gt3cm=float(np.mean([d["gt3cm"] for d in depth_ev])) if depth_ev else None,
                                       n_views=len(depth_ev)),
                floaters=dict(n_checked=int(sel.sum()), n_seen=int((seen > 0).sum()), n_floater=int(floater.sum()),
                              frac_of_seen=float(floater.sum() / max(1, (seen > 0).sum()))),
                train_s=round(t_train, 1), runtime_s=round(time.time() - t0, 1),
                gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                gpu_max_reserved_gb=round(torch.cuda.max_memory_reserved() / 2 ** 30, 2), per_view=ev, depth_views=depth_ev, hist=hist)
    np.save(out / "floater_mask.npy", np.flatnonzero(sel)[floater])
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    print(json.dumps({k: v for k, v in summ.items() if k not in ("hist", "per_view", "depth_views")}), flush=True)


if __name__ == "__main__":
    import gpu_guard
    o = sys.argv[sys.argv.index("--out") + 1]
    with gpu_guard.job("gsmap train " + o, out_dir=o, torch_cap=True):
        main()
