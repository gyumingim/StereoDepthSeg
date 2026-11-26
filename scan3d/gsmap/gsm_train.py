#!/usr/bin/env python3
"""gsmap 2026-11-25: 3D Gaussian splatting (gsplat.rasterization, 3DGS) on the keyframes, init + depth loss from fused depth.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_train.py DUMP --fused FUSEDIR --out OUT [--steps 20000]

Input: gsplat dump (134 keyframes 960x540 undistorted, BA poses T = cam->world, Kr) + fused depth dir from align_fuse.py
(Z, F: 1 = FFS, 2 = AI fill). Hold-out: every 8th keyframe (index % 8 == 0) is never used (no photo, no depth, no init).
Init: back-project train keyframes (stride 3 px), voxel 1 cm, colour = mean. Loss: 0.8 L1 + 0.2 D-SSIM
+ depth_w * weighted L1(expected depth, Z) with weight 1 on FFS pixels, --fill-w (default 0.1) on filled pixels.
  --depth-sigma (off by default, 11-26): FFS pixel weight (sigma_ref/sigma)^2 clipped [0.05, 4], sigma = per-pixel S map, sigma_ref = view median.
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


def coarse_area(Zf, T, Kr, views, W, H, device=None):
    """Surface area (m2) of a 2 cm TSDF (scanA tsdf_map, sigma 2 cm, min_obs 2, depth <= 4 m) of the given views' fused depth.
    Calibration 11-25 (coarse_area units): 165506 2DM 543k Gaussians / 2.57 m2 = 211k per m2 (mesh2d area 2.82 m2);
    121801 (big room, 494 kf) 2.51M (hit --max-gs 2.5M) / 168.4 m2 = 15k per m2. Default --gs-per-m2 200k = about the 165506
    density (does not bind small scenes; big scenes are still bound by --max-gs). Not tested in a run yet."""
    sys.path.insert(0, str(HERE.parent))
    import tsdf_map
    tm = tsdf_map.TSDFMap(voxel_m=0.02, z_max=4.0, sigma_max=0.2, sigma_ok=0.04, min_obs=2, trunc_max=0.06, max_blocks=200000,
                          device=device)
    img = np.zeros((H, W, 3), np.uint8)
    for c in views:
        Z = Zf[c]; V = np.isfinite(Z) & (Z <= 4.0)
        if V.any():
            tm.integrate(np.where(V, Z, np.nan).astype(np.float32), V, img, float(Kr[0, 0]), float(Kr[0, 2]), float(Kr[1, 2]), 0.08,
                         T[c], sigma=np.where(V, 0.02, np.inf).astype(np.float32))
    tm.update_mesh(force=True)
    Vm, _, Fm = tm.mesh_arrays()
    if len(Fm) == 0:
        return 0.0
    P = np.asarray(Vm, np.float64)[np.asarray(Fm)]
    return float(0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1).sum())


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
    ap.add_argument("--steps", type=int, default=12000); ap.add_argument("--seed", type=int, default=0)   # steps default 12000 from 11-25 (was 20000)
    ap.add_argument("--depth-w", type=float, default=0.1); ap.add_argument("--fill-w", type=float, default=0.1)
    ap.add_argument("--depth-sigma", default="", help="off by default. 'fused' = key S in the fused npz, or a dir of KF.npz files with key S "
                    "(e.g. ffs_4m/s4m): FFS pixel depth weight (median S of the view / S)^2 clipped to [0.05, 4]")
    ap.add_argument("--hold", type=int, default=8); ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--voxel", type=float, default=0.01); ap.add_argument("--max-gs", type=int, default=3_000_000)
    ap.add_argument("--aniso-max", type=float, default=0, help=">0: penalise log(max_scale/min_scale) above log(this) (anti-spike)")
    ap.add_argument("--aniso-w", type=float, default=1.0)
    ap.add_argument("--gs-per-m2", type=float, default=200000.0,
                    help=">0: stop growth at area_m2 x this (area = coarse 2 cm TSDF mesh area of the fused depth, coarse_area())")
    ap.add_argument("--exposure", type=int, default=0, help="E7: per-image RGB gain + offset (reg 0.01, train loss only)")
    ap.add_argument("--normals", default="", help="E7: dir with MoGe camera-frame normals N (depth_ai.py moge)")
    ap.add_argument("--normal-w", type=float, default=0.05)
    ap.add_argument("--surfel", type=int, default=0, help="1 = 2DGS (gsplat rasterization_2dgs) surfels + normal/distortion losses")
    ap.add_argument("--max-scale", type=float, default=0, help=">0: clamp every axis scale to <= this (m) after each step")
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
    test = np.arange(n)[np.arange(n) % a.hold == 0] if a.hold > 0 else np.array([], int); train = np.setdiff1d(np.arange(n), test)   # hold 0 = E8 final (all views)
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
    gt8 = torch.stack([torch.from_numpy(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) for im in imgs]).to(dev)   # uint8 on GPU (big runs)
    gtf = lambda i: gt8[i].float().div(255)  # noqa: E731
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
    area_m2 = None
    if a.gs_per_m2 > 0:   # captured surface area = marching-cubes area of a coarse 2 cm TSDF of the train views fused depth
        area_m2 = coarse_area(Zf, T, Kr, train, W, H)
        a.max_gs = int(min(a.max_gs, area_m2 * a.gs_per_m2))
        print(f"area cap: area_m2 {area_m2:.3f} x {a.gs_per_m2:.0f}/m2 -> max_gs {a.max_gs}", flush=True)
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
    strat = DefaultStrategy(verbose=False, refine_start_iter=500, refine_stop_iter=a.steps // 2, reset_every=3000, refine_every=100,
                            **(dict(key_for_gradient="gradient_2dgs") if a.surfel else {}))
    if a.surfel:
        from gsplat import rasterization_2dgs
    strat.check_sanity(params, opts); sstate = strat.initialize_state(scene_scale=scene_scale)
    zt = [torch.from_numpy(np.nan_to_num(z, nan=0.0)).half().to(dev) for z in Zf]   # half: big runs
    expo = torch.nn.Parameter(torch.zeros(n, 6, device=dev)) if a.exposure else None
    expo_opt = torch.optim.Adam([expo], lr=1e-3) if a.exposure else None
    nrm_t = None
    if a.normals:   # MoGe normals -> world frame, kept where the fused depth exists (edge-cut pixels)
        nrm_t = []
        for c_, i_ in enumerate(kf):
            q_ = Path(a.normals) / f"{i_:05d}.npz"; d_ = np.load(q_) if q_.exists() else None
            if d_ is None or "N" not in d_.files:
                nrm_t.append(None); continue
            Nc = d_["N"].astype(np.float32); Nw = Nc @ T[c_][:3, :3].T; Nw[~(Ff[c_] > 0)] = 0
            nrm_t.append(torch.from_numpy(Nw).half().to(dev))
    wnp = [np.where(F == 1, 1.0, np.where(F == 2, a.fill_w, 0.0)) for F in Ff]
    if a.depth_sigma:   # 11-26: FFS pixels weighted (sigma_ref / sigma)^2, clip [0.05, 4], sigma_ref = median sigma of the view
        for c_, i in enumerate(kf):
            fs = (fused / f"{i:05d}.npz") if a.depth_sigma == "fused" else (Path(a.depth_sigma) / f"{i:05d}.npz")
            d_ = np.load(fs) if fs.exists() else None
            if d_ is None or "S" not in d_:
                continue
            S = d_["S"].astype(np.float64); ok = (Ff[c_] == 1) & np.isfinite(S) & (S > 0)
            if ok.sum() < 10:
                continue
            ref = np.median(S[ok]); wnp[c_] = np.where(ok, np.clip((ref / np.where(ok, S, 1.0)) ** 2, 0.05, 4.0), wnp[c_])
        print(f"depth sigma weights from {a.depth_sigma}: mean FFS w "
              f"{np.mean([wnp[c][Ff[c] == 1].mean() for c in train if (Ff[c] == 1).any()]):.3f}", flush=True)
    wt = [torch.from_numpy(w_.astype(np.float16)).to(dev) for w_ in wnp]

    extra = {}

    def render(ids, deg, mode="RGB+ED", vm=None, K=None, distloss=False):
        vm = viewmats[ids] if vm is None else vm
        K = Ks[None].expand(len(vm), 3, 3) if K is None else K
        if a.surfel:   # 2DGS: returns rgb+ED, alpha, normals, surf normals, distortion, median depth, info
            r = rasterization_2dgs(params["means"], torch.nn.functional.normalize(params["quats"], dim=-1), torch.exp(params["scales"]),
                                   torch.sigmoid(params["opacities"]), torch.cat([params["sh0"], params["shN"]], 1), vm, K, W, H,
                                   sh_degree=deg, render_mode=mode, near_plane=0.02, far_plane=20.0, distloss=distloss)
            extra.update(rn=r[2], sn=r[3], rd=r[4])
            return r[0], r[1], r[6]
        return rasterization(params["means"], torch.nn.functional.normalize(params["quats"], dim=-1), torch.exp(params["scales"]),
                             torch.sigmoid(params["opacities"]), torch.cat([params["sh0"], params["shN"]], 1), vm, K, W, H,
                             sh_degree=deg, render_mode=mode, near_plane=0.02, far_plane=20.0, packed=False)

    log = open(out / "train.log", "w"); hist = []; stopped = False; capped_at = None
    for step in range(a.steps):
        deg = min(sh_deg, step // 1000)
        c = int(train[rng.integers(len(train))])
        rc, ra, info = render(torch.tensor([c], device=dev), deg, distloss=a.surfel and step >= 3000)
        rgb, ed = rc[..., :3], rc[0, ..., 3]
        if not stopped:
            strat.step_pre_backward(params, opts, sstate, step, info)
        gc = gtf(slice(c, c + 1))
        if expo is not None:
            rgb = rgb * (1 + expo[c, :3]) + expo[c, 3:]
        rgb = rgb * mt + gc * (1 - mt)   # invalid border pixels: no photo gradient
        l1 = (rgb - gc).abs().mean()
        ss = 1 - ssim(rgb.permute(0, 3, 1, 2), gc.permute(0, 3, 1, 2))
        loss = 0.8 * l1 + 0.2 * ss
        dpl = torch.zeros((), device=dev)
        if a.depth_w > 0:
            w = wt[c]
            if w.sum() > 0:
                dpl = (w.float() * (ed - zt[c].float()).abs()).sum() / (w > 0).sum()
                loss = loss + a.depth_w * dpl
        if a.surfel:   # gsplat 2DGS example defaults: normal consistency 0.05 from 7000, distortion 0.01 from 3000
            if step >= 7000:
                n_ = extra["rn"][0].permute(2, 0, 1); sn_ = (extra["sn"] * ra.detach())[0].permute(2, 0, 1)
                loss = loss + 0.05 * ((1 - (n_ * sn_).sum(0)) * mt[0, ..., 0]).mean()
            if step >= 3000:
                loss = loss + 0.01 * extra["rd"].mean()
        anl = torch.zeros((), device=dev)
        if a.aniso_max > 0:
            ls = params["scales"][:, :2] if a.surfel else params["scales"]   # 2DGS: only the two in-plane axes exist
            anl = torch.relu(ls.max(1).values - ls.min(1).values - math.log(a.aniso_max)).mean()
            loss = loss + a.aniso_w * anl
        if expo is not None:
            loss = loss + 0.01 * (expo[c] ** 2).sum()
        if nrm_t is not None and a.surfel and nrm_t[c] is not None and step >= 1000:
            nm_ = nrm_t[c].float(); vm_ = (nm_.abs().sum(-1) > 0).float()
            nl_ = (1 - (extra["rn"][0] * nm_).sum(-1).abs()) * vm_ * ra[0, ..., 0].detach()
            loss = loss + a.normal_w * nl_.sum() / vm_.sum().clamp(min=1)
        loss.backward()
        if not stopped:
            strat.step_post_backward(params, opts, sstate, step, info, packed=False)
            if len(params["means"]) > a.max_gs and strat.grow_grad2d != float("inf"):
                # cap reached: no more split/clone (grow threshold -> inf); pruning and opacity resets continue
                strat.grow_grad2d = float("inf"); capped_at = step
                print(f"step {step}: {len(params['means'])} > max-gs {a.max_gs}, growth stopped (pruning continues)", flush=True)
        for o in opts.values():
            o.step(); o.zero_grad(set_to_none=True)
        if expo_opt is not None:
            expo_opt.step(); expo_opt.zero_grad(set_to_none=True)
        sched.step()
        if a.max_scale > 0:
            with torch.no_grad():
                params["scales"].clamp_(max=math.log(a.max_scale))
        if step % 500 == 0 or step == a.steps - 1:
            s_ = dict(step=step, n=len(params["means"]), l1=round(float(l1), 4), dssim=round(float(ss), 4), depth=round(float(dpl), 4), aniso=round(float(anl), 4),
                      gpu_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2), t=round(time.time() - t0, 1))
            hist.append(s_); print(json.dumps(s_), flush=True); log.write(json.dumps(s_) + "\n"); log.flush()
    t_train = time.time() - t0

    # ---- save (before eval so a crash in eval keeps the map)
    pc = {k: v.detach().cpu() for k, v in params.items()}
    torch.save(dict(params=pc, sh_degree=sh_deg, T=T, Kr=Kr, size=(W, H), kf=kf, test=test, args=vars(a)), out / "gaussians.pt")
    if a.surfel:   # 3DGS-compatible export: surfel normal = 3rd rotation axis -> flat 3rd scale
        pe = dict(pc); sc = pe["scales"].clone(); sc[:, 2] = math.log(1e-4); pe["scales"] = sc
        write_ply(out / "splat.ply", pe)
    else:
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
            g = gtf(int(c))
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
                n_aniso5_opaque=int(((op > 0.3) & (np.exp(params["scales"].detach().cpu().numpy()[:, :2 if a.surfel else 3]).max(1) / np.exp(params["scales"].detach().cpu().numpy()[:, :2 if a.surfel else 3]).min(1) > 5)).sum()),
                n_dark_opaque=int(((op > 0.3) & ((params["sh0"][:, 0].detach().cpu().numpy() * C0 + 0.5).mean(1) < 0.1)).sum()),
                n_scale_gt5cm=int((np.exp(params["scales"].detach().cpu().numpy()).max(1) > 0.05).sum()),
                area_m2=area_m2, gs_cap=int(a.max_gs), capped_at_step=capped_at, n_init=N, init_fill_share=fillfrac_init, n_gauss=int(len(M)),
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
