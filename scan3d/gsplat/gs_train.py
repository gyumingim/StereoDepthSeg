#!/usr/bin/env python3
"""gsplat 2026-11-05: 2DGS (gsplat.rasterization_2dgs) on the scanA keyframes -> rendered median depth -> scanA tsdf_map -> mesh.

Input = a dump of scan_offline_dump.py (same keyframes / poses / undistorted 960x540 images / FFS depth as the TSDF run).
  ~/scanner/gsplat_env/venv/bin/python scan3d/gsplat/gs_train.py DUMP --out OUT [--variant A|B] [--steps 15000] [--seed 0]
Variant A = photometric only, random init in the camera frusta (depth U(0.3, 1.5) m).
Variant B = init from FFS depth points + L1 depth loss (rendered expected depth vs FFS Z, valid pixels, weight 0.1).
Losses (gsplat simple_trainer_2dgs defaults): 0.8 L1 + 0.2 D-SSIM, normal consistency 0.05 from step 7000,
distortion 0.01 from step 3000. Densification = gsplat DefaultStrategy (key gradient_2dgs), refine 500..7500.
Mesh = scanA tsdf_map.TSDFMap (voxel 5 mm, z_max 2.0, sigma_max 0.08, sigma_ok 0.02, min_obs 2), per-pixel sigma 0.01 m,
depth = render_median where alpha > 0.5. GPU: gpu_guard (gpu.lock, torch cap 0.75, OOM -> clean failure).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))


def ssim(x, y, C1=0.01 ** 2, C2=0.03 ** 2):
    import torch
    import torch.nn.functional as Fn
    g = torch.exp(-((torch.arange(11, device=x.device) - 5.0) ** 2) / (2 * 1.5 ** 2))
    g = (g / g.sum())
    w = (g[:, None] * g[None, :])[None, None].repeat(x.shape[1], 1, 1, 1)
    conv = lambda t: Fn.conv2d(t, w, padding=5, groups=x.shape[1])  # noqa: E731
    mx, my = conv(x), conv(y)
    sxx, syy, sxy = conv(x * x) - mx * mx, conv(y * y) - my * my, conv(x * y) - mx * my
    m = ((2 * mx * my + C1) * (2 * sxy + C2)) / ((mx * mx + my * my + C1) * (sxx + syy + C2))
    return m.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--out", required=True)
    ap.add_argument("--variant", default="A", choices=["A", "B"])
    ap.add_argument("--steps", type=int, default=15000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-init", type=int, default=100000)
    ap.add_argument("--depth-w", type=float, default=0.1)
    ap.add_argument("--max-gs", type=int, default=3_000_000, help="stop densifying above this many Gaussians (memory)")
    a = ap.parse_args()
    import torch
    from gsplat import rasterization_2dgs
    from gsplat.strategy import DefaultStrategy
    import tsdf_map
    t0 = time.time()
    torch.manual_seed(a.seed); np.random.seed(a.seed)
    dev = "cuda"
    dump, out = Path(a.dump), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cams = np.load(dump / "cams.npz")
    kf, T, Kr = cams["kf"], cams["T"], cams["Kr"].astype(np.float64)
    W, H = [int(v) for v in cams["size"]]
    imgs = [cv2.imread(str(dump / "img" / f"{i:05d}.png")) for i in kf]
    ffs = []
    for i in kf:
        p = dump / "ffs" / f"{i:05d}.npz"
        ffs.append(np.load(p)["Z"] if p.exists() else None)
    n_ffs = sum(z is not None for z in ffs)
    print(f"keyframes {len(kf)} ({W}x{H}), with FFS depth {n_ffs}", flush=True)
    gt_img = torch.stack([torch.from_numpy(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) for im in imgs]).float().div(255).to(dev)
    viewmats = torch.from_numpy(np.linalg.inv(T)).float().to(dev)
    Ks = torch.from_numpy(Kr).float().to(dev)
    fx = float(Kr[0, 0])
    rng = np.random.default_rng(a.seed)

    # ---- init points
    pts, cols = [], []
    if a.variant == "A":
        ci = rng.integers(0, len(kf), a.n_init)
        u = rng.uniform(0, W, a.n_init); v = rng.uniform(0, H, a.n_init); z = rng.uniform(0.3, 1.5, a.n_init)
        for c in range(len(kf)):
            m = ci == c
            X = np.stack([(u[m] - Kr[0, 2]) / fx * z[m], (v[m] - Kr[1, 2]) / Kr[1, 1] * z[m], z[m]], 1)
            pts.append(X @ T[c][:3, :3].T + T[c][:3, 3])
            cols.append(imgs[c][v[m].astype(int), u[m].astype(int), ::-1] / 255.0)
    else:
        per = max(1, a.n_init // max(1, n_ffs))
        for c, Z in enumerate(ffs):
            if Z is None:
                continue
            vv, uu = np.nonzero(np.isfinite(Z))
            if len(vv) == 0:
                continue
            s = rng.choice(len(vv), min(per, len(vv)), replace=False)
            vv, uu = vv[s], uu[s]; zz = Z[vv, uu]
            X = np.stack([(uu - Kr[0, 2]) / fx * zz, (vv - Kr[1, 2]) / Kr[1, 1] * zz, zz], 1)
            pts.append(X @ T[c][:3, :3].T + T[c][:3, 3])
            cols.append(imgs[c][vv, uu, ::-1] / 255.0)
    P = np.concatenate(pts).astype(np.float32); Cc = np.concatenate(cols).astype(np.float32)
    # init scale = 3 px footprint at the distance to the nearest camera centre
    cen = T[:, :3, 3]
    dmin = np.min(np.linalg.norm(P[:, None] - cen[None].astype(np.float32), axis=2), 1)
    sc0 = np.log(np.clip(3 * dmin / fx, 1e-4, None))
    scene_scale = float(np.linalg.norm(cen - cen.mean(0), axis=1).max() * 1.1)
    N = len(P)
    print(f"init {a.variant}: {N} points, scene_scale {scene_scale:.3f} m", flush=True)
    sh_deg = 3
    C0 = 0.28209479177387814
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
    strat = DefaultStrategy(verbose=False, key_for_gradient="gradient_2dgs", refine_start_iter=500, refine_stop_iter=7500,
                            reset_every=3000, refine_every=100)
    strat.check_sanity(params, opts)
    sstate = strat.initialize_state(scene_scale=scene_scale)
    ffs_t = [None if z is None else torch.from_numpy(np.nan_to_num(z, nan=0.0)).to(dev) for z in ffs]

    def render(ids, mode="RGB+ED", distloss=False):
        colors = torch.cat([params["sh0"], params["shN"]], 1)
        return rasterization_2dgs(params["means"], torch.nn.functional.normalize(params["quats"], dim=-1),
                                  torch.exp(params["scales"]), torch.sigmoid(params["opacities"]), colors,
                                  viewmats[ids], Ks[None].expand(len(ids), 3, 3), W, H, sh_degree=cur_deg,
                                  render_mode=mode, distloss=distloss, near_plane=0.05, far_plane=5.0)

    log = open(out / "train.log", "w")
    hist = []
    stopped_densify = False
    for step in range(a.steps):
        cur_deg = min(sh_deg, step // 1000)
        c = int(rng.integers(len(kf)))
        ids = torch.tensor([c], device=dev)
        rc, ra, rn, sn, rd, rm, info = render(ids, distloss=step >= 3000)
        rgb = rc[..., :3]; ed = rc[..., 3]
        if not stopped_densify:
            strat.step_pre_backward(params, opts, sstate, step, info)
        l1 = (rgb - gt_img[c:c + 1]).abs().mean()
        ss = 1 - ssim(rgb.permute(0, 3, 1, 2), gt_img[c:c + 1].permute(0, 3, 1, 2))
        loss = 0.8 * l1 + 0.2 * ss
        nl = dl = dpl = torch.zeros((), device=dev)
        if step >= 7000:
            n_ = rn[0].permute(2, 0, 1)
            sn_ = (sn * ra.detach())[0].permute(2, 0, 1)
            nl = (1 - (n_ * sn_).sum(0)).mean()
            loss = loss + 0.05 * nl
        if step >= 3000:
            dl = rd.mean()
            loss = loss + 0.01 * dl
        if a.variant == "B" and ffs_t[c] is not None:
            m = ffs_t[c] > 0
            if m.any():
                dpl = (ed[0][m] - ffs_t[c][m]).abs().mean()
                loss = loss + a.depth_w * dpl
        loss.backward()
        if not stopped_densify:
            strat.step_post_backward(params, opts, sstate, step, info, packed=False)
            if len(params["means"]) > a.max_gs:
                stopped_densify = True
                print(f"step {step}: {len(params['means'])} Gaussians > max-gs -> densification stopped", flush=True)
        for o in opts.values():
            o.step(); o.zero_grad(set_to_none=True)
        sched.step()
        if step % 500 == 0 or step == a.steps - 1:
            s_ = dict(step=step, n=len(params["means"]), l1=float(l1), dssim=float(ss), normal=float(nl), dist=float(dl),
                      depth=float(dpl), gpu_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2), t=round(time.time() - t0, 1))
            hist.append(s_); print(json.dumps(s_), flush=True); log.write(json.dumps(s_) + "\n"); log.flush()
    t_train = time.time() - t0

    # ---- render depth at every keyframe -> TSDF (same tsdf_map, 5 mm) -> mesh
    tsdf = tsdf_map.TSDFMap(voxel_m=float(cams["voxel"]), z_max=float(cams["zmax"]), sigma_max=0.08, sigma_ok=0.02, min_obs=2)
    (out / "depth").mkdir(exist_ok=True)
    psnr = []
    with torch.no_grad():
        cur_deg = sh_deg
        for c in range(len(kf)):
            rc, ra, rn, sn, rd, rm, info = render(torch.tensor([c], device=dev))
            Z = rm[0, ..., 0].float().cpu().numpy(); A = ra[0, ..., 0].float().cpu().numpy()
            psnr.append(float(-10 * torch.log10(((rc[0, ..., :3] - gt_img[c]) ** 2).mean())))
            V = (A > 0.5) & np.isfinite(Z) & (Z > 0.05)
            np.savez_compressed(out / "depth" / f"{kf[c]:05d}.npz", Z=np.where(V, Z, np.nan).astype(np.float32), A=A.astype(np.float16))
            tsdf.integrate(np.where(V, Z, np.nan).astype(np.float32), V, imgs[c], float(Kr[0, 0]), float(Kr[0, 2]), float(Kr[1, 2]),
                           0.08, T[c], sigma=np.where(V, 0.01, np.inf).astype(np.float32))
    tsdf.update_mesh(force=True)
    Vm, Cm, Fm = tsdf.mesh_arrays()
    tsdf_map.write_mesh_ply(out / "mesh.ply", Vm, Cm, Fm)
    summ = dict(made=time.strftime("%F %T"), dump=str(dump), args=vars(a), n_kf=int(len(kf)), n_ffs=int(n_ffs),
                n_gauss=int(len(params["means"])), psnr_mean=float(np.mean(psnr)), train_s=round(t_train, 1),
                runtime_s=round(time.time() - t0, 1), mesh_vertices=int(len(Vm)), mesh_faces=int(len(Fm)),
                gpu_max_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                gpu_max_reserved_gb=round(torch.cuda.max_memory_reserved() / 2 ** 30, 2), hist=hist)
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    torch.save({k: v.detach().cpu() for k, v in params.items()}, out / "gaussians.pt")
    print(json.dumps({k: v for k, v in summ.items() if k != "hist"}), flush=True)


def _out_arg():
    return sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else None


if __name__ == "__main__":
    import gpu_guard
    with gpu_guard.job("gs_train " + str(_out_arg()), out_dir=_out_arg(), torch_cap=True):
        main()
