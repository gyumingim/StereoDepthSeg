#!/usr/bin/env python3
"""viewcrafter 2026-11-25: ViewCrafter (512 model) on recording 165506 along a vc_traj.py side-orbit.

  cd ~/scanner/viewcrafter_env/ViewCrafter
  ../venv/bin/python ~/scanner/StereoDepthSeg-gsmap/scan3d/viewcrafter/vc_run.py --traj TRAJ.npz --dump DUMP --fuse FUSE_DA3 \
      --mode sparse|single --pc dust3r|ours --out OUT_DIR [--npts N | --npts-from OTHER_OUT/info.json]

Pipeline (same pieces as ViewCrafter's inference.py, only the 576x1024 hard-coding replaced by 320x512):
  1. inputs: our undistorted 960x540 keyframes, centre crop 864x540, resize 512x320 (= ViewCrafter-512 frame size).
     The anchor (trajectory frame 0) is the first input. 'single' uses only the anchor.
  2. point cloud
     dust3r: DUSt3R ViT-L 512 on the inputs exactly as ViewCrafter (complete pair graph, PointCloudOptimizer, mst, 300 it,
             lr 0.01 linear; sparse: clean_pointcloud + conf>3 OR bg mask (bg_trd 0.2, run_sparse.sh); single: image twice,
             no mask). Only change: focal preset to our calibrated focal (fx of K512), so the cloud is in our intrinsics.
             DUSt3R poses are free; the cloud is then moved into our BA frame with a Sim(3):
               sparse: R = chordal mean of R_ours_i R_dust3r_i^T, s and t by least squares on the camera centres;
               single: R, t from the anchor pose, s = median(Z_ours / Z_dust3r) on the anchor (fused FFS+DA3 depth).
     ours:   our fused depth (fuse/da3 = FFS + aligned DA3 fill) of the inputs +- --nbr keyframes, back-projected with our BA
             poses and Kr, colour from the keyframe images, random subsample to the DUSt3R point count (--npts-from).
  3. condition video: pytorch3d point rendering with ViewCrafter's own generate_traj + setup_renderer (radius 0.01, 10 pts/px,
     alpha compositing, black background) at 320x512 with K512 along the trajectory; frame 0 replaced by the anchor image
     (ViewCrafter does the same). A second render with white features gives the coverage mask (holes = hallucinated area).
  4. diffusion: ViewCrafter_25_512 (configs/inference_pvd_512.yaml), 50 DDIM steps, cfg 7.5, seed 123, 25 frames.
Outputs in OUT: inputs/, render.mp4 (condition), diffusion.mp4, frames npz (render, diffusion, cover), pcd stats, info.json.
"""
import argparse, copy, json, os, subprocess, sys, threading, time
from pathlib import Path

import cv2
import numpy as np

VC = Path.home() / "scanner" / "viewcrafter_env" / "ViewCrafter"


class GpuPeak:
    """nvidia-smi sampling (whole GPU, MiB) every 0.5 s in a thread."""
    def __init__(self):
        self.peak = 0; self.stop = False; self.t = threading.Thread(target=self.run, daemon=True); self.t.start()

    def run(self):
        while not self.stop:
            try:
                v = int(subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                                       capture_output=True, text=True, timeout=5).stdout.strip().splitlines()[0])
                self.peak = max(self.peak, v)
            except Exception:
                pass
            time.sleep(0.5)


def prep_inputs(dump, kf_ids, out):
    out.mkdir(parents=True, exist_ok=True); paths = []
    for j, k in enumerate(kf_ids):
        im = cv2.imread(str(dump / "img" / f"{k:05d}.png"))
        H, W = im.shape[:2]; c = (W - 864) // 2
        im = cv2.resize(im[:, c:c + 864], (512, 320), interpolation=cv2.INTER_AREA)
        p = out / f"{j}.png"; cv2.imwrite(str(p), im); paths.append(str(p))
    return paths


def to_video(frames01, path, fps=8):
    import imageio_ffmpeg
    raw = str(path) + ".raw.mp4"; h, w = frames01.shape[1:3]
    vw = cv2.VideoWriter(raw, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for f in frames01:
        vw.write(cv2.cvtColor((np.clip(f, 0, 1) * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
    vw.release()
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-i", raw, "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "16", str(path)], check=True)
    os.unlink(raw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", required=True); ap.add_argument("--dump", required=True); ap.add_argument("--fuse", required=True)
    ap.add_argument("--mode", choices=["sparse", "single"], required=True); ap.add_argument("--pc", choices=["dust3r", "ours"], required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--nbr", type=int, default=2)
    ap.add_argument("--npts", type=int, default=0); ap.add_argument("--npts-from", default=None)
    ap.add_argument("--exclude", type=int, nargs="*", default=[], help="keyframe indices kept out of our point cloud")
    ap.add_argument("--half", type=int, default=1); ap.add_argument("--ds", choices=["voxel", "random"], default="voxel")
    ap.add_argument("--steps", type=int, default=50); ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--ckpt", default=str(VC / "checkpoints" / "model_512.ckpt"))
    ap.add_argument("--dust3r", default=str(VC / "checkpoints" / "DUSt3R_ViTLarge_BaseDecoder_512_dpt.pth"))
    a = ap.parse_args()
    os.chdir(VC); sys.path.insert(0, str(VC)); sys.path.insert(0, str(VC / "extern" / "dust3r"))
    import torch
    _load = torch.load   # torch >= 2.6 defaults weights_only=True; the ViewCrafter/DUSt3R checkpoints (official HF files) pickle
    torch.load = lambda *x, **k: _load(*x, **{**k, "weights_only": False})   # argparse/lightning objects -> load as torch 1.13 did
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    dump, fuse = Path(a.dump), Path(a.fuse)
    tr = np.load(a.traj); meta = json.loads(str(tr["meta"])); c2w = tr["c2w"]; K = tr["K512"]
    cams = np.load(dump / "cams.npz"); T, kf, Kr = cams["T"], cams["kf"], cams["Kr"]
    idx = [meta["anchor"]] + [i for i in meta["inputs"] if i != meta["anchor"]]
    if a.mode == "single":
        idx = idx[:1]
    paths = prep_inputs(dump, [int(kf[i]) for i in idx], out / "inputs")
    dev = "cuda:0"; info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), args=vars(a), traj_meta=meta, input_idx=idx,
                                input_kf=[int(kf[i]) for i in idx], t={}); gp = GpuPeak()
    t0 = time.time(); torch.cuda.reset_peak_memory_stats()
    H, W = 320, 512

    # ---------------- point cloud (our world frame) ----------------
    if a.pc == "dust3r":
        from dust3r.inference import inference, load_model
        from dust3r.utils.image import load_images
        from dust3r.image_pairs import make_pairs
        from dust3r.cloud_opt import global_aligner, GlobalAlignerMode
        model = load_model(a.dust3r, dev)
        imgs = load_images(paths, size=512)
        assert tuple(imgs[0]["true_shape"][0]) == (H, W), imgs[0]["true_shape"]
        if len(imgs) == 1:
            imgs = [imgs[0], copy.deepcopy(imgs[0])]; imgs[1]["idx"] = 1; imgs[1]["instance"] = "1"
        pairs = make_pairs(imgs, scene_graph="complete", prefilter=None, symmetrize=True)
        outp = inference(pairs, model, dev, batch_size=1)
        scene = global_aligner(outp, device=dev, mode=GlobalAlignerMode.PointCloudOptimizer)
        scene.preset_focal([float(K[0, 0])] * len(imgs))
        scene.compute_global_alignment(init="mst", niter=300, schedule="linear", lr=0.01)
        if a.mode == "sparse":
            scene = scene.clean_pointcloud()
        info["t"]["dust3r_s"] = round(time.time() - t0, 2)
        pcd = [p.detach() for p in scene.get_pts3d(clip_thred=1.0)]
        dm = [d.detach() for d in scene.get_depthmaps()]
        Cd = scene.get_im_poses().detach().cpu().numpy()
        info["dust3r_focal"] = scene.get_focals().detach().cpu().numpy().ravel().tolist()
        info["dust3r_pp"] = scene.get_principal_points().detach().cpu().numpy().tolist()
        cols = np.array(scene.imgs)
        if a.mode == "sparse":
            scene.min_conf_thr = float(scene.conf_trf(torch.tensor(3.0)))
            masks = scene.get_masks()
            bg = [d > 0.2 * (torch.max(d[40:-40, :]) + torch.min(d[40:-40, :])) for d in dm]
            masks = [(m + b).cpu().numpy().astype(bool) for m, b in zip(masks, bg)]
            P = np.concatenate([p.cpu().numpy()[m] for p, m in zip(pcd, masks)]); Cc = np.concatenate([c[m] for c, m in zip(cols, masks)])
            To = T[idx]
            Rs = sum(To[i, :3, :3] @ Cd[i, :3, :3].T for i in range(len(idx)))
            U, _, Vt = np.linalg.svd(Rs); R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
            co, cd = To[:, :3, 3], Cd[:, :3, 3]; com, cdm = co.mean(0), cd.mean(0)
            s = float(((co - com) * ((cd - cdm) @ R.T)).sum() / ((cd - cdm) ** 2).sum()); t = com - s * R @ cdm
            res_c = np.linalg.norm(co - (s * cd @ R.T + t), axis=1) * 100
            res_r = [np.degrees(np.arccos(np.clip((np.trace(To[i, :3, :3].T @ R @ Cd[i, :3, :3]) - 1) / 2, -1, 1))) for i in range(len(idx))]
            info["sim3"] = dict(method="rot chordal mean + LSQ centres", scale=s, centre_res_cm=res_c.round(2).tolist(),
                                rot_res_deg=np.round(res_r, 2).tolist())
        else:
            P = pcd[0].cpu().numpy().reshape(-1, 3); Cc = cols[0].reshape(-1, 3)
            Za = np.load(fuse / f"{kf[idx[0]]:05d}.npz")["Z"]; c = (Za.shape[1] - 864) // 2
            Za = cv2.resize(Za[:, c:c + 864], (W, H), interpolation=cv2.INTER_NEAREST)
            Zd = dm[0].cpu().numpy(); ok = np.isfinite(Za) & (Za > 0.05) & (Zd > 1e-6)
            s = float(np.median(Za[ok] / Zd[ok])); R = T[idx[0], :3, :3] @ Cd[0, :3, :3].T
            t = T[idx[0], :3, 3] - s * R @ Cd[0, :3, 3]
            info["sim3"] = dict(method="anchor pose + median depth ratio (fused FFS+DA3)", scale=s, n_px=int(ok.sum()),
                                ratio_p10_p90=np.percentile(Za[ok] / Zd[ok] / s, [10, 90]).round(3).tolist())
        P = (s * P @ R.T + t).astype(np.float32); Cc = Cc.astype(np.float32)
        del model, scene, outp; torch.cuda.empty_cache()
    else:
        sel = sorted({j for i in idx for j in range(i - a.nbr, i + a.nbr + 1) if 0 <= j < len(kf)}) if a.mode == "sparse" else idx
        sel = [j for j in sel if j not in a.exclude]
        Ps, Cs = [], []
        zmax = float(cams["zmax"]); Ki = np.linalg.inv(Kr)
        for i in sel:
            Z = np.load(fuse / f"{kf[i]:05d}.npz")["Z"]; im = cv2.cvtColor(cv2.imread(str(dump / "img" / f"{kf[i]:05d}.png")), cv2.COLOR_BGR2RGB)
            # v2: drop the black undistortion border (and 3 px around it) -> no black specks in the cloud
            border = cv2.dilate((im.max(-1) <= 8).astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
            v, u = np.nonzero(np.isfinite(Z) & (Z > 0.05) & (Z < zmax) & ~border)
            z = Z[v, u]; X = (np.stack([u, v, np.ones_like(u)], 1).astype(np.float64) @ Ki.T) * z[:, None]
            Ps.append(X @ T[i, :3, :3].T + T[i, :3, 3]); Cs.append(im[v, u] / 255.0)
        P = np.concatenate(Ps).astype(np.float32); Cc = np.concatenate(Cs).astype(np.float32)
        n = a.npts or (json.loads(Path(a.npts_from).read_text())["n_points"] if a.npts_from else 512 * 320 * len(idx))
        info["ours"] = dict(keyframes=[int(kf[i]) for i in sel], n_before=int(len(P)), n_target=int(n), zmax=zmax)
        if len(P) > n and a.ds == "random":
            r = np.random.default_rng(0).choice(len(P), n, replace=False); P, Cc = P[r], Cc[r]
        elif len(P) > n:   # v2: voxel grid (mean point + colour per voxel), voxel size bisected to ~n points -> even density
            lo, hi = 1e-4, 0.05
            for _ in range(14):
                vs = np.sqrt(lo * hi); key = np.floor(P / vs).astype(np.int64)
                _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
                if abs(len(cnt) - n) < 0.03 * n:
                    break
                lo, hi = (vs, hi) if len(cnt) > n else (lo, vs)
            inv = inv.ravel(); m = len(cnt)
            P = np.stack([np.bincount(inv, P[:, k], m) for k in range(3)], 1) / cnt[:, None]
            Cc = np.stack([np.bincount(inv, Cc[:, k], m) for k in range(3)], 1) / cnt[:, None]
            info["ours"]["voxel_m"] = float(vs)
        info["ours"]["downsample"] = a.ds
        P, Cc = P.astype(np.float32), Cc.astype(np.float32)
    info["n_points"] = int(len(P)); info["t"]["pc_s"] = round(time.time() - t0, 2)
    np.savez_compressed(out / "pcd.npz", P=P.astype(np.float16), C=(Cc * 255).astype(np.uint8))

    # ---------------- condition render (ViewCrafter's renderer) ----------------
    from pytorch3d.structures import Pointclouds
    from utils.pvd_utils import generate_traj, setup_renderer
    t1 = time.time()
    n = len(c2w)
    cw = torch.from_numpy(c2w).float().to(dev)
    fs = torch.full((n, 1), float(K[0, 0]), device=dev); pp = torch.tensor([[K[0, 2], K[1, 2]]] * n, dtype=torch.float32, device=dev)
    cameras, nv = generate_traj(cw, H, W, fs, pp, dev)
    renderer = setup_renderer(cameras, image_size=(H, W))["renderer"]
    pts = torch.from_numpy(P).to(dev); col = torch.from_numpy(Cc).to(dev)
    with torch.no_grad():
        rend = renderer(Pointclouds(points=[pts], features=[col]).extend(nv))
        cover = renderer(Pointclouds(points=[pts], features=[torch.ones_like(col)]).extend(nv))[..., 0]
    anchor = torch.from_numpy(cv2.cvtColor(cv2.imread(paths[0]), cv2.COLOR_BGR2RGB) / 255.0).float().to(dev)
    rend_raw = rend.clone(); rend[0] = anchor
    info["t"]["render_s"] = round(time.time() - t1, 2)
    info["hole_frac"] = (1 - (cover > 0.5).float().mean((1, 2))).cpu().numpy().round(4).tolist()
    to_video(rend.cpu().numpy(), out / "render.mp4")
    del pts, col, renderer, cameras; torch.cuda.empty_cache()

    # ---------------- diffusion ----------------
    from configs.infer_config import get_parser
    from viewcrafter import ViewCrafter
    opts = get_parser().parse_args(["--config", "configs/inference_pvd_512.yaml", "--ckpt_path", a.ckpt, "--height", str(H),
                                    "--width", str(W), "--video_length", str(n), "--ddim_steps", str(a.steps), "--seed", str(a.seed),
                                    "--device", dev, "--out_dir", str(out)])
    vc = ViewCrafter.__new__(ViewCrafter); vc.opts = opts; vc.device = dev
    t2 = time.time(); vc.setup_diffusion(); info["t"]["load_diffusion_s"] = round(time.time() - t2, 2)
    if a.half:   # fp32 weights (~10 GB) + activations do not fit the 13.5 GB cap: fp16 weights for the sub-networks only;
        m = vc.diffusion   # the noise-schedule buffers of the top-level LatentDiffusion stay fp32 (zero-SNR alphas near 0)
        for sub in ("model", "first_stage_model", "cond_stage_model", "embedder", "image_proj_model"):
            if hasattr(m, sub):
                getattr(m, sub).half()
        torch.cuda.empty_cache()
    info["half_weights"] = bool(a.half)
    info["weights_gb_after_load"] = round(torch.cuda.memory_allocated() / 2 ** 30, 2)
    torch.cuda.synchronize(); t3 = time.time()
    try:
        diff = vc.run_diffusion(rend)            # [-1, 1], (n, H, W, 3)
    except Exception:
        import traceback; traceback.print_exc(); sys.stderr.flush(); raise
    torch.cuda.synchronize(); info["t"]["diffusion_s"] = round(time.time() - t3, 2)
    dif = ((diff.float() + 1) / 2).clamp(0, 1).cpu().numpy()
    to_video(dif, out / "diffusion.mp4")
    np.savez_compressed(out / "frames.npz", render=(rend_raw.cpu().numpy() * 255).astype(np.uint8), diffusion=(dif * 255).astype(np.uint8),
                        cover=(cover.cpu().numpy() > 0.5))
    gp.stop = True
    info["t"]["total_s"] = round(time.time() - t0, 2)
    info["peak_torch_alloc_gb"] = round(torch.cuda.max_memory_allocated() / 2 ** 30, 2)
    info["peak_torch_reserved_gb"] = round(torch.cuda.max_memory_reserved() / 2 ** 30, 2)
    info["peak_gpu_used_mib_nvsmi"] = gp.peak
    info["output"] = dict(frames=int(dif.shape[0]), height=int(dif.shape[1]), width=int(dif.shape[2]), fps_video=8)
    (out / "info.json").write_text(json.dumps(info, indent=1)); print(json.dumps(info, indent=1), flush=True)


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
    import gpu_guard
    with gpu_guard.job("viewcrafter " + " ".join(sys.argv[1:]), torch_cap=True):
        main()
