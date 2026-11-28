#!/usr/bin/env python3
"""vc_holdout 2026-11-28 step 2 (GPU, ViewCrafter venv): condition render of the cut cloud along each clip path + ViewCrafter-512
diffusion (model loaded once, 8 clips, each clip saved as soon as it is done -> resumable). Rules: GATE.md.

  cd ~/scanner/viewcrafter_env/ViewCrafter
  SCAN3D_GPU_FRAC=0.85 OMP_NUM_THREADS=8 nice -n 10 ../venv/bin/python -W ignore RUN/run.py [--render-only]
Condition render = ViewCrafter's own pytorch3d point renderer (utils/pvd_utils: radius 0.01, 10 pts/px, black bg), frame 0 = start photo.
Diffusion settings as vc_run.py: configs/inference_pvd_512.yaml, 50 DDIM, fp16 sub-network weights.
"""
import argparse, json, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
VC = Path.home() / "scanner/viewcrafter_env/ViewCrafter"
sys.path.insert(0, str(Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/viewcrafter"))
sys.path.insert(0, str(Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/gsplat"))


def main(a):
    import os
    os.chdir(VC); sys.path.insert(0, str(VC))
    import torch
    _load = torch.load
    torch.load = lambda *x, **k: _load(*x, **{**k, "weights_only": False})
    from pytorch3d.structures import Pointclouds
    from utils.pvd_utils import generate_traj, setup_renderer
    from vc_run import GpuPeak, to_video
    dev = "cuda:0"; H, W = 320, 512; gp = GpuPeak(); vc = None
    for reg in ("R1", "R2"):
        pd = HERE / "prep" / reg; info = json.loads((pd / "info.json").read_text()); K = np.array(info["K512"])
        cl = np.load(pd / "cloud.npz"); pts = torch.from_numpy(cl["P"]).float().to(dev); col = torch.from_numpy(cl["C"]).float().to(dev) / 255
        for c in info["clips"]:
            out = HERE / "clips" / c["clip"]; out.mkdir(parents=True, exist_ok=True)
            if (out / "frames.npz").exists() and not a.render_only:
                print("skip", c["clip"], flush=True); continue
            c2w = torch.from_numpy(np.load(pd / c["path"])).float().to(dev); n = len(c2w)
            rend, cover = [], []
            t1 = time.time()
            with torch.no_grad():
                for i in range(n):
                    cams, _ = generate_traj(c2w[i:i + 1], H, W, torch.full((1, 1), float(K[0, 0]), device=dev),
                                            torch.tensor([[K[0, 2], K[1, 2]]], dtype=torch.float32, device=dev), dev)
                    rr = setup_renderer(cams, image_size=(H, W))["renderer"]
                    rend.append(rr(Pointclouds(points=[pts], features=[col]))[0])
                    cover.append(rr(Pointclouds(points=[pts], features=[torch.ones_like(col)]))[0, ..., 0])
            rend = torch.stack(rend); cover = torch.stack(cover); rend_raw = rend.clone()
            start = torch.from_numpy(cv2.cvtColor(cv2.imread(str(pd / c["start_png"])), cv2.COLOR_BGR2RGB) / 255.0).float().to(dev)
            rend[0] = start
            meta = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), **c, render_s=round(time.time() - t1, 2),
                        hole_frac=(1 - (cover > 0.5).float().mean((1, 2))).cpu().numpy().round(4).tolist())
            cv2.imwrite(str(out / "cond_last.png"), cv2.cvtColor((rend_raw[-1].cpu().numpy() * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
            cv2.imwrite(str(out / "cover_last.png"), ((cover[-1] > 0.5).cpu().numpy() * 255).astype(np.uint8))
            print(c["clip"], "render", meta["render_s"], "s, hole first/last", meta["hole_frac"][0], meta["hole_frac"][-1], flush=True)
            if a.render_only:
                (out / "render_meta.json").write_text(json.dumps(meta, indent=1)); continue
            if vc is None:
                from configs.infer_config import get_parser
                from viewcrafter import ViewCrafter
                opts = get_parser().parse_args(["--config", "configs/inference_pvd_512.yaml", "--ckpt_path", str(VC / "checkpoints/model_512.ckpt"),
                                                "--height", str(H), "--width", str(W), "--video_length", str(n), "--ddim_steps", "50",
                                                "--seed", str(c["seed"]), "--device", dev, "--out_dir", str(out)])
                vc = ViewCrafter.__new__(ViewCrafter); vc.opts = opts; vc.device = dev
                t2 = time.time(); vc.setup_diffusion(); load_s = round(time.time() - t2, 2)
                m = vc.diffusion
                for sub in ("model", "first_stage_model", "cond_stage_model", "embedder", "image_proj_model"):
                    if hasattr(m, sub):
                        getattr(m, sub).half()
                torch.cuda.empty_cache(); print("model load s", load_s, flush=True)
            from pytorch_lightning import seed_everything
            seed_everything(c["seed"]); vc.opts.seed = c["seed"]
            torch.cuda.synchronize(); t3 = time.time()
            diff = vc.run_diffusion(rend)
            torch.cuda.synchronize(); meta["diffusion_s"] = round(time.time() - t3, 2)
            dif = ((diff.float() + 1) / 2).clamp(0, 1).cpu().numpy()
            to_video(dif, out / "diffusion.mp4"); to_video(rend.cpu().numpy(), out / "render.mp4")
            np.savez_compressed(out / "frames.npz", render=(rend_raw.cpu().numpy() * 255).astype(np.uint8),
                                diffusion=(dif * 255).astype(np.uint8), cover=(cover.cpu().numpy() > 0.5))
            cv2.imwrite(str(out / "vc_last.png"), cv2.cvtColor((dif[-1] * 255).astype(np.uint8), cv2.COLOR_RGB2BGR))
            meta["peak_gpu_mib_nvsmi_so_far"] = gp.peak; meta["peak_torch_alloc_gb"] = round(torch.cuda.max_memory_allocated() / 2 ** 30, 2)
            (out / "meta.json").write_text(json.dumps(meta, indent=1))
            print(c["clip"], "diffusion", meta["diffusion_s"], "s", flush=True)
        del pts, col; torch.cuda.empty_cache()
    gp.stop = True


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--render-only", action="store_true"); a = ap.parse_args()
    import gpu_guard
    with gpu_guard.job("vc_holdout " + " ".join(sys.argv[1:]), torch_cap=True):
        main(a)
