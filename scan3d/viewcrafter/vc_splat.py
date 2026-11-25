#!/usr/bin/env python3
"""viewcrafter 2026-11-25: render a trained splat (gsm_train.py gaussians.pt) along a vc_traj.py trajectory.

  ~/scanner/gsmap_env/venv/bin/python scan3d/viewcrafter/vc_splat.py RUN_DIR TRAJ.npz OUT_DIR

Same rasterization call as gsm_render.py (gsplat 3DGS, background 0.08 grey where alpha < 1), intrinsics K864 at 864x540
(the 16:10 centre crop of our 960x540 undistorted keyframes). Writes OUT_DIR/frames/NN.png (864x540), splat.mp4 (8 fps)
and alpha.npy (25 x 540 x 864 accumulated opacity).
"""
import json, sys, time
from pathlib import Path

import cv2
import numpy as np


def main():
    run, trp, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    import torch
    from gsplat import rasterization
    (out / "frames").mkdir(parents=True, exist_ok=True)
    ck = torch.load(run / "gaussians.pt", weights_only=False)
    p = {k: v.cuda() for k, v in ck["params"].items()}
    tr = np.load(trp); c2w, K = tr["c2w"], tr["K864"]
    W, H = 864, 540
    Kt = torch.from_numpy(K).float().cuda()
    t0 = time.time(); al = []
    vw = cv2.VideoWriter(str(out / "splat_raw.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 8, (W, H))
    with torch.no_grad():
        for j, T in enumerate(c2w):
            vm = torch.from_numpy(np.linalg.inv(T)).float().cuda()[None]
            rc, ra, _ = rasterization(p["means"], torch.nn.functional.normalize(p["quats"], dim=-1), torch.exp(p["scales"]),
                                      torch.sigmoid(p["opacities"]), torch.cat([p["sh0"], p["shN"]], 1), vm, Kt[None], W, H,
                                      sh_degree=ck["sh_degree"], near_plane=0.02, far_plane=20.0, packed=False)
            im = rc[0] + (1 - ra[0]) * 0.08
            im = cv2.cvtColor((im.clamp(0, 1).cpu().numpy() * 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(out / "frames" / f"{j:02d}.png"), im); vw.write(im); al.append(ra[0, ..., 0].cpu().numpy())
    vw.release()
    np.save(out / "alpha.npy", np.stack(al).astype(np.float16))
    import subprocess, imageio_ffmpeg
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-i", str(out / "splat_raw.mp4"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "18", str(out / "splat.mp4")], check=True)
    (out / "splat_raw.mp4").unlink()
    info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), run=str(run.resolve()), traj=str(trp.resolve()), frames=len(c2w), size=[W, H],
                seconds=round(time.time() - t0, 2), peak_alloc_gb=round(torch.cuda.max_memory_allocated() / 2 ** 30, 2),
                n_gauss=int(p["means"].shape[0]))
    (out / "info.json").write_text(json.dumps(info, indent=1)); print(info)


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "gsplat"))
    import gpu_guard
    with gpu_guard.job("viewcrafter splat render " + sys.argv[1], torch_cap=True):
        main()
