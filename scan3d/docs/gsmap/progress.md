# gsmap 2026-11-25 — photo-like Gaussian-splat map on 165506 (progress / resume notes)

Branch `gsmap-20261125` (from `gsplat-20261105` 090c27c), worktree `~/scanner/StereoDepthSeg-gsmap`. Desktop RTX 5080, one GPU job at a time
(`~/scanner/gpu.lock` via `scan3d/gsplat/gpu_guard.py`, torch cap 0.75). Phone not touched. Goal: Tesla "Reality View"-like look, not metric accuracy.

## Inputs (reused, not recomputed)
- Dump `scan3d/runs/gsmap_20261125/dump_ba` -> symlink to `~/scanner/StereoDepthSeg-gsplat/scan3d/runs/gsplat_20261105/dump/ba`
  (134 keyframes, every 10th frame 300..1680 with a pose, 960x540 undistorted PNG, BA poses `step2/ba_o0` of scanA_20261105 at VIO scale,
  SE(3)-aligned to the board frame (world y = down), FFS depth for 93 keyframes; 41 keyframes have no FFS at all).
- Env: `~/scanner/gsmap_env/venv` (python 3.12, reads gsplat_env + venv_ffs site-packages via .pth; own: depth_anything_3 (git clone, -e),
  moge (git clone, -e), utils3d_moge, lpips, plyfile, trimesh, moviepy 1.0.3, imageio-ffmpeg). torch 2.11 cu128 + gsplat 1.5.3 from the old envs.
  Weights in `~/.cache/huggingface/hub`: depth-anything/DA3-LARGE-1.1, Ruicheng/moge-2-vitl-normal. LPIPS alexnet in ~/.cache/torch.

## Steps run (all under `scan3d/runs/gsmap_20261125/`, not in git)
1. `depth_ai.py --model da3` -> `depth_da3/` (all 134 views in one pass, pose-conditioned with our BA w2c + Kr, process_res 504 -> 280x504,
   upsampled nearest to 960x540; 25 s, 9.8 GB). log `depth_da3.log`.
2. `depth_ai.py --model moge` -> `depth_moge/` (single image, fov_x from Kr 73.5 deg; 32 s, 2.3 GB).
3. `align_fuse.py` -> `fuse/{ffs,da3,moge}/` + `fuse/align_stats.json` (per-keyframe robust scale+shift to FFS, residuals, fused depth).
4. `run_all.sh` -> `gsA` (FFS only), `gsB` (FFS + DA3 fill), `gsC` (FFS + MoGe-2 fill): `gsm_train.py` (3DGS, 20000 steps, hold-out every
   8th keyframe = 17 test / 117 train) then `gsm_render.py` (chase.mp4, side_*.jpg, slice.png). Logs `gsX.log`, GPU samples `gsX.gpu.txt`.
   Smoke run (1500 steps, da3) in `smoke/` — only for checking the code.

## Resume
`bash scan3d/runs/gsmap_20261125/run_all.sh` skips finished variants. To refine a map later: load `gsX/gaussians.pt`
(params + T, Kr, kf, test split, args) and continue the loop of `gsm_train.py`; `gsX/splat.ply` is the INRIA 3DGS layout (SH degree 3).

## Status log
- 10:45 worktree, env, weights. 10:49 DA3 + MoGe depth done. 10:50 align/fuse done. 10:53 smoke OK (fixed packed=False, chase roll sign).
- 10:58 run_all.sh started (A, B, C sequential).
- 11:10 A/B/C done (each ~5 min train, 2.8 GB). B and C: held-out PSNR about the same as A, but more floaters (free-space check 13.8 / 13.6 % vs 6.0 %)
  and ~10x more dark opaque Gaussians (135k / 128k vs 14k) -> black streaks in the chase video from off-trajectory views.
- 11:15 `fill_check.py` (multi-view free-space check of filled pixels). Control on FFS pixels themselves: with ">= 2 views" rule 58 % of FFS
  pixels "violate" -> rule too strict; with ">= 2 views AND >= 30 % of checkable views": FFS 12 %, DA3 fill 18 %, MoGe fill 24 % dropped.
  -> the check separates fill from FFS only weakly (finding, n=1).
- 11:17 `run_de.sh`: D = DA3 fill after the check (fuse/da3c_0.3), E = DA3 fill with no depth loss on fill (--fill-w 0).
- 11:20 D done (floaters 5.8 %, but dark opaque Gaussians still 119k). Cause guess: black undistortion border (4.6 % of pixels) -> trainer got
  --mask-border (photo/depth/init/eval ignore it). E (--fill-w 0) killed unfinished. v2 run (run_v2.sh: A2,B2,D2,C2) started 11:23 and
  STOPPED by user order during A2 (gsA2_stopped/, incomplete). Report: scratchpad splat_demo/result_165506.html (media in m165506/).
- Resume: `bash scan3d/runs/gsmap_20261125/run_v2.sh` (delete gsA2_stopped first or leave; it trains gsA2 fresh).
