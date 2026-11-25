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
- 11:30 v3 (user order): run_v3.sh = border mask + anti-spike (relu(log(smax/smin) - log 5) mean, w 1.0; scales clamped <= 5 cm).
  A2 done 11:35 (masked test PSNR 33.9, full-frame 19.3 - not comparable with A 25.5 full-frame; dark opaque 5.5k vs A 13.6k;
  aniso>5 opaque 363 vs A 359k; floaters 0.8 % vs 6.0 %). ply copied to scratchpad splat_demo/gsA2.ply. B2 running, auto-copies gsB2.ply.
- 11:40 B2 done (masked PSNR 33.93, dark 7.7k, aniso>5 305, floaters 0.9 %), gsB2.ply copied.
- 11:41 gs2D (user order): gsm_train --surfel 1 (gsplat rasterization_2dgs, normal 0.05 from 7000, distortion 0.01 from 3000), FFS only,
  border mask, no anti-spike. 380 s, 528k surfels, masked PSNR 33.72, dark 8.0k, floaters 0.5 %, but 89.5k opaque discs with
  in-plane aspect > 5 (needle discs). Export: scale_2 = log(1e-4). Copied as gs2D.ply. Script run_2d.sh.

## ViewCrafter test (12:10-13:10, generative novel views from the side)
- Env `~/scanner/viewcrafter_env/` (own venv py3.12, torch 2.11 cu128 from uv cache, pytorch3d 0.7.9 built from source for sm_120
  with `env/build_p3d.sh` (needs NVCC_FLAGS -I for pip nvidia headers, no trailing space), open-clip-torch 2.17.1 (2.24 breaks:
  no input_patchnorm), PL 1.9.5, transformers 4.40). Repo clone `ViewCrafter/` @97cdb86 + local patch `env/viewcrafter_local.patch`
  (xformers -> torch SDPA shim; must pass 4-D tensors or SDPA falls back to the math kernel and OOMs).
  Weights: ViewCrafter_25_512 (HF, apache-2.0, 10.4 GB), ViewCrafter_25_sparse (1024, downloaded, NOT run: 23.5 GB official),
  DUSt3R ViT-L 512 dpt from HF safetensors converted by `env/hf_dust3r_to_pth.py` (naverlabs server ~1 MB/s, stalled; DUSt3R = CC BY-NC-SA 4.0),
  open_clip ViT-H-14 laion2b in ~/.cache/huggingface.
- Scripts `scan3d/viewcrafter/`: vc_traj.py (side orbit 60 deg about up axis through a point 0.80 m in front of kf 540; off-path
  0 -> 49 deg), vc_splat.py (gsA2 along the path), vc_run.py (DUSt3R or our fused FFS+DA3 point cloud -> ViewCrafter renderer ->
  512 diffusion, fp16 sub-network weights, cap 0.85), run_vc.sh (4 runs), vc_compose.py (grid videos, stats), run_check.sh +
  vc_check.py (known-answer path kf 540 -> held-out kf 970). Outputs in ~/scanner/viewcrafter_env/work/ (not in git).
- Result (n=1): runs on the 5080 at 320x512x25 frames, ~41 s diffusion + 15 s DUSt3R + ~45 s model load, peak 10.9 GB (nvidia-smi).
  Side views look clean (no spikes/holes) but content is invented (AprilTags turn into other pictograms, extra objects).
  Held-out kf 970: gsA2 splat PSNR 26.8 / LPIPS 0.061 vs ViewCrafter 16.3 / 0.292 (DUSt3R PC), 15.5 / 0.355 (our PC).
  Our PC vs DUSt3R PC: no clear winner. First our-PC run used random subsampling + black border points (kept as *_v1random) ->
  replaced by border mask + voxel-mean downsample to the DUSt3R point count.
  Report: scratchpad splat_demo/viewcrafter_165506.html (media vc165506/).
- Not done: 1024 model, other seeds, chaining clips > 25 frames, fp16/SDPA vs fp32/xformers equivalence.
- 11-25 16:16-17:03 run gsmap_20261125_121801 (hs_20261112_121801_std60, run.sh): 494 BA kf (fixes fix_u8), lensonly stand-in = raw VIO +1000 m z
  (se3_rmse 7.98 cm = BA vs raw VIO, not a GT); dump reframed to VIO world y-down (gsm_reframe.py). 2DGS OOM at 8.8M init points ->
  init voxel 2.5 cm, max-gs 2.5M, uint8/half GPU buffers. Result: PSNR(valid) 26.9, 2.51M Gaussians (cap hit), floaters 2.4 %,
  train 24 min, mesh 17.3M tri -> decimated 1.5M, texture 385 s, GLB 80 MB (atlas 14k -> 8192). Black cracks in renders = mesh holes.
- gsm_train: --gs-per-m2 (coarse 2 cm TSDF area cap; growth stops, pruning continues; default 200k/m2 from 165506 = 211k/m2),
  --steps default 12000. Not run yet (user: later).
- 17:52 PC power-off (FFS dense GPU + planes texture CPU ran together). Rule now: strictly one heavy job, nice, OMP 8.
- (a) ffs_dense done (gsm_ffs_dense.py, partners from full 60 fps video +-2 s, VIO-relative poses on BA kf, <= 3 partners,
  B 3-25 cm, perp >= 30): 411/494 kf with FFS (old 361; all old 361 + 50 new). valid_frac median 0.40 (old 0.42).
  83 still fail: 78 "baseline" (< 3 cm motion within 2 s), 5 "forward"; idx runs 55-79, 88-92, 217-226, 233-246, 259-262,
  275-279, 316-321, 332-345. Multi-view fail of the new FFS 27.7 % (old FFS 7.5 % same rule) -> new pairs noisier.
  Decision for (c): old FFS for the 361 old kf, new FFS only for the 50 new kf (ffs_hybrid).
- next: (b) planes texture -> (c) mesh fixes + retrain v2 -> (d) eval with scratchpad diag scripts -> (e) HTML.
- (b) planes done 18:32: gsm_planes (weld 6 mm: decimated mesh was 84k pieces; components < 200 tri / 0.005 m2 removed = 29.8 % faces,
  < 3 views 0.34 %; 65 plane regions, 6.8 % vertices snapped, 2 boxes, 1109 plane holes 0.075 m2) + texture2 (xatlas 29 min!) ->
  splat_demo/tex_121801_planes.glb (50.6 MB), planes_121801.jpg.
- (c) v2 run dir scan3d/runs/gsmap_20261125_121801_v2 (dump_ba = symlinks: old img, ffs hybrid 361 old + 50 dense). run_v2.sh part 1
  (align, align_nb, filter, train 12k, mesh weld + trunc 4/6 cm with render_cache) started 18:33. Resume = rerun run_v2.sh.
  Then: gsm_eval_mesh.py (ref = old run FFS) on old glb, planes glb, t4, t6 -> pick -> texture2 -> atlas8k -> tex_121801_v2.glb.
- 19:08 part1 done. eval (gsm_eval_mesh, ref = old FFS): t4 picked (holes 4.5 vs t6 4.1 %, front vs own 2DGS 1.8 vs 3.2 %); planes step skipped (old planes: holes 10.1 -> 12.7 %, side empty 18.3 -> 25.6 %). texture t4 started.
- 20:18 texture2 (xatlas) on welded t4 mesh killed after 70 min (xatlas is slow on welded meshes: v1 unwelded 77 s, planes 29 min). Fallback: per-face atlas gsm_texture.py 8192.
- 20:22 texture2 --chunk 0.75 (xatlas per 0.75 m chunk, packed together): 50 s instead of 70+ min. tex_121801_v2.glb 68 MB.
- (d) eval (gsm_eval_mesh, ref old FFS): holes 10.1 -> 4.5 %, side empty 18.3 -> 10.8 %, panel RMS 11.3 -> 4.4 cm, walls 5.6 -> 5.3 cm,
  front 13.6 -> 17.5 % (worse), PSNR 26.9 -> 26.3 (12k vs 20k steps). Crops v2_121801.jpg. (e) report_121801_v2.html. DONE, stopped.
- 20:35-20:50 v3 (v2 mesh edit only): gsm_v3fix.py (floor y 0.816 / ceiling y -2.117 planes, footprint close 0.8 m, 4 cm cells:
  floor coverage 78.8 -> 100 % (+3.11 m2), ceiling 87.5 -> 100 % (+0.37 m2); islands removed 0.13 % faces) -> runs/..._v2/v3/mesh_v3.json.
  texture2 --chunk 0.75 --fill-all 1: magenta 0 %, 245,862 faces (15.8 %) nearest-colour filled -> v3/tex_121801_v3.json.
  eval eval/v3_glb.json: holes 4.49 %, front 17.49 %, side empty 10.6 %, panel 4.4 cm, walls 5.46 cm. Ceiling chunks remain.
  splat_demo: tex_121801_v3.glb, v3_121801.jpg, report v3 section.
- 20:50 E0 (plan C:/Users/a3162/scan3d_handoff/ideas/gsmap_perfect_plan_20261125.md): gsm_eval2_ref.py -> runs/..._v2/eval2/ref
  (gold FFS 13.07M px = 82.6 % of old filtered FFS, 353 kf; 2DGS depth all 494 kf 480x270; room_poly 49.7 m2 (room_poly.png checked
  by eye once); 6 ROIs (roi_pick.jpg); 24 inside poses). gsm_eval2.py = P1-P4, S1-S7. eval2/run_e0.sh: held-out textures
  (texture2 --skip-held) + eval for old / v2 / v3 -> eval2/{old,v2,v3}.json.
- E1 gsm_carve.py (free-space carve + islands + < 200 tri), e1/run_e1.sh sweeps min-viol 3/5/10 after E0 -> e1/eval_carve_mv*.json.
