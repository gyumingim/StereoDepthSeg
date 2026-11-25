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
- 21:11 E0 baselines (eval2/{old,v2,v3}.json; P4 from eval2/held/*.glb built without held-out views):
  | | old | v2 | v3 |
  | P1 floater area % N3/N5/N10 | 6.60/6.11/4.79 | 0.71/0.59/0.42 | 0.83/0.71/0.51 |
  | P2 islands count / area % | 0 / 0 (old mesh is all < 1 m2 pieces: P2 undefined) | 5748 / 14.16 | 5233 / 14.91 |
  | P3 correct % | 82.21 | 83.74 | 83.74 |
  | P4 held-out PSNR / SSIM / LPIPS / hit / hit-only PSNR | 14.99/0.440/0.572/0.902/18.01 | 17.59/0.581/0.363/0.965/18.78 | 17.60/0.582/0.361/0.965/18.78 |
  | S1 front / hole % | 9.33/8.47 | 12.72/3.55 | 12.72/3.55 |
  | S4 floor / ceiling coverage of room_poly % | 19.8/3.8 | 23.0/5.8 | 29.3/6.5 |
  | S5 empty / backface % | 22.2/13.7 | 17.3/12.9 | 17.0/12.9 |
  | S7 area m2 / components | 131.9/84086 | 176.4/11424 | 179.7/10323 |
- 21:25 E1 sweep (e1/eval_carve_mv{3,5,10}.json): carve + islands + small comps removes 16.7-16.9 pct area (islands 14.4-14.5):
  P1 N5 0.71 -> 0.06, P2 -> 0, but P3 83.74 -> 83.08 (-0.66 pt) and P4 17.60 -> 16.18 dB (hit share 0.965 -> 0.927): all FAIL.
  Extra variant carve only (mv5, no island / small step, e1/eval_carve_mv5_aonly.json): removes 0.69 pct area, P1 N5 0.71 -> 0.077,
  P3 83.78, P4 17.58 (-0.02 dB = noise), P2 unchanged 14.7 pct -> passes the generic keep rule, not the E1 rule (P2 not down).
  Adopted as v4 (e1/tex_121801_v4.glb, full-view texture). Island step reverted: the islands are mostly real fragmented surfaces.
- 21:40 v4 delivered: splat_demo/tex_121801_v4.glb (66.7 MB, e1/tex_121801_v4.json: unseen 15.7 pct nearest-colour, magenta 0), sheet v4_121801.jpg (8 crops v3 vs v4: fewer specks on walls/floor; ceiling chunks remain). Stopped after E1 for the Fable judge.
- 21:45 judge round 1 (plan section 5). gsm_eval2 updated: P2 = inside defects at 48 poses (P2a back-face, P2b front px on welded
  components < 0.25 m2, P2c empty), P4 rendered 960x540 -> area 480x270 + lowres 240x135 + per-image colour fit, kf index 0 dropped
  (also from texturing: texture2 --drop-views 0). P1 -> S9, old P2 -> S8.
  v4 re-baseline r2/eval_v4.json: P2a 9.21, P2b 5.99, P2c 13.40 %, P3 83.78, P4 18.455 dB (hit-only 19.655, colour-fit 20.362,
  lowres 20.691, hit share 0.949), S1 front 12.61 / hole 3.61, S4 floor 29.1 / ceiling 6.2 % (v3 planes), area 178.6 m2.
- E3 gsm_e3planes.py on v4 (floor = v3 fit, ceiling parallel to floor, shards within 0.6 m below ceiling removed, both planes filled
  over room_poly, floor snap +-3 cm). Photo-consistency height sweep NOT implemented (fallback histogram mode).
  E3a (ceiling = topmost mode y -1.842, 2.66 m above floor) r2/eval_e3.json: P2c 13.40 -> 9.35 (-4.05 pt), P2a 9.22, P2b 6.27,
  P3 83.78 -> 82.37 (-1.41), P4 18.485, S1 front 14.06.
  E3b (ceiling y -2.117 = v3 fit height) r2/eval_e3b.json: P2c 10.13 (-3.27), P3 82.37, P4 18.442.
  Both FAIL (need P2c -5 pt; P3 -1.41 pt). Likely cause of the P3 drop: the floor plane (fit on a 14.8 m2 patch, 0.8 % tilt)
  is not the floor everywhere, so generated floor sits above the real floor in places (S1 front +1.45 pt). E3 stopped (2 fails).
- 22:45 E2 (e2/run_e2.sh, e2/run_e2_plain.sh; e2/mo*/eval.json, mesh.json): TSDF re-extraction -> v3fix -> carve mv5 a-only.
  | variant | P2a | P2b | P2c | P3 | P4 PSNR / hit / hit-only | S1 front / hole | area m2 | no-photo % |
  | v4 base | 9.21 | 5.99 | 13.40 | 83.78 | 18.455 / 0.949 / 19.655 | 12.61 / 3.61 | 178.6 | 15.7 |
  | mo4 + alpha 0.7 + edge + drop floaters | 7.47 | 5.33 | 25.22 | 81.20 | 14.295 / 0.820 / 18.284 | 10.91 / 7.89 | 99.7 | 5.4 |
  | mo8 (same flags) | 6.47 | 7.56 | 31.22 | 78.36 | 13.026 / 0.749 / 17.792 | 10.26 / 11.38 | 78.2 | 4.1 |
  | mo12 (same flags) | 5.62 | 9.38 | 37.26 | 73.96 | 12.044 / 0.674 / 17.591 | 9.45 / 16.59 | 60.8 | 3.2 |
  | mo4 only + drop floaters (alpha 0.5, no edge) | 8.72 | 6.05 | 15.91 | 84.18 | 17.716 / 0.929 / 19.434 | 11.91 / 3.91 | 157.6 | 13.1 |
  Keep rule (P2a + P2b -30 % rel, P3 not -0.5 pt): best is -16 % (mo4) with P3 -2.6 and P4 -4.2 dB -> all FAIL. min_obs and the
  alpha/edge flags mainly open holes (P2c up, hit share down); flakes barely change. E2 stopped (2 attempts).
- 23:03 E4 (e4/eval.json; mesh zmax 5 m + sigma x max(1, Z/3), then v3fix + carve): P2a 9.52, P2b 5.99, P2c 13.40 -> 11.90 (-1.5 pt),
  P3 83.56 (-0.22), P4 18.455 -> 19.016 dB (hit share 0.949 -> 0.965, hit-only 19.645), area 194.3 m2. E4 rule needs P2c -2 pt -> FAIL
  (narrow; P4 +0.56 dB). Not kept standalone; E6 re-fuse/mesh uses zmax 5 + depth-weighted sigma as the plan defines.
- E1b (e1b/eval.json; v4 + delete components < 0.25 m2 with no gold-FFS-agreeing view): removed 10,820 components = 15.1 % area.
  P2b 5.99 -> 0.97 (-84 %), P2a 7.55, P2c 16.92 (+3.5), P3 83.81, hit share 0.949 -> 0.919 (-3 pt), hit-only 19.655 -> 19.51 (-0.145 dB)
  -> FAIL (hit share and hit-only limits). Most unsupported pieces are real surfaces beyond FFS range (> 2 m).
- 23:05 E6 pre-check (e6/precheck.json, 60 FFS keyframes, fit near half / test far half): far-half median rel. error
  MoGe affine 5.6 %, MoGe scale-only pooled +-12 6.35 %, DA3 pose-conditioned raw 2.0 % (-64 %), DA3 scale pooled 2.2 %. -> E6 with DA3 raw.
  DA3 run: depth_da3 (494 views, chunks 100 / overlap 20, ~30 s, 8.3 GB).
- 23:36 E6 (e6/run_e6.sh: fuse = filtered FFS + DA3 raw fill, gate >= 2 kf >= 10 cm baseline within 3 %, zmax 5 (kept 91.5 % of AI px);
  retrain 12k; mesh t4 zmax 5 + depth sigma; v3fix; carve) e6/eval.json:
  P2a 9.21 -> 8.60, P2b 5.99 -> 2.45 (P2a+b -27 % rel), P2c 13.66 (+0.26), P3 83.50 (-0.28), P4 18.455 -> 16.939 dB
  (hit-only 18.13, colour-fit 20.27 vs v4 20.36, hit share 0.930), S1 front 12.35 / hole 4.15, area 181.5 m2.
  ROI RMS: door 9.07 -> 3.85, wall538 4.52 -> 3.90, panel 4.41 -> 5.97, cabinet 3.48 -> 5.80 cm.
  Keep rule FAILS (P4 -1.5 dB; P2a+b -27 % < 30 %). But the texture per-view gains hit the clip (0.74..1.60 vs v4 0.90..1.50) and the
  colour-fitted PSNR is equal -> P4 drop looks like a texture-gain artifact. Diagnostic e6/nogain (both meshes textured without gains) running.
- 00:00 E6 no-gain diagnostic (e6/nogain/eval_{v4,e6}.json, textures without per-view gains): v4 18.544 dB (hit-only 19.755,
  hit 0.949), E6 17.929 (hit-only 19.391, hit 0.930). Part of the 1.5 dB was the gain solve, but E6 is still -0.6 dB (-0.36 hit-only)
  -> E6 NOT kept. E7 runs on the v2 fuse (fuse/moge_f), v4 mesh settings (zmax 3). e7/run_e7.sh.
- 01:15 judge round 3: E7 (30k steps, exposure affine, MoGe normal prior 0.05) on both fuses; evaluated with texture gains OFF
  (e7/eval_ng.json, e7da3/eval_ng.json):
  | | E7-MoGe (v2 fuse, zmax 3) | E7-DA3 (e6 fuse, mesh zmax 5) |
  | 2DGS test PSNR | 26.20 | 27.01 |
  | P2a / P2b / P2c (sum) | 8.98 / 5.52 / 16.12 (30.62) | 8.34 / 2.63 / 15.45 (26.42) |
  | P3 | 88.55 | 88.06 |
  | P4 psnr / hit-only / colour-fit / hit share | 16.88 / 19.29 / 20.01 / 0.901 | 17.54 / 19.20 / 20.18 / 0.916 |
  | S1 front / hole | 9.22 / 2.23 | 9.26 / 2.68 |
  | ROI RMS panel / cabinet / door / wall538 (cm) | 5.01 / 3.78 / 4.61 / 3.80 | 8.07 / 8.17 / 3.76 / 1.57 |
  | area m2 / no-photo % | 138.0 / 13.4 | 155.0 / 11.3 |
  Pick (round-3 order): a. P2 sum 26.42 vs 30.62 (gap 4.2 > 1) -> DA3; b. P3 within 1 pt (0.49); c. hit-only within 0.5 dB (0.09)
  -> WINNER E7-DA3 = v5, E7-MoGe = v6. Known cost: panel 8.1 / cabinet 8.2 cm RMS (cabinet second layer 14 %).
  vs v4: P3 83.78 -> 88.06, P2 sum 28.60 -> 26.42, S1 front 12.6 -> 9.3. final/run_final.sh: v5/v6 full textures (no gains, kf 0
  dropped) + inside sheet + E3c.
- 01:33 v5 = E7-DA3 (final/tex_121801_v5.glb, 63 MB), v6 = E7-MoGe (final/tex_121801_v6.glb, 65 MB), both textured with all views
  except kf 0, no gains; inside sheet final/inside_121801_v5_v6.jpg (v4 / v5 / v6). All copied to splat_demo.
- E3c on v5 (final/e3c.json, final/eval_e3c.json): floor y 0.819 (gravity normal, median of 42,965 inliers), ceiling y -1.810 (topmost
  mode, 2.63 m above floor, NOT verified); filled floor 31.9 m2 / ceiling 45.6 m2; see-through guard blocked only 234 + 331 cells
  (ceiling cells are rarely inside any keyframe frustum, so the guard can hardly act there).
  P2a 8.34, P2b 2.65, P2c 15.45 -> 11.06 (sum 26.42 -> 22.05), P3 88.06 (=), P4 17.54 -> 17.63 (hit-only 19.20 -> 19.10, hit 0.916 -> 0.929),
  S1 unchanged 9.26 / 2.68 -> KEPT (generic rule). E8 started (e8/run_e8.sh, E3C=1, all 494 views, 30k steps).
- 02:11 E8 FINAL (e8/run_e8.sh, e8/eval.json): E7 settings on the E6 DA3 fuse, ALL 494 views (hold 0), 30k steps (1581 s, 1.71 M
  Gaussians, train PSNR 27.75), mesh zmax 5 + depth sigma, v3fix, carve mv5, E3c (floor y 0.821, ceiling y -1.812, filled 32.2 + 45.7 m2),
  texture all views except kf 0, no gains -> splat_demo/tex_121801_final.glb (66.7 MB), sheet splat_demo/final_121801.jpg (v4 / v5 / final).
  P2a 8.74, P2b 2.76, P2c 10.54 (sum 22.04; v4 28.60), P3 88.25 (v4 83.78), S1 front 9.29 / hole 2.46, S4 floor 98.0 / ceiling 19.9 %
  (S4 ceiling uses the old tilted v3 plane), area 233.6 m2, no-photo 15.4 % (nearest colour), magenta 0. P4 not measurable
  (no held-out views); last held-out build = v5 + E3c: 17.63 dB (hit-only 19.10).
  ROI RMS panel 8.13 / cabinet 6.20 (second layer 15.7 %) / door 2.19 / wall538 1.42 / floor 0.39 cm. Visual: ceiling chunks gone,
  ceiling + floor closed, desk top much cleaner; cabinet front has see-through patches (worse than v4), ceiling height unverified.
  Stopped after E8.
- 02:27 audit 2 ceiling fix: gsm_e3c --ceil-y (audit: true ceiling y ~ -1.855 from DA3 horizontal-normal histogram + photo-consistency
  sweep over 7 pairs). e8/c1855 (e3c on e8/carve.ply with --ceil-y -1.855, texture no gains, kf 0 dropped; e8/c1855/eval.json):
  P2a 8.74 / P2b 2.76 / P2c 10.58 (sum 22.08 vs 22.04), P3 88.25 (=), S1 9.29 / 2.46, area 234.9 m2. Crop e8/c1855/ceiling_cmp.jpg:
  lamp single and sharp in "ceiling +20" -> KEPT. splat_demo/tex_121801_final.glb = c1855; old one kept as tex_121801_final_c181.glb.
- Audit 2 corrected numbers (Fable audit, scratchpad audit2/): like-for-like P2 sum v4 + E3c 24.8 vs final 22.0 (the 28.6 -> 22.0 gain
  above mixes in E3c); REAL (not generated) floor / ceiling coverage of the room polygon 35.6 / 8.5 %; real (not generated) mesh area
  154 m2; cabinet and panel REGRESSED vs v4 (hit share 0.72 / 0.69 on those ROIs).
- Stopped (no retrains).
