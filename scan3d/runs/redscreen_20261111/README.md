# redscreen 2026-11-11: scanner scale from a red laptop screen + ID-1 card (no ChArUco)

Recording: hs_20261111_202906_std60 (S24 cam5 1920x1080 60 fps, 58 s, focus 0.5 dpt, std, complete, 0 unmatched). One recording only.
Branch redscreen-20261111 from fa7558a (kfsel = scanner defaults 11-11).

## Pre-registration (written before any measurement, 2026-11-11)
References (both 가정 = not measured in this test):
- Red area = full-screen 1920x1200 px of the laptop panel. Pitch 0.15729 mm/px from EDID (LEN88AE/SDC4208, 302 x 189 mm; project choice
  2026-10-24 in imu_gt_spl/board.json of 165506; ruler 10-21 gave 0.15756, +0.17 %) -> W 302.00 mm, H 188.75 mm, diag 356.1 mm.
  The task text gave 301.6 x 188.5 (0.1571 mm/px); difference 0.13 %, reported alongside. Assumes it is the same laptop as 165506.
- Card ID-1: 85.60 x 53.98 mm (ISO/IEC 7810), corner radius ~3.18 mm.
Scanner run: run_scanner.sh defaults (motion keyframes, kf_ba GPU BA, FFS all/2, TSDF dense) on the laptop worker.
  No GT folder in this recording -> run_scanner cannot pick a lens by itself. Lens = std_1920x1080_f0.50_calcap20261026.json
  (the 0.5 dpt app lens that cfg_gen/c4f60 was made from; board calibration only). Written into a lens-only folder REC/lensonly_f050
  (gt.npz: K, dist, t_ns = frames_cam5.csv t_mid, R_bc/p_bc = the VIO poses so the scorer alignment is identity; no board scoring).
  VIO config = build_livepose/cfg_gen/c4f60 (same as 165506).
Measurements (scale s = measured / reference; scale error = s - 1; >0 = scanner too big):
- (a) red: HSV red mask per BA keyframe -> 4 edge lines with sub-pixel fits on the MIDDLE part of each edge only (corners are rounded):
  exclude 15 % of edge length at each end (sensitivity 10 % / 20 %). Each 3D edge line triangulated from many keyframes (plane through
  camera centre and the image line, least squares over views) on a common fitted plane; W/H = distance between opposite 3D lines.
  Uncertainty = bootstrap over keyframes (1000x) + spread of the exclusion variants; edge-fit residuals reported.
- (b) red on the mesh: plane fit + red-coloured vertices -> rectangle fit (only if the mesh colours separate it).
- (c) card: same as (a) if the card is visible in enough keyframes (>= 10).
- (d) ChArUco board: only if one is visible (not expected).
Agreement rule (fixed now): red vs card scales agree if |s_red - s_card| <= 1 sigma_combined (sqrt of the summed variances);
  "compatible" if <= 2 sigma. Compared also with the BA k history: -1.4 ... +2 % vs board GT, +2 % on 165506.
Numbers are 미검증 until an independent check.

## Run (2026-11-11 20:41-20:45, laptop worker, job 20261111_204156_..._cmd, run_job.sh)
- run_scanner.sh had no GT to pick from (default would be imu_gt_spl -> missing -> fails). Used: `--gt lensonly_f050 --cfg cfg_gen/c4f60`
  (0.5 dpt lens std_1920x1080_f0.50_calcap20261026, fx 1286.25; 165506 used its own GT lens fx 1286.18, same focus). lp_replay.py patched:
  no imu_gt/align.json is fine in --time live (dt only reported).
- VIO c4f60: tracked, init 6.2 s, path 12.2 m, loop-closure error 15.7 cm. 324 motion keyframes -> kf_ba GPU BA -> FFS all/2 -> TSDF dense.
  BA vs VIO Sim3 scale 0.9985 (BA does not change the scale). Results in runs/laptop/20261111_202906_std60/ (not committed: data).
- Not in view: no ID-1 card (the coloured "direm" object on the printer is a box), no ChArUco (ArUco 4x4/5x5/6x6 every 0.5 s: 0 hits).
  -> (c) card and (d) board scale NOT possible on this recording.

## Results (미검증)
(a) red rectangle, 259 keyframes with >= 3 clean edges (top 256, others 259), camera 0.48-1.26 m (median 0.61 m), out_red/red_measure.json.
Per-view edge line fit rms 0.21 px median (0.28 p90). Edge middle only, 15 % excluded at each end.

| estimator | W mm | H mm | diag mm | W/H | s_W | s_H | s_mean |
|---|---|---|---|---|---|---|---|
| reference (EDID pitch, 가정) | 302.00 | 188.75 | 356.1 | 1.600 | | | |
| A free 3D lines (excl 15 %, thr 0.5) | 317.0 | 198.5 | 374.0 | 1.597 | 1.0497 | 1.0518 | 1.0507 |
| **B rectangle bundle (excl 15 %, thr 0.5)** | **316.2** | **198.3** | **373.3** | 1.595 | 1.0471 | 1.0507 | **1.0489** |
- sensitivity (B s_mean): excl 10 / 15 / 20 % -> 1.0488 / 1.0489 / 1.0490; edge threshold 0.3 / 0.5 / 0.7 -> 1.0518 / 1.0489 / 1.0464.
- bootstrap over keyframes (300x): s_mean sd 0.0003 (95 % 1.0484-1.0496) = statistical only. First / second half of the views: 1.0491 / 1.0492.
- B residuals: median |r| 0.81 px, rms 2.63 px (per-view rms median 1.17, p90 3.7, max 10.2 px) -> pose inconsistency of a few px, Huber loss.
- A: corner angle 89.95 deg, top/bottom W 316.97/317.02, left/right H 198.44/198.60 -> a clean rectangle.
- edge bias check: a constant edge offset d (glow, blur, clipping) adds the same mm to W and H; measured W/H 1.595 vs 1.600 allows
  only d ~ 0.6 mm (0.2-0.3 %). The +4.9 % is a proportional (scale) error, not an edge-position error.
- vs task-text reference 301.6 x 188.5: s_mean 1.0503.
(b) mesh (TSDF 5 mm colours, out_mesh/mesh_measure.json): red/bezel colour bleed; s_mean 1.036 (strict red) / 1.077 (base) / 1.150 (loose)
  -> brackets (a), not usable as a measurement (±5 %). Screen plane rms 0.5-1.2 mm (the red screen was reconstructed as one flat surface).
(c) card / (d) board: not in the recording.

Verdict (pre-registered rule needs the card; not testable): scanner scale from the red screen = **+4.9 % too big** (s = 1.049,
uncertainty est. ±0.4 % = threshold spread ±0.3 % + reference ±0.2 % + lens/rolling-shutter not modelled). Outside the earlier
BA k range vs board GT (−1.4 … +2 %, +2 % on 165506). VIO itself carries it (BA/VIO 0.9985).
Caveats: one recording; reference size 가정 (EDID-derived pitch, same laptop as 165506 assumed; user hand check 3.5 cards ≈ 300 mm agrees
with 302, not 316); OLED PWM banding + rolling shutter (6.5 ms) not modelled; JPEG q90 chroma; R channel clipped at 255 on the screen.

Viewer: C:\Users\a3162\scan3d_handoff\changes_0620\redscreen\scanner_202906_redscreen.ply (meshes.json "202906 · 빨간 화면 크기 시험";
backup meshes.json.bak_redscreen_*). Mesh rigidly moved into the measured red-rectangle frame (export_viewer.py); the drawn "board"
outline is the REFERENCE 302.0 x 188.75 mm -> the red patch sticking out of the outline is the scale error (no rescale applied).
Scripts: run_job.sh (laptop job), make_lensonly_gt.py, measure_red.py (a), measure_mesh.py (b), export_viewer.py.
