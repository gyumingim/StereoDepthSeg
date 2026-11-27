# W-fill GO gate (written 2026-11-27 ~22:53 real clock, BEFORE any pre-test run)

Method (no AI depth): per keyframe, Felzenszwalb segments of the undistorted photo (ba_partners2 dump img). For each segment, fit a plane
(RANSAC + Huber, camera frame) to its FFS pixels (fused F == 1 = ffsonly2 training depth). Fill F == 0 pixels of the segment with the plane
depth only if: support >= 300 px and >= 5 % of the segment, inlier share >= 90 %, inlier RMS <= max(0.6 cm, 0.4 % of median Z),
support pixel spread (minor PCA sd) >= 10 px, ray incidence <= 75 deg, fill only inside the support's convex hull dilated 15 px
(interpolation, not extrapolation), filled Z in (0.05, 4 m) and within [0.85 x min, 1.15 x max] of the support depth.
Filled pixels get F = 2 (training weight --fill-w 0.1, the existing gsm_train path). FFS pixels are never changed.

## Pre-test (hide-and-check, CPU only)
- H1: the same 10 keyframes as the dc probe (idx 50,150,250,400,470 + aisle 314,318,322,326,330), hide 20 % of valid FFS in random
  24x24 blocks (gsm_dc.hide_blocks, seed 0), fill from the rest, score AbsRel on hidden pixels that got filled. Recall = filled / hidden.
- H2: ROI keyframes cabinet_front_kf774, wall_kf538, floor_kf3205: hide 25 % of the valid FFS inside the ROI rect (24 px blocks, seed 0).
- BA: BA2 sparse depth targets (w5/sparse, the same as dc probe) on FFS-EMPTY pixels that the full-FFS fill covers, every 8th keyframe.
  Reference: fused FFS vs the same BA tracks on FFS pixels = 0.73 % median (w5 sparse.json).

## GO if ALL hold (pooled H1 + H2 for G1)
- G1 hidden filled: median AbsRel <= 1.0 %, p90 <= 3.0 %, share > 5 % <= 3 %.
- G2 BA points on filled FFS-empty pixels: n >= 300, median <= 2.0 %, share > 5 % <= 15 %.
- G3 yield: H1 recall >= 30 %, and the full run adds F = 2 pixels >= 3 % of the valid image area (mean over keyframes).
Any fail -> NO-GO. One stricter retry allowed (tighter RMS/inlier gates), re-scored on seed 1, both results reported.

## If GO: same chain as ffsonly2 (run_w5.sh steps without --sparse-depth): train 30k on dump ba_partners2 with --fused wfill/fused,
same args (hold 0), mesh, v3fix, carve, prune, e3c, texture SM2 + o1 (= w9/f2 recipe), pad. Judge vs ffsonly2 with the same rulers:
plane residual on the 3 ROIs (diag_plane script, global plane from the ORIGINAL ffsonly2 fused, stages d/e/f/g),
gsm_w2judge geo + photo (own poses ba2), gsm_w9detail detail gradient.
Adopt only if: plane residual (stage e combined RMS) not worse on any ROI by > 0.2 cm and better on >= 1 ROI by >= 0.3 cm,
P3 not worse by > 0.5 pt, held-out photo PSNR not worse by > 0.1 dB, detail gradient not worse by > 2 %.
Limits known in advance: n = 1 recording; held-out photo uses the existing judge views; hide test scores only where FFS exists
(real holes are more textureless) -> G2 is the check on real holes; BA points are sparse in textureless areas.

## Amendment v2 (~22:55 real clock, after Fable gate check, still BEFORE any run) — replaces G1/G2 where stricter
- H3 (real hole shapes): transplant the F == 0 mask of another keyframe (kf idx + 37 mod n) onto each H1/H2 keyframe, restricted to
  its FFS pixels (= hidden set), fill from the rest. Report error by distance-to-support bins (0-12, 12-40, 40-100, > 100 px).
- G1 now pooled H1+H2+H3: median <= 1.0 %, p90 <= 3.0 %, > 5 % share <= 3 %; AND on the ROI keyframes (H2) hidden-fill median |dZ|
  <= 1.5 x that ROI's stage-b within_rms_own (cabinet 0.75, wall 0.29, floor 0.13 cm -> 1.13 / 0.43 / 0.20 cm), floor at 0.3 cm.
- G2: > 5 % share <= 5 % (was 15 %), report signed error; n >= 300 kept but flagged weak if < 3000.
- G4 (cross-keyframe, real holes, no BA/AI): F == 2 pixels of keyframe A reprojected into keyframes B (>= 15 deg or >= 20 cm apart)
  where B has F == 1 and passes a z-buffer test: median AbsRel <= 1.5 %, > 5 % share <= 5 %, n >= 3000.
- G5 (ROI cheap decisive test, before any training): fill on the ROI footprint keyframes, stage b2 = F == 2 pixels vs the global ROI
  plane. GO needs b2 combined RMS <= ffsonly2 stage e (cabinet 2.24, wall 1.83, floor 0.44 cm) AND fill yield inside ROI holes > 0
  on cabinet or wall (else nothing to gain there).
- Predicted floaters: share of F == 2 pixels > 3 cm in front of the ffsonly2 mesh ray-cast (report; > 5 % -> NO-GO).
- Adoption ruler: plane residual on COMMON pixels hit by both meshes + coverage (hit share of footprint, generated faces excluded)
  + kept_frac_gate. Coverage in holes is the primary win; plane RMS must be "not worse". ffsonly2 re-measured in the same invocation.
- Noise floor: no seed repeat of ffsonly2 exists. Result is "n = 1, noise unknown, not confirmed" unless an ffsonly2 seed repeat is run
  (only if time and the margin is borderline). Mixed variables of a fill retrain: depth loss + init point cloud + densify budget.
- 22:57 note: method params unchanged from GATE.md (scale 300 / min_size 400 / inl 0.9 / min_area 1500). A yield-only look at 4 kf (72,36,200,320; scales 60-300, inl 0.8-0.9) gave fill 0.1-4.7 % of image; main fail reason = segment has too little FFS support. No hide/BA numbers looked at.
