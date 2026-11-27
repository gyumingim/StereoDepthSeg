# Item 3 plane-tie: GO gate (written 2026-11-27 ~23:12 real clock, BEFORE any training; shrunk per the main-session correction)

## Cheap check first (no training): plane_diag stages (ffsonly2, diag_plane_20261127/plane_diag.json)
combined RMS vs the global plane: cabinet c 3.18 -> d 2.08 / e 2.24; floor c 0.75 -> d 0.60 / e 0.44; wall c 0.85 -> d 2.06 / e 1.83 cm.
Per-view own-plane RMS on FFS pixels: b (fused FFS) vs f (2DGS depth on the same pixels): cabinet 0.75 -> 0.36, wall 0.29 -> 1.25, floor 0.13 -> 0.53 cm.
Reading: training does not add bumps overall (cabinet / floor better after training); only the wall view-level residual rises
(main session: mostly the wall edge band). So the expected gain of a training-side planar tie is small (추정). One cheap decisive test only.

## Test: three 7k-step arms, everything else = ffsonly2 args (same dump ba_partners2, same fused ffsonly_ba2/fused, surfel, ...)
- OFF s0: gsm_train_ptie.py (copy of gsm_train.py e07c76d9, ptie code inactive) --steps 7000 --hold 8 --seed 0
- ON  s0: same + --ptie labels --ptie-w 0.2 --ptie-start 3000   (own-plane flatness of the RENDERED depth inside FFS-supported planar
          segments, plane fitted to the render each step and detached -> no FFS offset is imposed)
- OFF s1: as OFF s0 with --seed 1 = noise floor of the 7k ruler (n = 1 pair of seeds).
Labels: ptie/labels (Felzenszwalb + W-fill plane gates + p95 guard), 10.9 % of valid pixels, 4.7 segments per keyframe (mean).

## Rulers (held-out keyframes = index % 8 == 0, 62 views, never trained in any arm)
- R1 held-out FFS residual on planar-label pixels: |rendered median depth - fused FFS (F == 1)|, RMS and median in cm (10 cm gate).
- R2 non-planar P3-like: pixels NOT in any planar label, rendered depth vs eval2 gold FFS (old-run FFS, in nobody's training set),
  share within 3 % (and front / behind shares).
- R3 held-out PSNR (train summary.json test_psnr, valid pixels).
- R4 (diagnostic only, circular): own-plane RMS of rendered points per labelled segment on held-out views.

## GO for "worth a 30k run on the NEW recording" (nothing is adopted for 121801 tonight)
ON vs OFF s0: R1 RMS better by >= 0.3 cm AND by more than |OFF s1 - OFF s0| (noise), R2 drop <= 0.5 pt, R3 drop <= 0.1 dB.
Else NO-GO. Never claim improvement from R4 alone. n = 1 recording, 7k steps (not the 30k map), noise from one seed pair only.
Mixed variables: none between ON and OFF s0 except the ptie loss (same code file, same seed; CUDA nondeterminism remains -> OFF s1).
- 23:12 CLOSED by Fable gate check: NO-GO by diagnosis, not tested (see progress Fable 점검).
