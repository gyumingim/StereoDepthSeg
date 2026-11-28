# vc_holdout 2026-11-28 — stop rule, written BEFORE any run

Test: "hide a well-measured region, let ViewCrafter fill it, compare to the real photo" (Fable design,
`scan3d_handoff/ideas/fable_viewcrafter_fusion_20261128.md` §6). Recording 121801 v2, map ffsonly2
(fused FFS-only depth `ffsonly_ba2/fused`, poses/images `ba_partners2/run/dump_ba`). No training.
Written 2026-11-28 before region cutting, rendering, diffusion or metrics were run (only photos looked at to pick the rectangles).

## Regions (fixed now)
| id | type | target keyframe (= GT photo, last clip frame) | rectangle in the 960x540 photo (x0,x1,y0,y1) |
|---|---|---|---|
| R1 | structure: wall + vertical room corner (x~450) + map paper | kf 538 (idx 36) | 230, 700, 70, 225 |
| R2 | object: grey locker front (stickers) | kf 774 (idx 72) | 550, 750, 120, 500 |

Region 3D set S = rectangle pixels back-projected with the target's fused depth, kept within ±0.25 m of the rectangle's median depth.
- Keyframe removal: every keyframe whose view contains >= 0.2 % of S (in frustum, not occluded: z <= Z_kf + 0.10 m, or Z_kf invalid)
  is removed from the condition (no image, no points). Its photo is ground truth only.
- Cloud cut: points within 5 cm of S are deleted; also points that project into the target's region mask with depth >= region depth - 5 cm.
- Start keyframes: the 2 non-removed keyframes nearest to the target by d = |dC| [m] + 0.5 * angle [rad], index gap >= 5 between the two.
- Path: 25 frames, slerp rotation + linear centre, start pose -> target pose. Seeds 123 and 7. 2 starts x 2 seeds x 2 regions = 8 clips.
- Condition: our fused points (remaining keyframes) rendered with ViewCrafter's own point renderer at 320x512; frame 0 = start photo.

## Arms (last frame, 512x320 = 864x540 centre crop resized)
- A0n: hole pixels filled with the colour of the nearest covered pixel of the condition render ("nearest colour").
- A0m: hole pixels filled with the median colour of covered pixels in a 12 px ring around the hole ("plane + surrounding wall colour").
- **A0 for the gate = whichever of A0n / A0m has the lower mean hole-LPIPS over that region's 4 clips** (the stronger baseline; conservative for B).
- B: ViewCrafter last frame, per-channel gain/offset fitted (least squares) on covered pixels outside the 8 px-dilated hole, then pasted
  into uncovered pixels only. (B_raw = no colour fit, reference only.)
- Outside the scored pixels all arms keep the condition render.

## Metrics (scored pixels H = region mask of the target view AND uncovered in the condition render)
1. LPIPS (alex, spatial map) vs GT photo, averaged over H; image pixels outside H are set to GT in both images before LPIPS. PSNR on H: reference only.
2. Consistency: seed-to-seed (same start) and start-to-start (same seed) LPIPS of B, same masking.
3. Eye table per clip: "invented object" (object in B's hole not in GT) / "missing object" (object in GT not in B).
4. Covered-pixel change: raw ViewCrafter last frame vs condition render on covered pixels (mean |diff| 0-255, PSNR).
5. Positive control (ruler check): W = A0 with inverted colours (255 - x) on H. Ruler valid only if LPIPS(W) - LPIPS(A0) >= 0.05 in all 8 clips.
   Also GT itself on H must give LPIPS 0.
Free-space check: not used in this test.

## Stop rule (Fable, unchanged)
Per region type: **GO only if B beats A0 by >= 0.05 LPIPS (LPIPS_A0 - LPIPS_B >= 0.05) in >= 3 of 4 clips AND invented-object clips <= 1 of 4.**
Else NO-GO. R1 NO-GO -> structure = A (plane UV) only. R2 NO-GO -> behind-object holes stay empty, marked "not seen".
Both NO-GO -> drop the video-model path. If the positive control fails, the result is "ruler blind = undecided", not GO.
No loosened retry, no re-pick of regions/starts/seeds after seeing results. n is small (1 recording, 2 regions, 4 clips each).
