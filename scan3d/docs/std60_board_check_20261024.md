# std60 board check, 2026-10-24

std 60 fps session (1920x1080, focus 0.5 dpt, exposure 2 ms, auto ISO), handheld ~30 s in front of the laptop-screen ChArUco
5x3 (`config/charuco/board_laptop_5x3.json`, **square 60.40 mm from panel EDID**, commit 7cafe2d), still 5 s → move 20 s →
still 5 s, ~0.85 m. Lens = std60 0.5 dpt default `config/lens/std_1920x1080_f0.50_cal20261024d.json`. RS skew 6.54 ms.
Recordings (not committed): `hs_20261024_165506_std60`, `hs_20261024_173432_std60`. A third name in the coordinator's list,
`hs_20261024_172843_std60`, was not in `recordings/` while polling — not processed.

## Conclusion
- **GT good on both**: spline corner median 0.098 / 0.087 px, 1642 / 1565 frames, PnP reprojection 0.12 / 0.09 px.
- **Scale check (s_fixedlever, spline GT)**: 0.990 [0.955, 1.020] and **0.929 [0.855, 0.944]** → both "OK" (|s−1| ≤ 0.15),
  but 173432 is 7 % low. Its IMU excitation is weak (dynamic-accel rms median 0.07 vs 0.83 m/s², more still windows), while
  VIO with the audit init gives scale 1.00–1.05 against this GT → the low s is more likely the IMU check than the GT (추정).
- **Align vs the assumed std60 values** (`config/phone_ov_main/variants.json`: HS chordal-mean R_ci, dt −2.17 ms):

  | rec | dt ms | dt − assumed ms | R_ci vs HS mean ° | sync frame residual med / p90 ° |
  |---|---|---|---|---|
  | 165506 | +4.67 | +6.84 | 1.08 | 0.27 / 1.34 |
  | 173432 | −1.62 | +0.55 | 1.42 | 0.10 / 0.92 |

  std60 chordal mean vs HS mean **0.96°**; the two std60 R_ci are each 0.82° from their mean (1.6° apart) — the HS
  recordings are 0.10–0.32° from theirs. dt disagrees by 6.3 ms between the two (HS range −7.4 … +1.3 ms).
- **VIO effect of the align choice is small and mixed** (one run per cell): measured vs assumed align, median window ATE
  base 2.25 vs 1.90 cm (only 165506 initialises), aud 1.42 vs 1.61 cm.
- **Init**: the baseline init (window 1.0 s / disparity 15) **did not initialise at all on 173432** (o0/o1/o2, also with the
  assumed align). Audit init (1.5 s / 2.30) initialised in 6/6 cells: median ATE 1.42 cm, |scale−1| 1.3 %, vs baseline
  2.25 cm, 5.4 % (3/6 cells, 165506 only).

## VIO numbers (camera-position ruler on `imu_gt_spl`, 60.40 mm GT)
Runner `imu_learn/std60_board_run.py` = final_baseline_run.py settings (config/mono incl. F1 fi_max 1e6/200, try_zupt=false,
lever, std 0.5 dpt lens, strict init, clones 11, frame cache, 2 OpenCV threads). base/aud use the recording's
`imu_gt/align.json`; baseA/audA use the assumed align (scores from the run-time evaluation with that dt).
Window ATE cm / scale; "—" = no initialisation (tracked fraction 0).

| rec | off | base | aud (1.5/2.30) | baseA | audA |
|---|---|---|---|---|---|
| 165506 | o0 | 2.44 / 0.940 | 1.29 / 1.000 | 1.76 / 0.971 | 1.33 / 0.994 |
| 165506 | o1 | 2.15 / 0.954 | 1.34 / 1.011 | 1.90 / 0.960 | 1.33 / 1.002 |
| 165506 | o2 | 2.25 / 0.946 | 1.13 / 1.015 | 2.07 / 0.952 | 1.13 / 1.012 |
| 173432 | o0 | — | 1.50 / 1.021 | — | 2.03 / 1.015 |
| 173432 | o1 | — | 1.50 / 1.004 | — | 1.88 / 1.033 |
| 173432 | o2 | — | 4.10 / 1.052 | — | 3.89 / 1.054 |

Sim3 ATE is within 0.15 cm of window ATE in every cell. Tracked fraction 0.83–0.91, n_slam during motion median 49–50
(zero in 1–4 % of motion frames) wherever it initialised. No divergence. For reference only (different rate, distance,
board): HS final baseline + F1 median window ATE 0.58 cm (docs/fi_limits_result.md).
v1 of this check (165506 only, square 60.3 mm) gave the same align and ATE within 0.01 cm; its files stay in
`runs/std60_board_20261024/` (top level) and its GT dirs were renamed `imu_gt*_sq603` in the recording.

## Recommendation (not applied)
- **std60 align for no-board runs**: keep dt **−2.17 ms** (the better-fitting recording 173432 gives −1.62 ms; 165506's +4.67 ms
  comes with a 2.7x larger sync residual; a median of two values, +1.5 ms, means little). For R_ci, the std60 chordal mean
  (`summary.json` → `std60_R_ci_chordal_mean`, 0.96° from the HS mean) is the better guess (추정) — both std60 recordings sit
  1.1–1.4° from the HS mean — but a VIO gain is not shown (mixed, one run per cell). Record 2–3 more std60 board
  recordings (bigger board / more corners) before switching.
- **Init 1.5 / 2.30**: recommended for std60 (6/6 initialised vs 3/6, ATE about −0.8 cm, scale error 1.3 % vs 5.4 %, two
  recordings). It changes the baseline: run the 15 HS recordings x o0/o1/o2 with it (same scorer) before adopting.

## Caveats
- Two recordings, one run per cell; base vs aud differ only in init window / disparity; A vs non-A only in dt / R_ci.
- Board square 60.40 mm is the panel EDID size at 1:1 px (user choice); a ruler gave +0.17 %.

## Files
- GT chain `runs/std60_board_20261024/gt.sh <rec>` (imu_gt → board_shift → batch_sync → imu_gt --rs → spline_gt →
  scale_ls), poller `poll.sh`, logs per recording in `runs/std60_board_20261024/<HHMMSS>/`.
- VIO `imu_learn/std60_board_run.py` (STD60_REC, STD60_CONDS), assumed-align runs `run_assumed.sh`.
- Summary `summary.py` → `summary.json`; motion stats `motion_stats.py`; v1 align comparison `compare_align.py/.json`.
