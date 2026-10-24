# std60 board check, 2026-10-24

Recording `recordings/hs_20261024_165506_std60` (not committed): std 60 fps session 1920x1080, focus 0.5 dpt, exposure 2 ms,
auto ISO, handheld 30 s in front of the laptop-screen ChArUco 5x3 (`config/charuco/board_laptop_5x3.json`), still 5 s →
move 20 s → still 5 s, ~0.85 m (PnP median). Lens = std60 0.5 dpt default
`config/lens/std_1920x1080_f0.50_cal20261024d.json`. Rolling-shutter skew in this session 6.54 ms (HS 240: 2.94 ms).

## Conclusion
- **GT is good**: 1625/1690 frame poses (96 %), per-frame reprojection median 0.11 px, spline GT corner median 0.098 px
  (p90 0.195), 1642 frames ok. Scale check s_fixedlever **0.992** [0.957, 1.022] (spline GT; RS GT 0.975) → OK (|s−1| ≤ 0.15).
  Absolute scale still carries the ±2 % square_mm ESTIMATE (60.3 mm, spec-based) — the IMU check agrees within its CI.
- **Alignment differs from the values assumed for the no-board std60 loop tests** (`config/phone_ov_main/variants.json`):
  R_ci **1.08°** away from the HS chordal mean (HS recordings: 0.10–0.32° from that mean); dt **+4.67 ms** vs assumed
  −2.17 ms → **+6.8 ms** (2000-iteration refit: +3.78 ms → +5.9 ms; the dt loss is flat, dt is weakly determined).
  The difference is close to the std60 RS skew (6.54 ms) — a different timestamp reference in the std session is a
  추정 (guess), not checked. One recording only; the HS dt values themselves range −7.4 … +1.3 ms.
- **VIO (final baseline) runs on std60 with this GT, no divergence**: window ATE 2.2–2.4 cm, scale 0.94–0.95.
  The audit init (window 1.5 s / disparity 2.30) is better here: ATE 1.1–1.3 cm, scale 1.00–1.01. Baseline init did
  not fail (initialised at the motion start in all offsets), so the audit init was run as a comparison, not a fallback.

## VIO numbers (camera-position ruler on `imu_gt_spl`)
Runner `imu_learn/std60_board_run.py` = final_baseline_run.py settings (config/mono incl. F1 fi_max 1e6/200,
try_zupt=false, lever, bench default lens → std 0.5 dpt file, strict init, clones 11, frame cache, 2 OpenCV threads,
dt / R_ci from this recording's `imu_gt/align.json`). One run per cell.

| cell | div | window ATE cm | Sim3 ATE cm | scale | tracked frac | init at s | n_slam motion median (frac 0) |
|---|---|---|---|---|---|---|---|
| o0 base | no | 2.44 | 2.29 | 0.938 | 0.825 | 4.93 | 50 (0.026) |
| o1 base | no | 2.16 | 2.06 | 0.952 | 0.855 | 3.93 | 50 (0.026) |
| o2 base | no | 2.25 | 2.12 | 0.944 | 0.888 | 2.93 | 50 (0.026) |
| o0 aud (1.5 / 2.30) | no | 1.29 | 1.29 | 0.998 | 0.832 | 4.73 | 50 (0.010) |
| o1 aud | no | 1.33 | 1.33 | 1.009 | 0.863 | 3.73 | 50 (0.013) |
| o2 aud | no | 1.13 | 1.11 | 1.013 | 0.896 | 2.73 | 50 (0.013) |

For reference only (different rate, distance, board): HS 240→60 fps final baseline + F1 median window ATE 0.58 cm
(docs/fi_limits_result.md). The ~6 % scale shortfall of base is beyond the ±2 % board-size uncertainty.

## Caveats
- One recording, one run per cell; base vs aud differ only in the init window / disparity (no other change).
- VIO used this recording's measured dt (+4.67 ms) and R_ci, NOT the assumed values used in the loop tests; the effect of
  the assumed values on this recording was not run (open item: rerun o0–o2 with dt −2.17 ms / mean R_ci).
- Per-frame attitude residual of the sync fit: median 0.27°, p90 1.34° (HS laptop-board recordings 0.23–0.32° / 0.4–0.8°);
  the high p90 is likely the 8-corner board during fast motion (추정).

## Files
- GT chain: `runs/std60_board_20261024/gt.sh` (imu_gt → board_shift → batch_sync → imu_gt --rs → spline_gt → scale_ls), logs
  and `scale_spl.json`, `batch_sync.json`, `batch_sync_it2000.json` (refit, not written to align.json).
- Align comparison: `runs/std60_board_20261024/compare_align.py` → `compare_align.json`.
- VIO: `imu_learn/std60_board_run.py run|score` → `runs/std60_board_20261024/o{0,1,2}_{base,aud}.json`, `score.json`.
