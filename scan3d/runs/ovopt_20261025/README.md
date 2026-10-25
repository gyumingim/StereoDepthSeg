# ovopt 2026-10-25 — OpenVINS + ov_bridge speed options (branch `ovopt-20261025`)

Goal (user): faster per-frame processing on the S24, same or better accuracy, less heat; everything roll-back-able.
**State: PC only.** The phone was not used (coordinator: no adb until the user approves phone work). Every phone number
below is from the other agent's 0.6.17 replay runs or an estimate ("추정"); the phone test plan is at the end.

## Options (all default off = 0.6.17 behaviour; ov_bridge `--ovopt LIST`, app intent `ov_opt`)
| option | where | what | class |
|---|---|---|---|
| skip_retri | ovpatch/VioManager.cpp | skip `retriangulate_active_tracks` (ROS-viz output only: active image drawing + 3-D of every track) | exact |
| par_tri | ovpatch/UpdaterMSCKF.cpp, UpdaterSLAM.cpp | per-feature triangulation + Gauss-Newton on the OpenCV pool, removal in original order | exact |
| par_jac | ovpatch/UpdaterMSCKF.cpp, UpdaterSLAM.cpp | per-feature Jacobian / nullspace / chi2 on the pool, serial stacking in original order | exact |
| par_ma | ovpatch/StateHelper.cpp | EKFUpdate M = P H^T, one state block per task (same Eigen calls per block) | exact |
| cov_move | ovpatch/StateHelper.cpp | marginalize(): move the new covariance (no N^2 copy) | exact |
| givens_rm | ovpatch/UpdaterMSCKF.cpp | MSCKF measurement compression: same Givens calls on a row-major copy (Eigen scalar rotation loop either way) | exact |
| frame_nocopy | main.cpp | remap straight from the received record (no 2 MB memcpy) | exact |
| all_exact | | all of the above | exact |
| ekf_gemm | StateHelper.cpp | M = P(:,Hvars) H^T as one GEMM | near-exact |
| qr_compress | UpdaterMSCKF.cpp | measurement compression with Householder QR (replaces givens_rm) | near-exact |
| ekf_chol | StateHelper.cpp | W = M U^-1 (S = U^T U), P -= W W^T, dx = W U^-T r (no S^-1, no K) | near-exact |
| par_cov | StateHelper.cpp | covariance update split in 2 equal-work parts on the pool | near-exact |
| prof | `--prof-log FILE` | per-frame stage timers (timing only) | - |
Heat (Phase 3, timing only, phone measurement pending): `--idle-spin 3` (pool waits with WFE between input polls),
`--idle-spin-ms W` (spin only W ms before the expected frame). App intents `spin_wfe 1`, `idle_spin_ms W`.

## PC identity (pose lines: every field except ms / remap_ms / wait_ms, cmp.py)
- `pc_ident/ident_pc_*.log`: captured C4 stream of hs_20261024_165506_std60 (845 frames, the 0.6.16/0.6.17 harness),
  config cfg_identity/. Main checkout build_par (0.6.17 bridge) = this branch default PC build = full-source build with
  no options = every exact option alone = all_exact: **845/845 identical, hash d7958f8247fbc180** (also = 0.6.16 P0).
  ekf_gemm / qr_compress differ from frame 145-146 on (near-exact, expected).
- `pc_prof/prof_pc_*.log`: phone replay stream raw80 (hs_20261021_213341, 80 s, 2327 raw 1920x1080 frames + phone
  remap, phone configs c4 and c4n400s100c20, --threads 2): A (no options) = B (all_exact) on every repeat:
  **c4 2327/2327 hash 64f4e472d4518fe8, c4n400s100c20 2327/2327 hash 4f1d2a88ed5e4a83**.
- Phone identity is NOT checked yet (ARM FMA contraction could in principle differ; the exact options do not change
  any arithmetic expression or Eigen kernel size, so identical results are expected — plan step P1).

## PC timing (raw80, unpaced, x86 Zen4 without AVX/FMA = the OpenVINS build flags; 3 interleaved repeats)
feed_ms = OpenVINS per frame (tracking frames), ms p50 / p95 (`pc_prof/prof_pc_2_summary.md`, `prof_pc_1_summary.md`):
| config | A none | B all_exact | C all_exact + ekf_gemm + qr_compress (run 1, without givens_rm) |
|---|---|---|---|
| c4 | 10.66 / 16.63 | 7.45 / 11.09 (-30 % / -33 %) | 6.98 / 9.72 (-31 % / -39 %, vs run-1 A 10.12 / 16.00) |
| c4n400s100c20 | 26.26 / 38.58 | 19.66 / 27.55 (-25 % / -29 %) | 17.91 / 24.28 (-32 % / -37 %, vs run-1 A 26.27 / 38.42) |
Stage p50 (c4, A -> B): retri 1.30 -> 0, ekf M=PH^T 2.45 -> 1.28, slam_ekf 2.91 -> 2.07, msckf_comp p95 3.36 -> 2.13,
msckf_jac p95 1.99 -> 1.07, remap 0.98 -> 0.85, track 2.5 (unchanged, OpenCV KLT).
The PC/phone stage ratio differs (phone 0.6.17 replay R1_c4: track 5.2, re-tri & marg 3.0, slam 2.2, msckf 1.6 ms);
phone gain is NOT measured. 추정: C4 phone p50 -3..-5 ms (re-tri & marg ~3 ms on the phone is mostly the skipped
visualization image work).

## Files
- scripts: `ident_pc.sh` (identity), `prof_pc.sh` + `prof_an.py` (timing / stages), `spin_pc.sh` (idle-spin variants),
  `ate_run.py` (offline GT accuracy, fps30 C4 protocol), `cmp.py` (pose comparison).
- `cfg_identity/` = runs/vio_v3w4kbl8 (config of the 0.6.16/0.6.17 identity harness).
- raw outputs: /tmp/ovopt/{ident,prof,prof2,spin,ate} on rtx5080 WSL (pose jsonl, prof csv, timing csv); summaries here.
