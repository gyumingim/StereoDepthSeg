# ovopt 2026-10-25 — OpenVINS + ov_bridge speed options (branch `ovopt-20261025`)

Goal (user): faster per-frame processing on the S24, same or better accuracy, less heat; everything roll-back-able.
PC work first; phone camera-free replay after the user's approval (13:00, "최적화 옵션 폰에서 녹화 재생으로 비교 ㄱ").
Numbers marked 추정 are estimates; everything else is measured (files below).

## Conclusion first
- **Exact options (`--ovopt all_exact`) do not change a single pose**: identical at 17 significant digits on the PC
  (3 inputs) and **on the phone** (raw80 c4, 2327/2327); with no options this branch = the 0.6.17 APK binary on the
  phone (2327/2327) and = the established fps30 C4 bench results on all 45 GT cells.
- PC (x86, raw80, 3 reps): c4 OpenVINS feed p50/p95 **10.61/17.05 -> 7.50/11.27 ms** (all_exact), **5.64/8.35 ms**
  with the near-exact EKF options added; c4n400s100c20 26.39/38.63 -> 19.70/27.31 -> 13.59/18.97 ms.
- Near-exact options change the trajectories only at rounding level: on 42 GT cells ATE median 0.5914 cm both, same 8
  divergent cells, paired difference 0.000 cm.
- Phone timing / heat: see "Phone" below (filled from out_L2_timing / out_L4_heat).

## Options (all default off = 0.6.17 behaviour; ov_bridge `--ovopt LIST`, app intent `ov_opt`)
| option | where | what | class |
|---|---|---|---|
| skip_retri | ovpatch/VioManager.cpp | skip `retriangulate_active_tracks` (ROS-viz output only: active image drawing + 3-D of every track) | exact |
| par_tri | ovpatch/UpdaterMSCKF.cpp, UpdaterSLAM.cpp | per-feature triangulation + Gauss-Newton on the OpenCV pool, removal in original order | exact |
| par_jac | ovpatch/UpdaterMSCKF.cpp, UpdaterSLAM.cpp | per-feature Jacobian / nullspace / chi2 on the pool, serial stacking in original order | exact |
| par_ma | ovpatch/StateHelper.cpp | EKFUpdate M = P H^T, one state block per task (same Eigen calls per block) | exact |
| par_s | ovpatch/StateHelper.cpp | P_small + S = H P H^T + R as one more task beside M | exact |
| cov_move | ovpatch/StateHelper.cpp | marginalize(): move the new covariance (no N^2 copy) | exact |
| givens_rm | ovpatch/UpdaterMSCKF.cpp | MSCKF measurement compression: same Givens calls on a row-major copy | exact |
| frame_nocopy | main.cpp | remap straight from the received record (no 2 MB memcpy) | exact |
| all_exact | | all of the above | exact |
| ekf_gemm | StateHelper.cpp | M = P(:,Hvars) H^T as one GEMM | near-exact |
| qr_compress | UpdaterMSCKF.cpp | measurement compression with Householder QR (replaces givens_rm) | near-exact |
| ekf_chol | StateHelper.cpp | W = M U^-1 (S = U^T U), P -= W W^T, dx = W U^-T r (no S^-1, no K) | near-exact |
| par_cov | StateHelper.cpp | covariance update split in 2 equal-work parts on the pool | near-exact |
Other flags (default off): `--prof-log FILE` (per-frame stage timers), `--pose-precision N` (17 for bit checks),
`--drop-late MS` (skip frames that would be fed > MS later than the best lag: no backlog spiral; changes results when
it drops), `--idle-spin 3` (pool waits with WFE between input polls), `--idle-spin-ms W` (spin only W ms before the
expected frame). App (arlat 0.6.18, versionCode 24) intents: ov_opt, ov_prof, drop_late_ms, idle_spin_ms, spin_wfe.

## PC identity (`pc_ident/`)
- C4 harness (hs_20261024_165506_std60, 845 frames, cfg_identity/): main checkout build_par (= 0.6.17 bridge) = this
  branch default build = full-source build with no options = each exact option alone = all_exact, 845/845, hash
  d7958f8247fbc180 (= 0.6.16 P0) - `ident_pc_5.log` (final binary 71bb15d9).
- 17 digits (`ident17_2.log`): every exact option alone + all_exact identical on the harness (845), raw80 c4 (2327,
  with remap: frame_nocopy active) and raw80 c4n400s100c20 all_exact (2327).
- Near-exact ekf_gemm / qr_compress / ekf_chol / par_cov differ from frame 145-163 on (expected).
- Idle-spin variants (paced feed, `spin_pc_1.log`): --idle-spin 2/3, -ms 15/10 all pose-identical; window 15 ms cut the
  pool spin time 23.0 -> 12.0 s over the 28 s stream (x86: mode 3 = mode 2, the WFE path is ARM only).

## PC timing (`pc_prof/`, raw80 = phone replay stream, phone configs, --threads 2, unpaced, 3 interleaved repeats)
OpenVINS feed per tracking frame, ms p50 / p95; CPU = user+sys of the whole run (80 s stream); final binary (run 4):
| config | A none | B all_exact | W = B + ekf_gemm + qr_compress + ekf_chol + par_cov |
|---|---|---|---|
| c4 | 10.61 / 17.05, CPU 32.3 s | 7.50 / 11.27 (-29 % / -34 %), CPU 29.5 s | 5.64 / 8.35 (-47 % / -51 %), CPU 25.0 s |
| c4n400s100c20 | 26.39 / 38.63, CPU 64.9 s | 19.70 / 27.31 (-25 % / -29 %), CPU 62.2 s | 13.59 / 18.97 (-49 % / -51 %), CPU 48.6 s |
Single near-exact options on top of all_exact (run 3, before par_s, c4 p50/p95): +ekf_gemm 6.94/10.44, +ekf_chol
7.02/10.54, +par_cov 7.17/10.81. Stage p50 c4 A -> B: retri 1.30 -> 0, M = PH^T 2.45 -> 1.28, slam EKF 2.91 -> 2.07,
msckf compression p95 3.36 -> 2.13. KLT tracking (2.5 ms, OpenCV) is unchanged. x86 build has no AVX/FMA (same flags
as libov_msckf_lib.so), so absolute ms do not transfer to the phone.

## Offline accuracy (`pc_ate/`, ate_run.py = fps30 C4 protocol, 42 HS GT cells + 213955, camera-position ruler)
- B0 (this branch, no options) = runs/fps30 C4 poses in 45/45 cells (bit-identical at 9 digits).
- W (all options) vs B0: same 8 divergent cells, ate_w median 0.5914 / 0.5914 cm, paired d_med 0.000 cm (34 cells).
- Phase 4 configs: see score files (filled when the schedule finishes).

## Backlog "cliff" guard (`pc_drop/`): `--drop-late 50`, PC 1-core overload (c4n400s100c20, real-time feed)
lag p95 7.8 s -> 86 ms (max 7.9 s -> 0.14 s), 11 % of frames skipped, ATE 0.32 -> 0.30 cm (A) / 0.32 -> 0.26 cm
(all_exact); 1 rep each. The phone replay study found the cliff (frames > 33 ms -> spin never runs -> slower).

## Phone (camera-free replay, raw80 + phone remap + phone configs with timeshift 0 like the replay study)
Runner `phone/phone_run.sh` (own dir /data/local/tmp/ovopt, own process names, reads the replay study's raw80.bin /
feeder3 / remap read-only), lists `phone/L*.txt`, analysis `phone/phone_an.py`, `phone_sum.py`, `heat_an.py`.
Screen was on (Settings app in front) during the runs; battery discharging (no charger).
- L1 identity (`out_L1_ident/`): I_none17 vs I_exact17 2327/2327 identical at 17 digits (hash 4df353f02098f19a);
  I_ref (0.6.17 APK binary) vs I_none 2327/2327 identical (aa71a3815e7d3ae4); near-exact W differs from frame 324.
- L1 showed a strong thermal confound (big-core cap 2784 -> 2016 MHz within 4 runs, SKIN start < 36 only), so L2 gates
  SKIN < 35 and AP < 37 C + 30 s cool-down before every run.
- L2 timing (`out_L2_timing/`, `phone_sum_L1L2.md`), proc = remap + feed per tracking frame, ms p50 / p95, per run:
  | config | A no options | B all_exact | W = B + ekf_gemm + qr_compress + ekf_chol + par_cov |
  |---|---|---|---|
  | c4 | 21.3 / 40.0 (14.7 % > 33 ms, backlog max 1.6 s); cool first run 14.8 / 21.2; hot run 25.5 / 43.4 | 12.3 / 19.2, 13.0 / 20.4 (+ L1 12.5 / 19.5), 0 % > 33 | 11.6 / 17.5, 11.7 / 17.9 (+ L1 11.9 / 17.2), 0 % |
  | c4n400s100c20 | 51.2 / 78.8, 51.1 / 77.5 (92 % > 33, 40 s backlog) | 31.3 / 52.2, 31.0 / 51.1 (42 %) | 25.0 / 43.3, 23.3 / 42.2 (20 %) |
  0.6.17 APK binary (L1, hot): 20.9 / 38.3. Every L2 run: A = B poses identical (c4 aa71a3815e7d3ae4, n400 8e2e804d687cf0e8).
- Phone stages p50 (c4, gated runs, `prof.csv`): retri (skipped by skip_retri) **3.9 ms**; KLT track 7.1 (A, partly
  backlog) vs 5.2-5.4 (B/W); M = P H^T 2.2 -> 1.8 (par_ma, weak on the phone) -> 0.8 (ekf_gemm); msckf compression p95
  4.2 -> 3.4 (givens_rm) -> 2.0 (qr); K / S^-1 0.49 -> 0.29 (ekf_chol); covariance update 0.51 -> 0.6 (par_cov: no gain,
  at n400 2.55 -> 3.63 = worse -> W3 without par_cov tested in L6).
- Heat proxies of L2 (all idle_spin 2): SKIN max 36.4-36.8, AP mean 41.5-43.2 C for every variant -> with spin 2 the
  options do not change heat (the pool spins when it does not compute); the heat lever is the spin mode (L4).

## Files
- scripts: `ident_pc.sh`, `ident17.sh` (identity), `prof_pc.sh` + `prof_an.py` + `cpu_an.py` (timing / stages / CPU),
  `spin_pc.sh` (idle-spin variants), `droplate_pc.sh` + `feeder3.cpp` + `ate_raw80.py` (backlog guard),
  `ate_run.py` (offline GT accuracy), `cmp.py` (pose comparison), `phone/` (phone runner, lists, analysis).
- raw PC outputs: `raw/*.tgz` (not committed, sha256 in raw/MANIFEST.sha256) + /tmp/ovopt on rtx5080 WSL.
