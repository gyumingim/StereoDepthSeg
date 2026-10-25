# ovopt 2026-10-25 — OpenVINS + ov_bridge speed options (branch `ovopt-20261025`)

Goal (user): faster per-frame processing on the S24, same or better accuracy, less heat; everything roll-back-able.
PC work first; phone camera-free replay after the user's approval (13:00, "최적화 옵션 폰에서 녹화 재생으로 비교 ㄱ").
Numbers marked 추정 are estimates; everything else is measured (files below). Two independent read-only verifiers
re-computed the identity / timing / ATE numbers (phase 1 and final); their wording notes are applied here.

## Conclusion first
- **Exact options (`--ovopt all_exact`) do not change a single pose**: identical at 17 significant digits on the PC
  (3 inputs) and **on the phone** (raw80 c4 and c4n400s100c20, every run); with no options this branch = the 0.6.17
  APK binary on the phone and = the established fps30 C4 bench results in 45/45 GT cells.
- **Phone c4 (real time, idle_spin 2, same start temperature)**: no options 21.3/40.0 ms p50/p95 (1 gated run, 14.7 %
  frames > 33 ms) -> all_exact 12.3/19.2, 13.0/20.4 (0 %). Of the difference ~5.4 ms/frame is saved directly by the
  options (retri visualisation work 3.9 ms, 2.7-4.0 depending on the thermal state); the rest is the backlog "cliff"
  that the slower run fell into (KLT 14.8 ms in backlog frames vs 6.0). Lower bound from the thermally unfavourable
  L1 pair: -2.2 ms p50 (-15 %).
- Near-exact set W3 = all_exact + ekf_gemm + qr_compress + ekf_chol: phone c4 10.9/16.6 (1 run; W with par_cov
  11.6-12.1/17.2-17.9 in 4 runs). Offline accuracy of every near-exact set (W, W2, W3, each option alone) = baseline:
  ATE median 0.5914 cm, paired 0.000 cm, same 8 divergent cells of 42.
- **Heat**: `--idle-spin-ms 15` (spin only the last 15 ms before the next frame) is as fast as the full idle spin and
  cooler (AP mean -1.3 C, max -1.7 C, 2 reps). WFE spin (`--idle-spin 3`) = no change. No spin = coolest but 22.8 ms p50.
- Configs: keep C4. max_slam 70 -0.018 cm paired (small, +24 % compute), max_slam 100 / 400 pts no clear gain, clones
  20 worse (11 vs 8 divergent cells), bigger update chunks slower without gain.

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
| par_cov | StateHelper.cpp | covariance update split in 2 equal-work parts on the pool (slower on the phone, 추정 from W vs W3) | near-exact |
Other flags (default off): `--prof-log FILE` (per-frame stage timers), `--pose-precision N` (17 for bit checks),
`--drop-late MS` (skip frames that would be fed > MS later than the best lag: no backlog spiral; changes results when
it drops), `--idle-spin 3` (pool waits with WFE between input polls), `--idle-spin-ms W` (spin only W ms before the
expected frame). App (arlat 0.6.18, versionCode 24) intents: ov_opt, ov_prof, drop_late_ms, idle_spin_ms, spin_wfe.
"near-exact" = same math, other rounding: converged cells differ only at rounding level, divergent cells can differ a lot.

## PC identity (`pc_ident/`)
- C4 harness (hs_20261024_165506_std60, 845 frames, cfg_identity/): main checkout build_par (= 0.6.17 bridge) = this
  branch default build = full-source build with no options = each exact option alone = all_exact, 845/845, hash
  d7958f8247fbc180 (= 0.6.16 P0) - `ident_pc_5.log` (final binary 71bb15d9).
- 17 digits (`ident17_2.log`): every exact option alone + all_exact identical on the harness (845), raw80 c4 (2327,
  with remap: frame_nocopy active) and raw80 c4n400s100c20 all_exact (2327).
- Idle-spin variants (paced feed, `spin_pc_1.log`): --idle-spin 2/3, -ms 15/10 all pose-identical.

## PC timing (`pc_prof/`, raw80 = phone replay stream, phone configs, --threads 2, unpaced, 3 interleaved repeats)
OpenVINS feed per tracking frame, ms p50 / p95; CPU = user+sys of the whole run (80 s stream); final binary (run 4):
| config | A none | B all_exact | W = B + ekf_gemm + qr_compress + ekf_chol + par_cov |
|---|---|---|---|
| c4 | 10.61 / 17.05, CPU 32.3 s | 7.50 / 11.27 (-29 % / -34 %), CPU 29.5 s | 5.64 / 8.35 (-47 % / -51 %), CPU 25.0 s |
| c4n400s100c20 | 26.39 / 38.63, CPU 64.9 s | 19.70 / 27.31 (-25 % / -29 %), CPU 62.2 s | 13.59 / 18.97 (-49 % / -51 %), CPU 48.6 s |
x86 build has no AVX/FMA (same flags as libov_msckf_lib.so); PC and phone stage ratios differ (phone retri is larger).

## Offline accuracy (`pc_ate/`, ate_run.py = fps30 C4 protocol: 42 HS GT cells (213955 excluded), camera-position ruler)
| cond | divergent | ate_w median cm | paired vs B0 (n, median / mean cm, better) | bench ms med / p95 |
|---|---|---|---|---|
| B0 this branch, no options | 8 | 0.5914 | - (= fps30 C4 poses in 45/45 cells) | 16.66 / 26.24 |
| X all_exact | 8 | 0.5914 | identical poses in 42/42 | 14.38 / 23.24 |
| W all options | 8 | 0.5914 | 34, 0.000 / 0.000, 16 | 11.79 / 18.87 |
| W2 (W without qr) | 8 | 0.5914 | 34, -0.000 / -0.000, 19 | 11.96 / 19.34 |
| W3 (W without par_cov) = recommended near-exact set | 8 | 0.5914 | 34, -0.000 / -0.000, 21 | 11.71 / 19.25 |
| G / Q / K / P (one near-exact option each) | 8 each | 0.5914 each | 34, 0.000 / 0.000 each | 15.07 / 17.12 / 17.60 / 16.87 med |
| S70 max_slam 70 | 8 | 0.5881 | 34, -0.0175 / -0.0151, 23 (sign test p 0.06) | 20.68 / 32.46 |
| S100 max_slam 100 | 8 | 0.6035 | 34, +0.0019 / -0.0048, 17 | 24.44 / 39.02 |
| N400 400 pts | 8 | 0.578 | 34, +0.0241 / -0.0474, 15 (mixed: 7 cells changed > 0.3 cm both ways) | 24.92 / 38.09 |
| C20 15 -> 20 clones | 11 (+134550 o0/o1, 221925 o1) | 0.6081 | 31, -0.0009 / +0.0197, 16 | 17.32 / 34.72 |
| U50 max_slam_in_update 50 | 8 | 0.5823 | 34, +0.0048 / +0.0028, 15 | 20.30 / 30.91 |
| M60 max_msckf_in_update 60 | 8 | 0.5663 | 34, -0.0144 / +0.0003, 21 | 17.77 / 28.45 |
Bench ms = single-threaded bench (num_opencv_threads 1, 8 jobs in parallel on the PC), so the par_* options do not
show there. Score file: `pc_ate/score_B0_W_W2_W3_X_G_Q_K_P_S70_S100_N400_C20_U50_M60.json`. Not run (stopped for time):
S100U50 (partial), N400M80, N400S100C20, N300S70.

## Backlog "cliff" guard (`pc_drop/`): `--drop-late 50`, PC 1-core overload (c4n400s100c20, real-time feed)
lag p95 7.8 s -> 86 ms (max 7.9 s -> 0.14 s), 11 % of frames skipped, ATE 0.32 -> 0.30 cm (A) / 0.32 -> 0.26 cm
(all_exact); 1 rep each. Not tested on the phone (L5 not run).

## Phone (camera-free replay, raw80 + phone remap + phone configs with timeshift 0 like the replay study)
Runner `phone/phone_run.sh` (own dir /data/local/tmp/ovopt, own process names, reads the replay study's raw80.bin /
feeder3 / remap read-only), lists `phone/L*.txt`, analysis `phone/phone_an.py`, `phone_sum.py`, `heat_an.py`.
Screen was on during all runs (foreground Settings, later the Brave browser); battery discharging until 14:20, after
that the user used the phone (Twitter) and charged it -> the remaining runs were stopped (status.txt).
- L1 identity (`out_L1_ident/`): I_none17 vs I_exact17 2327/2327 identical at 17 digits (hash 4df353f02098f19a);
  I_ref (0.6.17 APK binary) vs I_none (with --prof-log) 2327/2327 identical (aa71a3815e7d3ae4); near-exact differs from
  frame 324. L1 timing is thermally confounded (big-core cap 2784 -> 2016 MHz within 4 runs).
- L2-L6 gate: SKIN < 35 and AP < 37 C + 30 s cool-down; every gated run started at cap 2784 MHz, SKIN 34.0-34.9 C.
- Timing (proc = remap + feed per tracking frame, ms p50 / p95, per run; `phone_sum_L1L2.md`, `out_L6_flags/`):
  | config | A no options | B all_exact | W (with par_cov) | W3 (without par_cov) |
  |---|---|---|---|---|
  | c4 | 21.3 / 40.0 (14.7 % > 33 ms; 1 gated run) | 12.3 / 19.2, 13.0 / 20.4 (L4 spin-2 runs 12.7 / 19.2, 13.6 / 21.3) | 11.6 / 17.5, 11.7 / 17.9, 12.1 / 17.5 | 10.9 / 16.6 (1 run) |
  | c4n400s100c20 | 51.2 / 78.8, 51.1 / 77.5 (92 % > 33, 40 s backlog) | 31.3 / 52.2, 31.0 / 51.1 (42 %) | 25.0 / 43.3, 23.3 / 42.2 (20 %) | - |
- Phone stages p50 (c4, gated, `prof.csv`): retri 3.9 ms in A (skipped by all_exact); M = P H^T 2.2 -> 1.8 (par_ma +
  par_s, the B timer also holds S) -> 0.8 (ekf_gemm); msckf compression p95 4.2 -> 3.4 (givens_rm) -> 2.0 (qr);
  K / S^-1 0.49 -> 0.29 (ekf_chol); covariance update 0.51 (B) / 0.60 (W) / 0.53 (W3).
- Heat (L4, c4 all_exact, 80 s runs, 2 reps each; `out_L4_heat/heat_an.md`):
  | idle spin | proc p50 / p95 | > 33 ms | AP mean / max C | SKIN max C | battery current mean (coarse) |
  |---|---|---|---|---|---|
  | 2 (pool, = live now) | 12.7 / 19.2, 13.6 / 21.3 | 0 | 42.4 / 45.0, 42.6 / 45.4 | 36.4, 36.6 | -885, -735 |
  | 2 + `--idle-spin-ms 15` | 11.9 / 17.5, 12.3 / 18.0 | 0-0.1 % | 41.1 / 43.6, 41.2 / 43.5 | 36.3, 36.4 | -618, -710 |
  | 2 + `--idle-spin-ms 22` | 12.5 / 21.2, 12.4 / 19.7 | 0-0.1 % | 42.6 / 45.0, 42.7 / 45.8 | 36.7, 36.7 | -524, -618 |
  | 3 (pool, WFE wait) | 12.6 / 19.6, 13.3 / 20.5 | 0-0.1 % | 42.8 / 44.2, 42.2 / 44.5 | 36.4, 36.6 | -822, -866 |
  | 1 thread + main spin | 14.7 / 22.4, 15.2 / 23.5 | 0.2-0.3 % | 41.6 / 43.2, 41.8 / 43.5 | 36.5, 36.6 | -854, -729 |
  | 0 (no spin) | 22.8 / 33.6, 22.8 / 33.2 | 4.9-5.6 % | 38.6 / 39.8, 38.5 / 39.9 | 35.2, 35.3 | -501, -300 |
  Every L4 run has the same poses as no options (aa71a3815e7d3ae4). Battery "current now" (dumpsys) refreshes only every
  ~30 s -> coarse. 80 s runs: long-run heat (6 min live) not measured.
- Compiler flags (L6, 1 run each): `-march=armv8.2-a+fp16+dotprod` and `-flto=thin` builds give identical poses (17
  digits) and no speed gain (13.7 / 21.0, 14.0 / 21.5 vs default all_exact 12.3-13.6 p50) -> not adopted.

## Live test (needs the user: camera; after "ㄱ"), install, rollback
APK `...\scratchpad\arlat_0.6.18_ovopt.apk` (versionCode 24, 0.6.18-ovopt-20261025; libov_bridge.so sha256 7523659...
= the binary tested on the phone). Without the new intents it behaves like 0.6.17.
```
adb -s 100.79.156.20:5555 install -r C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\arlat_0.6.18_ovopt.apk
# WSL, from ~/scanner/StereoDepthSeg-ovopt (each 40 s C4, same motion as before):
LIVE=1 SEG=still0:0:8,hand:8:38,still:38:40 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4 42 --ei idle_spin 2
LIVE=1 SEG=still0:0:8,hand:8:38,still:38:40 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4 42 --ei idle_spin 2 --es ov_opt all_exact --ei ov_prof 1
LIVE=1 SEG=still0:0:8,hand:8:38,still:38:40 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4 42 --ei idle_spin 2 --ei idle_spin_ms 15 --es ov_opt all_exact,ekf_gemm,qr_compress,ekf_chol --ei ov_prof 1
# optional backlog guard: add --ei drop_late_ms 50
adb -s 100.79.156.20:5555 install -r -d C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\arlat_0.6.17.apk   # rollback
```
Code rollback: the branch is separate (gyumingim untouched); every option is off unless passed.

## Files
- scripts: `ident_pc.sh`, `ident17.sh` (identity), `prof_pc.sh` + `prof_an.py` + `cpu_an.py` (timing / stages / CPU),
  `spin_pc.sh` (idle-spin variants), `droplate_pc.sh` + `feeder3.cpp` + `ate_raw80.py` (backlog guard),
  `ate_run.py` (offline GT accuracy), `cmp.py` (pose comparison), `phone/` (phone runner, lists, analysis).
- raw outputs: phone runs in `phone/out_L*/` (po.jsonl.gz, tl.csv, prof.csv, samp.txt, run.txt), PC raw in `raw/*.tgz`
  (not committed, sha256 in raw/MANIFEST.sha256) and /tmp/ovopt on rtx5080 WSL; offline cells in /tmp/ovopt/ate.
