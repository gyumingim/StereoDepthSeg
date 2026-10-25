# imuprio 2026-10-25 — arlat 0.6.19: IMU priority send + prediction thread (branch `ovopt-20261025`)

Goal: the IMU forward-predicted pose should be fresher. Two causes found in 0.6.18 (read-only analysis): IMU records wait
behind ~2 MB image writes in the app's single FIFO, and `--predict` can only emit while the bridge main thread is idle.
Everything is behind flags; defaults = 0.6.18. Numbers marked 추정 are estimates. 1 run per cell unless noted.

## Conclusion first
- **VIO poses unchanged**: PC (17 digits) and phone (17 digits, raw80 c4 all_exact, 2327 frames) pose lines identical with
  and without the new flags; PC reordered + chunked input (= app imu_prio send order) identical as long as the IMU lead
  over the image stays < `--imu-hold` 0.25 s (lead 0 / 0.1 / 0.18 s identical; lead 0.4 s control differs).
- **Predicted pose age at a 1 kHz consumer (PC paced, 40 ms camera delay)**: predict 1 p50 25.5 / 23.3, p95 156 / 119 ms
  -> predict 2 + pred thread + chunked images p50 4.0 / 4.4, p95 9.5 / 11.1 ms (2 reps; PC load 13-19 from other agents,
  so the absolute numbers are 참고). Without chunked images the thread alone had a bad rep (p95 106 ms: the feeder's
  2 MB write blocked the IMU) -> use both together.
- **Accuracy at 200 Hz** (truth = the later VIO pose, not GT): rate lines pos err p50 0.06-0.08 / p95 0.24-0.27 cm, ori
  p50 0.04-0.05 / p95 0.19-0.21 deg at horizon p50 68-77 ms (no prediction: 1.15-1.31 / 4.3-5.8 cm).
- **Phone cost (camera-free replay, final binary, c4 all_exact idle_spin 2, 17-digit output)**: pred thread 7.7-7.9 s CPU
  and reader thread 11.7-12.1 s per 80 s run; whole bridge 172 s vs 148 s CPU (+24 s = +0.3 core, 1 rep each, includes the
  idle-spin threads); per-frame proc p50 9.8 / 10.1 vs 9.6 ms (p95 14.5 / 14.9 vs 14.2) = +0.2-0.5 ms (noise level, 추정).
  Prediction compute p50 0.29 ms on the phone (mostly 17-digit JSON formatting, 추정; live uses 9 digits).
- Live latency on the phone: not measured (needs the user + camera). Commands below.

## What changed (commits 8bc2628, 0fcf53a)
| where | flag (default) | what |
|---|---|---|
| ov_bridge main.cpp | record types 5 / 6 (never sent by old clients) | CHUNK / CHUNK_END = pieces of one type-2 STEREO payload; IMU may come between pieces |
| ov_bridge | `--pred-thread 0\|1` (0) | with `--predict 1\|2`, stdio: reader thread reads all records into an in-memory queue (main thread consumes the same sequence; at most 2 unprocessed images, then it stops reading), prediction thread propagates the latest filter-state snapshot (copied by the main thread after each frame) with all received IMU; mode 2 emits on a 1/HZ grid of IMU time, also while a frame is processed; incremental (continues from the previous prediction while the snapshot is unchanged; `OVB_PRED_CHECK=1` compares with from-scratch: 0 of 1189 / 2724 / 4262 differ). pred lines get `tb` |
| ov_bridge | `--imu-log FILE` (off) | `t_ns,rx_hdr_boot_ns,rx_boot_ns` per IMU record |
| arlat OvRunner / MainActivity | intent `imu_prio` (0) | IMU own queue; writer sends waiting IMU first, then the image as 64 KB chunks with IMU between chunks, flush after each IMU batch; IMU ahead of a queued image only up to t_image + 200 ms |
| arlat | intent `pred_thread` (1; only used with `predict 2`) | `--pred-thread 1`; `pred_thread 0` = 0.6.18 predict 2 |
| arlat | intent `imu_log` (0) | imu_samples.csv (type, event.timestamp, onSensorChanged time; ring 2^19), imu_enq.csv, imu_writes.csv (deq, write/flush returned), bridge_imu.csv |
| ov_phone_timing.py | - | predicted-pose age at a 1 kHz consumer; IMU delivery chain per sensor / per record |
APK 0.6.19-imuprio-20261025 (versionCode 25), protection unchanged (40 C self-stop, preflight, thermal stop rules untouched).

## Evidence
- `pc_ident/ident_pc.log` (C4 harness hs_20261024_165506_std60, 845 frames, 17 digits): new binary default = 0.6.18
  binary (71bb15d9) 845/845; predict 2 old / predict 2 + pred thread / predict 1 + pred thread = predict 0 incl. imu_held;
  reordered streams (`reorder.py`) L0 / L0.1 / L0.18 (+ pred thread) identical excl. imu_held; L0.4 unchunked differs
  (29/845). `pc_ident/ident_final_binary.log`: final binary (after the incremental / buffer change) on the L0.4 stream:
  default = 0.6.18 binary, pred thread identical, check 0 differ. (/tmp/idlespin was deleted by someone during the work,
  so the final-binary check used a stream derived from it.)
- `pc_paced/paced_seq.log` (+ `*.rx.json`): paced real-time feed (`pfeed.py`) of the 40 ms delayed harness stream, 2 reps,
  P1 = predict 1, P2o = 0.6.18 predict 2, P2t = pred thread, P2tc = pred thread + chunked (`reorder.py --lead 0.04
  --imu-per-chunk 16`). `pc_paced/check_incremental.log`: final binary P2tc age p50 5.1 / p95 17.0 ms (1 rep, load high).
- Phone (`phone/`, camera-free, raw80 c4 + all_exact + idle_spin 2, own dir /data/local/tmp/imuprio): out_Q_first_binary
  (first build; reader allocated a new 2 MB buffer per frame: reader 25 s CPU) and out_R_final_binary (R3, R1, R3b
  interleaved). All pose lines = ovopt L1 I_exact17 (hash 4df353f02098f19a). Battery discharging (status 3) in all R runs
  (R3b ended charging); the user had a foreground app (Toss) at 16:44:45 just before R3 (runs started with the launcher in
  front) -> R timing possibly contaminated.

## Not verified
- App writer (imu_prio, Java) not executed (needs the camera); framing = reorder.py emulation (same layout).
- Live IMU delivery latency, live predicted-pose age; benefit of imu_prio without pred_thread is ~0 (old bridge loop does
  not read while processing a frame).
- The 200 ms cap protects only images already queued in the app; an image reaching onImage > ~250 ms after exposure
  would change the feed order (same in 0.6.18). In a backlog (2 unprocessed images) the reader stops reading IMU too.

## Live A/B (needs the user: camera; each ~60 s C4, same motion; WSL from ~/scanner/StereoDepthSeg-ovopt)
```
adb -s 100.79.156.20:5555 install -r C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\arlat_0.6.19.apk
W3="--es ov_opt all_exact,ekf_gemm,qr_compress,ekf_chol --ei idle_spin 2 --ei idle_spin_ms 15"
# A: as 0.6.18 best + predict 1
LIVE=1 SEG=still0:0:8,hand:8:58,still:58:60 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4 62 $W3 --ei predict 1 --ei imu_log 1
# B: predict 2 at 200 Hz with the prediction thread + IMU priority send
LIVE=1 SEG=still0:0:8,hand:8:58,still:58:60 bash scan3d/tools/ov_phone_run.sh run 100.79.156.20:5555 c4 62 $W3 --ei predict 2 --ei predict_hz 200 --ei imu_prio 1 --ei imu_log 1
# report: python3 scan3d/tools/ov_phone_timing.py <run dir>  (pred latency, age at 1 kHz consumer, IMU delivery)
# rollback
adb -s 100.79.156.20:5555 install -r -d C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\arlat_0.6.18_ovopt.apk
```
Baseline from the 0.6.18 live run run_20261025_153602_main30_c4 (predict 1): age at a 1 kHz consumer p50 21.3 / p95 37.7 ms.
