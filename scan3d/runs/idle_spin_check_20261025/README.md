# ov_bridge --idle-spin check (2026-10-25, arlat 0.6.16, no camera)

## PC identity (pc/)
Recording hs_20261024_165506_std60, C4 setup (960x540, 30 fps, 15 clones, 200 pts, imu-raw), PC build build_par.
- Fast feed (vio_bench_hs via bench_wrap, pose_ab.sh): --threads 2 with --idle-spin 0 / 1 / 2, --threads 4 --idle-spin 2,
  config threads: 845/845 pose lines identical (all fields except ms / remap_ms / wait_ms: T_world_cam0, p, q, v, bg, ba,
  calib, counts). Also identical to the 0.6.14 stock bridge (/tmp/kb0615/pose_ab/A_stock.jsonl).
- Real-time paced feed of the same input stream (paced_feed.py, paced.sh): modes 0/1/2 also 845/845 identical
  (here mode 1/2 spun in ~12 900 of 15 921 waits).
- Mode 2 exit latency on PC (first stripe saw input -> parallel_for_ returned): all waits p50 1.9 us, p95 4.5, p99 28.7,
  max 396 us; waits right before a frame p50 0.9 us, max 49 us.

## Phone (phone/), camera-free, adb shell (root cgroup, not top-app)
feeder.cpp (NDK, sleeps between records, no spinning) replays the first 12 s of the same stream at real time into the
0.6.16 bridge binary from the APK (/data/local/tmp/idlespin), c4 config from /data/local/tmp/kb0615/cfg, --threads 2.
One rep per mode, order 0 -> 1 -> 2, 5 s apart; AP 31.6 -> 39.2 C, SKIN 31.1 -> 32.1 C over the three runs.

| mode | feed_ms p50 | p95 | max | exit_us before frames p50 / max |
|---|---|---|---|---|
| 0 off  | 23.7 | 44.2 | 61.1 | - |
| 1 main | 24.8 | 45.1 | 59.0 | - |
| 2 pool |  7.0 | 17.7 | 24.8 | 1.2 / 3.1 (all waits: p50 1.4, p99 7.0, max 342) |

Mode 1 alone did not help here (bench T1 "spin" did); mode 2 did. 1 rep, root cgroup, fixed replay timing, heat rising
over the order (against mode 2) -> live effect in the app (top-app cgroup, camera jitter) still unmeasured.
