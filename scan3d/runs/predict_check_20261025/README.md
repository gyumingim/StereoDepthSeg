# ov_bridge --predict check (2026-10-25, arlat 0.6.17, PC only, no camera)

`--predict 1`: after each frame update the bridge integrates the filter IMU state (read only, own copy) with every IMU
sample newer than the state time up to the newest received IMU sample (fast_state_propagate mean equations) and writes a
`{"type":"pred",...}` line once the input pipe is drained. `--predict 2` also writes one between frames every
1/`--predict-hz` s of IMU time.

Input: the captured C4 bridge input stream of hs_20261024_165506_std60 (/tmp/idlespin/pose_ab/stream.bin, from the 0.6.16
check), config runs/vio_v3w4kbl8 (= 0.6.16 paced check P0), --threads 2, bench args, paced real-time feed
(idle_spin_check_20261025/paced_feed.py). `make_delayed.py` makes streams where each camera record comes after the IMU up to
t_cam + 40 / 60 ms (= phone camera HAL latency). Scripts: run_pc.sh, pred_acc.py. 1 run each.

## Identity (pc/run_pc.log, pc/cmp_delayed.log)
- Original stream: --predict 0 / 1 / 2: 845/845 pose lines identical (all non-timing fields) to the 0.6.16 bridge P0.
- 40 ms delayed stream: --predict 0 / 1 / 2(200 Hz): 845/845 identical lines among themselves; state (p,q,v,bg,ba,T)
  845/845 identical to P0 too (only `imu_held` differs = the input order). 60 ms: 844 poses (last frame never fed, its
  IMU ends first), state identical.

## Accuracy (truth = the VIO pose of the same run at the same instant, interpolated; tracking only)
Recording is gentle hand motion: speed p50 0.16, p95 0.57, max 0.78 m/s.

| stream | src | horizon ms p50 / p95 | pred pos err cm p50 / p95 / max | ori deg p50 / p95 | no-pred (last pose) cm p50 / p95 | const-vel cm p50 / p95 |
|---|---|---|---|---|---|---|
| no delay | frame | 10 / 17 | 0.02 / 0.06 / 0.20 | 0.04 / 0.20 | 0.17 / 0.73 | 0.02 / 0.09 |
| 40 ms | frame | 42 / 49 | 0.04 / 0.15 / 0.49 | 0.04 / 0.19 | 0.71 / 2.53 | 0.10 / 0.35 |
| 40 ms | rate 200 Hz | 56 / 66 | 0.05 / 0.17 / 0.63 | 0.03 / 0.17 | 0.66 / 3.16 | 0.11 / 0.50 |
| 60 ms | frame | 63 / 71 | 0.06 / 0.20 / 0.61 | 0.03 / 0.13 | 1.05 / 3.66 | 0.18 / 0.61 |

Prediction removes ~95 % of the lag error at 40-70 ms horizons on this recording. Faster / shakier motion (drone) untested.
