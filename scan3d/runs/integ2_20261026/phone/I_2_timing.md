# phone replay out_I_2 (made 2026-10-26 06:13, input out_I_2)

feed_ms = VIO processing per frame (tl.csv, frames with init=1); parts = p50 on frames with n_slam>0 (prof.csv).
cpu9 cap = big-core scaling_max_freq (MHz): start (run.txt, one read) / median / min / max during run (samp.txt every 5 s); cpuinfo_max_freq read 3207 MHz at 01:27 (one read). CAP = median below 3207.

| run | start | n | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_s | ekf_cov | ekf_sym | msckf_ekf | cpu9 cap start/med/min/max | SKIN start/max | AP max | battery | cur med mA | aborted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| t2_N_r1 | 06:04:20 | 2003 | 12.37 | 8.55 | 16.80 | 401.4 | 36 | 0.64 | 0.46 | 0.00 | 0.54 | 0.06 | 0.77 | 2304/2304/2304/2304 CAP | 33.4/36.0 | 44.4 | AC st4 lvl95 T32.3 bst4 | 2.0 | 0 |
| t2_O_r1 | 06:06:09 | 2003 | 13.42 | 10.82 | 19.84 | 385.6 | 21 | 1.41 | 0.86 | 0.00 | 0.55 | 0.11 | 0.81 | 2304/2112/1632/2304 CAP | 35.5/36.8 | 45.2 | AC st4 lvl95 T34.3 bst4 | 5.0 | 0 |
| t2_O_r2 | 06:08:08 | 2003 | 15.73 | 13.26 | 21.63 | 399.0 | 27 | 1.55 | 1.09 | 0.00 | 0.63 | 0.13 | 0.89 | 2016/1728/1632/2208 CAP | 35.8/36.6 | 42.9 | AC st4 lvl95 T34.4 bst4 | 2.0 | 0 |
| t2_N_r2 | 06:10:16 | 2003 | 12.20 | 10.77 | 17.41 | 341.3 | 9 | 0.82 | 0.49 | 0.00 | 0.55 | 0.07 | 0.81 | 2208/1824/1824/2304 CAP | 35.7/36.8 | 44.0 | AC st4 lvl95 T34.5 bst4 | 2.0 | 0 |

## per condition (feed mean/p50/p95 and parts = mean of per-run values; max = max over runs; >33 ms = sum over runs; not aborted)

| max_slam | cond | runs | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_cov | cpu9 cap median (each run) |
|---|---|---|---|---|---|---|---|---|---|---|---|
