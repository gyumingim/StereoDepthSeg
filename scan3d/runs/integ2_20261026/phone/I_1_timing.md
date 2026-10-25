# phone replay out_I_1 (made 2026-10-26 06:02, input out_I_1)

feed_ms = VIO processing per frame (tl.csv, frames with init=1); parts = p50 on frames with n_slam>0 (prof.csv).
cpu9 cap = big-core scaling_max_freq (MHz): start (run.txt, one read) / median / min / max during run (samp.txt every 5 s); cpuinfo_max_freq read 3207 MHz at 01:27 (one read). CAP = median below 3207.

| run | start | n | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_s | ekf_cov | ekf_sym | msckf_ekf | cpu9 cap start/med/min/max | SKIN start/max | AP max | battery | cur med mA | aborted |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| d_m0 | 05:52:07 | 330 | 8.77 | 8.34 | 13.14 | 19.7 | 0 | nan | nan | nan | nan | nan | nan | 2304/2304/2304/2304 CAP | 29.6/31.9 | 38.6 | AC st4 lvl95 T28.1 bst4 | 2.0 | 0 |
| d_m1 | 05:52:59 | 861 | 8.93 | 8.63 | 12.77 | 28.0 | 0 | nan | nan | nan | nan | nan | nan | 2304/2304/2304/2304 CAP | 31.9/33.3 | 41.2 | AC st4 lvl95 T29.8 bst4 | 7.0 | 0 |
| t_N_r1 | 05:53:50 | 403 | 9.82 | 8.15 | 13.22 | 328.3 | 3 | 0.63 | 0.46 | 0.00 | 0.54 | 0.06 | 0.82 | 2304/2304/672/2304 CAP | 33.3/33.9 | 41.2 | AC st4 lvl95 T31.0 bst4 | 5.0 | 0 |
| t_O_r1 | 05:55:36 | 501 | 6.28 | 5.24 | 9.56 | 165.1 | 4 | 0.00 | 0.13 | 0.00 | 0.02 | 0.01 | 0.17 | 2304/2304/2304/2304 CAP | 33.7/34.5 | 41.1 | AC st4 lvl95 T31.7 bst4 | 5.0 | 0 |
| t_O_r2 | 05:57:18 | 1067 | 9.15 | 8.95 | 12.33 | 44.5 | 1 | 1.01 | 0.78 | 0.00 | 0.55 | 0.10 | 0.83 | 2304/2304/2304/2304 CAP | 34.5/35.9 | 44.8 | AC st4 lvl95 T32.6 bst4 | 5.0 | 0 |
| t_N_r2 | 05:58:51 | 1924 | 10.13 | 9.43 | 16.25 | 32.9 | 0 | 0.71 | 0.45 | 0.00 | 0.54 | 0.06 | 0.71 | 2304/2112/1632/2304 CAP | 35.6/36.7 | 45.5 | AC st4 lvl95 T33.7 bst4 | 5.0 | 0 |

## per condition (feed mean/p50/p95 and parts = mean of per-run values; max = max over runs; >33 ms = sum over runs; not aborted)

| max_slam | cond | runs | feed mean | feed p50 | p95 | max | >33 ms | slam_ekf p50 | ekf_ma | ekf_cov | cpu9 cap median (each run) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| s50 | N | 2 | 9.98 | 8.79 | 14.73 | 328.3 | 3 | 0.67 (mean 0.69) | 0.46 | 0.54 | 2304, 2112 |
| s50 | O | 2 | 7.72 | 7.10 | 10.94 | 165.1 | 5 | 0.50 (mean 0.51) | 0.45 | 0.29 | 2304, 2304 |
