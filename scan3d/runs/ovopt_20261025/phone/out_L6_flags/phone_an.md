| run | proc p50 / p95 / max | >33 ms % | feed p50 / p95 | remap p50 | lag p95 / max ms | pre-init remap / feed | n_slam | SKIN start / max | AP start / end | cpu9 cap | screen | stages p50 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F_lto_r1 | 13.99 / 21.48 / 33.7 | 0.0 | 12.07 / 19.29 | 1.92 | 24.81 / 36.7 | 1.17 / 3.19 | 50.0 | 34.7 / 36.7 | 35.6 / 40.4 | 2784000 | Awake | track 6.06 msckf_ekf 1.31 slam_ekf 2.37 msckf_comp 0.01 msckf_jac 0.29 retri 0.0 ekf_ma 2.02 ekf_cov 0.55 ekf_k 0.56 msckf_comp_p95 3.7 |
| F_march_r1 | 13.68 / 21.01 / 31.1 | 0.0 | 11.83 / 18.84 | 1.9 | 23.86 / 34.7 | 1.25 / 3.32 | 50.0 | 34.8 / 36.6 | 35.8 / 41.9 | 2784000 | Awake | track 5.9 msckf_ekf 1.26 slam_ekf 2.35 msckf_comp 0.01 msckf_jac 0.28 retri 0.0 ekf_ma 1.98 ekf_cov 0.54 ekf_k 0.52 msckf_comp_p95 3.7 |
| F_w3_r1 | 10.92 / 16.63 / 29.3 | 0.0 | 9.22 / 14.53 | 1.67 | 19.5 / 32.4 | 1.16 / 3.19 | 50.0 | 34.0 / 36.4 | 34.7 / 39.8 | 2784000 | Awake | track 5.19 msckf_ekf 0.7 slam_ekf 1.19 msckf_comp 0.01 msckf_jac 0.25 retri 0.0 ekf_ma 0.77 ekf_cov 0.53 ekf_k 0.28 msckf_comp_p95 1.9 |
| F_w_r1 | 12.13 / 17.48 / 29.6 | 0.0 | 10.25 / 15.42 | 1.86 | 20.27 / 31.8 | 1.16 / 3.23 | 50.0 | 34.9 / 36.6 | 36.0 / 41.2 | 2784000 | Awake | track 5.76 msckf_ekf 0.81 slam_ekf 1.32 msckf_comp 0.01 msckf_jac 0.28 retri 0.0 ekf_ma 0.84 ekf_cov 0.6 ekf_k 0.3 msckf_comp_p95 1.98 |

per variant (median over runs of the per-run values):
  F_lto: runs 1 proc p50 13.99 p95 21.48 >33% 0.0 feed p50 12.07 lag p95 24.81 | pre-init feed 3.19
  F_march: runs 1 proc p50 13.68 p95 21.01 >33% 0.0 feed p50 11.83 lag p95 23.86 | pre-init feed 3.32
  F_w: runs 1 proc p50 12.13 p95 17.48 >33% 0.0 feed p50 10.25 lag p95 20.27 | pre-init feed 3.23
  F_w3: runs 1 proc p50 10.92 p95 16.63 >33% 0.0 feed p50 9.22 lag p95 19.5 | pre-init feed 3.19
