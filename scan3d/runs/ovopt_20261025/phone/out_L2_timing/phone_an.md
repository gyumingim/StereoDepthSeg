| run | proc p50 / p95 / max | >33 ms % | feed p50 / p95 | remap p50 | lag p95 / max ms | pre-init remap / feed | n_slam | SKIN start / max | AP start / end | cpu9 cap | screen | stages p50 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| N_exact_r1 | 31.28 / 52.22 / 96.3 | 42.8 | 28.96 / 49.36 | 2.69 | 4027.75 / 4704.9 | 1.16 / 3.17 | 100.0 | 34.8 / 36.7 | 35.8 / 42.0 | 2784000 | Awake | track 9.35 msckf_ekf 4.57 slam_ekf 9.3 msckf_comp 0.94 msckf_jac 0.9 retri 0.0 ekf_ma 7.14 ekf_cov 2.55 ekf_k 1.69 msckf_comp_p95 7.58 |
| N_exact_r2 | 30.95 / 51.1 / 99.5 | 41.3 | 28.81 / 48.07 | 2.49 | 3311.99 / 4379.5 | 1.18 / 3.32 | 100.0 | 34.9 / 36.5 | 36.0 / 41.6 | 2784000 | Awake | track 9.23 msckf_ekf 4.51 slam_ekf 8.99 msckf_comp 0.97 msckf_jac 0.88 retri 0.0 ekf_ma 7.03 ekf_cov 2.62 ekf_k 1.69 msckf_comp_p95 7.52 |
| N_near_r1 | 25.0 / 43.34 / 108.1 | 20.8 | 22.86 / 40.72 | 2.0 | 1132.46 / 2316.5 | 1.2 / 3.31 | 100.0 | 34.9 / 36.8 | 35.9 / 41.1 | 2784000 | Awake | track 8.26 msckf_ekf 2.93 slam_ekf 6.3 msckf_comp 0.58 msckf_jac 0.74 retri 0.0 ekf_ma 3.2 ekf_cov 3.63 ekf_k 0.86 msckf_comp_p95 3.71 |
| N_near_r2 | 23.28 / 42.16 / 114.4 | 19.1 | 21.59 / 39.4 | 1.85 | 986.03 / 2275.8 | 1.19 / 3.27 | 100.0 | 34.8 / 36.5 | 35.9 / 41.5 | 2784000 | Awake | track 7.56 msckf_ekf 2.91 slam_ekf 5.81 msckf_comp 0.56 msckf_jac 0.73 retri 0.0 ekf_ma 2.89 ekf_cov 3.34 ekf_k 0.86 msckf_comp_p95 3.58 |
| N_none_r1 | 51.15 / 78.84 / 108.0 | 92.2 | 46.54 / 72.49 | 4.24 | 40742.24 / 44779.1 | 1.23 / 3.22 | 100.0 | 34.9 / 36.5 | 35.8 / 40.2 | 2784000 | Awake | track 19.13 msckf_ekf 5.15 slam_ekf 11.47 msckf_comp 1.29 msckf_jac 0.88 retri 4.74 ekf_ma 8.2 ekf_cov 2.56 ekf_k 1.67 msckf_comp_p95 9.93 |
| N_none_r2 | 51.06 / 77.5 / 117.9 | 91.5 | 46.81 / 71.66 | 4.2 | 39410.22 / 43251.8 | 1.23 / 3.1 | 100.0 | 34.9 / 36.4 | 36.0 / 40.2 | 2784000 | Awake | track 19.33 msckf_ekf 5.31 slam_ekf 10.93 msckf_comp 1.28 msckf_jac 0.87 retri 4.57 ekf_ma 8.31 ekf_cov 2.62 ekf_k 1.74 msckf_comp_p95 9.75 |
| T_exact_r2 | 12.29 / 19.16 / 30.8 | 0.0 | 10.63 / 17.28 | 1.71 | 21.75 / 34.6 | 1.21 / 3.32 | 50.0 | 34.5 / 36.5 | 35.9 / 42.1 | 2784000 | Awake | track 5.16 msckf_ekf 1.2 slam_ekf 2.11 msckf_comp 0.01 msckf_jac 0.25 retri 0.0 ekf_ma 1.79 ekf_cov 0.51 ekf_k 0.5 msckf_comp_p95 3.36 |
| T_exact_r3 | 13.03 / 20.38 / 33.8 | 0.0 | 11.3 / 18.2 | 1.75 | 23.37 / 36.7 | 1.22 / 3.29 | 50.0 | 34.6 / 36.5 | 35.3 / 41.6 | 2784000 | Awake | track 5.39 msckf_ekf 1.24 slam_ekf 2.24 msckf_comp 0.01 msckf_jac 0.26 retri 0.0 ekf_ma 1.92 ekf_cov 0.54 ekf_k 0.52 msckf_comp_p95 3.51 |
| T_near_r2 | 11.6 / 17.51 / 30.8 | 0.0 | 9.83 / 15.56 | 1.83 | 20.35 / 33.3 | 1.2 / 3.39 | 50.0 | 34.8 / 36.5 | 36.1 / 41.7 | 2784000 | Awake | track 5.44 msckf_ekf 0.81 slam_ekf 1.26 msckf_comp 0.01 msckf_jac 0.27 retri 0.0 ekf_ma 0.84 ekf_cov 0.6 ekf_k 0.29 msckf_comp_p95 2.0 |
| T_near_r3 | 11.69 / 17.88 / 28.0 | 0.0 | 9.8 / 15.6 | 1.86 | 20.72 / 31.5 | 1.2 / 3.21 | 50.0 | 34.7 / 36.6 | 35.4 / 41.2 | 2784000 | Awake | track 5.44 msckf_ekf 0.74 slam_ekf 1.32 msckf_comp 0.01 msckf_jac 0.27 retri 0.0 ekf_ma 0.8 ekf_cov 0.58 ekf_k 0.29 msckf_comp_p95 1.99 |
| T_none_r3 | 21.3 / 40.04 / 68.4 | 14.7 | 18.85 / 35.53 | 2.78 | 758.39 / 1570.2 | 1.24 / 3.2 | 50.0 | 34.7 / 36.6 | 35.7 / 41.5 | 2784000 | Awake | track 7.1 msckf_ekf 1.42 slam_ekf 2.83 msckf_comp 0.01 msckf_jac 0.34 retri 3.86 ekf_ma 2.2 ekf_cov 0.51 ekf_k 0.49 msckf_comp_p95 4.16 |

per variant (median over runs of the per-run values):
  N_exact: runs 2 proc p50 31.12 p95 51.66 >33% 42.05 feed p50 28.88 lag p95 3669.87 | pre-init feed 3.25
  N_near: runs 2 proc p50 24.14 p95 42.75 >33% 19.95 feed p50 22.23 lag p95 1059.24 | pre-init feed 3.29
  N_none: runs 2 proc p50 51.11 p95 78.17 >33% 91.85 feed p50 46.67 lag p95 40076.23 | pre-init feed 3.16
  T_exact: runs 2 proc p50 12.66 p95 19.77 >33% 0.0 feed p50 10.96 lag p95 22.56 | pre-init feed 3.3
  T_near: runs 2 proc p50 11.64 p95 17.7 >33% 0.0 feed p50 9.82 lag p95 20.54 | pre-init feed 3.3
  T_none: runs 1 proc p50 21.3 p95 40.04 >33% 14.7 feed p50 18.85 lag p95 758.39 | pre-init feed 3.2
