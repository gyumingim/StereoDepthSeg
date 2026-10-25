| run | proc p50 / p95 / max | >33 ms % | feed p50 / p95 | remap p50 | lag p95 / max ms | pre-init remap / feed | n_slam | SKIN start / max | AP start / end | cpu9 cap | screen | stages p50 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| I_exact17 | 12.52 / 19.53 / 29.9 | 0.0 | 10.9 / 17.57 | 1.65 | 22.08 / 32.8 | 1.24 / 3.32 | 50.0 | 35.7 / 36.6 | 42.3 / 42.4 | 2592000 | Awake | track 5.11 msckf_ekf 1.24 slam_ekf 2.14 msckf_comp 0.01 msckf_jac 0.25 retri 0.0 ekf_ma 1.83 ekf_cov 0.52 ekf_k 0.52 msckf_comp_p95 3.37 |
| I_near17 | 11.92 / 17.24 / 26.6 | 0.0 | 10.09 / 15.28 | 1.85 | 19.84 / 29.3 | 1.41 / 3.63 | 50.0 | 35.7 / 36.4 | 38.2 / 41.8 | 2112000 | Awake | track 5.54 msckf_ekf 0.83 slam_ekf 1.26 msckf_comp 0.01 msckf_jac 0.27 retri 0.0 ekf_ma 0.86 ekf_cov 0.63 ekf_k 0.31 msckf_comp_p95 2.0 |
| I_none | 25.51 / 43.41 / 69.0 | 22.4 | 22.54 / 38.15 | 2.93 | 1598.97 / 1751.9 | 1.69 / 3.9 | 50.0 | 35.7 / 36.1 | 37.6 / 41.4 | 2016000 | Awake | track 9.24 msckf_ekf 1.74 slam_ekf 2.95 msckf_comp 0.01 msckf_jac 0.38 retri 4.02 ekf_ma 2.6 ekf_cov 0.61 ekf_k 0.6 msckf_comp_p95 4.49 |
| I_none17 | 14.76 / 21.21 / 30.4 | 0.0 | 13.02 / 18.96 | 1.51 | 22.76 / 32.1 | 1.24 / 3.23 | 50.0 | 30.9 / 35.8 | 31.3 / 42.3 | 2784000 | Awake | track 4.8 msckf_ekf 1.19 slam_ekf 1.96 msckf_comp 0.01 msckf_jac 0.25 retri 2.65 ekf_ma 1.83 ekf_cov 0.42 ekf_k 0.41 msckf_comp_p95 3.04 |
| I_ref | 20.89 / 38.3 / 60.1 | 11.8 | 18.53 / 34.2 | 2.54 | 305.06 / 647.4 | 1.58 / 3.42 | 50.0 | 35.8 / 36.2 | 38.1 / 40.2 | 2016000 | Awake | tracking 6.82 propagation 0.53 msckf 2.16 slam_up 3.12 slam_delay 0.0 retri_marg 4.14 total 18.37 |

per variant (median over runs of the per-run values):
  I_exact17: runs 1 proc p50 12.52 p95 19.53 >33% 0.0 feed p50 10.9 lag p95 22.08 | pre-init feed 3.32
  I_near17: runs 1 proc p50 11.92 p95 17.24 >33% 0.0 feed p50 10.09 lag p95 19.84 | pre-init feed 3.63
  I_none: runs 1 proc p50 25.51 p95 43.41 >33% 22.4 feed p50 22.54 lag p95 1598.97 | pre-init feed 3.9
  I_none17: runs 1 proc p50 14.76 p95 21.21 >33% 0.0 feed p50 13.02 lag p95 22.76 | pre-init feed 3.23
  I_ref: runs 1 proc p50 20.89 p95 38.3 >33% 11.8 feed p50 18.53 lag p95 305.06 | pre-init feed 3.42
