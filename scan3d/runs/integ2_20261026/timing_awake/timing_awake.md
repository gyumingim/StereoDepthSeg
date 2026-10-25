# timing0626 awake replay A/B (made 2026-10-26 06:54, input out_W1)

timing0626: awake phone replay A/B (N=new rec20261025 profile, O=old ovopt). python3 tw_an.py OUT.md RUNDIR...
Per run: feed_ms (tl.csv, init==1) p50/p95/mean/max/>33; SLAM update = slam_jac+slam_ekf and slam_ekf alone (prof.csv, frames n_slam>0) p50/p95;
suspend check: rel rx lag = (rx_boot_ns/1e6 - t*1e3) - min over run; 'persist' = max over run of the 5 s rolling MIN of rel lag
(transient backlog after slow frames is removed by the rolling min; a suspend leaves a permanent step) -> VALID if persist < 200 ms,
all samp.txt wake=Awake, foreground only NotificationShade/launcher (no user app; fg sampled every 5 s from W1b on, start/end only before), midrun_not_awake=0, not aborted. Wall = rt_end-rt_start (s). Pairs in list order (ABBA): N_rk vs O_rk.

| run | start | frames (init) | feed p50 | p95 | mean | max | >33 | SLAM upd p50 | p95 | slam_ekf p50 | p95 | lag persist ms | lag max ms | wall s | wake | fg (start/samples/end) | cpu9 cap start/med/min/max/end | SKIN start/max | AP max | battery | valid |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| w_N_r1 | 06:31:55 | 2327 (2003) | 7.78 | 11.49 | 7.89 | 20.1 | 0 | 0.92 | 0.99 | 0.63 | 0.67 | 2 | 52 | 85.0 | Awake | NotificationShade | 2304/2304/2304/2304/2304 | 31.1/34.8 | 43.5 | AC st4 T29.8 | yes |
| w_O_r1 | 06:34:21 | 2327 (2003) | 8.13 | 11.75 | 8.24 | 19.3 | 0 | 1.30 | 1.47 | 1.01 | 1.14 | 2 | 25 | 85.0 | Awake | NotificationShade | 2304/2304/2304/2304/2112 | 33.4/36.2 | 45.0 | AC st4 T31.9 | yes |
| w_O_r2 | 06:51:50 | 1434 (1110) | 8.77 | 13.29 | 9.62 | 159.7 | 7 | 1.29 | 1.72 | 1.00 | 1.36 | 1 | 131 | 54.7 | Awake,Dozing | NotificationShade,com.nhn.android.blog | 2304/2304/1824/2592/1824 | 35.0/37.5 | 50.9 | bat st3 T33.8 | NO |

## paired N - O (pair k = w_N_rk vs w_O_rk, adjacent in ABBA; only pairs where both runs valid)

| metric | per-pair N-O (ms) | mean | sd | 95% CI (t, n-1) | N mean / O mean | rel |
|---|---|---|---|---|---|---|
| frame p50 | -0.36 | -0.36 | nan | [+nan, +nan] | 7.78 / 8.13 | -4% |
| frame p95 | -0.25 | -0.25 | nan | [+nan, +nan] | 11.49 / 11.75 | -2% |
| frame mean | -0.35 | -0.35 | nan | [+nan, +nan] | 7.89 / 8.24 | -4% |
| SLAM upd p50 | -0.38 | -0.38 | nan | [+nan, +nan] | 0.92 / 1.30 | -29% |
| SLAM upd p95 | -0.47 | -0.47 | nan | [+nan, +nan] | 0.99 / 1.47 | -32% |
| slam_ekf p50 | -0.38 | -0.38 | nan | [+nan, +nan] | 0.63 / 1.01 | -37% |
| slam_ekf p95 | -0.47 | -0.47 | nan | [+nan, +nan] | 0.67 / 1.14 | -41% |

valid pairs: 1 of 1: [('w_N_r1', 'w_O_r1')]
