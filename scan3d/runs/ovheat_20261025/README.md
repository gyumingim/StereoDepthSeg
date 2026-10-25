# ovheat (= speed role) 2026-10-25 — phone replay speed results

Heat/thermal stress tests are permanently forbidden (user 16:25); this folder keeps speed results only.
All phone numbers: camera-free replay of `/data/local/tmp/replay/raw80.bin` (hs_20261021_213341, 80 s, 2327 frames),
2 threads, `--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol`, idle-spin 2 (pool) unless noted, 1 repeat each.
proc = remap+feed ms of tracking frames. Analysis: `python3 ovheat_an.py phone/out/<run>`. Raw: `phone/out/<run>/`.

## P1: idle-spin window (C4 default cfg, n_slam 50), battery (not charging)
| run | spin window | proc p50/p95/max ms | >33 ms | note |
|---|---|---|---|---|
| P_w5_r1 | 5 ms | 14.1/21.8/40.9 | 0.10 % | CONTAMINATED: ovrobust phone run overlapped 16:34:21-16:36:56 |
| P_w10_r1 | 10 ms | 13.8/19.4/32.6 | 0 | first ~37 s overlapped ovrobust (contaminated) |
| P_w15_r1 | 15 ms | 10.7/15.2/35.0 | 0.05 % | |
| P_w20_r1 | 20 ms | 8.45/12.0/23.5 | 0 | |
| P_full_r1 | full (no window) | 8.48/11.9/23.9 | 0 | 1.88 cores busy |
| P_m1w15_r1 | main-only spin (mode 1) + 15 ms | 22.7/33.0/61.6 | 4.7 % | worst |
Window >= 20 ms is as fast as full spin with fewer busy cores (1.55 vs 1.88).

## D: --drop-late (heavy cfg c4n400s100c20 = num_pts 400, max_slam 100, clones 20; over the 33 ms budget on purpose)
| run | drop-late | power | proc p50/p95 ms | lag p95/max ms | dropped | cap9 min MHz | foreground at start |
|---|---|---|---|---|---|---|---|
| D_off_r1 | off | AC charging | 28.8/51.5 | 3744/4141 | - | 1632 | launcher |
| D_off_r2 | off | battery | 22.1/37.4 | 927/9241 | - | 2208 | screen dozing |
| D_50_r1 | 50 | battery | 19.6/30.1 | 34.6/106 | 4 | 1632 | notification shade (user active in Brave just before/after) |
| D_100_r1 | 100 | battery | 23.9/41.7 | 125/171 | 80 (po.jsonl; stderr says 81) | 1824 | screen dozing |
Without drop-late the backlog never recovers (lag seconds); drop-late 50 keeps lag ~35 ms with 4 dropped frames.
D_50_r1 ran 18:18-18:20 from an orphan runner WITHOUT the team phone lock (previous instance killed); no other phone
job was logged then (unverified). D_100_r1 / D_off_r2 ran under the lock (18:27-18:30). Clock caps differ between runs
-> timing comparisons across D runs are indicative only.

## PC ATE (42 GT cells, `pc_ate/score_table.md`): --klt-pyr N
W3 (=ovopt) ate med 0.591 cm, 8 diverged cells; klt-pyr 4: 0.581, 6 diverged, paired vs W3 +0.026 cm median (12/34 better);
klt-pyr 3: 0.702 (worse). PC ms not usable for timing (shared PC). klt-pyr phone timing NOT measured.

## Excluded
S6_w15_r1 (6-min loop, heat-related; po.jsonl not committed), failed_S6_w15_r1_noexec.
