# KLT idle-gap bench on Galaxy S24 (2026-10-24 23:52 - 10-25 00:26 KST)

Question: TrackKLT is 4.4 ms back-to-back but ~18 ms when paced at 33.3 ms (30 fps). Is it the CPU going slow/idle
between frames, and does keeping cores busy fix it?

## Conclusion (first)
- **Yes: it is frequency + core placement in the idle gap.** Paced with `sleep`, the scheduler (Samsung EMS,
  governor `energy_aware`) moves the ~13 %-util thread to mid cores 4-6 at ~1.0-1.1 GHz (freq sampled every 100 ms);
  back-to-back it runs on cpu9 (X4) at 2784 MHz. Pinning to cpu9 while still sleeping does NOT help (cpu9 then idles at
  ~800 MHz, 17.3 ms) -> clock drop is the cause, not only migration.
- **T1: `spin` (main thread busy-waits) -> 3.8-4.4 ms p50** (= back-to-back). `helper` pinned to the same core as the
  pinned main thread (`--main-cpu 9 --helper-cpu 9`, nice-19 spinner) -> 4.4-5.4 ms. Unpinned helper does nothing
  (helper lands on cpu9, main on cpu4-6).
- **T2/T4: main-only spin is not enough** (19-20 / 12-14 ms) because the TBB workers still sleep on slow cores.
  `tbbwarm --warm-ms 33.3` (= all T pool threads spin the whole gap, "tbbspin") -> **T2 3.4-4.2, T4 3.7-4.7 ms**.
- Short warm-ups do not help much: prespin 3 ms 16.3, 8 ms 12.0, 15 ms 6.4 ms (T1) -> the governor needs >10 ms ramp.
- uclamp (`sched_setattr` util_min) is EPERM for the adb shell user -> not tested (likely also EPERM for the app; guess).
- npts 400: same picture (b2b 5.6 / sleep 18.6-19.2 / spin 5.2-5.6 ms at T1; tbbspin T2 4.4-4.5, T4 4.8-5.3).
- Cost: spinning keeps 1 (T1) or T cores at 100 %. 60 s runs (9 x 200 frames): sleep T1 SKIN 35.4->34.4, AP 37.1->35.6;
  spin T1 SKIN 34.4->35.9, AP 35.6->42.4; tbbspin T2 SKIN 34.7->36.2, AP 36.2->42.8; helperpin9 T1 SKIN 35.8->36.1,
  AP 38.0->41.5 (runs back to back, so each start temp depends on the previous run; 1 repeat).
- Overlap flag: the phone binary of the 23:46 run was older than the source (printed "overlap on" with env unset; the
  current source defaults off). All runs here pass `--overlap 0` explicitly (live default). With overlap 1, paced
  sleep T2 = 18.2 ms vs 23.5-24.7 ms off (T1 18.2 either way) -> the 23:46 "all T ~18 ms" was measured with overlap ON.
  Hash identical everywhere (npts 200: 08c6a9769bf93044, npts 400: 0d2c2f2bb3fcc944, all modes / T / overlap).

## Table (TrackKLT ms per feed_new_camera, 200 frames, each value = one repeat; p1 + p2b pooled)
| config | T | p50 | p95 | main-core MHz (mean of samples) |
|---|---|---|---|---|
| back-to-back | 1 | 3.7 / 3.8 | 5.6 / 5.6 | 2784 |
| sleep (live) | 1 | 17.7 / 18.2 / 18.1 | 21.3 / 21.7 / 21.4 | 1014-1129 |
| spin | 1 | 3.8 / 4.0 / 4.4 | 5.8 / 5.9 / 5.9 | 2784 |
| spin pinned cpu9 | 1 | 3.9 / 4.1 | 5.9 / 5.7 | 2784 |
| sleep pinned cpu9 | 1 | 17.2 / 17.5 | 21.0 / 20.9 | 768-807 |
| helper (unpinned) | 1 | 17.8 / 17.9 | 22.6 / 22.2 | 899-943 |
| helper + main both cpu9 | 1 | 5.4 / 4.4 / 4.6 | 7.7 / 6.4 / 6.4 | 2119-2784 |
| tbbspin (warm 33.3) | 1 | 4.6 / 4.1 | 6.4 / 6.1 | 2487-2607 |
| tbbwarm 3 ms | 1 | 16.5 / 16.7 | 19.4 / 19.5 | ~1170 |
| prespin 3 / 8 / 15 ms | 1 | 16.1,16.5 / 11.8,12.1 / 6.2,6.6 | 19.2-19.8 / 14.7 / 8.0-8.4 | |
| uclamp 1024 (EPERM = sleep) | 1 | 18.1 / 17.9 | 21.0 / 21.4 | |
| back-to-back | 2 | 4.1 / 4.2 | 5.4 / 5.5 | 2784 |
| sleep | 2 | 24.7 / 24.5 / 23.5 | 28.8 / 28.6 / 27.9 | ~1000 |
| sleep, overlap 1 | 2 | 18.2 | 30.2 | |
| spin | 2 | 19.6 / 20.1 / 19.0 | 25.4 / 25.6 / 24.3 | 1911-2431 |
| tbbspin | 2 | 4.2 / 3.4 | 5.6 / 4.7 | 2380-2784 |
| back-to-back | 4 | 3.8 / 3.8 | 5.1 / 4.9 | 2784 |
| sleep | 4 | 21.6 / 22.2 / 19.9 | 26.2 / 26.7 / 24.9 | ~860-910 |
| spin | 4 | 12.4 / 12.7 / 14.4 | 18.3 / 17.7 / 18.2 | 2196-2777 |
| tbbspin | 4 | 4.7 / 3.7 | 7.6 / 7.3 | 2119-2572 |

npts 400 (p2b, 2 repeats): b2b T1 5.6/5.6 (p95 8.5/8.4), T2 4.5/5.0, T4 5.0/5.3; sleep T1 19.2/18.6 (22.2/21.6),
T2 13.6/24.1 (26.4/29.0), T4 23.1/23.4; spin T1 5.6/5.2 (8.2/8.0), T2 20.7/20.7, T4 14.5/14.7; tbbspin T1 5.5/5.4
(8.5/8.0), T2 4.5/4.4 (6.5/6.3), T4 5.3/4.8 (8.7/8.6). Full table: `summary_p1_p2b.txt`.

60 s heat runs (p3_heat, per-rep p50 over 9 x 200 frames): sleep T1 17.9..18.6, spin T1 4.0..4.4, sleep T2 24.1..24.9,
tbbspin T2 3.1..4.1, helperpin9 T1 4.3..4.7 (slow upward drift while heating).

## Setup
- Binary: `klt_bench` built from `scan3d/ov_bridge/klt_bench.cpp` of this commit (scripts/build.sh, NDK r27c, arm64),
  pushed as `/data/local/tmp/kb0615/klt_bench_idle`. p1 ran a build without `--uclamp-min` (otherwise same code);
  p2b / p3 ran the final build (stripped sha256 prefix 6472a780e3dd10c8).
- Command per run (scripts/run.sh): `LD_LIBRARY_PATH=. ./klt_bench_idle f200.u8 960 540 200 --klt-only --overlap 0
  <ARGS from plan> --freq-log .. --frame-log ..`; ARGS per run are in `scripts/plan*.txt` and at the end of each run txt.
  Input `f200.u8` = 200 C4 960x540 frames (hs_20261024_165506_std60, from the earlier klt_bench work), no camera.
- Per run files: `<tag>.txt` (stdout+stderr, summary line, cpuidle deltas, temps), `<tag>_freq.txt` (every 100 ms:
  scaling_cur_freq of policy0/4/7/9 + cpu of the main thread), `<tag>_frames.txt` (per-frame ms, cpu before/after).
- Temperature: phase p1 used `dumpsys thermalservice | grep -m1 mName=SKIN` which is the **cached** value - it stayed at
  35.9 the whole time (stale, not usable). p2b / p3 use the "Current temperatures from HAL" SKIN and AP values.
  Gate: start only if SKIN < 36 (p2b waited many times; SKIN hovered 34.4-36.3), cool 120 s if >= 38 (never hit).
- Folders: p0_aborted (runner bug: adb ate stdin, 1 run), p2_killed (2 runs before the temp fix), p1 (2 repeats,
  stale temps), p2b (repeat 3 of sleep/spin + new modes + npts 400), p3_heat (60 s runs).
  `prior_klt_bench_phone_20261024_2350.txt` = the earlier 23:46 run (overlap on, stale binary).

## Caveats
- adb shell process sits in cpu cgroup `/` (root); the live app is in `top-app`, which EMS may treat differently
  (placement / boost). The live gain of spin modes must be re-measured in the app (assumption, not checked).
- Frequency samples are 100 ms snapshots, mostly landing in the idle gap; they show the gap clock, not the clock
  during the 4-18 ms frame.
- The phone was not idle (other system load; AP temp varied 36-47 C between runs). Repeats: 2-3 per key config.
</content>
</invoke>
