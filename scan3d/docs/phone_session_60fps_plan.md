# Next phone session: 60 fps regular mode (2026-10-23 decision)

Decisions (user, 2026-10-23):
- VIO recordings and lens calibration use the **60 fps regular session** (`session std`), the same mode as live flight.
- **Focus fixed at 0.5 dpt** (about 2 m; app 0.6.6 default). Stepped focus may come later.
- **Manual exposure**: 2 ms, ISO 500. Earlier tests: std60 used 2 ms / ISO 400, and HS auto exposure chose 4 ms / ISO 243 indoors, so 2 ms / ISO 500 gives about the same brightness with half the motion blur. Adjust on site if the image is too dark.
- VIO baseline: `config/base_nozupt` (ZUPT off). Online intrinsics stay off.

Recording script: scratchpad `rec60.sh <ip:port> "<guide>" <secs> [exp_us] [iso]`. It waits for meta.json, then force-stops the app and prints the active camera clients, which must be empty.

## Order
1. Install arlat 0.6.6 (`phone_stereo/arlat/build/arlat.apk`) and check `app_version` in meta.
2. **Lens calibration at 0.5 dpt, std60, tripod.** Use the monitor board (mirror window off). Shoot several recordings with 4 s holds per pose: tilted board, board reaching the image edges, 2–3 distances.
   - Fit them jointly, then fit each recording separately.
   - Accept only if fx agrees within ±0.3 % and the principal point within ±3 px across recordings.
3. **Flight-like VIO recordings (std60).** Each starts still for ≥ 4 s (static init) with the board in view for GT.
   - Hover-like slow motion with half-stops: 6
   - Still, then fast takeoff-like motion: 5
   - Long stills, 30–60 s: 3
   - Fast rotation, roll/pitch like a drone: 3
4. **240 fps HS comparison**: 1–2 recordings of the same fast motions, to check rolling shutter at speed. The 60 regular readout is 6.5 ms vs 2.9 ms for 240 HS.
5. Optional: measure skew per mode (720p 240/120, 1080p 120) with 5 s clips.

Phone care: cool below 36 °C between takes (recording stops above 40 °C SKIN). Never leave the camera running. Delete from the phone only recordings verified on the PC, and only when the user asks.

## Rule: every recording starts with a still (user, 2026-10-23)
- Every VIO recording begins with **at least 5 s completely still, phone resting on a surface** (not handheld), like a drone on the ground before takeoff. Only then does motion start.
- Reason: 131326 initialized while slowly rotating and diverged (docs/collapse_nozupt_20261023.md). In flight, the start is always on the ground.
- Evaluation: VIO initializes in that opening still. Recordings without one are not used for VIO verdicts.

## Ruler check of GT scale (user, 2026-10-23; no rail available)
Tape two marks on the desk 30 cm apart, measured with a ruler. Rest the phone at the first mark for 5 s with the board in view, move it by hand to the second mark, and rest there for 5 s. Compare the GT displacement between the two stills with 30 cm; the target is about 0.5 %.
This is a third independent scale reference, next to gravity (IMU) and the ruler-measured board. Takes about 1 min.

## Lens calibration shot list (user, 2026-10-23): one pose per recording
Each recording is a single tripod pose: 4 s settle, then 4 s still (8 s total). The phone does not move during a recording, so there is no blur and no rolling shutter in the calibration.
Poses, about 12 recordings:
- Board in the centre, top-left, top-right, bottom-left, bottom-right, left edge, right edge (7), at about 2 m.
- Centre and two opposite corners at about 1.5 m and 2.5 m (4).
- Board tilted about 30°, left and right (2).
The fit uses all recordings jointly. Repeatability check: fit two disjoint halves of the poses; require fx within ±0.3 % and the principal point within ±3 px.

## Heat test in the VIO capture mode (user, 2026-10-23), after the calibrations
- Record continuously in the VIO mode (std60, 0.5 dpt, 2 ms / ISO 500) with the phone in its flight position. Log the app's thermalservice temperatures (SKIN, AP, battery, throttling status) every few seconds.
- Stop rules, enforced by the script, never left running: SKIN ≥ 40 °C, throttling ≥ moderate, a frame drop, or 15 min. Then force-stop and confirm that no camera client remains.
- Report: time to 38/40 °C, frame-rate drops, throttling steps, and the room temperature (assumption: about 22–24 °C indoors).
- Later: repeat with live OpenVINS running on the phone (mode ov), which heats more, and with the drone's airflow if available.

## On-phone OpenVINS timing (2026-10-23, arlat 0.6.7)
Question: can the phone run the current baseline at 60 fps in real time? Budget 16.7 ms per frame (30 fps: 33.3 ms).
Desktop reference (same bridge and config, replay of hs_20261022_133820 at 2.22 dpt, one process alone, i.e. not the 12-way parallel bench): per-frame processing p50 6.3 ms, p95 9.7 ms, max 34 ms, 0.1 % of frames over 16.7 ms. OpenVINS tracking p50 2.1 ms, SLAM update p50 2.0 ms, remap 1920x1080 → 640x360 about 0.7 ms.
The earlier "~17 ms per frame" desktop figure most likely includes video decoding in the bench (estimate).

### What runs on the phone
- The app's `mode ov` starts the bridge as a child process: `libov_bridge.so` is the ov_bridge executable, shipped in the APK as a library. It is built from `scan3d/ov_bridge/main.cpp` plus the OpenVINS sources at `~/scanner/src/open_vins`, compiled in (rpng master 6948812, "Merge PR #530 android", clean tree). Build script: `scan3d/ov_bridge/android/build_android.sh`. OpenCV and Ceres are static, ArUco is off.
- The app talks to the bridge over stdin/stdout with the binary protocol in `ov_bridge/PROTOCOL.md`. Pose JSON lines go to the PC bench server over adb reverse 8798 (optional).
- **`ov_cam main`** (new in 0.6.7) matches the baseline:
  - Camera: logical camera "0", the same camera as the std60 recordings, YUV 1920x1080 at 60 fps (regular session). The Y plane goes to the bridge.
  - Image settings: fixed focus 0.5 dpt; manual exposure 2 ms / ISO 500; OIS, EIS, NR, edge enhancement and distortion correction off (AOSP plus the Samsung tag), as in HsRecorder.
  - Undistortion: the bridge undistorts with `--remap` (v2 file, source 1920x1080) to 640x360. These are the same fixed-point maps as the bench: a desktop replay through this path reproduced the stored final-baseline result of 133820 o0 exactly (ATE 1.43 cm, Sim3 1.21, scale 0.9588).
  - IMU: uncalibrated gyro and accel (types 16/35, values 0..2), as in the bench (`--imu-raw`).
  - Bridge arguments: "bench" (`--max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000`). There is no auto-reset or init gate, so there is only one VioManager.
- **`ov_cam lens2`** (default, unchanged): lens 2, 640x480 at 30 fps, lens0u remap, "live" bridge arguments.
- **Config.** `scan3d/ov_bridge/android/make_phone_ov_main.py` writes `scan3d/config/phone_ov_main/`: the `remap_*.bin` files plus one folder per variant. The script uses the bench's own `write_config`, with the template config/mono, try_zupt false, strict init (window 1.0 s, disparity 15 px), num_opencv_threads 2, cfg noise, num_pts 200, clones 11, the lens file `hs_1920x1080_f2.22_cal20261022.json` and the lever arm.
  - Phone-only differences: R_ci is the mean over the 15 baseline recordings (spread ≤ 0.32°); p_CinI = (4.58, 0.58, −1.14) cm; timeshift_cam_imu = +2.17 ms (median −dt). Online extrinsics and time offset stay on.
  - **Focus is an assumption.** fx is scaled from 2.22 to 0.5 dpt with the focus law (k = 0.0080 / dpt): 1303.9 → 1286.0 px (Knew fx 434.5). Interpolating the 0.37 dpt node of the focus fit instead gives 1279.9 (0.5 % lower). Timing does not depend on this; accuracy does.
  - After today's 0.5 dpt calibration, regenerate with `--lens-file <new> --no-focus-scale` and push again.
- **Per run**, the app writes `files/ov/run_<time>_main60_<cfg>/`:

  | File | Content |
  |---|---|
  | `bridge_timing.csv` | Per frame: wait, remap and OpenVINS feed ms (ov_bridge `--timing-log`) |
  | `ov_timing.txt` | OpenVINS `record_timing_information`: tracking, propagation, msckf, slam, slam delayed, marg, total (s) |
  | `frames.csv` | Every image: sensor ts, copy µs, fate (0 sent, 1 pool full, 2 queue full) |
  | `poses.csv` | Every pose line, with sensor → pose latency |
  | `temps.csv` | Battery temperature and thermal status every 2 s |
  | `meta.json` | Settings, YUV offer and min frame duration, first capture result, counts, stop reason |
  | `frame_0..3.y` | Raw 1920x1080 Y snapshots |
  | Also | Config copy, `bridge_stderr.log`, `events.jsonl` |

### Steps (WSL; `S=<ip:port>`; phone on Wi-Fi adb)
1. Build (already done, do not repeat unless the code changes): `bash scan3d/ov_bridge/android/build_android.sh bridge` then `AAR=~/scanner/android/arcore/aar bash phone_stereo/arlat/build.sh`. The APK is `phone_stereo/arlat/build/arlat.apk` (0.6.7, versionCode 13). The 0.6.6 APK is kept in `~/scanner/android/apk_backup/arlat_0.6.6.apk`.
2. Install, then check the version:
   ```
   ADB=/mnt/c/Users/a3162/platform-tools/adb.exe
   mkdir -p /mnt/c/Users/a3162/scan3d_stage && cp phone_stereo/arlat/build/arlat.apk /mnt/c/Users/a3162/scan3d_stage/
   $ADB -s $S install -r 'C:\Users\a3162\scan3d_stage\arlat.apk'
   $ADB -s $S shell dumpsys package com.scan3d.arlat | grep versionName   # -> 0.6.7-ov-timing-20261023
   $ADB -s $S shell pm grant com.scan3d.arlat android.permission.CAMERA   # if the app asks / camera error
   ```
3. Push configs: `bash scan3d/tools/ov_phone_run.sh push $S`. This copies base60, n150, n100, c8, n100c8, fps30, fps30n100c8 and both remap files to `files/ov/`. Regenerate first if the lens changed: `venv_ffs/bin/python scan3d/ov_bridge/android/make_phone_ov_main.py [--lens-file F --no-focus-scale]`.
4. Smoke test, 20 s on the desk: `bash scan3d/tools/ov_phone_run.sh run $S base60 20`.
   - Check that meta.json `yuv_size_offered` is true and `yuv_min_frame_duration_ns` ≤ 16.7e6.
   - Check that `first_result` shows exp 2 ms and focus 0.5.
   - Check that the camera measures about 60 fps and that `bridge_timing.csv` has rows.
   - Look at one snapshot: `python -c "import numpy as np,cv2;cv2.imwrite('f.png',np.fromfile('frame_0.y',np.uint8).reshape(1080,1920))"`.
5. **Main run, 130 s**: `SEG=still0:0:8,hand:8:68,still:68:130 bash scan3d/tools/ov_phone_run.sh run $S base60 130`.
   - 0–8 s: phone resting on the desk (static init).
   - 8–68 s: handheld, hover-like motion with some fast rotations.
   - 68–130 s: resting still again.
   - Say the times out loud or use a stopwatch: the segments are seconds from the first frame.
6. Cool down below 36 °C between runs (the script refuses to start above that). Then run the fallbacks below in this order, stopping once one is real time: n150 → n100 → c8 → n100c8 → fps30 → fps30n100c8.
7. The run script prints the timing report itself. To print it again: `venv_ffs/bin/python scan3d/tools/ov_phone_timing.py scan3d/runs/phone_ov_timing/run_* --seg still0:0:8,hand:8:68,still:68:130`.

### Stop rules (in `ov_phone_run.sh`; never leave a run unattended)
- Before start: SKIN ≥ 36 °C → do not start.
- During the run, polled every 5 s: SKIN ≥ 40 °C or thermal status ≥ 2 (moderate) → stop. The app also stops itself at battery ≥ 42 °C or status ≥ severe, and at `secs`.
- How it stops: HOME (onPause closes the files and writes meta.json), then `am force-stop`.
- After every stop, the script prints the active camera clients and any ov_bridge process; both must be empty. Manual check: `bash scan3d/tools/ov_phone_run.sh clean $S`.
- Do not use plain force-stop alone: it kills the app before meta.json and the buffered CSVs are written.

### What to pull, how to judge
- The script pulls the whole run dir to `scan3d/runs/phone_ov_timing/<run>/` and adds `thermal_pc.txt` (the SKIN log).
- `ov_phone_timing.py` reports, per segment, using only frames after init:
  - per-frame ms (remap + OpenVINS) p50 / p95 / max, and the share of frames over 16.7 ms and over 33.3 ms;
  - the OpenVINS split (tracking / update / marginalisation);
  - wait and latency;
  - camera missing frames, Java and bridge drops;
  - maximum temperatures.
- **Verdict.** 60 fps real time = per-frame p95 ≤ 16.7 ms with 0 drops over the whole run, including the still part (SLAM updates continue there).
  - Report the share over budget even when the verdict passes.
  - Report the measured camera fps. If the camera alone delivers < 60 fps, the bottleneck is not OpenVINS.

### Fallbacks (config sets already on the phone after step 3)
| cfg | Change vs baseline | Extra `am` arguments |
|---|---|---|
| n150 | num_pts 150 | — |
| n100 | num_pts 100 | — |
| c8 | max_clones 8 | — |
| n100c8 | num_pts 100, clones 8 | — |
| fps30 | track_frequency 30, camera 30 fps | `--ei fps 30` |
| fps30n100c8 | 30 fps, num_pts 100, clones 8 | `--ei fps 30` |
| (any) | ISP-scaled 640x360 YUV instead of 1920x1080; removes the copy + remap cost | `--es ov_size 640x360` |

- None of the fallbacks has been scored for accuracy on the bench. Only base60 is the measured baseline. Before adopting a fallback, run it on the bench (`vio_bench_hs.py --npts/--clones/--fps`).
- **ov_size 640x360 is an assumption.** It assumes the ISP's 640x360 stream is the 1080p frame scaled by 1/3 (same crop). Check it against a 1080p snapshot before trusting any accuracy from it.

### Unknowns (estimates until measured)
- Whether the S24 delivers YUV 1920x1080 at 60 fps to an ImageReader. meta.json records the offer and the min frame duration.
- Whether the YUV stream has the same field of view and distortion state as the encoder stream of the recordings. Compare the snapshot with a recording frame.
- The pipe and Java copy cost of 2 MB frames at 60 fps (about 124 MB/s). frames.csv records `copy_us`; the latency column shows any backlog.
- Which cores the bridge runs on, and how much throttling there is as the phone warms up. Compare the first and last 20 s.
