# rec960 2026-11-16: 960x540 recording option + VIO 960-vs-1920 check

Branch `rec960-20261116` (from `livetest-20261115` + app commits 0.6.38d / 0.6.39 cherry-picked so the app base = the installed 0.6.39).

## A. App 0.6.41 (vc 55) `session std960` — BUILT, NOT INSTALLED, NOT RUN ON THE PHONE
- Finding: the S24 camera "0" offers **no 960x540 MediaCodec stream** (recorded meta `stream_config.mediacodec_sizes`:
  `... 1920x1080 1920x886 1440x1080 1232x1008 1088x1088 1280x720 960x720 720x480 640x480 640x360 ...`). A native ISP 960x540 stream
  is therefore impossible; 1280x720 / 640x360 are ISP-scaled streams whose crop/scaler geometry would need a new board calibration
  (10-24: the 720 stream with assumed K*2/3 drifted). So the scaling would not be "exactly x0.5" there.
- What std960 does: the SAME 1920x1080 camera stream (same session, requests, focus 0.5 dpt, exposure, preview) goes into a
  SurfaceTexture; `GlDownscale.java` draws it with GL_LINEAR at the output pixel centres into the encoder's input surface at
  960x540 = exact 2x2 box average (same as cv2.INTER_AREA 2x). Geometry: `x960 = (x1920 + 0.5)/2 - 0.5` (pixel centres at integers).
  Default 40 Mbps (= same bits per pixel as 160 Mbps at 1920). Folder `hs_<time>_std60_960`; meta `width/height 960x540`,
  `session "std"`, `session_option "std960"`, `camera_size`, `downscale` (texture matrix + pure check, buffer clock, frames drawn).
- Checks the app writes but nobody has seen yet (first real recording): `downscale.matrix_pure` must be true, `buffer_clock`
  decides the pts shift (frames_unmatched must stay ~0), `stream_config.encoder_size_offered_by_camera` false (expected),
  colour path camera YUV -> GPU RGB -> encoder YUV (luma range may differ slightly from the direct path).
- APK: `~/scanner/android/apk_backup/arlat_0.6.41_rec960.apk` (sha256 de319f6a...; same ov_bridge .so as 0.6.39, sha d5c0e585...).
  Build: `build_apk.sh` (log `apk_build.log`).

## Pipeline
- `runs/livepose_20261030/lp_replay.py`: a 960x540 recording gets a remap derived from c4f60's `remap_1920x1080.bin`
  (`(m + 0.5)/2 - 0.5`, source 960x540, written into the temp cfg; yaml/intrinsics unchanged). Checked on a 165506 frame:
  remap(1920 frame) vs remap(INTER_AREA 960 frame) phase shift 0.008 px (plain /2 would be 0.23 px).
- `scanA/run_scanner.sh`: meta 960x540 -> `--kf-res 960` automatically (keyframes 960 JPEG). The lens stays the 1920 board
  calibration and every 960 compute step already uses K x 0.5 (same as for 1920 input, so 1920 and 960 recordings are treated
  alike). Board / redscreen scale (need 1920 keyframes) refuse a 960 recording; objscan's 1920 crop does not apply.
- `live_test/live_server.py` + `live.html` + `start_live.ps1`: `-Res 960` / page "크기" select; server refuses 960 for app < 0.6.41;
  `/api/state.features` has `res960` (the launcher refuses -Res 960 on an older running server: `-Restart -NoPhone` first).

## B. Offline VIO 960 vs 1920 (laptop only)
- `make960.py` (emulation: decode 1920 -> INTER_AREA 960 -> FFV1 lossless), `b_job.sh` (laptop job), `b_launch.sh` (waits for
  the laptop, then one `remote_laptop.sh job 165506 ... --tag rec960`), `b_summary.py` (table + mesh_diff.png).
- Recording: hs_20261024_165506_std60 (std60, 0.5 dpt, board GT imu_gt_spl; the std60 recording with GT used for scanner checks).
- Results: `scan3d/runs/laptop/20261024_165506_std60/rec960/` (b_summary.json, b_table.md, mesh_diff.png, b_progress.txt).
