# ovdrone 2026-10-25 — drone link (VIOP v2 / MAVLink) + rolling-shutter de-rotation

Branch `ovdrone-20261025` (from `ovopt-20261025` 801254a). All features are behind flags; defaults = previous behaviour.

## Flags (all default off)
| flag | what |
|---|---|
| `--ovopt rs_derot[=US]` | gyro rolling-shutter de-rotation of new feature measurements (1 = live default 6540 us readout; negative = bottom row first) |
| `--ovopt rs_row_ppm=N` | fed-row -> sensor-row scale when no `--remap` (bridge row map otherwise) |
| `--ovopt rs_postinit` | apply rs_derot only after VIO initialisation (initialiser sees raw measurements). **Untested (no ATE run yet).** |
| `--mav-udp HOST:PORT` (+ `--mav-msg/--mav-mount/--mav-yaw-align/--mav-replay-time/--mav-log`) | MAVLink2 ODOMETRY / VISION_POSITION_ESTIMATE of each IMU-forward prediction (needs `--predict 1|2`) |
| `--viop-udp HOST:PORT` (+ `--viop-hb-hz` default 20, `--viop-log`, `--viop-replay-time`) | custom VIOP v2 link (`ov_bridge/viop_proto.h`): POSE 0x01 (92 B frame), HEARTBEAT 0x02 (28 B frame, sent from the bridge main loop, 3 STOPPING heartbeats on clean exit), TIME_REQ/RESP 0x10/0x11 |

## Identity (PC, raw80 C4, `ident_rs.sh`)
- `ident_rs.log` (16:07, build before heartbeat): harness 845/845 and raw80 2327/2327 poses 17-digit identical to the ovopt reference with flags off and with `all_exact`.
- `ident_rs_hb_postinit.log` (18:21–18:23, build 17:37 with heartbeat + rs_postinit, jobs r_ref r_mine r_mine_ex r_rs only): raw80 off and all_exact 2327/2327 identical (hash 1e58bb28d8b3b8ca); rs_derot (rs_postinit off) hash d42f241f05974d0e = same as the 16:07 build. The h_* and r_rsflip lines in that log are the OLD 16:07 files (cmp reads all files; harness stream /tmp/idlespin/pose_ab/stream.bin no longer exists).

## Phone replay L_drone2 (17:33–17:46, raw80 C4 real time, W3 = all_exact,ekf_gemm,qr_compress,ekf_chol + idle_spin 2 / 15 ms, 2 threads)
Battery discharging (status 3, AC false) all runs; SKIN start 27.9–33.0, max 33.8 °C; 0 aborted. Screen was on with KakaoTalk in the foreground (noted; not stopped).
Raw: `phone/out_L_drone2/<run>/`, summary `phone/phone_sum_L_drone2.md` (`phone_sum.py`), receivers `phone_rx/rx_{viop,mav}_phone/check.json`, prediction gaps `pred_gap.txt` (`pred_gap.py`).

| run | repeats | proc p50 / p95 ms | note |
|---|---|---|---|
| T_base | 2 | 10.7/15.2, 10.7/15.1 | W3 baseline |
| T_rs (rs_derot 6540 us) | 2 | 10.8/15.0, 10.7/14.9 | rs slot mean 0.085 / 0.084 ms, p95 0.115 / 0.113 ms per frame |
| M_pred (predict 2 @200 Hz) | 1 | 10.8/15.0 | |
| M_viop (+ VIOP UDP to PC over Tailscale) | 1 | 10.7/14.9 | 8746 POSE sent, 0 fail, 0 lost by seq; 1593 heartbeats 19.3/s, 0 lost |
| M_mav (+ MAVLink ODOMETRY UDP) | 1 | 10.7/15.2 | 8757 ODOMETRY, 82 HEARTBEAT, 164 TIMESYNC |

- Rolling-shutter correction cost ~0.085 ms/frame (≈0.8 % of proc); proc difference vs base within run-to-run noise (2 repeats each).
- Prediction output: **127 Hz actual for `--predict-hz 200`** (gap p50 6.02, p95 16.0, max 38 ms; 22 % of gaps > 10 ms). Causes: IMU at ~497 Hz (2 ms spacing) and the ">= 5 ms since last" rule gives 6 ms steps; and the bridge main loop is blocked while it processes a camera frame (synchronous `feed_measurement_camera`, ~10.7 ms p50). Phone-internal send age p50 0.17 / p95 1.75 ms; horizon (prediction ahead of VIO state) p50 27.6 / p95 39.6 ms.
- Network (Wi-Fi + Tailscale to PC): RTT min 86 ms, one-way arrival - t_pred p50 45 / p95 55 / max 357 ms; offset uncertainty ±43 ms (half min RTT), so absolute one-way numbers are uncertain. Not representative of USB to the ESP32.
- Content checks: UDP payload = bridge log (max abs diff 1e-3 VIOP int16 quantisation, 5e-6 MAVLink); gravity in world (VIOP) 9.802 up, MAVLink LOCAL_FRD -9.801 ✓.

## PC functional tests
- `viop_pc/` (16:52, loopback, feeder paused 2 s at 25 s): heartbeat gap max 2.06 s during the pause (heartbeats stop when the feed stops ✓), states 0/1/3, 3 STOPPING at exit; the byte-stream parser recovered 2314/2314 frames from a junk-injected byte stream (`viop_pc_test.log`).
- `mav_pc_rx/` (16:19): pymavlink decodes ODOMETRY/HEARTBEAT/TIMESYNC (`pymavlink_check.json`).

## Offline accuracy of rs_derot (PC, 42 GT cells, `score_B0_R294_R294F.json`, `seg_*`)
B0 (off) ATE median 0.591, R294 (2.94 ms) 0.523, R294F (wrong sign) 0.594 — but divergent cells B0 8 / R294 11 / R294F 10 (R294 newly diverges on 134550_o0/o1, 221925_o1). Not adopted. `rs_postinit` was added to test whether the new divergences come from initialisation; **not run yet**.

## Phone IMU sender (VIOP_T_IMU 0x06, esp32fc proposal) — NOT implemented, spec for integration
Why not in ov_bridge behind a flag: the bridge main loop is single-threaded and processes each camera frame synchronously (feed ~10.7 ms p50, ~15 ms p95, up to ~25 ms at start), so IMU frames sent from it would leave 10–25 ms holes; the esp32fc `imu_stream` treats > 10 ms without a sample as IMU failure. Also the bridge child process cannot open the USB device (no UsbManager permission).
Spec (app side, `com.scan3d.arlat`, intent extra `viop_imu` default 0):
1. Separate `SensorEventListener` on its own `HandlerThread` (not the VIO feeder thread), ACCELEROMETER + GYROSCOPE, `SENSOR_DELAY_FASTEST`, `maxReportLatencyUs = 0`.
2. On every GYROSCOPE event: pair with the newest accelerometer sample (|dt| ≤ 5 ms, else hold it and send at the next accel event), build one frame n = 1: payload 26 B = u64 t0_us (`event.timestamp / 1000`, CLOCK_BOOTTIME = POSE timebase), u8 n = 1, u8 flags (0, or `VIOP_IMUF_UNCAL` if the uncalibrated sensors are used), u16 sample_seq0 (+1 per sample), u16 dt_us = 0, i16 gyro[3] = round(rad/s / 1e-3), i16 acc[3] = round(m/s² / 5e-3), saturated. Axes: Android sensor axes unchanged.
3. Frame: `A5 5A`, header per `viop_proto.h` (type 0x06, src PHONE, own u16 frame seq), CRC-16/CCITT-FALSE over bytes [2, 8+len) → 36 B. Write it directly to the CDC bulk-OUT endpoint from that thread (the `ViopUsbForwarder` connection, shared with a lock; bulkTransfer timeout ≤ 2 ms, count failures, never block the sensor thread longer).
4. Bias correction (`VIOP_IMUF_BIAS_CORR`) only later, if the bridge exports its bias estimate; first version sends raw.
5. Log per run: frames sent, write failures, max write time, sample-to-write latency p50/p95 (meta.json `viop_imu`).
6. Merge `viop_fc_ext.h` 0x06 definitions into `viop_proto.h` (same ids/layouts); add 0x06 to `viop_rx.py` for the PC check.
Test plan when hardware exists: ESP32 echo/timestamp test of 500 Hz × 36 B over USB CDC, gap histogram on the ESP32 side.

## Other files
`ViopUsbForwarder.java.txt` = design skeleton for the phone → ESP32 USB CDC forwarder (not compiled, not in the app). `viop_c_test.c` = C header test. `rowscale.py/.txt` = fed→sensor row scale.
