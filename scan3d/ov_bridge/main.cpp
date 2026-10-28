// ov_bridge — OpenVINS (ROS 없음) VioManager 를 바이너리 레코드로 구동하는 작은 실행 파일.
// 프로토콜: PROTOCOL.md. 입력 = stdin 또는 TCP, 출력 = 같은 채널로 JSON 한 줄씩.
//
//   ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]
//             [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]
//             [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]
//             [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG] [--timing-log FILE] [--cpus LIST] [--threads N] [--klt-overlap 0|1]
//             [--idle-spin 0|1|2|3] [--idle-spin-ms W] [--predict 0|1|2] [--predict-hz HZ] [--ovopt LIST]
//             [--prof-log FILE] [--pose-precision N] [--drop-late MS] [--pred-thread 0|1] [--imu-log FILE]
//             [--drop-late-mode 0|1] [--drop-late-cap N] [--drop-late-win S] [--pred-stale-ms MS]
//             [--mav-udp HOST:PORT [--mav-msg odom|vpe|both] [--mav-mount PRESET|R9] [--mav-yaw-align 0|1]
//             [--mav-replay-time 0|1] [--mav-sysid N] [--mav-log FILE]]
//
// --mav-udp HOST:PORT (2026-10-25, branch ovdrone-20261025, drone output prototype; default off = nothing sent, output
//   unchanged): every --predict line (needs --predict 1|2) is also sent as MAVLink 2 ODOMETRY (#331, LOCAL_FRD / BODY_FRD,
//   VIO estimator, reset_counter, quality) and/or VISION_POSITION_ESTIMATE (#102) in UDP datagrams to HOST:PORT, plus a
//   1 Hz HEARTBEAT and immediate TIMESYNC replies (clock offset for the receiver). Frames, mount presets (imu, cam_fwd,
//   cam_down or 9 numbers R_body<-imu), covariance, time base: mav_out.h header and runs/ovdrone_20261025/README.md.
//   --mav-log FILE = one CSV row per sent prediction (incl. send age and the body-frame accelerometer for the gravity
//   check). stderr "[ov_bridge] mav: ..." (frame alignment per (re)init, end summary with send-age percentiles).
// --viop-udp HOST:PORT (2026-10-25, ovdrone; default off): instead of MAVLink, the custom VIOP v2 link of viop_proto.h for
//   the ESP32-S3 flight firmware: one POSE frame (92 B) per prediction + HEARTBEAT frames (--viop-hb-hz, default 20, sent
//   from the main input loop on IMU records, so they stop when the feed stops / the bridge hangs; 3 "stopping"
//   heartbeats on a clean exit) + TIME_RESP replies. --viop-log FILE, --viop-replay-time 0|1 as for MAVLink.
// --viop-hb-src 0|1 (esp32fix-20261026; default 0 = as above, unchanged): 1 = heartbeat from a dedicated thread on a
//   steady 1/HZ grid; a beat is skipped while no IMU record was received for 100 ms or the main loop handled no IMU
//   record for 200 ms (feed stopped / bridge hung -> heartbeats stop as before). See mav_out.h hb_run.
//
// integ-20261025 (all default off = unchanged):
// --profile rec20261025: named live profile = --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5 --idle-spin 2
//   (integ2-20261026 added sym_inplace,ekf_sparse; user 2026-10-26 morning added gyro_klt=5 (CHANGES poses; profile then refuses
//   --pipe 1); the integ-20261025 profile = add --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol; profile without gyro = add
//   --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse)
//   --idle-spin-ms 20 --drop-late 50 --health 1 --predict 2 --predict-hz 200 --pred-thread 1 (stdio only; user 19:40) (team measurements 2026-10-25, runs/integ_20261025/README.md). Applied before the
//   other arguments, so any explicit flag overrides its part. Without --profile nothing changes.
//   integ2 0.6.28 (user 2026-10-26, 60 fps only): profile rec60 also sets --loop 1 (rec20261025 unchanged) (BoW loop closure; VIO/control fields unchanged, adds
//   "map" + MAP_ODOM / DEBUG_VECT). Vocabulary = --loop-vocab, else voc_k10L4.voc next to the config / one dir above /
//   this build's source tree (PC); none found -> rc 2. --loop 0 = profile without loop; with --recover the loop is dropped.
// --pred-thread 1 with --mav-udp / --viop-udp: the prediction thread also sends each of its predictions (frame + 1/HZ
//   grid of IMU time) as MAVLink / VIOP POSE (before this, that combination sent nothing). The snapshot then also carries
//   the 9x9 marginal covariance (q, p, v) and the SLAM feature count. Without --pred-thread the old main-loop path
//   (predict(): emits only while the input is idle, >= 1/HZ after the last one -> ~127 Hz at 200 on the phone) is unchanged.
// OV_KLT_PYR from the environment is ignored (removed + logged) unless --klt-pyr N is given.
//             [--init-still 0|1] [--health 0|1] [--health-log FILE] [--health-cfg K=V,..]
//             [--klt-stats 0|1] (2026-10-26 ovgyro, output only: pose line "klt":[in, LK ok, kept, mean predicted
//             shift px, fallback tried, fallback ok, rotation 0|1, mean |LK result - start| px, mean |LK result - old|
//             px]); --ovopt gyro_klt=1|2 = gyro-aided KLT start points
//             (CHANGES results, default off; see ovpatch/gyro_klt.h)
//             [--recover N] [--recover-max-gap S] [--recover-min-feats N] [--test-blank T0:D,..]
//
// ---- 2026-10-25, branch ovrobust-20261025 (all default off = output and code path unchanged) ----
// --init-still 0|1 (NOT RECOMMENDED, 2026-10-25 offline 37/45 cells divergent): static initialization while still. Upstream OpenVINS waits for "still, then a jerk" whenever ZUPT
//   is off (VioManagerHelper: wait_for_jerk = updaterZUPT == nullptr), so the phone has to be moved to initialize.
//   1 = ovpatch/VioManagerHelper.cpp calls the same static initializer with wait_for_jerk = false: it initializes as
//   soon as the last init_window_time of IMU is still (accel std < init_imu_thresh in both halves, disparity <
//   init_max_disparity) -> gravity direction, gyro bias, accel bias, v = 0. ZUPT stays OFF: nothing ties the state
//   to "still" afterwards (this is NOT a zero-velocity update; only the initial state and its moment change).
// --health 0|1: per fed frame a health record "h":{...} in the pose line; --health-log FILE: the same as one CSV row per
//   fed frame (header = names). Fields: tracked (KLT ids on cam0), slam / msckf (as the pose line), msckf_in / _tri /
//   _used, slam_in / _clean / _used, delay_in / _ok (ovpatch health counters: features into each update, after
//   triangulation / cleaning, after the chi2 test), chi2_rej (= msckf_tri - msckf_used + slam_clean - slam_used),
//   pos_std / vel_std (m, m/s: sqrt of the trace of the IMU position / velocity marginal covariance), ori_std_deg,
//   since_upd (s since the last frame whose update used >= 1 feature; since init when none yet), state
//   INIT / OK / DEGRADED / LOST and lost_run (consecutive LOST frames). Read only: poses identical with / without.
//   Rules (--health-cfg overrides, defaults in HealthCfg): LOST = not finite or |v| > --max-speed, or since_upd >
//   upd_lost (1.0 s), or pos_std > std_lost (1.0 m), or tracked < trk_lost (10) for >= trk_lost_frames (15) frames;
//   DEGRADED = since_upd > upd_deg (0.2 s), pos_std > std_deg (0.25 m), tracked < trk_deg (40), or chi2 rejected >
//   rej_deg (0.5) of >= 10 candidates; else OK. Before init: init_ready = 1 when the last init_window_time of IMU is
//   still (accel std < init_imu_thresh in both halves, like the static initializer) and tracked >= 15: the upstream
//   "still, then jerk" init will initialize at the next motion (e.g. takeoff) - a go signal, no algorithm change. stderr "[ov_bridge] health ..." summary at the end.
// --recover N (default 0 = off): automatic recovery after N consecutive LOST frames of a VioManager that has done at
//   least one feature update (startup / still-after-init LOST never triggers it):
//   (policy v2 2026-10-25 after the blank test: wait until vision has been back for N frames with still no update;
//   full reset only with --recover-full 1; injected VioManager tracks num_pts, not init_max_features)
//   a) vision back for N frames (tracked >= --recover-min-feats, default 20), state finite, |v| <= --max-speed, since_upd <=
//      --recover-max-gap (default 3 s): new VioManager started from the old state (same injection as --reinit-state:
//      pose, velocity, biases, extrinsics, time offset, inflated covariance) -> same world frame, no jump;
//      line {"type":"reset","auto":true,"reason":"recover","reinit":"state",...}.
//   b) (--recover-full 1 only) since_upd > --recover-max-gap or the state is bad: new VioManager with a normal (static) initialization; once
//      it is initialized its poses are mapped onto the last pose output before the reset with a 4-DoF offset (yaw +
//      translation; gravity is common) -> the output stays one continuous frame ("seg" counts the segments, pose
//      lines carry "seg" and "frame_off":[yaw_rad,x,y,z] while an offset is active); line reason "recover",
//      "reinit":"full". c) otherwise wait (vision not back yet). stderr "[ov_bridge] recover ..." per event.
// --test-blank T0:D,... (TEST ONLY, default off): cam0 image replaced by black for D s starting T0 s after the first
//   camera record (IMU unchanged) -> synthetic loss of vision for recovery tests. Pose lines get "blank":true.
//
// --idle-spin 3 / --idle-spin-ms W (2026-10-25, ovopt, heat; both default off = unchanged): 3 = like 2 (whole pool
//   waits busy) but between the input polls every stripe waits for an event (aarch64 LDAXR + WFE on the shared "go"
//   word: woken by the stripe that sees input, by interrupts or by the ~100 us timer event stream) instead of a yield
//   loop -> the threads never sleep (scheduler sees them busy, keeps the clocks; guess) while the cores idle clock-gated.
//   On x86 3 == 2. W > 0 (modes 1-3): spin only within W ms before the next camera record is expected (header time of
//   the last camera record + EMA of the camera header gaps); earlier the wait blocks in poll() (IMU records still wake
//   it). Prior bench (klt_idle_bench_20261024): a 3-15 ms warm-up gave T1 KLT 16 / 12 / 6.4 ms vs 4 ms full spin, so
//   W must be large; the heat/time trade is a phone measurement. stderr idle spin line + "window .. blocked ..".
// --ovopt LIST (2026-10-25, branch ovopt-20261025): speed options of the patched OpenVINS copies (ovpatch/ovopt.h) and
//   of this bridge, comma separated, all off by default (= output and code path unchanged). Exact (pose lines
//   identical on the PC identity runs at 9 and 17 digits; phone replay check pending): skip_retri, par_tri, par_jac,
//   par_ma, par_s, cov_move, frame_nocopy, givens_rm; "all_exact" = all of them. Near-exact (other rounding, same math):
//   ekf_gemm, qr_compress (replaces givens_rm), ekf_chol, par_cov. "name=0" turns one off.
//   Changes results (2026-10-25, branch ovdrone-20261025, default off): rs_derot[=US] = gyro rolling-shutter de-rotation
//   of every new feature measurement to the frame-centre row time (ovpatch/VioManager.cpp; US = sensor readout first->
//   last row in microseconds, signed, plain "rs_derot" = 6540 = S24 live 1920x1080 @30 skew). With a v2 --remap file the
//   exact sensor row of each feature comes from the remap y table; without one rs_row_ppm=N (row scale x 1e6, 0 = 1)
//   maps fed rows to sensor rows. stderr "[ov_bridge] rs_derot: ..." at the end (mean / max shift px, mean gyro).
// --pose-precision N (2026-10-25, default 9 = unchanged): significant digits of the numbers in the pose / pred lines
//   (17 = round-trip exact doubles, for identity checks).
// --drop-late MS (2026-10-25, default 0 = off; CHANGES results when it drops): after initialisation a frame is not fed
//   (its IMU is) when its feed lag (CLOCK_BOOTTIME now - frame t_ns) exceeds the smallest lag seen in this run by more
//   than MS -> the backlog cannot grow once processing is slower than the frames (phone "cliff", 2026-10-25 replay).
//   Pose lines get "late_dropped" (only with the flag), stderr "[ov_bridge] drop-late ...".
// --drop-late-mode 0|1 (2026-10-26, branch droplatefix-20261026; only matters with --drop-late > 0; default 1):
//   0 = the 2026-10-25 rule above (reference = smallest lag of the whole run, never re-anchored, no limit on consecutive
//       drops). BUG kept only for A/B: if the clock offset / feed latency rises by more than MS for good (phone replay
//       P_r2: device suspend let CLOCK_BOOTTIME run ahead of the feeder's CLOCK_MONOTONIC; live: a lasting latency rise)
//       EVERY later frame is dropped -> IMU-only for the rest of the run while health stays OK and the prediction
//       thread keeps sending "tracking" (P_r2: horizon 69 s, |v| 127 m/s).
//   1 = fixed rule (default whenever --drop-late is given, incl. --profile rec20261025 / rec60):
//       a) while the lag of the late frames is flat (range over the last S s <= MS = drops do not reduce it) at most
//          --drop-late-cap N (default 3) frames in a row are dropped, the next late frame is fed anyway ("forced"); while
//          the lag moves (backlog draining / overload / drift) frames are dropped as in mode 0 (no cap);
//       b) re-anchor: when every frame of the last --drop-late-win S (default 3.0 s of frame time) was late (dropped or
//          forced), the reference becomes the smallest lag of that late run (stderr "drop-late: re-anchor ..."). A real
//          backlog is drained by the drops (lag falls back under the limit -> the run ends), so only a lasting offset
//          shift survives S seconds of drops;
//       c) --health: a frame fed while it is late (forced) or within a late run of > N frames is at least DEGRADED;
//          h gets "late_run" (consecutive late frames, mode 1 only);
//          Re-anchor only when the lag over that window is flat (max - min <= MS; verifier 2026-10-26): a lag that is
//          still rising after S s (overload backlog the capped drops cannot drain, or drifting clocks) is NOT re-anchored;
//          after 2 S of such a run "late-lost": pose/pred status lost, MAVLink/VIOP lost, health LOST, until a frame is
//          on time again (stderr "drop-late: ... late-lost");
//       d) --pred-stale-ms defaults to 500 (see there).
// --test-feed-sleep MS:T0 (TEST ONLY, default off): sleep MS before every fed frame from T0 s after the first camera
//   record (overload simulation for the drop-late tests).
//   stderr summary adds forced / re-anchors / max late run. Pose lines unchanged except "late_dropped" (as before).
// --pred-stale-ms MS (2026-10-26, droplatefix; default 0 = off, but 500 when --drop-late > 0 with mode 1; explicit value
//   wins): a prediction whose horizon h (newest IMU - filter state time) exceeds MS is marked lost: pred line "status":
//   "lost" + "stale":true, MAVLink quality 0 / system_status CRITICAL, VIOP POSE without VIOP_F_TRACKING (quality 0) and
//   VIOP heartbeat state LOST (main loop: same horizon test) -> the ESP32 (link_task.c: good = VALID && TRACKING &&
//   quality > 0) stops trusting the extrapolated pose. Same protocol, nothing new on the wire. Normal live h ~0.06 s.
//   frame_nocopy (bridge): a frame that is remapped is read straight from the received record buffer (no 2 MB copy
//   first). Needs a build with the OpenVINS sources compiled in (phone build, PC -DOV_FULL_SOURCE=ON); the PC build
//   that links libov_msckf_lib.so rejects --ovopt (except frame_nocopy / prof). stderr "[ov_bridge] ovopt ...",
//   ready line "ovopt".
// --prof-log FILE (2026-10-25): per fed frame one CSV row of OpenVINS stage times (ms) + counters from the ovpatch
//   timers (ovopt prof): t_ns,t,feed_ms,init,n_slam,n_msckf,<stages...>,<counters...> (header = names). Timing only.
//
// --predict MODE (2026-10-25, arlat 0.6.17 intent predict): IMU forward-predicted "now" pose. The camera frame reaches
//   the bridge ~40 ms after its exposure (HAL/ISP) and takes ~20 ms to process, while the IMU (~500 Hz) keeps arriving.
//   After a frame update the bridge integrates the latest filter IMU state (q, p, v, biases, IMU intrinsics, dt_cam_imu)
//   with every IMU sample newer than the state time (fed + still held ones) up to the newest received IMU sample, the
//   same mean equations as OpenVINS Propagator::fast_state_propagate (no covariance). It only READS the state (own copy
//   of the numbers): the filter, the record order and the normal "pose" lines are identical to --predict 0.
//   0 = off (default, output unchanged).
//   1 = one {"type":"pred",...} line per frame update, emitted after the frame's pose line as soon as the input pipe has
//       no more data waiting (= after the IMU records that queued up during the frame were read), so it uses all IMU
//       that has arrived.
//   2 = 1 + also between frames: whenever the input is idle and the newest IMU is >= 1/HZ s past the last prediction
//       (--predict-hz, default 100), another "pred" line (src "rate").
//   pred line: t_ns = predicted time on the IMU clock (sensor ns, = the newest IMU sample), t = the same instant on the
//   pose-line time axis (s, t_state axis: t_imu - dt_cam_imu), t_cam_ns = t_ns of the last frame pose, t_state, h =
//   horizon (s, IMU time - state time), n_imu, src "frame"|"rate", status, p, q (Hamilton xyzw world<-IMU), v,
//   T_world_cam0, q_GtoI_jpl. Only while initialised. stderr "[ov_bridge] predict ..." at the end (count, horizon).
// --pred-thread 1 (2026-10-25, arlat 0.6.19 intent pred_thread; default 0 = everything above unchanged; needs
//   --predict 1|2, stdio only): an input reader thread reads every record from stdin into an in-memory queue (the main
//   thread consumes it in the same order -> the VIO sees the identical record sequence; the reader holds at most 2
//   unprocessed image records, then it stops reading = pipe back-pressure as before) and hands every IMU sample at once
//   to a prediction thread. After each frame the main thread copies the filter IMU state (+ calibration) into a
//   snapshot under a mutex; the prediction thread propagates the latest snapshot with all received IMU newer than it
//   (same equations as predict above). So predictions continue while a frame is being processed (mode 2: whenever the
//   newest IMU crosses the next 1/HZ grid point of IMU time -> mean rate = HZ; mode 1 and 2: one "frame" line right
//   after each frame's pose line). The filter is not touched by the thread. pred lines get "tb" (CLOCK_BOOTTIME ns
//   when the line was formed). stderr "[ov_bridge] pred thread ..." at the end (lines, compute us, thread CPU s).
// --imu-log FILE (2026-10-25, arlat 0.6.19 intent imu_log; timing only): one CSV row per received IMU record
//   t_ns,rx_hdr_boot_ns,rx_boot_ns (CLOCK_BOOTTIME when its header / payload was read).
// Chunked image records (2026-10-25, arlat 0.6.19 intent imu_prio): types 5 (CHUNK) and 6 (CHUNK_END) carry consecutive
//   pieces of one type-2 STEREO payload; other records (IMU) may come between the pieces. CHUNK_END completes the
//   payload, which is then handled exactly like a type-2 record (rx_hdr = header of the first piece). Clients that never
//   send 5/6 see no change.
//
// --idle-spin MODE (2026-10-25, arlat 0.6.16 intent idle_spin): what the bridge does while it waits for the next input
//   record (between frames / IMU samples). klt_bench 2026-10-24 (S24, scan3d/runs/klt_idle_bench_20261024): with a 29 ms
//   sleep between frames the cores clock down and KLT takes 18-24 ms instead of ~4 ms back-to-back.
//   0 = block in read() (default, unchanged behaviour).
//   1 = "main spin": the main thread busy-polls the input fd (poll(fd, 0) loop) instead of blocking (= bench "spin").
//   2 = "pool spin": mode 1 + the whole OpenCV/TBB pool spins: cv::parallel_for_ over T = cv::getNumThreads() stripes,
//       every stripe polls the fd and all leave as soon as one sees input (= bench "tbbwarm" with the whole gap).
//   Both spin at most kIdleSpinMaxMs (200 ms) per wait, then fall back to a blocking poll (no burning while the app
//   pauses the feed, e.g. before the camera starts). Only the waiting changes: records, order and all VIO results are
//   identical to mode 0. stderr "[ov_bridge] idle spin ..." (start, cam300, end: waits / spun / timeouts / exit
//   latency), ready line "idle_spin" / "idle_spin_stripes". Exit latency (mode 2) = first stripe saw input ->
//   parallel_for_ returned on the main thread (= extra delay before the record is read), --timing-log spin_exit_us.
//   Costs heat (bench: +1.5 C SKIN / +7 C AP after 1 min).
// --timing-log extra columns (0.6.16, appended): rx_hdr_boot_ns / rx_boot_ns = clock_gettime(CLOCK_BOOTTIME) when this
//   camera record's header / whole payload was read (same clock as the app's elapsedRealtimeNanos t_arrive),
//   spin_wait_us = time spent spinning before the header (0 = data was already there, -1 = mode 0),
//   spin_exit_us = mode 2 exit latency of that wait (-1 = none).
//
// --threads N (2026-10-24, arlat 0.6.15 intent ov_threads): OpenCV thread pool size (cv::setNumThreads) = overrides the
//   config num_opencv_threads (N > 0; 0 / absent = config value). This pool runs every parallel part of a frame: the
//   --remap (cv::remap splits the output rows into bands over the pool; bit-identical for any N), the KLT pyramid
//   (pyrDown rows), LK (points), the FAST grid cells (OpenVINS Grider_GRID) and the KLT overlap below.
//   The phone OpenCV (Android SDK 4.10.0, static) is built with TBB, so N threads are real (checked with klt_bench).
//   stderr "[ov_bridge] opencv threads ..." + ready line "cv_threads" / "cv_parallel" / "klt_overlap".
// --klt-overlap 0|1 (2026-10-24, default 0): sets OV_KLT_OVERLAP for the patched TrackKLT (ovpatch/TrackKLT.cpp: FAST
//   top-off on the last frame runs in parallel with the current frame's histogram + pyramid; identical output).
//   0 = upstream TrackKLT code path (default: on the S24 the overlap was not faster, klt_bench 2026-10-24).
// --klt-pyr N (2026-10-25, ovheat, default 0 = off = upstream 5 levels): KLT pyramid levels (env OV_KLT_PYR for the
//   patched TrackKLT). CHANGES results. Speed/accuracy study: runs/ovheat_20261025/README.md.
//
// --cpus LIST (2026-10-24, arlat 0.6.14 intent cpus): e.g. 7,8,9 or 7-9. sched_setaffinity at the very start of main (before
//   OpenVINS / OpenCV create any thread, so all later threads inherit it) and again for every /proc/self/task thread;
//   re-applied + logged at the 300th camera record and at the end (Android cpuset changes can reset it).
//   stderr: "[ov_bridge] affinity <tag>: ..." with sched_getaffinity per thread (tid comm cpus). No --cpus = unchanged.
//
// --timing-log FILE (2026-10-23, on-phone timing): one CSV row per camera frame fed to OpenVINS:
//   t_ns,t,wait_ms,remap_ms,feed_ms,init,n_slam,n_msckf,cam_dropped,cam_queue
//   wait_ms = record received -> feed_measurement_camera start (IMU wait + backlog), remap_ms = --remap undistort,
//   feed_ms = feed_measurement_camera (OpenVINS track + propagate + update + marg = the "ms" of the pose line).
// --remap v2 (2026-10-23): a remap file may end with int32 src_w, src_h; then frames of exactly that size are remapped
//   to the map size (e.g. 1920x1080 raw -> 640x360 undistorted). Files without it: unchanged (map size = input size).
//
// --init-gate (2026-10-18, 10-18 오전 "그림자" 방식으로 바꿈): 기본 VioManager (원래 설정, 시차 기준 그대로) 옆에
// init_max_disparity 를 DISP_DEG 각도 (cam0 fx 로 화소 환산) 로 넓힌 "그림자" VioManager 를 하나 더 돌린다 (초기화 전에만).
//  - 그림자는 카메라 시작 뒤 0.8 s 동안의 프레임은 안 받는다 (시작 직후 흔들림·AE 로 곧바로 초기화되는 것 막기).
//  - 그림자의 정지 초기화가 끝나면 그 창 init_window_time(+0.05 s) 의 자이로가 조용했는지 (평균 각속도 벡터 크기 <= MEAN,
//    순간 최대 <= MAX) 확인해 아니면 그림자만 새로 만든다.
//  - 기본 쪽이 정지 초기화를 하면 그림자는 버린다 (= 예전과 완전히 같은 결과, 참조 녹화 그대로).
//  - 기본 쪽이 초기화 못 한 채로 폰이 움직이기 시작하면 (최근 0.1 s 자이로 > MAX, 또는 그림자가 이미 갱신 시작) 확인된 그림자로
//    바꿔 쓴다 ("init_gate":"shadow" 포즈 줄 + stderr 로그).
// 예전 방식 (기본 쪽 기준을 바로 넓히고 버리기) 은 175446 의 초기화 순간을 1 프레임 바꿔 궤적이 크게 달라졌다 (키프레임 142 -> 111):
// 이 녹화는 초기화 순간에 매우 민감하다. 회전은 자이로가 직접 막으므로 (바이어스 오차 <= MEAN) 영상 쪽 기준은 흐린 장면 KLT 잡음·가까운 물체 시차만큼
// 여유를 줄 수 있다. 근거: rec_20261018_002349 는 자이로 평균 0.001 rad/s 로 멈춰 있었는데 흐린 근접 장면이라 시차
// 3.95~6.6 px (> 3.09) 로 초기화가 늦어짐. 예전 문제 (천천히 도는 중 2.6~14.8 px 에서 초기화 → 바이어스 0.016~0.035 rad/s) 는
// 평균 각속도 기준으로 막힌다. 그림자 초기화를 버릴 때 ("moving"|"short") 는 stderr 로그만 남는다.
//
// --reinit-state (2026-10-17): lost(특징 끊김) auto-reset 때 정지 초기화 대신 옛 상태를 새 VioManager 에 넣어 같은 월드로 이어 감
// ({"type":"reset","auto":true,"reason":"lost","reinit":"state",...}). PROTOCOL.md 참고.
//
// 방어: OpenVINS 호출에서 난 예외(cv::Exception 포함)는 잡아서 VioManager 를 새로 만들고
// {"type":"reset","auto":true,"reason":"exception"} 를 보낸다 (프로세스는 계속 산다).
// use_stereo=true 설정이면 2장짜리 STEREO 레코드만 받는다 (mono 섞임 = TrackKLT levels2 단언 원인).
//
// stdio 모드에서는 OpenVINS 로그(printf -> fd 1)를 stderr 로 돌리고, 결과만 원래 stdout 으로 쓴다.

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <dirent.h>
#include <sched.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ovopt.h"  // ovpatch/: --ovopt flags + profiler (2026-10-25)
#include "gyro_klt.h"  // ovpatch/: gyro-aided KLT counters (2026-10-26 ovgyro)
#include "mav_out.h"  // --mav-udp drone output (2026-10-25, ovdrone)
#include "ovloop.h"  // loop/: --loop loop closure (2026-10-26, ovloop)

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "types/Landmark.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "track/TrackBase.h"
#include "cam/CamEqui.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace {

constexpr uint32_t kMagic = 0x4F564252u;  // "OVBR"
enum MsgType : uint32_t { kImu = 1, kStereo = 2, kReset = 3, kQuit = 4, kChunk = 5, kChunkEnd = 6 };
constexpr uint32_t kEof = 0xFFFFFFF0u, kBad = 0xFFFFFFF1u;  // --pred-thread reader end markers (never on the wire)
constexpr uint32_t kMaxPayload = 64u * 1024u * 1024u;

struct Args {
  std::string config;
  bool tcp = false;
  std::string host = "127.0.0.1";
  int port = 7600;
  size_t max_cam_queue = 3;
  double max_speed = 5.0;   // m/s 넘으면 lost (손에 든 폰 기준)
  int lost_frames = 15;     // 특징점 0 이 연속 N 프레임이면 lost
  int startup_lost_frames = 45;  // 초기화 뒤 첫 특징 갱신 전에는 이만큼 (15 fps 에서 3 s) 기다린 뒤 lost
  bool auto_reset = false;
  int emit_features = 0;    // JSON 에 SLAM 특징점 최대 N 개
  double imu_hold_s = 0.25; // 카메라를 기다리며 IMU 를 붙잡아 두는 최대 시간 (IMU·카메라 시간 순서 유지)
  // 정지 없이 다시 시작 (2026-10-17): 특징점이 끊겨 lost 가 되면 VioManager 를 새로 만들되, 정지 초기화를 기다리지 않고
  // 옛 상태(자세·위치·속도·바이어스, 마지막 특징 갱신 뒤 IMU 만으로 전파한 값)와 추정된 외부 파라미터·시간 오프셋을
  // initialize_with_gt 로 넣는다 → 같은 월드 좌표로 바로 이어서 추적. 마지막 특징 갱신이 reinit_max_gap 초보다 오래됐거나
  // 상태가 망가졌으면(속도 초과·NaN) 예전처럼 정지 초기화 (reason lost, reinit 없음).
  // 근거: rec_20261016_175446/162204/165645 는 lost 뒤 계속 움직여 정지 초기화를 한 번도 못 함 (재생 로그 "failed static init").
  bool reinit_state = false;
  double reinit_max_gap = 1.5;
  double reinit_sig_ori_deg = 3.0;  // 넣은 상태의 공분산 (IMU 만 전파한 1 s 동안 늘어난 불확실성, 추정)
  double reinit_sig_pos = 0.10;
  double reinit_sig_vel = 0.30;
  double gate_mean = 0.0;   // --init-gate (0 = 끔)
  double gate_max = 0.0;
  double gate_disp_deg = 0.0;
  // --remap FILE (2026-10-18, 폰 실시간): cam0 영상을 받은 뒤 이 맵으로 cv::remap (렌즈 2 원본 → lens0u 왜곡 편 영상).
  // 파일 = int32 w, h + float32 mapx[w*h] + float32 mapy[w*h] (scan3d/ov_bridge/android/make_phone_ov.py). 크기가 다른 영상은 그대로.
  std::string remap;
  std::string timing_log;   // --timing-log FILE (per-frame CSV)
  std::string cpus;         // --cpus LIST ("" = no pinning)
  int threads = 0;          // --threads N (0 = config num_opencv_threads)
  int klt_overlap = 0;      // --klt-overlap 0|1 (OV_KLT_OVERLAP for ovpatch/TrackKLT.cpp)
  int klt_pyr = 0;          // --klt-pyr N (2026-10-25 ovheat; OV_KLT_PYR for ovpatch/TrackKLT.cpp; 0 = upstream 5 levels)
  int idle_spin = 0;        // --idle-spin 0|1|2 (header)
  int predict = 0;          // --predict 0|1|2 (header)
  double predict_hz = 100.0;  // --predict-hz (mode 2)
  std::string ovopt;        // --ovopt LIST (header, ovpatch/ovopt.h)
  std::string prof_log;     // --prof-log FILE (header)
  double idle_spin_ms = 0.0;  // --idle-spin-ms W (header; 0 = spin the whole gap)
  int pose_precision = 9;     // --pose-precision N (header; 9 = unchanged)
  double drop_late_ms = 0.0;  // --drop-late MS (header; 0 = off)
  int drop_late_mode = 1;     // --drop-late-mode 0|1 (2026-10-26 droplatefix; 0 = old run-min rule, 1 = fixed)
  int drop_late_cap = 3;      // --drop-late-cap N (mode 1: max consecutive drops)
  double drop_late_win = 3.0; // --drop-late-win S (mode 1: re-anchor after S s of late frames)
  double test_feed_sleep_ms = 0.0, test_feed_sleep_t0 = 0.0;  // --test-feed-sleep MS:T0 (TEST ONLY, 2026-10-26)
  double pred_stale_ms = -1.0;  // --pred-stale-ms MS (-1 = auto: 500 with drop-late mode 1, else 0 = off)
  std::string profile;        // --profile NAME (integ-20261025; "" = none)
  int pred_thread = 0;        // --pred-thread 0|1 (header; 0 = off)
  std::string imu_log;        // --imu-log FILE (header)
  std::string mav_udp;        // --mav-udp HOST:PORT (header; "" = off)
  std::string viop_udp;       // --viop-udp HOST:PORT (header; "" = off)
  double viop_hb_hz = 20.0;   // --viop-hb-hz (VIOP heartbeat rate; 0 = none)
  int viop_hb_src = 0;        // --viop-hb-src 0|1 (esp32fix-20261026: 1 = steady heartbeat thread; 0 = main loop)
  std::string mav_msg = "odom", mav_mount = "imu", mav_log;
  int mav_yaw_align = 1, mav_replay_time = 0, mav_sysid = 1;
  // 2026-10-25 ovrobust (header)
  int init_still = 0;         // --init-still 0|1
  int health = 0;             // --health 0|1 (pose line "h")
  int klt_stats = 0;          // --klt-stats 0|1 (2026-10-26 ovgyro): pose line "klt" = temporal KLT counters (output only)
  int pipe_req = 0;          // --pipe 0|1 (integ2-20261026: 1 is refused; ovpipe-20261026 not merged)
  std::string health_log;     // --health-log FILE
  std::string health_cfg;     // --health-cfg K=V,...
  int recover = 0;            // --recover N (0 = off)
  double recover_max_gap = 3.0;
  int recover_min_feats = 20;
  int recover_full = 0;       // --recover-full 0|1 (full reset when the state is bad / the gap too long; default off)
  std::string test_blank;     // --test-blank T0:D,...
  // 2026-10-26 ovloop (header): loop closure, default off = no loop code runs, outputs unchanged
  int loop = 0;               // --loop 0|1
  std::string loop_vocab;     // --loop-vocab FILE (vocabulary, ovloop_tool train)
  std::string loop_cfg;       // --loop-cfg K=V,... (loop/ovloop.h Cfg)
  std::string loop_log;       // --loop-log FILE (JSON lines per keyframe / loop / optimisation)
  std::string loop_xf;          // --loop-xf CMD (ovxfeat 2026-10-26: XFeat / LighterGlue server for the verification)
  std::string loop_dump, loop_vpr;  // --loop-dump DIR / --loop-vpr FILE (offline VPR evaluation, loop/ovloop.h)
  int loop_sync = 0;          // --loop-sync 0|1 (1 = the VIO thread waits for each keyframe: deterministic PC evaluation only)
  double loop_out_hz = 5.0;   // --loop-out-hz HZ: VIOP MAP_ODOM / MAVLink DEBUG_VECT rate (+ at once on a new optimisation)
  int loop_explicit = 0;      // integ2 0.6.28: --loop given on the command line (else a --profile may have set it)
};

// --health rules (header); --health-cfg upd_deg=0.2,... overrides
struct HealthCfg {
  double upd_deg = 0.2, upd_lost = 1.0;  // s since the last feature update
  double std_deg = 0.25, std_lost = 1.0; // m, position std
  int trk_deg = 40, trk_lost = 10, trk_lost_frames = 15;
  double rej_deg = 0.5;                  // chi2 rejected share (>= 10 candidates)
  bool parse(const std::string &v) {
    std::stringstream ss(v);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      if (tok.empty()) continue;
      auto eq = tok.find('=');
      if (eq == std::string::npos) return false;
      std::string k = tok.substr(0, eq);
      double x = std::atof(tok.c_str() + eq + 1);
      if (k == "upd_deg") upd_deg = x;
      else if (k == "upd_lost") upd_lost = x;
      else if (k == "std_deg") std_deg = x;
      else if (k == "std_lost") std_lost = x;
      else if (k == "trk_deg") trk_deg = static_cast<int>(x);
      else if (k == "trk_lost") trk_lost = static_cast<int>(x);
      else if (k == "trk_lost_frames") trk_lost_frames = static_cast<int>(x);
      else if (k == "rej_deg") rej_deg = x;
      else return false;
    }
    return true;
  }
};
enum HState : int { H_INIT = 0, H_OK = 1, H_DEGRADED = 2, H_LOST = 3 };
const char *hstate_name(int s) { return s == H_OK ? "OK" : s == H_DEGRADED ? "DEGRADED" : s == H_LOST ? "LOST" : "INIT"; }
struct HealthRec {
  int state = H_INIT;
  bool init = false;
  long tracked = -1, slam = 0, msckf = 0;
  ovopt::Health c;  // counters of this frame
  long chi2_rej = 0, used = 0;
  double pos_std = NAN, vel_std = NAN, ori_std_deg = NAN, since_upd = NAN;
  int lost_run = 0;
  int init_ready = 0;  // before init: last init_window_time of fed IMU still (both halves accel std < init_imu_thresh) and
                       // tracked >= 15 -> the upstream "still then jerk" init will fire at the next motion (takeoff)
  double acc_std = NAN;  // max of the two half-window accel stds (m/s^2), before init
};

// --imu-log (2026-10-25): written by the thread that reads the input (main, or the --pred-thread reader)
FILE *g_imu_log = nullptr;
uint64_t g_imu_log_n = 0;
inline int64_t rd_i64(const uint8_t *p) {
  int64_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}
inline void imu_log_row(const uint8_t *payload, int64_t t_hdr, int64_t t_rx) {
  if (!g_imu_log) return;
  std::fprintf(g_imu_log, "%lld,%lld,%lld\n", static_cast<long long>(rd_i64(payload)), static_cast<long long>(t_hdr),
               static_cast<long long>(t_rx));
  if (++g_imu_log_n % 500 == 0) std::fflush(g_imu_log);
}

std::unique_ptr<mav::Out> g_mav;  // --mav-udp (nullptr = off)
// esp32fix-20261026: set only with --viop-hb-src 1; the input reader thread touches the link object only through this
// pointer (default: never), and that object is then never destroyed (released at the clean exit, see main) so a detached
// reader can never use a destroyed object.
std::atomic<mav::Out *> g_hb_feed_out{nullptr};

// ---- --idle-spin (header) ----
constexpr double kIdleSpinMaxMs = 200.0;  // spin at most this long per wait, then block
const char *idle_spin_name(int m) { return m == 1 ? "main" : m == 2 ? "pool" : m == 3 ? "pool_wfe" : "off"; }
inline int64_t boot_ns() {
  timespec ts;
  ::clock_gettime(CLOCK_BOOTTIME, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
inline int64_t mono_ns() {
  timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
inline void cpu_relax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#elif defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#endif
}
// --idle-spin 3 (2026-10-25, ovopt): wait for an event instead of spinning. aarch64: arm the exclusive monitor on the
// flag word (ldaxr) and WFE: the core idles clock-gated in the instruction until another core stores to the flag, an
// interrupt arrives or the generic-timer event stream fires (Linux "evtstrm", ~100 us) - the thread never sleeps, so
// for the scheduler it stays 100 % busy (keeps the clocks, guess: unless the governor's frequency-invariance counts
// stalled cycles), but the core draws less power than a yield loop. Other CPUs: same as cpu_relax.
inline void wait_event_on(const std::atomic<int> &flag) {
#if defined(__aarch64__)
  int v;
  asm volatile("ldaxr %w0, [%1]" : "=&r"(v) : "r"(&flag) : "memory");
  if (v == 0) asm volatile("wfe" ::: "memory");
#else
  (void)flag;
  cpu_relax();
#endif
}
// non-blocking: input readable (data, EOF/HUP or error -> the following read() reports it)
inline bool fd_ready(int fd) {
  pollfd p{fd, POLLIN, 0};
  int r = ::poll(&p, 1, 0);
  if (r < 0) return errno != EINTR;
  return r > 0;
}
// ---- --pred-thread input queue (header; 2026-10-25). Without --pred-thread g_inq stays null and every wait below is
// the original fd poll.
struct InRec {
  uint32_t type = 0;  // record type (chunks already assembled into kStereo) or kEof / kBad
  std::vector<uint8_t> buf;
  int64_t t_hdr_boot = 0, t_rx_boot = 0, t_hdr_mono = 0;
};
constexpr int kMaxPendingImg = 2;  // reader stops reading at a new image while this many images are unprocessed
struct InQueue {
  std::mutex m;
  std::condition_variable cv, cv_space;
  std::deque<InRec> q;
  std::atomic<int> n{0};
  int pending_img = 0;
  uint64_t space_waits = 0;
  std::vector<std::vector<uint8_t>> freeb;  // big record buffers handed back by the main thread (reused by the reader)
  std::vector<uint8_t> take_buf() {
    std::lock_guard<std::mutex> lk(m);
    if (freeb.empty()) return {};
    std::vector<uint8_t> b = std::move(freeb.back());
    freeb.pop_back();
    return b;
  }
  void give_buf(std::vector<uint8_t> &&b) {
    if (b.capacity() < (1u << 20)) return;
    std::lock_guard<std::mutex> lk(m);
    if (freeb.size() < 4) freeb.push_back(std::move(b));
  }
  void push(InRec &&r) {
    {
      std::lock_guard<std::mutex> lk(m);
      if (r.type == kStereo) pending_img++;
      q.push_back(std::move(r));
      n.store(static_cast<int>(q.size()), std::memory_order_release);
    }
    cv.notify_one();
  }
  bool wait_for(int ms) {  // until non-empty (true) or timeout; ms < 0 = no timeout
    std::unique_lock<std::mutex> lk(m);
    if (ms < 0) cv.wait(lk, [&] { return !q.empty(); });
    else cv.wait_for(lk, std::chrono::milliseconds(ms), [&] { return !q.empty(); });
    return !q.empty();
  }
  InRec pop() {
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&] { return !q.empty(); });
    InRec r = std::move(q.front());
    q.pop_front();
    n.store(static_cast<int>(q.size()), std::memory_order_release);
    if (r.type == kStereo) {
      pending_img--;
      cv_space.notify_one();
    }
    return r;
  }
  void wait_space() {
    std::unique_lock<std::mutex> lk(m);
    if (pending_img >= kMaxPendingImg) space_waits++;
    cv_space.wait(lk, [&] { return pending_img < kMaxPendingImg; });
  }
};
InQueue *g_inq = nullptr;
inline bool input_ready(int fd) { return g_inq ? g_inq->n.load(std::memory_order_acquire) > 0 : fd_ready(fd); }
struct IdleSpin {
  int mode = 0, stripes = 1;
  uint64_t waits = 0, spun = 0, timeouts = 0;  // waits = calls, spun = input was not there yet
  double spin_s = 0.0;                          // total time spent spinning
  std::vector<float> exit_us;                   // mode 2 exit latencies (every spun wait)
  double last_wait_us = -1.0, last_exit_us = -1.0;  // the wait before the latest record
  // --idle-spin-ms W (2026-10-25, ovopt; 0 = off = spin the whole gap, unchanged): spin only within W ms before the
  // next camera record is expected (last camera record header + frame period, EMA of the header gaps); earlier the
  // wait blocks in poll() (IMU records still wake it and are handled at once). Timing only, like --idle-spin.
  double win_ms = 0.0;
  int64_t last_frame_ns = 0;
  double period_ns = 0.0;
  uint64_t blocked = 0;     // waits that ended (input arrived) while still blocking before the window
  double blocked_s = 0.0;   // time spent blocking before the window

  void note_frame(int64_t t) {  // a camera record header was read at t (CLOCK_MONOTONIC)
    if (last_frame_ns > 0) {
      const double dt = static_cast<double>(t - last_frame_ns);
      if (dt > 5e6 && dt < 200e6) period_ns = period_ns > 0.0 ? 0.9 * period_ns + 0.1 * dt : dt;
    }
    last_frame_ns = t;
  }

  // returns when fd has input (or EOF / error)
  void wait(int fd) {
    if (mode == 0) return;
    waits++;
    last_exit_us = -1.0;
    if (input_ready(fd)) {
      last_wait_us = 0.0;
      return;
    }
    if (win_ms > 0.0) {  // --idle-spin-ms: block until the spin window opens (or input arrives)
      const int64_t tb = mono_ns();
      const int64_t open_at = (period_ns > 0.0 && last_frame_ns > 0)
                                  ? last_frame_ns + static_cast<int64_t>(period_ns - win_ms * 1e6)
                                  : tb + static_cast<int64_t>(kIdleSpinMaxMs * 1e6);  // no frame rhythm yet: block
      if (tb < open_at) {
        pollfd p{fd, POLLIN, 0};
        int r;
        if (g_inq) {  // --pred-thread: wait on the input queue
          const int64_t left = open_at - mono_ns();
          r = left > 0 ? (g_inq->wait_for(static_cast<int>((left + 999999) / 1000000)) ? 1 : 0) : 0;
        } else
          do {
            const int64_t left = open_at - mono_ns();
            r = left > 0 ? ::poll(&p, 1, static_cast<int>((left + 999999) / 1000000)) : 0;
          } while (r < 0 && errno == EINTR);
        const int64_t te = mono_ns();
        blocked_s += (te - tb) / 1e9;
        if (r != 0) {  // input (or EOF / error) before the window
          blocked++;
          last_wait_us = (te - tb) / 1e3;
          return;
        }
      }
    }
    spun++;
    const int64_t t0 = mono_ns(), tmax = t0 + static_cast<int64_t>(kIdleSpinMaxMs * 1e6);
    bool timed_out = false;
    if (mode == 1) {
      while (!input_ready(fd)) {
        cpu_relax();
        if (mono_ns() > tmax) {
          timed_out = true;
          break;
        }
      }
    } else if (mode == 3) {  // --idle-spin 3: mode 2 with an event wait between the polls (see wait_event_on)
      std::atomic<int> go{0};
      std::atomic<int64_t> t_seen{0};
      std::atomic<bool> to{false};
      cv::parallel_for_(
          cv::Range(0, stripes),
          [&](const cv::Range &) {
            int k = 0;
            while (!go.load(std::memory_order_acquire)) {
              if (input_ready(fd)) {
                int64_t z = 0;
                t_seen.compare_exchange_strong(z, mono_ns());
                go.store(1, std::memory_order_release);  // the store wakes the other stripes out of WFE
                break;
              }
              wait_event_on(go);
              if ((++k & 7) == 0 && mono_ns() > tmax) {
                to.store(true);
                go.store(1, std::memory_order_release);
                break;
              }
            }
          },
          stripes);
      const int64_t t_ret = mono_ns();
      timed_out = to.load() && t_seen.load() == 0;
      if (t_seen.load() > 0) {
        last_exit_us = (t_ret - t_seen.load()) / 1e3;
        if (exit_us.size() < 2000000) exit_us.push_back(static_cast<float>(last_exit_us));
      }
    } else {
      std::atomic<bool> go{false};
      std::atomic<int64_t> t_seen{0};
      std::atomic<bool> to{false};
      cv::parallel_for_(
          cv::Range(0, stripes),
          [&](const cv::Range &) {
            int k = 0;
            while (!go.load(std::memory_order_relaxed)) {
              if (input_ready(fd)) {
                int64_t z = 0;
                t_seen.compare_exchange_strong(z, mono_ns());
                go.store(true);
                break;
              }
              cpu_relax();
              if ((++k & 63) == 0 && mono_ns() > tmax) {
                to.store(true);
                go.store(true);
                break;
              }
            }
          },
          stripes);
      const int64_t t_ret = mono_ns();
      timed_out = to.load() && t_seen.load() == 0;
      if (t_seen.load() > 0) {
        last_exit_us = (t_ret - t_seen.load()) / 1e3;
        if (exit_us.size() < 2000000) exit_us.push_back(static_cast<float>(last_exit_us));
      }
    }
    const int64_t t1 = mono_ns();
    spin_s += (t1 - t0) / 1e9;
    last_wait_us = (t1 - t0) / 1e3;
    if (timed_out) {  // nothing for kIdleSpinMaxMs: block until input
      timeouts++;
      pollfd p{fd, POLLIN, 0};
      if (g_inq) g_inq->wait_for(-1);  // --pred-thread
      else
        while (::poll(&p, 1, -1) < 0 && errno == EINTR) {
        }
    }
  }
  void log(const char *tag) const {
    std::vector<float> e = exit_us;
    std::sort(e.begin(), e.end());
    auto q = [&](double f) { return e.empty() ? -1.0 : static_cast<double>(e[std::min(e.size() - 1, size_t(f * e.size()))]); };
    std::fprintf(stderr,
                 "[ov_bridge] idle spin %s: mode %d (%s) stripes %d | waits %llu spun %llu timeouts %llu spin %.2f s | "
                 "exit_us n %zu p50 %.1f p95 %.1f p99 %.1f max %.1f",
                 tag, mode, idle_spin_name(mode), stripes, static_cast<unsigned long long>(waits),
                 static_cast<unsigned long long>(spun), static_cast<unsigned long long>(timeouts), spin_s, e.size(), q(0.5),
                 q(0.95), q(0.99), e.empty() ? -1.0 : static_cast<double>(e.back()));
    if (win_ms > 0.0)  // --idle-spin-ms (2026-10-25)
      std::fprintf(stderr, " | window %.1f ms period %.2f ms blocked %llu (%.2f s)", win_ms, period_ns / 1e6,
                   static_cast<unsigned long long>(blocked), blocked_s);
    std::fprintf(stderr, "\n");
  }
};
IdleSpin g_idle;

// ---- --cpus (CPU affinity) ----
bool parse_cpus(const std::string &v, cpu_set_t &set) {
  CPU_ZERO(&set);
  int n = 0;
  std::stringstream ss(v);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty()) continue;
    int a = -1, b = -1;
    if (std::sscanf(tok.c_str(), "%d-%d", &a, &b) == 2) {
    } else if (std::sscanf(tok.c_str(), "%d", &a) == 1) b = a;
    else return false;
    if (a < 0 || b < a || b >= CPU_SETSIZE) return false;
    for (int c = a; c <= b; c++) { CPU_SET(c, &set); n++; }
  }
  return n > 0;
}

std::string cpus_str(const cpu_set_t &set) {
  std::string r;
  for (int c = 0; c < 64; c++)
    if (CPU_ISSET(c, &set)) r += (r.empty() ? "" : ",") + std::to_string(c);
  return r;
}

std::vector<int> self_tids() {
  std::vector<int> t;
  if (DIR *d = ::opendir("/proc/self/task")) {
    while (dirent *e = ::readdir(d))
      if (e->d_name[0] >= '0' && e->d_name[0] <= '9') t.push_back(std::atoi(e->d_name));
    ::closedir(d);
  }
  return t;
}

std::string thread_comm(int tid) {
  char path[64], buf[64] = {0};
  std::snprintf(path, sizeof(path), "/proc/self/task/%d/comm", tid);
  if (FILE *f = std::fopen(path, "r")) {
    if (std::fgets(buf, sizeof(buf), f)) buf[std::strcspn(buf, "\n")] = 0;
    std::fclose(f);
  }
  return buf[0] ? buf : "?";
}

// set (if pin) and log the affinity of every thread of this process
void apply_affinity(const cpu_set_t *pin, const char *tag) {
  std::string line;
  int ok = 0, fail = 0, err = 0;
  for (int tid : self_tids()) {
    if (pin) {
      if (::sched_setaffinity(tid, sizeof(cpu_set_t), pin) == 0) ok++;
      else { fail++; err = errno; }
    }
    cpu_set_t got;
    CPU_ZERO(&got);
    std::string g = ::sched_getaffinity(tid, sizeof(got), &got) == 0 ? cpus_str(got) : "err";
    line += " " + std::to_string(tid) + ":" + thread_comm(tid) + "=" + g;
  }
  if (pin)
    std::fprintf(stderr, "[ov_bridge] affinity %s: set %s ok %d fail %d%s%s; threads%s\n", tag, cpus_str(*pin).c_str(), ok,
                 fail, fail ? " errno " : "", fail ? std::strerror(err) : "", line.c_str());
  else
    std::fprintf(stderr, "[ov_bridge] affinity %s: (not pinned) threads%s\n", tag, line.c_str());
}

cpu_set_t g_pin;
bool g_pinned = false;

// JSON 문자열 이스케이프 (예외 메시지에 따옴표·줄바꿈이 들어 있음)
std::string jstr(const std::string &s) {
  std::string o = "\"";
  for (char ch : s) {
    unsigned char c = static_cast<unsigned char>(ch);
    if (c == '"' || c == '\\') {
      o += '\\';
      o += ch;
    } else if (c < 0x20) {
      char b[8];
      std::snprintf(b, sizeof(b), "\\u%04x", c);
      o += b;
    } else {
      o += ch;
    }
  }
  return o + "\"";
}

// ---------- I/O ----------
bool read_full(int fd, void *buf, size_t n) {
  auto *p = static_cast<uint8_t *>(buf);
  while (n > 0) {
    ssize_t r = ::read(fd, p, n);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    p += r;
    n -= static_cast<size_t>(r);
  }
  return true;
}

bool write_full(int fd, const std::string &s, bool is_socket) {
  const char *p = s.data();
  size_t n = s.size();
  while (n > 0) {
    ssize_t r = is_socket ? ::send(fd, p, n, MSG_NOSIGNAL) : ::write(fd, p, n);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    p += r;
    n -= static_cast<size_t>(r);
  }
  return true;
}

template <typename T> T rd(const uint8_t *p) {  // little-endian (x86)
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

// ---------- JSON helpers ----------
void jnum(std::ostringstream &o, double v) {
  if (std::isfinite(v))
    o << v;
  else
    o << "null";
}
template <typename M> void jarr(std::ostringstream &o, const M &m) {  // row-major
  o << "[";
  bool first = true;
  for (int r = 0; r < m.rows(); r++)
    for (int c = 0; c < m.cols(); c++) {
      if (!first) o << ",";
      first = false;
      jnum(o, m(r, c));
    }
  o << "]";
}

struct CamMsg {
  int64_t t_ns;
  double t;
  ov_core::CameraData data;
  double remap_ms = 0.0;
  std::chrono::steady_clock::time_point t_recv;
  int64_t rx_hdr_boot_ns = 0, rx_boot_ns = 0;     // --timing-log: record header / payload read (CLOCK_BOOTTIME)
  double spin_wait_us = -1.0, spin_exit_us = -1.0;  // --idle-spin wait before this record (IdleSpin)
  bool blank = false;                                // --test-blank: image replaced by black
};

// ---- --pred-thread (header; 2026-10-25) ----
std::mutex g_out_m;  // serialises output writes of the main thread and the prediction thread (--pred-thread only)

// --loop (2026-10-26 ovloop): VioManager + read access to the feature tracker (protected member). Only constructed with
// --loop 1; no member, no override -> the filter runs exactly as VioManager.
class VioAcc : public ov_msckf::VioManager {
public:
  using ov_msckf::VioManager::VioManager;
  std::shared_ptr<ov_core::TrackBase> feats() const { return trackFEATS; }
};

// --loop: "map" object of a pose / pred line = the VIO pose in the loop-closure map frame, p_map = Rz(yaw) p + t
// (published T_map_odom, rate limited). The VIO fields of the line are not changed.
void map_fields(std::ostringstream &o, const ovloop::MapOdom &m, const Eigen::Matrix3d &R_ItoG, const Eigen::Vector3d &p,
                const Eigen::Vector3d &v, const Eigen::Matrix3d *R_ItoC, const Eigen::Vector3d *p_IinC) {
  Eigen::Matrix3d Rz;
  Rz << std::cos(m.yaw), -std::sin(m.yaw), 0, std::sin(m.yaw), std::cos(m.yaw), 0, 0, 0, 1;
  const Eigen::Matrix3d R = Rz * R_ItoG;
  const Eigen::Vector3d pm = Rz * p + m.t, vm = Rz * v;
  Eigen::Quaterniond q(R);
  o << ",\"map\":{\"valid\":" << (m.valid ? "true" : "false") << ",\"p\":";
  jarr(o, pm);
  o << ",\"q\":[";
  jnum(o, q.x()); o << ","; jnum(o, q.y()); o << ","; jnum(o, q.z()); o << ","; jnum(o, q.w());
  o << "],\"v\":";
  jarr(o, vm);
  if (R_ItoC && p_IinC) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d R_CtoG = R * R_ItoC->transpose();
    T.block<3, 3>(0, 0) = R_CtoG;
    T.block<3, 1>(0, 3) = pm - R_CtoG * (*p_IinC);
    o << ",\"T_world_cam0\":";
    jarr(o, T);
  }
  o << ",\"yaw\":";
  jnum(o, m.yaw);
  o << ",\"t\":";
  jarr(o, m.t);
  o << ",\"tgt_yaw\":";
  jnum(o, m.tgt_yaw);
  o << ",\"tgt_t\":";
  jarr(o, m.tgt_t);
  o << ",\"n_loop\":" << m.n_loop << ",\"n_kf\":" << m.n_kf << ",\"ver\":" << m.ver << ",\"conv\":" << (m.converging ? 1 : 0)
    << ",\"session\":" << m.session << "}";
}

// copy of the filter numbers a prediction needs (made by the main thread after a frame, read by the prediction thread)
struct PredSnap {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false, frame = false, lost = false;
  double stale_s = 0.0;  // --pred-stale-ms / 1000 (0 = off; 2026-10-26 droplatefix)
  int64_t t0_ns = 0, cam_t_ns = 0;
  double t_state = 0.0, t_off = 0.0, grav = 9.81;
  Eigen::Matrix<double, 16, 1> x;  // [q_GtoI(4) p(3) v(3) bg(3) ba(3)]
  Eigen::Matrix3d Dw, Da, Tg, R_A, R_G, R_ItoC;
  Eigen::Vector3d p_IinC;
  // integ-20261025: only filled with --mav-udp / --viop-udp (drone output from the prediction thread)
  bool has_link = false;
  const void *epoch = nullptr;  // VioManager instance (frame alignment / reset counter)
  bool seeded = false;          // armhist 10-29: this VioManager was seeded by state injection (VIOP_F_SEEDED)
  Eigen::Matrix<double, 9, 9> P9 = Eigen::Matrix<double, 9, 9>::Zero();
  double sigma_w = 0.0;
  int n_feat = 0;
  bool map_on = false;  // --loop (2026-10-26 ovloop): published T_map_odom at snapshot time
  ovloop::MapOdom map;
};

class Predictor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Predictor(int mode, double hz, int precision, int out_fd, bool is_socket)
      : mode_(mode), period_ns_(static_cast<int64_t>(std::llround(1e9 / hz))), hz_(hz), prec_(precision), out_fd_(out_fd),
        sock_(is_socket) {}
  void start() { th_ = std::thread([this] { run(); }); }
  void stop() {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_.notify_all();
    if (th_.joinable()) th_.join();
  }
  // reader thread: every received IMU record (same drop rule as Bridge::on_imu: not newer -> dropped, non-finite)
  void push_imu(const uint8_t *p) {
    PImu s;
    s.t_ns = rd_i64(p);
    double v[6];
    std::memcpy(v, p + 8, sizeof(v));
    s.wm << v[0], v[1], v[2];
    s.am << v[3], v[4], v[5];
    if (!s.wm.allFinite() || !s.am.allFinite()) return;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (s.t_ns <= newest_) return;
      imu_.push_back(s);
      newest_ = s.t_ns;
      while (imu_.size() > 2 && imu_.front().t_ns < s.t_ns - 3000000000LL) imu_.pop_front();
    }
    cv_.notify_one();
  }
  void set_snap(const PredSnap &s) {  // main thread, after the frame's pose line was written
    {
      std::lock_guard<std::mutex> lk(m_);
      snap_ = s;
      if (s.frame) frame_due_ = true;
      n_snap_++;
    }
    cv_.notify_one();
  }
  void invalidate() {
    std::lock_guard<std::mutex> lk(m_);
    snap_.valid = false;
    frame_due_ = false;
  }
  void reset_imu() {  // RESET record (the bridge restarts its IMU time check too)
    std::lock_guard<std::mutex> lk(m_);
    imu_.clear();
    newest_ = INT64_MIN;
    next_due_ = 0;
  }
  void log() const {
    std::vector<float> e = us_, h = h_;
    std::sort(e.begin(), e.end());
    std::sort(h.begin(), h.end());
    auto q = [](const std::vector<float> &v, double f) {
      return v.empty() ? -1.0 : static_cast<double>(v[std::min(v.size() - 1, size_t(f * v.size()))]);
    };
    std::fprintf(stderr,
                 "[ov_bridge] pred thread: mode %d hz %.0f | lines %llu (frame %llu rate %llu) snapshots %llu | compute us p50 "
                 "%.1f p95 %.1f max %.1f | horizon ms p50 %.1f p95 %.1f max %.1f | thread cpu %.3f s (pred) %.3f s (reader)\n",
                 mode_, hz_, static_cast<unsigned long long>(n_frame_ + n_rate_), static_cast<unsigned long long>(n_frame_),
                 static_cast<unsigned long long>(n_rate_), static_cast<unsigned long long>(n_snap_), q(e, 0.5), q(e, 0.95),
                 e.empty() ? -1.0 : static_cast<double>(e.back()), q(h, 0.5), q(h, 0.95),
                 h.empty() ? -1.0 : static_cast<double>(h.back()), cpu_s_, reader_cpu_s);
    if (n_stale_ > 0 || stale_on)
      std::fprintf(stderr, "[ov_bridge] pred thread stale (--pred-stale-ms): %llu lines marked lost\n",
                   static_cast<unsigned long long>(n_stale_));
    if (check_)
      std::fprintf(stderr, "[ov_bridge] pred thread check (OVB_PRED_CHECK): incremental vs from scratch %llu compared, %llu differ\n",
                   static_cast<unsigned long long>(n_check_), static_cast<unsigned long long>(n_check_bad_));
  }
  double reader_cpu_s = 0.0;  // set by the reader thread at its end
  bool stale_on = false;      // --pred-stale-ms active (summary line only)

private:
  struct PImu {
    int64_t t_ns;
    Eigen::Vector3d wm, am;
  };
  void run() {
    std::vector<ov_core::ImuData> d;
    std::unique_lock<std::mutex> lk(m_);
    while (true) {
      cv_.wait(lk, [&] { return stop_ || (snap_.valid && (frame_due_ || (mode_ == 2 && newest_ >= next_due_))); });
      if (stop_) break;
      const bool frame = frame_due_;
      frame_due_ = false;
      const uint64_t gen = n_snap_;
      const int64_t t1n = newest_;
      if (mode_ == 2 && t1n >= next_due_)  // rate grid on IMU time: mean rate = hz (restart after a gap)
        next_due_ = (next_due_ <= 0 || t1n >= next_due_ + period_ns_) ? t1n + period_ns_ : next_due_ + period_ns_;
      const PredSnap s = snap_;
      // IMU samples after the state time (+ the one before it, for the interpolated first sample), up to t1n
      const double time0 = s.t_state + s.t_off, time1 = (t1n - s.t0_ns) * 1e-9;
      d.clear();
      const bool cont = have_cache_ && cache_gen_ == gen && cache_last_.timestamp <= time1;
      if (cont) {  // only the samples after the last integrated one
        d.push_back(cache_last_);
        size_t j = imu_.size();
        while (j > 0 && (imu_[j - 1].t_ns - s.t0_ns) * 1e-9 > cache_last_.timestamp) j--;
        for (; j < imu_.size(); j++) {
          ov_core::ImuData m;
          m.timestamp = (imu_[j].t_ns - s.t0_ns) * 1e-9;
          if (m.timestamp > time1) break;
          m.wm = imu_[j].wm;
          m.am = imu_[j].am;
          d.push_back(m);
        }
      }
      auto full_gather = [&](std::vector<ov_core::ImuData> &out) {  // samples from the snapshot time (interpolated start)
        bool have_before = false;
        ov_core::ImuData before;
        size_t i0 = imu_.size();
        while (i0 > 0 && (imu_[i0 - 1].t_ns - s.t0_ns) * 1e-9 > time0) i0--;
        if (i0 > 0) {
          const auto &bb = imu_[i0 - 1];
          before.timestamp = (bb.t_ns - s.t0_ns) * 1e-9;
          before.wm = bb.wm;
          before.am = bb.am;
          have_before = true;
        }
        for (size_t i = i0; i < imu_.size(); i++) {
          ov_core::ImuData m;
          m.timestamp = (imu_[i].t_ns - s.t0_ns) * 1e-9;
          m.wm = imu_[i].wm;
          m.am = imu_[i].am;
          if (m.timestamp > time1) break;
          if (out.empty()) {
            ov_core::ImuData f = m;
            if (have_before && m.timestamp > before.timestamp) {
              double w = (time0 - before.timestamp) / (m.timestamp - before.timestamp);
              f.wm = (1 - w) * before.wm + w * m.wm;
              f.am = (1 - w) * before.am + w * m.am;
            }
            f.timestamp = time0;
            out.push_back(f);
          }
          out.push_back(m);
        }
      };
      if (!cont) full_gather(d);
      std::vector<ov_core::ImuData> dfull;
      if (check_ && cont) full_gather(dfull);  // OVB_PRED_CHECK=1: incremental vs from-scratch
      lk.unlock();
      const auto c0 = std::chrono::steady_clock::now();
      std::string line = compute(s, d, frame ? "frame" : "rate", cont);
      cache_gen_ = gen;
      if (check_ && cont) {  // the incremental result must equal the from-scratch one bit for bit
        const Eigen::Matrix<double, 16, 1> xi = cache_x_;
        const auto keep_last = cache_last_;
        const int keep_n = cache_n_;
        compute(s, dfull, "check", false);
        n_check_++;
        if (!(cache_x_.array() == xi.array()).all()) n_check_bad_++;
        cache_x_ = xi;
        cache_last_ = keep_last;
        cache_n_ = keep_n;
        have_cache_ = true;
      }
      const float us = std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - c0).count();
      {
        std::lock_guard<std::mutex> lo(g_out_m);
        write_full(out_fd_, line, sock_);
      }
      lk.lock();
      (frame ? n_frame_ : n_rate_)++;
      if (us_.size() < 2000000) us_.push_back(us);
      if (h_.size() < 2000000) h_.push_back(static_cast<float>(last_h_ * 1e3));
    }
    timespec ts;
    ::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    cpu_s_ = ts.tv_sec + ts.tv_nsec * 1e-9;
  }
  // = Bridge::predict (same mean equations as Propagator::fast_state_propagate), from a snapshot
  std::string compute(const PredSnap &s, const std::vector<ov_core::ImuData> &d, const char *src, bool cont) {
    Eigen::Matrix<double, 16, 1> x = cont ? cache_x_ : s.x;
    const int n_prev = cont ? cache_n_ : 0;
    const Eigen::Vector3d bg = x.block(10, 0, 3, 1), ba = x.block(13, 0, 3, 1);
    const Eigen::Vector3d grav(0.0, 0.0, s.grav);
    for (size_t i = 0; i + 1 < d.size(); i++) {
      const auto &m0 = d[i], &m1 = d[i + 1];
      const double dt = m1.timestamp - m0.timestamp;
      if (dt <= 0.0) continue;
      Eigen::Vector3d a1 = s.R_A * s.Da * (m0.am - ba), a2 = s.R_A * s.Da * (m1.am - ba);
      Eigen::Vector3d a = 0.5 * (a1 + a2);
      Eigen::Vector3d w1 = s.R_G * s.Dw * (m0.wm - bg - s.Tg * a1), w2 = s.R_G * s.Dw * (m1.wm - bg - s.Tg * a2);
      Eigen::Vector3d w = 0.5 * (w1 + w2);
      Eigen::Matrix3d R = ov_core::quat_2_Rot(x.block(0, 0, 4, 1));
      Eigen::Vector3d v = x.block(7, 0, 3, 1), p = x.block(4, 0, 3, 1);
      x.block(0, 0, 4, 1) = ov_core::rot_2_quat(ov_core::exp_so3(-w * dt) * R);
      x.block(4, 0, 3, 1) = p + v * dt + 0.5 * R.transpose() * a * dt * dt - 0.5 * grav * dt * dt;
      x.block(7, 0, 3, 1) = v + R.transpose() * a * dt - grav * dt;
    }
    const double time0 = s.t_state + s.t_off;
    if (!d.empty()) {  // cache for the next prediction from the same snapshot
      cache_x_ = x;
      cache_last_ = d.back();
      cache_n_ = n_prev + static_cast<int>(d.size()) - 1;
      have_cache_ = true;
    } else
      have_cache_ = false;
    const double h = cont ? (d.empty() ? 0.0 : d.back().timestamp - time0) : (d.size() >= 2 ? d.back().timestamp - time0 : 0.0);
    last_h_ = h;
    Eigen::Vector4d q_GtoI = x.block(0, 0, 4, 1);
    Eigen::Matrix3d R_ItoG = ov_core::quat_2_Rot(q_GtoI).transpose();
    Eigen::Vector3d p_IinG = x.block(4, 0, 3, 1), v = x.block(7, 0, 3, 1);
    Eigen::Quaterniond qh(R_ItoG);
    Eigen::Matrix3d R_CtoG = R_ItoG * s.R_ItoC.transpose();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_CtoG;
    T.block<3, 1>(0, 3) = p_IinG - R_CtoG * s.p_IinC;
    const double t_pred = time0 + h;
    const bool stale = s.stale_s > 0.0 && h > s.stale_s;  // --pred-stale-ms (2026-10-26): too long IMU-only -> lost
    const bool lost = s.lost || stale;
    if (stale && std::strcmp(src, "check") != 0) n_stale_++;
    if (g_mav && s.has_link && std::strcmp(src, "check") != 0) {  // integ-20261025: drone link from the prediction thread
      Eigen::Vector3d w_I = Eigen::Vector3d::Zero(), f_I = Eigen::Vector3d::Zero();
      if (!d.empty()) {  // newest IMU sample used (= Bridge::predict: rate + specific force, bias removed)
        w_I = d.back().wm - bg;
        f_I = d.back().am - ba;
      }
      const int n_int = cont ? cache_n_ : (d.empty() ? 0 : static_cast<int>(d.size()) - 1);
      const double rate = (n_int > 0 && h > 0.0) ? n_int / h : 400.0;
      g_mav->emit(s.epoch, s.t0_ns + static_cast<int64_t>(std::llround(t_pred * 1e9)), R_ItoG, p_IinG, v, w_I, s.P9, lost,
                  s.sigma_w * s.sigma_w * std::min(rate, 2000.0), f_I, src,
                  s.t0_ns + static_cast<int64_t>(std::llround(time0 * 1e9)), s.n_feat, s.seeded);
    }
    std::ostringstream o;
    o.precision(prec_);
    o << "{\"type\":\"pred\",\"t_ns\":" << (s.t0_ns + static_cast<int64_t>(std::llround(t_pred * 1e9))) << ",\"t\":";
    jnum(o, t_pred - s.t_off);
    o << ",\"t_cam_ns\":" << s.cam_t_ns << ",\"t_state\":";
    jnum(o, s.t_state);
    o << ",\"h\":";
    jnum(o, h);
    o << ",\"n_imu\":" << (cont ? cache_n_ : (d.empty() ? 0 : static_cast<int>(d.size()) - 1)) << ",\"src\":\"" << src
      << "\",\"status\":\"" << (lost ? "lost" : "tracking") << "\",\"p\":";
    jarr(o, p_IinG);
    o << ",\"q\":[";
    jnum(o, qh.x()); o << ","; jnum(o, qh.y()); o << ","; jnum(o, qh.z()); o << ","; jnum(o, qh.w());
    o << "],\"v\":";
    jarr(o, v);
    o << ",\"T_world_cam0\":";
    jarr(o, T);
    o << ",\"q_GtoI_jpl\":";
    jarr(o, q_GtoI);
    if (s.map_on) map_fields(o, s.map, R_ItoG, p_IinG, v, &s.R_ItoC, &s.p_IinC);  // --loop (2026-10-26 ovloop)
    if (stale) o << ",\"stale\":true";
    o << ",\"tb\":" << boot_ns() << "}\n";
    return o.str();
  }

  int mode_;
  int64_t period_ns_;
  double hz_;
  int prec_, out_fd_;
  bool sock_;
  std::thread th_;
  std::mutex m_;
  std::condition_variable cv_;
  bool stop_ = false, frame_due_ = false;
  PredSnap snap_;
  std::deque<PImu> imu_;
  int64_t newest_ = INT64_MIN, next_due_ = 0;
  uint64_t n_frame_ = 0, n_rate_ = 0, n_snap_ = 0, n_stale_ = 0;
  std::vector<float> us_, h_;
  double last_h_ = 0.0, cpu_s_ = 0.0;
  // incremental prediction cache (prediction thread only)
  bool check_ = std::getenv("OVB_PRED_CHECK") != nullptr;
  uint64_t n_check_ = 0, n_check_bad_ = 0;
  bool have_cache_ = false;
  uint64_t cache_gen_ = ~0ull;
  Eigen::Matrix<double, 16, 1> cache_x_;
  ov_core::ImuData cache_last_;
  int cache_n_ = 0;
};

class Bridge {
public:
  Bridge(const Args &a) : args_(a) {
    parser_ = std::make_shared<ov_core::YamlParser>(a.config);
    std::string verbosity = "INFO";
    parser_->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    params_.print_and_load(parser_);
    params_.use_multi_threading_subs = false;  // 순서대로 처리 (카메라 feed 는 이 스레드에서만)
    int cfg_threads = params_.num_opencv_threads;
    if (a.threads > 0) params_.num_opencv_threads = a.threads;  // --threads: VioManager ctor calls cv::setNumThreads(this)
    {
      std::string bi = cv::getBuildInformation();
      auto pos = bi.find("Parallel framework:");
      if (pos != std::string::npos) {
        pos = bi.find_first_not_of(' ', pos + 19);
        cv_parallel_ = bi.substr(pos, bi.find('\n', pos) - pos);
      }
    }
    std::fprintf(stderr, "[ov_bridge] opencv threads: config %d, --threads %d -> %d | %s | cpus %d | klt overlap %d\n",
                 cfg_threads, a.threads, params_.num_opencv_threads, cv_parallel_.c_str(), cv::getNumberOfCPUs(),
                 a.klt_overlap);
    if (!a.remap.empty()) load_remap(a.remap);
    if (!a.timing_log.empty()) {
      tlog_ = std::fopen(a.timing_log.c_str(), "w");
      if (tlog_)
        std::fprintf(tlog_, "t_ns,t,wait_ms,remap_ms,feed_ms,init,n_slam,n_msckf,cam_dropped,cam_queue,"
                            "rx_hdr_boot_ns,rx_boot_ns,spin_wait_us,spin_exit_us\n");
      else std::fprintf(stderr, "[ov_bridge] timing log: cannot open %s\n", a.timing_log.c_str());
    }
    if (!a.prof_log.empty()) {  // --prof-log (2026-10-25): ovpatch stage timers, one row per fed frame
      plog_ = std::fopen(a.prof_log.c_str(), "w");
      if (plog_) {
        std::fprintf(plog_, "t_ns,t,feed_ms,init,n_slam,n_msckf");
        for (int s = 0; s < ovopt::N_SLOTS; s++) std::fprintf(plog_, ",%s", ovopt::slot_name(s));
        for (int k = 0; k < ovopt::N_COUNTS; k++) std::fprintf(plog_, ",%s", ovopt::count_name(k));
        std::fprintf(plog_, "\n");
      } else std::fprintf(stderr, "[ov_bridge] prof log: cannot open %s\n", a.prof_log.c_str());
    }
    if (!a.health_log.empty()) {  // --health-log (2026-10-25 ovrobust)
      hlog_ = std::fopen(a.health_log.c_str(), "w");
      if (hlog_)
        std::fprintf(hlog_, "t_ns,t,state,init,tracked,slam,msckf,msckf_in,msckf_tri,msckf_used,slam_in,slam_clean,slam_used,"
                            "delay_in,delay_ok,chi2_rej,pos_std,vel_std,ori_std_deg,since_upd,lost_run,seg,blank,recover_n,init_ready,acc_std\n");
      else std::fprintf(stderr, "[ov_bridge] health log: cannot open %s\n", a.health_log.c_str());
    }
    if (!a.health_cfg.empty() && !hcfg_.parse(a.health_cfg)) {
      std::fprintf(stderr, "[ov_bridge] bad --health-cfg %s\n", a.health_cfg.c_str());
      ok_ = false;
      return;
    }
    health_on_ = a.health || hlog_ || a.recover > 0;
    pred_stale_s_ = a.pred_stale_ms > 0.0 ? a.pred_stale_ms / 1000.0 : 0.0;  // resolved in main (2026-10-26)
    {  // --test-blank T0:D,...
      std::stringstream ss(a.test_blank);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        double t0 = 0, d = 0;
        if (tok.empty()) continue;
        if (std::sscanf(tok.c_str(), "%lf:%lf", &t0, &d) != 2 || d <= 0) {
          std::fprintf(stderr, "[ov_bridge] bad --test-blank %s\n", a.test_blank.c_str());
          ok_ = false;
          return;
        }
        blank_.push_back({t0, t0 + d});
      }
    }
    if (health_on_ || a.init_still || !blank_.empty())
      std::fprintf(stderr, "[ov_bridge] ovrobust: init_still %d | health %d log %s | rules upd %.2f/%.2f s std %.2f/%.2f m trk %d/%d(%d fr) "
                   "rej %.2f | recover %d (max gap %.1f s, min feats %d) | test blank %zu windows\n",
                   a.init_still, a.health, a.health_log.empty() ? "-" : a.health_log.c_str(), hcfg_.upd_deg, hcfg_.upd_lost,
                   hcfg_.std_deg, hcfg_.std_lost, hcfg_.trk_deg, hcfg_.trk_lost, hcfg_.trk_lost_frames, hcfg_.rej_deg, a.recover,
                   a.recover_max_gap, a.recover_min_feats, blank_.size());
    if (a.gate_mean > 0.0 && a.gate_disp_deg > 0.0 && params_.camera_intrinsics.count(0)) {
      double fx = params_.camera_intrinsics.at(0)->get_value()(0);
      double px = fx * std::tan(a.gate_disp_deg * M_PI / 180.0);
      std::fprintf(stderr, "[ov_bridge] init gate: gyro mean <= %.4f max <= %.3f rad/s, init_max_disparity %.2f -> %.2f px (%.2f deg, fx %.1f)\n",
                   a.gate_mean, a.gate_max, params_.init_options.init_max_disparity, px, a.gate_disp_deg, fx);
      shadow_params_ = params_;
      shadow_params_.init_options.init_max_disparity = px;  // 기본 params_ 는 그대로 (그림자만 넓힘)
      use_shadow_ = true;
    }
    if (!parser_->successful()) {
      PRINT_ERROR("[ov_bridge] unable to parse all parameters in %s\n", a.config.c_str());
      ok_ = false;
      return;
    }
    if (a.loop) {  // --loop (2026-10-26 ovloop): loop-closure thread (keyframes only; never writes the filter)
      ovloop::Cfg lc;
      lc.vocab = a.loop_vocab;
      lc.log = a.loop_log;
      lc.sync = a.loop_sync;
      lc.dump = a.loop_dump;
      lc.vpr = a.loop_vpr;
      lc.xf = a.loop_xf;
      std::string err;
      if (!a.loop_cfg.empty() && !lc.parse(a.loop_cfg, err)) {
        std::fprintf(stderr, "[ov_bridge] %s\n", err.c_str());
        ok_ = false;
        return;
      }
      loop_.reset(new ovloop::LoopCloser(lc));
      if (!loop_->ok()) {
        std::fprintf(stderr, "[ov_bridge] loop: %s\n", loop_->err().c_str());
        ok_ = false;
        return;
      }
    }
    make_system();
  }

  bool ok() const { return ok_; }

  // --init-gate 그림자 VioManager (넓힌 시차 기준) 새로 만들기. refeed = 지금까지 넣은 최근 IMU 를 다시 넣음.
  void make_shadow(bool refeed) {
    shadow_.reset();
    shadow_checked_ = shadow_ok_ = false;
    if (!use_shadow_) return;
    shadow_ = new_vio(shadow_params_);
    if (refeed)
      for (const auto &m : imu_recent_) shadow_->feed_measurement_imu(m);
  }

  // VioManager 새로 만들기. 붙잡아 둔 IMU(imu_hold_) 는 새 시스템에 그대로 넣을 수 있으므로 남긴다.
  // --loop (2026-10-26 ovloop): VioAcc (= VioManager + tracker read access) only with --loop 1
  std::shared_ptr<ov_msckf::VioManager> new_vio(ov_msckf::VioManagerOptions &p) {
    if (loop_) return std::make_shared<VioAcc>(p);
    return std::make_shared<ov_msckf::VioManager>(p);
  }

  void make_system(ov_msckf::VioManagerOptions *p = nullptr) {  // VioManager takes a non-const reference
    const bool had_sys = sys_ != nullptr;
    if (p) sys_ = new_vio(*p);
    else sys_ = new_vio(params_);
    sys_seeded_ = false;  // armhist 10-29: reinit_from_state sets it again after the injection
    if (loop_ && had_sys) loop_->reset_session("VIO re-initialised");  // --loop: new world frame -> new graph
    pred_pending_ = false;
    snap_dirty_ = true;  // --pred-thread: the published snapshot is stale
    make_shadow(false);
    init_checked_ = false;
    cams_.clear();
    last_imu_t_ = -1e300;
    zero_feat_frames_ = 0;
    feat_seen_ = false;
    lost_ = false;
    h_last_upd_t_ = -1e300;   // --health (2026-10-25): per VioManager
    h_upd_seen_ = false;
    h_lost_run_ = 0;
    h_trk_low_run_ = 0;
  }

  // --timing-log receive info of the record handle() gets next (serve; timing only, never used by the VIO)
  void set_rx(int64_t hdr_boot_ns, int64_t boot_ns, double spin_wait_us, double spin_exit_us) {
    rx_hdr_boot_ns_ = hdr_boot_ns;
    rx_boot_ns_ = boot_ns;
    rx_spin_wait_us_ = spin_wait_us;
    rx_spin_exit_us_ = spin_exit_us;
  }

  // full VIO re-initialisation (never state injection): RESET record, and the FC VIO reset request (armhist 10-29)
  void full_reset() {
    make_system();
    imu_hold_.clear();
    imu_recent_.clear();
    last_upd_t_ = -1e300;
    newest_imu_t_ = -1e300;
    first_cam_t_ = -1e300;
    off_active_ = off_pending_ = false;  // --recover frame offset (2026-10-25)
  }

  // 입력 하나 처리. 출력 JSON 줄들을 out 에 덧붙인다. false = QUIT.
  bool handle(uint32_t type, const std::vector<uint8_t> &buf, std::string &out) {
    if (g_mav && g_mav->vio_reset_req.exchange(false)) {  // armhist 10-29: FC asked (VIOP_T_VIO_RESET_REQ), main thread
      full_reset();
      out += "{\"type\":\"reset\",\"reason\":\"fc_request\"}\n";
    }
    switch (type) {
    case kImu:
      if (buf.size() != 56) {
        out += err_line("IMU payload must be 56 bytes");
        return true;
      }
      on_imu(buf.data(), out);
      if (g_mav && have_t0_) {
        g_mav->note_imu(t0_ns_ + static_cast<int64_t>(std::llround(newest_imu_t_ * 1e9)));
        if (g_mav->hb_src) {  // esp32fix-20261026 --viop-hb-src 1: the heartbeat thread sends; the main loop only reports
          g_mav->note_main(sys_ && sys_->initialized(), lost_ || late_lost_ || hb_stale());  // integ2: + droplatefix lost flags
          if (!g_inq) g_mav->note_feed();  // no reader thread: the main loop is where IMU is received
        } else
          g_mav->hb_tick(sys_ && sys_->initialized(), lost_ || late_lost_ || hb_stale());  // VIOP heartbeat from the main loop (stops if the feed stops)
      }
      return true;
    case kStereo:
      on_stereo(buf, out);
      return true;
    case kReset:
      full_reset();
      out += "{\"type\":\"reset\"}\n";
      return true;
    case kQuit:
      return false;
    default:
      out += err_line("unknown message type " + std::to_string(type));
      return true;
    }
  }

  std::string ready_line() const {
    std::ostringstream o;
    o << "{\"type\":\"ready\",\"num_cameras\":" << params_.state_options.num_cameras
      << ",\"use_stereo\":" << (params_.use_stereo ? "true" : "false") << ",\"imu_hold_s\":" << args_.imu_hold_s
      << ",\"cv_threads\":" << cv::getNumThreads() << ",\"cv_parallel\":" << jstr(cv_parallel_)
      << ",\"klt_overlap\":" << args_.klt_overlap << ",\"idle_spin\":" << g_idle.mode
      << ",\"idle_spin_stripes\":" << g_idle.stripes << ",\"predict\":" << args_.predict
      << ",\"predict_hz\":" << args_.predict_hz;
    if (!args_.ovopt.empty() || !args_.prof_log.empty()) o << ",\"ovopt\":" << jstr(ovopt::describe());  // 2026-10-25
    if (args_.idle_spin_ms > 0.0) o << ",\"idle_spin_ms\":" << args_.idle_spin_ms;                       // 2026-10-25
    if (args_.pred_thread) o << ",\"pred_thread\":" << args_.pred_thread;                                 // 2026-10-25
    if (!args_.profile.empty()) o << ",\"profile\":" << jstr(args_.profile);                            // integ-20261025
    if (g_mav) o << ",\"mav_udp\":" << jstr(args_.mav_udp) << ",\"mav_mount\":" << jstr(g_mav->mount_name);  // ovdrone
    if (args_.init_still) o << ",\"init_still\":1";                                                   // 2026-10-25 ovrobust
    if (health_on_) o << ",\"health\":" << args_.health << ",\"recover\":" << args_.recover;
    if (!blank_.empty()) o << ",\"test_blank\":" << jstr(args_.test_blank);
    o << "}\n";
    return o.str();
  }

  // ---- --predict (header) ----
  // a prediction is due: mode 1/2 after a frame update, mode 2 also every 1/predict_hz of IMU time
  bool predict_due() const {
    if (args_.predict <= 0 || !sys_ || !sys_->initialized()) return false;
    if (pred_pending_) return true;
    return args_.predict == 2 && newest_imu_t_ > last_pred_imu_t_ &&
           newest_imu_t_ - last_pred_imu_t_ >= 1.0 / args_.predict_hz - 1e-6;
  }

  // propagate a copy of the current IMU state to the newest received IMU sample, append a "pred" line
  void predict(std::string &out) {
    const char *src = pred_pending_ ? "frame" : "rate";
    pred_pending_ = false;
    last_pred_imu_t_ = newest_imu_t_;
    auto st = sys_->get_state();
    if (st->_timestamp < 0.0) return;
    const double t_off = st->_calib_dt_CAMtoIMU->value()(0);
    const double time0 = st->_timestamp + t_off, time1 = newest_imu_t_;
    // IMU samples after time0: fed (imu_recent_) then held (imu_hold_), both in time order and disjoint
    std::vector<ov_core::ImuData> d;
    const ov_core::ImuData *before = nullptr;
    auto take = [&](const ov_core::ImuData &m) {
      if (m.timestamp <= time0) {
        before = &m;
        return;
      }
      if (m.timestamp > time1) return;
      if (d.empty()) {  // first sample after time0: start with the sample interpolated at time0
        ov_core::ImuData s = m;
        if (before && m.timestamp > before->timestamp) {
          double f = (time0 - before->timestamp) / (m.timestamp - before->timestamp);
          s.wm = (1 - f) * before->wm + f * m.wm;
          s.am = (1 - f) * before->am + f * m.am;
        }
        s.timestamp = time0;
        d.push_back(s);
      }
      d.push_back(m);
    };
    for (const auto &m : imu_recent_) take(m);
    for (const auto &m : imu_hold_) take(m);

    Eigen::Matrix<double, 16, 1> x = st->_imu->value();  // copy: [q_GtoI(4) p(3) v(3) bg(3) ba(3)]
    const Eigen::Vector3d bg = x.block(10, 0, 3, 1), ba = x.block(13, 0, 3, 1);
    const Eigen::Matrix3d Dw = ov_msckf::State::Dm(st->_options.imu_model, st->_calib_imu_dw->value());
    const Eigen::Matrix3d Da = ov_msckf::State::Dm(st->_options.imu_model, st->_calib_imu_da->value());
    const Eigen::Matrix3d Tg = ov_msckf::State::Tg(st->_calib_imu_tg->value());
    const Eigen::Matrix3d R_A = st->_calib_imu_ACCtoIMU->Rot(), R_G = st->_calib_imu_GYROtoIMU->Rot();
    const Eigen::Vector3d grav(0.0, 0.0, params_.gravity_mag);
    for (size_t i = 0; i + 1 < d.size(); i++) {  // = Propagator::fast_state_propagate mean
      const auto &m0 = d[i], &m1 = d[i + 1];
      const double dt = m1.timestamp - m0.timestamp;
      if (dt <= 0.0) continue;
      Eigen::Vector3d a1 = R_A * Da * (m0.am - ba), a2 = R_A * Da * (m1.am - ba);
      Eigen::Vector3d a = 0.5 * (a1 + a2);
      Eigen::Vector3d w1 = R_G * Dw * (m0.wm - bg - Tg * a1), w2 = R_G * Dw * (m1.wm - bg - Tg * a2);
      Eigen::Vector3d w = 0.5 * (w1 + w2);
      Eigen::Matrix3d R = ov_core::quat_2_Rot(x.block(0, 0, 4, 1));
      Eigen::Vector3d v = x.block(7, 0, 3, 1), p = x.block(4, 0, 3, 1);
      x.block(0, 0, 4, 1) = ov_core::rot_2_quat(ov_core::exp_so3(-w * dt) * R);
      x.block(4, 0, 3, 1) = p + v * dt + 0.5 * R.transpose() * a * dt * dt - 0.5 * grav * dt * dt;
      x.block(7, 0, 3, 1) = v + R.transpose() * a * dt - grav * dt;
    }
    const double h = d.size() >= 2 ? d.back().timestamp - time0 : 0.0;
    Eigen::Vector4d q_GtoI = x.block(0, 0, 4, 1);
    Eigen::Matrix3d R_ItoG = ov_core::quat_2_Rot(q_GtoI).transpose();
    Eigen::Vector3d p_IinG = x.block(4, 0, 3, 1), v = x.block(7, 0, 3, 1);
    apply_off(R_ItoG, p_IinG, v, q_GtoI);  // --recover full reset: output frame offset (2026-10-25; no-op otherwise)
    Eigen::Quaterniond qh(R_ItoG);
    auto calib0 = st->_calib_IMUtoCAM.at(0);
    Eigen::Matrix3d R_CtoG = R_ItoG * calib0->Rot().transpose();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_CtoG;
    T.block<3, 1>(0, 3) = p_IinG - R_CtoG * calib0->pos();
    const double t_pred = time0 + h;  // IMU clock (s from t0)
    const bool stale = pred_stale_s_ > 0.0 && h > pred_stale_s_;  // --pred-stale-ms (2026-10-26)
    if (stale) n_pred_stale_++;
    if (g_mav) {  // --mav-udp (2026-10-25): same predicted state -> MAVLink (reads only)
      const ov_core::ImuData *nw = nullptr;  // newest IMU sample (rate + specific force for the output)
      if (!imu_hold_.empty()) nw = &imu_hold_.back();
      if (!imu_recent_.empty() && (!nw || imu_recent_.back().timestamp > nw->timestamp)) nw = &imu_recent_.back();
      Eigen::Vector3d w_I = Eigen::Vector3d::Zero(), f_I = Eigen::Vector3d::Zero();
      if (nw) {
        w_I = nw->wm - bg;
        f_I = nw->am - ba;
      }
      std::vector<std::shared_ptr<ov_type::Type>> vars = {st->_imu->q(), st->_imu->p(), st->_imu->v()};
      Eigen::Matrix<double, 9, 9> P9 = ov_msckf::StateHelper::get_marginal_covariance(st, vars);
      const double rate = (d.size() >= 2 && h > 0.0) ? (d.size() - 1) / h : 400.0;
      const double sw = params_.imu_noises.sigma_w;
      g_mav->emit(sys_.get(), t0_ns_ + static_cast<int64_t>(std::llround(t_pred * 1e9)), R_ItoG, x.block(4, 0, 3, 1),
                  x.block(7, 0, 3, 1), w_I, P9, lost_ || late_lost_ || stale, sw * sw * std::min(rate, 2000.0), f_I, src,
                  t0_ns_ + static_cast<int64_t>(std::llround(time0 * 1e9)), static_cast<int>(st->_features_SLAM.size()));
    }
    n_pred_++;
    if (h_pred_.size() < 2000000) h_pred_.push_back(static_cast<float>(h * 1e3));
    std::ostringstream o;
    o.precision(args_.pose_precision);  // --pose-precision (default 9 = unchanged)
    o << "{\"type\":\"pred\",\"t_ns\":" << (t0_ns_ + static_cast<int64_t>(std::llround(t_pred * 1e9))) << ",\"t\":";
    jnum(o, t_pred - t_off);
    o << ",\"t_cam_ns\":" << pred_cam_t_ns_ << ",\"t_state\":";
    jnum(o, st->_timestamp);
    o << ",\"h\":";
    jnum(o, h);
    o << ",\"n_imu\":" << (d.empty() ? 0 : d.size() - 1) << ",\"src\":\"" << src << "\",\"status\":\""
      << ((lost_ || late_lost_ || stale) ? "lost" : "tracking") << "\",\"p\":";
    jarr(o, p_IinG);
    o << ",\"q\":[";
    jnum(o, qh.x()); o << ","; jnum(o, qh.y()); o << ","; jnum(o, qh.z()); o << ","; jnum(o, qh.w());
    o << "],\"v\":";
    jarr(o, v);
    o << ",\"T_world_cam0\":";
    jarr(o, T);
    o << ",\"q_GtoI_jpl\":";
    jarr(o, q_GtoI);
    if (loop_) {  // --loop (2026-10-26 ovloop)
      const Eigen::Matrix3d R_ItoC = calib0->Rot();
      const Eigen::Vector3d p_IinC = calib0->pos();
      map_fields(o, loop_->published(), R_ItoG, p_IinG, v, &R_ItoC, &p_IinC);
    }
    o << "}\n";
    out += o.str();
  }

  // --pred-thread (2026-10-25): copy the numbers a prediction needs (main thread; only reads the state)
  bool snap_dirty() const { return snap_dirty_; }
  void make_snap(PredSnap &s) {
    snap_dirty_ = false;
    s.valid = false;
    s.frame = pred_pending_;
    pred_pending_ = false;
    if (!sys_ || !sys_->initialized()) return;
    auto st = sys_->get_state();
    if (st->_timestamp < 0.0) return;
    s.valid = true;
    s.lost = lost_ || late_lost_;  // late_lost_: --drop-late mode 1 (2026-10-26)
    s.stale_s = pred_stale_s_;
    s.t0_ns = t0_ns_;
    s.cam_t_ns = pred_cam_t_ns_;
    s.t_state = st->_timestamp;
    s.t_off = st->_calib_dt_CAMtoIMU->value()(0);
    s.grav = params_.gravity_mag;
    s.x = st->_imu->value();
    s.Dw = ov_msckf::State::Dm(st->_options.imu_model, st->_calib_imu_dw->value());
    s.Da = ov_msckf::State::Dm(st->_options.imu_model, st->_calib_imu_da->value());
    s.Tg = ov_msckf::State::Tg(st->_calib_imu_tg->value());
    s.R_A = st->_calib_imu_ACCtoIMU->Rot();
    s.R_G = st->_calib_imu_GYROtoIMU->Rot();
    auto calib0 = st->_calib_IMUtoCAM.at(0);
    s.R_ItoC = calib0->Rot();
    s.p_IinC = calib0->pos();
    if (g_mav) {  // integ-20261025: what the drone link needs (= Bridge::predict)
      s.has_link = true;
      s.epoch = sys_.get();
      s.seeded = sys_seeded_;
      std::vector<std::shared_ptr<ov_type::Type>> vars = {st->_imu->q(), st->_imu->p(), st->_imu->v()};
      s.P9 = ov_msckf::StateHelper::get_marginal_covariance(st, vars);
      s.sigma_w = params_.imu_noises.sigma_w;
      s.n_feat = static_cast<int>(st->_features_SLAM.size());
    }
    s.map_on = loop_ != nullptr;  // --loop (2026-10-26 ovloop)
    if (loop_) s.map = loop_->published();
  }

  void log_gyro_klt() const {  // ovgyro summary (2026-10-26): whole-run temporal KLT counters
    if (ovopt::g.gyro_klt == 0 && !args_.klt_stats) return;
    const auto &k = scan3d_gyro::total;
    std::fprintf(stderr, "[ov_bridge] gyro_klt mode %d: frames %ld (no rotation %ld) | pts in %ld LK ok %ld (%.2f %%) kept %ld "
                 "(%.2f %%) | fallback %ld/%ld | predicted shift px mean %.3f max %.2f\n", ovopt::g.gyro_klt, k.frames, k.no_rot,
                 k.pts_in, k.klt_ok, k.pts_in ? 100.0 * k.klt_ok / k.pts_in : 0.0, k.pts_out,
                 k.pts_in ? 100.0 * k.pts_out / k.pts_in : 0.0, k.fb_ok, k.fb_try, k.shift_n ? k.shift_sum / k.shift_n : 0.0,
                 k.shift_max);
    std::fprintf(stderr, "[ov_bridge] gyro_klt mode %d: LK result - start px mean %.3f | LK result - old px mean %.3f (n %ld) | "
                 "guards: capped pts %ld, translation refused frames %ld\n",
                 ovopt::g.gyro_klt, k.res_n ? k.res_sum / k.res_n : 0.0, k.res_n ? k.flow_sum / k.res_n : 0.0, k.res_n, k.capped,
                 k.t_blocked);
  }

  void log_rs() const {  // ovopt rs_derot summary (2026-10-25)
    if (ovopt::g.rs_derot == 0) return;
    const auto &S = ovopt::rs_stats;
    std::fprintf(stderr, "[ov_bridge] rs_derot: readout %.1f us | %s | frames %ld (no imu %ld) feats %ld | shift px mean %.4f "
                 "max %.3f | gyro rad/s mean %.3f max %.3f\n", ovopt::rs_readout_s() * 1e6,
                 ovopt::rs_mapy ? "remap rows" : "row scale", S.frames, S.no_imu, S.feats,
                 S.feats ? S.sum_shift / S.feats : 0.0, S.max_shift, S.frames ? S.sum_w / S.frames : 0.0, S.max_w);
  }

  void log_drop() const {  // --drop-late summary (2026-10-25)
    if (args_.drop_late_ms <= 0.0) return;
    std::fprintf(stderr, "[ov_bridge] drop-late: limit %.1f ms | dropped %llu frames | min lag %.1f ms\n", args_.drop_late_ms,
                 static_cast<unsigned long long>(n_late_dropped_), lag_min_ms_ < 1e299 ? lag_min_ms_ : -1.0);
    if (args_.drop_late_mode == 1)
      std::fprintf(stderr, "[ov_bridge] drop-late mode 1: cap %d win %.2f s | forced %llu | re-anchors %llu | late-lost %llu | max late run %d frames | "
                   "pred stale %.0f ms (main-loop stale preds %llu)\n", args_.drop_late_cap, args_.drop_late_win,
                   static_cast<unsigned long long>(n_late_forced_), static_cast<unsigned long long>(n_late_reanchor_),
                   static_cast<unsigned long long>(n_late_lost_), late_run_max_, pred_stale_s_ * 1e3, static_cast<unsigned long long>(n_pred_stale_));
    else
      std::fprintf(stderr, "[ov_bridge] drop-late mode 0 (old rule: run-min reference, no cap)\n");
  }

  void log_predict() const {
    if (args_.predict <= 0) return;
    std::vector<float> e = h_pred_;
    std::sort(e.begin(), e.end());
    auto q = [&](double f) { return e.empty() ? -1.0 : static_cast<double>(e[std::min(e.size() - 1, size_t(f * e.size()))]); };
    std::fprintf(stderr, "[ov_bridge] predict: mode %d hz %.0f | lines %llu | horizon ms p50 %.1f p95 %.1f max %.1f\n",
                 args_.predict, args_.predict_hz, static_cast<unsigned long long>(n_pred_), q(0.5), q(0.95),
                 e.empty() ? -1.0 : static_cast<double>(e.back()));
  }

private:
  double to_sec(int64_t t_ns) {
    if (!have_t0_) {
      t0_ns_ = t_ns;
      have_t0_ = true;
    }
    return static_cast<double>(t_ns - t0_ns_) * 1e-9;
  }

  static std::string err_line(const std::string &m) {
    return "{\"type\":\"error\",\"msg\":" + jstr(m) + "}\n";
  }

  // OpenVINS 안에서 난 예외 (예: TrackKLT 의 cv::Exception) → 프로세스를 죽이지 않고 VioManager 를 새로 만든다.
  // 클라이언트에는 auto-reset 과 같은 줄을 보내 맵·궤적 초기화 규칙(on-vio-reset)을 그대로 타게 한다.
  void on_exception(const char *where, const char *what, std::string &out) {
    n_exceptions_++;
    std::fprintf(stderr, "[ov_bridge] exception in %s: %s -> VioManager reset (#%llu)\n", where, what,
                 static_cast<unsigned long long>(n_exceptions_));
    try {
      make_system();
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] make_system failed too: %s\n", e.what());
      throw;
    }
    out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"exception\",\"where\":" + jstr(where) +
           ",\"msg\":" + jstr(std::string(what).substr(0, 300)) + "}\n";
  }

  void on_imu(const uint8_t *p, std::string &out) {
    int64_t t_ns = rd<int64_t>(p);
    double t = to_sec(t_ns);
    if (t <= newest_imu_t_) {  // 역행/중복은 버림
      n_imu_dropped_++;
      return;
    }
    ov_core::ImuData m;
    m.timestamp = t;
    m.wm << rd<double>(p + 8), rd<double>(p + 16), rd<double>(p + 24);
    m.am << rd<double>(p + 32), rd<double>(p + 40), rd<double>(p + 48);
    if (!m.wm.allFinite() || !m.am.allFinite()) {
      n_imu_dropped_++;
      return;
    }
    imu_hold_.push_back(m);
    newest_imu_t_ = t;
    pump(out);
  }

  void on_stereo(const std::vector<uint8_t> &buf, std::string &out) {
    if (buf.size() < 20) {
      out += err_line("STEREO payload too short");
      return;
    }
    const uint8_t *p = buf.data();
    int64_t t_ns = rd<int64_t>(p);
    uint32_t w = rd<uint32_t>(p + 8), h = rd<uint32_t>(p + 12);
    uint8_t n = p[16];
    size_t img = static_cast<size_t>(w) * h;
    if (n < 1 || n > 2 || w == 0 || h == 0 || buf.size() != 20 + n * img) {
      out += err_line("STEREO size mismatch");
      return;
    }
    if (n > params_.state_options.num_cameras) n = static_cast<uint8_t>(params_.state_options.num_cameras);
    // 스테레오 설정에서 mono/stereo 를 섞으면 TrackKLT 가 깨진다: mono 프레임은 cam1 의 이전 피라미드를 갱신하지 않으므로
    // 처음 몇 프레임이 mono 이고 그다음 stereo 가 오면 img_pyramid_last[1] 이 빈 벡터인 채로
    // calcOpticalFlowPyrLK(…, img1pyr=빈 벡터) → "levels2 >= 0" 단언 (cv::Exception) → 프로세스 종료.
    // 그래서 use_stereo=true 면 2장짜리만 받는다 (클라이언트도 섞어 보내지 않음).
    if (params_.use_stereo && params_.state_options.num_cameras >= 2 && n != 2) {
      n_cam_rejected_++;
      return;
    }
    // 크기가 바뀌어도 이전 피라미드와 맞지 않아 KLT 가 깨진다 → 처음 크기만 받는다.
    if (img_w_ == 0) {
      img_w_ = w;
      img_h_ = h;
    } else if (w != img_w_ || h != img_h_) {
      n_cam_rejected_++;
      return;
    }
    CamMsg c;
    c.rx_hdr_boot_ns = rx_hdr_boot_ns_;
    c.rx_boot_ns = rx_boot_ns_;
    c.spin_wait_us = rx_spin_wait_us_;
    c.spin_exit_us = rx_spin_exit_us_;
    c.t_ns = t_ns;
    c.t = to_sec(t_ns);
    c.data.timestamp = c.t;
    for (int i = 0; i < n; i++) {
      // legacy map (no source size): map size == frame size; v2 map: frame size == recorded source size
      bool do_remap = i == 0 && !map_x_.empty() &&
                      (remap_src_w_ > 0 ? (remap_src_w_ == static_cast<int>(w) && remap_src_h_ == static_cast<int>(h))
                                        : (map_x_.cols == static_cast<int>(w) && map_x_.rows == static_cast<int>(h)));
      cv::Mat g;
      if (do_remap && ovopt::g.frame_nocopy) {
        // --ovopt frame_nocopy: remap straight from the record buffer (valid during this call; the remap output owns
        // its pixels, so nothing keeps pointing into buf after this loop iteration)
        g = cv::Mat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, const_cast<uint8_t *>(p + 20 + i * img));
      } else {
        g.create(static_cast<int>(h), static_cast<int>(w), CV_8UC1);
        std::memcpy(g.data, p + 20 + i * img, img);
      }
      if (do_remap) {
        auto r0 = std::chrono::steady_clock::now();
        cv::Mat u;
        cv::remap(g, u, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        g = u;
        c.remap_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - r0).count();
      }
      if (i == 0) c.t_recv = std::chrono::steady_clock::now();  // after the remap: wait_ms = IMU wait + backlog only
      c.data.sensor_ids.push_back(i);
      c.data.images.push_back(g);
      if (params_.use_mask && params_.masks.count(i))
        c.data.masks.push_back(params_.masks.at(i));
      else
        c.data.masks.push_back(cv::Mat::zeros(g.rows, g.cols, CV_8UC1));
    }
    if (!cams_.empty() && c.t <= cams_.back().t) {
      n_cam_dropped_++;
      return;
    }
    if (first_cam_t_ < -1e299) first_cam_t_ = c.t;
    for (const auto &bw : blank_) {  // --test-blank (2026-10-25, TEST ONLY): black cam0 image in the window
      if (c.t - first_cam_t_ >= bw.first && c.t - first_cam_t_ < bw.second) {
        c.data.images[0] = cv::Mat::zeros(c.data.images[0].rows, c.data.images[0].cols, CV_8UC1);
        c.blank = true;
        n_blanked_++;
      }
    }
    cams_.push_back(std::move(c));
    while (cams_.size() > args_.max_cam_queue) {  // 밀리면 오래된 카메라만 버린다 (IMU 는 안 버림)
      cams_.pop_front();
      n_cam_dropped_++;
    }
    pump(out);
  }

  // IMU 와 카메라를 시간 순서대로 VioManager 에 넣는다 (ROS1Visualizer::callback_inertial 과 같은 규칙 + IMU 붙잡기).
  //  1) 맨 앞 카메라보다 늦은 IMU 가 이미 들어갔으면 (t_cam + dt_cam_imu < last_imu) 그 카메라를 넣는다.
  //  2) 기다리는 카메라가 있으면 그 카메라를 넘을 때까지 IMU 를 넣는다.
  //  3) 기다리는 카메라가 없으면 IMU 를 imu_hold_s 동안 붙잡아 둔다 (늦게 오는 카메라보다 IMU 가 한참 앞서 들어가면
  //     초기화 직후 ZUPT 의 IMU 버퍼가 그 카메라 구간을 못 덮어 "No IMU measurements to propagate with (0 of 2)" 경고가 난다:
  //     ZUPT 는 초기화 뒤에 들어온 IMU 만 갖고 있기 때문).
  void pump(std::string &out) {
    while (true) {
      double dt = sys_->get_state()->_calib_dt_CAMtoIMU->value()(0);
      if (!cams_.empty() && cams_.front().t < last_imu_t_ - dt) {
        CamMsg c = std::move(cams_.front());
        cams_.pop_front();
        if (late_drop(c)) continue;  // --drop-late (2026-10-25, default off)
        feed_camera(c, out);
        continue;
      }
      if (!imu_hold_.empty() && (!cams_.empty() || imu_hold_.front().timestamp < newest_imu_t_ - args_.imu_hold_s)) {
        ov_core::ImuData m = imu_hold_.front();
        imu_hold_.pop_front();
        try {
          sys_->feed_measurement_imu(m);
        } catch (const std::exception &e) {
          on_exception("feed_measurement_imu", e.what(), out);
          continue;
        }
        if (shadow_) {
          try {
            shadow_->feed_measurement_imu(m);
          } catch (const std::exception &e) {
            std::fprintf(stderr, "[ov_bridge] init gate: shadow imu exception %s -> new shadow\n", e.what());
            make_shadow(false);
          }
        }
        last_imu_t_ = m.timestamp;
        imu_recent_.push_back(m);  // 상태 주입 재시작 때 새 VioManager 에 다시 넣을 최근 IMU (2 s)
        while (!imu_recent_.empty() && imu_recent_.front().timestamp < m.timestamp - 2.0) imu_recent_.pop_front();
        continue;
      }
      break;
    }
  }

  // --drop-late MS (2026-10-25, ovopt; 0 = off = unchanged): skip a frame (IMU still fed) when it would be fed more
  // than MS later than the best case seen so far: lag = CLOCK_BOOTTIME now - frame t_ns (live: the camera timestamp
  // is on the same clock; replay: + a constant offset), relative to the minimum lag of this run. Stops the backlog
  // spiral once processing exceeds the frame period (phone replay 2026-10-25 "cliff"). Only after initialisation.
  bool late_drop(const CamMsg &c) {
    if (args_.drop_late_ms <= 0.0) return false;
    if (args_.drop_late_mode == 1) return late_drop_v1(c);
    const double lag_ms = static_cast<double>(boot_ns() - c.t_ns) / 1e6;
    if (lag_ms < lag_min_ms_) lag_min_ms_ = lag_ms;
    if (!sys_->initialized() || lag_ms - lag_min_ms_ <= args_.drop_late_ms) return false;
    n_late_dropped_++;
    if (n_late_dropped_ <= 20 || n_late_dropped_ % 100 == 0)
      std::fprintf(stderr, "[ov_bridge] drop-late: frame t=%.3f %.1f ms late (> %.1f) -> skipped (#%llu)\n", c.t,
                   lag_ms - lag_min_ms_, args_.drop_late_ms, static_cast<unsigned long long>(n_late_dropped_));
    return true;
  }

  // --drop-late-mode 1 (2026-10-26, droplatefix): run-min reference + re-anchor after drop_late_win s of only late frames,
  // at most drop_late_cap drops in a row (then one late frame is fed = "forced"). See the header.
  bool late_drop_v1(const CamMsg &c) {
    const double lag_ms = static_cast<double>(boot_ns() - c.t_ns) / 1e6;
    if (lag_ms < lag_min_ms_) lag_min_ms_ = lag_ms;
    const bool late = sys_->initialized() && lag_ms - lag_min_ms_ > args_.drop_late_ms;
    if (!late) {
      if (late_lost_) std::fprintf(stderr, "[ov_bridge] drop-late: t=%.3f lag back under the limit -> late-lost cleared\n", c.t);
      late_run_ = 0;
      late_consec_drop_ = 0;
      late_lost_ = false;
      late_win_.clear();
      return false;
    }
    if (late_run_ == 0) late_run_t0_ = c.t;
    late_run_++;
    late_run_max_ = std::max(late_run_max_, late_run_);
    late_win_.emplace_back(c.t, lag_ms);  // lags of this late run within the last drop_late_win s
    while (!late_win_.empty() && late_win_.front().first < c.t - args_.drop_late_win) late_win_.pop_front();
    double lo = 1e300, hi = -1e300;  // lag range of this late run over the last drop_late_win s
    for (const auto &e : late_win_) {
      lo = std::min(lo, e.second);
      hi = std::max(hi, e.second);
    }
    const bool moving = hi - lo > args_.drop_late_ms;  // falling (backlog draining) or rising (overload / drifting clock)
    if (c.t - late_run_t0_ >= args_.drop_late_win) {  // only late frames for drop_late_win s
      if (!moving) {  // flat over the window: lasting offset shift -> re-anchor (verifier 2026-10-26: a lag that still
        n_late_reanchor_++;  // moves = overload backlog / drift must NOT be re-anchored)
        std::fprintf(stderr, "[ov_bridge] drop-late: re-anchor #%llu at t=%.3f after %d late frames (%.2f s, window range %.1f ms): "
                     "reference %.1f -> %.1f ms (+%.1f)\n", static_cast<unsigned long long>(n_late_reanchor_), c.t, late_run_,
                     c.t - late_run_t0_, hi - lo, lag_min_ms_, lo, lo - lag_min_ms_);
        lag_min_ms_ = lo;
        late_run_ = 0;
        late_consec_drop_ = 0;
        if (late_lost_) std::fprintf(stderr, "[ov_bridge] drop-late: t=%.3f late-lost cleared by the re-anchor\n", c.t);
        late_lost_ = false;
        late_win_.clear();
        if (lag_ms - lag_min_ms_ > args_.drop_late_ms) n_late_forced_++;
        return false;  // fed
      }
      if (!late_lost_ && c.t - late_run_t0_ >= 2.0 * args_.drop_late_win) {  // still moving after 2 windows: not safe
        late_lost_ = true;
        n_late_lost_++;
        std::fprintf(stderr, "[ov_bridge] drop-late: t=%.3f late for %.2f s and lag still moving (window range %.1f ms) -> late-lost "
                     "(outputs marked lost until the lag is back or flat)\n", c.t, c.t - late_run_t0_, hi - lo);
      }
    }
    // cap only while the lag is flat (= drops do not help: offset shift); a moving lag is a backlog the drops drain
    // (old rule, no cap) or a drifting clock (all dropped -> --pred-stale-ms / late-lost mark the outputs lost)
    if (!moving && late_consec_drop_ >= args_.drop_late_cap) {  // cap reached: feed this late frame
      late_consec_drop_ = 0;
      n_late_forced_++;
      if (n_late_forced_ <= 20 || n_late_forced_ % 100 == 0)
        std::fprintf(stderr, "[ov_bridge] drop-late: frame t=%.3f %.1f ms late -> fed anyway after %d drops (forced #%llu)\n", c.t,
                     lag_ms - lag_min_ms_, args_.drop_late_cap, static_cast<unsigned long long>(n_late_forced_));
      return false;
    }
    late_consec_drop_++;
    n_late_dropped_++;
    if (n_late_dropped_ <= 20 || n_late_dropped_ % 100 == 0)
      std::fprintf(stderr, "[ov_bridge] drop-late: frame t=%.3f %.1f ms late (> %.1f) -> skipped (#%llu)\n", c.t,
                   lag_ms - lag_min_ms_, args_.drop_late_ms, static_cast<unsigned long long>(n_late_dropped_));
    return true;
  }
  // --pred-stale-ms for the VIOP heartbeat (main loop): newest IMU too far past the filter state time
  bool hb_stale() const {
    if (pred_stale_s_ <= 0.0 || !sys_ || !sys_->initialized()) return false;
    auto st = sys_->get_state();
    if (st->_timestamp < 0.0) return false;
    return newest_imu_t_ - (st->_timestamp + st->_calib_dt_CAMtoIMU->value()(0)) > pred_stale_s_;
  }

  // --init-gate: 이 카메라 시각까지 init 창 동안 자이로가 조용했나 (넣은 IMU 기준). why 에 이유.
  bool init_still(double t, const char *&why) {
    double W = params_.init_options.init_window_time + 0.05;
    // 카메라 시작 직후 프레임은 그림자에 아예 안 넣으므로 (feed_shadow) 여기서는 따로 안 본다.
    if (imu_recent_.empty() || imu_recent_.front().timestamp > t - kGateMinImuS) {  // OpenVINS 도 창이 덜 차도 초기화함
      why = "short";
      return false;
    }
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    double mx = 0.0;
    int n = 0;
    for (const auto &m : imu_recent_) {
      if (m.timestamp < t - W || m.timestamp > t + 0.05) continue;
      s += m.wm;
      mx = std::max(mx, m.wm.norm());
      n++;
    }
    if (n < 10) {
      why = "short";
      return false;
    }
    gate_last_mean_ = (s / n).norm();
    gate_last_max_ = mx;
    if (gate_last_mean_ > args_.gate_mean || mx > args_.gate_max) {
      why = "moving";
      return false;
    }
    return true;
  }

  void feed_camera(const CamMsg &c, std::string &out) {
    if (args_.test_feed_sleep_ms > 0.0 && first_cam_t_ > -1e299 && c.t - first_cam_t_ >= args_.test_feed_sleep_t0)  // TEST ONLY
      std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(args_.test_feed_sleep_ms * 1000.0)));
    snap_dirty_ = true;              // --pred-thread: publish a new snapshot after this record
    if (plog_) ovopt::prof_reset();  // --prof-log: stage timers of this frame only
    ovopt::health_reset();           // --health counters of this frame only (always counted, cheap)
    scan3d_gyro::frame_stats = scan3d_gyro::Stats();  // ovgyro KLT counters of this frame only (output only)
    scan3d_gyro::pred.health_ok = !health_on_ || hrec_.state == H_OK;  // ovgyro mode 5 guard (read only otherwise)
    auto t0 = std::chrono::steady_clock::now();
    try {
      sys_->feed_measurement_camera(c.data);
    } catch (const std::exception &e) {  // cv::Exception 도 std::exception
      on_exception("feed_measurement_camera", e.what(), out);
      return;
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    scan3d_gyro::add(scan3d_gyro::total, scan3d_gyro::frame_stats);  // ovgyro whole-run counters (output only)
    if (plog_) {  // --prof-log row (timing only)
      bool init = sys_->initialized();
      size_t ns = init ? sys_->get_features_SLAM().size() : 0, nm = init ? sys_->get_good_features_MSCKF().size() : 0;
      std::fprintf(plog_, "%lld,%.6f,%.3f,%d,%zu,%zu", static_cast<long long>(c.t_ns), c.t, ms, init ? 1 : 0, ns, nm);
      for (int s = 0; s < ovopt::N_SLOTS; s++) std::fprintf(plog_, ",%.4f", ovopt::prof_ms[s]);
      for (int k = 0; k < ovopt::N_COUNTS; k++) std::fprintf(plog_, ",%ld", ovopt::prof_cnt[k]);
      std::fprintf(plog_, "\n");
      if (++plog_rows_ % 30 == 0) std::fflush(plog_);
    }
    if (tlog_) {  // --timing-log: one row per fed frame
      double wait_ms = std::chrono::duration<double, std::milli>(t0 - c.t_recv).count();
      bool init = sys_->initialized();
      size_t ns = init ? sys_->get_features_SLAM().size() : 0, nm = init ? sys_->get_good_features_MSCKF().size() : 0;
      std::fprintf(tlog_, "%lld,%.6f,%.3f,%.3f,%.3f,%d,%zu,%zu,%llu,%zu,%lld,%lld,%.1f,%.1f\n", static_cast<long long>(c.t_ns),
                   c.t, wait_ms, c.remap_ms, ms, init ? 1 : 0, ns, nm, static_cast<unsigned long long>(n_cam_dropped_),
                   cams_.size(), static_cast<long long>(c.rx_hdr_boot_ns), static_cast<long long>(c.rx_boot_ns),
                   c.spin_wait_us, c.spin_exit_us);
      std::fflush(tlog_);  // every row: the app may have to kill the bridge (no SIGTERM handler)
      ++tlog_rows_;
    }
    // --init-gate: 그림자 VioManager (기본 쪽이 아직 정지 초기화 전일 때만)
    const char *gate_tag = shadow_ ? feed_shadow(c) : nullptr;
    if (health_on_) {  // --health / --health-log / --recover (2026-10-25): read-only record of this frame
      try {
        health_compute(c);
      } catch (const std::exception &e) {
        std::fprintf(stderr, "[ov_bridge] health exception: %s\n", e.what());
      }
    }
    try {
      std::string pl = pose_line(c, ms);
      if (gate_tag && !pl.empty() && pl.back() == '\n' && pl.size() > 2 && pl[pl.size() - 2] == '}')
        pl = pl.substr(0, pl.size() - 2) + ",\"init_gate\":\"" + gate_tag + "\"}\n";
      out += pl;
    } catch (const std::exception &e) {
      on_exception("pose_line", e.what(), out);
      return;
    }
    if (loop_) loop_after_frame(c);  // --loop (2026-10-26 ovloop): keyframe copy + map output (reads only)
    if (args_.predict > 0 && sys_->initialized()) {  // --predict: serve() emits it once the input is drained
      pred_pending_ = true;
      pred_cam_t_ns_ = c.t_ns;
    }
    if (lost_ && args_.auto_reset) {
      std::string extra;
      if (!(args_.reinit_state && reinit_from_state(extra))) make_system();
      out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"lost\"" + extra + "}\n";
    } else if (args_.recover > 0) {
      try_recover(c, out);  // --recover (2026-10-25, default off)
    }
  }

  // ---- --health (2026-10-25, ovrobust): one read-only record per fed frame ----
  void health_compute(const CamMsg &c) {
    HealthRec r;
    r.c = ovopt::h;
    r.tracked = ovopt::h.tracked;
    r.init = sys_->initialized();
    r.chi2_rej = (r.c.msckf_tri - r.c.msckf_used) + (r.c.slam_clean - r.c.slam_used);
    r.used = r.c.msckf_used + r.c.slam_used;
    if (r.tracked >= 0 && r.tracked < hcfg_.trk_lost) h_trk_low_run_++;
    else if (r.tracked >= 0) h_trk_low_run_ = 0;
    if (!r.init) {
      r.state = H_INIT;
      h_lost_run_ = 0;
      const double W = params_.init_options.init_window_time, tn = last_imu_t_;
      auto half_std = [&](double a, double b) {  // = StaticInitializer sample std of |a - mean|
        Eigen::Vector3d m = Eigen::Vector3d::Zero();
        int n = 0;
        for (const auto &x : imu_recent_)
          if (x.timestamp > a && x.timestamp <= b) { m += x.am; n++; }
        if (n < 2) return static_cast<double>(NAN);
        m /= n;
        double v = 0.0;
        for (const auto &x : imu_recent_)
          if (x.timestamp > a && x.timestamp <= b) v += (x.am - m).squaredNorm();
        return std::sqrt(v / (n - 1));
      };
      if (!imu_recent_.empty() && imu_recent_.front().timestamp <= tn - W) {
        const double s1 = half_std(tn - 0.5 * W, tn), s2 = half_std(tn - W, tn - 0.5 * W);
        r.acc_std = std::max(s1, s2);
        r.init_ready = (s1 < params_.init_options.init_imu_thresh && s2 < params_.init_options.init_imu_thresh &&
                        r.tracked >= 15) ? 1 : 0;
      }
      if (r.init_ready) h_n_ready_++;
    } else {
      auto st = sys_->get_state();
      r.slam = static_cast<long>(sys_->get_features_SLAM().size());
      r.msckf = static_cast<long>(sys_->get_good_features_MSCKF().size());
      const double ts = st->_timestamp;
      if (h_last_upd_t_ < -1e299) h_last_upd_t_ = ts;  // first initialized frame: count from here
      if (r.used > 0) {
        h_last_upd_t_ = ts;
        h_upd_seen_ = true;
      }
      r.since_upd = ts - h_last_upd_t_;
      std::vector<std::shared_ptr<ov_type::Type>> vars = {st->_imu->q(), st->_imu->p(), st->_imu->v()};
      Eigen::MatrixXd C = ov_msckf::StateHelper::get_marginal_covariance(st, vars);  // 9x9 copy, read only
      r.ori_std_deg = std::sqrt(std::max(0.0, C.block(0, 0, 3, 3).trace())) * 180.0 / M_PI;
      r.pos_std = std::sqrt(std::max(0.0, C.block(3, 3, 3, 3).trace()));
      r.vel_std = std::sqrt(std::max(0.0, C.block(6, 6, 3, 3).trace()));
      const Eigen::Vector3d v = st->_imu->vel(), p = st->_imu->pos();
      const bool bad = !v.allFinite() || !p.allFinite() || !std::isfinite(r.pos_std) || v.norm() > args_.max_speed;
      h_state_bad_ = bad;
      const long cand = r.c.msckf_tri + r.c.slam_clean;
      if (bad || r.since_upd > hcfg_.upd_lost || r.pos_std > hcfg_.std_lost || h_trk_low_run_ >= hcfg_.trk_lost_frames)
        r.state = H_LOST;
      else if (r.since_upd > hcfg_.upd_deg || r.pos_std > hcfg_.std_deg || (r.tracked >= 0 && r.tracked < hcfg_.trk_deg) ||
               (cand >= 10 && r.chi2_rej > hcfg_.rej_deg * cand))
        r.state = H_DEGRADED;
      else
        r.state = H_OK;
      // --drop-late-mode 1 (2026-10-26): a frame fed late (forced) or inside a long late run is at least DEGRADED
      if (r.state == H_OK && args_.drop_late_ms > 0.0 && args_.drop_late_mode == 1 && late_run_ > args_.drop_late_cap)
        r.state = H_DEGRADED;
      if (late_lost_) r.state = H_LOST;  // --drop-late mode 1: lag still rising after 2 windows (2026-10-26)
      h_lost_run_ = r.state == H_LOST ? h_lost_run_ + 1 : 0;
    }
    r.lost_run = h_lost_run_;
    // summary counters
    h_n_[r.state]++;
    if (r.state != h_prev_state_) {
      h_transitions_++;
      if (h_prev_state_ >= 0 && h_n_trans_log_ < 200) {
        h_n_trans_log_++;
        std::fprintf(stderr, "[ov_bridge] health t=%.3f %s -> %s (tracked %ld used %ld since_upd %.2f pos_std %.3f)\n", c.t,
                     hstate_name(h_prev_state_), hstate_name(r.state), r.tracked, r.used, r.since_upd, r.pos_std);
      }
      h_prev_state_ = r.state;
    }
    h_max_lost_run_ = std::max(h_max_lost_run_, r.lost_run);
    hrec_ = r;
    if (hlog_) {
      std::fprintf(hlog_, "%lld,%.6f,%s,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.5g,%.5g,%.5g,%.4f,%d,%d,%d,%llu,%d,%.4g\n",
                   static_cast<long long>(c.t_ns), c.t, hstate_name(r.state), r.init ? 1 : 0, r.tracked, r.slam, r.msckf,
                   r.c.msckf_in, r.c.msckf_tri, r.c.msckf_used, r.c.slam_in, r.c.slam_clean, r.c.slam_used, r.c.delay_in,
                   r.c.delay_ok, r.chi2_rej, r.pos_std, r.vel_std, r.ori_std_deg, r.since_upd, r.lost_run, seg_, c.blank ? 1 : 0,
                   static_cast<unsigned long long>(n_recover_state_ + n_recover_full_), r.init_ready, r.acc_std);
      if (++hlog_rows_ % 30 == 0) std::fflush(hlog_);
    }
  }

  void health_json(std::ostringstream &o) const {
    const HealthRec &r = hrec_;
    o << ",\"h\":{\"state\":\"" << hstate_name(r.state) << "\",\"tracked\":" << r.tracked << ",\"used\":" << r.used
      << ",\"chi2_rej\":" << r.chi2_rej << ",\"msckf_in\":" << r.c.msckf_in << ",\"msckf_used\":" << r.c.msckf_used
      << ",\"slam_in\":" << r.c.slam_in << ",\"slam_used\":" << r.c.slam_used << ",\"delay_ok\":" << r.c.delay_ok
      << ",\"pos_std\":";
    jnum(o, r.pos_std);
    o << ",\"vel_std\":";
    jnum(o, r.vel_std);
    o << ",\"ori_std_deg\":";
    jnum(o, r.ori_std_deg);
    o << ",\"since_upd\":";
    jnum(o, r.since_upd);
    o << ",\"lost_run\":" << r.lost_run;
    if (args_.drop_late_ms > 0.0 && args_.drop_late_mode == 1) o << ",\"late_run\":" << late_run_;  // 2026-10-26
    if (!r.init) o << ",\"init_ready\":" << r.init_ready;
    o << "}";
  }

 public:
  // ---- --loop (2026-10-26 ovloop) ----
  // VIO thread, after the pose line: copy a keyframe when the rule fires (reads the filter / tracker only), then the
  // map-frame message for the drone link (VIOP MAP_ODOM or MAVLink DEBUG_VECT), rate limited.
  void loop_after_frame(const CamMsg &c) {
    if (!sys_->initialized() || lost_ || late_lost_ || c.data.images.empty() || c.data.images[0].empty() || c.blank) return;
    auto st = sys_->get_state();
    const double ts = st->_timestamp;
    if (ts < 0.0 || !st->_clones_IMU.count(ts)) return;
    const Eigen::Matrix3d R_ItoG = st->_imu->Rot().transpose();
    const Eigen::Vector3d p_IinG = st->_imu->pos();
    if (g_mav && (last_map_.ver != map_sent_ver_ ||
                  (args_.loop_out_hz > 0.0 && c.t - map_sent_t_ >= 1.0 / args_.loop_out_hz - 1e-6))) {
      g_mav->emit_map(sys_.get(), c.t_ns, last_map_.valid, last_map_.converging, last_map_.yaw, last_map_.t,
                      last_map_.tgt_yaw, last_map_.tgt_t, last_map_.n_loop);
      map_sent_ver_ = last_map_.ver;
      map_sent_t_ = c.t;
    }
    if (!loop_->rule().want(ts, R_ItoG, p_IinG)) return;
    const auto t0 = std::chrono::steady_clock::now();
    std::unique_ptr<ovloop::KfIn> kf(new ovloop::KfIn());
    kf->t_ns = c.t_ns;
    kf->t = ts;
    kf->img = c.data.images[0].clone();
    kf->R_ItoG = R_ItoG;
    kf->p_IinG = p_IinG;
    auto calib0 = st->_calib_IMUtoCAM.at(0);
    kf->R_ItoC = calib0->Rot();
    kf->p_IinC = calib0->pos();
    auto cam = st->_cam_intrinsics_cameras.at(0);
    const Eigen::MatrixXd cv_ = cam->get_value();
    kf->fx = cv_(0);
    kf->fy = cv_(1);
    kf->cx = cv_(2);
    kf->cy = cv_(3);
    for (int i = 4; i < cv_.rows(); i++) kf->dist.push_back(cv_(i));
    kf->fisheye = std::dynamic_pointer_cast<ov_core::CamEqui>(cam) != nullptr;
    for (const auto &cl : st->_clones_IMU) {
      if (cl.first > ts) continue;
      const Eigen::Matrix3d R = cl.second->Rot();  // R_GtoI
      const Eigen::Vector3d p = cl.second->pos();
      kf->clone_t.push_back(cl.first);
      kf->clone_x.push_back({R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), p(0), p(1), p(2)});
    }
    auto tf = std::static_pointer_cast<VioAcc>(sys_)->feats();
    auto obs = tf->get_last_obs();
    auto ids = tf->get_last_ids();
    auto db = tf->get_feature_database();
    if (obs.count(0) && ids.count(0)) {
      const auto &o0 = obs.at(0);
      const auto &i0 = ids.at(0);
      kf->tracks.reserve(o0.size());
      for (size_t i = 0; i < o0.size() && i < i0.size(); i++) {
        ovloop::TrackIn tr;
        tr.id = i0[i];
        tr.u = o0[i].pt.x;
        tr.v = o0[i].pt.y;
        auto sit = st->_features_SLAM.find(tr.id);
        if (sit != st->_features_SLAM.end()) {  // = VioManager::retriangulate_active_tracks (SLAM part)
          const auto &lm = sit->second;
          Eigen::Vector3d pG = lm->get_xyz(false);
          if (ov_type::LandmarkRepresentation::is_relative_representation(lm->_feat_representation)) {
            if (lm->_anchor_cam_id < 0 || !st->_clones_IMU.count(lm->_anchor_clone_timestamp)) continue;
            const Eigen::Matrix3d R_ItoCa = st->_calib_IMUtoCAM.at(lm->_anchor_cam_id)->Rot();
            const Eigen::Vector3d p_IinCa = st->_calib_IMUtoCAM.at(lm->_anchor_cam_id)->pos();
            const auto &an = st->_clones_IMU.at(lm->_anchor_clone_timestamp);
            pG = an->Rot().transpose() * R_ItoCa.transpose() * (lm->get_xyz(false) - p_IinCa) + an->pos();
          }
          tr.slam = true;
          tr.p_G = pG;
        } else {
          ov_core::Feature f;
          if (!db->get_feature_clone(tr.id, f) || !f.timestamps.count(0) || !f.uvs_norm.count(0)) continue;
          const auto &tt = f.timestamps.at(0);
          const auto &un = f.uvs_norm.at(0);
          for (size_t k = 0; k < tt.size() && k < un.size(); k++) {
            if (tt[k] > ts) continue;
            tr.obs_t.push_back(tt[k]);
            tr.obs_x.push_back(un[k](0));
            tr.obs_y.push_back(un[k](1));
          }
          if (tr.obs_t.size() < 3) continue;
        }
        kf->tracks.push_back(std::move(tr));
      }
    }
    kf->cap_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    loop_->push(std::move(kf));
  }
  void log_loop() {
    if (!loop_) return;
    loop_->finish(30.0);
    loop_->report();
  }

  void log_health() const {
    if (!health_on_) return;
    std::fprintf(stderr, "[ov_bridge] health: frames INIT %llu OK %llu DEGRADED %llu LOST %llu | transitions %llu | max LOST run %d "
                 "frames | recover state %llu full %llu | blanked %llu | init_ready frames %llu\n",
                 static_cast<unsigned long long>(h_n_[0]), static_cast<unsigned long long>(h_n_[1]),
                 static_cast<unsigned long long>(h_n_[2]), static_cast<unsigned long long>(h_n_[3]),
                 static_cast<unsigned long long>(h_transitions_), h_max_lost_run_,
                 static_cast<unsigned long long>(n_recover_state_), static_cast<unsigned long long>(n_recover_full_),
                 static_cast<unsigned long long>(n_blanked_), static_cast<unsigned long long>(h_n_ready_));
  }

 private:
  // ---- --recover N (2026-10-25, ovrobust, header) ----
  void try_recover(const CamMsg &c, std::string &out) {
    const HealthRec &r = hrec_;
    // frames in a row with vision back (tracked >= min feats) while LOST: OpenVINS usually resumes by itself within a
    // few frames after a loss of vision (2026-10-25 blank test: 0.1 s) - injecting at the first frame back only resets a
    // filter that was about to recover, so wait --recover N frames with vision back and still no update.
    h_vis_back_run_ = (r.state == H_LOST && r.tracked >= args_.recover_min_feats) ? h_vis_back_run_ + 1 : 0;
    if (!sys_->initialized() || !h_upd_seen_ || h_lost_run_ < args_.recover) return;
    const bool gap_ok = r.since_upd >= 0.0 && r.since_upd <= args_.recover_max_gap;
    if (!h_state_bad_ && gap_ok && h_vis_back_run_ >= args_.recover) {
      std::string extra;
      if (reinit_from_state(extra, args_.recover_max_gap)) {
        n_recover_state_++;
        std::fprintf(stderr, "[ov_bridge] recover #%llu t=%.3f: state injection (since_upd %.2f s, tracked %ld, pos_std %.3f)\n",
                     static_cast<unsigned long long>(n_recover_state_ + n_recover_full_), c.t, r.since_upd, r.tracked, r.pos_std);
        out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"recover\",\"t\":" + std::to_string(c.t) + extra +
               ",\"n_recover\":" + std::to_string(n_recover_state_ + n_recover_full_) + "}\n";
        return;
      }
    }
    if (args_.recover_full && (h_state_bad_ || !gap_ok)) {  // full reset (opt-in); 4-DoF output offset once re-initialized
      if (have_last_out_) {
        off_anchor_R_ = last_out_R_;
        off_anchor_p_ = last_out_p_;
        off_pending_ = true;
      }
      make_system();
      n_recover_full_++;
      std::fprintf(stderr, "[ov_bridge] recover #%llu t=%.3f: full reset (since_upd %.2f s, bad %d, tracked %ld) -> new init, "
                   "offset %s\n", static_cast<unsigned long long>(n_recover_state_ + n_recover_full_), c.t, r.since_upd,
                   h_state_bad_ ? 1 : 0, r.tracked, off_pending_ ? "pending" : "none (no previous pose)");
      out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"recover\",\"reinit\":\"full\",\"t\":" + std::to_string(c.t) +
             ",\"n_recover\":" + std::to_string(n_recover_state_ + n_recover_full_) + "}\n";
    }
    // else: vision not back yet and the gap is still small -> wait
  }

  static double yaw_of(const Eigen::Matrix3d &R) { return std::atan2(R(1, 0), R(0, 0)); }  // R = world <- body
  static Eigen::Matrix3d Rz(double a) {
    Eigen::Matrix3d R;
    R << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1;
    return R;
  }
  // output frame offset (--recover full reset): world_out = Rz(yaw) * world_new + t. No-op when not active.
  void apply_off(Eigen::Matrix3d &R_ItoG, Eigen::Vector3d &p_IinG, Eigen::Vector3d &v, Eigen::Vector4d &q_GtoI) const {
    if (!off_active_) return;
    const Eigen::Matrix3d Rz_ = Rz(off_yaw_);
    R_ItoG = Rz_ * R_ItoG;
    p_IinG = Rz_ * p_IinG + off_t_;
    v = Rz_ * v;
    q_GtoI = ov_core::rot_2_quat(R_ItoG.transpose());
  }

  // --init-gate 그림자 처리 (기본 sys_ 에 카메라를 넣은 직후). 반환: 포즈 줄 표시 ("shadow" = 이번에 그림자로 바꿈) 또는 nullptr.
  const char *feed_shadow(const CamMsg &c) {
    if (sys_->get_state()->_timestamp >= 0.0) {  // 기본 쪽이 스스로 정지 초기화함 → 예전과 같음, 그림자 버림
      std::fprintf(stderr, "[ov_bridge] init gate: default init at t=%.3f -> shadow dropped\n", sys_->get_state()->_timestamp);
      shadow_.reset();
      return nullptr;
    }
    if (first_cam_t_ > -1e299 && c.t - first_cam_t_ < kGateSkipCamS) return nullptr;  // 카메라 시작 직후는 그림자에 안 넣음
    try {
      shadow_->feed_measurement_camera(c.data);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] init gate: shadow camera exception %s -> new shadow\n", e.what());
      make_shadow(true);
      return nullptr;
    }
    if (!shadow_checked_ && shadow_->get_state()->_timestamp >= 0.0) {
      shadow_checked_ = true;
      const char *why = "";
      double ti = shadow_->get_state()->_timestamp;
      if (!init_still(ti, why)) {
        n_init_vetoed_++;
        std::fprintf(stderr, "[ov_bridge] init gate: shadow init at t=%.3f vetoed (%s, gyro mean %.4f max %.3f rad/s) #%llu\n",
                     ti, why, gate_last_mean_, gate_last_max_, static_cast<unsigned long long>(n_init_vetoed_));
        make_shadow(true);
        return nullptr;
      }
      shadow_ok_ = true;
      std::fprintf(stderr, "[ov_bridge] init gate: shadow init at t=%.3f ok (gyro mean %.4f max %.3f rad/s)\n", ti,
                   gate_last_mean_, gate_last_max_);
    }
    if (!shadow_ok_) return nullptr;
    // 움직이기 시작했나: 최근 0.1 s 자이로 최대 > MAX, 또는 그림자가 이미 특징 갱신을 시작함
    double mx = 0.0;
    for (auto it = imu_recent_.rbegin(); it != imu_recent_.rend() && it->timestamp > c.t - 0.1; ++it)
      mx = std::max(mx, it->wm.norm());
    if (mx <= args_.gate_max && !shadow_->initialized()) return nullptr;
    n_shadow_used_++;
    std::fprintf(stderr, "[ov_bridge] init gate: default not initialized at motion start (t=%.3f, gyro %.3f rad/s) -> "
                 "using gated shadow init (t=%.3f) #%llu\n", c.t, mx, shadow_->get_state()->_timestamp,
                 static_cast<unsigned long long>(n_shadow_used_));
    sys_ = shadow_;
    shadow_.reset();
    init_checked_ = true;
    zero_feat_frames_ = 0;
    feat_seen_ = false;
    lost_ = false;
    return "shadow";
  }

  // lost → 새 VioManager 에 옛 상태를 넣어 정지 없이 이어 가기. 성공하면 true, extra 에 JSON 필드 (",\"reinit\":\"state\",...").
  bool reinit_from_state(std::string &extra, double max_gap = -1.0) {
    const bool from_recover = max_gap >= 0.0;
    if (max_gap < 0.0) max_gap = args_.reinit_max_gap;  // --recover passes --recover-max-gap (2026-10-25)
    else last_upd_t_ = h_last_upd_t_;                   // --recover: the health record's last update time
    if (!sys_->initialized() || (!from_recover && lost_bad_)) return false;
    auto st = sys_->get_state();
    double t = st->_timestamp;
    double gap = t - last_upd_t_;
    std::ostringstream o;
    o.precision(6);
    if (!(gap >= 0.0 && gap <= max_gap)) {
      o << ",\"reinit\":\"static\",\"gap\":";
      jnum(o, gap);
      extra = o.str();
      return false;
    }
    Eigen::Matrix<double, 16, 1> imu = st->_imu->value();
    Eigen::Matrix<double, 7, 1> ext = st->_calib_IMUtoCAM.at(0)->value();
    Eigen::VectorXd dt = st->_calib_dt_CAMtoIMU->value();
    if (!imu.allFinite() || !ext.allFinite() || !dt.allFinite()) return false;
    if (from_recover) {
      // --recover (2026-10-25): an injected VioManager never runs its own initialization, which is where upstream raises
      // the tracker from init_max_features (50) to num_pts -> without this it would track only 50 points afterwards.
      // (--reinit-state keeps the old behaviour unchanged.)
      ov_msckf::VioManagerOptions pi = params_;
      pi.init_options.init_max_features = params_.num_pts;
      make_system(&pi);
    } else {
      make_system();
    }
    auto ns = sys_->get_state();
    ns->_calib_IMUtoCAM.at(0)->set_value(ext);
    ns->_calib_IMUtoCAM.at(0)->set_fej(ext);
    ns->_calib_dt_CAMtoIMU->set_value(dt);
    ns->_calib_dt_CAMtoIMU->set_fej(dt);
    Eigen::Matrix<double, 17, 1> s;
    s(0) = t;
    s.block(1, 0, 16, 1) = imu;
    sys_->initialize_with_gt(s);
    shadow_.reset();       // 상태 주입은 정지 초기화가 아님 (--init-gate 그림자 필요 없음)
    init_checked_ = true;
    // initialize_with_gt 의 공분산(속도 0.01 m/s 등)은 정지 가정이라 너무 작다 → IMU 만 전파한 뒤의 불확실성으로 키움
    std::vector<std::shared_ptr<ov_type::Type>> order = {ns->_imu};
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(15, 15);
    C.block(0, 0, 3, 3) = std::pow(args_.reinit_sig_ori_deg * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();
    C.block(3, 3, 3, 3) = std::pow(args_.reinit_sig_pos, 2) * Eigen::Matrix3d::Identity();
    C.block(6, 6, 3, 3) = std::pow(args_.reinit_sig_vel, 2) * Eigen::Matrix3d::Identity();
    C.block(9, 9, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();
    C.block(12, 12, 3, 3) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();
    ov_msckf::StateHelper::set_initial_covariance(ns, C, order);
    // 옛 시스템에 이미 넣은 최근 IMU 를 다시 넣는다 (다음 카메라까지 전파할 IMU 가 새 시스템에 없으므로)
    for (const auto &m : imu_recent_) {
      if (m.timestamp < t - 0.5) continue;
      sys_->feed_measurement_imu(m);
      last_imu_t_ = m.timestamp;
    }
    n_reinit_state_++;
    sys_seeded_ = true;  // armhist 10-29: this W carries the old state -> VIOP_F_SEEDED (does not clear the FC history latch)
    o << ",\"reinit\":\"state\",\"gap\":";
    jnum(o, gap);
    o << ",\"t_state\":";
    jnum(o, t);
    o << ",\"v_norm\":";
    jnum(o, imu.block(7, 0, 3, 1).norm());  // IMU 값 = [q(4) p(3) v(3) bg(3) ba(3)]
    o << ",\"n_reinit_state\":" << n_reinit_state_;
    extra = o.str();
    std::fprintf(stderr, "[ov_bridge] lost -> reinit from state t=%.3f gap=%.2f s |v|=%.2f (#%llu)\n", t, gap,
                 imu.block(7, 0, 3, 1).norm(), static_cast<unsigned long long>(n_reinit_state_));
    return true;
  }

  std::string pose_line(const CamMsg &c, double ms) {
    auto st = sys_->get_state();
    bool init = sys_->initialized();
    size_t n_slam = 0, n_msckf = 0;
    std::vector<Eigen::Vector3d> slam;
    if (init) {
      slam = sys_->get_features_SLAM();
      n_slam = slam.size();
      n_msckf = sys_->get_good_features_MSCKF().size();
    }
    std::ostringstream o;
    o.precision(args_.pose_precision);  // --pose-precision (default 9 = unchanged)
    o << "{\"type\":\"pose\",\"t_ns\":" << c.t_ns << ",\"t\":";
    jnum(o, c.t);
    o << ",\"mono\":" << (c.data.sensor_ids.size() == 1 ? "true" : "false");
    o << ",\"t_state\":";  // OpenVINS state time (s, same base as t) -> client flags stale poses (REVIEW 9)
    jnum(o, st->_timestamp);
    if (!init) {
      o << ",\"status\":\"init\",\"init\":false,\"T_world_cam0\":null";
    } else {
      Eigen::Vector4d q_GtoI = st->_imu->quat();  // JPL
      Eigen::Matrix3d R_GtoI = ov_core::quat_2_Rot(q_GtoI);
      Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
      Eigen::Vector3d p_IinG = st->_imu->pos(), v = st->_imu->vel();
      Eigen::Quaterniond qh(R_ItoG);  // Hamilton, world <- IMU
      auto calib0 = st->_calib_IMUtoCAM.at(0);
      Eigen::Matrix3d R_ItoC = calib0->Rot();
      Eigen::Vector3d p_IinC = calib0->pos();
      if (off_pending_) {  // --recover full reset (2026-10-25): first pose of the new segment -> the anchor (yaw + xyz)
        off_yaw_ = yaw_of(off_anchor_R_) - yaw_of(R_ItoG);
        off_t_ = off_anchor_p_ - Rz(off_yaw_) * p_IinG;
        off_active_ = true;
        off_pending_ = false;
        seg_++;
        std::fprintf(stderr, "[ov_bridge] recover: segment %d frame offset yaw %.2f deg t %.3f %.3f %.3f m\n", seg_,
                     off_yaw_ * 180.0 / M_PI, off_t_(0), off_t_(1), off_t_(2));
      }
      apply_off(R_ItoG, p_IinG, v, q_GtoI);  // no-op unless a --recover offset is active
      qh = Eigen::Quaterniond(R_ItoG);
      Eigen::Matrix3d R_CtoG = R_ItoG * R_ItoC.transpose();
      Eigen::Vector3d p_CinG = p_IinG - R_CtoG * p_IinC;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = R_CtoG;
      T.block<3, 1>(0, 3) = p_CinG;
      if (p_IinG.allFinite() && R_ItoG.allFinite()) {  // --recover anchor for a later full reset
        last_out_R_ = R_ItoG;
        last_out_p_ = p_IinG;
        have_last_out_ = true;
      }

      bool bad = !p_IinG.allFinite() || !v.allFinite() || !R_CtoG.allFinite() || !T.allFinite() ||
                 v.norm() > args_.max_speed;
      zero_feat_frames_ = (n_slam + n_msckf == 0) ? zero_feat_frames_ + 1 : 0;
      if (n_slam + n_msckf > 0) {
        feat_seen_ = true;
        last_upd_t_ = st->_timestamp;  // 마지막으로 영상이 상태를 고친 시각 (상태 주입 재시작의 gap 기준)
      }
      lost_bad_ = bad;
      // 초기화 직후(정지 ZUPT 가 끝나고 처음 움직일 때)는 clone 창이 차고 시차가 생길 때까지 특징 갱신이 0 인 게 정상이다
      // (rec_20261016_143702: 10~14 프레임, 천천히 움직이기 시작하면 더 길다). 첫 특징 갱신 전에는 startup_lost_frames 까지 기다린다.
      int lim = feat_seen_ ? args_.lost_frames : std::max(args_.lost_frames, args_.startup_lost_frames);
      lost_ = bad || zero_feat_frames_ >= lim;

      o << ",\"status\":\"" << ((lost_ || late_lost_) ? "lost" : "tracking") << "\",\"init\":true";  // late_lost_: 2026-10-26
      o << ",\"p\":";
      jarr(o, p_IinG);
      o << ",\"q\":[";
      jnum(o, qh.x()); o << ","; jnum(o, qh.y()); o << ","; jnum(o, qh.z()); o << ","; jnum(o, qh.w());
      o << "],\"v\":";
      jarr(o, v);
      o << ",\"T_world_cam0\":";
      jarr(o, T);
      o << ",\"q_GtoI_jpl\":";
      jarr(o, q_GtoI);
      o << ",\"bg\":";
      jarr(o, st->_imu->bias_g());
      o << ",\"ba\":";
      jarr(o, st->_imu->bias_a());
      o << ",\"R_ItoC0\":";
      jarr(o, R_ItoC);
      o << ",\"p_IinC0\":";
      jarr(o, p_IinC);
      o << ",\"intr0\":";                                   // 2026-10-23: cam0 intrinsics (online calib check)
      jarr(o, st->_cam_intrinsics.at(0)->value());
      if (loop_) {  // --loop (2026-10-26 ovloop): map-frame pose; the smoothing steps once per frame (frame time)
        last_map_ = loop_->step_output(c.t, p_IinG);
        map_fields(o, last_map_, R_ItoG, p_IinG, v, &R_ItoC, &p_IinC);
      }
    }
    o << ",\"dt_cam_imu\":";
    jnum(o, st->_calib_dt_CAMtoIMU->value()(0));
    o << ",\"n_slam\":" << n_slam << ",\"n_msckf\":" << n_msckf << ",\"ms\":";
    jnum(o, ms);
    o << ",\"remap_ms\":";
    jnum(o, c.remap_ms);
    o << ",\"cam_dropped\":" << n_cam_dropped_ << ",\"imu_dropped\":" << n_imu_dropped_
      << ",\"cam_rejected\":" << n_cam_rejected_ << ",\"exceptions\":" << n_exceptions_
      << ",\"imu_held\":" << imu_hold_.size() << ",\"n_reinit_state\":" << n_reinit_state_;
    if (args_.drop_late_ms > 0.0) o << ",\"late_dropped\":" << n_late_dropped_;  // --drop-late only (2026-10-25)
    if (args_.health && health_on_) health_json(o);                                // --health (2026-10-25)
    if (args_.klt_stats) {  // --klt-stats (2026-10-26 ovgyro): in, LK ok, kept, mean predicted shift px, fallback try/ok, rotation
      const auto &k = scan3d_gyro::frame_stats;
      o << ",\"klt\":[" << k.pts_in << "," << k.klt_ok << "," << k.pts_out << ",";
      jnum(o, k.shift_n ? k.shift_sum / k.shift_n : 0.0);
      o << "," << k.fb_try << "," << k.fb_ok << "," << (scan3d_gyro::pred.have ? 1 : 0) << ",";
      jnum(o, k.res_n ? k.res_sum / k.res_n : 0.0);   // mean |LK result - start| px
      o << ",";
      jnum(o, k.res_n ? k.flow_sum / k.res_n : 0.0);  // mean |LK result - old| px
      for (double v : {k.shift_n ? k.pdx / k.shift_n : 0.0, k.shift_n ? k.pdy / k.shift_n : 0.0, k.res_n ? k.fdx / k.res_n : 0.0,
                       k.res_n ? k.fdy / k.res_n : 0.0}) {  // mean predicted (x, y) and mean LK flow (x, y) px
        o << ",";
        jnum(o, v);
      }
      o << "]";
    }
    if (args_.recover > 0 && (seg_ > 0 || off_active_)) {                         // --recover frame offset
      o << ",\"seg\":" << seg_ << ",\"frame_off\":[";
      jnum(o, off_yaw_); o << ","; jnum(o, off_t_(0)); o << ","; jnum(o, off_t_(1)); o << ","; jnum(o, off_t_(2));
      o << "]";
    }
    if (c.blank) o << ",\"blank\":true";                                            // --test-blank
    if (args_.emit_features > 0 && init) {
      o << ",\"slam_pts\":[";
      size_t k = std::min(slam.size(), static_cast<size_t>(args_.emit_features));
      for (size_t i = 0; i < k; i++) {
        if (i) o << ",";
        jarr(o, slam[i]);
      }
      o << "]";
    }
    o << "}\n";
    return o.str();
  }

  Args args_;
  std::string cv_parallel_ = "?";  // cv::getBuildInformation "Parallel framework:" value
  cv::Mat map_x_, map_y_;  // --remap (CV_16SC2 + CV_16UC1 고정소수 맵, 빠름)

  void load_remap(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "rb");
    int32_t wh[2] = {0, 0};
    if (!f || std::fread(wh, 4, 2, f) != 2 || wh[0] <= 0 || wh[1] <= 0) {
      std::fprintf(stderr, "[ov_bridge] remap: cannot read %s\n", path.c_str());
      if (f) std::fclose(f);
      return;
    }
    cv::Mat mx(wh[1], wh[0], CV_32FC1), my(wh[1], wh[0], CV_32FC1);
    size_t n = static_cast<size_t>(wh[0]) * wh[1];
    bool ok = std::fread(mx.data, 4, n, f) == n && std::fread(my.data, 4, n, f) == n;
    int32_t src[2] = {0, 0};  // v2: optional source frame size after the maps
    if (ok && std::fread(src, 4, 2, f) == 2 && src[0] > 0 && src[1] > 0) {
      remap_src_w_ = src[0];
      remap_src_h_ = src[1];
    }
    std::fclose(f);
    if (!ok) {
      std::fprintf(stderr, "[ov_bridge] remap: short file %s\n", path.c_str());
      return;
    }
    cv::convertMaps(mx, my, map_x_, map_y_, CV_16SC2);
    if (ovopt::g.rs_derot != 0 && remap_src_h_ > 0) {  // rs_derot: exact sensor row of each fed pixel (2026-10-25)
      rs_mapy_ = my;
      ovopt::rs_mapy = reinterpret_cast<const float *>(rs_mapy_.data);
      ovopt::rs_map_w = wh[0];
      ovopt::rs_map_h = wh[1];
      ovopt::rs_src_h = remap_src_h_;
      std::fprintf(stderr, "[ov_bridge] rs_derot: readout %.1f us, sensor rows from the remap y table (source h %d)\n",
                   ovopt::rs_readout_s() * 1e6, remap_src_h_);
    }
    std::fprintf(stderr, "[ov_bridge] remap %dx%d (source %dx%d) from %s\n", wh[0], wh[1],
                 remap_src_w_ > 0 ? remap_src_w_ : wh[0], remap_src_h_ > 0 ? remap_src_h_ : wh[1], path.c_str());
  }
  int remap_src_w_ = 0, remap_src_h_ = 0;  // v2 remap file: frames of this size are remapped (0 = map size)
  cv::Mat rs_mapy_;                        // rs_derot: float copy of the remap y table (only with the option)
  FILE *tlog_ = nullptr;                   // --timing-log
  FILE *plog_ = nullptr;                   // --prof-log (2026-10-25)
  uint64_t plog_rows_ = 0;
  int64_t rx_hdr_boot_ns_ = 0, rx_boot_ns_ = 0;     // set_rx
  double rx_spin_wait_us_ = -1.0, rx_spin_exit_us_ = -1.0;
  uint64_t tlog_rows_ = 0;
  bool pred_pending_ = false;            // --predict: a frame update since the last prediction
  bool snap_dirty_ = false;              // --pred-thread: state changed since the last make_snap
  double last_pred_imu_t_ = -1e300;      // newest_imu_t_ of the last prediction
  int64_t pred_cam_t_ns_ = 0;            // t_ns of the last frame pose
  uint64_t n_pred_ = 0;
  std::vector<float> h_pred_;            // horizons (ms)

public:
  ~Bridge() {
    if (tlog_) std::fclose(tlog_);
    if (plog_) std::fclose(plog_);
    if (hlog_) std::fclose(hlog_);
  }

private:

  bool ok_ = true;
  std::shared_ptr<ov_core::YamlParser> parser_;
  ov_msckf::VioManagerOptions params_;
  std::shared_ptr<ov_msckf::VioManager> sys_;
  std::deque<CamMsg> cams_;
  bool have_t0_ = false;
  int64_t t0_ns_ = 0;
  double last_imu_t_ = -1e300;    // VioManager 에 마지막으로 넣은 IMU 시각
  double newest_imu_t_ = -1e300;  // 받은 IMU 중 가장 새 시각 (붙잡아 둔 것 포함)
  std::deque<ov_core::ImuData> imu_hold_;
  uint32_t img_w_ = 0, img_h_ = 0;
  int zero_feat_frames_ = 0;
  bool feat_seen_ = false;  // 이 VioManager 가 특징 갱신(SLAM/MSCKF > 0)을 한 번이라도 했는지
  bool lost_ = false;
  bool lost_bad_ = false;           // lost 원인이 속도 초과·NaN (상태 주입 재시작 안 함)
  double last_upd_t_ = -1e300;      // 마지막 특징 갱신 시각 (make_system 이 지우지 않음: 주입 뒤 또 끊기면 gap 이 커져 정지 초기화로)
  std::deque<ov_core::ImuData> imu_recent_;
  uint64_t n_reinit_state_ = 0;
  bool sys_seeded_ = false;  // armhist 10-29: sys_ was seeded by reinit_from_state
  uint64_t n_init_vetoed_ = 0;
  bool init_checked_ = false;  // 이 VioManager 의 초기화를 --init-gate 로 확인했나
  bool use_shadow_ = false;    // --init-gate 켜짐
  ov_msckf::VioManagerOptions shadow_params_;           // 그림자 설정 (넓힌 시차 기준)
  std::shared_ptr<ov_msckf::VioManager> shadow_;        // 초기화 전에만 있음
  bool shadow_checked_ = false, shadow_ok_ = false;     // 그림자 초기화를 자이로로 확인했나 / 통과했나
  uint64_t n_shadow_used_ = 0;
  double gate_last_mean_ = 0.0, gate_last_max_ = 0.0;
  double first_cam_t_ = -1e300;                   // 받은 첫 카메라 시각 (--init-gate). RESET 때 지움
  static constexpr double kGateSkipCamS = 0.8;   // --init-gate: 카메라 시작 뒤 이 시간 동안의 프레임은 그림자에 안 넣음
  static constexpr double kGateMinImuS = 0.8;    // --init-gate: 확인에 쓸 IMU 최소 길이 (s)
  uint64_t n_cam_dropped_ = 0, n_imu_dropped_ = 0, n_cam_rejected_ = 0, n_exceptions_ = 0;
  uint64_t n_late_dropped_ = 0;     // --drop-late
  // ---- 2026-10-25 ovrobust: --health / --recover / --test-blank ----
  bool health_on_ = false;
  HealthCfg hcfg_;
  HealthRec hrec_;                  // record of the latest fed frame
  FILE *hlog_ = nullptr;
  uint64_t hlog_rows_ = 0;
  double h_last_upd_t_ = -1e300;    // state time of the last frame whose update used >= 1 feature (per VioManager)
  bool h_upd_seen_ = false;         // this VioManager has done >= 1 feature update
  bool h_state_bad_ = false;
  int h_lost_run_ = 0, h_trk_low_run_ = 0, h_max_lost_run_ = 0;
  int h_prev_state_ = -1;
  uint64_t h_n_[4] = {0, 0, 0, 0}, h_transitions_ = 0, h_n_trans_log_ = 0;
  uint64_t n_recover_state_ = 0, n_recover_full_ = 0, n_blanked_ = 0, h_n_ready_ = 0;
  int h_vis_back_run_ = 0;
  std::vector<std::pair<double, double>> blank_;  // --test-blank windows (s after the first camera record)
  bool have_last_out_ = false, off_pending_ = false, off_active_ = false;
  Eigen::Matrix3d last_out_R_ = Eigen::Matrix3d::Identity(), off_anchor_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_out_p_ = Eigen::Vector3d::Zero(), off_anchor_p_ = Eigen::Vector3d::Zero();
  double off_yaw_ = 0.0;
  Eigen::Vector3d off_t_ = Eigen::Vector3d::Zero();
  int seg_ = 0;
  double lag_min_ms_ = 1e300;       // --drop-late: smallest feed lag seen (mode 1: current reference, re-anchored)
  // --drop-late-mode 1 (2026-10-26 droplatefix)
  int late_run_ = 0, late_consec_drop_ = 0, late_run_max_ = 0;  // consecutive late frames (dropped + forced) / drops
  double late_run_t0_ = 0.0;
  std::deque<std::pair<double, double>> late_win_;  // (frame t, lag ms) of the current late run, last drop_late_win s
  bool late_lost_ = false;          // late for > 2 windows with a rising lag -> outputs marked lost
  uint64_t n_late_lost_ = 0;
  uint64_t n_late_forced_ = 0, n_late_reanchor_ = 0, n_pred_stale_ = 0;
  double pred_stale_s_ = 0.0;       // --pred-stale-ms / 1000 (resolved in the constructor)
  // --loop (2026-10-26 ovloop)
  std::unique_ptr<ovloop::LoopCloser> loop_;
  ovloop::MapOdom last_map_;          // published value of the latest pose line
  uint32_t map_sent_ver_ = 0;
  double map_sent_t_ = -1e300;
};

// 한 연결(또는 stdin) 을 끝까지 처리. 반환 false = QUIT 받음.
bool serve(Bridge &b, int in_fd, int out_fd, bool is_socket) {
  std::vector<uint8_t> buf;
  std::string out;
  uint32_t hdr[3];
  uint64_t n_cam_rec = 0;
  std::vector<uint8_t> ca_buf;  // chunked image being assembled (types 5/6)
  bool ca_active = false;
  int64_t ca_t_hdr = 0;
  // --idle-spin 2: stripes = OpenCV pool size (VioManager ctor already called cv::setNumThreads)
  g_idle.stripes = std::max(1, cv::getNumThreads());
  std::fprintf(stderr, "[ov_bridge] idle spin: mode %d (%s) stripes %d max %.0f ms", g_idle.mode,
               idle_spin_name(g_idle.mode), g_idle.stripes, kIdleSpinMaxMs);
  if (g_idle.win_ms > 0.0) std::fprintf(stderr, " window %.1f ms", g_idle.win_ms);  // --idle-spin-ms (2026-10-25)
  std::fprintf(stderr, "\n");
  if (!write_full(out_fd, b.ready_line(), is_socket)) return true;
  while (true) {
    g_idle.wait(in_fd);  // --idle-spin: busy-wait for input (mode 0: no-op, read() blocks)
    if (!read_full(in_fd, hdr, sizeof(hdr))) break;
    const int64_t t_hdr = boot_ns();
    if (hdr[1] == kStereo && g_idle.win_ms > 0.0) g_idle.note_frame(mono_ns());  // --idle-spin-ms frame rhythm
    if (hdr[0] != kMagic || hdr[2] > kMaxPayload) {
      write_full(out_fd, "{\"type\":\"error\",\"msg\":\"bad header, closing\"}\n", is_socket);
      return true;
    }
    int64_t t_hdr_rec = t_hdr;
    if (hdr[1] == kChunk || hdr[1] == kChunkEnd) {  // chunked image (2026-10-25, header): append, handle at CHUNK_END
      if (!ca_active) {
        ca_active = true;
        ca_buf.clear();
        ca_t_hdr = t_hdr;
        if (g_idle.win_ms > 0.0) g_idle.note_frame(mono_ns());
      }
      const size_t o0 = ca_buf.size();
      if (o0 + hdr[2] > kMaxPayload) {
        write_full(out_fd, "{\"type\":\"error\",\"msg\":\"chunked payload too large, closing\"}\n", is_socket);
        return true;
      }
      ca_buf.resize(o0 + hdr[2]);
      if (hdr[2] && !read_full(in_fd, ca_buf.data() + o0, hdr[2])) break;
      if (hdr[1] == kChunk) continue;
      ca_active = false;
      buf.swap(ca_buf);
      hdr[1] = kStereo;
      t_hdr_rec = ca_t_hdr;
    } else {
      buf.resize(hdr[2]);
      if (hdr[2] && !read_full(in_fd, buf.data(), hdr[2])) break;
    }
    if (g_imu_log && hdr[1] == kImu && buf.size() == 56) imu_log_row(buf.data(), t_hdr, boot_ns());  // --imu-log
    b.set_rx(t_hdr_rec, boot_ns(), g_idle.mode ? g_idle.last_wait_us : -1.0, g_idle.last_exit_us);
    out.clear();
    bool cont = true;
    if (hdr[1] == kStereo && ++n_cam_rec == 300) {
      apply_affinity(g_pinned ? &g_pin : nullptr, "cam300");
      if (g_idle.mode) g_idle.log("cam300");
    }
    try {
      cont = b.handle(hdr[1], buf, out);
    } catch (const std::exception &e) {  // 마지막 방어선: 어떤 예외로도 프로세스가 죽지 않게
      std::fprintf(stderr, "[ov_bridge] unhandled exception: %s\n", e.what());
      out += "{\"type\":\"error\",\"msg\":" + jstr(std::string("exception: ") + e.what()) + "}\n";
    }
    if (!out.empty() && !write_full(out_fd, out, is_socket)) return true;
    if (!cont) return false;
    // --predict: once nothing more is waiting on the input (the IMU that queued up during the frame has been read)
    if (b.predict_due() && !fd_ready(in_fd)) {
      out.clear();
      try {
        b.predict(out);
      } catch (const std::exception &e) {
        std::fprintf(stderr, "[ov_bridge] predict exception: %s\n", e.what());
      }
      if (!out.empty() && !write_full(out_fd, out, is_socket)) return true;
    }
  }
  return true;
}

// --pred-thread input reader (header): reads every record, assembles chunked images, gives IMU to the predictor at
// once, queues the records in order for the main thread.
void reader_main(int fd, InQueue *q, Predictor *pred) {
  std::vector<uint8_t> ca;
  bool ca_active = false;
  int64_t ca_hb = 0, ca_hm = 0;
  uint32_t hdr[3];
  auto finish = [&](uint32_t type) {
    InRec e;
    e.type = type;
    q->push(std::move(e));
    timespec ts;
    ::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    if (pred) pred->reader_cpu_s = ts.tv_sec + ts.tv_nsec * 1e-9;
  };
  while (true) {
    if (!read_full(fd, hdr, sizeof(hdr))) break;
    const int64_t tb = boot_ns(), tm = mono_ns();
    if (hdr[0] != kMagic || hdr[2] > kMaxPayload) {
      finish(kBad);
      return;
    }
    const bool chunk = hdr[1] == kChunk || hdr[1] == kChunkEnd;
    if (hdr[1] == kStereo || (chunk && !ca_active)) q->wait_space();  // a new image: at most kMaxPendingImg queued
    if (chunk) {
      if (!ca_active) {
        ca_active = true;
        ca = q->take_buf();  // reuse (capacity kept, no new pages)
        ca.clear();
        ca_hb = tb;
        ca_hm = tm;
      }
      const size_t o0 = ca.size();
      if (o0 + hdr[2] > kMaxPayload) {
        finish(kBad);
        return;
      }
      ca.resize(o0 + hdr[2]);
      if (hdr[2] && !read_full(fd, ca.data() + o0, hdr[2])) break;
      if (hdr[1] == kChunk) continue;
      ca_active = false;
      InRec r;
      r.type = kStereo;
      r.buf.swap(ca);
      r.t_hdr_boot = ca_hb;
      r.t_hdr_mono = ca_hm;
      r.t_rx_boot = boot_ns();
      q->push(std::move(r));
      continue;
    }
    InRec r;
    r.type = hdr[1];
    if (hdr[2] >= (1u << 20)) r.buf = q->take_buf();  // unchunked image: reuse a buffer
    r.buf.resize(hdr[2]);
    if (hdr[2] && !read_full(fd, r.buf.data(), hdr[2])) break;
    r.t_hdr_boot = tb;
    r.t_hdr_mono = tm;
    r.t_rx_boot = boot_ns();
    if (r.type == kImu && r.buf.size() == 56) {
      imu_log_row(r.buf.data(), tb, r.t_rx_boot);
      if (pred) pred->push_imu(r.buf.data());
      if (mav::Out *ho = g_hb_feed_out.load(std::memory_order_acquire)) ho->note_feed();  // esp32fix: --viop-hb-src 1 only
    }
    const bool quit = r.type == kQuit;
    q->push(std::move(r));
    if (quit) {
      finish(kEof);
      return;
    }
  }
  finish(kEof);
}

// --pred-thread main loop (stdio): same record handling as serve(), records come from the reader thread's queue.
bool serve_q(Bridge &b, int in_fd, int out_fd, Predictor *pred) {
  auto *q = new InQueue();  // not freed if the reader is still blocked in read() at QUIT (process ends anyway)
  g_inq = q;
  std::string out;
  uint64_t n_cam_rec = 0;
  g_idle.stripes = std::max(1, cv::getNumThreads());
  std::fprintf(stderr, "[ov_bridge] idle spin: mode %d (%s) stripes %d max %.0f ms", g_idle.mode,
               idle_spin_name(g_idle.mode), g_idle.stripes, kIdleSpinMaxMs);
  if (g_idle.win_ms > 0.0) std::fprintf(stderr, " window %.1f ms", g_idle.win_ms);
  std::fprintf(stderr, " | pred thread (input reader thread on)\n");
  if (!write_full(out_fd, b.ready_line(), false)) return true;
  if (pred) pred->start();
  std::thread rd([q, in_fd, pred] { reader_main(in_fd, q, pred); });
  bool ret = true, write_fail = false;
  while (true) {
    g_idle.wait(in_fd);  // --idle-spin on the queue (mode 0: pop() blocks)
    InRec r = q->pop();
    if (r.type == kEof) break;
    if (r.type == kBad) {
      std::lock_guard<std::mutex> lo(g_out_m);
      write_full(out_fd, "{\"type\":\"error\",\"msg\":\"bad header, closing\"}\n", false);
      break;
    }
    if (r.type == kStereo && g_idle.win_ms > 0.0) g_idle.note_frame(r.t_hdr_mono);
    if (r.type == kReset && pred) pred->reset_imu();  // (IMU read after the RESET may already be in: dropped once, rare)
    b.set_rx(r.t_hdr_boot, r.t_rx_boot, g_idle.mode ? g_idle.last_wait_us : -1.0, g_idle.last_exit_us);
    out.clear();
    bool cont = true;
    if (r.type == kStereo && ++n_cam_rec == 300) {
      apply_affinity(g_pinned ? &g_pin : nullptr, "cam300");
      if (g_idle.mode) g_idle.log("cam300");
    }
    try {
      cont = b.handle(r.type, r.buf, out);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] unhandled exception: %s\n", e.what());
      out += "{\"type\":\"error\",\"msg\":" + jstr(std::string("exception: ") + e.what()) + "}\n";
    }
    q->give_buf(std::move(r.buf));  // handle() keeps no pointer into the record (frame_nocopy: remap output owns pixels)
    if (!out.empty()) {
      std::lock_guard<std::mutex> lo(g_out_m);
      if (!write_full(out_fd, out, false)) {
        write_fail = true;
        break;
      }
    }
    if (pred && b.snap_dirty()) {  // after the pose line: the prediction thread continues from the new state
      PredSnap s;
      b.make_snap(s);
      if (s.valid) pred->set_snap(s);
      else pred->invalidate();
    }
    if (!cont) {
      ret = false;
      break;
    }
  }
  if (pred) pred->stop();
  std::fprintf(stderr, "[ov_bridge] pred thread reader: image space waits %llu\n",
               static_cast<unsigned long long>(q->space_waits));
  if (write_fail) rd.detach();  // the reader may still block in read(); the process is about to end
  else rd.join();                // EOF / bad header / QUIT: the reader has returned or returns now
  g_inq = nullptr;
  return ret;
}

void usage() {
  std::fprintf(stderr,
               "usage: ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]\n"
               "                 [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]\n"
               "                 [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]\n"
               "                 [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG] [--timing-log FILE] [--cpus LIST] [--threads N]\n"
               "                 [--klt-overlap 0|1] [--klt-pyr N] [--idle-spin 0|1|2|3] [--idle-spin-ms W] [--predict 0|1|2] [--predict-hz HZ]\n"
               "                 [--ovopt LIST] [--prof-log FILE] [--pose-precision N] [--drop-late MS]\n"
               "                 [--drop-late-mode 0|1] [--drop-late-cap N] [--drop-late-win S] [--pred-stale-ms MS]\n"
               "                 [--pred-thread 0|1] [--imu-log FILE] [--profile rec20261025|rec60|none]\n"
               "                 [--mav-udp HOST:PORT | --viop-udp HOST:PORT] [--viop-hb-hz HZ] [--viop-hb-src 0|1]\n"
               "                 [--init-still 0|1] [--health 0|1] [--health-log FILE] [--health-cfg K=V,..] [--klt-stats 0|1] [--pipe 0] [--recover N]\n"
               "                 [--recover-max-gap S] [--recover-min-feats N] [--recover-full 0|1] [--test-blank T0:D,..]\n"
               "                 [--loop 0|1 --loop-vocab FILE [--loop-cfg K=V,..] [--loop-log FILE] [--loop-sync 0|1] [--loop-out-hz HZ] [--loop-dump DIR] [--loop-vpr FILE] [--loop-xf CMD]]\n");
}

}  // namespace

int main(int argc, char **argv) {
  signal(SIGPIPE, SIG_IGN);
  if (argc < 2) {
    usage();
    return 2;
  }
  Args a;
  a.config = argv[1];
  // --profile NAME (integ-20261025): preset values first, explicit flags below override them
  for (int i = 2; i + 1 < argc; i++) {
    if (std::string(argv[i]) != "--profile") continue;
    const std::string pn = argv[i + 1];
    if (pn == "rec20261025" || pn == "rec60") {  // rec60 = same with idle-spin-ms 0 (60 fps: window > period)
      // integ2-20261026: + ovupd sym_inplace (exact), ekf_sparse (near-exact); user 2026-10-26 morning: + gyro_klt=5 (ovgyro, changes poses)
      a.ovopt = "all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse,gyro_klt=5";
      a.idle_spin = 2;
      a.idle_spin_ms = 20.0;
      a.drop_late_ms = 50.0;
      a.health = 1;  // ovrobust health status (read only: poses identical; pose lines get "h")
      a.predict = 2;  // user 2026-10-25 19:40: 200 Hz IMU-forward prediction from the prediction thread (read only)
      a.predict_hz = 200.0;
      a.pred_thread = 1;  // dropped again below for --tcp (needs stdio)
      // integ2 0.6.28 (user 2026-10-26, 60 fps only): rec60 + loop closure (BoW v1, own thread; VIO/control pose unchanged,
      // adds "map" fields, VIOP MAP_ODOM 0x07, MAVLink DEBUG_VECT). rec20261025 (30 fps) unchanged. Vocabulary: --loop-vocab,
      // else voc_k10L4.voc next to the config (or its parent dir, or this build's source tree) - resolved below.
      // --loop 0 turns it off; dropped with --recover.
      if (pn == "rec60") a.loop = 1;
      if (pn == "rec60") a.idle_spin_ms = 0.0;
    } else if (pn != "none") {
      std::fprintf(stderr, "[ov_bridge] unknown --profile %s (known: rec20261025, rec60, none)\n", pn.c_str());
      return 2;
    }
    a.profile = pn;
  }
  for (int i = 2; i < argc; i++) {
    std::string s = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        std::exit(2);
      }
      return argv[++i];
    };
    if (s == "--stdio") a.tcp = false;
    else if (s == "--tcp") {
      a.tcp = true;
      std::string hp = next();
      auto c = hp.rfind(':');
      if (c == std::string::npos) a.port = std::stoi(hp);
      else {
        a.host = hp.substr(0, c);
        a.port = std::stoi(hp.substr(c + 1));
      }
    } else if (s == "--max-cam-queue") a.max_cam_queue = static_cast<size_t>(std::max(1, std::stoi(next())));
    else if (s == "--max-speed") a.max_speed = std::stod(next());
    else if (s == "--lost-frames") a.lost_frames = std::stoi(next());
    else if (s == "--startup-lost-frames") a.startup_lost_frames = std::stoi(next());
    else if (s == "--auto-reset") a.auto_reset = true;
    else if (s == "--emit-features") a.emit_features = std::stoi(next());
    else if (s == "--imu-hold") a.imu_hold_s = std::max(0.0, std::stod(next()));
    else if (s == "--reinit-state") a.reinit_state = true;
    else if (s == "--remap") a.remap = next();
    else if (s == "--timing-log") a.timing_log = next();
    else if (s == "--cpus") a.cpus = next();
    else if (s == "--threads") a.threads = std::max(0, std::stoi(next()));
    else if (s == "--klt-overlap") a.klt_overlap = std::stoi(next()) != 0 ? 1 : 0;
    else if (s == "--profile") next();  // integ-20261025 (applied above)
    else if (s == "--klt-pyr") a.klt_pyr = std::max(0, std::min(8, std::stoi(next())));  // 2026-10-25 ovheat
    else if (s == "--idle-spin") {
      a.idle_spin = std::stoi(next());
      if (a.idle_spin < 0 || a.idle_spin > 3) {
        std::fprintf(stderr, "[ov_bridge] --idle-spin must be 0, 1, 2 or 3\n");
        return 2;
      }
    }
    else if (s == "--idle-spin-ms") {
      a.idle_spin_ms = std::stod(next());
      if (!(a.idle_spin_ms >= 0.0 && a.idle_spin_ms <= 1000.0)) {
        std::fprintf(stderr, "[ov_bridge] --idle-spin-ms must be in [0, 1000]\n");
        return 2;
      }
    }
    else if (s == "--predict") {
      a.predict = std::stoi(next());
      if (a.predict < 0 || a.predict > 2) {
        std::fprintf(stderr, "[ov_bridge] --predict must be 0, 1 or 2\n");
        return 2;
      }
    } else if (s == "--predict-hz") {
      a.predict_hz = std::stod(next());
      if (!(a.predict_hz > 0.0 && a.predict_hz <= 1000.0)) {
        std::fprintf(stderr, "[ov_bridge] --predict-hz must be in (0, 1000]\n");
        return 2;
      }
    }
    else if (s == "--ovopt") a.ovopt = next();
    else if (s == "--prof-log") a.prof_log = next();
    else if (s == "--mav-udp") a.mav_udp = next();
    else if (s == "--viop-udp") a.viop_udp = next();
    else if (s == "--viop-replay-time") a.mav_replay_time = std::stoi(next());
    else if (s == "--viop-log") a.mav_log = next();
    else if (s == "--viop-hb-hz") a.viop_hb_hz = std::stod(next());
    else if (s == "--viop-hb-src") a.viop_hb_src = std::stoi(next());  // esp32fix-20261026
    else if (s == "--mav-msg") a.mav_msg = next();
    else if (s == "--mav-mount") a.mav_mount = next();
    else if (s == "--mav-yaw-align") a.mav_yaw_align = std::stoi(next());
    else if (s == "--mav-replay-time") a.mav_replay_time = std::stoi(next());
    else if (s == "--mav-sysid") a.mav_sysid = std::stoi(next());
    else if (s == "--mav-log") a.mav_log = next();
    else if (s == "--pose-precision") a.pose_precision = std::max(1, std::min(17, std::stoi(next())));
    else if (s == "--drop-late") a.drop_late_ms = std::max(0.0, std::stod(next()));
    else if (s == "--drop-late-mode") a.drop_late_mode = std::stoi(next()) != 0 ? 1 : 0;           // 2026-10-26
    else if (s == "--drop-late-cap") a.drop_late_cap = std::max(1, std::stoi(next()));            // 2026-10-26
    else if (s == "--drop-late-win") a.drop_late_win = std::max(0.1, std::stod(next()));          // 2026-10-26
    else if (s == "--test-feed-sleep") {  // TEST ONLY (2026-10-26 droplatefix): overload simulation
      const std::string v = next();
      const auto k = v.find(':');
      a.test_feed_sleep_ms = std::stod(v.substr(0, k));
      a.test_feed_sleep_t0 = k == std::string::npos ? 0.0 : std::stod(v.substr(k + 1));
    }
    else if (s == "--pred-stale-ms") a.pred_stale_ms = std::max(0.0, std::stod(next()));          // 2026-10-26
    else if (s == "--pred-thread") a.pred_thread = std::stoi(next()) != 0 ? 1 : 0;   // 2026-10-25
    else if (s == "--imu-log") a.imu_log = next();                                     // 2026-10-25
    else if (s == "--init-still") a.init_still = std::stoi(next()) != 0 ? 1 : 0;   // 2026-10-25 ovrobust
    else if (s == "--health") a.health = std::stoi(next()) != 0 ? 1 : 0;
    else if (s == "--klt-stats") a.klt_stats = std::stoi(next()) != 0 ? 1 : 0;  // 2026-10-26 ovgyro
    else if (s == "--pipe") a.pipe_req = std::stoi(next()) != 0 ? 1 : 0;  // integ2-20261026: refused below (ovpipe not merged)
    else if (s == "--health-log") a.health_log = next();
    else if (s == "--health-cfg") a.health_cfg = next();
    else if (s == "--recover") a.recover = std::max(0, std::stoi(next()));
    else if (s == "--recover-max-gap") a.recover_max_gap = std::stod(next());
    else if (s == "--recover-min-feats") a.recover_min_feats = std::stoi(next());
    else if (s == "--recover-full") a.recover_full = std::stoi(next()) != 0 ? 1 : 0;
    else if (s == "--test-blank") a.test_blank = next();
    else if (s == "--loop") { a.loop = std::stoi(next()) != 0 ? 1 : 0; a.loop_explicit = 1; }  // 2026-10-26 ovloop
    else if (s == "--loop-vocab") a.loop_vocab = next();
    else if (s == "--loop-cfg") a.loop_cfg = next();
    else if (s == "--loop-log") a.loop_log = next();
    else if (s == "--loop-dump") a.loop_dump = next();
    else if (s == "--loop-vpr") a.loop_vpr = next();
    else if (s == "--loop-xf") a.loop_xf = next();
    else if (s == "--loop-sync") a.loop_sync = std::stoi(next()) != 0 ? 1 : 0;
    else if (s == "--loop-out-hz") a.loop_out_hz = std::max(0.0, std::stod(next()));
    else if (s == "--reinit-max-gap") a.reinit_max_gap = std::stod(next());
    else if (s == "--reinit-sigma") {  // ori_deg,pos_m,vel_m_s
      std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &a.reinit_sig_ori_deg, &a.reinit_sig_pos, &a.reinit_sig_vel) != 3) {
        usage();
        return 2;
      }
    } else if (s == "--init-gate") {  // mean,max rad/s, disparity deg
      std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &a.gate_mean, &a.gate_max, &a.gate_disp_deg) != 3) {
        usage();
        return 2;
      }
    } else {
      usage();
      return 2;
    }
  }

  g_idle.mode = a.idle_spin;
  g_idle.win_ms = a.idle_spin_ms;
  if (!a.profile.empty() && a.profile != "none" && (a.tcp || a.predict <= 0)) a.pred_thread = 0;  // profile thread: stdio + predict only
  if (a.pred_thread && (a.predict <= 0 || a.tcp)) {  // 2026-10-25
    std::fprintf(stderr, "[ov_bridge] --pred-thread needs --predict 1|2 and --stdio\n");
    return 2;
  }
  if (!a.imu_log.empty()) {  // 2026-10-25
    g_imu_log = std::fopen(a.imu_log.c_str(), "w");
    if (g_imu_log) std::fprintf(g_imu_log, "t_ns,rx_hdr_boot_ns,rx_boot_ns\n");
    else std::fprintf(stderr, "[ov_bridge] imu log: cannot open %s\n", a.imu_log.c_str());
  }

  // --ovopt / --prof-log (2026-10-25): set the ovpatch flags before any VioManager exists
  if (!a.ovopt.empty()) {
    std::string err;
    if (!ovopt::parse(a.ovopt, err)) {
      std::fprintf(stderr, "[ov_bridge] %s\n", err.c_str());
      return 2;
    }
  }
  if (a.pipe_req) {  // integ2-20261026
    if (ovopt::g.gyro_klt)
      std::fprintf(stderr, "[ov_bridge] --ovopt gyro_klt cannot be used with --pipe 1: gyro_klt reads the filter state (velocity, SLAM depth) "
                           "after the update of the previous frame, the pipelined tracker runs before that update. Use --pipe 0.\n");
    else
      std::fprintf(stderr, "[ov_bridge] --pipe 1 is not in this build (ovpipe-20261026 not merged: stalls / deadlock). Use --pipe 0.\n");
    return 2;
  }
  if (a.loop && !a.loop_explicit && a.recover > 0) {  // integ2 0.6.28: profile loop + explicit --recover -> recover wins
    std::fprintf(stderr, "[ov_bridge] --profile %s: loop closure dropped (--recover given; v1 cannot combine them)\n", a.profile.c_str());
    a.loop = 0;
  }
  if (a.loop && a.loop_vocab.empty()) {  // integ2 0.6.28: default vocabulary for the profile loop (or a bare --loop 1)
    std::string cdir = a.config;
    const size_t sl = cdir.find_last_of('/');
    cdir = sl == std::string::npos ? std::string(".") : cdir.substr(0, sl);
    std::vector<std::string> cand = {cdir + "/voc_k10L4.voc", cdir + "/../voc_k10L4.voc"};
#ifdef OVLOOP_SRC_VOC
    cand.push_back(OVLOOP_SRC_VOC);
#endif
    for (const auto &c : cand) {
      std::ifstream f(c, std::ios::binary);
      if (f.good()) { a.loop_vocab = c; break; }
    }
    if (a.loop_vocab.empty()) {
      std::fprintf(stderr, "[ov_bridge] --loop 1%s needs a vocabulary: --loop-vocab FILE, or voc_k10L4.voc next to the config "
                           "(%s) or one dir above; or --loop 0\n", a.loop_explicit ? "" : " (from --profile)", cdir.c_str());
      return 2;
    }
    std::fprintf(stderr, "[ov_bridge] loop vocabulary (default): %s\n", a.loop_vocab.c_str());
  }
  if (a.loop && a.recover > 0) {  // 2026-10-26 ovloop: the --recover output offset and the map frame are not combined (v1)
    std::fprintf(stderr, "[ov_bridge] --loop 1 cannot be used with --recover (v1: the map frame follows the raw VIO world)\n");
    return 2;
  }
  if (!a.loop && (!a.loop_vocab.empty() || !a.loop_cfg.empty() || !a.loop_log.empty() || a.loop_sync))
    std::fprintf(stderr, "[ov_bridge] --loop-* options ignored without --loop 1\n");
  if (!a.prof_log.empty()) ovopt::g.prof = 1;
  ovopt::g.init_still = a.init_still;  // --init-still (2026-10-25): read by ovpatch/VioManagerHelper.cpp
#ifndef OVOPT_FULL
  if (a.init_still) {
    std::fprintf(stderr, "[ov_bridge] --init-still needs a build with the OpenVINS sources (OV_FULL_SOURCE / phone build)\n");
    return 2;
  }
#endif
#ifndef OVOPT_FULL
  {  // PC build linking libov_msckf_lib.so: only the bridge-side options work, the OpenVINS ones would silently do nothing
    ovopt::Flags f = ovopt::g;
    f.frame_nocopy = 0;
    f.prof = 0;
    if (f.skip_retri || f.par_tri || f.par_jac || f.par_ma || f.par_s || f.cov_move || f.givens_rm || f.ekf_gemm ||
        f.qr_compress || f.ekf_chol || f.par_cov || f.rs_derot || f.gyro_klt || f.ekf_sparse || f.sym_inplace || f.init_gemm) {
      std::fprintf(stderr, "[ov_bridge] --ovopt %s needs a build with the OpenVINS sources (OV_FULL_SOURCE / phone build)\n",
                   a.ovopt.c_str());
      return 2;
    }
  }
#endif
  if (!a.ovopt.empty() || ovopt::g.prof)
    std::fprintf(stderr, "[ov_bridge] ovopt: %s (--ovopt '%s', prof log %s)\n", ovopt::describe().c_str(), a.ovopt.c_str(),
                 a.prof_log.empty() ? "-" : a.prof_log.c_str());

  // --mav-udp (2026-10-25, ovdrone): drone output prototype, needs the prediction stream
  if (!a.mav_udp.empty() || !a.viop_udp.empty()) {
    auto mo = std::make_unique<mav::Out>();
    const bool viop = !a.viop_udp.empty();
    const std::string hp = viop ? a.viop_udp : a.mav_udp;
    mo->proto = viop ? 1 : 0;
    auto c = hp.rfind(':');
    std::string err;
    if (viop && !a.mav_udp.empty()) err = "--viop-udp and --mav-udp: pick one";
    else if (a.predict < 1) err = "--mav-udp / --viop-udp need --predict 1 or 2";
    else if (c == std::string::npos) err = "--mav-udp / --viop-udp need HOST:PORT";
    else {
      mo->host = hp.substr(0, c);
      mo->port = std::atoi(hp.c_str() + c + 1);
      if (mo->port <= 0 || mo->port > 65535) err = "bad --mav-udp port";
    }
    if (err.empty() && !mav::Out::parse_mount(a.mav_mount, mo->R_BI, mo->mount_name)) err = "bad --mav-mount " + a.mav_mount;
    if (err.empty()) {
      mo->msgs = a.mav_msg == "odom" ? 1 : a.mav_msg == "vpe" ? 2 : a.mav_msg == "both" ? 3 : 0;
      if (!mo->msgs) err = "--mav-msg must be odom, vpe or both";
    }
    mo->yaw_align = a.mav_yaw_align != 0;
    mo->hb_hz = a.viop_hb_hz;
    if (viop && (a.viop_hb_hz < 0.0 || a.viop_hb_hz > 200.0)) err = "--viop-hb-hz must be in [0, 200]";
    if (a.viop_hb_src != 0 && a.viop_hb_src != 1) err = "--viop-hb-src must be 0 or 1";
    mo->hb_src = viop ? a.viop_hb_src : 0;  // esp32fix-20261026
    mo->replay_time = a.mav_replay_time != 0;
    mo->sysid = a.mav_sysid;
    if (err.empty() && !a.mav_log.empty() && !(mo->log = std::fopen(a.mav_log.c_str(), "w"))) err = "cannot open --mav-log";
    if (err.empty() && !mo->open(err)) err = "mav: " + err;
    if (!err.empty()) {
      std::fprintf(stderr, "[ov_bridge] %s\n", err.c_str());
      return 2;
    }
    std::fprintf(stderr, "[ov_bridge] %s: udp %s:%d msgs %s mount %s yaw_align %d replay_time %d sysid %d\n", 
                 viop ? "viop" : "mav", mo->host.c_str(), mo->port, a.mav_msg.c_str(), mo->mount_name.c_str(), a.mav_yaw_align, a.mav_replay_time, a.mav_sysid);
    g_mav = std::move(mo);
    g_mav->hb_thread_start();  // --viop-hb-src 1 only (beats start once IMU is received and handled)
    if (g_mav->hb_src) g_hb_feed_out.store(g_mav.get(), std::memory_order_release);
    if (g_mav->hb_src) std::fprintf(stderr, "[ov_bridge] viop heartbeat: thread %.1f Hz (feed %d ms / main loop %d ms liveness)\n",
                                    g_mav->hb_hz, g_mav->hb_feed_ms, g_mav->hb_main_ms);
  }

  // --klt-overlap: read once by ovpatch/TrackKLT.cpp at the first camera frame
  ::setenv("OV_KLT_OVERLAP", a.klt_overlap ? "1" : "0", 1);
  // --klt-pyr (2026-10-25, ovheat): read once by ovpatch/TrackKLT.cpp; only set when given (unset = upstream levels)
  if (a.klt_pyr > 0) ::setenv("OV_KLT_PYR", std::to_string(a.klt_pyr).c_str(), 1);
  else if (const char *e = std::getenv("OV_KLT_PYR")) {  // integ-20261025: never change behaviour from the environment
    std::fprintf(stderr, "[ov_bridge] OV_KLT_PYR=%s in the environment ignored (use --klt-pyr N)\n", e);
    ::unsetenv("OV_KLT_PYR");
  }
  // --pred-stale-ms auto (2026-10-26 droplatefix): 500 ms with --drop-late mode 1, else off; explicit value wins
  if (a.pred_stale_ms < 0.0) a.pred_stale_ms = (a.drop_late_ms > 0.0 && a.drop_late_mode == 1) ? 500.0 : 0.0;
  if (a.drop_late_ms > 0.0 || a.pred_stale_ms > 0.0)
    std::fprintf(stderr, "[ov_bridge] drop-late %.0f ms mode %d cap %d win %.2f s | pred-stale %.0f ms\n", a.drop_late_ms,
                 a.drop_late_mode, a.drop_late_cap, a.drop_late_win, a.pred_stale_ms);
  if (!a.profile.empty())
    std::fprintf(stderr, "[ov_bridge] profile %s: ovopt '%s' idle_spin %d idle_spin_ms %.0f drop_late %.0f health %d predict %d hz %.0f pred_thread %d (after explicit flags)\n",
                 a.profile.c_str(), a.ovopt.c_str(), a.idle_spin, a.idle_spin_ms, a.drop_late_ms, a.health, a.predict,
                 a.predict_hz, a.pred_thread);

  // --cpus: pin before any thread exists (Bridge / OpenVINS / OpenCV threads inherit it)
  if (!a.cpus.empty()) {
    if (!parse_cpus(a.cpus, g_pin)) {
      std::fprintf(stderr, "[ov_bridge] bad --cpus %s\n", a.cpus.c_str());
      return 2;
    }
    g_pinned = true;
    if (::sched_setaffinity(0, sizeof(g_pin), &g_pin) != 0)
      std::fprintf(stderr, "[ov_bridge] affinity: sched_setaffinity(0, %s) failed: %s\n", a.cpus.c_str(), std::strerror(errno));
  }
  apply_affinity(g_pinned ? &g_pin : nullptr, "start");

  // OpenVINS 는 printf 로 stdout 에 로그를 쓴다 -> 결과 채널과 섞이지 않게 fd1 을 stderr 로.
  int out_fd = ::dup(STDOUT_FILENO);
  std::fflush(stdout);
  ::dup2(STDERR_FILENO, STDOUT_FILENO);

  Bridge b(a);
  if (!b.ok()) return 1;

  apply_affinity(g_pinned ? &g_pin : nullptr, "ready");
  if (!a.tcp) {
    if (a.pred_thread) {  // --pred-thread (2026-10-25)
      // heap, never freed: after an output write failure the reader thread is detached and may still use it
      auto *pr = new Predictor(a.predict, a.predict_hz, a.pose_precision, out_fd, false);
      pr->stale_on = a.pred_stale_ms > 0.0;
      serve_q(b, STDIN_FILENO, out_fd, pr);
      pr->log();
      if (g_imu_log) std::fflush(g_imu_log);  // not closed: same reason (closed by process exit)
    } else {
      serve(b, STDIN_FILENO, out_fd, false);
      if (g_imu_log) std::fclose(g_imu_log);
      g_imu_log = nullptr;
    }
    apply_affinity(g_pinned ? &g_pin : nullptr, "end");
    if (g_idle.mode) g_idle.log("end");
    b.log_predict();
    b.log_drop();
    b.log_rs();
    b.log_gyro_klt();  // 2026-10-26 ovgyro
    b.log_loop();      // 2026-10-26 ovloop (--loop 1 only)
    if (g_mav) {
      g_mav->hb_thread_stop();  // --viop-hb-src 1 (no-op otherwise)
      for (int i = 0; i < 3; i++) g_mav->hb_tick(false, false, true);  // VIOP: "stopping" heartbeats on a clean exit
      g_mav->report();
      g_mav->close();
      if (g_hb_feed_out.load()) (void)g_mav.release();  // --viop-hb-src 1: keep the object alive for a detached reader
    }
    b.log_health();
    return 0;
  }

  int srv = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(a.port));
  if (::inet_pton(AF_INET, a.host.c_str(), &addr.sin_addr) != 1) {
    std::fprintf(stderr, "[ov_bridge] bad host %s\n", a.host.c_str());
    return 2;
  }
  if (::bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || ::listen(srv, 1) < 0) {
    std::perror("[ov_bridge] bind/listen");
    return 1;
  }
  std::fprintf(stderr, "[ov_bridge] listening on %s:%d\n", a.host.c_str(), a.port);
  // 클라이언트 하나씩. 연결이 끊겨도 VIO 상태는 유지 (새 세션이면 클라이언트가 RESET 을 보낸다).
  while (true) {
    int c = ::accept(srv, nullptr, nullptr);
    if (c < 0) {
      if (errno == EINTR) continue;
      std::perror("[ov_bridge] accept");
      break;
    }
    ::setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    bool cont = serve(b, c, c, true);
    ::close(c);
    if (!cont) break;
  }
  ::close(srv);
  return 0;
}
