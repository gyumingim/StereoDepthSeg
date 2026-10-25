// ov_bridge — OpenVINS (ROS 없음) VioManager 를 바이너리 레코드로 구동하는 작은 실행 파일.
// 프로토콜: PROTOCOL.md. 입력 = stdin 또는 TCP, 출력 = 같은 채널로 JSON 한 줄씩.
//
//   ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]
//             [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]
//             [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]
//             [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG] [--timing-log FILE] [--cpus LIST] [--threads N] [--klt-overlap 0|1]
//             [--idle-spin 0|1|2|3] [--idle-spin-ms W] [--predict 0|1|2] [--predict-hz HZ] [--ovopt LIST]
//             [--prof-log FILE] [--init-still 0|1] [--health 0|1] [--health-log FILE] [--health-cfg K=V,..]
//             [--recover N] [--recover-max-gap S] [--recover-min-feats N] [--test-blank T0:D,..]
//
// ---- 2026-10-25, branch ovrobust-20261025 (all default off = output and code path unchanged) ----
// --init-still 0|1: static initialization while still. Upstream OpenVINS waits for "still, then a jerk" whenever ZUPT
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
//   rej_deg (0.5) of >= 10 candidates; else OK. stderr "[ov_bridge] health ..." summary at the end.
// --recover N (default 0 = off): automatic recovery after N consecutive LOST frames of a VioManager that has done at
//   least one feature update (startup / still-after-init LOST never triggers it):
//   a) vision back (tracked >= --recover-min-feats, default 20), state finite, |v| <= --max-speed and since_upd <=
//      --recover-max-gap (default 3 s): new VioManager started from the old state (same injection as --reinit-state:
//      pose, velocity, biases, extrinsics, time offset, inflated covariance) -> same world frame, no jump;
//      line {"type":"reset","auto":true,"reason":"recover","reinit":"state",...}.
//   b) since_upd > --recover-max-gap or the state is bad: new VioManager with a normal (static) initialization; once
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
// --pose-precision N (2026-10-25, default 9 = unchanged): significant digits of the numbers in the pose / pred lines
//   (17 = round-trip exact doubles, for identity checks).
// --drop-late MS (2026-10-25, default 0 = off; CHANGES results when it drops): after initialisation a frame is not fed
//   (its IMU is) when its feed lag (CLOCK_BOOTTIME now - frame t_ns) exceeds the smallest lag seen in this run by more
//   than MS -> the backlog cannot grow once processing is slower than the frames (phone "cliff", 2026-10-25 replay).
//   Pose lines get "late_dropped" (only with the flag), stderr "[ov_bridge] drop-late ...".
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
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ovopt.h"  // ovpatch/: --ovopt flags + profiler (2026-10-25)

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace {

constexpr uint32_t kMagic = 0x4F564252u;  // "OVBR"
enum MsgType : uint32_t { kImu = 1, kStereo = 2, kReset = 3, kQuit = 4 };
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
  int idle_spin = 0;        // --idle-spin 0|1|2 (header)
  int predict = 0;          // --predict 0|1|2 (header)
  double predict_hz = 100.0;  // --predict-hz (mode 2)
  std::string ovopt;        // --ovopt LIST (header, ovpatch/ovopt.h)
  std::string prof_log;     // --prof-log FILE (header)
  double idle_spin_ms = 0.0;  // --idle-spin-ms W (header; 0 = spin the whole gap)
  int pose_precision = 9;     // --pose-precision N (header; 9 = unchanged)
  double drop_late_ms = 0.0;  // --drop-late MS (header; 0 = off)
  // 2026-10-25 ovrobust (header)
  int init_still = 0;         // --init-still 0|1
  int health = 0;             // --health 0|1 (pose line "h")
  std::string health_log;     // --health-log FILE
  std::string health_cfg;     // --health-cfg K=V,...
  int recover = 0;            // --recover N (0 = off)
  double recover_max_gap = 3.0;
  int recover_min_feats = 20;
  std::string test_blank;     // --test-blank T0:D,...
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
};

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
    if (fd_ready(fd)) {
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
      while (!fd_ready(fd)) {
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
              if (fd_ready(fd)) {
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
              if (fd_ready(fd)) {
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
                            "delay_in,delay_ok,chi2_rej,pos_std,vel_std,ori_std_deg,since_upd,lost_run,seg,blank,recover_n\n");
      else std::fprintf(stderr, "[ov_bridge] health log: cannot open %s\n", a.health_log.c_str());
    }
    if (!a.health_cfg.empty() && !hcfg_.parse(a.health_cfg)) {
      std::fprintf(stderr, "[ov_bridge] bad --health-cfg %s\n", a.health_cfg.c_str());
      ok_ = false;
      return;
    }
    health_on_ = a.health || hlog_ || a.recover > 0;
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
    make_system();
  }

  bool ok() const { return ok_; }

  // --init-gate 그림자 VioManager (넓힌 시차 기준) 새로 만들기. refeed = 지금까지 넣은 최근 IMU 를 다시 넣음.
  void make_shadow(bool refeed) {
    shadow_.reset();
    shadow_checked_ = shadow_ok_ = false;
    if (!use_shadow_) return;
    shadow_ = std::make_shared<ov_msckf::VioManager>(shadow_params_);
    if (refeed)
      for (const auto &m : imu_recent_) shadow_->feed_measurement_imu(m);
  }

  // VioManager 새로 만들기. 붙잡아 둔 IMU(imu_hold_) 는 새 시스템에 그대로 넣을 수 있으므로 남긴다.
  void make_system() {
    sys_ = std::make_shared<ov_msckf::VioManager>(params_);
    pred_pending_ = false;
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

  // 입력 하나 처리. 출력 JSON 줄들을 out 에 덧붙인다. false = QUIT.
  bool handle(uint32_t type, const std::vector<uint8_t> &buf, std::string &out) {
    switch (type) {
    case kImu:
      if (buf.size() != 56) {
        out += err_line("IMU payload must be 56 bytes");
        return true;
      }
      on_imu(buf.data(), out);
      return true;
    case kStereo:
      on_stereo(buf, out);
      return true;
    case kReset:
      make_system();
      imu_hold_.clear();
      imu_recent_.clear();
      last_upd_t_ = -1e300;
      newest_imu_t_ = -1e300;
      first_cam_t_ = -1e300;
      off_active_ = off_pending_ = false;  // --recover frame offset (2026-10-25)
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
      << (lost_ ? "lost" : "tracking") << "\",\"p\":";
    jarr(o, p_IinG);
    o << ",\"q\":[";
    jnum(o, qh.x()); o << ","; jnum(o, qh.y()); o << ","; jnum(o, qh.z()); o << ","; jnum(o, qh.w());
    o << "],\"v\":";
    jarr(o, v);
    o << ",\"T_world_cam0\":";
    jarr(o, T);
    o << ",\"q_GtoI_jpl\":";
    jarr(o, q_GtoI);
    o << "}\n";
    out += o.str();
  }

  void log_drop() const {  // --drop-late summary (2026-10-25)
    if (args_.drop_late_ms <= 0.0) return;
    std::fprintf(stderr, "[ov_bridge] drop-late: limit %.1f ms | dropped %llu frames | min lag %.1f ms\n", args_.drop_late_ms,
                 static_cast<unsigned long long>(n_late_dropped_), lag_min_ms_ < 1e299 ? lag_min_ms_ : -1.0);
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
    const double lag_ms = static_cast<double>(boot_ns() - c.t_ns) / 1e6;
    if (lag_ms < lag_min_ms_) lag_min_ms_ = lag_ms;
    if (!sys_->initialized() || lag_ms - lag_min_ms_ <= args_.drop_late_ms) return false;
    n_late_dropped_++;
    if (n_late_dropped_ <= 20 || n_late_dropped_ % 100 == 0)
      std::fprintf(stderr, "[ov_bridge] drop-late: frame t=%.3f %.1f ms late (> %.1f) -> skipped (#%llu)\n", c.t,
                   lag_ms - lag_min_ms_, args_.drop_late_ms, static_cast<unsigned long long>(n_late_dropped_));
    return true;
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
    if (plog_) ovopt::prof_reset();  // --prof-log: stage timers of this frame only
    ovopt::health_reset();           // --health counters of this frame only (always counted, cheap)
    auto t0 = std::chrono::steady_clock::now();
    try {
      sys_->feed_measurement_camera(c.data);
    } catch (const std::exception &e) {  // cv::Exception 도 std::exception
      on_exception("feed_measurement_camera", e.what(), out);
      return;
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
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
      std::fprintf(hlog_, "%lld,%.6f,%s,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.5g,%.5g,%.5g,%.4f,%d,%d,%d,%llu\n",
                   static_cast<long long>(c.t_ns), c.t, hstate_name(r.state), r.init ? 1 : 0, r.tracked, r.slam, r.msckf,
                   r.c.msckf_in, r.c.msckf_tri, r.c.msckf_used, r.c.slam_in, r.c.slam_clean, r.c.slam_used, r.c.delay_in,
                   r.c.delay_ok, r.chi2_rej, r.pos_std, r.vel_std, r.ori_std_deg, r.since_upd, r.lost_run, seg_, c.blank ? 1 : 0,
                   static_cast<unsigned long long>(n_recover_state_ + n_recover_full_));
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
    o << ",\"lost_run\":" << r.lost_run << "}";
  }

 public:
  void log_health() const {
    if (!health_on_) return;
    std::fprintf(stderr, "[ov_bridge] health: frames INIT %llu OK %llu DEGRADED %llu LOST %llu | transitions %llu | max LOST run %d "
                 "frames | recover state %llu full %llu | blanked %llu\n",
                 static_cast<unsigned long long>(h_n_[0]), static_cast<unsigned long long>(h_n_[1]),
                 static_cast<unsigned long long>(h_n_[2]), static_cast<unsigned long long>(h_n_[3]),
                 static_cast<unsigned long long>(h_transitions_), h_max_lost_run_,
                 static_cast<unsigned long long>(n_recover_state_), static_cast<unsigned long long>(n_recover_full_),
                 static_cast<unsigned long long>(n_blanked_));
  }

 private:
  // ---- --recover N (2026-10-25, ovrobust, header) ----
  void try_recover(const CamMsg &c, std::string &out) {
    if (!sys_->initialized() || !h_upd_seen_ || h_lost_run_ < args_.recover) return;
    const HealthRec &r = hrec_;
    const bool gap_ok = r.since_upd >= 0.0 && r.since_upd <= args_.recover_max_gap;
    if (!h_state_bad_ && gap_ok && r.tracked >= args_.recover_min_feats) {
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
    if (h_state_bad_ || !gap_ok) {  // full reset; output frame continued with a 4-DoF offset once re-initialized
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
    make_system();
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

      o << ",\"status\":\"" << (lost_ ? "lost" : "tracking") << "\",\"init\":true";
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
    std::fprintf(stderr, "[ov_bridge] remap %dx%d (source %dx%d) from %s\n", wh[0], wh[1],
                 remap_src_w_ > 0 ? remap_src_w_ : wh[0], remap_src_h_ > 0 ? remap_src_h_ : wh[1], path.c_str());
  }
  int remap_src_w_ = 0, remap_src_h_ = 0;  // v2 remap file: frames of this size are remapped (0 = map size)
  FILE *tlog_ = nullptr;                   // --timing-log
  FILE *plog_ = nullptr;                   // --prof-log (2026-10-25)
  uint64_t plog_rows_ = 0;
  int64_t rx_hdr_boot_ns_ = 0, rx_boot_ns_ = 0;     // set_rx
  double rx_spin_wait_us_ = -1.0, rx_spin_exit_us_ = -1.0;
  uint64_t tlog_rows_ = 0;
  bool pred_pending_ = false;            // --predict: a frame update since the last prediction
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
  uint64_t n_recover_state_ = 0, n_recover_full_ = 0, n_blanked_ = 0;
  std::vector<std::pair<double, double>> blank_;  // --test-blank windows (s after the first camera record)
  bool have_last_out_ = false, off_pending_ = false, off_active_ = false;
  Eigen::Matrix3d last_out_R_ = Eigen::Matrix3d::Identity(), off_anchor_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_out_p_ = Eigen::Vector3d::Zero(), off_anchor_p_ = Eigen::Vector3d::Zero();
  double off_yaw_ = 0.0;
  Eigen::Vector3d off_t_ = Eigen::Vector3d::Zero();
  int seg_ = 0;
  double lag_min_ms_ = 1e300;       // --drop-late: smallest feed lag seen
};

// 한 연결(또는 stdin) 을 끝까지 처리. 반환 false = QUIT 받음.
bool serve(Bridge &b, int in_fd, int out_fd, bool is_socket) {
  std::vector<uint8_t> buf;
  std::string out;
  uint32_t hdr[3];
  uint64_t n_cam_rec = 0;
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
    buf.resize(hdr[2]);
    if (hdr[2] && !read_full(in_fd, buf.data(), hdr[2])) break;
    b.set_rx(t_hdr, boot_ns(), g_idle.mode ? g_idle.last_wait_us : -1.0, g_idle.last_exit_us);
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

void usage() {
  std::fprintf(stderr,
               "usage: ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]\n"
               "                 [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]\n"
               "                 [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]\n"
               "                 [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG] [--timing-log FILE] [--cpus LIST] [--threads N]\n"
               "                 [--klt-overlap 0|1] [--idle-spin 0|1|2|3] [--idle-spin-ms W] [--predict 0|1|2] [--predict-hz HZ]\n"
               "                 [--ovopt LIST] [--prof-log FILE] [--pose-precision N] [--drop-late MS]\n"
               "                 [--init-still 0|1] [--health 0|1] [--health-log FILE] [--health-cfg K=V,..] [--recover N]\n"
               "                 [--recover-max-gap S] [--recover-min-feats N] [--test-blank T0:D,..]\n");
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
    else if (s == "--pose-precision") a.pose_precision = std::max(1, std::min(17, std::stoi(next())));
    else if (s == "--drop-late") a.drop_late_ms = std::max(0.0, std::stod(next()));
    else if (s == "--init-still") a.init_still = std::stoi(next()) != 0 ? 1 : 0;   // 2026-10-25 ovrobust
    else if (s == "--health") a.health = std::stoi(next()) != 0 ? 1 : 0;
    else if (s == "--health-log") a.health_log = next();
    else if (s == "--health-cfg") a.health_cfg = next();
    else if (s == "--recover") a.recover = std::max(0, std::stoi(next()));
    else if (s == "--recover-max-gap") a.recover_max_gap = std::stod(next());
    else if (s == "--recover-min-feats") a.recover_min_feats = std::stoi(next());
    else if (s == "--test-blank") a.test_blank = next();
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

  // --ovopt / --prof-log (2026-10-25): set the ovpatch flags before any VioManager exists
  if (!a.ovopt.empty()) {
    std::string err;
    if (!ovopt::parse(a.ovopt, err)) {
      std::fprintf(stderr, "[ov_bridge] %s\n", err.c_str());
      return 2;
    }
  }
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
        f.qr_compress || f.ekf_chol || f.par_cov) {
      std::fprintf(stderr, "[ov_bridge] --ovopt %s needs a build with the OpenVINS sources (OV_FULL_SOURCE / phone build)\n",
                   a.ovopt.c_str());
      return 2;
    }
  }
#endif
  if (!a.ovopt.empty() || ovopt::g.prof)
    std::fprintf(stderr, "[ov_bridge] ovopt: %s (--ovopt '%s', prof log %s)\n", ovopt::describe().c_str(), a.ovopt.c_str(),
                 a.prof_log.empty() ? "-" : a.prof_log.c_str());

  // --klt-overlap: read once by ovpatch/TrackKLT.cpp at the first camera frame
  ::setenv("OV_KLT_OVERLAP", a.klt_overlap ? "1" : "0", 1);

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
    serve(b, STDIN_FILENO, out_fd, false);
    apply_affinity(g_pinned ? &g_pin : nullptr, "end");
    if (g_idle.mode) g_idle.log("end");
    b.log_predict();
    b.log_drop();
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
