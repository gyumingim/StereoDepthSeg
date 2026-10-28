// mav_out.h — drone pose output of ov_bridge: IMU-forward-predicted pose over UDP (2026-10-25, branch
// ovdrone-20261025, prototype). Two wire formats (one per run):
//   proto 1 = VIOP v2 (--viop-udp HOST:PORT): compact framed POSE message (92-byte frame, viop_proto.h) for the ESP32-S3
//             custom flight firmware (user decisions 2026-10-25): world z-up / phone-IMU body frame, the ESP32 applies
//             its mount; same bytes over USB CDC / ESP-NOW later. Clock: TIME_REQ frames answered at once (TIME_RESP)
//             by the socket thread. No heartbeat.
//   proto 0 = MAVLink 2 (--mav-udp HOST:PORT, optional mode kept from the first prototype), described below. Header-only, no MAVLink library: the few messages are hand-encoded (field order, CRC
// extra and lengths checked against pymavlink 2.x common.xml, runs/ovdrone_20261025/README.md). Default off: only
// ov_bridge --mav-udp HOST:PORT creates it. Nothing here talks to a flight controller; it only sends UDP datagrams.
//
// Messages (system id --mav-sysid, default 1; component 197 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY):
//   ODOMETRY (#331) per prediction (--mav-msg odom|vpe|both, default odom): frame_id MAV_FRAME_LOCAL_FRD (20),
//     child_frame_id MAV_FRAME_BODY_FRD (12), estimator_type MAV_ESTIMATOR_TYPE_VIO (3), reset_counter, quality.
//   VISION_POSITION_ESTIMATE (#102) per prediction with vpe/both (x y z in the same local FRD frame, roll pitch yaw ZYX).
//   HEARTBEAT (#0) 1 Hz (type MAV_TYPE_ONBOARD_CONTROLLER, autopilot MAV_AUTOPILOT_INVALID, system_status STANDBY before
//     the first pose / ACTIVE while tracking / CRITICAL when lost).
//   TIMESYNC (#111) reply: a request (tc1 == 0) received on the same socket is answered at once by a small thread with
//     tc1 = CLOCK_BOOTTIME ns of the phone, ts1 echoed -> the receiver gets the phone-boot <-> PC clock offset (NTP style).
//
// Frames (see README "frame conventions"):
//   G = OpenVINS world: z up (gravity -z), x/y = yaw of the VIO initialisation, origin = init position.
//   I = phone IMU (Android sensor axes: x right edge, y top edge, z out of the screen), what OpenVINS estimates.
//   B = vehicle body FRD (x forward, y right, z down): B = R_BI * I, --mav-mount preset or 9 numbers (row-major R_BI).
//   L = MAVLink LOCAL_FRD: origin = G origin, z down, x = the vehicle's forward direction (horizontal) at the first pose
//       after each (re)initialisation: R_LG = diag(1,-1,-1) * Rz(-psi0), psi0 = heading of B's x axis in G then.
//       (--mav-yaw-align 0: psi0 = 0, i.e. L = G with y and z flipped.)
//   position = R_LG p_IinG (IMU position, no lever arm to a vehicle reference point), q = R_LB (body -> local, w x y z),
//   velocity in B (ODOMETRY convention for child_frame BODY_FRD), angular rate in B (newest gyro sample - bias).
//   Covariance: OpenVINS marginal covariance of (q, p, v) at the filter state time (NOT propagated over the prediction
//   horizon -> slightly optimistic), rotated: position in L, attitude error (JPL, IMU frame) in B (small-angle = roll /
//   pitch / yaw about the body axes), velocity in B; rate diagonal = sigma_w^2 * IMU rate. Upper-right triangle,
//   row-major (MAVLink convention).
//   quality: 0 when lost, else clamp(100 * (1 - sigma_pos / 0.20 m), 1, 100), sigma_pos = sqrt(trace(P_pos) / 3)
//   (provisional, not tuned).
// Time base: time_usec = predicted instant on the phone CLOCK_BOOTTIME (Android sensor timestamps = elapsedRealtimeNanos)
//   in us. --mav-replay-time 1 (replay of an old stream only): + min over IMU records of (receive boot time - sensor
//   time), i.e. the stream is re-timed to its delivery on this run.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigen>

#include "viop_proto.h"

namespace mav {

inline int64_t boot_ns() {
  timespec t;
  clock_gettime(CLOCK_BOOTTIME, &t);
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

// MAVLink X.25 CRC (crc_accumulate)
inline void crc_acc(uint8_t b, uint16_t &crc) {
  uint8_t tmp = b ^ static_cast<uint8_t>(crc & 0xff);
  tmp ^= static_cast<uint8_t>(tmp << 4);
  crc = static_cast<uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

// little-endian payload writer
struct Buf {
  std::vector<uint8_t> b;
  void u8(uint8_t v) { b.push_back(v); }
  void i8(int8_t v) { b.push_back(static_cast<uint8_t>(v)); }
  void u32(uint32_t v) { for (int i = 0; i < 4; i++) b.push_back(static_cast<uint8_t>(v >> (8 * i))); }
  void u64(uint64_t v) { for (int i = 0; i < 8; i++) b.push_back(static_cast<uint8_t>(v >> (8 * i))); }
  void i64(int64_t v) { u64(static_cast<uint64_t>(v)); }
  void f32(float v) { uint32_t u; std::memcpy(&u, &v, 4); u32(u); }
};

struct Out {
  // ---- config ----
  std::string host;
  int port = 0;
  int sysid = 1, compid = 197;
  int msgs = 1;  // bit 0 ODOMETRY, bit 1 VISION_POSITION_ESTIMATE
  int proto = 0; // 0 MAVLink 2, 1 VIOP (viop_proto.h)
  uint32_t viop_seq = 0;
  bool yaw_align = true;
  bool replay_time = false;
  Eigen::Matrix3d R_BI = Eigen::Matrix3d::Identity();
  std::string mount_name = "imu";
  FILE *log = nullptr;  // --mav-log CSV
  // ---- state ----
  int fd = -1;
  sockaddr_in dst{};
  uint8_t seq = 0;
  const void *epoch = nullptr;  // VioManager instance of the current alignment
  uint8_t reset_counter = 0;
  Eigen::Matrix3d R_LG = Eigen::Matrix3d::Identity();
  int64_t replay_off_ns = INT64_MAX;  // min(rx boot - sensor t) over IMU records
  int64_t imu_rx_boot = 0;            // receive boot time of the newest IMU record
  std::atomic<int> status{3};         // HEARTBEAT system_status: 3 standby, 4 active, 5 critical
  std::atomic<bool> vio_reset_req{false};  // armhist 10-29: VIOP_T_VIO_RESET_REQ from the FC -> main loop full re-init
  uint64_t n_vio_reset_req = 0;
  std::atomic<bool> stop{false};
  std::thread th;
  std::mutex mu;  // send() from the main thread and the timesync / heartbeat thread
  std::mutex emu; // integ-20261025: emit() (main or prediction thread) vs note_imu() / hb_tick() (main thread)
  // ---- stats ----
  uint64_t n_sent = 0, n_fail = 0, n_ts = 0, n_hb = 0;
  std::vector<float> age_ms, int_ms;  // send - predicted instant (boot clock); send - newest IMU receive

  static bool parse_mount(const std::string &s, Eigen::Matrix3d &R, std::string &name) {
    // nominal axis permutations from the phone calibration (T_imu_cam: cam x ~ -I y, cam y ~ -I x, cam z ~ -I z)
    if (s == "imu") { R.setIdentity(); name = s; return true; }
    if (s == "cam_fwd") {  // rear camera looks forward, image upright (phone upright, landscape, its +x edge up)
      R << 0, 0, -1, 0, -1, 0, -1, 0, 0; name = s; return true;
    }
    if (s == "cam_down") {  // rear camera looks down, image top = forward (phone flat, screen up, its +x edge forward)
      R << 1, 0, 0, 0, -1, 0, 0, 0, -1; name = s; return true;
    }
    std::vector<double> v;
    size_t p = 0;
    while (p <= s.size()) {
      size_t e = s.find(',', p);
      if (e == std::string::npos) e = s.size();
      try { v.push_back(std::stod(s.substr(p, e - p))); } catch (...) { return false; }
      p = e + 1;
    }
    if (v.size() != 9) return false;
    R << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    if ((R * R.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-3 || R.determinant() < 0.99) return false;
    name = "custom";
    return true;
  }

  ~Out() { hb_thread_stop(); }  // esp32fix: never destroy a joinable heartbeat thread (--viop-hb-src 1, e.g. --tcp exit)

  bool open(std::string &err) {
    fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { err = std::string("socket: ") + std::strerror(errno); return false; }
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) { err = "bad --mav-udp host " + host; return false; }
    sockaddr_in any{};
    any.sin_family = AF_INET;
    any.sin_port = 0;
    any.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&any), sizeof(any)) < 0) { err = std::string("bind: ") + std::strerror(errno); return false; }
    timeval tv{0, 200000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (log)
      std::fprintf(log, "t_pred_ns,time_usec,send_boot_ns,age_ms,int_ms,src,status,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,"
                        "sig_pos_m,quality,reset,f_bx,f_by,f_bz\n");
    th = std::thread([this] { loop(); });
    return true;
  }

  void close() {
    hb_thread_stop();
    if (fd < 0) return;
    stop = true;
    if (th.joinable()) th.join();
    ::close(fd);
    fd = -1;
    if (log) std::fclose(log), log = nullptr;
  }

  // frame + send (MAVLink 2, payload trailing zeros truncated as the spec requires)
  bool send(uint32_t msgid, uint8_t crc_extra, std::vector<uint8_t> pl) {
    while (pl.size() > 1 && pl.back() == 0) pl.pop_back();
    std::lock_guard<std::mutex> lk(mu);
    std::vector<uint8_t> f;
    f.reserve(pl.size() + 12);
    f.push_back(0xFD);
    f.push_back(static_cast<uint8_t>(pl.size()));
    f.push_back(0);  // incompat flags
    f.push_back(0);  // compat flags
    f.push_back(seq++);
    f.push_back(static_cast<uint8_t>(sysid));
    f.push_back(static_cast<uint8_t>(compid));
    f.push_back(static_cast<uint8_t>(msgid & 0xff));
    f.push_back(static_cast<uint8_t>((msgid >> 8) & 0xff));
    f.push_back(static_cast<uint8_t>((msgid >> 16) & 0xff));
    f.insert(f.end(), pl.begin(), pl.end());
    uint16_t crc = 0xffff;
    for (size_t i = 1; i < f.size(); i++) crc_acc(f[i], crc);
    crc_acc(crc_extra, crc);
    f.push_back(static_cast<uint8_t>(crc & 0xff));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    ssize_t r = ::sendto(fd, f.data(), f.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&dst), sizeof(dst));
    return r == static_cast<ssize_t>(f.size());
  }

  bool send_raw(const void *d, size_t n) {
    std::lock_guard<std::mutex> lk(mu);
    ssize_t r = ::sendto(fd, d, n, MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&dst), sizeof(dst));
    return r == static_cast<ssize_t>(n);
  }

  // background thread: TIMESYNC replies (immediately on receipt) + 1 Hz HEARTBEAT
  void loop() {
    int64_t next_hb = 0;
    uint8_t rx[300];
    while (!stop) {
      int64_t now = boot_ns();
      if (proto == 0 && now >= next_hb) {
        Buf p;
        p.u32(0);   // custom_mode
        p.u8(18);   // type MAV_TYPE_ONBOARD_CONTROLLER
        p.u8(8);    // autopilot MAV_AUTOPILOT_INVALID
        p.u8(0);    // base_mode
        p.u8(static_cast<uint8_t>(status.load()));
        p.u8(3);    // mavlink_version
        send(0, 50, p.b);
        n_hb++;
        next_hb = now + 1000000000LL;
      }
      sockaddr_in src{};
      socklen_t sl = sizeof(src);
      ssize_t n = ::recvfrom(fd, rx, sizeof(rx), 0, reinterpret_cast<sockaddr *>(&src), &sl);
      if (n <= 0) continue;
      const int64_t t_rx = boot_ns();
      if (n >= VIOP_HDR + 2 && rx[0] == VIOP_SYNC0) {  // VIOP TIME_REQ -> TIME_RESP with the phone boot time at receipt
        if (viop_check(rx, static_cast<int>(n)) == 0 && rx[3] == VIOP_T_TIME_REQ && rx[4] == sizeof(viop_time_t)) {
          viop_time_t t;
          std::memcpy(&t, rx + VIOP_HDR, sizeof(t));
          t.t_phone_rx_ns = static_cast<uint64_t>(t_rx);
          uint8_t f[VIOP_MAX_FRAME];
          const int fs = viop_build(f, VIOP_T_TIME_RESP, VIOP_SRC_PHONE, static_cast<uint16_t>(rx[6] | (rx[7] << 8)), &t,
                                    sizeof(t));
          {
            std::lock_guard<std::mutex> lk(mu);
            ::sendto(fd, f, fs, MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&src), sl);
          }
          n_ts++;
        }
        // armhist 10-29: FC (via whatever carries drone -> phone bytes to this socket) asks for a FULL VIO re-init
        if (viop_check(rx, static_cast<int>(n)) == 0 && rx[3] == VIOP_T_VIO_RESET_REQ && rx[4] == sizeof(viop_cmd_t)) {
          viop_cmd_t c;
          std::memcpy(&c, rx + VIOP_HDR, sizeof(c));
          if (c.cmd == VIOP_CMD_VIO_RESET && c.confirm == VIOP_CMD_CONFIRM(VIOP_CMD_VIO_RESET)) {
            vio_reset_req.store(true);
            n_vio_reset_req++;
            std::fprintf(stderr, "[ov_bridge] viop: VIO reset request from the FC (#%llu)\n",
                         static_cast<unsigned long long>(n_vio_reset_req));
          }
        }
        continue;
      }
      // MAVLink 2 TIMESYNC request: FD len 0 0 seq sys comp 6F 00 00 payload(tc1 int64, ts1 int64; zeros truncated)
      if (n < 12 || rx[0] != 0xFD || rx[7] != 111 || rx[8] != 0 || rx[9] != 0) continue;
      const int len = rx[1];
      if (n < 12 + len) continue;
      uint16_t crc = 0xffff;
      for (int i = 1; i < 10 + len; i++) crc_acc(rx[i], crc);
      crc_acc(34, crc);
      if (rx[10 + len] != (crc & 0xff) || rx[11 + len] != (crc >> 8)) continue;
      uint8_t pl[16] = {0};
      std::memcpy(pl, rx + 10, std::min(len, 16));
      int64_t tc1, ts1;
      std::memcpy(&tc1, pl, 8);
      std::memcpy(&ts1, pl + 8, 8);
      if (tc1 != 0) continue;
      Buf p;
      p.i64(t_rx);  // tc1 = phone boot ns at receipt (reply leaves right after)
      p.i64(ts1);
      send(111, 34, p.b);
      n_ts++;
    }
  }

  // called for every IMU record (main thread) with its sensor time (ns)
  void note_imu(int64_t t_sensor_ns) {
    std::lock_guard<std::mutex> lk(emu);
    imu_rx_boot = boot_ns();
    if (replay_time) replay_off_ns = std::min(replay_off_ns, imu_rx_boot - t_sensor_ns);
  }

  // one prediction -> ODOMETRY / VISION_POSITION_ESTIMATE. P9 = marginal covariance of (q_GtoI JPL error, p, v).
  void emit(const void *sys_epoch, int64_t t_pred_ns, const Eigen::Matrix3d &R_ItoG, const Eigen::Vector3d &p_IinG,
            const Eigen::Vector3d &v_IinG, const Eigen::Vector3d &w_I, const Eigen::Matrix<double, 9, 9> &P9, bool lost,
            double sigma_w2_rate, const Eigen::Vector3d &f_I, const char *src, int64_t t_state_ns = 0, int n_feat = 0,
            bool seeded = false) {
    if (fd < 0) return;
    std::lock_guard<std::mutex> lk(emu);
    if (proto == 1) {
      emit_viop(sys_epoch, t_pred_ns, t_state_ns, R_ItoG, p_IinG, v_IinG, w_I, f_I, P9, lost, n_feat, src, seeded);
      return;
    }
    if (sys_epoch != epoch) {  // first pose of a (re)initialised filter: align L, count resets
      if (epoch != nullptr) reset_counter++;
      epoch = sys_epoch;
      double psi0 = 0.0;
      if (yaw_align) {
        Eigen::Vector3d fwd = R_ItoG * R_BI.transpose() * Eigen::Vector3d::UnitX();
        if (std::hypot(fwd(0), fwd(1)) > 0.2) psi0 = std::atan2(fwd(1), fwd(0));
      }
      Eigen::Matrix3d F = Eigen::Vector3d(1, -1, -1).asDiagonal();
      R_LG = F * Eigen::AngleAxisd(-psi0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      std::fprintf(stderr, "[ov_bridge] mav: frame aligned (reset_counter %d, psi0 %.1f deg, mount %s)\n", reset_counter,
                   psi0 * 180.0 / M_PI, mount_name.c_str());
    }
    status = lost ? 5 : 4;
    const Eigen::Matrix3d R_LB = R_LG * R_ItoG * R_BI.transpose();
    const Eigen::Matrix3d R_BG = R_BI * R_ItoG.transpose();
    const Eigen::Vector3d pL = R_LG * p_IinG, vB = R_BG * v_IinG, wB = R_BI * w_I;
    const Eigen::Quaterniond q(R_LB);
    // covariance 6x6 (x y z | roll pitch yaw) and velocity 3x3
    Eigen::Matrix<double, 6, 6> C;
    C.block<3, 3>(0, 0) = R_LG * P9.block<3, 3>(3, 3) * R_LG.transpose();
    C.block<3, 3>(3, 3) = R_BI * P9.block<3, 3>(0, 0) * R_BI.transpose();
    C.block<3, 3>(0, 3) = R_LG * P9.block<3, 3>(3, 0) * R_BI.transpose();
    C.block<3, 3>(3, 0) = C.block<3, 3>(0, 3).transpose();
    const Eigen::Matrix3d Cv = R_BG * P9.block<3, 3>(6, 6) * R_BG.transpose();
    const double sig_pos = std::sqrt(std::max(0.0, C.block<3, 3>(0, 0).trace() / 3.0));
    int quality = lost ? 0 : static_cast<int>(std::lround(100.0 * (1.0 - sig_pos / 0.20)));
    if (!lost) quality = std::min(100, std::max(1, quality));
    int64_t t_boot = t_pred_ns + (replay_time && replay_off_ns != INT64_MAX ? replay_off_ns : 0);
    const uint64_t time_usec = static_cast<uint64_t>(std::max<int64_t>(0, t_boot / 1000));
    bool ok = true;
    if (msgs & 1) {
      Buf p;
      p.u64(time_usec);
      p.f32(pL(0)); p.f32(pL(1)); p.f32(pL(2));
      p.f32(q.w()); p.f32(q.x()); p.f32(q.y()); p.f32(q.z());
      p.f32(vB(0)); p.f32(vB(1)); p.f32(vB(2));
      p.f32(wB(0)); p.f32(wB(1)); p.f32(wB(2));
      for (int i = 0; i < 6; i++)
        for (int j = i; j < 6; j++) p.f32(static_cast<float>(C(i, j)));
      Eigen::Matrix<double, 6, 6> V = Eigen::Matrix<double, 6, 6>::Zero();
      V.block<3, 3>(0, 0) = Cv;
      for (int i = 3; i < 6; i++) V(i, i) = sigma_w2_rate;
      for (int i = 0; i < 6; i++)
        for (int j = i; j < 6; j++) p.f32(static_cast<float>(V(i, j)));
      p.u8(20);  // frame_id MAV_FRAME_LOCAL_FRD
      p.u8(12);  // child_frame_id MAV_FRAME_BODY_FRD
      p.u8(reset_counter);
      p.u8(3);   // estimator_type MAV_ESTIMATOR_TYPE_VIO
      p.i8(static_cast<int8_t>(quality));
      ok = send(331, 91, p.b) && ok;
    }
    if (msgs & 2) {
      const double roll = std::atan2(R_LB(2, 1), R_LB(2, 2)), pitch = -std::asin(std::max(-1.0, std::min(1.0, R_LB(2, 0)))),
                   yaw = std::atan2(R_LB(1, 0), R_LB(0, 0));
      Buf p;
      p.u64(time_usec);
      p.f32(pL(0)); p.f32(pL(1)); p.f32(pL(2));
      p.f32(roll); p.f32(pitch); p.f32(yaw);
      for (int i = 0; i < 6; i++)
        for (int j = i; j < 6; j++) p.f32(static_cast<float>(C(i, j)));
      p.u8(reset_counter);
      ok = send(102, 158, p.b) && ok;
    }
    const int64_t now = boot_ns();
    if (ok) n_sent++; else n_fail++;
    const double age = (now - t_boot) * 1e-6, im = imu_rx_boot ? (now - imu_rx_boot) * 1e-6 : -1.0;
    if (age_ms.size() < 2000000) { age_ms.push_back(static_cast<float>(age)); int_ms.push_back(static_cast<float>(im)); }
    if (log) {
      const Eigen::Vector3d fB = R_BI * f_I;
      std::fprintf(log, "%lld,%llu,%lld,%.3f,%.3f,%s,%d,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,"
                        "%.4f,%.4f,%.4f\n",
                   static_cast<long long>(t_pred_ns), static_cast<unsigned long long>(time_usec), static_cast<long long>(now), age,
                   im, src, lost ? 0 : 1, pL(0), pL(1), pL(2), q.w(), q.x(), q.y(), q.z(), vB(0), vB(1), vB(2), wB(0), wB(1),
                   wB(2), sig_pos, quality, reset_counter, fB(0), fB(1), fB(2));
    }
  }

  // VIOP v2 POSE frame (viop_proto.h): world z-up / phone IMU frame, no mount, no yaw alignment (the ESP32 does that)
  // VIOP HEARTBEAT: called from the bridge main loop on every IMU record (never from a timer thread, so a stalled feed /
  // hung bridge stops the heartbeat); sends when the period has elapsed. stopping = final heartbeats on a clean exit.
  double hb_hz = 20.0;
  int64_t hb_last = 0, pose_last = 0;
  uint16_t hb_seq = 0;
  uint8_t last_quality = 0;
  uint64_t n_hb_viop = 0;
  void hb_tick(bool initialized, bool lost, bool stopping = false) {
    if (fd < 0 || proto != 1 || hb_hz <= 0.0) return;
    std::lock_guard<std::mutex> lk(emu);
    const int64_t now = boot_ns();
    if (!stopping && now - hb_last < static_cast<int64_t>(1e9 / hb_hz)) return;
    hb_build_send(now, initialized, lost, stopping, imu_rx_boot);
  }

  // --viop-hb-src 1 (esp32fix-20261026; default 0 = hb_tick from the main loop above, unchanged): a dedicated thread
  // sends the heartbeat on a steady 1/hb_hz grid (CLOCK_MONOTONIC sleep_until) instead of "on the next main-loop IMU
  // record once the period has elapsed" (PC paced: gaps up to 99-122 ms at 20 Hz, 18.7-18.9 Hz mean, because the main
  // loop is busy with a frame). Liveness is kept: a beat is sent only while (a) an IMU record was RECEIVED within
  // hb_feed_ms (reader thread with --pred-thread, else main loop) and (b) the main loop handled a record within
  // hb_main_ms (a hung VIO / bridge stops the heartbeat even though the prediction thread may still emit poses).
  // Otherwise the beat is skipped (counted), so the ESP32 heartbeat timeout (250 ms) still fires. state / lost come
  // from the main loop (atomics). The 3 STOPPING beats on a clean exit are sent after the thread is stopped.
  int hb_src = 0;
  int hb_feed_ms = 100, hb_main_ms = 200;
  std::atomic<int64_t> hb_feed_boot{0}, hb_main_boot{0};
  std::atomic<bool> hb_init{false}, hb_lost{false}, hb_stop{false};
  std::thread hb_th;
  uint64_t n_hb_skip_feed = 0, n_hb_skip_main = 0, n_hb_late = 0;
  std::vector<float> hb_gap_ms;  // thread mode: gaps between sent beats (send boot time)
  void note_feed() { if (hb_src) hb_feed_boot.store(boot_ns(), std::memory_order_relaxed); }
  void note_main(bool initialized, bool lost) {
    hb_init.store(initialized, std::memory_order_relaxed);
    hb_lost.store(lost, std::memory_order_relaxed);
    hb_main_boot.store(boot_ns(), std::memory_order_release);
  }
  void hb_thread_start() {
    if (fd < 0 || proto != 1 || hb_hz <= 0.0 || hb_src != 1 || hb_th.joinable()) return;
    hb_stop = false;
    hb_th = std::thread([this] { hb_run(); });
  }
  void hb_thread_stop() {
    hb_stop = true;
    if (hb_th.joinable()) hb_th.join();
  }
  void hb_run() {
    const auto per = std::chrono::nanoseconds(static_cast<int64_t>(std::llround(1e9 / hb_hz)));
    auto next = std::chrono::steady_clock::now() + per;
    int64_t last_sent = 0;
    while (!hb_stop.load()) {
      std::this_thread::sleep_until(next);
      if (hb_stop.load()) break;
      const auto wake = std::chrono::steady_clock::now();
      next += per;
      if (wake > next) { next = wake + per; n_hb_late++; }  // starved for > 1 period: no burst of catch-up beats
      const int64_t now = boot_ns(), fb = hb_feed_boot.load(std::memory_order_relaxed),
                    mb = hb_main_boot.load(std::memory_order_acquire);
      if (!fb || now - fb > static_cast<int64_t>(hb_feed_ms) * 1000000) { n_hb_skip_feed++; continue; }
      if (!mb || now - mb > static_cast<int64_t>(hb_main_ms) * 1000000) { n_hb_skip_main++; continue; }
      std::lock_guard<std::mutex> lk(emu);
      hb_build_send(now, hb_init.load(std::memory_order_relaxed), hb_lost.load(std::memory_order_relaxed), false, fb);
      if (last_sent && hb_gap_ms.size() < 2000000) hb_gap_ms.push_back(static_cast<float>((now - last_sent) * 1e-6));
      last_sent = now;
    }
  }

  // caller holds emu. imu_rx = receive boot time of the newest IMU record (age -> VIOP_HBF_IMU_STALE)
  void hb_build_send(int64_t now, bool initialized, bool lost, bool stopping, int64_t imu_rx) {
    hb_last = now;
    viop_heartbeat_t hb;
    std::memset(&hb, 0, sizeof(hb));
    hb.t_phone_us = static_cast<uint64_t>(now / 1000);
    hb.state = stopping ? VIOP_HB_STOPPING : !initialized ? VIOP_HB_STARTING : lost ? VIOP_HB_LOST : VIOP_HB_TRACKING;
    const int64_t imu_age = imu_rx ? (now - imu_rx) / 1000000 : 65535;
    const int64_t pose_age = pose_last ? (now - pose_last) / 1000000 : 65535;
    hb.flags = static_cast<uint8_t>((stopping ? VIOP_HBF_STOPPING : 0) | (imu_age > 100 ? VIOP_HBF_IMU_STALE : 0) |
                                    (pose_age > 200 ? VIOP_HBF_POSE_STALE : 0));
    hb.imu_age_ms = static_cast<uint16_t>(std::min<int64_t>(65535, imu_age));
    hb.pose_age_ms = static_cast<uint16_t>(std::min<int64_t>(65535, pose_age));
    hb.hb_period_ms = static_cast<uint16_t>(std::min(65535.0, 1000.0 / hb_hz));
    hb.reset_counter = reset_counter;
    hb.quality = last_quality;
    uint8_t fr[VIOP_MAX_FRAME];
    const int fs = viop_build(fr, VIOP_T_HEARTBEAT, VIOP_SRC_PHONE, hb_seq++, &hb, sizeof(hb));
    if (send_raw(fr, static_cast<size_t>(fs))) n_hb_viop++;
  }

  int viop_reset_left = VIOP_RESET_REPEAT;  // armhist 10-29: F_RESET on the first VIOP_RESET_REPEAT poses of a new W (was 1)
  static uint16_t sat16(double v) { return static_cast<uint16_t>(std::max(0.0, std::min(65535.0, std::round(v)))); }
  void emit_viop(const void *sys_epoch, int64_t t_pred_ns, int64_t t_state_ns, const Eigen::Matrix3d &R_ItoG,
                 const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &w_I, const Eigen::Vector3d &f_I,
                 const Eigen::Matrix<double, 9, 9> &P9, bool lost, int n_feat, const char *src, bool seeded = false) {
    if (sys_epoch != epoch) {
      if (epoch != nullptr) reset_counter++;
      epoch = sys_epoch;
      viop_reset_left = VIOP_RESET_REPEAT;
      std::fprintf(stderr, "[ov_bridge] viop: new world frame (reset_counter %d%s)\n", reset_counter,
                   seeded ? ", seeded from the old state" : "");
    }
    status = lost ? 5 : 4;
    const int64_t off = (replay_time && replay_off_ns != INT64_MAX) ? replay_off_ns : 0;
    const int64_t t_pred = t_pred_ns + off;
    viop_pose_t k;
    std::memset(&k, 0, sizeof(k));
    k.t_pred_us = static_cast<uint64_t>(std::max<int64_t>(0, t_pred / 1000));
    k.horizon_100us = static_cast<uint16_t>(std::max<int64_t>(0, std::min<int64_t>(65535, (t_pred_ns - t_state_ns) / 100000)));
    Eigen::Quaterniond q(R_ItoG);
    q.normalize();
    if (q.w() < 0) q.coeffs() *= -1.0;  // w >= 0 (same rotation)
    float pw[3], vw[3], wi[3];
    for (int i = 0; i < 3; i++) {
      pw[i] = static_cast<float>(p(i));
      vw[i] = static_cast<float>(v(i));
      wi[i] = static_cast<float>(w_I(i));
      k.att_std_100urad[i] = sat16(std::sqrt(std::max(0.0, P9(i, i))) * 1e4);
      k.pos_std_mm[i] = sat16(std::sqrt(std::max(0.0, P9(3 + i, 3 + i))) * 1e3);
      k.vel_std_mm_s[i] = sat16(std::sqrt(std::max(0.0, P9(6 + i, 6 + i))) * 1e3);
    }
    std::memcpy(k.p_w, pw, 12);
    std::memcpy(k.v_w, vw, 12);
    std::memcpy(k.w_i, wi, 12);
    const double qc[4] = {q.w(), q.x(), q.y(), q.z()};
    int16_t qi[4];
    for (int i = 0; i < 4; i++) qi[i] = static_cast<int16_t>(std::lround(std::max(-1.0, std::min(1.0, qc[i])) * 32767.0));
    std::memcpy(k.q_wi, qi, 8);
    const double sig_pos = std::sqrt(std::max(0.0, P9.block<3, 3>(3, 3).trace() / 3.0));
    const int quality = lost ? 0 : std::min(100, std::max(1, static_cast<int>(std::lround(100.0 * (1.0 - sig_pos / 0.20)))));
    k.flags = static_cast<uint16_t>(VIOP_F_VALID | (lost ? 0 : VIOP_F_TRACKING) |
                                    (std::strcmp(src, "frame") == 0 ? VIOP_F_FROM_FRAME : 0) |
                                    (viop_reset_left > 0 ? VIOP_F_RESET : 0) | (replay_time ? VIOP_F_REPLAY : 0) |
                                    (seeded ? VIOP_F_SEEDED : 0));
    if (viop_reset_left > 0) viop_reset_left--;
    k.reset_counter = reset_counter;
    k.quality = static_cast<uint8_t>(quality);
    k.n_feat = static_cast<uint16_t>(std::max(0, std::min(65535, n_feat)));
    const int64_t t_send = boot_ns();
    k.send_delay_us = static_cast<uint32_t>(std::max<int64_t>(0, std::min<int64_t>(0xffffffffLL, (t_send - t_pred) / 1000)));
    uint8_t fr[VIOP_MAX_FRAME];
    const int fs = viop_build(fr, VIOP_T_POSE, VIOP_SRC_PHONE, static_cast<uint16_t>(viop_seq++ & 0xffff), &k, sizeof(k));
    const bool ok = send_raw(fr, static_cast<size_t>(fs));
    const int64_t now = boot_ns();
    pose_last = now;
    last_quality = static_cast<uint8_t>(quality);
    if (ok) n_sent++; else n_fail++;
    const double age = (t_send - t_pred) * 1e-6, im = imu_rx_boot ? (now - imu_rx_boot) * 1e-6 : -1.0;
    if (age_ms.size() < 2000000) { age_ms.push_back(static_cast<float>(age)); int_ms.push_back(static_cast<float>(im)); }
    if (log)
      std::fprintf(log, "%lld,%lld,%lld,%.3f,%.3f,%s,%d,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,"
                        "%.4f,%.4f,%.4f\n",
                   static_cast<long long>(t_pred_ns), static_cast<long long>(t_pred / 1000), static_cast<long long>(t_send),
                   age, im, src, lost ? 0 : 1, p(0), p(1), p(2), q.w(), q.x(), q.y(), q.z(), v(0), v(1), v(2), w_I(0), w_I(1),
                   w_I(2), sig_pos, quality, reset_counter, f_I(0), f_I(1), f_I(2));
  }

  // --loop (2026-10-26 ovloop): loop-closure correction T_map_odom (W frame, z up; p_map = Rz(yaw) p_w + t), main thread.
  // VIOP: VIOP_T_MAP_ODOM (viop_proto.h). MAVLink: two DEBUG_VECT (#250, crc extra 49) "MAPODOM" (x y z = t) and
  // "MAPYAW" (x = yaw rad, y = target yaw rad, z = n_loops + 0.5 * converging) -> a flight controller only logs them;
  // ODOMETRY / VISION_POSITION_ESTIMATE (control) are unchanged. MAVLink values stay in the VIO world W (not in L).
  uint16_t map_seq = 0;
  uint64_t n_map = 0;
  void emit_map(const void *sys_epoch, int64_t t_ns, bool valid, bool converging, double yaw, const Eigen::Vector3d &t,
                double tgt_yaw, const Eigen::Vector3d &tgt_t, int n_loops) {
    std::lock_guard<std::mutex> lk(emu);  // epoch / reset_counter are written by emit() (prediction thread)
    if (sys_epoch != epoch) return;  // the pose stream has not announced this world frame yet
    const int64_t off = (replay_time && replay_off_ns != INT64_MAX) ? replay_off_ns : 0;
    const uint64_t t_us = static_cast<uint64_t>(std::max<int64_t>(0, (t_ns + off) / 1000));
    if (proto == 1) {
      viop_map_odom_t m;
      std::memset(&m, 0, sizeof(m));
      m.t_us = t_us;
      float a[3], b[3];
      for (int i = 0; i < 3; i++) {
        a[i] = static_cast<float>(t(i));
        b[i] = static_cast<float>(tgt_t(i));
      }
      std::memcpy(m.t_mo, a, 12);
      std::memcpy(m.tgt_t, b, 12);
      m.yaw_mo = static_cast<float>(yaw);
      m.tgt_yaw = static_cast<float>(tgt_yaw);
      m.n_loops = static_cast<uint16_t>(std::max(0, std::min(65535, n_loops)));
      m.reset_counter = reset_counter;
      m.flags = static_cast<uint8_t>((valid ? VIOP_MOF_VALID : 0) | (converging ? VIOP_MOF_CONVERGING : 0));
      uint8_t fr[VIOP_MAX_FRAME];
      const int fs = viop_build(fr, VIOP_T_MAP_ODOM, VIOP_SRC_PHONE, map_seq++, &m, sizeof(m));
      if (send_raw(fr, static_cast<size_t>(fs))) n_map++;
      return;
    }
    auto dbg = [&](const char *name, float x, float y, float z) {
      Buf p;
      p.u64(t_us);
      p.f32(x);
      p.f32(y);
      p.f32(z);
      char nm[10] = {0};
      std::strncpy(nm, name, sizeof(nm));
      for (char ch : nm) p.u8(static_cast<uint8_t>(ch));
      return send(250, 49, p.b);
    };
    bool ok = dbg("MAPODOM", static_cast<float>(t(0)), static_cast<float>(t(1)), static_cast<float>(t(2)));
    ok = dbg("MAPYAW", static_cast<float>(yaw), static_cast<float>(tgt_yaw),
             static_cast<float>(n_loops + (converging ? 0.5 : 0.0)) * (valid ? 1.0f : -1.0f)) && ok;
    if (ok) n_map++;
  }

  void report() const {
    auto q = [](std::vector<float> e, double f) {
      if (e.empty()) return -1.0;
      std::sort(e.begin(), e.end());
      return static_cast<double>(e[std::min(e.size() - 1, static_cast<size_t>(f * e.size()))]);
    };
    std::fprintf(stderr, "[ov_bridge] %s: udp %s:%d msgs %d mount %s | sent %llu fail %llu heartbeat %llu timesync %llu | "
                 "age ms (send - predicted instant%s) p50 %.2f p95 %.2f max %.2f | send - newest IMU receive ms p50 %.2f "
                 "p95 %.2f max %.2f\n", proto == 1 ? "viop" : "mav", host.c_str(), port, msgs, mount_name.c_str(), static_cast<unsigned long long>(n_sent),
                 static_cast<unsigned long long>(n_fail), static_cast<unsigned long long>(proto == 1 ? n_hb_viop : n_hb),
                 static_cast<unsigned long long>(n_ts), replay_time ? ", replay-timed" : "", q(age_ms, 0.5), q(age_ms, 0.95),
                 q(age_ms, 1.0), q(int_ms, 0.5), q(int_ms, 0.95), q(int_ms, 1.0));
    if (proto == 1 && hb_src == 1)
      std::fprintf(stderr, "[ov_bridge] viop heartbeat thread: %.1f Hz | sent gap ms p50 %.2f p95 %.2f max %.2f | skipped: feed "
                   "stale %llu main loop stale %llu | late wakeups %llu\n", hb_hz, q(hb_gap_ms, 0.5), q(hb_gap_ms, 0.95),
                   q(hb_gap_ms, 1.0), static_cast<unsigned long long>(n_hb_skip_feed),
                   static_cast<unsigned long long>(n_hb_skip_main), static_cast<unsigned long long>(n_hb_late));
  }
};

} // namespace mav
