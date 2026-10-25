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
            double sigma_w2_rate, const Eigen::Vector3d &f_I, const char *src, int64_t t_state_ns = 0, int n_feat = 0) {
    if (fd < 0) return;
    std::lock_guard<std::mutex> lk(emu);
    if (proto == 1) {
      emit_viop(sys_epoch, t_pred_ns, t_state_ns, R_ItoG, p_IinG, v_IinG, w_I, f_I, P9, lost, n_feat, src);
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
    hb_last = now;
    viop_heartbeat_t hb;
    std::memset(&hb, 0, sizeof(hb));
    hb.t_phone_us = static_cast<uint64_t>(now / 1000);
    hb.state = stopping ? VIOP_HB_STOPPING : !initialized ? VIOP_HB_STARTING : lost ? VIOP_HB_LOST : VIOP_HB_TRACKING;
    const int64_t imu_age = imu_rx_boot ? (now - imu_rx_boot) / 1000000 : 65535;
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

  bool viop_reset_pending = true;
  static uint16_t sat16(double v) { return static_cast<uint16_t>(std::max(0.0, std::min(65535.0, std::round(v)))); }
  void emit_viop(const void *sys_epoch, int64_t t_pred_ns, int64_t t_state_ns, const Eigen::Matrix3d &R_ItoG,
                 const Eigen::Vector3d &p, const Eigen::Vector3d &v, const Eigen::Vector3d &w_I, const Eigen::Vector3d &f_I,
                 const Eigen::Matrix<double, 9, 9> &P9, bool lost, int n_feat, const char *src) {
    if (sys_epoch != epoch) {
      if (epoch != nullptr) reset_counter++;
      epoch = sys_epoch;
      viop_reset_pending = true;
      std::fprintf(stderr, "[ov_bridge] viop: new world frame (reset_counter %d)\n", reset_counter);
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
                                    (viop_reset_pending ? VIOP_F_RESET : 0) | (replay_time ? VIOP_F_REPLAY : 0));
    viop_reset_pending = false;
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
  }
};

} // namespace mav
