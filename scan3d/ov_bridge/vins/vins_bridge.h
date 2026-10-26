// vins_bridge.h (2026-10-26, branch ovswba-20261026): VINS-Mono sliding-window estimator (vins/, ported almost as-is
// from HKUST-Aerial-Robotics/VINS-Mono 90dabb5, GPL-3) running next to OpenVINS in ov_bridge.
//
// ov_bridge --swba 1 (default 0 = this code is never called, poses bit-identical). Design (runs/ovswba_20261026/PLAN.md):
//  - Front-end = the OpenVINS KLT tracker (read only, like --loop): every ~1/freq s (VINS default freq 10 Hz; 60 fps ->
//    every 6th frame) the VIO thread copies the current frame's tracked points (id, undistorted normalised x y, pixel u v)
//    and the OpenVINS pose; every IMU sample fed to the filter is copied too.
//  - VINS-Mono Estimator (processIMU / processImage, its own initialisation, sliding window 10, IMU pre-integration,
//    marginalisation) runs on those inputs, in its own thread (or inline with --swba-sync 1 for deterministic PC runs).
//    The IMU / frame loop is VINS estimator_node.cpp process() (IMU interpolated at the frame time).
//  - Output (integration choice (b)): the OpenVINS / control pose is NOT changed. VINS gives a 4-DoF correction
//    T_swba_odom (yaw about gravity + translation, same form as the loop-closure map frame): its target makes the
//    corrected OpenVINS pose of the latest VINS frame equal the (aligned) VINS pose; the published value follows the
//    target with the same rate limiter as ovloop (0.10 m/s, 3 deg/s) -> the corrected pose never jumps.
//    Alignment: the first VINS solve (and every VINS restart) sets A (VINS world -> OpenVINS world, 4-DoF) so that the
//    target equals the published value at that moment (no step).
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ovloop.h"  // ovloop::MapOdom (same published-correction struct and JSON writer as the loop map frame)

namespace ovswba {

struct Cfg {
  std::string log;           // --swba-log FILE: JSON line per VINS frame (pose, timing, flags)
  int sync = 0;              // --swba-sync 1: run VINS inline in the VIO thread (deterministic PC evaluation only)
  double freq = 10.0;        // VINS input rate (Hz) = VINS feature_tracker "freq" (default 10)
  int noise = 1;             // 1 = VINS EuRoC config values (acc_n 0.08 gyr_n 0.004 acc_w 4e-5 gyr_w 2e-6),
                             // 2 = OpenVINS continuous densities * sqrt(IMU rate) (white and random walk)
  double acc_n = -1, gyr_n = -1, acc_w = -1, gyr_w = -1;  // explicit override (> 0)
  double solver_time = 0.04;   // VINS max_solver_time (s)
  int iters = 8;               // VINS max_num_iterations
  double parallax_px = 10.0;   // VINS keyframe_parallax (pixel at FOCAL_LENGTH 460)
  double imu_rate = 400.0;     // for noise 2 (set by the bridge from the config)
  int queue_max = 4;           // frames waiting (async); more -> the oldest is dropped (its IMU stays in the buffer)
  int nice = 10;               // VINS thread nice value
  int log_level = 1;           // VINS console messages (0 quiet, 1 warn, 2 info, 3 debug)
  double smooth_v = 0.10;      // published correction rate limit (m/s of the current body position)
  double smooth_w_deg = 3.0;   // and yaw (deg/s)
  bool parse(const std::string &kv, std::string &err);
};

struct FrameIn {
  double t = 0.0;              // IMU clock (= OpenVINS camera time + dt_CAMtoIMU)
  int64_t t_ns = 0;            // camera t_ns (log)
  std::vector<int> id;
  std::vector<float> x, y, u, v;  // undistorted normalised, pixel
  bool ov_valid = false;       // OpenVINS initialised and tracking at this frame
  Eigen::Matrix3d R_ItoG = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  double cap_ms = 0.0;
};

class Runner {
public:
  // R_ItoC / p_IinC: OpenVINS camera extrinsics (VINS RIC = R_ItoC^T, TIC = -R_ItoC^T p_IinC); sigmas: OpenVINS
  // continuous noise densities (only for noise 2); g = gravity magnitude
  Runner(const Cfg &c, const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC, double sig_a, double sig_ab,
         double sig_w, double sig_wb, double g);
  ~Runner();
  const Cfg &cfg() const { return c_; }
  // VIO thread
  bool want_frame(double t) const { return t - last_push_t_ >= 1.0 / c_.freq - 0.004; }
  void push_imu(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr);
  void push_frame(std::unique_ptr<FrameIn> f);
  void reset_session(const char *why);  // OpenVINS re-initialised: new OpenVINS world -> new alignment
  // output (main thread steps the smoothing once per frame, like ovloop)
  ovloop::MapOdom step_output(double t, const Eigen::Vector3d &p_vio);
  ovloop::MapOdom published() const;
  void finish(double timeout_s);
  void report() const;

private:
  struct Imu {
    double t;
    Eigen::Vector3d a, w;
  };
  void run();
  void drain();  // = VINS estimator_node getMeasurements() + process() for every complete measurement
  void process(std::vector<Imu> &imu, FrameIn &f);
  void log_line(const std::string &s);

  Cfg c_;
  double last_push_t_ = -1e300;
  std::thread th_;
  mutable std::mutex qm_;
  std::condition_variable qcv_, done_cv_;
  std::deque<Imu> imu_;
  std::deque<std::unique_ptr<FrameIn>> fr_;
  bool stop_ = false;
  uint64_t n_pushed_ = 0, n_done_ = 0, n_dropped_ = 0, n_old_ = 0;
  std::atomic<int> reset_req_{0};
  std::mutex vm_;  // VINS state (drain may run in the VIO thread with sync, or the worker)
  double cur_t_ = -1.0;  // estimator_node current_time
  // alignment / output (VINS thread writes the target, main thread steps)
  bool have_A_ = false;
  double A_yaw_ = 0.0;
  Eigen::Vector3d A_t_ = Eigen::Vector3d::Zero();
  int vins_resets_seen_ = 0;
  mutable std::mutex om_;
  ovloop::MapOdom out_;
  double out_last_t_ = 0.0;
  bool out_have_t_ = false;
  // stats
  std::vector<double> ms_total_, ms_solve_, ms_margin_, ms_cap_;
  std::vector<int> iters_;
  uint64_t n_frames_ = 0, n_nonlin_ = 0, n_kf_ = 0, n_fail_ = 0, n_init_ = 0;
  FILE *log_ = nullptr;
};

}  // namespace ovswba
