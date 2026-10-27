// ovrefine.h (2026-10-27, branch ovrefine-20261027): "take the OpenVINS state as it is and refine only the last few
// frames" (user 10-27 11:55). No separate VINS estimator: every new snapshot builds a fresh window of the last N
// OpenVINS snapshots, initialised with the OpenVINS values, and solves it once with the VINS-Mono factors
// (IntegrationBase / IMUFactor, ProjectionFactor, PoseLocalParameterization; vins/factor, GPL-3) plus a prior on the
// first state from the OpenVINS marginal covariance (anchor). The result is an extra "ref" pose only: the filter, the
// VIO / control fields and the OpenVINS state are never changed. Plan: runs/ovrefine_20261027/PLAN.md.
// ov_bridge --refine 1 (default 0 = this code is never constructed or called).
#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class IntegrationBase;

namespace ovrefine {

struct Cfg {
  std::string log;            // --refine-log FILE: one JSON line per window
  double freq = 10.0;         // snapshot rate (Hz)
  int n = 10;                 // window size (snapshots)
  int min_obs = 3;            // a track is used when seen in >= min_obs snapshots of the window
  int min_feat = 20;          // fewer usable tracks -> window rejected
  int iters = 10;             // Ceres max iterations (deterministic: no time limit)
  double prior_scale = 1.0;   // OpenVINS covariance multiplier of the first-state prior (> 1 = looser anchor)
  double sigma_px = -1.0;     // reprojection sigma (px); < 0 = the OpenVINS up_msckf_sigma_px (set by the bridge)
  double huber = 1.0;         // Huber loss (in sigma units), VINS value
  double dp_max = 0.05;       // reject when |p_ref - p_ov| of the newest snapshot exceeds this (m)
  double dth_max_deg = 2.0;   // or the rotation difference (deg)
  double cost_max = 25.0;     // or the final cost per scalar residual exceeds this (2 * cost / num_residuals, sigma^2 units)
  double max_gap = 0.5;       // snapshot gap (s) that empties the window
  double min_depth = 0.1, max_depth = 40.0, init_px = 3.0;  // triangulation checks (m, m, px)
  int noise = 2;              // 2 = OpenVINS densities * sqrt(IMU rate) (same noise), 1 = VINS EuRoC values
  double imu_rate = 400.0;    // set by the bridge (noise 2)
  bool parse(const std::string &kv, std::string &err);
};

struct Snap {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double t = 0.0;             // IMU clock (= OpenVINS state time)
  int64_t t_ns = 0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // R_ItoG
  Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero(), bg = Eigen::Vector3d::Zero(),
                  ba = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Identity();  // OpenVINS order [th p v bg ba]
  Eigen::Matrix3d R_ItoC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinC = Eigen::Vector3d::Zero();
  double fx = 500.0;          // camera 0 focal (px of the processed image) -> sqrt_info = fx / sigma_px
  std::vector<int> id;
  std::vector<double> x, y;   // undistorted normalised
  std::shared_ptr<IntegrationBase> pre;  // IMU from the previous snapshot to this one (null for the first)
};

struct Out {  // correction published for the frames after the newest accepted window (until the next snapshot)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
  Eigen::Vector3d dt = Eigen::Vector3d::Zero();     // T_ref(t) = [dR | dt] * T_ov(t)
};

class Refiner {
public:
  Refiner(const Cfg &c, double sig_a, double sig_ab, double sig_w, double sig_wb, double g);
  ~Refiner();
  const Cfg &cfg() const { return c_; }
  bool want(double t) const { return t - last_t_ >= 1.0 / c_.freq - 0.004; }
  void push_imu(double t, const Eigen::Vector3d &a, const Eigen::Vector3d &w);
  // VIO thread, before the pose line of this frame: adds the snapshot and solves the window (inline)
  void add(std::unique_ptr<Snap> s);
  void reset(const char *why);  // OpenVINS re-initialised / lost: empty the window, correction off
  const Out &out() const { return out_; }
  uint64_t n_win() const { return n_win_; }
  uint64_t n_ok() const { return n_ok_; }
  uint64_t n_rej() const { return n_rej_; }
  void report() const;

private:
  struct Imu {
    double t;
    Eigen::Vector3d a, w;
  };
  Imu imu_at(double t) const;  // linear interpolation in the buffer (held at the ends)
  std::shared_ptr<IntegrationBase> integrate(double t0, double t1, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg);
  void solve();
  void set_globals() const;
  void log_line(const std::string &s);

  Cfg c_;
  double acc_n_, gyr_n_, acc_w_, gyr_w_, g_;
  double last_t_ = -1e300;
  std::deque<Imu> imu_;
  std::deque<std::unique_ptr<Snap>> win_;
  Out out_;
  uint64_t n_snap_ = 0, n_win_ = 0, n_ok_ = 0, n_rej_ = 0, n_reset_ = 0;
  uint64_t rej_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<double> ms_, ms_solve_;
  std::vector<int> nfeat_, iters_;
  std::vector<double> dp_ok_;
  FILE *log_ = nullptr;
};

}  // namespace ovrefine
