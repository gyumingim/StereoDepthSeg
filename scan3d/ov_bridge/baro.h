// baro.h — barometer-aided height for ov_bridge (2026-10-26, branch ovbaro-20261026; default off = never called).
//
// Plan / evidence: scan3d/runs/ovbaro_20261026/PLAN.md + README.md.
// Measurement: h = (R*T/g) * ln(p0 / p)  [m, relative to the first sample p0], sample time t_b on the IMU clock (s).
// Model:       h = p_z(t_state) + v_z * (t_b - t_state_imu) + b_c + b_g + n
//   b_c  slow bias (weather / temperature), random walk rw [m/sqrt(s)]
//   b_g  indoor pressure fluctuation, first-order Gauss-Markov (sigma gm_sigma [m], time constant gm_tau [s])
//   n    white noise sn [m]
// The two biases are two 1-D ov_type::Vec appended to the OpenVINS state with StateHelper::initialize_invertible (like a
// SLAM feature) when the first sample is accepted: b_c = h - p_z (cross-covariance -P(p_z, .), so p does not move), b_g = 0
// (var gm_sigma^2, uncorrelated). Their propagation uses StateHelper::EKFPropagation, updates StateHelper::EKFUpdate
// (public OpenVINS API only; no ovpatch change). A new VioManager (reset / reinit / recover) has a new
// State -> the biases are anchored again.
// Gates per sample (status in --baro-log): not initialised / settle s after (re)init / lost -> unused; |v_z| > vz_max;
// ground_m > 0: below anchor height + ground_m; STEP: the innovation jumps by > step_jump between consecutive samples ->
// no updates until it passes chi2 again (2 samples, short burst) or, after reanchor s with a steady innovation, b_c is moved
// by the median innovation (permanent step, e.g. a door) -> p never jumps; chi2 > chi2 (not a step) -> Huber update
// (R inflated so the normalised innovation equals chi2: bounded pull per sample).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Vec.h"

namespace baro {

struct Cfg {
  double sn = 0.02;         // white noise (m)
  double gm_sigma = 0.25;   // Gauss-Markov bias sigma (m)
  double gm_tau = 10.0;     // Gauss-Markov time constant (s)
  double rw = 0.0065;       // slow bias random walk (m/sqrt(s))
  double b0_sigma = 0.05;   // extra initial uncertainty of b_c beyond P(p_z) (m)
  double chi2 = 9.0;        // 1-dof gate (3 sigma)
  double vz_max = 1.5;      // m/s
  double settle = 2.0;      // s after (re)initialisation without baro updates
  double step_jump = 0.15;  // m: innovation change between consecutive samples = step / burst
  double reanchor = 1.5;    // s of STEP before b_c is re-anchored
  double steady = 0.10;     // m: max spread (p90-p10) of the last 0.5 s of innovations for a re-anchor
  double temp_c = 20.0;     // air temperature for the hypsometric scale
  double delay_ms = 0.0;    // sensor delay: sample time = t_ns - delay
  double ground_m = 0.0;    // 0 = off; > 0: no updates below anchor height + ground_m (drone ground effect)
  double max_age = 0.30;    // s: samples older than the state by more than this are dropped
  bool parse(const std::string &v) {
    std::stringstream ss(v);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
      if (kv.empty()) continue;
      auto e = kv.find('=');
      if (e == std::string::npos) return false;
      std::string k = kv.substr(0, e);
      double x = std::atof(kv.c_str() + e + 1);
      if (k == "sn") sn = x;
      else if (k == "gm_sigma") gm_sigma = x;
      else if (k == "gm_tau") gm_tau = x;
      else if (k == "rw") rw = x;
      else if (k == "b0_sigma") b0_sigma = x;
      else if (k == "chi2") chi2 = x;
      else if (k == "vz_max") vz_max = x;
      else if (k == "settle") settle = x;
      else if (k == "step_jump") step_jump = x;
      else if (k == "reanchor") reanchor = x;
      else if (k == "steady") steady = x;
      else if (k == "temp_c") temp_c = x;
      else if (k == "delay_ms") delay_ms = x;
      else if (k == "ground_m") ground_m = x;
      else if (k == "max_age") max_age = x;
      else return false;
    }
    return sn > 0 && gm_sigma >= 0 && gm_tau > 0 && rw >= 0 && chi2 > 0;
  }
  std::string describe() const {
    char b[512];
    std::snprintf(b, sizeof(b), "sn=%g,gm_sigma=%g,gm_tau=%g,rw=%g,b0_sigma=%g,chi2=%g,vz_max=%g,settle=%g,step_jump=%g,"
                  "reanchor=%g,steady=%g,temp_c=%g,delay_ms=%g,ground_m=%g,max_age=%g", sn, gm_sigma, gm_tau, rw, b0_sigma,
                  chi2, vz_max, settle, step_jump, reanchor, steady, temp_c, delay_ms, ground_m, max_age);
    return b;
  }
};

enum Status { USED = 0, HUBER, NOT_INIT, SETTLE, LOST, OLD, VZ, GROUND, STEP_HOLD, STEP_END, REANCHOR, ANCHOR, FAIL, N_STATUS };
inline const char *status_name(int s) {
  static const char *n[] = {"used", "huber", "not_init", "settle", "lost", "old", "vz", "ground", "step_hold", "step_end",
                            "reanchor", "anchor", "fail"};
  return s >= 0 && s < N_STATUS ? n[s] : "?";
}

struct Sample {
  double t;    // IMU clock (s, bridge time base), already minus delay
  int64_t t_ns;
  double hpa;
};

class Aid {
public:
  Cfg cfg;
  FILE *log = nullptr;
  long n_rx = 0, n_status[N_STATUS] = {0};
  int n_anchor = 0, n_reanchor = 0;
  double max_abs_corr = 0.0;  // largest |dp_z| one update applied (m)

  // a sample arrived (record type 7 or --baro-csv); t = IMU-clock seconds
  void push(int64_t t_ns, double t, double hpa) {
    if (!(hpa > 100.0 && hpa < 1300.0)) return;
    n_rx++;
    if (!have_p0_) {
      p0_ = hpa;
      have_p0_ = true;
    }
    q_.push_back({t - cfg.delay_ms * 1e-3, t_ns, hpa});
  }
  double height(double hpa) const { return 287.05 * (cfg.temp_c + 273.15) / 9.80665 * std::log(p0_ / hpa); }

  // after a camera update. initialised / lost from the bridge; state = current VioManager state.
  void update(const std::shared_ptr<ov_msckf::State> &st, bool initialised, bool lost) {
    if (q_.empty()) return;
    if (!initialised || !st) {
      while (!q_.empty()) finish(q_.front(), NOT_INIT, NAN, NAN, NAN), q_.pop_front();
      st_ = nullptr;
      return;
    }
    if (st.get() != st_) {  // new VioManager (or first time initialised)
      st_ = st.get();
      vc_.reset();
      vg_.reset();
      t_init_ = st->_timestamp;
      mode_step_ = false;
      have_r_last_ = false;
      t_prop_ = -1e300;
    }
    const double t_off = st->_calib_dt_CAMtoIMU->value()(0);
    const double t_si = st->_timestamp + t_off;  // state time on the IMU clock
    if (vc_) propagate(st, t_si);
    while (!q_.empty() && q_.front().t <= t_si + 1e-6) {
      Sample s = q_.front();
      q_.pop_front();
      one(st, s, t_si, lost);
    }
  }

  void json(std::ostringstream &o) const {  // pose line "baro":[used, huber, rejected, b_c, b_g, mode_step]
    long rej = 0;
    for (int k = NOT_INIT; k < N_STATUS; k++)
      if (k != ANCHOR && k != REANCHOR && k != STEP_END) rej += n_status[k];
    o << ",\"baro\":[" << n_status[USED] << "," << n_status[HUBER] << "," << rej << ",";
    if (vc_) {
      o.precision(6);
      o << vc_->value()(0) << "," << vg_->value()(0);
    } else
      o << "null,null";
    o << "," << (mode_step_ ? 1 : 0) << "]";
  }
  void summary() const {
    std::fprintf(stderr, "[ov_bridge] baro: rx %ld |", n_rx);
    for (int k = 0; k < N_STATUS; k++)
      if (n_status[k]) std::fprintf(stderr, " %s %ld", status_name(k), n_status[k]);
    std::fprintf(stderr, " | anchors %d reanchors %d | max |dp_z| per update %.4f m | %s\n", n_anchor, n_reanchor,
                 max_abs_corr, cfg.describe().c_str());
  }

private:
  std::deque<Sample> q_;
  bool have_p0_ = false;
  double p0_ = 0.0;
  const ov_msckf::State *st_ = nullptr;
  std::shared_ptr<ov_type::Vec> vc_, vg_;  // b_c, b_g
  double t_init_ = 0.0, t_prop_ = -1e300, anchor_z_ = 0.0;
  bool mode_step_ = false, have_r_last_ = false;
  double r_last_ = 0.0, t_r_last_ = -1e300, step_t0_ = 0.0;
  int step_ok_ = 0;
  std::deque<std::pair<double, double>> step_r_;  // (t, innovation) while in STEP

  void finish(const Sample &s, int status, double h, double r, double S, double z = NAN, double dz = 0.0) {
    n_status[status]++;
    if (!log) return;
    std::fprintf(log, "%lld,%.6f,%.5f,%.4f,%.4f,%.4f,%.5f,%s,%.4f,%.4f,%.5f,%d\n", static_cast<long long>(s.t_ns), s.t, s.hpa, h, z, r,
                 std::isfinite(S) ? std::sqrt(S) : NAN, status_name(status), vc_ ? vc_->value()(0) : NAN,
                 vg_ ? vg_->value()(0) : NAN, dz, mode_step_ ? 1 : 0);
  }

  // Gauss-Markov b_g and random-walk b_c from t_prop_ to t (covariance and mean)
  void propagate(const std::shared_ptr<ov_msckf::State> &st, double t) {
    if (t_prop_ < -1e299) {
      t_prop_ = t;
      return;
    }
    double dt = t - t_prop_;
    if (dt <= 0) return;
    t_prop_ = t;
    const double phi = std::exp(-dt / cfg.gm_tau);
    Eigen::MatrixXd Phi(1, 1), Q(1, 1);
    Phi(0, 0) = phi;
    Q(0, 0) = cfg.gm_sigma * cfg.gm_sigma * (1.0 - phi * phi);
    ov_msckf::StateHelper::EKFPropagation(st, {vg_}, {vg_}, Phi, Q);
    Eigen::VectorXd v = vg_->value();
    v(0) *= phi;
    vg_->set_value(v);
    add_var_c(st, cfg.rw * cfg.rw * dt);
  }

  void add_var_c(const std::shared_ptr<ov_msckf::State> &st, double q) {
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(1, 1), Q(1, 1);
    Q(0, 0) = q;
    ov_msckf::StateHelper::EKFPropagation(st, {vc_}, {vc_}, Phi, Q);
  }

  void anchor(const std::shared_ptr<ov_msckf::State> &st, double h, double dtb) {
    auto pv = st->_imu->p(), vv = st->_imu->v();
    const double zp = pv->value()(2) + vv->value()(2) * dtb;
    vc_ = std::make_shared<ov_type::Vec>(1);
    vg_ = std::make_shared<ov_type::Vec>(1);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);
    vc_->set_value(zero);
    vc_->set_fej(zero);
    vg_->set_value(zero);
    vg_->set_fej(zero);
    Eigen::MatrixXd HR = Eigen::MatrixXd::Zero(1, 6), HL = Eigen::MatrixXd::Identity(1, 1), R(1, 1);
    HR(0, 2) = 1.0;
    HR(0, 5) = dtb;
    R(0, 0) = cfg.sn * cfg.sn + cfg.b0_sigma * cfg.b0_sigma;
    Eigen::VectorXd res(1);
    res(0) = h - zp;  // b_c = h - z_p (only the new variable is updated)
    ov_msckf::StateHelper::initialize_invertible(st, vc_, {pv, vv}, HR, HL, R, res);
    Eigen::MatrixXd HR0 = Eigen::MatrixXd::Zero(1, 3);
    R(0, 0) = cfg.gm_sigma * cfg.gm_sigma;
    ov_msckf::StateHelper::initialize_invertible(st, vg_, {pv}, HR0, HL, R, zero);
    anchor_z_ = zp;
    n_anchor++;
  }

  void one(const std::shared_ptr<ov_msckf::State> &st, const Sample &s, double t_si, bool lost) {
    const double h = height(s.hpa);
    if (lost) return finish(s, LOST, h, NAN, NAN);
    if (st->_timestamp - t_init_ < cfg.settle) return finish(s, SETTLE, h, NAN, NAN);
    const double dtb = s.t - t_si;  // <= 0
    if (dtb < -cfg.max_age) return finish(s, OLD, h, NAN, NAN);
    auto pv = st->_imu->p(), vv = st->_imu->v();
    const double vz = vv->value()(2), z = pv->value()(2) + vz * dtb;
    if (std::fabs(vz) > cfg.vz_max) return finish(s, VZ, h, NAN, NAN, z);
    if (!vc_) {
      anchor(st, h, dtb);
      t_prop_ = t_si;
      return finish(s, ANCHOR, h, 0.0, NAN, z);
    }
    if (cfg.ground_m > 0 && z < anchor_z_ + cfg.ground_m) return finish(s, GROUND, h, NAN, NAN, z);
    std::vector<std::shared_ptr<ov_type::Type>> order = {pv, vv, vc_, vg_};
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 8);
    H(0, 2) = 1.0;
    H(0, 5) = dtb;
    H(0, 6) = 1.0;
    H(0, 7) = 1.0;
    const double r = h - (z + vc_->value()(0) + vg_->value()(0));
    Eigen::MatrixXd Ps = ov_msckf::StateHelper::get_marginal_covariance(st, order);
    const double hph = (H * Ps * H.transpose())(0, 0);
    const double R0 = cfg.sn * cfg.sn, S = hph + R0;
    const bool jump = have_r_last_ && s.t - t_r_last_ < 0.5 && std::fabs(r - r_last_) > cfg.step_jump;
    have_r_last_ = true;
    r_last_ = r;
    t_r_last_ = s.t;
    if (!mode_step_ && jump) {
      mode_step_ = true;
      step_t0_ = s.t;
      step_ok_ = 0;
      step_r_.clear();
    }
    if (mode_step_) {
      step_r_.push_back({s.t, r});
      while (!step_r_.empty() && step_r_.front().first < s.t - 0.5) step_r_.pop_front();
      if (r * r / S <= cfg.chi2) {
        if (++step_ok_ >= 2) {
          mode_step_ = false;
          return finish(s, STEP_END, h, r, S, z);
        }
        return finish(s, STEP_HOLD, h, r, S, z);
      }
      step_ok_ = 0;
      if (s.t - step_t0_ >= cfg.reanchor && step_r_.size() >= 4) {
        std::vector<double> rr;
        for (auto &x : step_r_) rr.push_back(x.second);
        std::sort(rr.begin(), rr.end());
        const size_t n = rr.size(), hi = std::min(n - 1, (n * 9) / 10);
        const double med = rr[n / 2], spread = rr[hi] - rr[n / 10];
        if (spread <= cfg.steady) {
          Eigen::VectorXd v = vc_->value();
          v(0) += med;
          vc_->set_value(v);
          add_var_c(st, cfg.sn * cfg.sn + cfg.b0_sigma * cfg.b0_sigma);
          mode_step_ = false;
          have_r_last_ = false;
          n_reanchor++;
          return finish(s, REANCHOR, h, r, S, z);
        }
      }
      return finish(s, STEP_HOLD, h, r, S, z);
    }
    double R = R0;
    int status = USED;
    if (r * r / S > cfg.chi2) {  // Huber: normalised innovation limited to sqrt(chi2)
      R = std::max(R0, r * r / cfg.chi2 - hph);
      status = HUBER;
    }
    Eigen::VectorXd res(1);
    res(0) = r;
    Eigen::MatrixXd Rm(1, 1);
    Rm(0, 0) = R;
    const double z_before = pv->value()(2);
    try {
      ov_msckf::StateHelper::EKFUpdate(st, order, H, res, Rm);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] baro: EKFUpdate exception %s\n", e.what());
      return finish(s, FAIL, h, r, S, z);
    }
    const double dz = pv->value()(2) - z_before;
    max_abs_corr = std::max(max_abs_corr, std::fabs(dz));
    finish(s, status, h, r, S, z, dz);
  }
};

}  // namespace baro
