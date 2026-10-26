// vins_bridge.cpp (2026-10-26, ovswba): see vins_bridge.h. The measurement loop (drain / process) is VINS-Mono
// estimator_node.cpp getMeasurements() + process() without ROS (relocalisation and publishing removed).
// GPL-3.0 (VINS-Mono, OpenVINS).
#include "vins_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>
#include <utility>

#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "estimator.h"
#include "parameters.h"

namespace ovswba {
namespace {

// One VINS Estimator per process, with static storage like the global "Estimator estimator;" in estimator_node.cpp:
// its members without initialisers (pre_integrations[], initial_timestamp, ...) must start zeroed, as in VINS.
Estimator &est() {
  static Estimator e;
  return e;
}

double wrap(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}
Eigen::Matrix3d Rz(double y) {
  Eigen::Matrix3d R;
  R << std::cos(y), -std::sin(y), 0, std::sin(y), std::cos(y), 0, 0, 0, 1;
  return R;
}
double yaw_of(const Eigen::Matrix3d &R) { return std::atan2(R(1, 0), R(0, 0)); }
double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t i = std::min(v.size() - 1, static_cast<size_t>(p / 100.0 * (v.size() - 1) + 0.5));
  return v[i];
}
void jv(std::ostringstream &o, const Eigen::Vector3d &v) {
  char b[128];
  std::snprintf(b, sizeof b, "[%.6f,%.6f,%.6f]", v(0), v(1), v(2));
  o << b;
}
void jq(std::ostringstream &o, const Eigen::Matrix3d &R) {
  Eigen::Quaterniond q(R);
  char b[160];
  std::snprintf(b, sizeof b, "[%.8f,%.8f,%.8f,%.8f]", q.x(), q.y(), q.z(), q.w());
  o << b;
}

}  // namespace

bool Cfg::parse(const std::string &kv, std::string &err) {
  std::stringstream ss(kv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    const size_t e = item.find('=');
    if (e == std::string::npos) {
      err = "--swba-cfg: expected K=V, got '" + item + "'";
      return false;
    }
    const std::string k = item.substr(0, e);
    double v = 0.0;
    try {
      v = std::stod(item.substr(e + 1));
    } catch (...) {
      err = "--swba-cfg: bad number in '" + item + "'";
      return false;
    }
    if (k == "freq") freq = v;
    else if (k == "noise") noise = static_cast<int>(v);
    else if (k == "acc_n") acc_n = v;
    else if (k == "gyr_n") gyr_n = v;
    else if (k == "acc_w") acc_w = v;
    else if (k == "gyr_w") gyr_w = v;
    else if (k == "solver_time") solver_time = v;
    else if (k == "iters") iters = static_cast<int>(v);
    else if (k == "parallax_px") parallax_px = v;
    else if (k == "queue_max") queue_max = static_cast<int>(v);
    else if (k == "nice") nice = static_cast<int>(v);
    else if (k == "log_level") log_level = static_cast<int>(v);
    else if (k == "smooth_v") smooth_v = v;
    else if (k == "smooth_w_deg") smooth_w_deg = v;
    else {
      err = "--swba-cfg: unknown key '" + k + "'";
      return false;
    }
  }
  if (freq <= 0.0 || iters < 1 || solver_time <= 0.0 || queue_max < 1 || (noise != 1 && noise != 2)) {
    err = "--swba-cfg: out of range (freq > 0, iters >= 1, solver_time > 0, queue_max >= 1, noise 1|2)";
    return false;
  }
  return true;
}

Runner::Runner(const Cfg &c, const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC, double sig_a, double sig_ab,
               double sig_w, double sig_wb, double g)
    : c_(c) {
  // = VINS readParameters() with a config: fixed extrinsic (ESTIMATE_EXTRINSIC 0), synchronised (td 0, not estimated),
  // global shutter model, INIT_DEPTH / bias thresholds as in VINS.
  vins_log_level = c_.log_level;
  RIC.clear();
  TIC.clear();
  RIC.push_back(R_ItoC.transpose());              // camera -> IMU rotation
  TIC.push_back(-R_ItoC.transpose() * p_IinC);    // camera origin in the IMU frame
  ESTIMATE_EXTRINSIC = 0;
  ESTIMATE_TD = 0;
  TD = 0.0;
  ROLLING_SHUTTER = 0;
  TR = 0.0;
  INIT_DEPTH = 5.0;
  BIAS_ACC_THRESHOLD = 0.1;
  BIAS_GYR_THRESHOLD = 0.1;
  SOLVER_TIME = c_.solver_time;
  NUM_ITERATIONS = c_.iters;
  MIN_PARALLAX = c_.parallax_px / FOCAL_LENGTH;
  G = Eigen::Vector3d(0.0, 0.0, g);
  if (c_.noise == 1) {  // VINS-Mono config/euroc/euroc_config.yaml
    ACC_N = 0.08;
    GYR_N = 0.004;
    ACC_W = 0.00004;
    GYR_W = 2.0e-6;
  } else {  // OpenVINS continuous densities -> per-sample standard deviation (VINS propagates noise * dt per sample)
    const double s = std::sqrt(c_.imu_rate);
    ACC_N = sig_a * s;
    GYR_N = sig_w * s;
    ACC_W = sig_ab * s;
    GYR_W = sig_wb * s;
  }
  if (c_.acc_n > 0) ACC_N = c_.acc_n;
  if (c_.gyr_n > 0) GYR_N = c_.gyr_n;
  if (c_.acc_w > 0) ACC_W = c_.acc_w;
  if (c_.gyr_w > 0) GYR_W = c_.gyr_w;
  est().clearState();
  est().setParameter();
  if (!c_.log.empty()) log_ = std::fopen(c_.log.c_str(), "w");
  std::fprintf(stderr, "[ov_bridge] swba: VINS-Mono window %d, freq %.1f Hz, iters %d, solver %.3f s, parallax %.1f px, noise %d "
               "(acc_n %.3g gyr_n %.3g acc_w %.3g gyr_w %.3g), g %.4f, %s\n", WINDOW_SIZE, c_.freq, NUM_ITERATIONS,
               SOLVER_TIME, c_.parallax_px, c_.noise, ACC_N, GYR_N, ACC_W, GYR_W, g, c_.sync ? "sync (inline)" : "own thread");
  if (!c_.sync) th_ = std::thread([this] { run(); });
}

Runner::~Runner() {
  {
    std::lock_guard<std::mutex> l(qm_);
    stop_ = true;
  }
  qcv_.notify_all();
  if (th_.joinable()) th_.join();
  if (log_) std::fclose(log_);
}

void Runner::push_imu(double t, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr) {
  bool have_frame;
  {
    std::lock_guard<std::mutex> l(qm_);
    imu_.push_back({t, acc, gyr});
    // bound: before the first frame keep 1 s (VINS integrates everything before its first frame into nothing useful);
    // later 5 s (frames stopped); dropped samples are counted as old
    const double keep = cur_t_ < 0.0 && fr_.empty() ? 1.0 : 5.0;
    while (!imu_.empty() && imu_.front().t < t - keep) imu_.pop_front();
    have_frame = !fr_.empty();
  }
  if (!have_frame) return;
  if (c_.sync) drain();
  else qcv_.notify_one();
}

void Runner::push_frame(std::unique_ptr<FrameIn> f) {
  last_push_t_ = f->t;
  {
    std::lock_guard<std::mutex> l(qm_);
    fr_.push_back(std::move(f));
    n_pushed_++;
    while (static_cast<int>(fr_.size()) > c_.queue_max) {  // async only in practice (sync drains every time)
      fr_.pop_front();
      n_dropped_++;
      n_done_++;
    }
  }
  if (c_.sync) drain();
  else qcv_.notify_one();
}

void Runner::reset_session(const char *why) {
  std::lock_guard<std::mutex> l(om_);
  have_A_ = false;  // new OpenVINS world: the next VINS frame realigns (target = published at that moment)
  if (log_) std::fprintf(log_, "{\"type\":\"ov_reset\",\"why\":\"%s\"}\n", why);
}

void Runner::run() {
#ifdef __linux__
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), c_.nice);
#endif
  while (true) {
    {
      std::unique_lock<std::mutex> l(qm_);
      qcv_.wait(l, [&] { return stop_ || (!fr_.empty() && !imu_.empty() && imu_.back().t > fr_.front()->t); });
      if (stop_) return;
    }
    drain();
  }
}

// VINS estimator_node.cpp getMeasurements(): a frame is complete once an IMU sample newer than it exists; frames
// older than the oldest buffered IMU are thrown away; the IMU samples before the frame are popped, the first one after
// it is used (for interpolation) but stays in the buffer.
void Runner::drain() {
  std::lock_guard<std::mutex> lv(vm_);
  while (true) {
    std::vector<Imu> imu;
    std::unique_ptr<FrameIn> f;
    {
      std::lock_guard<std::mutex> l(qm_);
      if (fr_.empty() || imu_.empty()) return;
      const double img_t = fr_.front()->t + est().td;
      if (!(imu_.back().t > img_t)) return;
      if (!(imu_.front().t < img_t)) {  // "throw img, only should happen at the beginning"
        fr_.pop_front();
        n_old_++;
        n_done_++;
        done_cv_.notify_all();
        continue;
      }
      f = std::move(fr_.front());
      fr_.pop_front();
      while (imu_.front().t < img_t) {
        imu.push_back(imu_.front());
        imu_.pop_front();
      }
      imu.push_back(imu_.front());
    }
    process(imu, *f);
    {
      std::lock_guard<std::mutex> l(qm_);
      n_done_++;
    }
    done_cv_.notify_all();
  }
}

void Runner::process(std::vector<Imu> &imu, FrameIn &f) {
  const auto t0 = std::chrono::steady_clock::now();
  Estimator &E = est();
  // = estimator_node.cpp process(): IMU up to the frame time, the last sample interpolated at the frame time
  const double img_t = f.t + E.td;
  double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
  for (const auto &m : imu) {
    const double t = m.t;
    if (t <= img_t) {
      if (cur_t_ < 0) cur_t_ = t;
      const double dt = t - cur_t_;
      ROS_ASSERT(dt >= 0);
      cur_t_ = t;
      dx = m.a(0); dy = m.a(1); dz = m.a(2);
      rx = m.w(0); ry = m.w(1); rz = m.w(2);
      E.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
    } else {
      const double dt_1 = img_t - cur_t_;
      const double dt_2 = t - img_t;
      cur_t_ = img_t;
      ROS_ASSERT(dt_1 >= 0);
      ROS_ASSERT(dt_2 >= 0);
      ROS_ASSERT(dt_1 + dt_2 > 0);
      const double w1 = dt_2 / (dt_1 + dt_2);
      const double w2 = dt_1 / (dt_1 + dt_2);
      dx = w1 * dx + w2 * m.a(0);
      dy = w1 * dy + w2 * m.a(1);
      dz = w1 * dz + w2 * m.a(2);
      rx = w1 * rx + w2 * m.w(0);
      ry = w1 * ry + w2 * m.w(1);
      rz = w1 * rz + w2 * m.w(2);
      E.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
    }
  }
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
  for (size_t i = 0; i < f.id.size(); i++) {
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    xyz_uv_velocity << f.x[i], f.y[i], 1.0, f.u[i], f.v[i], 0.0, 0.0;  // velocity only used with ESTIMATE_TD
    image[f.id[i]].emplace_back(0, xyz_uv_velocity);
  }
  std_msgs::Header header;
  header.stamp.t = f.t;
  const bool was_nl = E.solver_flag == Estimator::NON_LINEAR;
  E.ovswba_solver_ms = E.ovswba_margin_ms = 0.0;
  E.ovswba_iters = 0;
  E.failure_occur = false;
  E.processImage(image, header);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  n_frames_++;
  const bool fail = E.failure_occur;
  const bool nl = !fail && E.solver_flag == Estimator::NON_LINEAR;
  if (fail) {
    n_fail_++;
    std::lock_guard<std::mutex> l(om_);
    have_A_ = false;  // VINS restarted (new VINS world): realign when it is non-linear again
  }
  if (nl && !was_nl) n_init_++;
  if (nl) {
    n_nonlin_++;
    if (E.marginalization_flag == Estimator::MARGIN_OLD) n_kf_++;
    ms_total_.push_back(ms);
    ms_solve_.push_back(E.ovswba_solver_ms);
    ms_margin_.push_back(E.ovswba_margin_ms);
    iters_.push_back(E.ovswba_iters);
  }
  ms_cap_.push_back(f.cap_ms);
  std::ostringstream o;
  o << "{\"type\":\"frame\",\"t_ns\":" << f.t_ns << ",\"t\":";
  char b[64];
  std::snprintf(b, sizeof b, "%.6f", f.t);
  o << b << ",\"n_in\":" << f.id.size() << ",\"flag\":\"" << (fail ? "fail" : nl ? "nl" : "init") << "\"";
  std::snprintf(b, sizeof b, "%.3f", ms);
  o << ",\"ms\":" << b;
  std::snprintf(b, sizeof b, "%.3f", E.ovswba_solver_ms);
  o << ",\"solve_ms\":" << b;
  std::snprintf(b, sizeof b, "%.3f", E.ovswba_margin_ms);
  o << ",\"margin_ms\":" << b << ",\"it\":" << E.ovswba_iters << ",\"kf\":" << (E.marginalization_flag == Estimator::MARGIN_OLD ? 1 : 0)
    << ",\"n_feat\":" << E.f_manager.getFeatureCount() << ",\"ov\":" << (f.ov_valid ? 1 : 0);
  if (nl) {
    const Eigen::Matrix3d R_v = E.Rs[WINDOW_SIZE];
    const Eigen::Vector3d p_v = E.Ps[WINDOW_SIZE];
    o << ",\"p_vins\":";
    jv(o, p_v);
    o << ",\"q_vins\":";
    jq(o, R_v);
    o << ",\"v_vins\":";
    jv(o, E.Vs[WINDOW_SIZE]);
    o << ",\"ba\":";
    jv(o, E.Bas[WINDOW_SIZE]);
    o << ",\"bg\":";
    jv(o, E.Bgs[WINDOW_SIZE]);
    if (f.ov_valid) {
      std::lock_guard<std::mutex> l(om_);
      if (!have_A_) {  // A: VINS world -> OpenVINS world such that the target equals the published value now
        const double yp = out_.valid ? out_.yaw : 0.0;
        const Eigen::Vector3d tp = out_.valid ? out_.t : Eigen::Vector3d::Zero();
        const Eigen::Matrix3d R_m = Rz(yp) * f.R_ItoG;
        const Eigen::Vector3d p_m = Rz(yp) * f.p_IinG + tp;
        A_yaw_ = yaw_of(R_m * R_v.transpose());
        A_t_ = p_m - Rz(A_yaw_) * p_v;
        have_A_ = true;
        o << ",\"align\":1";
      }
      const Eigen::Matrix3d R_mv = Rz(A_yaw_) * R_v;
      const Eigen::Vector3d p_mv = Rz(A_yaw_) * p_v + A_t_;
      const double ty = yaw_of(R_mv * f.R_ItoG.transpose());
      const Eigen::Vector3d tt = p_mv - Rz(ty) * f.p_IinG;
      out_.tgt_yaw = ty;
      out_.tgt_t = tt;
      if (!out_.valid) {
        out_.valid = true;
        out_.yaw = ty;
        out_.t = tt;
      }
      out_.ver++;
      out_.n_kf = static_cast<int>(n_kf_);
      o << ",\"p_al\":";
      jv(o, p_mv);
      o << ",\"q_al\":";
      jq(o, R_mv);
      o << ",\"p_ov\":";
      jv(o, f.p_IinG);
      o << ",\"q_ov\":";
      jq(o, f.R_ItoG);
      std::snprintf(b, sizeof b, "%.7f", ty);
      o << ",\"tgt_yaw\":" << b << ",\"tgt_t\":";
      jv(o, tt);
    }
  }
  o << "}";
  log_line(o.str());
}

void Runner::log_line(const std::string &s) {
  if (!log_) return;
  std::fputs(s.c_str(), log_);
  std::fputc('\n', log_);
}

// = ovloop::LoopCloser::step_output (same limiter): the corrected position of the CURRENT body moves at most smooth_v
// m/s and the yaw at most smooth_w_deg deg/s towards the target, on top of the VIO motion.
ovloop::MapOdom Runner::step_output(double t, const Eigen::Vector3d &p) {
  std::lock_guard<std::mutex> l(om_);
  if (!out_have_t_) {
    out_have_t_ = true;
    out_last_t_ = t;
  }
  const double dt = std::min(0.5, std::max(0.0, t - out_last_t_));
  out_last_t_ = std::max(out_last_t_, t);
  if (!out_.valid) return out_;
  const double w = c_.smooth_w_deg * M_PI / 180.0 * dt, vmax = c_.smooth_v * dt;
  const double dyaw = wrap(out_.tgt_yaw - out_.yaw);
  const double yaw_new = wrap(out_.yaw + std::max(-w, std::min(w, dyaw)));
  const Eigen::Vector3d P_old = Rz(out_.yaw) * p + out_.t, P_tgt = Rz(out_.tgt_yaw) * p + out_.tgt_t;
  Eigen::Vector3d d = P_tgt - P_old;
  if (d.norm() > vmax) d *= vmax / d.norm();
  const Eigen::Vector3d P_new = P_old + d;
  out_.yaw = yaw_new;
  out_.t = P_new - Rz(yaw_new) * p;
  out_.converging = (P_tgt - P_new).norm() > 1e-4 || std::fabs(wrap(out_.tgt_yaw - yaw_new)) > 1e-5;
  if (!out_.converging) {
    out_.yaw = out_.tgt_yaw;
    out_.t = out_.tgt_t;
  }
  return out_;
}

ovloop::MapOdom Runner::published() const {
  std::lock_guard<std::mutex> l(om_);
  return out_;
}

void Runner::finish(double timeout_s) {
  if (c_.sync) {
    drain();
    return;
  }
  std::unique_lock<std::mutex> l(qm_);
  // frames still waiting for a later IMU sample can never complete at the end of input: wait for the others only
  done_cv_.wait_for(l, std::chrono::duration<double>(timeout_s), [&] {
    return fr_.empty() || (fr_.size() == 1 && !(imu_.size() && imu_.back().t > fr_.front()->t));
  });
}

void Runner::report() const {
  std::fprintf(stderr, "[ov_bridge] swba: frames pushed %llu processed %llu dropped(queue) %llu thrown(old) %llu | non-linear %llu "
               "(keyframes %llu) inits %llu failures %llu | asserts %ld\n",
               static_cast<unsigned long long>(n_pushed_), static_cast<unsigned long long>(n_frames_),
               static_cast<unsigned long long>(n_dropped_), static_cast<unsigned long long>(n_old_),
               static_cast<unsigned long long>(n_nonlin_), static_cast<unsigned long long>(n_kf_),
               static_cast<unsigned long long>(n_init_), static_cast<unsigned long long>(n_fail_), vins_assert_fail.load());
  std::fprintf(stderr, "[ov_bridge] swba: ms p50/p95/max per non-linear frame: total %.2f/%.2f/%.2f solve %.2f/%.2f/%.2f "
               "margin %.2f/%.2f/%.2f | VIO-thread copy %.3f/%.3f/%.3f ms\n",
               pct(ms_total_, 50), pct(ms_total_, 95), pct(ms_total_, 100), pct(ms_solve_, 50), pct(ms_solve_, 95),
               pct(ms_solve_, 100), pct(ms_margin_, 50), pct(ms_margin_, 95), pct(ms_margin_, 100), pct(ms_cap_, 50),
               pct(ms_cap_, 95), pct(ms_cap_, 100));
}

}  // namespace ovswba
