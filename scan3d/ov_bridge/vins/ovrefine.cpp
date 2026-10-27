// ovrefine.cpp (2026-10-27, ovrefine): see ovrefine.h. Factors = VINS-Mono (vins/factor, GPL-3.0), state = OpenVINS
// (GPL-3.0). Nothing here writes to the filter.
#include "ovrefine.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <sstream>

#include <ceres/ceres.h>

#include "factor/imu_factor.h"
#include "factor/integration_base.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "parameters.h"
#include "utility/utility.h"

namespace ovrefine {
namespace {

double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t i = std::min(v.size() - 1, static_cast<size_t>(p / 100.0 * (v.size() - 1) + 0.5));
  return v[i];
}

const char *kRej[8] = {"unusable", "cost_up", "cost_max", "few_feat", "dp", "dth", "nonfinite", "few_imu"};

// Prior on the first window state from the OpenVINS marginal covariance. Residual r = L [p - p0, 2 vec(q0^-1 q), v - v0,
// ba - ba0, bg - bg0] (VINS state order), L^T L = (scale * P)^-1. The OpenVINS IMU orientation error (JPL, R_GtoI =
// Exp(-th) R^_GtoI) is R_ItoG = R^_ItoG Exp(th) = the VINS right perturbation (PoseLocalParameterization: q * dq), so
// the covariance blocks are only reordered. Jacobians in the VINS manifold convention (7 columns, last one 0).
class PriorFactor : public ceres::SizedCostFunction<15, 7, 9> {
public:
  PriorFactor(const Snap &s, double scale) : p0_(s.p), q0_(s.R), v0_(s.v), ba0_(s.ba), bg0_(s.bg) {
    // OpenVINS order [th 0, p 3, v 6, bg 9, ba 12] -> VINS order [p 0, th 3, v 6, ba 9, bg 12]
    const int map_[5] = {3, 0, 6, 12, 9};  // VINS block k comes from OpenVINS block map_[k]
    Eigen::Matrix<double, 15, 15> C;
    for (int a = 0; a < 5; a++)
      for (int b = 0; b < 5; b++) C.block<3, 3>(3 * a, 3 * b) = s.P.block<3, 3>(map_[a], map_[b]);
    // 15:36 fix: the first version inverted C and took an LLT of the inverse; that failed numerically for every window
    // with prior_scale 4 (all R10p4 windows "unusable"; probable cause: an eigenvalue of C slightly below 0 that the
    // fixed +1e-12 no longer covered after scaling). Now: eigen-decomposition C = V D V^T, eigenvalues clamped at 0 and
    // + 1e-12 (= the old C + 1e-12 I whenever that was positive definite), L = D^-1/2 V^T -> L^T L = (C + 1e-12 I)^-1.
    C = 0.5 * (C + C.transpose()) * scale;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(C);
    ok_ = es.info() == Eigen::Success;
    Eigen::Matrix<double, 15, 1> d = es.eigenvalues();
    n_neg_ = 0;
    for (int i = 0; i < 15; i++) {
      if (d(i) < 0.0) n_neg_++;
      d(i) = 1.0 / std::sqrt(std::max(d(i), 0.0) + 1e-12);
    }
    L_ = d.asDiagonal() * es.eigenvectors().transpose();
    ok_ = ok_ && L_.allFinite();
  }
  bool ok() const { return ok_; }
  int n_neg() const { return n_neg_; }
  bool Evaluate(double const *const *x, double *res, double **J) const override {
    const Eigen::Vector3d p(x[0][0], x[0][1], x[0][2]);
    const Eigen::Quaterniond q(x[0][6], x[0][3], x[0][4], x[0][5]);
    const Eigen::Vector3d v(x[1][0], x[1][1], x[1][2]), ba(x[1][3], x[1][4], x[1][5]), bg(x[1][6], x[1][7], x[1][8]);
    Eigen::Matrix<double, 15, 1> r;
    const Eigen::Quaterniond dq = q0_.inverse() * q;
    r.segment<3>(0) = p - p0_;
    r.segment<3>(3) = 2.0 * dq.vec();
    r.segment<3>(6) = v - v0_;
    r.segment<3>(9) = ba - ba0_;
    r.segment<3>(12) = bg - bg0_;
    Eigen::Map<Eigen::Matrix<double, 15, 1>> rm(res);
    rm = L_ * r;
    if (J) {
      if (J[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> Jp(J[0]);
        Eigen::Matrix<double, 15, 7> A = Eigen::Matrix<double, 15, 7>::Zero();
        A.block<3, 3>(0, 0).setIdentity();
        A.block<3, 3>(3, 3) = Utility::Qleft(dq).bottomRightCorner<3, 3>();
        Jp = L_ * A;
      }
      if (J[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> Jb(J[1]);
        Eigen::Matrix<double, 15, 9> B = Eigen::Matrix<double, 15, 9>::Zero();
        B.block<9, 9>(6, 0).setIdentity();
        Jb = L_ * B;
      }
    }
    return true;
  }

private:
  Eigen::Vector3d p0_;
  Eigen::Quaterniond q0_;
  Eigen::Vector3d v0_, ba0_, bg0_;
  Eigen::Matrix<double, 15, 15> L_;
  bool ok_ = false;
  int n_neg_ = 0;
};

}  // namespace

bool Cfg::parse(const std::string &kv, std::string &err) {
  std::stringstream ss(kv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    const size_t e = item.find('=');
    if (e == std::string::npos) {
      err = "--refine-cfg: expected K=V, got '" + item + "'";
      return false;
    }
    const std::string k = item.substr(0, e);
    double v = 0.0;
    try {
      v = std::stod(item.substr(e + 1));
    } catch (...) {
      err = "--refine-cfg: bad number in '" + item + "'";
      return false;
    }
    if (k == "freq") freq = v;
    else if (k == "n") n = static_cast<int>(v);
    else if (k == "min_obs") min_obs = static_cast<int>(v);
    else if (k == "min_feat") min_feat = static_cast<int>(v);
    else if (k == "iters") iters = static_cast<int>(v);
    else if (k == "prior_scale") prior_scale = v;
    else if (k == "sigma_px") sigma_px = v;
    else if (k == "huber") huber = v;
    else if (k == "dp_max") dp_max = v;
    else if (k == "dth_max_deg") dth_max_deg = v;
    else if (k == "cost_max") cost_max = v;
    else if (k == "max_gap") max_gap = v;
    else if (k == "min_depth") min_depth = v;
    else if (k == "max_depth") max_depth = v;
    else if (k == "init_px") init_px = v;
    else if (k == "noise") noise = static_cast<int>(v);
    else {
      err = "--refine-cfg: unknown key '" + k + "'";
      return false;
    }
  }
  if (freq <= 0.0 || n < 2 || min_obs < 2 || iters < 1 || prior_scale <= 0.0 || (noise != 1 && noise != 2) || dp_max < 0.0 ||
      dth_max_deg < 0.0 || max_gap <= 0.0 || huber <= 0.0) {
    err = "--refine-cfg: out of range (freq > 0, n >= 2, min_obs >= 2, iters >= 1, prior_scale > 0, noise 1|2, dp_max >= 0, huber > 0)";
    return false;
  }
  return true;
}

Refiner::Refiner(const Cfg &c, double sig_a, double sig_ab, double sig_w, double sig_wb, double g)
    : c_(c), acc_n_(sig_a), gyr_n_(sig_w), acc_w_(sig_ab), gyr_w_(sig_wb), g_(g) {
  if (!c_.log.empty()) log_ = std::fopen(c_.log.c_str(), "w");
  std::fprintf(stderr, "[ov_bridge] refine: window %d snapshots @ %.1f Hz, min_obs %d, iters %d, prior_scale %.3g, sigma_px %.3g, "
               "noise %d, gates dp %.3f m dth %.2f deg cost %.1f feat %d\n",
               c_.n, c_.freq, c_.min_obs, c_.iters, c_.prior_scale, c_.sigma_px, c_.noise, c_.dp_max, c_.dth_max_deg, c_.cost_max,
               c_.min_feat);
}

Refiner::~Refiner() {
  if (log_) std::fclose(log_);
}

// VINS factors read globals (IntegrationBase noise at construction, IMU residual gravity, projection sqrt_info).
// --refine and --swba are exclusive (main.cpp), so nothing else uses them in this process.
void Refiner::set_globals() const {
  if (c_.noise == 2) {  // OpenVINS continuous densities -> per-sample standard deviation (VINS propagates noise * dt)
    const double s = std::sqrt(c_.imu_rate);
    ACC_N = acc_n_ * s;
    GYR_N = gyr_n_ * s;
    ACC_W = acc_w_ * s;
    GYR_W = gyr_w_ * s;
  } else {
    ACC_N = 0.08;
    GYR_N = 0.004;
    ACC_W = 0.00004;
    GYR_W = 2.0e-6;
  }
  G = Eigen::Vector3d(0.0, 0.0, g_);
}

void Refiner::push_imu(double t, const Eigen::Vector3d &a, const Eigen::Vector3d &w) {
  if (!imu_.empty() && t <= imu_.back().t) return;
  imu_.push_back({t, a, w});
  while (imu_.size() > 2 && imu_.front().t < t - 5.0) imu_.pop_front();
}

Refiner::Imu Refiner::imu_at(double t) const {
  if (imu_.empty()) return {t, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  if (t <= imu_.front().t) return {t, imu_.front().a, imu_.front().w};
  if (t >= imu_.back().t) return {t, imu_.back().a, imu_.back().w};
  auto it = std::lower_bound(imu_.begin(), imu_.end(), t, [](const Imu &m, double x) { return m.t < x; });
  const Imu &b = *it, &a = *(it - 1);
  const double u = (t - a.t) / std::max(1e-12, b.t - a.t);
  return {t, (1 - u) * a.a + u * b.a, (1 - u) * a.w + u * b.w};
}

std::shared_ptr<IntegrationBase> Refiner::integrate(double t0, double t1, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg) {
  const Imu a0 = imu_at(t0);
  auto pre = std::make_shared<IntegrationBase>(a0.a, a0.w, ba, bg);
  double tp = t0;
  for (const auto &m : imu_) {
    if (m.t <= t0 || m.t >= t1) continue;
    if (m.t - tp > 1e-9) pre->push_back(m.t - tp, m.a, m.w);
    tp = m.t;
  }
  const Imu a1 = imu_at(t1);
  if (t1 - tp > 1e-9) pre->push_back(t1 - tp, a1.a, a1.w);
  return pre;
}

void Refiner::reset(const char *why) {
  if (win_.empty() && !out_.valid) return;
  win_.clear();
  out_ = Out();
  n_reset_++;
  if (log_) std::fprintf(log_, "{\"type\":\"reset\",\"why\":\"%s\"}\n", why);
}

void Refiner::add(std::unique_ptr<Snap> s) {
  last_t_ = s->t;
  n_snap_++;
  if (n_snap_ == 1 && imu_.size() > 20) {  // IMU rate from the samples so far (noise 2), fixed for the run
    std::vector<double> d;
    for (size_t i = 1; i < imu_.size(); i++) d.push_back(imu_[i].t - imu_[i - 1].t);
    const double md = pct(d, 50);
    if (md > 1e-5) c_.imu_rate = 1.0 / md;
    std::fprintf(stderr, "[ov_bridge] refine: IMU rate %.1f Hz (median dt of %zu samples)\n", c_.imu_rate, d.size());
  }
  if (!win_.empty() && s->t - win_.back()->t > c_.max_gap) reset("gap");
  set_globals();
  if (!win_.empty()) s->pre = integrate(win_.back()->t, s->t, win_.back()->ba, win_.back()->bg);
  win_.push_back(std::move(s));
  while (static_cast<int>(win_.size()) > c_.n) win_.pop_front();
  if (static_cast<int>(win_.size()) == c_.n) solve();
}

void Refiner::solve() {
  const auto t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(win_.size());
  const Snap &L = *win_.back();
  n_win_++;
  std::vector<std::array<double, 7>> pose(n);
  std::vector<std::array<double, 9>> sb(n);
  for (int i = 0; i < n; i++) {
    const Snap &s = *win_[i];
    const Eigen::Quaterniond q(s.R);
    pose[i] = {s.p(0), s.p(1), s.p(2), q.x(), q.y(), q.z(), q.w()};
    sb[i] = {s.v(0), s.v(1), s.v(2), s.ba(0), s.ba(1), s.ba(2), s.bg(0), s.bg(1), s.bg(2)};
  }
  // camera extrinsics (fixed) = newest snapshot: VINS ric = R_CtoI, tic = p_CinI
  const Eigen::Matrix3d ric = L.R_ItoC.transpose();
  const Eigen::Vector3d tic = -ric * L.p_IinC;
  const Eigen::Quaterniond qic(ric);
  std::array<double, 7> ex = {tic(0), tic(1), tic(2), qic.x(), qic.y(), qic.z(), qic.w()};
  const double sig = c_.sigma_px > 0 ? c_.sigma_px : 1.0;
  ProjectionFactor::sqrt_info = L.fx / sig * Eigen::Matrix2d::Identity();

  // tracks seen in >= min_obs snapshots
  std::map<int, std::vector<std::pair<int, Eigen::Vector2d>>> tr;
  for (int i = 0; i < n; i++)
    for (size_t k = 0; k < win_[i]->id.size(); k++) tr[win_[i]->id[k]].push_back({i, Eigen::Vector2d(win_[i]->x[k], win_[i]->y[k])});
  // camera poses at the OpenVINS values (triangulation)
  std::vector<Eigen::Matrix3d> Rc(n);
  std::vector<Eigen::Vector3d> pc(n);
  for (int i = 0; i < n; i++) {
    Rc[i] = win_[i]->R * ric;
    pc[i] = win_[i]->p + win_[i]->R * tic;
  }
  struct F {
    int id;
    double inv;
    std::vector<std::pair<int, Eigen::Vector2d>> obs;
  };
  std::vector<F> feats;
  int n_tri_fail = 0;
  for (auto &kv : tr) {
    auto &ob = kv.second;
    if (static_cast<int>(ob.size()) < c_.min_obs) continue;
    const int a = ob[0].first;
    Eigen::MatrixXd A(2 * ob.size(), 4);
    std::vector<Eigen::Matrix3d> Rr(ob.size());
    std::vector<Eigen::Vector3d> tr_(ob.size());
    for (size_t k = 0; k < ob.size(); k++) {
      const int j = ob[k].first;
      Rr[k] = Rc[j].transpose() * Rc[a];             // anchor camera -> camera j
      tr_[k] = Rc[j].transpose() * (pc[a] - pc[j]);  // anchor origin in camera j
      Eigen::Matrix<double, 3, 4> P;
      P.leftCols<3>() = Rr[k];
      P.col(3) = tr_[k];
      A.row(2 * k) = ob[k].second(0) * P.row(2) - P.row(0);
      A.row(2 * k + 1) = ob[k].second(1) * P.row(2) - P.row(1);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    const Eigen::Vector4d h = svd.matrixV().col(3);
    if (std::fabs(h(3)) < 1e-12) {
      n_tri_fail++;
      continue;
    }
    const Eigen::Vector3d X = h.head<3>() / h(3);
    bool good = X(2) > c_.min_depth && X(2) < c_.max_depth;
    for (size_t k = 0; good && k < ob.size(); k++) {
      const Eigen::Vector3d Xj = Rr[k] * X + tr_[k];
      if (Xj(2) < c_.min_depth) {
        good = false;
        break;
      }
      const double e = L.fx * (Xj.head<2>() / Xj(2) - ob[k].second).norm();
      if (e > c_.init_px) good = false;
    }
    if (!good) {
      n_tri_fail++;
      continue;
    }
    feats.push_back({kv.first, 1.0 / X(2), ob});
  }

  int reason = -1;
  ceres::Solver::Summary sum;
  double ms_solve = 0.0;
  Eigen::Vector3d p_ref = L.p;
  Eigen::Matrix3d R_ref = L.R;
  int n_res_img = 0;
  std::vector<ceres::ResidualBlockId> grp[3];  // prior, IMU, image (diagnostic costs with --refine-log)
  double gc0[3] = {0, 0, 0}, gc1[3] = {0, 0, 0}, imu_r[3] = {0, 0, 0};
  int prior_neg = -1;  // negative eigenvalues of the OpenVINS prior covariance (clamped)
  if (static_cast<int>(feats.size()) < c_.min_feat) reason = 3;
  for (int i = 1; i < n && reason < 0; i++)
    if (!win_[i]->pre || win_[i]->pre->sum_dt <= 0.0) reason = 7;
  if (reason < 0) {
    ceres::Problem::Options po;
    ceres::Problem prob(po);
    for (int i = 0; i < n; i++) {
      prob.AddParameterBlock(pose[i].data(), 7, new PoseLocalParameterization());
      prob.AddParameterBlock(sb[i].data(), 9);
    }
    prob.AddParameterBlock(ex.data(), 7, new PoseLocalParameterization());
    prob.SetParameterBlockConstant(ex.data());
    auto *pf = new PriorFactor(*win_[0], c_.prior_scale);
    prior_neg = pf->n_neg();
    if (!pf->ok()) {
      delete pf;
      reason = 0;
    } else {
      grp[0].push_back(prob.AddResidualBlock(pf, nullptr, pose[0].data(), sb[0].data()));
      for (int i = 1; i < n; i++) {
        grp[1].push_back(prob.AddResidualBlock(new IMUFactor(win_[i]->pre.get()), nullptr, pose[i - 1].data(), sb[i - 1].data(),
                                               pose[i].data(), sb[i].data()));
        // unweighted IMU residual at the OpenVINS values (diagnostic: p m, q rad, v m/s)
        const Snap &A = *win_[i - 1], &B = *win_[i];
        const Eigen::Matrix<double, 15, 1> r0 = B.pre->evaluate(A.p, Eigen::Quaterniond(A.R), A.v, A.ba, A.bg, B.p,
                                                                Eigen::Quaterniond(B.R), B.v, B.ba, B.bg);
        imu_r[0] = std::max(imu_r[0], r0.segment<3>(O_P).norm());
        imu_r[1] = std::max(imu_r[1], r0.segment<3>(O_R).norm());
        imu_r[2] = std::max(imu_r[2], r0.segment<3>(O_V).norm());
      }
      ceres::LossFunction *loss = new ceres::HuberLoss(c_.huber);
      for (auto &f : feats) {
        const int a = f.obs[0].first;
        const Eigen::Vector3d pi(f.obs[0].second(0), f.obs[0].second(1), 1.0);
        for (size_t k = 1; k < f.obs.size(); k++) {
          const Eigen::Vector3d pj(f.obs[k].second(0), f.obs[k].second(1), 1.0);
          grp[2].push_back(prob.AddResidualBlock(new ProjectionFactor(pi, pj), loss, pose[a].data(), pose[f.obs[k].first].data(),
                                                 ex.data(), &f.inv));
          n_res_img++;
        }
      }
      ceres::Solver::Options so;
      so.linear_solver_type = ceres::DENSE_SCHUR;
      so.trust_region_strategy_type = ceres::DOGLEG;
      so.max_num_iterations = c_.iters;
      so.num_threads = 1;
      so.minimizer_progress_to_stdout = false;
      so.logging_type = ceres::SILENT;
      auto gcost = [&](double *out) {
        if (!log_) return;
        for (int g = 0; g < 3; g++) {
          ceres::Problem::EvaluateOptions eo;
          eo.residual_blocks = grp[g];
          eo.apply_loss_function = true;
          double c = 0.0;
          prob.Evaluate(eo, &c, nullptr, nullptr, nullptr);
          out[g] = c;
        }
      };
      gcost(gc0);
      const auto ts = std::chrono::steady_clock::now();
      ceres::Solve(so, &prob, &sum);
      ms_solve = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ts).count();
      gcost(gc1);
      p_ref = Eigen::Vector3d(pose[n - 1][0], pose[n - 1][1], pose[n - 1][2]);
      R_ref = Eigen::Quaterniond(pose[n - 1][6], pose[n - 1][3], pose[n - 1][4], pose[n - 1][5]).normalized().toRotationMatrix();
      const int nres = std::max(1, sum.num_residuals);
      if (!sum.IsSolutionUsable()) reason = 0;
      else if (!p_ref.allFinite() || !R_ref.allFinite()) reason = 6;
      else if (sum.final_cost > sum.initial_cost) reason = 1;
      else if (2.0 * sum.final_cost / nres > c_.cost_max) reason = 2;
    }
  }
  const double dp = (p_ref - L.p).norm();
  const double dth = std::acos(std::max(-1.0, std::min(1.0, ((R_ref * L.R.transpose()).trace() - 1.0) / 2.0))) * 180.0 / M_PI;
  if (reason < 0 && dp > c_.dp_max) reason = 4;
  if (reason < 0 && dth > c_.dth_max_deg) reason = 5;
  if (reason < 0) {
    n_ok_++;
    out_.valid = true;
    out_.dR = R_ref * L.R.transpose();
    out_.dt = p_ref - out_.dR * L.p;
    dp_ok_.push_back(dp);
  } else {
    n_rej_++;
    rej_[reason]++;
    out_ = Out();
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  ms_.push_back(ms);
  ms_solve_.push_back(ms_solve);
  nfeat_.push_back(static_cast<int>(feats.size()));
  iters_.push_back(static_cast<int>(sum.iterations.size()));
  if (log_) {
    std::fprintf(log_,
                 "{\"type\":\"win\",\"t_ns\":%lld,\"t\":%.6f,\"n\":%d,\"feat\":%zu,\"tri_fail\":%d,\"res_img\":%d,\"cost0\":%.6g,\"cost1\":%.6g,"
                 "\"it\":%zu,\"term\":%d,\"dp_mm\":%.4f,\"dth_deg\":%.5f,\"ok\":%d,\"why\":\"%s\",\"ms\":%.3f,\"solve_ms\":%.3f,"
                 "\"p_ref\":[%.6f,%.6f,%.6f],\"p_ov\":[%.6f,%.6f,%.6f],\"n_ok\":%llu,\"n_rej\":%llu,"
                 "\"gc0\":[%.4g,%.4g,%.4g],\"gc1\":[%.4g,%.4g,%.4g],\"imu_r0\":[%.4g,%.4g,%.4g],\"prior_neg\":%d}\n",
                 static_cast<long long>(L.t_ns), L.t, n, feats.size(), n_tri_fail, n_res_img, sum.initial_cost, sum.final_cost,
                 sum.iterations.size(), static_cast<int>(sum.termination_type), dp * 1000.0, dth, reason < 0 ? 1 : 0,
                 reason < 0 ? "" : kRej[reason], ms, ms_solve, p_ref(0), p_ref(1), p_ref(2), L.p(0), L.p(1), L.p(2),
                 static_cast<unsigned long long>(n_ok_), static_cast<unsigned long long>(n_rej_), gc0[0], gc0[1], gc0[2], gc1[0],
                 gc1[1], gc1[2], imu_r[0], imu_r[1], imu_r[2], prior_neg);
  }
}

void Refiner::report() const {
  std::vector<double> nf(nfeat_.begin(), nfeat_.end()), it(iters_.begin(), iters_.end());
  std::fprintf(stderr, "[ov_bridge] refine: snapshots %llu windows %llu accepted %llu rejected %llu (", static_cast<unsigned long long>(n_snap_),
               static_cast<unsigned long long>(n_win_), static_cast<unsigned long long>(n_ok_), static_cast<unsigned long long>(n_rej_));
  for (int i = 0; i < 8; i++) std::fprintf(stderr, "%s %llu%s", kRej[i], static_cast<unsigned long long>(rej_[i]), i < 7 ? ", " : "");
  std::fprintf(stderr, ") resets %llu | ms p50/p95/max %.2f/%.2f/%.2f solve %.2f/%.2f | feat p50 %.0f it p50 %.0f | accepted dp mm p50/p95 %.2f/%.2f\n",
               static_cast<unsigned long long>(n_reset_), pct(ms_, 50), pct(ms_, 95), pct(ms_, 100), pct(ms_solve_, 50),
               pct(ms_solve_, 95), pct(nf, 50), pct(it, 50), pct(dp_ok_, 50) * 1000, pct(dp_ok_, 95) * 1000);
}

}  // namespace ovrefine
