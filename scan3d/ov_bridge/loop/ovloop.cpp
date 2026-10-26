// ovloop.cpp (2026-10-26, branch ovloop-20261026): see ovloop.h
#include "ovloop.h"

#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>

#include "lc_features.h"

namespace ovloop {

namespace {
using Clock = std::chrono::steady_clock;
inline double ms_since(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }
inline double yaw_of(const Eigen::Matrix3d &R) { return std::atan2(R(1, 0), R(0, 0)); }  // R = world <- body (ZYX)
inline Eigen::Matrix3d Rz(double a) {
  Eigen::Matrix3d R;
  R << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1;
  return R;
}
inline double wrap(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return S;
}
double pct(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<size_t>(p / 100.0 * (v.size() - 1) + 0.5))];
}
void jv(std::ostringstream &o, const Eigen::Vector3d &v) {
  o << "[" << v(0) << "," << v(1) << "," << v(2) << "]";
}

// 4-DoF relative pose residual (VINS-Mono FourDOFError form): the translation of j seen from i in i's yaw-only frame
// (roll / pitch removed using the VIO gravity, which is observable) and the yaw difference.
struct FourDof {
  FourDof(const Eigen::Vector3d &dt, double dyaw, double w) : dt_(dt), dyaw_(dyaw), w_(w), wy_(w) {}
  FourDof(const Eigen::Vector3d &dt, double dyaw, double wt, double wy) : dt_(dt), dyaw_(dyaw), w_(wt), wy_(wy) {}
  template <typename T> bool operator()(const T *yi, const T *ti, const T *yj, const T *tj, T *r) const {
    const T dx = tj[0] - ti[0], dy = tj[1] - ti[1], dz = tj[2] - ti[2];
    const T c = cos(yi[0]), s = sin(yi[0]);
    r[0] = T(w_) * (c * dx + s * dy - T(dt_(0)));
    r[1] = T(w_) * (-s * dx + c * dy - T(dt_(1)));
    r[2] = T(w_) * (dz - T(dt_(2)));
    T a = yj[0] - yi[0] - T(dyaw_);
    if (a > T(M_PI)) a -= T(2.0 * M_PI);
    else if (a < T(-M_PI)) a += T(2.0 * M_PI);
    if (a > T(M_PI)) a -= T(2.0 * M_PI);
    else if (a < T(-M_PI)) a += T(2.0 * M_PI);
    r[3] = T(wy_) * a;
    return true;
  }
  Eigen::Vector3d dt_;
  double dyaw_, w_, wy_;
};
}  // namespace

// ---------------------------------------------------------------- config
bool Cfg::parse(const std::string &kv, std::string &err) {
  std::stringstream ss(kv);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty()) continue;
    auto eq = tok.find('=');
    if (eq == std::string::npos) {
      err = "bad --loop-cfg item " + tok;
      return false;
    }
    const std::string k = tok.substr(0, eq);
    double v = 0.0;
    try {
      v = std::stod(tok.substr(eq + 1));
    } catch (...) {
      err = "bad --loop-cfg value " + tok;
      return false;
    }
    struct D { const char *n; double *d; int *i; };
    const D tab[] = {{"kf_min_dt", &kf_min_dt, nullptr}, {"kf_trans", &kf_trans, nullptr}, {"kf_rot_deg", &kf_rot_deg, nullptr},
                     {"kf_max_dt", &kf_max_dt, nullptr}, {"queue_max", nullptr, &queue_max}, {"nice", nullptr, &nice},
                     {"n_feat", nullptr, &n_feat}, {"levels", nullptr, &levels}, {"fast_th", nullptr, &fast_th},
                     {"min_gap_s", &min_gap_s, nullptr}, {"min_path_m", &min_path_m, nullptr}, {"min_score", &min_score, nullptr}, {"rel_score", &rel_score, nullptr},
                     {"max_cand", nullptr, &max_cand}, {"vpr_min", &vpr_min, nullptr}, {"diag_prev", nullptr, &diag_prev}, {"match_dist", nullptr, &match_dist}, {"ratio", &ratio, nullptr},
                     {"min_inliers", nullptr, &min_inliers}, {"pnp_px", &pnp_px, nullptr}, {"pnp_iters", nullptr, &pnp_iters},
                     {"max_rp_deg", &max_rp_deg, nullptr}, {"max_corr_m", &max_corr_m, nullptr},
                     {"max_corr_rel", &max_corr_rel, nullptr}, {"max_corr_yaw_deg", &max_corr_yaw_deg, nullptr},
                     {"consist_kf", nullptr, &consist_kf}, {"consist_old_s", &consist_old_s, nullptr},
                     {"consist_m", &consist_m, nullptr}, {"consist_yaw_deg", &consist_yaw_deg, nullptr},
                     {"seq_edges", nullptr, &seq_edges}, {"loop_w", &loop_w, nullptr}, {"huber", &huber, nullptr}, {"weight_mode", nullptr, &weight_mode}, {"seq_sig_m", &seq_sig_m, nullptr}, {"seq_sig_rel", &seq_sig_rel, nullptr}, {"seq_sig_deg", &seq_sig_deg, nullptr}, {"seq_sig_deg_m", &seq_sig_deg_m, nullptr}, {"loop_sig_m", &loop_sig_m, nullptr}, {"loop_sig_deg", &loop_sig_deg, nullptr}, {"huber_sig", &huber_sig, nullptr},
                     {"opt_iters", nullptr, &opt_iters}, {"smooth_v", &smooth_v, nullptr},
                     {"smooth_w_deg", &smooth_w_deg, nullptr}};
    bool found = false;
    for (const auto &d : tab)
      if (k == d.n) {
        if (d.d) *d.d = v;
        else *d.i = static_cast<int>(std::lround(v));
        found = true;
      }
    if (!found) {
      err = "unknown --loop-cfg key " + k;
      return false;
    }
  }
  if (smooth_v <= 0 || smooth_w_deg <= 0 || min_inliers < 6 || levels < 1 || n_feat < 50 || queue_max < 1) {
    err = "--loop-cfg value out of range";
    return false;
  }
  return true;
}

std::string Cfg::describe() const {
  std::ostringstream o;
  o << "kf " << kf_min_dt << "s/" << kf_trans << "m/" << kf_rot_deg << "deg max_dt " << kf_max_dt << " q " << queue_max
    << " nice " << nice << " | feat " << n_feat << " L" << levels << " fast " << fast_th << " | gap " << min_gap_s << "s/" << min_path_m << "m"
    << " score " << min_score << "/" << rel_score << " cand " << max_cand << " | match " << match_dist << "/" << ratio
    << " inl " << min_inliers << " pnp " << pnp_px << "px x" << pnp_iters << " rp " << max_rp_deg << "deg corr "
    << max_corr_m << "m+" << max_corr_rel << "*path yaw " << max_corr_yaw_deg << "deg | consist " << consist_kf << "kf "
    << consist_old_s << "s " << consist_m << "m " << consist_yaw_deg << "deg | graph seq " << seq_edges << " w "
    << loop_w << " huber " << huber << " it " << opt_iters << " | smooth " << smooth_v << "m/s " << smooth_w_deg
    << "deg/s | sync " << sync;
  return o.str();
}

// ---------------------------------------------------------------- keyframe rule (VIO thread)
bool KfRule::want(double t, const Eigen::Matrix3d &R_ItoG, const Eigen::Vector3d &p) {
  if (!have_) {
    have_ = true;
    t_ = t;
    R_ = R_ItoG;
    p_ = p;
    return true;
  }
  const double dt = t - t_;
  if (dt < c_.kf_min_dt) return false;
  const double dp = (p - p_).norm();
  const double dr = Eigen::AngleAxisd(R_.transpose() * R_ItoG).angle() * 180.0 / M_PI;
  const bool want = dp >= c_.kf_trans || dr >= c_.kf_rot_deg || (dt >= c_.kf_max_dt && dp >= 0.25 * c_.kf_trans);
  if (want) {
    t_ = t;
    R_ = R_ItoG;
    p_ = p;
  }
  return want;
}

// ---------------------------------------------------------------- loop-thread state
struct LoopCloser::Kf {
  int idx = 0;
  double t = 0.0;
  int64_t t_ns = 0;
  Eigen::Matrix3d R_v = Eigen::Matrix3d::Identity();  // VIO R_ItoG
  Eigen::Vector3d p_v = Eigen::Vector3d::Zero();
  double yaw_v = 0.0;
  double path = 0.0;  // cumulative VIO path (m) at this keyframe
  BowVec bow;
  std::vector<float> g;              // --loop-vpr global descriptor (empty = none)
  std::vector<Eigen::Vector3d> P3;  // 3D points in this keyframe's camera frame
  cv::Mat D3;                       // their descriptors
  cv::Mat DM;                       // matching set (detected + tracked points) and their undistorted pixels
  std::vector<cv::Point2f> ptsM;
  // pose graph (corrected)
  double yaw_c = 0.0;
  double t_c[3] = {0, 0, 0};
};

struct LoopCloser::Edge {
  int i = 0, j = 0;  // i older
  Eigen::Vector3d dt = Eigen::Vector3d::Zero();
  double dyaw = 0.0;
  bool loop = false;
};

struct LoopCloser::Verified {
  int old_idx = -1;
  int n_match = 0, n_inl = 0;
  double rms_px = 0.0;
  Eigen::Matrix3d R_oc = Eigen::Matrix3d::Identity();  // T_oldI_curI
  Eigen::Vector3d t_oc = Eigen::Vector3d::Zero();
  Eigen::Vector3d drift = Eigen::Vector3d::Zero();  // p_loop - p_vio (current keyframe, VIO world)
  double dyaw = 0.0;                                // yaw_loop - yaw_vio
  double rp_deg = 0.0;
  char dir = '-';  // PnP direction: A old 3D -> current 2D, B current 3D -> old 2D
};

struct LoopCloser::State {
  int session = 0;
  std::vector<Kf> kfs;
  std::vector<Edge> edges;
  BowDb db;
  Brief brief;
  struct Recent {
    int cur, old;
    Verified v;
    bool accepted;
  };
  std::deque<Recent> recent;  // verified loops of the last keyframes
  double corr_yaw = 0.0;      // current target correction (applied to new keyframes before the next optimisation)
  Eigen::Vector3d corr_t = Eigen::Vector3d::Zero();
  int n_loop = 0;
  uint32_t n_opt = 0;
  // stats (whole run)
  uint64_t n_kf = 0, n_cand = 0, n_geo = 0, n_verified = 0, n_accepted = 0, n_rej_consist = 0;
  std::vector<double> ms_total, ms_tri, ms_feat, ms_query, ms_geo, ms_opt, ms_cap;
};

LoopCloser::LoopCloser(const Cfg &c) : c_(c), rule_(c_), s_(new State) {
  if (c_.vocab.empty()) {
    ok_ = false;
    err_ = "--loop 1 needs --loop-vocab FILE";
    return;
  }
  if (!voc_.load(c_.vocab, err_)) {
    ok_ = false;
    return;
  }
  if (!c_.log.empty() && !(log_ = std::fopen(c_.log.c_str(), "w"))) {
    ok_ = false;
    err_ = "cannot open --loop-log " + c_.log;
    return;
  }
  if (!c_.vpr.empty()) {  // OVPR file: "OVPR" u32 dim, then records (i64 t_ns, f32[dim])
    FILE *f = std::fopen(c_.vpr.c_str(), "rb");
    char mg[4];
    uint32_t dim = 0;
    if (!f || std::fread(mg, 1, 4, f) != 4 || std::memcmp(mg, "OVPR", 4) != 0 || std::fread(&dim, 4, 1, f) != 1 || dim == 0 ||
        dim > 65536) {
      if (f) std::fclose(f);
      ok_ = false;
      err_ = "bad --loop-vpr " + c_.vpr;
      return;
    }
    vpr_dim_ = static_cast<int>(dim);
    int64_t t;
    std::vector<float> v(dim);
    while (std::fread(&t, 8, 1, f) == 1 && std::fread(v.data(), 4, dim, f) == dim) {
      double n = 0;
      for (float x : v) n += static_cast<double>(x) * x;
      n = std::sqrt(std::max(n, 1e-30));
      for (float &x : v) x = static_cast<float>(x / n);
      vpr_[t] = v;
    }
    std::fclose(f);
    std::fprintf(stderr, "[ov_bridge] loop: VPR descriptors %s: %zu keyframes, dim %d, min cos %.3f (candidates from VPR, not BoW)\n",
                 c_.vpr.c_str(), vpr_.size(), vpr_dim_, c_.vpr_min);
  }
  if (!c_.dump.empty()) std::fprintf(stderr, "[ov_bridge] loop: dumping keyframes to %s\n", c_.dump.c_str());
  std::fprintf(stderr, "[ov_bridge] loop: vocab %s (k %d L %d, %d words, sha %s) | %s\n", c_.vocab.c_str(), voc_.k, voc_.L,
               voc_.n_words(), voc_.sha().c_str(), c_.describe().c_str());
  th_ = std::thread([this] { run(); });
}

LoopCloser::~LoopCloser() {
  {
    std::lock_guard<std::mutex> l(qm_);
    stop_ = true;
  }
  qcv_.notify_all();
  if (th_.joinable()) th_.join();
  if (log_) std::fclose(log_);
}

void LoopCloser::push(std::unique_ptr<KfIn> kf) {
  std::unique_lock<std::mutex> l(qm_);
  kf->session = session_;
  if (static_cast<int>(q_.size()) >= c_.queue_max) {
    n_dropped_++;
    return;
  }
  q_.push_back(std::move(kf));
  const uint64_t my = ++n_pushed_;
  qcv_.notify_one();
  if (c_.sync) done_cv_.wait(l, [&] { return n_done_ >= my || stop_; });
}

void LoopCloser::reset_session(const char *why) {
  {
    std::lock_guard<std::mutex> l(qm_);
    session_++;
  }
  rule_.reset();
  std::lock_guard<std::mutex> l(om_);
  const uint32_t ver = out_.ver;
  out_ = MapOdom();
  out_.session = session_;
  out_.ver = ver;
  out_have_t_ = false;
  std::fprintf(stderr, "[ov_bridge] loop: session %d (%s): new graph, T_map_odom = identity\n", session_, why);
}

void LoopCloser::run() {
#ifdef SYS_gettid
  if (c_.nice != 0) setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), c_.nice);
#endif
  for (;;) {
    std::unique_ptr<KfIn> kf;
    {
      std::unique_lock<std::mutex> l(qm_);
      qcv_.wait(l, [&] { return stop_ || !q_.empty(); });
      if (stop_ && q_.empty()) return;
      kf = std::move(q_.front());
      q_.pop_front();
    }
    try {
      process(*kf);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] loop: exception %s (keyframe t=%.3f skipped)\n", e.what(), kf->t);
    }
    {
      std::lock_guard<std::mutex> l(qm_);
      n_done_++;
    }
    done_cv_.notify_all();
  }
}

void LoopCloser::log_line(const std::string &s) {
  if (!log_) return;
  std::lock_guard<std::mutex> l(logm_);
  std::fputs(s.c_str(), log_);
  std::fflush(log_);
}

// ---------------------------------------------------------------- one keyframe (loop thread)
void LoopCloser::process(KfIn &in) {
  State &S = *s_;
  if (in.session != S.session) {  // VIO re-initialised: new world frame -> start over
    S.session = in.session;
    S.kfs.clear();
    S.edges.clear();
    S.db.clear();
    S.recent.clear();
    S.corr_yaw = 0.0;
    S.corr_t.setZero();
    S.n_loop = 0;
  }
  const auto T0 = Clock::now();
  S.n_kf++;
  if (!c_.dump.empty()) {  // --loop-dump: binary PGM + index row (offline VPR evaluation)
    char fn[64];
    std::snprintf(fn, sizeof(fn), "/kf_%lld.pgm", static_cast<long long>(in.t_ns));
    if (FILE *f = std::fopen((c_.dump + fn).c_str(), "wb")) {
      std::fprintf(f, "P5\n%d %d\n255\n", in.img.cols, in.img.rows);
      for (int r = 0; r < in.img.rows; r++) std::fwrite(in.img.ptr<uint8_t>(r), 1, in.img.cols, f);
      std::fclose(f);
    }
    if (FILE *f = std::fopen((c_.dump + "/index.csv").c_str(), "a")) {
      std::fprintf(f, "%lld,%d,%.6f\n", static_cast<long long>(in.t_ns), in.session, in.t);
      std::fclose(f);
    }
  }
  S.ms_cap.push_back(in.cap_ms);
  // camera of this keyframe (VIO world)
  const Eigen::Matrix3d R_GtoC_kf = in.R_ItoC * in.R_ItoG.transpose();
  const Eigen::Vector3d p_CinG_kf = in.p_IinG - R_GtoC_kf.transpose() * in.p_IinC;
  const bool has_dist = std::any_of(in.dist.begin(), in.dist.end(), [](double d) { return d != 0.0; });
  cv::Mat K = (cv::Mat_<double>(3, 3) << in.fx, 0, in.cx, 0, in.fy, in.cy, 0, 0, 1);
  cv::Mat D = has_dist ? cv::Mat(in.dist, true).reshape(1, 1) : cv::Mat();

  // 1. 3D points of the tracked features (camera frame of this keyframe)
  auto t1 = Clock::now();
  std::map<double, int> ci;
  for (size_t i = 0; i < in.clone_t.size(); i++) ci[in.clone_t[i]] = static_cast<int>(i);
  std::vector<Eigen::Vector3d> P3;
  std::vector<KeyPt> kp3;
  for (const auto &tr : in.tracks) {
    Eigen::Vector3d pG;
    if (tr.slam) {
      pG = tr.p_G;
    } else {
      if (tr.obs_t.size() < 3) continue;
      Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      std::vector<Eigen::Matrix3d> Rs;
      std::vector<Eigen::Vector3d> ps;
      std::vector<Eigen::Vector2d> zs;
      for (size_t k = 0; k < tr.obs_t.size(); k++) {
        auto it = ci.find(tr.obs_t[k]);
        if (it == ci.end()) continue;
        const auto &x = in.clone_x[it->second];
        Eigen::Matrix3d R_GtoI;
        R_GtoI << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8];
        const Eigen::Vector3d p_IinG(x[9], x[10], x[11]);
        const Eigen::Matrix3d R_GtoC = in.R_ItoC * R_GtoI;
        const Eigen::Vector3d p_CinG = p_IinG - R_GtoC.transpose() * in.p_IinC;
        Eigen::Vector3d bi(tr.obs_x[k], tr.obs_y[k], 1.0);
        bi = (R_GtoC.transpose() * bi).normalized();
        const Eigen::Matrix3d Bp = skew(bi);
        const Eigen::Matrix3d Ai = Bp.transpose() * Bp;
        A += Ai;
        b += Ai * p_CinG;
        Rs.push_back(R_GtoC);
        ps.push_back(p_CinG);
        zs.emplace_back(tr.obs_x[k], tr.obs_y[k]);
      }
      if (Rs.size() < 3) continue;
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
      const Eigen::Vector3d sv = svd.singularValues();
      if (!(sv(2) > 1e-12) || sv(0) / sv(2) > 3000.0) continue;  // weak geometry (small parallax)
      pG = A.ldlt().solve(b);
      if (!pG.allFinite()) continue;
      double e2 = 0.0;
      bool bad = false;
      for (size_t k = 0; k < Rs.size(); k++) {
        const Eigen::Vector3d pc = Rs[k] * (pG - ps[k]);
        if (pc(2) < 0.1) {
          bad = true;
          break;
        }
        const double ex = (pc(0) / pc(2) - zs[k](0)) * in.fx, ey = (pc(1) / pc(2) - zs[k](1)) * in.fy;
        e2 += ex * ex + ey * ey;
        if (ex * ex + ey * ey > 9.0) bad = true;
      }
      if (bad || e2 / Rs.size() > 4.0) continue;
    }
    const Eigen::Vector3d pc = R_GtoC_kf * (pG - p_CinG_kf);
    if (!pc.allFinite() || pc(2) < 0.15 || pc(2) > 15.0) continue;
    KeyPt k;
    k.x = k.lx = tr.u;
    k.y = k.ly = tr.v;
    k.level = 0;
    P3.push_back(pc);
    kp3.push_back(k);
  }
  const double ms_tri = ms_since(t1);

  // 2. features: pyramid, FAST, gravity-steered box-BRIEF
  t1 = Clock::now();
  Pyramid P;
  P.build(in.img, c_.levels);
  std::vector<KeyPt> kp2;
  S.brief.detect(P, c_.n_feat, c_.fast_th, kp2);
  const Eigen::Vector3d gC = R_GtoC_kf * Eigen::Vector3d(0, 0, -1);  // gravity (down) in the camera
  auto steer = [&](KeyPt &k) {
    const double x = (k.x - in.cx) / in.fx, y = (k.y - in.cy) / in.fy;
    const double dx = gC(0) - x * gC(2), dy = gC(1) - y * gC(2);
    k.angle = (dx * dx + dy * dy > 0.04) ? static_cast<float>(std::atan2(in.fy * dy, in.fx * dx)) : 0.0f;
  };
  for (auto &k : kp2) steer(k);
  for (auto &k : kp3) steer(k);
  cv::Mat D2, D3;
  std::vector<uint8_t> v2, v3;
  S.brief.describe(P, kp2, D2, v2);
  S.brief.describe(P, kp3, D3, v3);
  {  // keep valid rows only
    cv::Mat d;
    std::vector<KeyPt> k;
    for (size_t i = 0; i < kp2.size(); i++)
      if (v2[i]) {
        d.push_back(D2.row(static_cast<int>(i)));
        k.push_back(kp2[i]);
      }
    D2 = d;
    kp2.swap(k);
    cv::Mat d3;
    std::vector<Eigen::Vector3d> p3;
    for (size_t i = 0; i < kp3.size(); i++)
      if (v3[i]) {
        d3.push_back(D3.row(static_cast<int>(i)));
        p3.push_back(P3[i]);
      }
    D3 = d3;
    P3.swap(p3);
  }
  Kf kf;
  kf.idx = static_cast<int>(S.kfs.size());
  kf.t = in.t;
  kf.t_ns = in.t_ns;
  kf.R_v = in.R_ItoG;
  kf.p_v = in.p_IinG;
  kf.yaw_v = yaw_of(in.R_ItoG);
  kf.path = S.kfs.empty() ? 0.0 : S.kfs.back().path + (in.p_IinG - S.kfs.back().p_v).norm();
  voc_.transform(D2, kf.bow);
  // matching set of this keyframe (query side) = detected keypoints + ALL tracked points (the old keyframe's 3D points
  // sit on OpenVINS FAST track corners; the detector here does not necessarily fire at the same pixels)
  cv::Mat DM = D2.clone();
  std::vector<cv::Point2f> ptsM;
  for (const auto &k : kp2) ptsM.emplace_back(k.x, k.y);
  {
    std::vector<KeyPt> kt;
    for (const auto &tr : in.tracks) {
      KeyPt k;
      k.x = k.lx = tr.u;
      k.y = k.ly = tr.v;
      steer(k);
      kt.push_back(k);
    }
    cv::Mat DT;
    std::vector<uint8_t> vt;
    S.brief.describe(P, kt, DT, vt);
    for (size_t i = 0; i < kt.size(); i++)
      if (vt[i]) {
        DM.push_back(DT.row(static_cast<int>(i)));
        ptsM.emplace_back(kt[i].x, kt[i].y);
      }
  }
  kf.P3 = P3;
  kf.D3 = D3;
  {  // corrected pose before the next optimisation = current correction
    const Eigen::Vector3d tc = Rz(S.corr_yaw) * kf.p_v + S.corr_t;
    kf.yaw_c = wrap(kf.yaw_v + S.corr_yaw);
    kf.t_c[0] = tc(0);
    kf.t_c[1] = tc(1);
    kf.t_c[2] = tc(2);
  }
  const double ms_feat = ms_since(t1);

  // 3. candidates (older than min_gap_s)
  t1 = Clock::now();
  int max_entry = 0;
  while (max_entry < static_cast<int>(S.kfs.size()) && S.kfs[max_entry].t <= in.t - c_.min_gap_s) max_entry++;
  std::vector<std::pair<int, float>> sc;
  const float s_prev = S.kfs.empty() ? 0.0f : bow_score(kf.bow, S.kfs.back().bow);
  if (!vpr_.empty()) {  // --loop-vpr: cosine similarity of the global descriptors
    auto it = vpr_.find(in.t_ns);
    if (it != vpr_.end()) kf.g = it->second;
    if (!kf.g.empty())
      for (int e = 0; e < max_entry; e++) {
        const auto &g = S.kfs[e].g;
        if (g.size() != kf.g.size()) continue;
        double d = 0;
        for (size_t q = 0; q < g.size(); q++) d += static_cast<double>(g[q]) * kf.g[q];
        sc.emplace_back(e, static_cast<float>(d));
      }
  } else if (max_entry > 0)
    S.db.query(kf.bow, max_entry, sc);
  std::sort(sc.begin(), sc.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b) { return a.second > b.second; });
  const double thr = vpr_.empty() ? std::max(c_.min_score, c_.rel_score * s_prev) : c_.vpr_min;
  std::vector<std::pair<int, float>> cand;
  for (const auto &x : sc) {
    if (static_cast<int>(cand.size()) >= c_.max_cand || x.second < thr) break;
    if (kf.path - S.kfs[x.first].path < c_.min_path_m) continue;
    cand.push_back(x);
  }
  S.n_cand += cand.size();
  const double ms_query = ms_since(t1);

  // 4. geometric verification
  t1 = Clock::now();
  std::ostringstream cj;
  cj.precision(6);
  Verified best;
  bool have_best = false;
  // current keyframe 2D points for PnP (undistorted pixels)
  std::vector<cv::Point2f> pts2 = ptsM;
  if (has_dist && !pts2.empty()) cv::undistortPoints(std::vector<cv::Point2f>(pts2), pts2, K, D, cv::noArray(), K);
  // PnP of 3D points (camera frame of keyframe "obj") against 2D points of keyframe "img": X_img = R X_obj + t.
  // Own brute-force matching (ratio test, one-to-one), RANSAC (EPnP), LM refinement on the inliers, re-count.
  auto pnp = [&](const std::vector<Eigen::Vector3d> &P3o, const cv::Mat &D3o, const cv::Mat &Di,
                 const std::vector<cv::Point2f> &pi, Eigen::Matrix3d &R, Eigen::Vector3d &t, int &n_match, int &n_inl,
                 double &rms, std::string &why) {
    n_match = n_inl = 0;
    rms = 0.0;
    std::vector<int> m_o, m_i;
    {
      std::vector<int> best_o(Di.rows, -1), best_d(Di.rows, 1 << 20);
      for (int i = 0; i < D3o.rows; i++) {
        const uint8_t *a = D3o.ptr<uint8_t>(i);
        int b1 = 1 << 20, b2 = 1 << 20, bj = -1;
        for (int j = 0; j < Di.rows; j++) {
          const int d = hamming32(a, Di.ptr<uint8_t>(j));
          if (d < b1) {
            b2 = b1;
            b1 = d;
            bj = j;
          } else if (d < b2)
            b2 = d;
        }
        if (bj < 0 || b1 > c_.match_dist || b1 >= c_.ratio * b2) continue;
        if (b1 < best_d[bj]) {
          best_d[bj] = b1;
          best_o[bj] = i;
        }
      }
      for (int j = 0; j < Di.rows; j++)
        if (best_o[j] >= 0) {
          m_o.push_back(best_o[j]);
          m_i.push_back(j);
        }
    }
    n_match = static_cast<int>(m_o.size());
    if (n_match < c_.min_inliers) {
      why = "matches";
      return false;
    }
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    for (size_t q = 0; q < m_o.size(); q++) {
      const Eigen::Vector3d &X = P3o[m_o[q]];
      obj.emplace_back(static_cast<float>(X(0)), static_cast<float>(X(1)), static_cast<float>(X(2)));
      img.push_back(pi[m_i[q]]);
    }
    cv::Mat rvec, tvec;
    std::vector<int> inl;
    if (!cv::solvePnPRansac(obj, img, K, cv::noArray(), rvec, tvec, false, c_.pnp_iters, static_cast<float>(c_.pnp_px), 0.99,
                            inl, cv::SOLVEPNP_EPNP)) {
      why = "pnp";
      return false;
    }
    if (static_cast<int>(inl.size()) < c_.min_inliers) {
      n_inl = static_cast<int>(inl.size());
      why = "inliers";
      return false;
    }
    std::vector<cv::Point3f> o2;
    std::vector<cv::Point2f> i2;
    for (int q : inl) {
      o2.push_back(obj[q]);
      i2.push_back(img[q]);
    }
    cv::solvePnPRefineLM(o2, i2, K, cv::noArray(), rvec, tvec);
    std::vector<cv::Point2f> pr;
    cv::projectPoints(obj, rvec, tvec, K, cv::noArray(), pr);
    double e2 = 0.0;
    for (size_t q = 0; q < obj.size(); q++) {
      const double ex = pr[q].x - img[q].x, ey = pr[q].y - img[q].y, e = ex * ex + ey * ey;
      if (e <= c_.pnp_px * c_.pnp_px) {
        n_inl++;
        e2 += e;
      }
    }
    rms = n_inl ? std::sqrt(e2 / n_inl) : 0.0;
    if (n_inl < c_.min_inliers) {
      why = "inliers";
      return false;
    }
    cv::Mat Rm;
    cv::Rodrigues(rvec, Rm);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) R(r, c) = Rm.at<double>(r, c);
    t = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    return true;
  };
  kf.DM = DM;
  kf.ptsM = pts2;
  if (c_.diag_prev && !S.kfs.empty()) cand.insert(cand.begin(), {static_cast<int>(S.kfs.size()) - 1, -1.0f});  // self-test
  for (size_t ci2 = 0; ci2 < cand.size(); ci2++) {
    const bool diag = cand[ci2].second < 0.0f;
    const Kf &old = S.kfs[cand[ci2].first];
    Verified v;
    std::string why;
    S.n_geo++;
    v.old_idx = old.idx;
    Eigen::Matrix3d R_oc_C;
    Eigen::Vector3d t_oc_C;
    // direction A: old keyframe 3D -> current 2D; B (if A fails, e.g. the old keyframe has few 3D points such as the
    // first keyframe after initialisation): current 3D -> old 2D
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    int nm = 0, ni = 0;
    double rms = 0.0;
    bool ok = pnp(old.P3, old.D3, DM, pts2, R, t, nm, ni, rms, why);
    if (ok) {  // X_cur = R X_old + t
      R_oc_C = R.transpose();
      t_oc_C = -R.transpose() * t;
      v.dir = 'A';
    } else {
      std::string whyB;
      int nmB = 0, niB = 0;
      double rmsB = 0.0;
      ok = pnp(kf.P3, kf.D3, old.DM, old.ptsM, R, t, nmB, niB, rmsB, whyB);
      if (ok) {  // X_old = R X_cur + t
        R_oc_C = R;
        t_oc_C = t;
        v.dir = 'B';
        nm = nmB;
        ni = niB;
        rms = rmsB;
      } else if (nmB > nm) {
        nm = nmB;
        ni = niB;
        why = whyB + "B";
      }
    }
    v.n_match = nm;
    v.n_inl = ni;
    v.rms_px = rms;
    {
      if (ok) {
        // T_oldI_curI = T_IC * T_oldC_curC * T_CI
        v.R_oc = in.R_ItoC.transpose() * R_oc_C * in.R_ItoC;
        v.t_oc = in.R_ItoC.transpose() * (R_oc_C * in.p_IinC + t_oc_C - in.p_IinC);
        const Eigen::Matrix3d R_L = old.R_v * v.R_oc;  // loop-implied current attitude (VIO world of the old kf)
        const Eigen::Vector3d p_L = old.p_v + old.R_v * v.t_oc;
        const Eigen::Vector3d g_l = R_L.transpose() * Eigen::Vector3d::UnitZ(), g_v = in.R_ItoG.transpose() * Eigen::Vector3d::UnitZ();
        v.rp_deg = std::acos(std::max(-1.0, std::min(1.0, g_l.dot(g_v)))) * 180.0 / M_PI;
        v.dyaw = wrap(yaw_of(R_L * in.R_ItoG.transpose()));
        v.drift = p_L - in.p_IinG;
        const double path_between = kf.path - old.path;
        if (v.rp_deg > c_.max_rp_deg) {
          ok = false;
          why = "rollpitch";
        } else if (diag) {
          why = "diag";
          ok = false;
        } else if (v.drift.norm() > c_.max_corr_m + c_.max_corr_rel * path_between) {
          ok = false;
          why = "drift";
        } else if (std::fabs(v.dyaw) * 180.0 / M_PI > c_.max_corr_yaw_deg) {
          ok = false;
          why = "yaw";
        }
      }
    }
    if (ci2) cj << ",";
    cj << "[" << old.idx << "," << cand[ci2].second << "," << v.n_match << "," << v.n_inl << ",\"" << (ok ? "ok" : why)
       << "\"," << v.rp_deg << ",\"" << v.dir << "\"," << v.drift.norm() << "," << v.dyaw * 180.0 / M_PI << "," << v.rms_px << "]";
    if (ok && !diag) {
      S.n_verified++;
      if (!have_best || v.n_inl > best.n_inl) {
        best = v;
        have_best = true;
      }
    }
  }
  const double ms_geo = ms_since(t1);

  // 5. temporal consistency
  std::vector<std::pair<int, Verified>> accept;  // (cur kf idx, loop)
  if (have_best) {
    bool consistent = false;
    for (auto &r : S.recent) {
      if (kf.idx - r.cur > c_.consist_kf) continue;
      if (std::fabs(S.kfs[r.old].t - S.kfs[best.old_idx].t) > c_.consist_old_s) continue;
      if ((r.v.drift - best.drift).norm() > c_.consist_m) continue;
      if (std::fabs(wrap(r.v.dyaw - best.dyaw)) * 180.0 / M_PI > c_.consist_yaw_deg) continue;
      consistent = true;
      if (!r.accepted) {
        r.accepted = true;
        accept.emplace_back(r.cur, r.v);
      }
    }
    if (consistent) accept.emplace_back(kf.idx, best);
    else S.n_rej_consist++;
    S.recent.push_back({kf.idx, best.old_idx, best, consistent});
  }
  while (!S.recent.empty() && kf.idx - S.recent.front().cur > c_.consist_kf) S.recent.pop_front();

  // 6. insert the keyframe + edges
  S.db.add(kf.bow);
  S.kfs.push_back(std::move(kf));
  Kf &K_ = S.kfs.back();
  for (int d = 1; d <= c_.seq_edges && K_.idx - d >= 0; d++) {
    const Kf &o = S.kfs[K_.idx - d];
    Edge e;
    e.i = o.idx;
    e.j = K_.idx;
    e.dt = Rz(o.yaw_v).transpose() * (K_.p_v - o.p_v);
    e.dyaw = wrap(K_.yaw_v - o.yaw_v);
    S.edges.push_back(e);
  }
  std::ostringstream lj;
  lj.precision(9);
  for (const auto &a : accept) {
    const Kf &o = S.kfs[a.second.old_idx];
    Edge e;
    e.i = o.idx;
    e.j = a.first;
    e.dt = Rz(o.yaw_v).transpose() * (o.R_v * a.second.t_oc);
    e.dyaw = wrap(yaw_of(o.R_v * a.second.R_oc) - o.yaw_v);
    e.loop = true;
    S.edges.push_back(e);
    S.n_loop++;
    S.n_accepted++;
    const Kf &c = S.kfs[a.first];
    Eigen::Quaterniond q(a.second.R_oc);
    lj << "{\"type\":\"loop\",\"session\":" << S.session << ",\"cur\":" << c.idx << ",\"old\":" << o.idx
       << ",\"t_cur_ns\":" << c.t_ns << ",\"t_old_ns\":" << o.t_ns << ",\"n_match\":" << a.second.n_match
       << ",\"n_inl\":" << a.second.n_inl << ",\"rms_px\":" << a.second.rms_px << ",\"rp_deg\":" << a.second.rp_deg
       << ",\"q_oc\":[" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "],\"t_oc\":";
    jv(lj, a.second.t_oc);
    lj << ",\"drift\":";
    jv(lj, a.second.drift);
    lj << ",\"dyaw\":" << a.second.dyaw << ",\"path_between\":" << (c.path - o.path) << "}\n";
  }

  // 7. optimise when a loop was added
  double ms_opt = 0.0;
  if (!accept.empty()) {
    t1 = Clock::now();
    optimise();
    ms_opt = ms_since(t1);
    S.ms_opt.push_back(ms_opt);
  }
  const double ms_total = ms_since(T0);
  S.ms_total.push_back(ms_total);
  S.ms_tri.push_back(ms_tri);
  S.ms_feat.push_back(ms_feat);
  S.ms_query.push_back(ms_query);
  S.ms_geo.push_back(ms_geo);
  if (log_) {
    std::ostringstream o;
    o.precision(9);
    Eigen::Quaterniond q(K_.R_v);
    o << "{\"type\":\"kf\",\"session\":" << S.session << ",\"idx\":" << K_.idx << ",\"t_ns\":" << K_.t_ns << ",\"t\":" << K_.t
      << ",\"p_v\":";
    jv(o, K_.p_v);
    o << ",\"q_v\":[" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "],\"path\":" << K_.path
      << ",\"n_tracks\":" << in.tracks.size() << ",\"n3d\":" << K_.P3.size() << ",\"n2d\":" << D2.rows
      << ",\"n_words\":" << K_.bow.size() << ",\"s_prev\":" << s_prev << ",\"n_db\":" << max_entry << ",\"cand\":[" << cj.str()
      << "],\"acc\":" << accept.size() << ",\"ms\":[" << in.cap_ms << "," << ms_tri << "," << ms_feat << "," << ms_query << ","
      << ms_geo << "," << ms_opt << "," << ms_total << "]}\n";
    log_line(o.str() + lj.str());
  }
}

void LoopCloser::optimise() {
  State &S = *s_;
  ceres::Problem prob;
  for (auto &k : S.kfs) {
    prob.AddParameterBlock(&k.yaw_c, 1);
    prob.AddParameterBlock(k.t_c, 3);
  }
  prob.SetParameterBlockConstant(&S.kfs[0].yaw_c);
  prob.SetParameterBlockConstant(S.kfs[0].t_c);
  for (const auto &e : S.edges) {
    FourDof *f;
    ceres::LossFunction *loss = nullptr;
    if (c_.weight_mode == 1) {
      const double d = e.dt.norm(), d2r = M_PI / 180.0;
      const double st = e.loop ? c_.loop_sig_m : c_.seq_sig_m + c_.seq_sig_rel * d;
      const double sy = (e.loop ? c_.loop_sig_deg : c_.seq_sig_deg + c_.seq_sig_deg_m * d) * d2r;
      f = new FourDof(e.dt, e.dyaw, 1.0 / st, 1.0 / sy);
      if (e.loop) loss = new ceres::HuberLoss(c_.huber_sig);
    } else {
      f = new FourDof(e.dt, e.dyaw, e.loop ? c_.loop_w : 1.0);
      if (e.loop) loss = new ceres::HuberLoss(c_.huber);
    }
    auto *cost = new ceres::AutoDiffCostFunction<FourDof, 4, 1, 3, 1, 3>(f);
    prob.AddResidualBlock(cost, loss, &S.kfs[e.i].yaw_c, S.kfs[e.i].t_c, &S.kfs[e.j].yaw_c, S.kfs[e.j].t_c);
  }
  ceres::Solver::Options o;
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE)) {
    o.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    o.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  } else {
    o.linear_solver_type = ceres::DENSE_QR;
  }
  o.max_num_iterations = c_.opt_iters;
  o.num_threads = 1;
  o.logging_type = ceres::SILENT;
  o.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary sum;
  ceres::Solve(o, &prob, &sum);
  for (auto &k : S.kfs) k.yaw_c = wrap(k.yaw_c);
  const Kf &L = S.kfs.back();
  S.corr_yaw = wrap(L.yaw_c - L.yaw_v);
  S.corr_t = Eigen::Vector3d(L.t_c[0], L.t_c[1], L.t_c[2]) - Rz(S.corr_yaw) * L.p_v;
  S.n_opt++;
  {
    std::lock_guard<std::mutex> l(om_);
    if (out_.session == S.session) {  // ignore results of an old session
      out_.valid = true;
      out_.tgt_yaw = S.corr_yaw;
      out_.tgt_t = S.corr_t;
      out_.n_loop = S.n_loop;
      out_.n_kf = static_cast<int>(S.kfs.size());
      out_.ver++;
    }
  }
  if (log_) {
    std::ostringstream o2;
    o2.precision(9);
    o2 << "{\"type\":\"opt\",\"session\":" << S.session << ",\"n_kf\":" << S.kfs.size() << ",\"n_edges\":" << S.edges.size()
       << ",\"n_loop\":" << S.n_loop << ",\"iters\":" << sum.iterations.size() << ",\"cost0\":" << sum.initial_cost
       << ",\"cost1\":" << sum.final_cost << ",\"tgt_yaw\":" << S.corr_yaw << ",\"tgt_t\":";
    jv(o2, S.corr_t);
    o2 << ",\"t_kf_ns\":" << L.t_ns << "}\n";
    log_line(o2.str());
  }
}

// ---------------------------------------------------------------- output (main thread)
MapOdom LoopCloser::step_output(double t, const Eigen::Vector3d &p) {
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
  if (!out_.converging) {  // exactly the target once reached (no rounding creep)
    out_.yaw = out_.tgt_yaw;
    out_.t = out_.tgt_t;
  }
  return out_;
}

MapOdom LoopCloser::published() const {
  std::lock_guard<std::mutex> l(om_);
  return out_;
}

void LoopCloser::finish(double timeout_s) {
  std::unique_lock<std::mutex> l(qm_);
  const bool ok = done_cv_.wait_for(l, std::chrono::duration<double>(timeout_s), [&] { return n_done_ >= n_pushed_; });
  if (!ok)
    std::fprintf(stderr, "[ov_bridge] loop: finish timeout (%llu of %llu keyframes processed)\n",
                 static_cast<unsigned long long>(n_done_), static_cast<unsigned long long>(n_pushed_));
}

void LoopCloser::report() const {
  const State &S = *s_;
  uint64_t pushed, dropped;
  {
    std::lock_guard<std::mutex> l(qm_);
    pushed = n_pushed_;
    dropped = n_dropped_;
  }
  std::fprintf(stderr,
               "[ov_bridge] loop: keyframes pushed %llu dropped %llu processed %llu | candidates %llu geo %llu verified %llu "
               "accepted %llu (consistency-rejected %llu) | optimisations %u | sessions %d\n",
               static_cast<unsigned long long>(pushed), static_cast<unsigned long long>(dropped),
               static_cast<unsigned long long>(S.n_kf), static_cast<unsigned long long>(S.n_cand),
               static_cast<unsigned long long>(S.n_geo), static_cast<unsigned long long>(S.n_verified),
               static_cast<unsigned long long>(S.n_accepted), static_cast<unsigned long long>(S.n_rej_consist), S.n_opt,
               session_);
  std::fprintf(stderr,
               "[ov_bridge] loop: ms p50/p95/max per keyframe: total %.2f/%.2f/%.2f tri %.2f/%.2f feat %.2f/%.2f query "
               "%.2f/%.2f geo %.2f/%.2f | VIO-thread copy %.3f/%.3f/%.3f | opt %.1f/%.1f/%.1f (n %zu)\n",
               pct(S.ms_total, 50), pct(S.ms_total, 95), pct(S.ms_total, 100), pct(S.ms_tri, 50), pct(S.ms_tri, 95),
               pct(S.ms_feat, 50), pct(S.ms_feat, 95), pct(S.ms_query, 50), pct(S.ms_query, 95), pct(S.ms_geo, 50),
               pct(S.ms_geo, 95), pct(S.ms_cap, 50), pct(S.ms_cap, 95), pct(S.ms_cap, 100), pct(S.ms_opt, 50),
               pct(S.ms_opt, 95), pct(S.ms_opt, 100), S.ms_opt.size());
}

}  // namespace ovloop
