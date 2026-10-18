// arloop: loop closure on top of ARCore poses. See arloop.h for the model.
#include "arloop.h"

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define ARLOOP_NEW_ARUCO 1
#else
#include <opencv2/aruco.hpp>
#define ARLOOP_NEW_ARUCO 0
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <sys/resource.h>

namespace arloop {
namespace {

// ARCore world (Y up) -> Z-up world: (x, y, z) -> (x, -z, y)
const Eigen::Matrix3d A = (Eigen::Matrix3d() << 1, 0, 0, 0, 0, -1, 0, 1, 0).finished();
// OpenGL camera (y up, -z forward) <-> OpenCV camera (y down, z forward)
const Eigen::Matrix3d F = Eigen::Vector3d(1, -1, -1).asDiagonal();

double now_ms() {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double norm_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Eigen::Matrix3d Rz(double y) { return Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

void ypr(const Eigen::Matrix3d &R, double &yaw, double &pitch, double &roll) {   // R = Rz(yaw) Ry(pitch) Rx(roll)
  yaw = std::atan2(R(1, 0), R(0, 0));
  pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  roll = std::atan2(R(2, 1), R(2, 2));
}

Eigen::Matrix3d from_ypr(double yaw, double pitch, double roll) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double yaw_of(const Eigen::Matrix3d &R) { return std::atan2(R(1, 0), R(0, 0)); }

// 4-DoF relative pose residual (VINS-Mono FourDOFError): R_i^T (t_j - t_i) - t_ij, yaw_j - yaw_i - yaw_ij.
// R_i = Rz(yaw_i) Ry(pitch_i) Rx(roll_i) with pitch/roll of keyframe i fixed.
struct FourDof {
  FourDof(const Eigen::Vector3d &t, double yaw, double pitch, double roll, double wt, double wy)
      : t_(t), yaw_(yaw), cp_(std::cos(pitch)), sp_(std::sin(pitch)), cr_(std::cos(roll)), sr_(std::sin(roll)),
        wt_(wt), wy_(wy) {}
  template <typename T>
  bool operator()(const T *yi, const T *ti, const T *yj, const T *tj, T *r) const {
    T d0 = tj[0] - ti[0], d1 = tj[1] - ti[1], d2 = tj[2] - ti[2];
    T c = cos(yi[0]), s = sin(yi[0]);
    T a0 = c * d0 + s * d1, a1 = -s * d0 + c * d1, a2 = d2;                  // Rz^T
    T b0 = T(cp_) * a0 - T(sp_) * a2, b1 = a1, b2 = T(sp_) * a0 + T(cp_) * a2;   // Ry^T
    T e0 = b0, e1 = T(cr_) * b1 + T(sr_) * b2, e2 = -T(sr_) * b1 + T(cr_) * b2;  // Rx^T
    r[0] = (e0 - T(t_.x())) * T(wt_);
    r[1] = (e1 - T(t_.y())) * T(wt_);
    r[2] = (e2 - T(t_.z())) * T(wt_);
    T dy = yj[0] - yi[0] - T(yaw_);
    r[3] = atan2(sin(dy), cos(dy)) * T(wy_);
    return true;
  }
  Eigen::Vector3d t_;
  double yaw_, cp_, sp_, cr_, sr_, wt_, wy_;
};

}  // namespace

LoopCloser::LoopCloser(const Options &o) : o_(o) {
  orb_ = cv::ORB::create(o_.orb_features, 1.2f, 8, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31, 12);
  if (o_.threaded) th_ = std::thread([this] { worker(); });
}

LoopCloser::~LoopCloser() {
  stop_ = true;
  qcv_.notify_all();
  if (th_.joinable()) th_.join();
}

void LoopCloser::add_keyframe(KeyframeIn kf) {
  {
    std::lock_guard<std::mutex> l(cmu_);
    last_seen_ns_ = std::max(last_seen_ns_, kf.t_ns);
  }
  if (!o_.threaded) {
    process(kf);
    return;
  }
  {
    std::lock_guard<std::mutex> l(qmu_);
    if (q_.size() > 8) q_.pop_front();   // worker fell behind: keep the newest keyframes
    q_.push_back(std::move(kf));
  }
  qcv_.notify_one();
}

void LoopCloser::worker() {
  // background priority (nice 10, this thread only on Linux/Android): loop closure must never take CPU from ARCore
  setpriority(PRIO_PROCESS, 0, 10);
  while (!stop_) {
    KeyframeIn kf;
    {
      std::unique_lock<std::mutex> l(qmu_);
      qcv_.wait(l, [this] { return stop_ || !q_.empty(); });
      if (stop_) return;
      kf = std::move(q_.front());
      q_.pop_front();
    }
    try {
      process(kf);
    } catch (const std::exception &e) {
      event(std::string("error ") + e.what());
    }
  }
}

void LoopCloser::event(const std::string &s) {
  std::lock_guard<std::mutex> l(smu_);
  events_.push_back(s);
  if (events_.size() > 8) events_.pop_front();
}

Pose4 LoopCloser::correction_of(const KF &kf) const {
  Pose4 c;
  c.yaw = norm_angle(kf.opt_yaw - kf.yaw);
  c.t = Eigen::Vector3d(kf.opt_t[0], kf.opt_t[1], kf.opt_t[2]) - Rz(c.yaw) * kf.t;
  return c;
}

void LoopCloser::apply_current(KF &kf) const {
  Pose4 c;
  if (!kfs_.empty()) c = correction_of(*kfs_.back());
  Eigen::Vector3d t = Rz(c.yaw) * kf.t + c.t;
  for (int k = 0; k < 3; k++) kf.opt_t[k] = t[k];
  kf.opt_yaw = norm_angle(kf.yaw + c.yaw);
}

void LoopCloser::process(KeyframeIn &in) {
  double t0 = now_ms();
  auto kf = std::make_unique<KF>();
  kf->id = static_cast<int>(kfs_.size());
  kf->t_ns = in.t_ns;
  kf->R = A * in.q_gl.normalized().toRotationMatrix() * F;
  kf->t = A * in.t_gl;
  ypr(kf->R, kf->yaw, kf->pitch, kf->roll);
  kf->fx = in.fx; kf->fy = in.fy; kf->cx = in.cx; kf->cy = in.cy;
  apply_current(*kf);

  // odometry edges to the 2 previous keyframes. Loose on purpose: ARCore drift is slow but correlated, and stiff
  // multi-edge odometry (4 edges, 2 cm) made it cheaper to ignore a 0.2 m loop than to close it (offline test 2026-10-18)
  int n = static_cast<int>(kfs_.size());
  for (int i = std::max(0, n - 2); i < n; i++) {
    const KF &p = *kfs_[i];
    Edge e;
    e.i = i; e.j = kf->id; e.marker = false;
    e.t_ij = p.R.transpose() * (kf->t - p.t);
    e.yaw_ij = norm_angle(kf->yaw - p.yaw);
    e.w_t = 1.0 / 0.05; e.w_yaw = 1.0 / 0.02; e.robust = false;
    edges_.push_back(e);
  }
  KF &k = *kf;
  kfs_.push_back(std::move(kf));
  n_kf_ = static_cast<int>(kfs_.size());

  // markers are found in both modes: for marker edges (markers) and to keep ORB off them (features) — a marker's
  // repeated grid made consistent false feature loops (-16 deg) in the offline test
  std::vector<std::vector<cv::Point2f>> mcorners;
  if ((o_.markers || o_.features) && !in.gray.empty()) detect_markers(k, in.gray, mcorners);

  if (o_.features && !in.gray.empty()) {
    cv::Mat mask;
    if (!mcorners.empty()) {
      mask = cv::Mat(in.gray.size(), CV_8UC1, cv::Scalar(255));
      for (auto &c : mcorners) {
        cv::Point2f ctr = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
        std::vector<cv::Point> poly;
        for (auto &q : c) poly.emplace_back(ctr + (q - ctr) * 1.5f);
        cv::fillConvexPoly(mask, poly, cv::Scalar(0));
      }
    }
    orb_->detectAndCompute(in.gray, mask, k.kps, k.desc);   // empty mask = whole image
    if (!in.depth_mm.empty() && !k.desc.empty()) {
      double sx = static_cast<double>(in.depth_mm.cols) / in.gray.cols, sy = static_cast<double>(in.depth_mm.rows) / in.gray.rows;
      std::vector<int> rows;
      for (size_t i = 0; i < k.kps.size(); i++) {
        int u = static_cast<int>(k.kps[i].pt.x * sx), v = static_cast<int>(k.kps[i].pt.y * sy);
        if (u < 0 || v < 0 || u >= in.depth_mm.cols || v >= in.depth_mm.rows) continue;
        double z = in.depth_mm.at<uint16_t>(v, u) * 1e-3;   // ARCore DEPTH16: low 13 bits mm (confidence bits unused by acquireDepthImage16Bits)
        if (z < 0.25 || z > 6.0) continue;
        double x = (k.kps[i].pt.x - k.cx) / k.fx * z, y = (k.kps[i].pt.y - k.cy) / k.fy * z;
        k.p3d.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        rows.push_back(static_cast<int>(i));
      }
      if (rows.size() >= 50) {
        k.desc3d.create(static_cast<int>(rows.size()), k.desc.cols, k.desc.type());
        for (size_t r = 0; r < rows.size(); r++) k.desc.row(rows[r]).copyTo(k.desc3d.row(static_cast<int>(r)));
        n_depth_kf_++;
      } else {
        k.p3d.clear();
      }
    } else if (!k.desc.empty() && k.id > 0) {
      triangulate(k, *kfs_[k.id - 1]);
    }
    if (find_loop(k)) optimize("loop");
  }
  proc_ms_ = now_ms() - t0;
}

void LoopCloser::detect_markers(KF &kf, const cv::Mat &gray, std::vector<std::vector<cv::Point2f>> &corners) {
  std::vector<int> ids;
#if ARLOOP_NEW_ARUCO
  static thread_local cv::aruco::ArucoDetector det(cv::aruco::getPredefinedDictionary(o_.marker_dict),
                                                   cv::aruco::DetectorParameters());
  det.detectMarkers(gray, corners, ids);
#else
  static thread_local cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(o_.marker_dict);
  static thread_local cv::Ptr<cv::aruco::DetectorParameters> prm = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(gray, dict, corners, ids, prm);
#endif
  if (ids.empty() || !o_.markers) return;
  double L = o_.marker_len;
  std::vector<cv::Point3f> obj = {{float(-L / 2), float(L / 2), 0}, {float(L / 2), float(L / 2), 0},
                                  {float(L / 2), float(-L / 2), 0}, {float(-L / 2), float(-L / 2), 0}};
  cv::Matx33d K(kf.fx, 0, kf.cx, 0, kf.fy, kf.cy, 0, 0, 1);
  bool want_opt = false;
  for (size_t m = 0; m < ids.size(); m++) {
    if (cv::arcLength(corners[m], true) < 4 * 30) continue;   // too small to trust the pose
    cv::Vec3d rv, tv;
    if (!cv::solvePnP(obj, corners[m], K, cv::noArray(), rv, tv, false, cv::SOLVEPNP_IPPE_SQUARE)) continue;
    Eigen::Vector3d t_cm(tv[0], tv[1], tv[2]);
    if (t_cm.z() <= 0.1 || t_cm.norm() > 5.0) continue;
    cv::Matx33d Rcv;
    cv::Rodrigues(rv, Rcv);
    Eigen::Matrix3d R_cm;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) R_cm(r, c) = Rcv(r, c);
    Edge e;
    e.i = kf.id; e.j = ids[m]; e.marker = true;
    e.t_ij = t_cm;
    e.yaw_ij = norm_angle(yaw_of(kf.R * R_cm) - kf.yaw);
    double dist = t_cm.norm();
    e.w_t = 1.0 / (0.01 + 0.02 * dist); e.w_yaw = 1.0 / 0.03; e.robust = true;
    auto it = markers_.find(ids[m]);
    if (it == markers_.end()) {   // first sighting: marker node from the keyframe's corrected pose
      Marker mk;
      mk.id = ids[m];
      Eigen::Matrix3d Rk = from_ypr(kf.opt_yaw, kf.pitch, kf.roll);
      Eigen::Vector3d tm = Eigen::Vector3d(kf.opt_t[0], kf.opt_t[1], kf.opt_t[2]) + Rk * t_cm;
      for (int c = 0; c < 3; c++) mk.opt_t[c] = tm[c];
      mk.opt_yaw = yaw_of(Rk * R_cm);
      markers_[ids[m]] = mk;
      n_markers_ = static_cast<int>(markers_.size());
      it = markers_.find(ids[m]);
      event("marker " + std::to_string(ids[m]) + " first seen at kf " + std::to_string(kf.id));
    } else if (kf.t_ns - it->second.last_t_ns > 2'000'000'000LL) {
      want_opt = true;      // seen again after a while = marker loop
      event("marker " + std::to_string(ids[m]) + " seen again (kf " + std::to_string(kf.id) + ")");
    } else if (it->second.sightings % 15 == 14) {
      want_opt = true;
    }
    it->second.sightings++;
    it->second.last_t_ns = kf.t_ns;
    edges_.push_back(e);
    n_marker_edges_++;
  }
  if (want_opt) optimize("marker");
}

// 3D points for keyframe c without ARCore depth: match ORB with the previous keyframe p and triangulate with the two
// ARCore poses (ARCore depth costs ~100 ms of pose latency on the S24: 75 -> 176 ms median, 2026-10-18).
void LoopCloser::triangulate(KF &c, const KF &p) {
  if (p.desc.empty() || (c.t - p.t).norm() < 0.05) return;
  cv::BFMatcher bf(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> mm;
  bf.knnMatch(c.desc, p.desc, mm, 2);
  // world -> camera for both
  Eigen::Matrix3d Rc = c.R.transpose(), Rp = p.R.transpose();
  Eigen::Vector3d tc = -Rc * c.t, tp = -Rp * p.t;
  auto proj = [](const Eigen::Matrix3d &R, const Eigen::Vector3d &t, double fx, double fy, double cx, double cy) {
    cv::Matx34d P;
    for (int r = 0; r < 3; r++) {
      for (int q = 0; q < 3; q++) P(r, q) = R(r, q);
      P(r, 3) = t[r];
    }
    cv::Matx33d K(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    return cv::Mat(K * P);
  };
  cv::Mat Pc = proj(Rc, tc, c.fx, c.fy, c.cx, c.cy), Pp = proj(Rp, tp, p.fx, p.fy, p.cx, p.cy);
  std::vector<cv::Point2f> uc, up;
  std::vector<int> qi;
  for (auto &m : mm)
    if (m.size() == 2 && m[0].distance < 0.8f * m[1].distance && m[0].distance < 64) {
      uc.push_back(c.kps[m[0].queryIdx].pt);
      up.push_back(p.kps[m[0].trainIdx].pt);
      qi.push_back(m[0].queryIdx);
    }
  if (uc.size() < 50) return;
  cv::Mat X;
  cv::triangulatePoints(Pc, Pp, uc, up, X);
  X.convertTo(X, CV_64F);
  std::vector<int> rows;
  for (int i = 0; i < X.cols; i++) {
    double w = X.at<double>(3, i);
    if (std::abs(w) < 1e-9) continue;
    Eigen::Vector3d Xw(X.at<double>(0, i) / w, X.at<double>(1, i) / w, X.at<double>(2, i) / w);
    Eigen::Vector3d xc = Rc * Xw + tc, xp = Rp * Xw + tp;
    if (xc.z() < 0.25 || xp.z() < 0.25 || xc.z() > 8.0) continue;
    double ec = std::hypot(c.fx * xc.x() / xc.z() + c.cx - uc[i].x, c.fy * xc.y() / xc.z() + c.cy - uc[i].y);
    double ep = std::hypot(p.fx * xp.x() / xp.z() + p.cx - up[i].x, p.fy * xp.y() / xp.z() + p.cy - up[i].y);
    if (ec > 2.0 || ep > 2.0) continue;
    Eigen::Vector3d rc = Xw - c.t, rp = Xw - p.t;
    double par = std::acos(std::max(-1.0, std::min(1.0, rc.normalized().dot(rp.normalized())))) * 180 / M_PI;
    if (par < 1.0) continue;   // too little parallax for a usable depth
    c.p3d.emplace_back(static_cast<float>(xc.x()), static_cast<float>(xc.y()), static_cast<float>(xc.z()));
    rows.push_back(qi[i]);
  }
  if (rows.size() >= 40) {
    c.desc3d.create(static_cast<int>(rows.size()), c.desc.cols, c.desc.type());
    for (size_t r = 0; r < rows.size(); r++) c.desc.row(rows[r]).copyTo(c.desc3d.row(static_cast<int>(r)));
    n_depth_kf_++;
  } else {
    c.p3d.clear();
  }
}

bool LoopCloser::find_loop(KF &kf) {
  if (kf.desc.empty() || kf.kps.size() < 50) return false;
  double t_s = kf.t_ns * 1e-9;
  if (t_s - last_loop_t_ < 1.0) return false;    // one accepted loop per second is plenty
  Eigen::Vector3d tc(kf.opt_t[0], kf.opt_t[1], kf.opt_t[2]);
  Eigen::Vector3d zc = kf.R.col(2);
  std::vector<std::pair<double, int>> cands;
  for (const auto &p : kfs_) {
    if (p->id == kf.id || p->desc3d.empty()) continue;
    if ((kf.t_ns - p->t_ns) * 1e-9 < o_.min_gap_s) continue;
    double d = (Eigen::Vector3d(p->opt_t[0], p->opt_t[1], p->opt_t[2]) - tc).norm();
    if (d > o_.loop_radius) continue;
    if (p->R.col(2).dot(zc) < std::cos(60.0 * M_PI / 180.0)) continue;   // looking a similar way
    cands.emplace_back(d, p->id);
  }
  if (cands.empty()) return false;
  std::sort(cands.begin(), cands.end());
  if (cands.size() > 3) cands.resize(3);
  cv::BFMatcher bf(cv::NORM_HAMMING);
  cv::Matx33d K(kf.fx, 0, kf.cx, 0, kf.fy, kf.cy, 0, 0, 1);
  for (auto &c : cands) {
    KF &p = *kfs_[c.second];
    std::vector<std::vector<cv::DMatch>> mm;
    bf.knnMatch(kf.desc, p.desc3d, mm, 2);
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    for (auto &m : mm)
      if (m.size() == 2 && m[0].distance < 0.8f * m[1].distance && m[0].distance < 64) {
        obj.push_back(p.p3d[m[0].trainIdx]);
        img.push_back(kf.kps[m[0].queryIdx].pt);
      }
    if (static_cast<int>(obj.size()) < o_.min_inliers + 5) continue;
    cv::Vec3d rv, tv;
    std::vector<int> inl;
    if (!cv::solvePnPRansac(obj, img, K, cv::noArray(), rv, tv, false, 200, 3.0f, 0.99, inl, cv::SOLVEPNP_EPNP)) continue;
    // weak matches made false loops in the offline test (25 inliers -> 10 deg wrong yaw): need enough inliers AND ratio
    if (static_cast<int>(inl.size()) < o_.min_inliers || inl.size() < 0.35 * obj.size()) continue;
    cv::Matx33d Rcv;
    cv::Rodrigues(rv, Rcv);
    Eigen::Matrix3d R_ck;   // X_c = R_ck X_k + t_ck
    for (int r = 0; r < 3; r++)
      for (int q = 0; q < 3; q++) R_ck(r, q) = Rcv(r, q);
    Eigen::Matrix3d R_kc = R_ck.transpose();
    Eigen::Vector3d t_kc = -R_kc * Eigen::Vector3d(tv[0], tv[1], tv[2]);
    // sanity vs. the current estimate: a false match implies a huge jump
    Eigen::Matrix3d Rk = from_ypr(p.opt_yaw, p.pitch, p.roll), Rc = from_ypr(kf.opt_yaw, kf.pitch, kf.roll);
    Eigen::Vector3d pred = Rk.transpose() * (tc - Eigen::Vector3d(p.opt_t[0], p.opt_t[1], p.opt_t[2]));
    double jump = (t_kc - pred).norm();
    double ang = Eigen::AngleAxisd(R_kc.transpose() * (Rk.transpose() * Rc)).angle() * 180.0 / M_PI;
    if (jump > o_.max_jump || ang > 15.0) {
      event("loop rejected kf " + std::to_string(kf.id) + "->" + std::to_string(p.id) + " jump " +
            std::to_string(jump).substr(0, 4) + " m, " + std::to_string(int(ang)) + " deg");
      continue;
    }
    Edge e;
    e.i = p.id; e.j = kf.id; e.marker = false;
    e.t_ij = t_kc;
    e.yaw_ij = norm_angle(yaw_of(p.R * R_kc) - p.yaw);
    e.w_t = 1.0 / 0.03; e.w_yaw = 1.0 / 0.01; e.robust = true;
    // correction this loop implies for the current keyframe
    Eigen::Vector3d tgt = Eigen::Vector3d(p.opt_t[0], p.opt_t[1], p.opt_t[2]) + Rk * t_kc;
    Eigen::Vector3d dt = tgt - tc;
    double dyaw = norm_angle(p.opt_yaw + e.yaw_ij - kf.opt_yaw);
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const Pending &q) { return kf.id - q.kf > 10; }),
                   pending_.end());
    int match = -1;
    for (size_t q = 0; q < pending_.size(); q++)
      if (pending_[q].kf != kf.id && (dt - pending_[q].dt).norm() < 0.20 &&
          std::abs(norm_angle(dyaw - pending_[q].dyaw)) < 4.0 * M_PI / 180.0) match = static_cast<int>(q);
    if (match < 0) {   // remember it and wait for a confirming loop from one of the next keyframes
      Pending q;
      q.kf = kf.id; q.e = e; q.dt = dt; q.dyaw = dyaw;
      pending_.push_back(q);
      char buf[160];
      std::snprintf(buf, sizeof buf, "loop? kf %d->%d inl %zu implies dt %.2f %.2f %.2f dyaw %.1f deg (waiting)", kf.id, p.id,
                    inl.size(), dt.x(), dt.y(), dt.z(), dyaw * 180 / M_PI);
      event(buf);
      if (getenv("ARLOOP_DEBUG")) std::fprintf(stderr, "%s\n", buf);
      return false;
    }
    edges_.push_back(pending_[match].e);
    edges_.push_back(e);
    pending_.clear();
    n_loops_ += 2;
    last_loop_t_ = t_s;
    event("loop kf " + std::to_string(kf.id) + "->" + std::to_string(p.id) + " inliers " + std::to_string(inl.size()) +
          " jump " + std::to_string(jump).substr(0, 4) + " m (confirmed)");
    return true;
  }
  return false;
}

void LoopCloser::optimize(const char *why) {
  if (kfs_.size() < 2) return;
  double t0 = now_ms();
  ceres::Problem prob;
  for (auto &k : kfs_) {
    prob.AddParameterBlock(&k->opt_yaw, 1);
    prob.AddParameterBlock(k->opt_t, 3);
  }
  prob.SetParameterBlockConstant(&kfs_[0]->opt_yaw);
  prob.SetParameterBlockConstant(kfs_[0]->opt_t);
  for (auto &e : edges_) {
    KF &a = *kfs_[e.i];
    double *yj, *tj;
    if (e.marker) {
      Marker &m = markers_.at(e.j);
      yj = &m.opt_yaw; tj = m.opt_t;
    } else {
      KF &b = *kfs_[e.j];
      yj = &b.opt_yaw; tj = b.opt_t;
    }
    auto *cost = new ceres::AutoDiffCostFunction<FourDof, 4, 1, 3, 1, 3>(
        new FourDof(e.t_ij, e.yaw_ij, a.pitch, a.roll, e.w_t, e.w_yaw));
    prob.AddResidualBlock(cost, e.robust ? new ceres::HuberLoss(3.0) : nullptr, &a.opt_yaw, a.opt_t, yj, tj);
  }
  ceres::Solver::Options so;
  so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  so.max_num_iterations = 15;
  so.num_threads = 1;
  ceres::Solver::Summary sum;
  ceres::Solve(so, &prob, &sum);
  for (auto &k : kfs_) k->opt_yaw = norm_angle(k->opt_yaw);
  Pose4 c = correction_of(*kfs_.back());
  {
    std::lock_guard<std::mutex> l(cmu_);
    // start the blend from wherever the previous blend currently is
    double a = o_.blend_s > 0 ? std::min(1.0, (last_seen_ns_ - corr_change_ns_) * 1e-9 / o_.blend_s) : 1.0;
    Pose4 cur;
    cur.yaw = corr_prev_.yaw + a * norm_angle(corr_target_.yaw - corr_prev_.yaw);
    cur.t = corr_prev_.t + a * (corr_target_.t - corr_prev_.t);
    corr_prev_ = cur;
    corr_target_ = c;
    corr_change_ns_ = last_seen_ns_;
  }
  last_opt_ms_ = now_ms() - t0;
  n_opt_++;
  char buf[160];
  std::snprintf(buf, sizeof buf, "opt (%s) %d kf %zu edges %.0f ms -> corr yaw %.2f deg t %.2f %.2f %.2f", why,
                static_cast<int>(kfs_.size()), edges_.size(), last_opt_ms_.load(), c.yaw * 180 / M_PI, c.t.x(), c.t.y(), c.t.z());
  event(buf);
}

void LoopCloser::correct(int64_t t_ns, const Eigen::Vector3d &t_gl, const Eigen::Quaterniond &q_gl,
                         Eigen::Vector3d &t_out, Eigen::Quaterniond &q_out) {
  Pose4 c;
  {
    std::lock_guard<std::mutex> l(cmu_);
    last_seen_ns_ = std::max(last_seen_ns_, t_ns);
    double a = o_.blend_s > 0 ? std::min(1.0, std::max(0.0, (t_ns - corr_change_ns_) * 1e-9 / o_.blend_s)) : 1.0;
    c.yaw = corr_prev_.yaw + a * norm_angle(corr_target_.yaw - corr_prev_.yaw);
    c.t = corr_prev_.t + a * (corr_target_.t - corr_prev_.t);
  }
  Eigen::Matrix3d R = A * q_gl.normalized().toRotationMatrix() * F;
  Eigen::Vector3d t = A * t_gl;
  Eigen::Matrix3d R2 = Rz(c.yaw) * R;
  Eigen::Vector3d t2 = Rz(c.yaw) * t + c.t;
  t_out = A.transpose() * t2;
  q_out = Eigen::Quaterniond(A.transpose() * R2 * F);
}

std::string LoopCloser::stats_json() {
  std::ostringstream s;
  Pose4 c;
  {
    std::lock_guard<std::mutex> l(cmu_);
    c = corr_target_;
  }
  std::lock_guard<std::mutex> l(smu_);
  s << "{\"kf\":" << n_kf_ << ",\"depth_kf\":" << n_depth_kf_ << ",\"loops\":" << n_loops_
    << ",\"marker_edges\":" << n_marker_edges_ << ",\"markers\":" << n_markers_ << ",\"opt\":" << n_opt_
    << ",\"opt_ms\":" << static_cast<int>(last_opt_ms_.load()) << ",\"proc_ms\":" << static_cast<int>(proc_ms_.load())
    << ",\"corr_yaw_deg\":" << c.yaw * 180 / M_PI << ",\"corr_t\":[" << c.t.x() << "," << c.t.y() << "," << c.t.z()
    << "],\"events\":[";
  bool first = true;
  for (auto &e : events_) {
    s << (first ? "" : ",") << "\"" << e << "\"";
    first = false;
  }
  s << "]}";
  return s.str();
}

}  // namespace arloop
