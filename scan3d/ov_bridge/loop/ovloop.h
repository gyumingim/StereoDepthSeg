// ovloop.h (2026-10-26, branch ovloop-20261026): loop closure for ov_bridge (place recognition + 4-DoF pose graph).
//
// ov_bridge --loop 1 (default 0 = this code is never called, poses bit-identical). Design (user decision 2026-10-26):
//  - The VIO front-end is unchanged. The VIO (main) thread only COPIES a keyframe (image + VIO pose + tracked points with
//    their past observations + clone poses + SLAM landmarks) when the keyframe rule fires (KfRule, a few Hz) - reads only.
//  - A separate low-priority thread (LoopCloser) does everything else: triangulation, features, BoW, candidate search,
//    geometric verification (PnP RANSAC against the old keyframe's 3D points), temporal consistency, 4-DoF pose graph
//    (x, y, z, yaw; roll / pitch from the gravity-aligned VIO) with Ceres.
//  - The loop thread never calls OpenCV parallel_for_ code (own FAST + box-BRIEF features, own matcher, own pyramid):
//    OpenCV 4 runs any parallel_for_ that starts while another one is active SEQUENTIALLY (global nested-region flag), so
//    an OpenCV-parallel call here could serialise the VIO thread's parallel KLT. cv::FAST and solvePnPRansac are serial.
//  - Output: the VIO / prediction pose (drone control) is NOT changed. The correction is published separately as a
//    map-frame transform T_map_odom (yaw about gravity + translation): p_map = Rz(yaw) p_odom + t. The published value
//    follows the optimiser's target with a rate limit (MapOdom smoothing below), so the map-frame pose never jumps.
#pragma once
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "vocab.h"

namespace ovloop {

struct Cfg {
  std::string vocab;           // --loop-vocab FILE (required with --loop 1)
  std::string log;             // --loop-log FILE (JSON lines per keyframe / loop / optimisation; "" = none)
  // VPR interface (user decision 10-26: learned global descriptors BoQ / SALAD next to the BoW baseline)
  std::string dump;            // --loop-dump DIR: write each keyframe image (kf_<t_ns>.pgm) + index.csv (offline VPR)
  std::string vpr;             // --loop-vpr FILE: precomputed global descriptors per keyframe t_ns (offline PC
                               // evaluation): candidates = cosine similarity instead of BoW (verification unchanged)
  double vpr_min = 0.5;        // cosine similarity lower bound for a VPR candidate
  // ovxfeat 2026-10-26 (user decision: XFeat + LighterGlue in the geometric verification). --loop-xf "CMD" starts a
  // feature / matching server (runs/ovxfeat_20261026/xf_server.py, PC prototype) as a child process; candidates stay
  // BoW (or VPR). xf_mode 1 = XFeat mutual nearest neighbour, 2 = LighterGlue. "" / xf_mode 0 = box-BRIEF (unchanged).
  std::string xf;
  int xf_mode = 0;
  int xf_tri = 0;              // 1: triangulate XFeat matches with the previous keyframe (VIO poses) -> more 3D points
  double xf_tri_deg = 1.0;     // minimum parallax for xf_tri
  double xf_tri_px = 2.0;      // reprojection bound (px) for xf_tri
  // xfopt 2026-10-27 (all default off = ovxfeat behaviour unchanged)
  int log_cand = 0;            // 1: log every verification path result with its pose ("cx" in the kf line; decisions unchanged)
  int xf_both = 0;             // 1 (with xf_mode): also run the box-BRIEF path for each candidate (logged)
  int xf_pick = 0;             // 0: XFeat result; 1: hybrid = box-BRIEF result when it passes, else XFeat
  int xf_min_inl = 0;          // XFeat path gates (0 = off / the shared value): inliers, inliers / matches, 8x6 grid cells,
  double xf_min_ratio = 0.0;   //   roll-pitch bound (deg), yaw correction bound (deg), PnP RANSAC threshold (px)
  int xf_min_cells = 0;
  double xf_rp_deg = 0.0;
  double xf_yaw_deg = 0.0;
  double xf_pnp_px = 0.0;
  int xf_tri_mv = 0;           // 1: also triangulate with the keyframe before the previous one; a point seen from both must agree
                               //    (|X1 - X2| <= xf_tri_rel * depth); 2: keep only points triangulated from both
  double xf_tri_rel = 0.05;
  double xf_max_tri_frac = 0.0;  // XFeat path gate: reject when triangulated-3D inliers > this share of the inliers (0 = off)
  double xf_kf_every = 0.0;    // lgopt: verify loop candidates only when >= this many s after the last verified-candidate keyframe (0 = every keyframe)
  int xf_tri_mode = 0;         // lgopt 2026-10-27: matcher for the xf_tri triangulation with the previous keyframe(s)
                               //   (0 = xf_mode; 1 = mutual NN while loop candidates use xf_mode 2 = LighterGlue)
  int consist_n = 1;           // temporal consistency: earlier consistent verified loops needed (1 = ovloop rule)
  // appnpu 2026-10-28: --loop-xf unix:NAME connects to a feature / matching server that already runs (the phone app's NPU
  // server, abstract socket when NAME starts with '@') instead of starting CMD as a child. Same request protocol. In this
  // mode a server that cannot be reached / dies / does not answer within xf_timeout_ms is NOT fatal: XFeat is off for the
  // rest of the run and verification falls back to the box-BRIEF path (logged "xf_down"). CMD mode is unchanged.
  int xf_timeout_ms = 3000;
  int diag_prev = 0;           // 1: also verify against the previous keyframe (log only, never a loop): pipeline self-test
  int sync = 0;                // --loop-sync 1: the VIO thread waits until each keyframe is processed (deterministic, PC eval only)
  // keyframe rule (VIO thread)
  double kf_min_dt = 0.25;     // s between keyframes at least (<= 4 Hz)
  double kf_trans = 0.08;      // m moved since the last keyframe, or
  double kf_rot_deg = 8.0;     // deg rotated since the last keyframe
  double kf_max_dt = 2.0;      // s: a keyframe anyway (slow motion) if moved >= kf_trans / 4
  int queue_max = 6;           // keyframes waiting for the loop thread; more -> the new one is dropped (counted)
  int nice = 10;               // loop thread nice value (Linux / Android per-thread priority)
  // features
  int n_feat = 600;            // keypoints for BoW + matching (all levels)
  int levels = 3;              // pyramid levels (scale 1/1.4 per level)
  int fast_th = 20;
  // candidates
  double min_gap_s = 3.0;      // a candidate keyframe must be older than this (s)
  double min_path_m = 1.0;     // and the VIO path since it must be at least this (m): no 'loops' with the recent past
  double min_score = 0.015;    // BoW L1 score lower bound
  double rel_score = 0.25;     // and >= rel_score * score(current, previous keyframe)
  int max_cand = 3;            // geometric checks per keyframe at most
  // geometric verification
  int match_dist = 64;         // Hamming bound (256 bit)
  double ratio = 0.8;          // best / second-best
  int min_inliers = 25;
  double pnp_px = 3.0;         // RANSAC reprojection threshold (px)
  int pnp_iters = 200;
  double max_rp_deg = 3.0;     // roll / pitch of the PnP pose vs the VIO gravity
  double max_corr_m = 0.5;     // implied drift |p_loop - p_vio| <= max_corr_m + max_corr_rel * path between
  double max_corr_rel = 0.05;
  double max_corr_yaw_deg = 10.0;
  // temporal consistency: a verified loop is accepted only when another verified loop within consist_kf keyframes
  // (same place: old keyframes within consist_old_s) implies the same drift correction (consist_m, consist_yaw_deg)
  int consist_kf = 3;
  double consist_old_s = 4.0;
  double consist_m = 0.05;
  double consist_yaw_deg = 1.5;
  // pose graph
  int seq_edges = 4;           // sequential edges to the previous N keyframes
  double loop_w = 1.0;         // loop edge weight
  double huber = 0.1;
  // weight_mode 1: residuals divided by a per-edge sigma (sequential: VIO relative accuracy grows with the distance;
  // loop: PnP accuracy), Huber on loop edges in sigma units (huber_sig). 0 = unit weights (VINS-Mono style).
  int weight_mode = 0;
  double seq_sig_m = 0.002, seq_sig_rel = 0.01, seq_sig_deg = 0.05, seq_sig_deg_m = 0.5;  // sigma_t = a + rel*d, yaw = b + c*d
  double loop_sig_m = 0.01, loop_sig_deg = 0.5;
  double huber_sig = 2.0;
  int opt_iters = 30;
  // output smoothing of T_map_odom (published follows the target): the map-frame position of the CURRENT body moves at
  // most smooth_v m/s and the yaw at most smooth_w_deg deg/s faster than the VIO motion itself
  double smooth_v = 0.10;
  double smooth_w_deg = 3.0;
  bool parse(const std::string &kv, std::string &err);  // "k=v,k=v" (--loop-cfg)
  std::string describe() const;
};

struct TrackIn {
  size_t id = 0;
  float u = 0, v = 0;          // pixel in this keyframe image (as tracked)
  bool slam = false;
  Eigen::Vector3d p_G = Eigen::Vector3d::Zero();     // SLAM landmark (VIO world)
  std::vector<double> obs_t;   // past observations (cam 0): clone time + undistorted normalised coordinate
  std::vector<float> obs_x, obs_y;
};

// one keyframe as copied by the VIO thread
struct KfIn {
  int64_t t_ns = 0;
  double t = 0.0;              // camera time (s, bridge clock)
  int session = 0;
  cv::Mat img;                 // 8-bit gray, owned copy
  Eigen::Matrix3d R_ItoG = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ItoC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinC = Eigen::Vector3d::Zero();
  double fx = 0, fy = 0, cx = 0, cy = 0;
  std::vector<double> dist;    // distortion (radtan k1 k2 p1 p2 or equidistant k1..k4); all 0 = pinhole
  bool fisheye = false;
  std::vector<TrackIn> tracks;
  std::vector<double> clone_t;  // clone poses (R_GtoI row-major 9, p_IinG 3)
  std::vector<std::array<double, 12>> clone_x;
  double cap_ms = 0.0;          // VIO-thread copy time
};

struct MapOdom {
  bool valid = false;           // at least one optimisation in this session
  double yaw = 0.0;             // published (smoothed)
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double tgt_yaw = 0.0;         // optimiser target
  Eigen::Vector3d tgt_t = Eigen::Vector3d::Zero();
  int n_loop = 0, n_kf = 0, session = 0;
  uint32_t ver = 0;             // optimisations so far (whole run)
  bool converging = false;      // published != target
};

// keyframe rule (VIO thread, cheap)
class KfRule {
public:
  explicit KfRule(const Cfg &c) : c_(c) {}
  bool want(double t, const Eigen::Matrix3d &R_ItoG, const Eigen::Vector3d &p);
  void reset() { have_ = false; }

private:
  const Cfg &c_;
  bool have_ = false;
  double t_ = 0.0;
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
};

class LoopCloser {
public:
  explicit LoopCloser(const Cfg &c);
  ~LoopCloser();
  bool ok() const { return ok_; }
  const std::string &err() const { return err_; }
  const Cfg &cfg() const { return c_; }
  KfRule &rule() { return rule_; }
  // VIO thread
  void push(std::unique_ptr<KfIn> kf);
  void reset_session(const char *why);  // VIO re-initialisation: new world frame -> new graph / database
  int session() const { return session_; }
  // output: the main thread steps the smoothing once per frame (frame time, VIO position) and gets the published value
  MapOdom step_output(double t, const Eigen::Vector3d &p_vio);
  MapOdom published() const;
  void finish(double timeout_s);  // wait until the queued keyframes are processed (end of input)
  void report() const;  // stderr summary (call after finish)

private:
  struct Kf;
  struct Edge;
  struct Verified;
  void run();
  void process(KfIn &in);
  bool verify(const Kf &cur, const Kf &old, Verified &out, std::string &why);
  void optimise();
  void log_line(const std::string &s);
  // --loop-xf server (loop thread only)
  bool xf_start(std::string &err);
  bool xf_feat(int64_t id, const cv::Mat &img, const std::vector<cv::Point2f> &q, std::vector<cv::Point2f> &kps, float &ms);
  bool xf_match(int64_t a, int64_t b, int mode, std::vector<std::array<int, 2>> &m, float &ms);
  int xf_in_ = -1, xf_out_ = -1;  // write to server stdin / read from its stdout
  int xf_pid_ = -1;
  bool xf_ok_ = false;
  bool xf_unix_ = false;          // appnpu: --loop-xf unix:NAME (one socket fd = xf_in_ = xf_out_)
  void xf_down(const char *why);  // appnpu: unix mode, server lost -> BRIEF fallback for the rest of the run

  Cfg c_;
  bool ok_ = true;
  std::string err_;
  Vocab voc_;
  int vpr_dim_ = 0;
  std::map<int64_t, std::vector<float>> vpr_;  // t_ns -> L2-normalised global descriptor (--loop-vpr)
  KfRule rule_;
  std::thread th_;
  mutable std::mutex qm_;
  std::condition_variable qcv_, done_cv_;
  std::deque<std::unique_ptr<KfIn>> q_;
  bool stop_ = false;
  uint64_t n_pushed_ = 0, n_done_ = 0, n_dropped_ = 0;
  int session_ = 0;
  std::atomic<int> reset_req_{0};
  // loop-thread state
  struct State;
  std::unique_ptr<State> s_;
  // output state (main thread writes, prediction thread reads)
  mutable std::mutex om_;
  MapOdom out_;
  double out_last_t_ = -1e300;
  bool out_have_t_ = false;
  Eigen::Vector3d out_last_p_ = Eigen::Vector3d::Zero();
  FILE *log_ = nullptr;
  std::mutex logm_;
};

}  // namespace ovloop
