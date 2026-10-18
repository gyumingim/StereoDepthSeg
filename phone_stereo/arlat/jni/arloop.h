// arloop: loop closure on top of ARCore poses (phone, JNI) — marker and/or feature (no marker) versions.
//
// ARCore gives good short-term motion but drifts over a long walk. Each keyframe (ARCore pose + 640x480 gray CPU image,
// optionally ARCore depth) becomes a node of a 4-DoF pose graph (x, y, z, yaw; roll and pitch come from gravity and are
// trusted) — the VINS-Mono loop-closure formulation:
//   odometry edges  : relative ARCore motion between each keyframe and its 4 predecessors
//   marker edges    : ArUco marker seen from a keyframe (IPPE square PnP, known size). Markers are graph nodes, so
//                     seeing a marker again pulls both sightings onto one marker pose.          (Options::markers)
//   feature loops   : ORB features of a keyframe vs. earlier keyframes near the current estimate whose features have
//                     3D (triangulated from the previous keyframe with the ARCore poses, or ARCore depth if given)
//                     -> 3D-2D PnP RANSAC -> relative pose edge (Huber loss).                    (Options::features)
// After a new constraint the graph is solved with Ceres in the worker thread; the drift correction of the newest
// keyframe (yaw + translation) is blended in over blend_s seconds and applied to every ARCore pose by correct().
// Nothing here blocks the ARCore thread: add_keyframe() only queues.
//
// Frames: ARCore world is Y-up, ARCore camera is OpenGL (x right, y up, -z forward). Inside, world is Z-up
// (x, -z, y) and the camera is OpenCV (x right, y down, z forward) so yaw is rotation about +Z.
#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace arloop {

struct Options {
  bool markers = true;
  bool features = true;
  int marker_dict = 0;          // cv::aruco::DICT_4X4_50
  double marker_len = 0.15;     // m, printed black square side
  int orb_features = 600;
  int min_inliers = 30;         // PnP inliers for a feature loop (and >= 35 % of the matches)
  double loop_radius = 4.0;     // m, candidate keyframes within this distance of the current estimate
  double min_gap_s = 8.0;       // candidates at least this much older (not the keyframes we just passed)
  double max_jump = 3.0;        // m, reject a loop whose implied correction is larger (false match guard)
  double blend_s = 1.5;         // s, correction is blended in over this time (no pose jumps)
  bool threaded = true;         // false: process in add_keyframe (offline replay)
};

struct KeyframeIn {
  int64_t t_ns = 0;
  Eigen::Vector3d t_gl;         // ARCore camera pose (world Y-up, camera OpenGL)
  Eigen::Quaterniond q_gl;
  cv::Mat gray;                 // CV_8UC1 (CPU image, sensor orientation)
  double fx = 0, fy = 0, cx = 0, cy = 0;
  cv::Mat depth_mm;             // CV_16UC1 ARCore depth (any size, same field of view) or empty
};

struct Pose4 {                  // world correction: p' = Rz(yaw) p + t
  double yaw = 0;
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

class LoopCloser {
 public:
  explicit LoopCloser(const Options &o);
  ~LoopCloser();
  void add_keyframe(KeyframeIn kf);
  // ARCore pose -> drift-corrected pose (same ARCore frames)
  void correct(int64_t t_ns, const Eigen::Vector3d &t_gl, const Eigen::Quaterniond &q_gl, Eigen::Vector3d &t_out,
               Eigen::Quaterniond &q_out);
  std::string stats_json();

 private:
  struct KF {
    int id;
    int64_t t_ns;
    Eigen::Matrix3d R;          // measured (ARCore) camera->world, Z-up world, OpenCV camera
    Eigen::Vector3d t;
    double yaw, pitch, roll;
    double opt_t[3];
    double opt_yaw;
    double fx, fy, cx, cy;
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    cv::Mat desc3d;             // rows = features with depth
    std::vector<cv::Point3f> p3d;
  };
  struct Marker {
    int id;
    double opt_t[3];
    double opt_yaw;
    int sightings = 0;
    int64_t last_t_ns = 0;
  };
  struct Edge {
    int i;                      // keyframe index
    int j;                      // keyframe index, or marker id when marker = true
    bool marker;
    Eigen::Vector3d t_ij;       // R_i^T (t_j - t_i)
    double yaw_ij;
    double w_t, w_yaw;
    bool robust;
  };

  // a feature loop is accepted only when two keyframes within 10 of each other imply the same correction
  // (repeated patterns such as a marker grid made single loops 10-25 deg wrong in the offline test)
  struct Pending {
    int kf = -1;
    Edge e;
    Eigen::Vector3d dt;
    double dyaw;
  };
  std::vector<Pending> pending_;

  void worker();
  void process(KeyframeIn &in);
  void detect_markers(KF &kf, const cv::Mat &gray, std::vector<std::vector<cv::Point2f>> &corners);
  bool find_loop(KF &kf);
  void triangulate(KF &c, const KF &p);
  void optimize(const char *why);
  void apply_current(KF &kf) const;   // opt pose of a new keyframe from the measured one and the current correction
  Pose4 correction_of(const KF &kf) const;
  void event(const std::string &s);

  Options o_;
  cv::Ptr<cv::ORB> orb_;
  std::vector<std::unique_ptr<KF>> kfs_;
  std::map<int, Marker> markers_;
  std::vector<Edge> edges_;
  std::atomic<int> n_kf_{0}, n_markers_{0}, n_loops_{0}, n_marker_edges_{0}, n_opt_{0}, n_depth_kf_{0};
  std::atomic<double> last_opt_ms_{0}, proc_ms_{0};
  double last_loop_t_ = -1e9;
  std::deque<std::string> events_;

  // correction state (read by correct())
  std::mutex cmu_;
  Pose4 corr_prev_, corr_target_;
  int64_t corr_change_ns_ = 0;
  int64_t last_seen_ns_ = 0;

  // queue
  std::mutex qmu_;
  std::condition_variable qcv_;
  std::deque<KeyframeIn> q_;
  std::atomic<bool> stop_{false};
  std::thread th_;
  std::mutex smu_;   // stats/events
};

}  // namespace arloop
