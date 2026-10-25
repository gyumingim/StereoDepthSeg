// ovopt.h — scan3d speed options for the patched OpenVINS copies in ovpatch/ (2026-10-25, branch ovopt-20261025).
//
// Every option is OFF by default (all zero) and then the patched files run the upstream code path unchanged.
// ov_bridge sets them from its command line (--ovopt LIST, --prof-log FILE) before it creates the VioManager.
// The options only take effect in builds that compile the OpenVINS sources together with ovpatch/ (phone build
// android/CMakeLists.txt, PC build with -DOV_FULL_SOURCE=ON); the PC build that links libov_msckf_lib.so has only the
// TrackKLT patch and ov_bridge refuses --ovopt there (OVOPT_FULL not defined).
//
// "exact" options keep every floating point operation of upstream (same code, same order per feature / per block),
// they only skip output that nothing reads or run independent per-feature / per-block work on several threads.
// Checked on the PC (x86) identity harness: pose lines identical (all fields; 9 digits, and 17 digits with
// ov_bridge --pose-precision 17). NOT yet checked on the phone (ARM) - see runs/ovopt_20261025/README.md phone plan.
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <opencv2/core/utility.hpp>

namespace ovopt {

struct Flags {
  // --- exact ---
  int skip_retri = 0;  // VioManager: skip retriangulate_active_tracks() (only feeds get_active_tracks/get_active_image = ROS viz)
  int par_tri = 0;     // UpdaterMSCKF::update + UpdaterSLAM::delayed_init: triangulate + Gauss-Newton features in parallel
  int par_jac = 0;     // UpdaterMSCKF::update + UpdaterSLAM::update: per-feature Jacobian / nullspace / chi2 in parallel
  int par_ma = 0;      // StateHelper::EKFUpdate: M = P*H^T computed per state variable block in parallel
  int par_s = 0;       // StateHelper::EKFUpdate: P_small + S = H P H^T + R as one more task next to M (needs par_ma or ekf_gemm)
  int cov_move = 0;    // StateHelper::marginalize: move (not copy) the new covariance matrix
  int frame_nocopy = 0;  // ov_bridge: remap a frame straight from the received record (no 2 MB copy first)
  int givens_rm = 0;   // UpdaterMSCKF: measurement compression Givens loop on a row-major copy (same rotations)
  // --- near-exact (different rounding, same math) ---
  int ekf_gemm = 0;    // StateHelper::EKFUpdate: M = P(:,H vars)*H^T as one GEMM on gathered columns (not per block)
  int qr_compress = 0; // UpdaterMSCKF: measurement compression with Householder QR instead of Givens rotations
  int ekf_chol = 0;    // StateHelper::EKFUpdate: W = M U^-1 (S = U^T U), P -= W W^T, dx = W U^-T r (no S^-1, no K)
  int par_cov = 0;     // StateHelper::EKFUpdate: covariance update split in 2 balanced parts on the OpenCV pool
  // --- profiling ---
  int prof = 0;        // per-stage timers (ov_bridge --prof-log FILE)
};
extern Flags g;

// parse "a,b,c" (names of the int fields above, "name=N" for a value, "all_exact" = every exact option);
// returns false and sets err on an unknown name
bool parse(const std::string &list, std::string &err);
std::string describe();  // "skip_retri=1 par_tri=0 ..."

// Run f(i) for i in [0, n). When on (and n > 1) the calls are spread over the OpenCV thread pool (cv::parallel_for_,
// TBB on the phone); each i must only write its own output slot, so the result does not depend on the thread split.
template <class F> inline void par_for(bool on, int n, const F &f) {
  if (!on || n < 2 || cv::getNumThreads() < 2) {
    for (int i = 0; i < n; i++)
      f(i);
    return;
  }
  cv::parallel_for_(
      cv::Range(0, n),
      [&](const cv::Range &r) {
        for (int i = r.start; i < r.end; i++)
          f(i);
      },
      n);
}

// ---- profiler: per-frame stage times (ms) + counters, reset by ov_bridge after each fed frame ----
enum Slot : int {
  S_TRACK = 0,     // trackFEATS->feed_new_camera (+ aruco)
  S_PROP,          // propagate_and_clone
  S_SELECT,        // feature selection / sorting before the MSCKF update
  S_MSCKF_CLEAN,   // UpdaterMSCKF::update step 1-2 (clean measurements, clone poses)
  S_MSCKF_TRI,     // step 3 triangulation + Gauss-Newton
  S_MSCKF_JAC,     // step 4 Jacobians + nullspace + chi2 + stacking
  S_MSCKF_COMP,    // step 5 measurement compression
  S_MSCKF_EKF,     // step 6 EKFUpdate
  S_SLAM_MARG,     // StateHelper::marginalize_slam
  S_SLAM_JAC,      // UpdaterSLAM::update linear system (all chunks)
  S_SLAM_EKF,      // UpdaterSLAM::update EKFUpdate (all chunks)
  S_DELAY_TRI,     // UpdaterSLAM::delayed_init clean + triangulation + Gauss-Newton
  S_DELAY_INIT,    // UpdaterSLAM::delayed_init Jacobians + StateHelper::initialize
  S_RETRI,         // retriangulate_active_tracks
  S_DB_CLEAN,      // feature database cleanup + cleanup_measurements
  S_ANCHOR,        // UpdaterSLAM::change_anchors
  S_MARG_CLONE,    // StateHelper::marginalize_old_clone
  S_EKF_MA,        // EKFUpdate parts (all callers): M = P*H^T
  S_EKF_S,         //   S = H*P_small*H^T + R
  S_EKF_K,         //   llt + K = M*S^-1
  S_EKF_COV,       //   P -= K*M^T (upper)
  S_EKF_SYM,       //   P = selfadjoint(P) + diagonal check
  S_EKF_DX,        //   dx = K*res + state update
  N_SLOTS
};
enum Count : int {
  C_EKF_CALLS = 0,  // EKFUpdate calls
  C_EKF_ROWS,       // sum of EKFUpdate rows
  C_COV_N,          // covariance size at the end of the frame
  C_MSCKF_FEATS,    // features into UpdaterMSCKF::update
  C_MSCKF_ROWS,     // stacked rows before compression
  C_MSCKF_COLS,     // stacked state columns
  C_SLAM_UP,        // SLAM features into UpdaterSLAM::update (all chunks)
  C_SLAM_DELAY,     // features into delayed_init
  N_COUNTS
};
extern double prof_ms[N_SLOTS];
extern long prof_cnt[N_COUNTS];
const char *slot_name(int s);
const char *count_name(int c);
void prof_reset();

inline double now_ms() {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Scope {  // adds the scope's wall time to slot s when profiling is on
  int s;
  double t0;
  explicit Scope(int slot) : s(slot), t0(g.prof ? now_ms() : 0.0) {}
  ~Scope() {
    if (g.prof)
      prof_ms[s] += now_ms() - t0;
  }
};
struct Lap {  // consecutive stages: to(s) adds the time since the previous lap point to slot s
  double t;
  Lap() : t(g.prof ? now_ms() : 0.0) {}
  void to(int s) {
    if (g.prof) {
      double n = now_ms();
      prof_ms[s] += n - t;
      t = n;
    }
  }
};
inline void count(int c, long v) {
  if (g.prof)
    prof_cnt[c] += v;
}
inline void count_set(int c, long v) {
  if (g.prof)
    prof_cnt[c] = v;
}

} // namespace ovopt
