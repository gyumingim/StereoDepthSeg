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
  // --- 2026-10-26 ovupd (NOT in all_exact; default off = code path above unchanged) ---
  int ekf_sparse = 0;  // near-exact. StateHelper::EKFUpdate: when H is block-sparse (SLAM chunks: each feature row touches
                       // only its own landmark + 1-2 clones + calibration), M = P H^T is built from the non-zero H blocks
                       // only (per H variable: P(:,var) * H(rows,var)^T) and S = H (P_small H^T) + R reuses the M rows of
                       // the H variables (no P_small, no H P_small H^T). Dense H (MSCKF after compression, delayed init)
                       // falls back to the other options. Value = max fill ratio x100 for the sparse path (1 = 50).
  int init_gemm = 0;   // near-exact. StateHelper::initialize_invertible (new SLAM landmark): M = P(:,H vars) H_R^T as one GEMM
                       // on gathered columns (not per state variable x H variable), H_R P_small H_R^T = H_R * (rows of M)
  int sym_inplace = 0; // exact. StateHelper::EKFUpdate: P = selfadjoint(P) as an in-place copy of the upper triangle
                       // into the lower one (same values, no N x N temporary)
  // --- changes results (2026-10-25, branch ovdrone-20261025) ---
  int rs_derot = 0;    // VioManager: gyro rolling-shutter de-rotation of each new feature measurement to the frame-centre
                       // row time. Value = sensor readout time first->last row in microseconds (signed, > 0 = top row of
                       // the fed image first); 1 = the live default 6540 us (S24 main lens 1920x1080 @30, meta first_result
                       // skew_ns 6539...). 0 = off (upstream code path).
  int rs_postinit = 0; // rs_derot only after the VIO is initialised (the initialiser sees raw measurements); 0 = always
  int rs_row_ppm = 0;  // rs_derot without a bridge row map (no --remap): fed-row fraction -> sensor-row fraction multiplier
                       // x 1e6 (undistort crop: fy_raw*H_fed/(fy_fed*H_raw)); 0 = 1.0. Ignored when the bridge set rs_map.
  // --- changes results (2026-10-26, branch ovgyro-20261026) ---
  int gyro_klt = 0;    // TrackKLT mono temporal KLT start points moved by the gyro rotation between the frames (VioManager
                       // integrates bias-corrected gyro over the frame gap, R_ItoC, t_d); 2 = also re-track LK failures from
                       // the upstream start point. 0 = off (upstream results; only output counters run). See gyro_klt.h.
  // --- profiling ---
  int prof = 0;        // per-stage timers (ov_bridge --prof-log FILE)
  // --- behaviour (2026-10-25, branch ovrobust; NOT part of --ovopt / all_exact; set by ov_bridge --init-still) ---
  int init_still = 0;  // VioManagerHelper::try_to_initialize: wait_for_jerk = false with ZUPT off (static init while still)
};
extern Flags g;

// ---- rs_derot (2026-10-25): row map + statistics ----
// When ov_bridge remaps the frames (--remap), it points rs_mapy at the map's y table (fed w x h floats = source row of
// each fed pixel) and rs_src_h at the source (sensor) image height: the row time then uses the exact sensor row of the
// feature (lens distortion included). Otherwise row fraction = rs_row_ppm/1e6 * (v/H - 0.5).
extern const float *rs_mapy;
extern int rs_map_w, rs_map_h, rs_src_h;
struct RsStats {
  long frames = 0, feats = 0, no_imu = 0;
  double sum_shift = 0.0, max_shift = 0.0, sum_w = 0.0, max_w = 0.0;
};
extern RsStats rs_stats;
inline double rs_readout_s() { return g.rs_derot == 1 ? 6540e-6 : g.rs_derot * 1e-6; }
// ---- health counters (2026-10-25, branch ovrobust; ov_bridge --health / --health-log / --recover) ----
// Always counted (a few integer adds per frame, no floating point, no state access): which features went into each
// update and how many survived. ov_bridge resets them before every fed frame and reads them after it. Only the serial
// parts of the updaters touch them (entry / after triangulation / after the chi2 loop), so par_* options do not race.
struct Health {
  long tracked = -1;    // VioManager: trackFEATS ids on cam0 after feed_new_camera (-1 = not set this frame)
  long msckf_in = 0;    // UpdaterMSCKF::update: features in
  long msckf_tri = 0;   //   after clean + triangulation (= candidates for the chi2 test)
  long msckf_used = 0;  //   after the chi2 test (stacked into the update)
  long slam_in = 0;     // UpdaterSLAM::update: SLAM features in (all chunks)
  long slam_clean = 0;  //   after measurement cleaning (= chi2 candidates)
  long slam_used = 0;   //   after the chi2 test
  long delay_in = 0;    // UpdaterSLAM::delayed_init: features in
  long delay_ok = 0;    //   initialized as SLAM landmarks
};
extern Health h;
inline void health_reset() { h = Health(); }

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
  S_RS,            // rs_derot feature de-rotation (2026-10-25)
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
  C_SPARSE_TRY,     // ekf_sparse: EKFUpdate calls checked (2026-10-26 ovupd)
  C_SPARSE_USED,    //   calls that took the sparse path
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
