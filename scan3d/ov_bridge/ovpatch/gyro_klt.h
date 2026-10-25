// gyro_klt.h — scan3d gyro-aided KLT (2026-10-26, branch ovgyro-20261026).
//
// Storage lives in ovpatch/TrackKLT.cpp (that file is in every build: PC lib build, PC full build, phone, klt_bench), so
// this header has no other dependency. Default everything 0 = upstream TrackKLT behaviour (no prediction, no extra work).
//
// mode (set by VioManager from ovopt::g.gyro_klt, ov_bridge --ovopt gyro_klt=N; CHANGES results when > 0):
//   1 = before the temporal KLT of a frame, move each tracked point's initial guess to where the pure camera rotation
//       between the last and the current frame (integrated bias-corrected gyro, R_ItoC, t_d) maps it:
//       p1 = distort(n(R_C1C0 * [undistort(p0); 1])). The pyramidal LK search itself is unchanged (same window, levels,
//       criteria, OPTFLOW_USE_INITIAL_FLOW as upstream), only its start point differs. Translation / depth ignored.
//   2 = 1 + points whose LK failed (status 0) from the rotated guess are tracked again from the upstream guess (old
//       position) and kept when that succeeds (RANSAC afterwards as upstream). Extra cost only for failed points.
//   3 = 1 + camera translation once the VIO is initialised (state velocity x frame gap, depth from SLAM landmarks:
//       own landmark by feature id, else the median inverse depth). Added 2026-10-26 after the PC check showed that pure
//       rotation mispredicts hand-held motion where translation and rotation cancel (raw80: start-point error > flow).
//   4 = 3 + the fallback of mode 2.
//   5 = 3 with guards (2026-10-26, after the coordinator's verifier): translation only when the bridge health of the last
//       frame is OK (or health is off) and |v| <= 3 m/s; per point, when the translation part alone would move the start
//       point by > 60 px (about f x change of the normalised coordinates) the rotation-only (gyro) prediction is used.
//       (A first version capped the whole predicted shift at 30 px; that also removed correct predictions in fast
//       rotation - raw80 start error 1.8 -> 2.75 px - so the cap applies to the filter-dependent translation part only.
//       Cap scan on raw80 / harness (start error px): 30 -> 4.40 / 1.17, 60 -> 1.83 / 1.12, 100+ = mode 3 (1.81 / 1.12);
//       verifier: a +1 m/s wrong velocity gives ~32 px, +3 m/s ~65 px mean start error.)
// Counters (always counted when a frame is tracked; read + reset by ov_bridge per fed frame for --klt-stats 1; output only).
#pragma once

#include <cstdlib>
#include <unordered_map>

#include <Eigen/Core>

namespace scan3d_gyro {
struct Pred {
  int mode = 0;                                   // see above (0 = off)
  bool have = false;                              // VioManager set R for the frame being tracked now
  Eigen::Matrix3d R_C1C0 = Eigen::Matrix3d::Identity();  // current camera <- last camera (rotation only)
  double dt = 0.0;                                // frame gap used (s)
  int n_imu = 0;                                  // gyro intervals inside the gap
  double held = 0.0;                              // s at the end of the gap with no gyro yet (last sample held)
  // mode 3/4 (translation, only once initialised): X_C1 = R_C1C0 X_C0 + t_C1 with t_C1 = -R_C1C0 R_GtoC0 v_G dt (state
  // velocity at the last update, constant over the gap); inverse depth of a point = its SLAM landmark's (by feature id)
  // or else the median over the SLAM landmarks in front of the last camera.
  bool have_t = false;
  Eigen::Vector3d t_C1 = Eigen::Vector3d::Zero();
  double invz_med = 0.0;
  std::unordered_map<size_t, double> invz;        // feature id -> 1/Z in the last camera (SLAM landmarks)
  bool health_ok = true;                          // set by ov_bridge before each frame: health off or last state OK
};
inline bool fallback_on(int mode) { return mode == 2 || mode == 4; }
inline bool guards_on(int mode) { return mode == 5; }
constexpr double kGuardShiftPx = 60.0;  // mode 5: max start-point shift from the translation part
// env OV_GK_CAP_PX overrides kGuardShiftPx (study knob, read once; unset = constant)
inline double guard_shift_px() {
  static const double v = [] { const char *e = std::getenv("OV_GK_CAP_PX"); return e ? std::atof(e) : kGuardShiftPx; }();
  return v;
}
constexpr double kGuardVmax = 3.0;      // mode 5: max state speed (m/s) for the translation term
struct Stats {
  long frames = 0;    // temporal KLT calls (mono)
  long pts_in = 0;    // points handed to temporal KLT
  long klt_ok = 0;    // LK status 1 (after the fallback in mode 2)
  long pts_out = 0;   // kept after LK + RANSAC + bounds / mask
  long fb_try = 0;    // mode 2: points re-tracked from the old position
  long fb_ok = 0;     //   of which LK succeeded
  double shift_sum = 0.0;  // sum of |predicted - old| px over predicted points
  long shift_n = 0;
  double shift_max = 0.0;
  long no_rot = 0;    // frames tracked in mode > 0 without a rotation (no IMU / first frame)
  long capped = 0;    // mode 5: points whose translation part exceeded the cap (rotation-only prediction used)
  long t_blocked = 0; // mode 5: frames where the translation term was refused (health not OK or |v| too large)
  double res_sum = 0.0;   // sum |LK result - LK start| px (first LK call, successes) = how far LK still had to move
  double flow_sum = 0.0;  // sum |LK result - old position| px (same points) = image motion
  long res_n = 0;
  double pdx = 0.0, pdy = 0.0;  // sums of predicted shift vectors (over shift_n points)
  double fdx = 0.0, fdy = 0.0;  // sums of LK flow vectors (over res_n points)
};
extern Pred pred;
extern Stats frame_stats;  // this fed frame (ov_bridge resets it)
extern Stats total;        // whole run
inline void add(Stats &a, const Stats &b) {
  a.frames += b.frames; a.pts_in += b.pts_in; a.klt_ok += b.klt_ok; a.pts_out += b.pts_out; a.fb_try += b.fb_try;
  a.fb_ok += b.fb_ok; a.shift_sum += b.shift_sum; a.shift_n += b.shift_n; a.no_rot += b.no_rot;
  a.res_sum += b.res_sum; a.flow_sum += b.flow_sum; a.res_n += b.res_n;
  a.pdx += b.pdx; a.pdy += b.pdy; a.fdx += b.fdx; a.fdy += b.fdy;
  a.capped += b.capped; a.t_blocked += b.t_blocked;
  if (b.shift_max > a.shift_max) a.shift_max = b.shift_max;
}
}  // namespace scan3d_gyro
