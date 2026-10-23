#!/usr/bin/env python3
"""Patch a COPY of OpenVINS (open_vins_feat) to log per-feature update outcomes to $OV_FEATLOG.<suffix>.
Line: kind t featid status u v pred_u pred_v t_lastobs chi2 thr
kind M = MSCKF update, S = SLAM update, N = new SLAM (delayed init); status A accepted, R chi2-rejected,
T triangulation/refine failed. pred = measured - residual of the newest observation (before nullspace projection).
Logging only reads values; estimates are unchanged."""
import sys
from pathlib import Path

root = Path(sys.argv[1]) / "ov_msckf" / "src" / "update"
HELP = r'''
// ---- feature log (open_vins_feat copy, 2026-10-23) ----
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
namespace {
FILE *featlog_file_(const char *suf) {
  static FILE *f = nullptr;
  static bool tried = false;
  if (!tried) {
    tried = true;
    const char *e = std::getenv("OV_FEATLOG");
    if (e && *e) {
      std::string p = std::string(e) + suf;
      f = std::fopen(p.c_str(), "w");
      if (f) std::setvbuf(f, nullptr, _IOLBF, 1 << 16);
    }
  }
  return f;
}
void featlog_(const char *suf, char kind, double t, const std::shared_ptr<ov_core::Feature> &fp, char st, const Eigen::VectorXd *res,
              double chi2, double thr) {
  FILE *f = featlog_file_(suf);
  if (!f) return;
  auto it = fp->uvs.find(0);
  auto jt = fp->timestamps.find(0);
  if (it == fp->uvs.end() || it->second.empty() || jt == fp->timestamps.end() || jt->second.empty()) return;
  const auto &uv = it->second.back();
  size_t n = it->second.size();
  double pu = NAN, pv = NAN;
  if (res && (size_t)res->rows() >= 2 * n) {
    pu = uv(0) - (*res)(2 * n - 2);
    pv = uv(1) - (*res)(2 * n - 1);
  }
  std::fprintf(f, "%c %.6f %zu %c %.2f %.2f %.2f %.2f %.6f %.3f %.3f\n", kind, t, fp->featid, st, (double)uv(0), (double)uv(1), pu, pv,
               jt->second.back(), chi2, thr);
}
} // namespace
// ---- end feature log ----
'''


def sub(s, old, new, count=1):
    n = s.count(old)
    if n != count:
        sys.exit(f"pattern found {n}x (want {count}): {old[:80]!r}")
    return s.replace(old, new)


# ---------------- MSCKF
p = root / "UpdaterMSCKF.cpp"
s = p.read_text()
s = sub(s, '#include <boost/math/distributions/chi_squared.hpp>\n',
        '#include <boost/math/distributions/chi_squared.hpp>\n' + HELP)
s = sub(s, """    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
""", """    if (!success_tri || !success_refine) {
      featlog_(".msckf", 'M', state->_timestamp, *it1, 'T', nullptr, NAN, NAN);
      (*it1)->to_delete = true;
""")
s = sub(s, """    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // Nullspace project
""", """    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);
    Eigen::VectorXd res0_ = res;

    // Nullspace project
""")
s = sub(s, """    if (chi2 > _options.chi2_multipler * chi2_check) {
      (*it2)->to_delete = true;
""", """    if (chi2 > _options.chi2_multipler * chi2_check) {
      featlog_(".msckf", 'M', state->_timestamp, *it2, 'R', &res0_, chi2, _options.chi2_multipler * chi2_check);
      (*it2)->to_delete = true;
""")
s = sub(s, """    // We are good!!! Append to our large H vector
""", """    featlog_(".msckf", 'M', state->_timestamp, *it2, 'A', &res0_, chi2, _options.chi2_multipler * chi2_check);
    // We are good!!! Append to our large H vector
""")
p.write_text(s)

# ---------------- SLAM
p = root / "UpdaterSLAM.cpp"
s = p.read_text()
s = sub(s, '#include <boost/math/distributions/chi_squared.hpp>\n',
        '#include <boost/math/distributions/chi_squared.hpp>\n' + HELP)
s = sub(s, """    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
""", """    if (!success_tri || !success_refine) {
      featlog_(".slam", 'N', state->_timestamp, *it1, 'T', nullptr, NAN, NAN);
      (*it1)->to_delete = true;
""")
s = sub(s, """    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);
""", """    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);
    Eigen::VectorXd res0_ = res;
""", count=2)
s = sub(s, """      state->_features_SLAM.insert({(*it2)->featid, landmark});
      (*it2)->to_delete = true;
      it2++;
    } else {
      (*it2)->to_delete = true;
""", """      featlog_(".slam", 'N', state->_timestamp, *it2, 'A', &res0_, NAN, chi2_multipler);
      state->_features_SLAM.insert({(*it2)->featid, landmark});
      (*it2)->to_delete = true;
      it2++;
    } else {
      featlog_(".slam", 'N', state->_timestamp, *it2, 'R', &res0_, NAN, chi2_multipler);
      (*it2)->to_delete = true;
""")
s = sub(s, """        landmark->update_fail_count++;
      }
      (*it2)->to_delete = true;
""", """        landmark->update_fail_count++;
      }
      featlog_(".slam", 'S', state->_timestamp, *it2, 'R', &res0_, chi2, chi2_multipler * chi2_check);
      (*it2)->to_delete = true;
""")
s = sub(s, """    // Debug print when we are going to update the aruco tags
""", """    featlog_(".slam", 'S', state->_timestamp, *it2, 'A', &res0_, chi2, chi2_multipler * chi2_check);
    // Debug print when we are going to update the aruco tags
""")
p.write_text(s)
print("patched", root)
