// ovopt.cpp — option storage + parser + profiler slots for ovpatch/ovopt.h (2026-10-25).
#include "ovopt.h"

#include <cstdlib>
#include <sstream>

namespace ovopt {

Flags g;
double prof_ms[N_SLOTS] = {0};
long prof_cnt[N_COUNTS] = {0};

namespace {
struct Field {
  const char *name;
  int Flags::*ptr;
  bool exact;
};
const Field kFields[] = {
    {"skip_retri", &Flags::skip_retri, true}, {"par_tri", &Flags::par_tri, true},
    {"par_jac", &Flags::par_jac, true},       {"par_ma", &Flags::par_ma, true},
    {"cov_move", &Flags::cov_move, true},     {"frame_nocopy", &Flags::frame_nocopy, true},
    {"givens_rm", &Flags::givens_rm, true},
    {"ekf_gemm", &Flags::ekf_gemm, false},
    {"qr_compress", &Flags::qr_compress, false}, {"ekf_chol", &Flags::ekf_chol, false},
    {"par_cov", &Flags::par_cov, false},      {"prof", &Flags::prof, false},
};
} // namespace

bool parse(const std::string &list, std::string &err) {
  std::stringstream ss(list);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty() || tok == "none" || tok == "0")
      continue;
    std::string name = tok;
    int val = 1;
    auto eq = tok.find('=');
    if (eq != std::string::npos) {
      name = tok.substr(0, eq);
      val = std::atoi(tok.c_str() + eq + 1);
    }
    if (name == "all_exact") {
      for (const auto &f : kFields)
        if (f.exact)
          g.*(f.ptr) = val;
      continue;
    }
    bool found = false;
    for (const auto &f : kFields) {
      if (name == f.name) {
        g.*(f.ptr) = val;
        found = true;
        break;
      }
    }
    if (!found) {
      err = "unknown --ovopt option '" + name + "'";
      return false;
    }
  }
  return true;
}

std::string describe() {
  std::ostringstream o;
  bool first = true;
  for (const auto &f : kFields) {
    o << (first ? "" : " ") << f.name << "=" << g.*(f.ptr);
    first = false;
  }
  return o.str();
}

const char *slot_name(int s) {
  static const char *n[N_SLOTS] = {"track",     "prop",      "select",     "msckf_clean", "msckf_tri", "msckf_jac",
                                   "msckf_comp", "msckf_ekf", "slam_marg",  "slam_jac",    "slam_ekf",  "delay_tri",
                                   "delay_init", "retri",     "db_clean",   "anchor",      "marg_clone", "ekf_ma",
                                   "ekf_s",      "ekf_k",     "ekf_cov",    "ekf_sym",     "ekf_dx"};
  return (s >= 0 && s < N_SLOTS) ? n[s] : "?";
}
const char *count_name(int c) {
  static const char *n[N_COUNTS] = {"ekf_calls", "ekf_rows", "cov_n", "msckf_feats", "msckf_rows", "msckf_cols", "slam_up", "slam_delay"};
  return (c >= 0 && c < N_COUNTS) ? n[c] : "?";
}
void prof_reset() {
  for (int i = 0; i < N_SLOTS; i++)
    prof_ms[i] = 0.0;
  for (int i = 0; i < N_COUNTS; i++)
    prof_cnt[i] = 0;
}

} // namespace ovopt
