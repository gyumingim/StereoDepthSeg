# ovbaro 2026-10-26: hooks for baro.h in ov_bridge/main.cpp (applied once; asserts each anchor is unique)
p = '/home/system/scanner/StereoDepthSeg-ovbaro/scan3d/ov_bridge/main.cpp'
s = open(p, encoding='utf-8').read()
def rep(a, b):
    global s
    assert s.count(a) == 1, (a, s.count(a))
    s = s.replace(a, b)
rep('#include "mav_out.h"  // --mav-udp drone output (2026-10-25, ovdrone)\n',
    '#include "mav_out.h"  // --mav-udp drone output (2026-10-25, ovdrone)\n#include "baro.h"     // --baro barometer-aided height (2026-10-26, ovbaro)\n')
rep('enum MsgType : uint32_t { kImu = 1, kStereo = 2, kReset = 3, kQuit = 4, kChunk = 5, kChunkEnd = 6 };',
    'enum MsgType : uint32_t { kImu = 1, kStereo = 2, kReset = 3, kQuit = 4, kChunk = 5, kChunkEnd = 6, kBaro = 7 };')
rep('  std::string test_blank;     // --test-blank T0:D,...\n};',
    '  std::string test_blank;     // --test-blank T0:D,...\n'
    '  int baro = 0;               // --baro 0|1 (2026-10-26 ovbaro; 0 = off: type-7 records counted and ignored)\n'
    '  std::string baro_csv;       // --baro-csv FILE (TEST: t_ns,hPa rows released as IMU time passes them)\n'
    '  std::string baro_cfg;       // --baro-cfg K=V,.. (baro.h Cfg)\n'
    '  std::string baro_log;       // --baro-log FILE (one CSV row per sample)\n};')
# ctor: before make_system(); at the end of the ctor
rep(r'''      PRINT_ERROR("[ov_bridge] unable to parse all parameters in %s\n", a.config.c_str());
      ok_ = false;
      return;
    }
    make_system();
  }''', r'''      PRINT_ERROR("[ov_bridge] unable to parse all parameters in %s\n", a.config.c_str());
      ok_ = false;
      return;
    }
    if (a.baro && !baro_setup()) {
      ok_ = false;
      return;
    }
    make_system();
  }

  // ---- --baro (2026-10-26, ovbaro; baro.h) ----
  bool baro_setup() {
    if (!args_.baro_cfg.empty() && !baro_.cfg.parse(args_.baro_cfg)) {
      std::fprintf(stderr, "[ov_bridge] bad --baro-cfg %s\n", args_.baro_cfg.c_str());
      return false;
    }
    if (!args_.baro_log.empty()) {
      baro_.log = std::fopen(args_.baro_log.c_str(), "w");
      if (!baro_.log) {
        std::fprintf(stderr, "[ov_bridge] cannot open --baro-log %s\n", args_.baro_log.c_str());
        return false;
      }
      std::fprintf(baro_.log, "t_ns,t,hpa,h,z_state,innov,sqrt_S,status,b_c,b_g,dz,step\n");
    }
    if (!args_.baro_csv.empty()) {
      FILE *f = std::fopen(args_.baro_csv.c_str(), "r");
      if (!f) {
        std::fprintf(stderr, "[ov_bridge] cannot open --baro-csv %s\n", args_.baro_csv.c_str());
        return false;
      }
      char line[256];
      long long t;
      double v;
      while (std::fgets(line, sizeof(line), f))
        if (std::sscanf(line, "%lld,%lf", &t, &v) == 2) baro_csv_.push_back({static_cast<int64_t>(t), v});
      std::fclose(f);
    }
    std::fprintf(stderr, "[ov_bridge] baro: on | csv %s (%zu samples) | log %s | %s\n",
                 args_.baro_csv.empty() ? "-" : args_.baro_csv.c_str(), baro_csv_.size(),
                 args_.baro_log.empty() ? "-" : args_.baro_log.c_str(), baro_.cfg.describe().c_str());
    return true;
  }
  void baro_rx(int64_t t_ns, double hpa) {  // a pressure sample on the IMU clock (needs the time base: dropped before)
    if (!have_t0_) {
      n_baro_early_++;
      return;
    }
    baro_.push(t_ns, to_sec(t_ns), hpa);
  }
  void log_baro() const {
    if (args_.baro) baro_.summary();
    if (args_.baro && baro_.log) std::fflush(baro_.log);
    if (n_baro_ignored_ || n_baro_early_)
      std::fprintf(stderr, "[ov_bridge] baro records: ignored (--baro 0) %ld, before the first IMU %ld\n", n_baro_ignored_, n_baro_early_);
  }''')
rep(r'''    case kQuit:
      return false;
    default:''', r'''    case kQuit:
      return false;
    case kBaro:  // 2026-10-26 ovbaro: <qd t_ns, hPa
      if (buf.size() != 16) {
        out += err_line("BARO payload must be 16 bytes");
        return true;
      }
      if (args_.baro) baro_rx(rd<int64_t>(buf.data()), rd<double>(buf.data() + 8));
      else n_baro_ignored_++;
      return true;
    default:''')
rep(r'''    imu_hold_.push_back(m);
    newest_imu_t_ = t;
    pump(out);''', r'''    imu_hold_.push_back(m);
    newest_imu_t_ = t;
    while (baro_csv_i_ < baro_csv_.size() && baro_csv_[baro_csv_i_].first <= t_ns) {  // --baro-csv (TEST): arrives with IMU time
      baro_rx(baro_csv_[baro_csv_i_].first, baro_csv_[baro_csv_i_].second);
      baro_csv_i_++;
    }
    pump(out);''')
rep(r'''    scan3d_gyro::add(scan3d_gyro::total, scan3d_gyro::frame_stats);  // ovgyro whole-run counters (output only)
''', r'''    scan3d_gyro::add(scan3d_gyro::total, scan3d_gyro::frame_stats);  // ovgyro whole-run counters (output only)
    if (args_.baro) {  // --baro (2026-10-26): pressure height EKF update on the state of this frame (baro.h)
      try {
        baro_.update(sys_->get_state(), sys_->initialized(), lost_);
      } catch (const std::exception &e) {
        std::fprintf(stderr, "[ov_bridge] baro exception: %s\n", e.what());
      }
    }
''')
rep(r'''    if (c.blank) o << ",\"blank\":true";                                            // --test-blank
''', r'''    if (c.blank) o << ",\"blank\":true";                                            // --test-blank
    if (args_.baro) {                                                               // --baro (2026-10-26)
      auto pr = o.precision();
      baro_.json(o);
      o.precision(pr);
    }
''')
rep(r'''    if (!blank_.empty()) o << ",\"test_blank\":" << jstr(args_.test_blank);
''', r'''    if (!blank_.empty()) o << ",\"test_blank\":" << jstr(args_.test_blank);
    if (args_.baro) o << ",\"baro\":" << jstr(baro_.cfg.describe());                    // 2026-10-26 ovbaro
''')
rep(r'''  std::vector<std::pair<double, double>> blank_;  // --test-blank windows (s after the first camera record)
''', r'''  std::vector<std::pair<double, double>> blank_;  // --test-blank windows (s after the first camera record)
  baro::Aid baro_;                                  // --baro (2026-10-26 ovbaro)
  std::vector<std::pair<int64_t, double>> baro_csv_;  // --baro-csv samples (t_ns, hPa)
  size_t baro_csv_i_ = 0;
  long n_baro_ignored_ = 0, n_baro_early_ = 0;
''')
rep(r'''    else if (s == "--test-blank") a.test_blank = next();
''', r'''    else if (s == "--test-blank") a.test_blank = next();
    else if (s == "--baro") a.baro = std::stoi(next()) != 0 ? 1 : 0;   // 2026-10-26 ovbaro
    else if (s == "--baro-csv") a.baro_csv = next();
    else if (s == "--baro-cfg") a.baro_cfg = next();
    else if (s == "--baro-log") a.baro_log = next();
''')
rep(r'''    b.log_health();
    return 0;''', r'''    b.log_health();
    b.log_baro();  // 2026-10-26 ovbaro
    return 0;''')
rep(r'''"                 [--recover-max-gap S] [--recover-min-feats N] [--recover-full 0|1] [--test-blank T0:D,..]\n");''',
    r'''"                 [--recover-max-gap S] [--recover-min-feats N] [--recover-full 0|1] [--test-blank T0:D,..]\n"
               "                 [--baro 0|1] [--baro-csv FILE] [--baro-cfg K=V,..] [--baro-log FILE]\n");''')
rep(r"// ---- 2026-10-25, branch ovrobust-20261025", r"""// ---- 2026-10-26, branch ovbaro-20261026 (default off = output and code path unchanged) ----
// --baro 0|1: barometer-aided height. Record type 7 BARO (<qd: t_ns on the IMU clock, pressure hPa, 16 B) = one
//   TYPE_PRESSURE sample (arlat intent baro 1). With --baro 1 every camera update is followed by EKF updates of p_z with the
//   pressure height (2 bias states appended to the OpenVINS state, robust gates; baro.h header, runs/ovbaro_20261026).
//   Pose lines get "baro":[used, huber, rejected, b_c, b_g, step], ready line "baro" (config), stderr summary at the end.
//   --baro 0 (default): type-7 records are counted and ignored (stderr line at the end), nothing else changes.
// --baro-csv FILE (TEST ONLY): t_ns,hPa rows fed as if received when the IMU time passes them (PC synthetic tests).
// --baro-cfg K=V,.. (baro.h Cfg: sn, gm_sigma, gm_tau, rw, chi2, vz_max, settle, step_jump, reanchor, steady, temp_c,
//   delay_ms, ground_m, max_age, b0_sigma); --baro-log FILE: one CSV row per sample (status used/huber/step_hold/...).
//
// ---- 2026-10-25, branch ovrobust-20261025""")
open(p, 'w', encoding='utf-8', newline='\n').write(s)
print('edited')
