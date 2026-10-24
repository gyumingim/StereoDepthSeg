// klt_bench — OpenVINS mono KLT tracking + phone remap timing vs OpenCV thread count (2026-10-24, arlat 0.6.15 work).
// CPU only (no camera). Same OpenCV calls / parameters as TrackKLT (C4: 960x540, 200 pts, FAST 15, grid 5x5, min_px 10,
// HISTOGRAM, pyramid 5 levels, win 15) so the per-stage split shows where "tracking" time goes and what scales with threads.
//
//   klt_bench FRAMES.u8 W H N [--threads 1,2,4,6] [--raw RAW.y --remap MAP.bin] [--cpus LIST] [--reps R] [--fx F,FY,CX,CY]
//             [--period-ms P]
//
// Built with ovpatch/TrackKLT.cpp (KLT overlap): env OV_KLT_OVERLAP=0 = upstream TrackKLT path, so the same binary gives
// the before / after hash and ms. --period-ms P paces the TrackKLT loop to one feed per P ms (live-like idle gaps -> CPU
// frequency governor behaves like the 30 fps app instead of a 100 % busy loop).
//
// FRAMES.u8 = N raw uint8 W x H frames back to back (consecutive fed frames). RAW.y = one 1920x1080 Y frame, MAP.bin = the
// ov_bridge --remap file (int32 w,h + float32 mapx + mapy [+ int32 src_w,src_h]); converted with cv::convertMaps like ov_bridge.
// For each thread count T (cv::setNumThreads(T)): per-stage median / p95 ms over the frames, then the real ov_core::TrackKLT
// run over all frames (ms per feed_new_camera) with a hash of its outputs (ids + float bits of every tracked point) =
// determinism check across T. Prints cv::getBuildInformation's parallel framework line and cv::getNumThreads.
//
// Idle-gap experiment (2026-10-24 late, S24: paced 33.3 ms made TrackKLT 4.4 -> 18 ms p50 for every T):
//   --idle MODE        what the paced loop does between frames (needs --period-ms):
//       sleep          sleep_until next period (default = the live bridge's behaviour)
//       spin           busy-wait until next period (main thread never idles)
//       prespin        sleep until (next - warm-ms), then busy-wait the rest
//       helper         main sleeps; one nice-19 helper thread spins for the whole loop (--helper-cpu N pins it)
//       tbbwarm        sleep until (next - warm-ms), then a cv::parallel_for_ over T stripes that each spin until the
//                      period starts (wakes the TBB workers + their cores right before the frame)
//   --warm-ms W        prespin / tbbwarm lead time (default 3)
//   --main-cpu N       pin only the main thread (after TBB pool exists) to cpu N
//   --uclamp-min V     sched_setattr util_min V (0..1024) on the main thread only (scheduler/governor hint, no spinning)
//   --npts N           TrackKLT num_features (default 200 = live C4)
//   --helper-cpu N     pin the helper thread to cpu N
//   --overlap 0|1      sets OV_KLT_OVERLAP before the first TrackKLT (0 = upstream path = live default)
//   --klt-only         skip the per-stage section (only the real TrackKLT loop runs; less heat)
//   --freq-log FILE    sampler thread: every 100 ms t_ms + scaling_cur_freq of policy0/4/7/9 + main thread's cpu
//   --frame-log FILE   per frame: T idx ms cpu_before cpu_after
//   Summary line per run adds mean kHz of each policy over the TrackKLT loop, main-thread cpu histogram and cpuidle
//   deltas (usage count / time of each idle state summed per cluster).
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <atomic>
#include <opencv2/core/utility.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "cam/CamRadtan.h"
#include "feat/FeatureDatabase.h"
#include "track/Grider_FAST.h"
#include "track/Grider_GRID.h"
#include "track/TrackKLT.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using Clock = std::chrono::steady_clock;
static double ms_since(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

struct Stat {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double q(double p) const {
    if (v.empty()) return 0;
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    return s[std::min(s.size() - 1, (size_t)(p * (s.size() - 1) + 0.5))];
  }
  double mean() const {
    double a = 0;
    for (double x : v) a += x;
    return v.empty() ? 0 : a / v.size();
  }
};

static std::vector<int> parse_list(const std::string &s) {
  std::vector<int> r;
  std::stringstream ss(s);
  std::string t;
  while (std::getline(ss, t, ','))
    if (!t.empty()) r.push_back(std::stoi(t));
  return r;
}

static uint64_t fnv(uint64_t h, const void *p, size_t n) {
  auto *b = static_cast<const uint8_t *>(p);
  for (size_t i = 0; i < n; i++) h = (h ^ b[i]) * 1099511628211ull;
  return h;
}

static long read_long(const std::string &path) {
  std::ifstream f(path);
  long v = -1;
  if (f) f >> v;
  return v;
}
static const int kPol[4] = {0, 4, 7, 9};  // S24 (Exynos 2400) cpufreq policies: 0-3 little, 4-6, 7-8, 9 = X4
static void pin_self(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (sched_setaffinity((pid_t)syscall(SYS_gettid), sizeof(set), &set) != 0) std::perror("pin");
}
// cpuidle totals per cluster: [cluster][state] usage, time(us)
struct Idle {
  long usage[4][4] = {}, time[4][4] = {};
};
static int cluster_of(int c) { return c < 4 ? 0 : c < 7 ? 1 : c < 9 ? 2 : 3; }
static Idle read_idle() {
  Idle r;
  for (int c = 0; c < 10; c++)
    for (int s = 0; s < 4; s++) {
      std::string b = "/sys/devices/system/cpu/cpu" + std::to_string(c) + "/cpuidle/state" + std::to_string(s) + "/";
      long u = read_long(b + "usage"), t = read_long(b + "time");
      if (u >= 0) r.usage[cluster_of(c)][s] += u, r.time[cluster_of(c)][s] += t;
    }
  return r;
}
// util clamp min on the calling thread (v < 0 = reset to default); keeps policy / params
static int set_uclamp_min(int v) {
  struct {
    uint32_t size, sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime, sched_deadline, sched_period;
    uint32_t sched_util_min, sched_util_max;
  } a;
  std::memset(&a, 0, sizeof(a));
  a.size = sizeof(a);
  a.sched_flags = 0x08 | 0x10 | 0x20;  // SCHED_FLAG_KEEP_POLICY | KEEP_PARAMS | UTIL_CLAMP_MIN
  a.sched_util_min = v < 0 ? (uint32_t)-1 : (uint32_t)v;
  return (int)syscall(SYS_sched_setattr, (pid_t)syscall(SYS_gettid), &a, 0);
}
static inline void spin_until(Clock::time_point t) {
  while (Clock::now() < t) {
  }
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: klt_bench FRAMES.u8 W H N [--threads 1,2,4,6] [--raw RAW.y --remap MAP.bin] [--cpus LIST] "
                         "[--reps R] [--fx FX,FY,CX,CY] [--period-ms P] [--idle sleep|spin|prespin|helper|tbbwarm] "
                         "[--warm-ms W] [--main-cpu N] [--helper-cpu N] [--overlap 0|1] [--klt-only] [--npts N] [--uclamp-min V] "
                         "[--freq-log FILE] [--frame-log FILE]\n");
    return 2;
  }
  std::string fpath = argv[1];
  int W = std::stoi(argv[2]), H = std::stoi(argv[3]), N = std::stoi(argv[4]);
  std::vector<int> threads = {1, 2, 4, 6};
  std::string raw, mapf, cpus;
  int reps = 1;
  double period_ms = 0;  // --period-ms P: TrackKLT loop paced to one feed per P ms (30 fps = 33.3; idle gaps like live)
  std::string idle = "sleep", freq_log, frame_log;
  double warm_ms = 3;
  int main_cpu = -1, helper_cpu = -1, overlap = -1, uclamp_min = -1;
  bool klt_only = false;
  int npts = 200;  // --npts N: TrackKLT num_features (live C4 = 200)
  double fx = 652.100131, fy = 652.078817, cx = 485.001645, cy = 275.872626;  // config/phone_ov_main/c4
  for (int i = 5; i < argc; i++) {
    std::string s = argv[i];
    std::string v = i + 1 < argc ? argv[i + 1] : "";
    if (s == "--threads") threads = parse_list(v), i++;
    else if (s == "--raw") raw = v, i++;
    else if (s == "--remap") mapf = v, i++;
    else if (s == "--cpus") cpus = v, i++;
    else if (s == "--reps") reps = std::stoi(v), i++;
    else if (s == "--period-ms") period_ms = std::stod(v), i++;
    else if (s == "--idle") idle = v, i++;
    else if (s == "--warm-ms") warm_ms = std::stod(v), i++;
    else if (s == "--main-cpu") main_cpu = std::stoi(v), i++;
    else if (s == "--helper-cpu") helper_cpu = std::stoi(v), i++;
    else if (s == "--overlap") overlap = std::stoi(v), i++;
    else if (s == "--klt-only") klt_only = true;
    else if (s == "--npts") npts = std::stoi(v), i++;
    else if (s == "--uclamp-min") uclamp_min = std::stoi(v), i++;
    else if (s == "--freq-log") freq_log = v, i++;
    else if (s == "--frame-log") frame_log = v, i++;
    else if (s == "--fx") std::sscanf(v.c_str(), "%lf,%lf,%lf,%lf", &fx, &fy, &cx, &cy), i++;
    else {
      std::fprintf(stderr, "bad arg %s\n", s.c_str());
      return 2;
    }
  }
  if (!cpus.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c : parse_list(cpus)) CPU_SET(c, &set);
    if (sched_setaffinity(0, sizeof(set), &set) != 0) std::perror("sched_setaffinity");
  }
  ov_core::Printer::setPrintLevel("WARNING");
  if (overlap >= 0) ::setenv("OV_KLT_OVERLAP", overlap ? "1" : "0", 1);
  if (idle != "sleep" && idle != "spin" && idle != "prespin" && idle != "helper" && idle != "tbbwarm") {
    std::fprintf(stderr, "bad --idle %s\n", idle.c_str());
    return 2;
  }
  std::printf("idle %s | period %.1f ms | warm %.1f ms | main_cpu %d | helper_cpu %d | OV_KLT_OVERLAP=%s\n", idle.c_str(),
              period_ms, warm_ms, main_cpu, helper_cpu, std::getenv("OV_KLT_OVERLAP") ? std::getenv("OV_KLT_OVERLAP") : "unset");
  FILE *ffreq = freq_log.empty() ? nullptr : std::fopen(freq_log.c_str(), "w");
  FILE *fframe = frame_log.empty() ? nullptr : std::fopen(frame_log.c_str(), "w");
  if (ffreq) std::fprintf(ffreq, "# T rep t_ms f0 f4 f7 f9 main_cpu (kHz; policy0=cpu0-3, 4=cpu4-6, 7=cpu7-8, 9=cpu9)\n");
  if (fframe) std::fprintf(fframe, "# T rep idx ms cpu_before cpu_after\n");

  // frames
  std::vector<cv::Mat> frames;
  {
    std::ifstream f(fpath, std::ios::binary);
    for (int i = 0; i < N; i++) {
      cv::Mat m(H, W, CV_8UC1);
      if (!f.read(reinterpret_cast<char *>(m.data), (std::streamsize)W * H)) break;
      frames.push_back(m);
    }
  }
  N = (int)frames.size();
  // remap input (same conversion as ov_bridge load_remap)
  cv::Mat rawm, m1, m2;
  if (!raw.empty() && !mapf.empty()) {
    std::ifstream fm(mapf, std::ios::binary);
    int32_t wh[2];
    fm.read(reinterpret_cast<char *>(wh), 8);
    cv::Mat mx(wh[1], wh[0], CV_32FC1), my(wh[1], wh[0], CV_32FC1);
    fm.read(reinterpret_cast<char *>(mx.data), (std::streamsize)mx.total() * 4);
    fm.read(reinterpret_cast<char *>(my.data), (std::streamsize)my.total() * 4);
    int32_t src[2] = {wh[0], wh[1]};
    fm.read(reinterpret_cast<char *>(src), 8);
    cv::convertMaps(mx, my, m1, m2, CV_16SC2);
    rawm = cv::Mat(src[1], src[0], CV_8UC1);
    std::ifstream fr(raw, std::ios::binary);
    fr.read(reinterpret_cast<char *>(rawm.data), (std::streamsize)rawm.total());
    std::printf("remap %dx%d -> %dx%d\n", src[0], src[1], wh[0], wh[1]);
  }
  std::string bi = cv::getBuildInformation();
  auto pos = bi.find("Parallel framework:");
  std::printf("OpenCV %s | %s | cpus %d | frames %d %dx%d\n", CV_VERSION,
              pos == std::string::npos ? "?" : bi.substr(pos, bi.find('\n', pos) - pos).c_str(), cv::getNumberOfCPUs(), N, W, H);

  const cv::Size win(15, 15);
  const int levels = 5, fast_th = 15, gx = 5, gy = 5;
  std::printf("npts %d\n", npts);
  std::printf("%-3s %8s %8s %8s %8s %8s %8s %8s | %8s %8s %8s  %s\n", "T", "remap", "eqhist", "pyramid", "fast+sub", "lk",
              "ransac", "sum", "klt_p50", "klt_p95", "klt_mean", "hash");
  for (int T : threads) {
    cv::setNumThreads(T);
    for (int rep = 0; rep < reps; rep++) {
      Stat s_remap, s_eq, s_pyr, s_fast, s_lk, s_rs, s_sum;
      uint64_t rh = 0;  // hash of the remap output (first frame): must not depend on T
      std::vector<cv::Mat> pyr_prev;
      std::vector<cv::Point2f> pts_prev;
      for (int i = 0; i < (klt_only ? 0 : N); i++) {
        double r = 0;
        if (!rawm.empty()) {
          auto t0 = Clock::now();
          cv::Mat u;
          cv::remap(rawm, u, m1, m2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
          r = ms_since(t0);
          s_remap.add(r);
          if (i == 0) rh = fnv(1469598103934665603ull, u.data, u.total());
        }
        auto t0 = Clock::now();
        cv::Mat img;
        cv::equalizeHist(frames[i], img);
        double e = ms_since(t0);
        t0 = Clock::now();
        std::vector<cv::Mat> pyr;
        cv::buildOpticalFlowPyramid(img, pyr, win, levels);
        double p = ms_since(t0);
        // detection: all 25 cells (TrackKLT only fills cells under half full; this is the upper bound per call)
        t0 = Clock::now();
        std::vector<std::pair<int, int>> locs;
        for (int x = 0; x < gx; x++)
          for (int y = 0; y < gy; y++) locs.emplace_back(x, y);
        std::vector<cv::KeyPoint> kp;
        cv::Mat mask = cv::Mat::zeros(H, W, CV_8UC1);
        ov_core::Grider_GRID::perform_griding(pyr.at(0), mask, locs, kp, npts, gx, gy, fast_th, true);
        double d = ms_since(t0);
        double l = 0, rs = 0;
        if (!pyr_prev.empty() && pts_prev.size() >= 10) {
          t0 = Clock::now();
          std::vector<cv::Point2f> p1 = pts_prev;
          std::vector<uchar> st;
          std::vector<float> err;
          cv::calcOpticalFlowPyrLK(pyr_prev, pyr, pts_prev, p1, st, err, win, levels,
                                   cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
                                   cv::OPTFLOW_USE_INITIAL_FLOW);
          l = ms_since(t0);
          t0 = Clock::now();
          std::vector<cv::Point2f> n0, n1;
          for (size_t k = 0; k < p1.size(); k++) {
            n0.emplace_back((pts_prev[k].x - cx) / fx, (pts_prev[k].y - cy) / fy);
            n1.emplace_back((p1[k].x - cx) / fx, (p1[k].y - cy) / fy);
          }
          std::vector<uchar> mr;
          cv::findFundamentalMat(n0, n1, cv::FM_RANSAC, 2.0 / std::max(fx, fy), 0.999, mr);
          rs = ms_since(t0);
          s_lk.add(l);
          s_rs.add(rs);
        }
        s_eq.add(e);
        s_pyr.add(p);
        s_fast.add(d);
        s_sum.add(r + e + p + d + l + rs);
        pyr_prev = pyr;
        pts_prev.clear();
        for (size_t k = 0; k < kp.size() && k < (size_t)npts; k++) pts_prev.push_back(kp[k].pt);
      }
      // real TrackKLT
      std::unordered_map<size_t, std::shared_ptr<ov_core::CamBase>> cams;
      auto cam = std::make_shared<ov_core::CamRadtan>(W, H);
      Eigen::Matrix<double, 8, 1> calib;
      calib << fx, fy, cx, cy, 0, 0, 0, 0;
      cam->set_value(calib);
      cams[0] = cam;
      ov_core::TrackKLT trk(cams, npts, 0, false, ov_core::TrackBase::HistogramMethod::HISTOGRAM, fast_th, gx, gy, 10);
      Stat s_klt;
      uint64_t h = 1469598103934665603ull;
      cv::Mat nomask = cv::Mat::zeros(H, W, CV_8UC1);
      // idle-gap instrumentation (see header)
      if (main_cpu >= 0) {
        cv::parallel_for_(cv::Range(0, T), [](const cv::Range &) {});  // create the pool before pinning main
        pin_self(main_cpu);
      }
      if (uclamp_min >= 0) {
        int rc = set_uclamp_min(uclamp_min);
        std::printf("  uclamp_min %d on main thread: %s\n", uclamp_min, rc == 0 ? "ok" : std::strerror(errno));
      }
      std::atomic<bool> run_aux{true};
      std::atomic<int> cur_cpu{-1};
      std::vector<std::array<long, 4>> fsum;  // freq samples during the loop
      int cpu_hist[10] = {};
      std::thread sampler, helper;
      auto tstart = Clock::now();
      sampler = std::thread([&] {
        auto tn = Clock::now();
        while (run_aux.load()) {
          std::array<long, 4> f;
          for (int k = 0; k < 4; k++)
            f[k] = read_long("/sys/devices/system/cpu/cpufreq/policy" + std::to_string(kPol[k]) + "/scaling_cur_freq");
          int c = cur_cpu.load();
          fsum.push_back(f);
          if (c >= 0 && c < 10) cpu_hist[c]++;
          if (ffreq)
            std::fprintf(ffreq, "%d %d %.1f %ld %ld %ld %ld %d\n", T, rep, ms_since(tstart), f[0], f[1], f[2], f[3], c);
          tn += std::chrono::milliseconds(100);
          std::this_thread::sleep_until(tn);
        }
      });
      if (idle == "helper" && period_ms > 0)
        helper = std::thread([&] {
          setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 19);
          if (helper_cpu >= 0) pin_self(helper_cpu);
          volatile unsigned long x = 0;
          while (run_aux.load(std::memory_order_relaxed)) x = x + 1;
        });
      Idle idle0 = read_idle();
      const auto warm = std::chrono::microseconds((long long)(warm_ms * 1000));
      auto tpace = Clock::now();
      for (int i = 0; i < N; i++) {
        if (period_ms > 0) {
          tpace += std::chrono::microseconds((long long)(period_ms * 1000));
          if (idle == "spin") {
            spin_until(tpace);
          } else if (idle == "prespin") {
            std::this_thread::sleep_until(tpace - warm);
            spin_until(tpace);
          } else if (idle == "tbbwarm") {
            std::this_thread::sleep_until(tpace - warm);
            const auto tp = tpace;
            cv::parallel_for_(cv::Range(0, T), [tp](const cv::Range &) { spin_until(tp); }, T);
            spin_until(tpace);
          } else {
            std::this_thread::sleep_until(tpace);
          }
        }
        int cb = sched_getcpu();
        cur_cpu.store(cb);
        ov_core::CameraData m;
        m.timestamp = i / 30.0;
        m.sensor_ids.push_back(0);
        m.images.push_back(frames[i]);
        m.masks.push_back(nomask);
        auto t0 = Clock::now();
        trk.feed_new_camera(m);
        double kms = ms_since(t0);
        s_klt.add(kms);
        if (fframe) std::fprintf(fframe, "%d %d %d %.3f %d %d\n", T, rep, i, kms, cb, sched_getcpu());
        trk.get_feature_database()->cleanup_measurements(m.timestamp - 0.5);
        auto obs = trk.get_last_obs()[0];
        auto ids = trk.get_last_ids()[0];
        for (size_t k = 0; k < obs.size(); k++) {
          h = fnv(h, &obs[k].pt.x, 4);
          h = fnv(h, &obs[k].pt.y, 4);
          h = fnv(h, &ids[k], sizeof(size_t));
        }
      }
      std::printf("%-3d %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f | %8.3f %8.3f %8.3f  %016llx  (getNumThreads %d, remap hash %016llx)\n", T,
                  s_remap.q(0.5), s_eq.q(0.5), s_pyr.q(0.5), s_fast.q(0.5), s_lk.q(0.5), s_rs.q(0.5), s_sum.q(0.5),
                  s_klt.q(0.5), s_klt.q(0.95), s_klt.mean(), (unsigned long long)h, cv::getNumThreads(),
                  (unsigned long long)rh);
      Idle idle1 = read_idle();
      run_aux.store(false);
      sampler.join();
      if (helper.joinable()) helper.join();
      if (uclamp_min >= 0) set_uclamp_min(-1);  // reset for the next T
      if (main_cpu >= 0) {  // unpin for the next T
        cpu_set_t set;
        CPU_ZERO(&set);
        for (int c = 0; c < 10; c++) CPU_SET(c, &set);
        sched_setaffinity((pid_t)syscall(SYS_gettid), sizeof(set), &set);
      }
      double fm[4] = {};
      for (auto &f : fsum)
        for (int k = 0; k < 4; k++) fm[k] += f[k];
      for (int k = 0; k < 4; k++) fm[k] = fsum.empty() ? 0 : fm[k] / fsum.size() / 1000.0;
      std::printf("  T%d idle=%s klt_p50 %.3f p95 %.3f mean %.3f | freq MHz mean pol0 %.0f pol4 %.0f pol7 %.0f pol9 %.0f "
                  "(%zu samples) | main cpu hist", T, idle.c_str(), s_klt.q(0.5), s_klt.q(0.95), s_klt.mean(), fm[0], fm[1], fm[2],
                  fm[3], fsum.size());
      for (int c = 0; c < 10; c++)
        if (cpu_hist[c]) std::printf(" c%d:%d", c, cpu_hist[c]);
      std::printf("\n  cpuidle delta (usage/time_ms per state) ");
      const char *cn[4] = {"L0-3", "M4-6", "M7-8", "X9"};
      for (int k = 0; k < 4; k++) {
        std::printf("%s[", cn[k]);
        for (int s = 0; s < 4; s++)
          if (idle1.usage[k][s] || idle0.usage[k][s])
            std::printf(" s%d:%ld/%ld", s, idle1.usage[k][s] - idle0.usage[k][s], (idle1.time[k][s] - idle0.time[k][s]) / 1000);
        std::printf(" ] ");
      }
      std::printf("\n");
      std::fflush(stdout);
      if (ffreq) std::fflush(ffreq);
      if (fframe) std::fflush(fframe);
    }
  }
  return 0;
}
