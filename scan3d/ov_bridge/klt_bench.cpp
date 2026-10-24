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
#include <sched.h>
#include <unistd.h>

#include <algorithm>
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

int main(int argc, char **argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: klt_bench FRAMES.u8 W H N [--threads 1,2,4,6] [--raw RAW.y --remap MAP.bin] [--cpus LIST] "
                         "[--reps R] [--fx FX,FY,CX,CY] [--period-ms P]\n");
    return 2;
  }
  std::string fpath = argv[1];
  int W = std::stoi(argv[2]), H = std::stoi(argv[3]), N = std::stoi(argv[4]);
  std::vector<int> threads = {1, 2, 4, 6};
  std::string raw, mapf, cpus;
  int reps = 1;
  double period_ms = 0;  // --period-ms P: TrackKLT loop paced to one feed per P ms (30 fps = 33.3; idle gaps like live)
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
  const int levels = 5, fast_th = 15, gx = 5, gy = 5, npts = 200;
  std::printf("%-3s %8s %8s %8s %8s %8s %8s %8s | %8s %8s %8s  %s\n", "T", "remap", "eqhist", "pyramid", "fast+sub", "lk",
              "ransac", "sum", "klt_p50", "klt_p95", "klt_mean", "hash");
  for (int T : threads) {
    cv::setNumThreads(T);
    for (int rep = 0; rep < reps; rep++) {
      Stat s_remap, s_eq, s_pyr, s_fast, s_lk, s_rs, s_sum;
      uint64_t rh = 0;  // hash of the remap output (first frame): must not depend on T
      std::vector<cv::Mat> pyr_prev;
      std::vector<cv::Point2f> pts_prev;
      for (int i = 0; i < N; i++) {
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
      auto tpace = Clock::now();
      for (int i = 0; i < N; i++) {
        if (period_ms > 0) {
          tpace += std::chrono::microseconds((long long)(period_ms * 1000));
          std::this_thread::sleep_until(tpace);
        }
        ov_core::CameraData m;
        m.timestamp = i / 30.0;
        m.sensor_ids.push_back(0);
        m.images.push_back(frames[i]);
        m.masks.push_back(nomask);
        auto t0 = Clock::now();
        trk.feed_new_camera(m);
        s_klt.add(ms_since(t0));
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
      std::fflush(stdout);
    }
  }
  return 0;
}
