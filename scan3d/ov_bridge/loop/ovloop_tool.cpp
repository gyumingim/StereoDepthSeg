// ovloop_tool (2026-10-26, branch ovloop-20261026): offline helper for the loop-closure vocabulary.
//   ovloop_tool train OUT.voc [--k 10] [--L 4] [--iters 6] [--step 15] [--nfeat 600] [--levels 3] [--fast 20]
//                [--max-desc 600000] [--angle-deg 90] CACHE_DIR...
// CACHE_DIR = a frame cache of the bench (recordings/<rec>/cache/frames_960x540_<key>/, imu_learn/frame_cache.py):
// c_<id>.u8 = raw uint8 frames (n, H, W) (size from the directory name). Every --step-th frame is used; features are the
// SAME extractor as ov_bridge --loop (loop/lc_features.cpp) with a fixed steering angle (--angle-deg 90 = image "down"
// = gravity for an upright camera; ov_bridge steers each keypoint by the real gravity direction).
//   ovloop_tool info VOC
#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "lc_features.h"
#include "vocab.h"

using namespace ovloop;

static bool parse_wh(const std::string &dir, int &W, int &H) {
  auto p = dir.find("frames_");
  if (p == std::string::npos) return false;
  return std::sscanf(dir.c_str() + p + 7, "%dx%d", &W, &H) == 2;
}

int main(int argc, char **argv) {
  if (argc >= 3 && std::string(argv[1]) == "info") {
    Vocab v;
    std::string err;
    if (!v.load(argv[2], err)) {
      std::fprintf(stderr, "%s\n", err.c_str());
      return 1;
    }
    std::printf("vocab %s: k %d L %d nodes %zu words %d sha %s\n", argv[2], v.k, v.L, v.nodes.size(), v.n_words(), v.sha().c_str());
    return 0;
  }
  if (argc < 4 || std::string(argv[1]) != "train") {
    std::fprintf(stderr, "usage: ovloop_tool train OUT.voc [opts] CACHE_DIR... | ovloop_tool info VOC\n");
    return 2;
  }
  const std::string out = argv[2];
  int k = 10, L = 4, iters = 6, step = 15, nfeat = 600, levels = 3, fast = 20;
  size_t max_desc = 600000;
  double angle_deg = 90.0;
  std::vector<std::string> dirs;
  for (int i = 3; i < argc; i++) {
    std::string s = argv[i];
    auto nx = [&]() { return std::string(argv[++i]); };
    if (s == "--k") k = std::stoi(nx());
    else if (s == "--L") L = std::stoi(nx());
    else if (s == "--iters") iters = std::stoi(nx());
    else if (s == "--step") step = std::stoi(nx());
    else if (s == "--nfeat") nfeat = std::stoi(nx());
    else if (s == "--levels") levels = std::stoi(nx());
    else if (s == "--fast") fast = std::stoi(nx());
    else if (s == "--max-desc") max_desc = std::stoul(nx());
    else if (s == "--angle-deg") angle_deg = std::stod(nx());
    else dirs.push_back(s);
  }
  Brief brief;
  std::vector<uint8_t> store;
  std::vector<int> doc;
  int n_img = 0;
  const auto T0 = std::chrono::steady_clock::now();
  for (const auto &d : dirs) {
    int W = 0, H = 0;
    if (!parse_wh(d, W, H)) {
      std::fprintf(stderr, "skip %s (no WxH in the name)\n", d.c_str());
      continue;
    }
    DIR *dp = opendir(d.c_str());
    if (!dp) {
      std::fprintf(stderr, "skip %s (cannot open)\n", d.c_str());
      continue;
    }
    std::vector<std::string> files;
    while (dirent *e = readdir(dp)) {
      std::string n = e->d_name;
      if (n.size() > 3 && n.substr(n.size() - 3) == ".u8" && n[0] == 'c') files.push_back(d + "/" + n);
    }
    closedir(dp);
    std::sort(files.begin(), files.end());
    int n_this = 0;
    for (const auto &f : files) {
      FILE *fp = std::fopen(f.c_str(), "rb");
      if (!fp) continue;
      struct stat st;
      stat(f.c_str(), &st);
      const size_t fs = static_cast<size_t>(W) * H;
      const long nf = static_cast<long>(st.st_size / fs);
      cv::Mat img(H, W, CV_8U);
      for (long i = 0; i < nf; i += step) {
        if (std::fseek(fp, static_cast<long>(i * fs), SEEK_SET) != 0 || std::fread(img.data, 1, fs, fp) != fs) break;
        Pyramid P;
        P.build(img, levels);
        std::vector<KeyPt> kp;
        brief.detect(P, nfeat, fast, kp);
        for (auto &q : kp) q.angle = static_cast<float>(angle_deg * M_PI / 180.0);
        cv::Mat D;
        std::vector<uint8_t> valid;
        brief.describe(P, kp, D, valid);
        for (int r = 0; r < D.rows; r++)
          if (valid[r]) {
            store.insert(store.end(), D.ptr<uint8_t>(r), D.ptr<uint8_t>(r) + kDescBytes);
            doc.push_back(n_img);
          }
        n_img++;
        n_this++;
      }
      std::fclose(fp);
    }
    std::fprintf(stderr, "%s: %d images, total descriptors %zu\n", d.c_str(), n_this, doc.size());
  }
  if (doc.empty()) {
    std::fprintf(stderr, "no descriptors\n");
    return 1;
  }
  std::vector<const uint8_t *> data;
  std::vector<int> dd;
  {  // uniform subsample to max_desc (fixed seed)
    std::vector<size_t> idx(doc.size());
    for (size_t i = 0; i < idx.size(); i++) idx[i] = i;
    if (idx.size() > max_desc) {
      std::mt19937 rng(7);
      std::shuffle(idx.begin(), idx.end(), rng);
      idx.resize(max_desc);
      std::sort(idx.begin(), idx.end());
    }
    for (size_t i : idx) {
      data.push_back(store.data() + i * kDescBytes);
      dd.push_back(doc[i]);
    }
  }
  Vocab v;
  v.train(data, dd, k, L, iters, 20261026u);
  if (!v.save(out)) {
    std::fprintf(stderr, "cannot write %s\n", out.c_str());
    return 1;
  }
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
  std::printf("trained %s: images %d descriptors %zu (used %zu) k %d L %d iters %d -> nodes %zu words %d sha %s (%.1f s)\n",
              out.c_str(), n_img, doc.size(), data.size(), k, L, iters, v.nodes.size(), v.n_words(), v.sha().c_str(), s);
  return 0;
}
