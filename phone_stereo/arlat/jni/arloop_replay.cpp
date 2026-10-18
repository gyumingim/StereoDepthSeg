// PC replay of an ARCore keyframe dump (files/arloop_kf.bin, written by the arlat app with --ei dump N in ARCore
// mode) through the same LoopCloser, synchronously. Prints events and the drift correction at the end, and writes
// <out>.csv: t_ns, ARCore x y z, corrected x y z (ARCore world).
//   arloop_replay DUMP [--markers 0|1] [--features 0|1] [--dict N] [--len M] [--out PREFIX]
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "arloop.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s DUMP [--markers 0|1] [--features 0|1] [--dict N] [--len M] [--out PREFIX]\n", argv[0]);
    return 2;
  }
  arloop::Options o;
  o.threaded = false;
  std::string out = "arloop_replay";
  for (int i = 2; i + 1 < argc; i += 2) {
    std::string k = argv[i], v = argv[i + 1];
    if (k == "--markers") o.markers = v != "0";
    else if (k == "--features") o.features = v != "0";
    else if (k == "--dict") o.marker_dict = std::stoi(v);
    else if (k == "--len") o.marker_len = std::stod(v);
    else if (k == "--out") out = v;
  }
  std::ifstream f(argv[1], std::ios::binary);
  arloop::LoopCloser lc(o);
  struct Row { int64_t t; double p[3]; };
  std::vector<Row> rows;
  std::vector<std::vector<double>> poses;
  while (true) {
    uint32_t magic;
    if (!f.read(reinterpret_cast<char *>(&magic), 4)) break;
    if (magic != 0x464B5241) { std::fprintf(stderr, "bad magic\n"); return 1; }   // "ARKF"
    arloop::KeyframeIn kf;
    double p[7], K[4];
    int32_t w, h, dw, dh;
    f.read(reinterpret_cast<char *>(&kf.t_ns), 8);
    f.read(reinterpret_cast<char *>(p), 56);
    f.read(reinterpret_cast<char *>(&w), 4);
    f.read(reinterpret_cast<char *>(&h), 4);
    f.read(reinterpret_cast<char *>(K), 32);
    kf.gray.create(h, w, CV_8UC1);
    f.read(reinterpret_cast<char *>(kf.gray.data), w * h);
    f.read(reinterpret_cast<char *>(&dw), 4);
    f.read(reinterpret_cast<char *>(&dh), 4);
    if (dw > 0 && dh > 0) {
      kf.depth_mm.create(dh, dw, CV_16UC1);
      f.read(reinterpret_cast<char *>(kf.depth_mm.data), 2 * dw * dh);
    }
    if (!f) break;
    kf.t_gl = Eigen::Vector3d(p[0], p[1], p[2]);
    kf.q_gl = Eigen::Quaterniond(p[6], p[3], p[4], p[5]);
    kf.fx = K[0]; kf.fy = K[1]; kf.cx = K[2]; kf.cy = K[3];
    poses.push_back(std::vector<double>(p, p + 7));
    rows.push_back({kf.t_ns, {p[0], p[1], p[2]}});
    lc.add_keyframe(std::move(kf));
  }
  std::ofstream csv(out + ".csv");
  csv << "t_ns,x,y,z,cx,cy,cz\n";
  for (size_t i = 0; i < poses.size(); i++) {
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    auto &p = poses[i];
    // correction as known at the end (blend finished)
    lc.correct(rows.back().t + 10'000'000'000LL, Eigen::Vector3d(p[0], p[1], p[2]), Eigen::Quaterniond(p[6], p[3], p[4], p[5]), t, q);
    csv << rows[i].t << "," << p[0] << "," << p[1] << "," << p[2] << "," << t.x() << "," << t.y() << "," << t.z() << "\n";
  }
  std::cout << lc.stats_json() << std::endl;
  std::cout << "keyframes " << poses.size() << " -> " << out << ".csv" << std::endl;
  return 0;
}
