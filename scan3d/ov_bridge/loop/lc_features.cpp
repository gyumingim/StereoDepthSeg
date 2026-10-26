// ovloop lc_features.cpp (2026-10-26): see lc_features.h
#include "lc_features.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

#include <opencv2/features2d.hpp>

namespace ovloop {

namespace {
void integral(const cv::Mat &g, cv::Mat &I) {  // own (cv::integral may be parallel / IPP)
  I.create(g.rows + 1, g.cols + 1, CV_32S);
  std::memset(I.ptr<int32_t>(0), 0, sizeof(int32_t) * (g.cols + 1));
  for (int y = 0; y < g.rows; y++) {
    const uint8_t *r = g.ptr<uint8_t>(y);
    const int32_t *up = I.ptr<int32_t>(y);
    int32_t *o = I.ptr<int32_t>(y + 1);
    o[0] = 0;
    int32_t s = 0;
    for (int x = 0; x < g.cols; x++) {
      s += r[x];
      o[x + 1] = up[x + 1] + s;
    }
  }
}

void downscale(const cv::Mat &src, double s, cv::Mat &dst) {  // bilinear, own loop (cv::resize is parallel)
  const int W = std::max(1, static_cast<int>(src.cols / s + 0.5)), H = std::max(1, static_cast<int>(src.rows / s + 0.5));
  dst.create(H, W, CV_8U);
  std::vector<int> x0(W);
  std::vector<float> fx(W);
  for (int x = 0; x < W; x++) {
    double sx = (x + 0.5) * s - 0.5;
    sx = std::min(std::max(sx, 0.0), src.cols - 1.001);
    x0[x] = static_cast<int>(sx);
    fx[x] = static_cast<float>(sx - x0[x]);
  }
  for (int y = 0; y < H; y++) {
    double sy = (y + 0.5) * s - 0.5;
    sy = std::min(std::max(sy, 0.0), src.rows - 1.001);
    const int y0 = static_cast<int>(sy);
    const float fy = static_cast<float>(sy - y0);
    const uint8_t *a = src.ptr<uint8_t>(y0), *b = src.ptr<uint8_t>(y0 + 1);
    uint8_t *o = dst.ptr<uint8_t>(y);
    for (int x = 0; x < W; x++) {
      const int i = x0[x];
      const float t = (1 - fx[x]) * a[i] + fx[x] * a[i + 1], u = (1 - fx[x]) * b[i] + fx[x] * b[i + 1];
      o[x] = static_cast<uint8_t>((1 - fy) * t + fy * u + 0.5f);
    }
  }
}

inline int box5(const cv::Mat &I, int x, int y) {
  return I.at<int32_t>(y + 3, x + 3) - I.at<int32_t>(y - 2, x + 3) - I.at<int32_t>(y + 3, x - 2) + I.at<int32_t>(y - 2, x - 2);
}
}  // namespace

void Pyramid::build(const cv::Mat &gray, int levels) {
  levels = std::max(1, std::min(levels, 6));
  img.assign(levels, cv::Mat());
  integ.assign(levels, cv::Mat());
  scale.assign(levels, 1.0);
  img[0] = gray;
  for (int l = 1; l < levels; l++) {
    scale[l] = std::pow(1.4, l);
    downscale(gray, scale[l], img[l]);
  }
  for (int l = 0; l < levels; l++) integral(img[l], integ[l]);
}

Brief::Brief() {
  std::mt19937 rng(20261026u);
  std::normal_distribution<double> nd(0.0, 31.0 / 5.0);
  auto draw = [&]() {
    int v;
    do v = static_cast<int>(std::lround(nd(rng)));
    while (v < -13 || v > 13);
    return v;
  };
  std::vector<int> base;
  while (base.size() < 256 * 4) {
    int x1 = draw(), y1 = draw(), x2 = draw(), y2 = draw();
    if (std::abs(x1 - x2) + std::abs(y1 - y2) < 3) continue;  // too close: weak test
    base.insert(base.end(), {x1, y1, x2, y2});
  }
  for (int b = 0; b < kBins; b++) {
    const double a = b * 2.0 * M_PI / kBins, c = std::cos(a), s = std::sin(a);
    pat_[b].resize(256 * 4);
    for (int i = 0; i < 256; i++)
      for (int k = 0; k < 2; k++) {
        const double x = base[i * 4 + 2 * k], y = base[i * 4 + 2 * k + 1];
        pat_[b][i * 4 + 2 * k] = static_cast<int8_t>(std::lround(c * x - s * y));
        pat_[b][i * 4 + 2 * k + 1] = static_cast<int8_t>(std::lround(s * x + c * y));
      }
  }
}

void Brief::detect(const Pyramid &P, int n_feat, int fast_th, std::vector<KeyPt> &out) const {
  out.clear();
  double area = 0.0;
  for (const auto &m : P.img) area += static_cast<double>(m.rows) * m.cols;
  for (size_t l = 0; l < P.img.size(); l++) {
    const cv::Mat &g = P.img[l];
    const int n_l = std::max(10, static_cast<int>(n_feat * g.rows * g.cols / area + 0.5));
    std::vector<cv::KeyPoint> kp;
    cv::FAST(g, kp, fast_th, true);
    const int M = kMargin + 1;
    kp.erase(std::remove_if(kp.begin(), kp.end(), [&](const cv::KeyPoint &k) {
               return k.pt.x < M || k.pt.y < M || k.pt.x >= g.cols - M || k.pt.y >= g.rows - M;
             }), kp.end());
    std::sort(kp.begin(), kp.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) { return a.response > b.response; });
    const int cs = 32, gx = (g.cols + cs - 1) / cs, gy = (g.rows + cs - 1) / cs;
    const int cap = std::max(2, (2 * n_l + gx * gy - 1) / (gx * gy));
    std::vector<int> cnt(gx * gy, 0);
    int taken = 0;
    const double s = P.scale[l];
    for (const auto &k : kp) {
      if (taken >= n_l) break;
      int &c = cnt[static_cast<int>(k.pt.y) / cs * gx + static_cast<int>(k.pt.x) / cs];
      if (c >= cap) continue;
      c++;
      taken++;
      KeyPt q;
      q.level = static_cast<int>(l);
      q.lx = k.pt.x;
      q.ly = k.pt.y;
      q.x = static_cast<float>((k.pt.x + 0.5) * s - 0.5);
      q.y = static_cast<float>((k.pt.y + 0.5) * s - 0.5);
      q.response = k.response;
      out.push_back(q);
    }
  }
}

void Brief::describe(const Pyramid &P, const std::vector<KeyPt> &kps, cv::Mat &desc, std::vector<uint8_t> &valid) const {
  desc = cv::Mat::zeros(static_cast<int>(kps.size()), 32, CV_8U);
  valid.assign(kps.size(), 0);
  for (size_t i = 0; i < kps.size(); i++) {
    const KeyPt &k = kps[i];
    if (k.level < 0 || k.level >= static_cast<int>(P.img.size())) continue;
    const cv::Mat &I = P.integ[k.level];
    const int W = P.img[k.level].cols, H = P.img[k.level].rows;
    const int x = static_cast<int>(std::lround(k.lx)), y = static_cast<int>(std::lround(k.ly));
    if (x < kMargin || y < kMargin || x >= W - kMargin || y >= H - kMargin) continue;
    double a = std::fmod(k.angle, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    const int b = static_cast<int>(std::lround(a / (2.0 * M_PI / kBins))) % kBins;
    const int8_t *p = pat_[b].data();
    uint8_t *d = desc.ptr<uint8_t>(static_cast<int>(i));
    for (int j = 0; j < 256; j++) {
      const int v1 = box5(I, x + p[4 * j], y + p[4 * j + 1]), v2 = box5(I, x + p[4 * j + 2], y + p[4 * j + 3]);
      if (v1 < v2) d[j >> 3] |= static_cast<uint8_t>(1u << (7 - (j & 7)));
    }
    valid[i] = 1;
  }
}

}  // namespace ovloop
