// npuopt 2026-10-28: XFeat OFFICIAL post-processing in C++ (modules/xfeat.py detectAndCompute, H x W = net input):
//   K = pixel-shuffle(softmax65 drop dustbin); NMS 5x5 (x == maxpool5(x)) & K > 0.05;
//   score = K at the pixel (grid_sample nearest == the pixel itself) * bilinear(sigmoid(heat)) at the pixel
//           (grid_sample align_corners=False, zeros padding, normgrid x/(W-1));
//   top-k by score (desc, raster index asc on ties); desc = bicubic(L2-normalised feats) (A = -0.75, zeros padding) then L2.
// Layout of o (cat1out 130): feats 0-63 | kpt logits 64-128 | heat logit 129, each [Hc][Wc], Hc = H/8, Wc = W/8.
// Needs from the includer: struct Kp {float s; int x, y;}; struct Cand {float s; int idx;}; cand_less(); now_us().
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

struct OffT {
  double soft = 0, nms = 0, topk = 0, desc = 0;
};
struct OffBuf {
  std::vector<float> K, Hs, inv;  // full-res prob, sigmoid(heat) per cell, 1/||feat|| per cell
  std::vector<Cand> cand;
};

static inline float off_cub1(float x) { const float A = -0.75f; return ((A + 2.f) * x - (A + 3.f)) * x * x + 1.f; }
static inline float off_cub2(float x) { const float A = -0.75f; return ((A * x - 5.f * A) * x + 8.f * A) * x - 4.f * A; }

template <typename T>
static void post_off(const T *o, int H, int W, OffBuf &B, std::vector<Kp> &kp, std::vector<float> &desc, int topn, OffT &t,
                     int ymax = -1) {
  const int Hc = H / 8, Wc = W / 8, hw = Hc * Wc;
  if (ymax < 0) ymax = H;
  B.K.resize((size_t)H * W);
  B.Hs.resize(hw);
  B.inv.resize(hw);
  double a = now_us();
  for (int c0 = 0; c0 < hw; c0++) {  // softmax 65 + shuffle; sigmoid heat; inverse feature norm
    const int cy = c0 / Wc, cx = c0 % Wc;
    float m = -1e30f, e[65], s = 0.f;
    for (int c = 0; c < 65; c++) m = std::max(m, (float)o[(64 + c) * hw + c0]);
    for (int c = 0; c < 65; c++) { e[c] = std::exp((float)o[(64 + c) * hw + c0] - m); s += e[c]; }
    float *k0 = &B.K[(size_t)(cy * 8) * W + cx * 8];
    for (int c = 0; c < 64; c++) k0[(c / 8) * W + c % 8] = e[c] / s;
    B.Hs[c0] = 1.f / (1.f + std::exp(-(float)o[129 * hw + c0]));
    float ss = 0.f;
    for (int c = 0; c < 64; c++) { float v = (float)o[c * hw + c0]; ss += v * v; }
    B.inv[c0] = 1.f / std::max(std::sqrt(ss), 1e-12f);  // F.normalize eps 1e-12
  }
  double b = now_us();
  const float sxs = (float)Wc / (float)(W - 1), sys = (float)Hc / (float)(H - 1);
  B.cand.clear();
  for (int y = 0; y < ymax; y++) {
    const float *row = &B.K[(size_t)y * W];
    for (int x = 0; x < W; x++) {
      float v = row[x];
      if (!(v > 0.05f)) continue;
      bool mx = true;
      int ya = std::max(0, y - 2), yb = std::min(H - 1, y + 2), xa = std::max(0, x - 2), xb = std::min(W - 1, x + 2);
      for (int yy = ya; yy <= yb && mx; yy++) {
        const float *r2 = &B.K[(size_t)yy * W];
        for (int xx = xa; xx <= xb; xx++)
          if (r2[xx] > v) { mx = false; break; }
      }
      if (!mx) continue;
      float ix = x * sxs - 0.5f, iy = y * sys - 0.5f;  // bilinear sigmoid(heat), zeros padding
      int x0 = (int)std::floor(ix), y0 = (int)std::floor(iy);
      float tx = ix - x0, ty = iy - y0, h = 0.f;
      for (int dy = 0; dy < 2; dy++)
        for (int dx = 0; dx < 2; dx++) {
          int xx = x0 + dx, yy = y0 + dy;
          if (xx < 0 || yy < 0 || xx >= Wc || yy >= Hc) continue;
          h += (dx ? tx : 1.f - tx) * (dy ? ty : 1.f - ty) * B.Hs[yy * Wc + xx];
        }
      B.cand.push_back({v * h, y * W + x});
    }
  }
  double c_ = now_us();
  auto &cand = B.cand;
  size_t n = std::min<size_t>(topn, cand.size());
  if (cand.size() > n) std::nth_element(cand.begin(), cand.begin() + n, cand.end(), cand_less);
  std::sort(cand.begin(), cand.begin() + n, cand_less);
  while (n > 0 && !(cand[n - 1].s > 0.f)) n--;  // official keeps score > 0 only
  cand.resize(n);
  double d = now_us();
  kp.resize(n);
  desc.assign(n * 64, 0.f);
  for (size_t i = 0; i < n; i++) {
    int x = cand[i].idx % W, y = cand[i].idx / W;
    kp[i] = {cand[i].s, x, y};
    float ix = x * sxs - 0.5f, iy = y * sys - 0.5f;
    int x0 = (int)std::floor(ix), y0 = (int)std::floor(iy);
    float tx = ix - x0, ty = iy - y0;
    float wx[4] = {off_cub2(tx + 1.f), off_cub1(tx), off_cub1(1.f - tx), off_cub2(2.f - tx)};
    float wy[4] = {off_cub2(ty + 1.f), off_cub1(ty), off_cub1(1.f - ty), off_cub2(2.f - ty)};
    float *dd = &desc[i * 64];
    for (int j = 0; j < 4; j++) {
      int yy = y0 - 1 + j;
      if (yy < 0 || yy >= Hc) continue;
      for (int k = 0; k < 4; k++) {
        int xx = x0 - 1 + k;
        if (xx < 0 || xx >= Wc) continue;
        int c0 = yy * Wc + xx;
        float w = wy[j] * wx[k] * B.inv[c0];
        for (int c = 0; c < 64; c++) dd[c] += w * (float)o[c * hw + c0];
      }
    }
    float ss = 0.f;
    for (int c = 0; c < 64; c++) ss += dd[c] * dd[c];
    float r = 1.f / std::max(std::sqrt(ss), 1e-12f);
    for (int c = 0; c < 64; c++) dd[c] *= r;
  }
  double e_ = now_us();
  t.soft += b - a;
  t.nms += c_ - b;
  t.topk += d - c_;
  t.desc += e_ - d;
}
