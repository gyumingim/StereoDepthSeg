// nncrun 2026-10-28: XFeat post-processing on CPU, two versions with identical results (same float operations per value,
// same tie rule as numpy stable argsort: score desc, then raster index asc).
//  post_v1<T>: single thread, stage-timed (softmax 65 -> 8x8 shuffle -> 5x5 NMS + score -> top-k -> desc sample + L2).
//  post_v2<T>: fused softmax+shuffle, NMS/scan split over a small persistent thread pool, nth_element top-k, reads the
//              ENN output buffer directly (T = float or __fp16, no memcpy / no full fp16->fp32 conversion).
#pragma once
#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

struct Cand {
  float s;
  int idx;  // y * PW + x
};
static inline bool cand_less(const Cand &a, const Cand &b) { return a.s > b.s || (a.s == b.s && a.idx < b.idx); }
struct PostT {
  double soft = 0, shuf = 0, nms = 0, topk = 0, desc = 0;
};

template <typename T>
static void post_v1(const T *o, std::vector<float> &P, std::vector<float> &K, std::vector<Cand> &cand, std::vector<Kp> &kp,
                    std::vector<float> &desc, int topn, PostT &t) {
  const int hw = Ph * Pw;
  double a = now_us();
  P.resize(64 * hw);
  for (int c0 = 0; c0 < hw; c0++) {
    float m = -1e30f, e[65], s = 0;
    for (int c = 0; c < 65; c++) m = std::max(m, (float)o[(64 + c) * hw + c0]);
    for (int c = 0; c < 65; c++) {
      e[c] = std::exp((float)o[(64 + c) * hw + c0] - m);
      s += e[c];
    }
    for (int c = 0; c < 64; c++) P[c * hw + c0] = e[c] / s;
  }
  double b = now_us();
  K.resize(PH * PW);
  for (int c = 0; c < 64; c++)
    for (int cy = 0; cy < Ph; cy++)
      for (int cx = 0; cx < Pw; cx++) K[(cy * 8 + c / 8) * PW + cx * 8 + c % 8] = P[c * hw + cy * Pw + cx];
  double c_ = now_us();
  cand.clear();
  for (int y = 0; y < PH; y++)
    for (int x = 0; x < PW; x++) {
      float v = K[y * PW + x];
      if (v <= 0.05f) continue;
      bool mx = true;
      for (int dy = -2; dy <= 2 && mx; dy++)
        for (int dx = -2; dx <= 2; dx++) {
          int yy = y + dy, xx = x + dx;
          if (yy < 0 || xx < 0 || yy >= PH || xx >= PW) continue;
          if (K[yy * PW + xx] > v) {
            mx = false;
            break;
          }
        }
      if (!mx) continue;
      float hl = (float)o[129 * hw + (y / 8) * Pw + x / 8];
      cand.push_back({v / (1.f + std::exp(-hl)), y * PW + x});
    }
  double d = now_us();
  size_t n = std::min<size_t>(topn, cand.size());
  std::partial_sort(cand.begin(), cand.begin() + n, cand.end(), cand_less);
  cand.resize(n);
  double e_ = now_us();
  kp.resize(n);
  desc.assign(n * 64, 0.f);
  for (size_t i = 0; i < n; i++) {
    int x = cand[i].idx % PW, y = cand[i].idx / PW;
    kp[i] = {cand[i].s, x, y};
    int c0 = (y / 8) * Pw + x / 8;
    float ss = 0;
    for (int c = 0; c < 64; c++) {
      float v = (float)o[c * hw + c0];
      desc[i * 64 + c] = v;
      ss += v * v;
    }
    float r = 1.f / std::sqrt(std::max(ss, 1e-12f));
    for (int c = 0; c < 64; c++) desc[i * 64 + c] *= r;
  }
  double f = now_us();
  t.soft += b - a;
  t.shuf += c_ - b;
  t.nms += d - c_;
  t.topk += e_ - d;
  t.desc += f - e_;
}

// minimal persistent pool: run(job) calls job(i) for i = 0..n-1, i = 0 on the caller thread
struct Pool {
  int n = 1;
  std::vector<std::thread> th;
  std::mutex m;
  std::condition_variable cv, cvd;
  std::function<void(int)> job;
  int gen = 0, done = 0;
  bool stop = false;
  std::vector<int> cpus;
  static void pin(int cpu) {
    if (cpu < 0) return;
    cpu_set_t s;
    CPU_ZERO(&s);
    CPU_SET(cpu, &s);
    sched_setaffinity(0, sizeof s, &s);
  }
  void start(int nt, const std::vector<int> &cp) {
    n = nt;
    cpus = cp;
    if (!cpus.empty()) pin(cpus[0]);
    for (int i = 1; i < n; i++)
      th.emplace_back([this, i] {
        if ((int)cpus.size() > i) pin(cpus[i]);
        int g = 0;
        for (;;) {
          std::unique_lock<std::mutex> lk(m);
          cv.wait(lk, [&] { return stop || gen != g; });
          if (stop) return;
          g = gen;
          lk.unlock();
          job(i);
          lk.lock();
          if (++done == n - 1) cvd.notify_one();
        }
      });
  }
  void run(const std::function<void(int)> &j) {
    if (n == 1) {
      j(0);
      return;
    }
    {
      std::lock_guard<std::mutex> lk(m);
      job = j;
      done = 0;
      gen++;
    }
    cv.notify_all();
    j(0);
    std::unique_lock<std::mutex> lk(m);
    cvd.wait(lk, [&] { return done == n - 1; });
  }
  ~Pool() {
    {
      std::lock_guard<std::mutex> lk(m);
      stop = true;
    }
    cv.notify_all();
    for (auto &t : th) t.join();
  }
};

static bool g_skip = false;
static std::atomic<long> g_skipped{0};
template <typename T>
static void post_v2(const T *o, std::vector<float> &K, std::vector<std::vector<Cand>> &cands, std::vector<Cand> &cand,
                    std::vector<Kp> &kp, std::vector<float> &desc, int topn, Pool &pool, PostT &t) {
  const int hw = Ph * Pw;
  K.resize(PH * PW);
  cands.resize(pool.n);
  double a = now_us();
  pool.run([&](int id) {  // fused softmax + shuffle, cell rows split over threads
    int r0 = Ph * id / pool.n, r1 = Ph * (id + 1) / pool.n;
    for (int cy = r0; cy < r1; cy++)
      for (int cx = 0; cx < Pw; cx++) {
        int c0 = cy * Pw + cx;
        float m = -1e30f, e[65], s = 0;
        for (int c = 0; c < 65; c++) m = std::max(m, (float)o[(64 + c) * hw + c0]);
        float *k0 = &K[(cy * 8) * PW + cx * 8];
        if (g_skip) {  // exact skip: every prob of the cell = e/s <= exp(lmax64 - m) (s >= 1); if <= 0.05 no pixel of
                       // the cell passes the threshold and, being <= 0.05, it can never beat a candidate in NMS
          float l64 = -1e30f;
          for (int c = 0; c < 64; c++) l64 = std::max(l64, (float)o[(64 + c) * hw + c0]);
          if (std::exp(l64 - m) <= 0.05f) {
            for (int r = 0; r < 8; r++) std::memset(k0 + r * PW, 0, 8 * sizeof(float));
            g_skipped++;
            continue;
          }
        }
        for (int c = 0; c < 65; c++) {
          e[c] = std::exp((float)o[(64 + c) * hw + c0] - m);
          s += e[c];
        }
        for (int c = 0; c < 64; c++) k0[(c / 8) * PW + c % 8] = e[c] / s;
      }
  });
  double b = now_us();
  pool.run([&](int id) {  // NMS + score, pixel rows split; per-thread lists keep raster order
    int y0 = PH * id / pool.n, y1 = PH * (id + 1) / pool.n;
    auto &cv = cands[id];
    cv.clear();
    for (int y = y0; y < y1; y++) {
      const float *row = &K[y * PW];
      for (int x = 0; x < PW; x++) {
        float v = row[x];
        if (v <= 0.05f) continue;
        bool mx = true;
        int ya = std::max(0, y - 2), yb = std::min(PH - 1, y + 2), xa = std::max(0, x - 2), xb = std::min(PW - 1, x + 2);
        for (int yy = ya; yy <= yb && mx; yy++) {
          const float *r2 = &K[yy * PW];
          for (int xx = xa; xx <= xb; xx++)
            if (r2[xx] > v) {
              mx = false;
              break;
            }
        }
        if (!mx) continue;
        float hl = (float)o[129 * hw + (y / 8) * Pw + x / 8];
        cv.push_back({v / (1.f + std::exp(-hl)), y * PW + x});
      }
    }
  });
  double c_ = now_us();
  cand.clear();
  for (auto &cv : cands) cand.insert(cand.end(), cv.begin(), cv.end());
  size_t n = std::min<size_t>(topn, cand.size());
  if (cand.size() > n) std::nth_element(cand.begin(), cand.begin() + n, cand.end(), cand_less);
  std::sort(cand.begin(), cand.begin() + n, cand_less);
  cand.resize(n);
  double d = now_us();
  kp.resize(n);
  desc.resize(n * 64);
  for (size_t i = 0; i < n; i++) {
    int x = cand[i].idx % PW, y = cand[i].idx / PW;
    kp[i] = {cand[i].s, x, y};
    int c0 = (y / 8) * Pw + x / 8;
    float ss = 0, *dd = &desc[i * 64];
    for (int c = 0; c < 64; c++) {
      float v = (float)o[c * hw + c0];
      dd[c] = v;
      ss += v * v;
    }
    float r = 1.f / std::sqrt(std::max(ss, 1e-12f));
    for (int c = 0; c < 64; c++) dd[c] *= r;
  }
  double e_ = now_us();
  t.soft += b - a;  // softmax+shuffle fused
  t.nms += c_ - b;
  t.topk += d - c_;
  t.desc += e_ - d;
}

// xpsoft (post-in-model softmax) layout: feats 0-63 | softmax probs (dustbin dropped) 64-127 | sigmoid(heat) 128.
// Same rules as acc_lib: K = shuffle(probs), 5x5 NMS, K > 0.05, score = K * heat (multiply), top-k (score desc, index asc).
template <typename T>
static void post_soft(const T *o, std::vector<float> &K, std::vector<Cand> &cand, std::vector<Kp> &kp,
                      std::vector<float> &desc, int topn, PostT &t) {
  const int hw = Ph * Pw;
  K.resize(PH * PW);
  double a = now_us();
  for (int c = 0; c < 64; c++) {
    const T *pc = o + (64 + c) * hw;
    for (int cy = 0; cy < Ph; cy++) {
      float *row = &K[(cy * 8 + c / 8) * PW + c % 8];
      for (int cx = 0; cx < Pw; cx++) row[cx * 8] = (float)pc[cy * Pw + cx];
    }
  }
  double b = now_us();
  cand.clear();
  for (int y = 0; y < PH; y++) {
    const float *rowk = &K[y * PW];
    for (int x = 0; x < PW; x++) {
      float v = rowk[x];
      if (v <= 0.05f) continue;
      bool mx = true;
      int ya = std::max(0, y - 2), yb = std::min(PH - 1, y + 2), xa = std::max(0, x - 2), xb = std::min(PW - 1, x + 2);
      for (int yy = ya; yy <= yb && mx; yy++) {
        const float *r2 = &K[yy * PW];
        for (int xx = xa; xx <= xb; xx++)
          if (r2[xx] > v) {
            mx = false;
            break;
          }
      }
      if (!mx) continue;
      cand.push_back({v * (float)o[128 * hw + (y / 8) * Pw + x / 8], y * PW + x});
    }
  }
  double c_ = now_us();
  size_t n = std::min<size_t>(topn, cand.size());
  if (cand.size() > n) std::nth_element(cand.begin(), cand.begin() + n, cand.end(), cand_less);
  std::sort(cand.begin(), cand.begin() + n, cand_less);
  cand.resize(n);
  double d = now_us();
  kp.resize(n);
  desc.resize(n * 64);
  for (size_t i = 0; i < n; i++) {
    int x = cand[i].idx % PW, y = cand[i].idx / PW;
    kp[i] = {cand[i].s, x, y};
    int c0 = (y / 8) * Pw + x / 8;
    float ss = 0, *dd = &desc[i * 64];
    for (int c = 0; c < 64; c++) {
      float v = (float)o[c * hw + c0];
      dd[c] = v;
      ss += v * v;
    }
    float r = 1.f / std::sqrt(std::max(ss, 1e-12f));
    for (int c = 0; c < 64; c++) dd[c] *= r;
  }
  double e_ = now_us();
  t.shuf += b - a;
  t.nms += c_ - b;
  t.topk += d - c_;
  t.desc += e_ - d;
}
