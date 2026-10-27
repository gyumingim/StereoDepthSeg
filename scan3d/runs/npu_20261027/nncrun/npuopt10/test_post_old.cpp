// PC timing of the existing simple post (xfpost.h post_v2 1 thread, same harness)
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
static double now_us() { return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static const int PH = 352, PW = 640, Ph = 44, Pw = 80;
struct Kp { float s; int x, y; };
#include "xfpost.h"
int main(int argc, char **argv) {
  int N = 13; size_t fs = (size_t)130 * Ph * Pw;
  std::vector<float> all(fs * N);
  FILE *f = fopen(argv[1], "rb"); size_t r = fread(all.data(), 4, all.size(), f); fclose(f);
  std::vector<float> K, desc; std::vector<std::vector<Cand>> cands; std::vector<Cand> cand; std::vector<Kp> kp;
  Pool pool; pool.start(1, {}); PostT t; std::vector<double> tt;
  for (int rep = 0; rep < 20; rep++) for (int i = 0; i < N; i++) {
    double a = now_us(); post_v2(all.data() + fs * i, K, cands, cand, kp, desc, 1024, pool, t); tt.push_back(now_us() - a); }
  std::sort(tt.begin(), tt.end()); printf("post_v2(1thr) p50 %.0f p95 %.0f us\n", tt[tt.size()/2], tt[(size_t)(0.95*(tt.size()-1))]);
}
