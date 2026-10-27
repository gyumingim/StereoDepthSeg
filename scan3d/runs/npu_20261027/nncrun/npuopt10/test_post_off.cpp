// PC check of xfpost_off.h: ./t RAW.bin H W NFRAMES OUT_KP.bin REPS  (raw = cat1out 130 f32 per frame)
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>
static double now_us() { return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Kp { float s; int x, y; };
struct Cand { float s; int idx; };
static inline bool cand_less(const Cand &a, const Cand &b) { return a.s > b.s || (a.s == b.s && a.idx < b.idx); }
#include "xfpost_off.h"
int main(int argc, char **argv) {
  int H = atoi(argv[2]), W = atoi(argv[3]), N = atoi(argv[4]), reps = atoi(argv[6]);
  size_t fs = (size_t)130 * (H / 8) * (W / 8);
  std::vector<float> all(fs * N);
  FILE *f = fopen(argv[1], "rb"); size_t r = fread(all.data(), 4, all.size(), f); fclose(f);
  if (r != all.size()) { printf("short read %zu\n", r); return 1; }
  FILE *fk = fopen(argv[5], "wb");
  OffBuf B; std::vector<Kp> kp; std::vector<float> desc; std::vector<double> tt; OffT t;
  for (int rep = 0; rep < reps; rep++)
    for (int i = 0; i < N; i++) {
      double a = now_us();
      post_off(all.data() + fs * i, H, W, B, kp, desc, 1024, t, argc > 7 ? atoi(argv[7]) : -1);
      tt.push_back(now_us() - a);
      if (rep == 0) {
        int32_t n = kp.size(); fwrite(&n, 4, 1, fk);
        for (auto &q : kp) { int32_t xy[2] = {q.x, q.y}; fwrite(xy, 4, 2, fk); }
        fwrite(desc.data(), 4, desc.size(), fk);
      }
    }
  fclose(fk);
  std::sort(tt.begin(), tt.end());
  double c = tt.size();
  printf("post_off %dx%d n=%zu p50 %.0f us p95 %.0f | mean soft %.0f nms %.0f topk %.0f desc %.0f us\n", W, H, tt.size(),
         tt[tt.size() / 2], tt[(size_t)(0.95 * (tt.size() - 1))], t.soft / c, t.nms / c, t.topk / c, t.desc / c);
}
