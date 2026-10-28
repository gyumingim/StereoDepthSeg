// PC check / timing of xfpost_soft.h vs xfpost_off.h:
//   ./ts MODE RAW.bin H W NFRAMES OUT_KP.bin REPS [YMAX]   MODE = off (raw = cat1out 130 f32) | soft (raw = out129 f32)
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
static double now_us() { return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Kp { float s; int x, y; };
struct Cand { float s; int idx; };
static inline bool cand_less(const Cand &a, const Cand &b) { return a.s > b.s || (a.s == b.s && a.idx < b.idx); }
#include "xfpost_soft.h"
int main(int argc, char **argv) {
  bool soft = !strcmp(argv[1], "soft");
  int H = atoi(argv[3]), W = atoi(argv[4]), N = atoi(argv[5]), reps = atoi(argv[7]), ymax = argc > 8 ? atoi(argv[8]) : -1;
  size_t fs = (size_t)(soft ? 129 : 130) * (H / 8) * (W / 8);
  std::vector<float> all(fs * N);
  FILE *f = fopen(argv[2], "rb"); size_t r = fread(all.data(), 4, all.size(), f); fclose(f);
  if (r != all.size()) { printf("short read %zu\n", r); return 1; }
  FILE *fk = fopen(argv[6], "wb");
  OffBuf B; std::vector<Kp> kp; std::vector<float> desc; std::vector<double> tt; OffT t;
  for (int rep = 0; rep < reps; rep++)
    for (int i = 0; i < N; i++) {
      double a = now_us();
      if (soft) post_soft(all.data() + fs * i, H, W, B, kp, desc, 1024, t, ymax);
      else post_off(all.data() + fs * i, H, W, B, kp, desc, 1024, t, ymax);
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
  printf("post_%s %dx%d n=%zu p50 %.0f us p95 %.0f | mean soft %.0f nms %.0f topk %.0f desc %.0f us\n", soft ? "soft" : "off", W, H, tt.size(),
         tt[tt.size() / 2], tt[(size_t)(0.95 * (tt.size() - 1))], t.soft / c, t.nms / c, t.topk / c, t.desc / c);
}
