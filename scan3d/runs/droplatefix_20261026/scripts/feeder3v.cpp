// verifier copy of runs/ovopt_20261025/feeder3.cpp (pacing unchanged: due = w0 + (t - t0)/speed) + test injections:
//   feeder3v STREAM delay progress speed MAX_T STALL_AT STALL_MS MODE
//   MAX_T: stop after this many seconds of stream time (0 = all)
//   MODE 0 = no stall; 1 = "catch-up" stall: sleep STALL_MS once at stream time STALL_AT, schedule unchanged (= feeder3 after
//   a real stall: it then writes the backlog as fast as the reader takes it); 2 = "shift": sleep STALL_MS once and move the
//   schedule origin w0 by STALL_MS (= what a phone suspend looks like to a CLOCK_MONOTONIC pacer vs the bridge's CLOCK_BOOTTIME)
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
static int64_t now_ns() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec; }
struct Rec { std::vector<uint8_t> d; };
std::deque<Rec> q; std::mutex mu; std::condition_variable cv; size_t qbytes = 0; bool eof = false;
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb");
  if (!f) return 1;
  double delay = atof(argv[2]);
  double speed = atof(argv[4]);
  double max_t = atof(argv[5]), stall_at = atof(argv[6]), stall_ms = atof(argv[7]);
  int mode = atoi(argv[8]);
  std::thread rd([&] {
    for (;;) {
      uint8_t h[12];
      if (fread(h, 1, 12, f) != 12) break;
      uint32_t n; memcpy(&n, h + 8, 4);
      Rec r; r.d.resize(12 + n); memcpy(r.d.data(), h, 12);
      if (n && fread(r.d.data() + 12, 1, n, f) != n) break;
      std::unique_lock<std::mutex> lk(mu);
      cv.wait(lk, [] { return qbytes < (64u << 20); });
      qbytes += r.d.size(); q.push_back(std::move(r)); cv.notify_all();
    }
    std::lock_guard<std::mutex> lk(mu); eof = true; cv.notify_all();
  });
  usleep((useconds_t)(delay * 1e6));
  int64_t t_first = -1, last = -1, w0 = now_ns();
  long nrec = 0, ncam = 0, late_cam_over5 = 0; double late_max = 0, late_cam_max = 0; bool stalled = false;
  for (;;) {
    Rec r;
    { std::unique_lock<std::mutex> lk(mu); cv.wait(lk, [] { return !q.empty() || eof; }); if (q.empty()) break;
      r = std::move(q.front()); q.pop_front(); qbytes -= r.d.size(); cv.notify_all(); }
    uint32_t h[3]; memcpy(h, r.d.data(), 12);
    bool cam = h[1] == 2;
    if ((h[1] == 1 || h[1] == 2) && h[2] >= 8) {
      int64_t t; memcpy(&t, r.d.data() + 12, 8);
      if (t_first < 0) t_first = last = t;
      if (t > last) last = t;
    }
    double st = (last - t_first) / 1e9;
    if (max_t > 0 && st > max_t) break;
    if (mode && !stalled && st >= stall_at) {
      stalled = true;
      usleep((useconds_t)(stall_ms * 1000));
      if (mode == 2) w0 += (int64_t)(stall_ms * 1e6);
      fprintf(stderr, "feeder3v: stall %.0f ms at stream t %.3f mode %d\n", stall_ms, st, mode);
    }
    int64_t due = w0 + (int64_t)((last - t_first) / speed);
    if (due > now_ns()) { timespec ts{(time_t)(due / 1000000000LL), (long)(due % 1000000000LL)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) != 0) {} }
    double late = (now_ns() - due) / 1e6;
    if (late > late_max) late_max = late;
    if (cam) { ncam++; if (late > late_cam_max) late_cam_max = late; if (late > 5) late_cam_over5++; }
    size_t off = 0, len = r.d.size();
    while (off < len) { ssize_t w = write(1, r.d.data() + off, len - off); if (w <= 0) return 2; off += w; }
    nrec++;
  }
  // send nothing more; closing stdout = EOF for the bridge
  fprintf(stderr, "feeder3v speed %.2f: %ld records, %ld frames, %.2f s, late_max %.2f ms, late_cam_max %.2f ms, cam late>5ms %ld\n",
          speed, nrec, ncam, (now_ns() - w0) / 1e9, late_max, late_cam_max, late_cam_over5);
  fflush(stderr);
  _exit(0);
}
