// (ovopt 2026-10-25: unchanged copy of the phone replay study's feeder3.cpp, scratchpad/replay; built for x86 to pace
//  raw80 on the PC for the --drop-late overload check: g++ -O2 -pthread feeder3.cpp -o feeder3)
// paced replay of a bridge input stream to stdout, streamed from disk (file too big for RAM).
// feeder3 STREAM [start_delay_s] [progress_file] [speed]  (feeder2 + speed factor: due = start + (t - t0) / speed)
// Records are written when wall time >= start + (record t - first t) (sleep, no spin). A reader thread keeps up to
// 64 MB of records ahead in memory so disk reads never sit on the pacing path. Stats: late_ms (write start - due).
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
static int64_t now_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
struct Rec { std::vector<uint8_t> d; };
std::deque<Rec> q;
std::mutex mu;
std::condition_variable cv;
size_t qbytes = 0;
bool eof = false;
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb");
  if (!f) return 1;
  posix_fadvise(fileno(f), 0, 0, POSIX_FADV_SEQUENTIAL);
  double delay = argc > 2 ? atof(argv[2]) : 1.0;
  const char *prog = argc > 3 ? argv[3] : nullptr;
  double speed = argc > 4 ? atof(argv[4]) : 1.0;
  std::thread rd([&] {
    for (;;) {
      uint8_t h[12];
      if (fread(h, 1, 12, f) != 12) break;
      uint32_t n;
      memcpy(&n, h + 8, 4);
      Rec r;
      r.d.resize(12 + n);
      memcpy(r.d.data(), h, 12);
      if (n && fread(r.d.data() + 12, 1, n, f) != n) break;
      std::unique_lock<std::mutex> lk(mu);
      cv.wait(lk, [] { return qbytes < (64u << 20); });
      qbytes += r.d.size();
      q.push_back(std::move(r));
      cv.notify_all();
    }
    std::lock_guard<std::mutex> lk(mu);
    eof = true;
    cv.notify_all();
  });
  usleep((useconds_t)(delay * 1e6));
  int64_t t_first = -1, last = -1, w0 = now_ns();
  long nrec = 0, ncam = 0;
  double late_max = 0, late_cam_max = 0;
  long late_cam_over5 = 0;
  for (;;) {
    Rec r;
    {
      std::unique_lock<std::mutex> lk(mu);
      cv.wait(lk, [] { return !q.empty() || eof; });
      if (q.empty()) break;
      r = std::move(q.front());
      q.pop_front();
      qbytes -= r.d.size();
      cv.notify_all();
    }
    uint32_t h[3];
    memcpy(h, r.d.data(), 12);
    bool cam = h[1] == 2;
    if ((h[1] == 1 || h[1] == 2) && h[2] >= 8) {
      int64_t t;
      memcpy(&t, r.d.data() + 12, 8);
      if (t_first < 0) t_first = last = t;
      if (t > last) last = t;
    }
    int64_t due = w0 + (int64_t)((last - t_first) / speed);
    if (due > now_ns()) {
      timespec ts{(time_t)(due / 1000000000LL), (long)(due % 1000000000LL)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) != 0) {
      }
    }
    double late = (now_ns() - due) / 1e6;
    if (late > late_max) late_max = late;
    if (cam) {
      ncam++;
      if (late > late_cam_max) late_cam_max = late;
      if (late > 5) late_cam_over5++;
    }
    size_t off = 0, len = r.d.size();
    while (off < len) {
      ssize_t w = write(1, r.d.data() + off, len - off);
      if (w <= 0) return 2;
      off += w;
    }
    nrec++;
    if (cam && prog && ncam % 150 == 0) {
      FILE *p = fopen(prog, "w");
      if (p) { fprintf(p, "frames %ld t %.1f s late_cam_max %.1f ms\n", ncam, (now_ns() - w0) / 1e9, late_cam_max); fclose(p); }
    }
  }
  rd.join();
  fprintf(stderr, "feeder3 speed %.2f: %ld records, %ld frames, %.2f s, late_max %.2f ms, late_cam_max %.2f ms, cam late>5ms %ld\n",
          speed, nrec, ncam, (now_ns() - w0) / 1e9, late_max, late_cam_max, late_cam_over5);
  return 0;
}
