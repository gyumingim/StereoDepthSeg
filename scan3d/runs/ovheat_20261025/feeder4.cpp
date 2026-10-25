// feeder4 (ovheat 2026-10-25) = feeder3 (replay study / ovopt, paced replay of a bridge input stream to stdout) + LOOPS.
// feeder4 STREAM [start_delay_s] [progress_file] [speed] [loops] [skip_s]
//   loops (default 1 = exactly feeder3): after EOF the file is read again; passes k >= 1 leave out the records of the
//   first skip_s seconds (default 0; e.g. the still start before the motion that initialises the VIO) and every record
//   time (IMU / camera t_ns, the first 8 payload bytes) of pass k is shifted by k * (t_last - t_first - skip + gap),
//   gap = 33.333 ms, so time keeps rising and pacing continues without a pause. The bridge sees a pose jump at each
//   seam (scene restarts; use ov_bridge --auto-reset so the jump resets + re-initialises it) - timing study only.
// Records are written when wall time >= start + (record t - first t) / speed (sleep, no spin). A reader thread keeps
// up to 64 MB of records ahead in memory. Stats: late_ms (write start - due).
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
  const char *path = argv[1];
  double delay = argc > 2 ? atof(argv[2]) : 1.0;
  const char *prog = argc > 3 ? argv[3] : nullptr;
  double speed = argc > 4 ? atof(argv[4]) : 1.0;
  int loops = argc > 5 ? atoi(argv[5]) : 1;
  if (loops < 1) loops = 1;
  const int64_t skip = (int64_t)((argc > 6 ? atof(argv[6]) : 0.0) * 1e9);
  std::thread rd([&] {
    int64_t span = -1, f0 = -1, f1 = -1;  // first / last record time of pass 0
    for (int k = 0; k < loops; k++) {
      FILE *f = fopen(path, "rb");
      if (!f) break;
      posix_fadvise(fileno(f), 0, 0, POSIX_FADV_SEQUENTIAL);
      const int64_t off = k == 0 ? 0 : (int64_t)k * span;
      bool skipping = k > 0 && skip > 0;
      for (;;) {
        uint8_t h[12];
        if (fread(h, 1, 12, f) != 12) break;
        uint32_t n;
        memcpy(&n, h + 8, 4);
        Rec r;
        r.d.resize(12 + n);
        memcpy(r.d.data(), h, 12);
        if (n && fread(r.d.data() + 12, 1, n, f) != n) break;
        uint32_t ty;
        memcpy(&ty, h + 4, 4);
        if ((ty == 1 || ty == 2) && n >= 8) {
          int64_t t;
          memcpy(&t, r.d.data() + 12, 8);
          if (k == 0) {
            if (f0 < 0) f0 = t;
            if (t > f1) f1 = t;
          } else {
            if (skipping && t < f0 + skip) continue;  // pass k >= 1: leave out the first skip_s seconds
            skipping = false;
            t += off;
            memcpy(r.d.data() + 12, &t, 8);
          }
        } else if (skipping) {
          continue;
        }
        std::unique_lock<std::mutex> lk(mu);
        cv.wait(lk, [] { return qbytes < (64u << 20); });
        qbytes += r.d.size();
        q.push_back(std::move(r));
        cv.notify_all();
      }
      fclose(f);
      if (k == 0) span = f1 - f0 - skip + 33333333LL;
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
    size_t o = 0, len = r.d.size();
    while (o < len) {
      ssize_t w = write(1, r.d.data() + o, len - o);
      if (w <= 0) return 2;
      o += w;
    }
    nrec++;
    if (cam && prog && ncam % 150 == 0) {
      FILE *p = fopen(prog, "w");
      if (p) { fprintf(p, "frames %ld t %.1f s late_cam_max %.1f ms\n", ncam, (now_ns() - w0) / 1e9, late_cam_max); fclose(p); }
    }
  }
  rd.join();
  fprintf(stderr, "feeder4 speed %.2f loops %d: %ld records, %ld frames, %.2f s, late_max %.2f ms, late_cam_max %.2f ms, cam late>5ms %ld\n",
          speed, loops, nrec, ncam, (now_ns() - w0) / 1e9, late_max, late_cam_max, late_cam_over5);
  return 0;
}
