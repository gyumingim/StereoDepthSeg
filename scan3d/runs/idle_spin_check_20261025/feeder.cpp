// paced replay of a bridge input stream to stdout (record t_ns pacing, like paced_feed.py). feeder STREAM [start_delay_s]
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <vector>
static int64_t now_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
int main(int argc, char **argv) {
  FILE *f = fopen(argv[1], "rb");
  if (!f) return 1;
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<uint8_t> d(n);
  if (fread(d.data(), 1, n, f) != (size_t)n) return 1;
  double delay = argc > 2 ? atof(argv[2]) : 1.0;
  usleep((useconds_t)(delay * 1e6));
  int64_t t_first = -1, last = -1, w0 = now_ns();
  long i = 0, nrec = 0;
  while (i + 12 <= n) {
    uint32_t h[3];
    memcpy(h, &d[i], 12);
    long len = 12 + h[2];
    if ((h[1] == 1 || h[1] == 2) && h[2] >= 8) {
      int64_t t;
      memcpy(&t, &d[i + 12], 8);
      if (t_first < 0) t_first = last = t;
      if (t > last) last = t;
    }
    int64_t due = w0 + (last - t_first);
    if (due > now_ns()) {  // sleep (no spinning: the feeder must not keep cores awake itself)
      timespec ts{(time_t)(due / 1000000000LL), (long)(due % 1000000000LL)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) != 0) {
      }
    }
    long off = 0;
    while (off < len) {
      ssize_t w = write(1, &d[i + off], len - off);
      if (w <= 0) return 2;
      off += w;
    }
    i += len;
    nrec++;
  }
  fprintf(stderr, "feeder: %ld records, %.2f s\n", nrec, (now_ns() - w0) / 1e9);
  return 0;
}
