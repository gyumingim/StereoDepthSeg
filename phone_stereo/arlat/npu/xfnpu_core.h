// xfnpu_core.h (appnpu 2026-10-28): XFeat + LighterGlue feature / matching server for ov_bridge --loop-xf unix:NAME.
//
// Runs inside the arlat app process (the phone NPU is reached through the vendor library, which only an app process can
// dlopen: see enn_backend.h). One worker thread: accepts the bridge's loop thread on an abstract unix socket and answers the
// xf_server.py protocol (runs/lgopt_20261027/xf_server.py, little endian):
//   'F' i64 id, i32 W, i32 H, u8[W*H] gray, i32 nq, f32[2*nq] query pixels (OpenVINS tracked points)
//       -> i32 nk, f32[2*nk] XFeat keypoints (pixels of the given image), f32 ms   (point list of id = [queries, keypoints])
//   'M' i64 idA, i64 idB, i32 mode (1 mutual NN cos >= min_cos, 2 LighterGlue) -> i32 n, n x (i32 ia, i32 ib, f32 s), f32 ms
//   'Q' -> end of this session (the server keeps listening until stop()).
// 0.6.31 explicit errors (never "0 points / 0 matches" for a failure): nk / n = -1 NPU (EnnExecuteModel) failure,
//   -2 image size does not fit the model, -3 keyframe not in the cache (never sent / evicted), -4 bad mode; followed by
//   the f32 ms, then the server closes the session. The bridge treats any negative count as "server lost" (xf_down ->
//   loop closure stops, run INVALID).
// The NPU calls go through the Backend interface (phone: ENN; PC test: torch stand-in over a pipe). No CPU / GPU model path.
//
// XFeat: model input 1 x 1 x Hm x Wm float (Wm = W, Hm = H rounded up to 32: 960x540 -> 960x544, bottom rows replicated),
//   x = gray / 255, then (x - mean) / sqrt(var + 1e-5) over the whole padded image (= npuopt10 xf960_eval.py run960).
//   Output 129 channels (feats 64 | softmax probs 64 | sigmoid heat) -> xfpost_soft.h, or 130 (cat1out) -> xfpost_off.h;
//   keypoints with y >= H dropped (ymax), top-k by the official score. Query descriptors: same bicubic sampling of the
//   L2-normalised dense map at the (sub-pixel) query position, then L2 (xf_server.py: InterpolateSparse2d bicubic).
// LighterGlue: model input [1,N,512] = desc0 | desc1 | cos0 | sin0 | cos1 | sin1 (cos/sin of k @ W, each 48 duplicated to 96,
//   k = kornia-normalised keypoints (k - [W/2, H/2]) / (max(W, H) / 2)), output [1,N,2N+2] = A | BT | z0 | z1
//   -> CPU (nncrun lgpipe.h lgpost 2): sc_ij = log(A_ij BT_ji) + logsig(z0_i) + logsig(z1_j), mutual argmax over the REAL
//   points only, exp(max) > lg_thr. Hooks (not yet verified on a real model): output [1,N,3] = in-model decision
//   (lgpost L6p: row argmax, column argmax, row max sc) and input [1,N,513] = masked model (col 512 = 1 real / 0 padding,
//   one mask for both sides -> both sides use min(nA, nB) points).
#pragma once
#include <sched.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace xfnpu {

static inline double now_us() {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static inline double wall_ms() {
  return std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
}
struct Kp {
  float s;
  int x, y;
};
struct Cand {
  float s;
  int idx;
};
static inline bool cand_less(const Cand &a, const Cand &b) { return a.s > b.s || (a.s == b.s && a.idx < b.idx); }
#if defined(__aarch64__)
#include <arm_neon.h>
#endif
#include "xfpost_soft.h"  // includes xfpost_off.h (npuopt10, ovxfeat b718927 / 6843464, copied unchanged)

// ---- NPU backend (phone: ENN, PC test: pipe to a torch stand-in) ----
class Backend {
public:
  virtual ~Backend() {}
  virtual bool init(std::string &err) = 0;
  // returns a model handle >= 0; in / out = the single input / output buffer (float32)
  virtual int load(const std::string &path, std::string &err) = 0;
  virtual float *in(int m) = 0;
  virtual size_t in_bytes(int m) = 0;
  virtual const float *out(int m) = 0;
  virtual size_t out_bytes(int m) = 0;
  virtual bool run(int m, std::string &err) = 0;
  virtual void close_all() = 0;
  virtual std::string describe() = 0;
};

struct Cfg {
  std::string sock;              // abstract socket name without '@'
  std::string xf_model, lg_model, wr_path, log_path;
  int topk = 512;                // XFeat keypoints per keyframe
  std::string lg_in = "x512";    // x512: LighterGlue on the XFeat points only; trk768: tracks + XFeat (masked model needed)
  int lg_pad = 2;                // unmasked model, fewer real points than N: 0 zero desc at the centre, 1 repeat, 2 repeat + copy sum
  float min_cos = 0.82f;         // mutual NN (mode 1)
  double lg_thr = 0.1;           // LighterGlue match probability
  std::vector<int> cpus;         // worker thread affinity ({} = no pinning)
  int nice = 10;                 // worker thread nice
  double cache_mb = 300.0;       // descriptor cache (fp16) cap; oldest keyframes dropped (logged)
  int nn_fast = 0;               // 0.6.35 npunight: 1 = mutual NN dot products with 16 partial sums (NEON on arm64; other
                                 //   summation order than the plain loop -> cosines can differ in the last bits)
  int lg_post = 2;               // 0.6.35 npunight: 2 = log-score post (as 0.6.34), 3 = product post (nncrun lgpost 3, no N^2 log)
};

// IEEE half <-> float (round to nearest even), own code
static inline uint16_t f2h(float f) {
  uint32_t x;
  std::memcpy(&x, &f, 4);
  const uint32_t s = (x >> 16) & 0x8000u;
  int e = static_cast<int>((x >> 23) & 0xff) - 127 + 15;
  uint32_t m = x & 0x7fffffu;
  if (((x >> 23) & 0xff) == 0xff) return static_cast<uint16_t>(s | 0x7c00u | (m ? 0x200u : 0u));
  if (e >= 31) return static_cast<uint16_t>(s | 0x7c00u);
  if (e <= 0) {
    if (e < -10) return static_cast<uint16_t>(s);
    m |= 0x800000u;
    const int sh = 14 - e;
    uint32_t hm = m >> sh;
    const uint32_t rem = m & ((1u << sh) - 1), half = 1u << (sh - 1);
    if (rem > half || (rem == half && (hm & 1u))) hm++;
    return static_cast<uint16_t>(s | hm);
  }
  uint32_t hm = m >> 13;
  const uint32_t rem = m & 0x1fffu;
  uint32_t h = s | (static_cast<uint32_t>(e) << 10) | hm;
  if (rem > 0x1000u || (rem == 0x1000u && (hm & 1u))) h++;
  return static_cast<uint16_t>(h);
}
static inline float h2f(uint16_t h) {
  const uint32_t s = (h & 0x8000u) << 16;
  uint32_t e = (h >> 10) & 0x1f, m = h & 0x3ffu, x;
  if (e == 0) {
    if (m == 0) x = s;
    else {
      e = 127 - 15 + 1;
      while (!(m & 0x400u)) { m <<= 1; e--; }
      m &= 0x3ffu;
      x = s | (e << 23) | (m << 13);
    }
  } else if (e == 31) x = s | 0x7f800000u | (m << 13);
  else x = s | ((e + 127 - 15) << 23) | (m << 13);
  float f;
  std::memcpy(&f, &x, 4);
  return f;
}

struct Half {  // fp16 ENN buffer element, read as float (post_soft / post_off / sample_desc are templates on the element)
  uint16_t v;
  operator float() const { return h2f(v); }
};

static inline std::string jn(double v) {  // JSON number, NaN / inf -> null
  if (!std::isfinite(v)) return "null";
  char b[32];
  std::snprintf(b, sizeof b, "%.1f", v);
  return b;
}

static inline double logsig(double z) { return z >= 0 ? -std::log1p(std::exp(-z)) : z - std::log1p(std::exp(z)); }

class Server {
public:
  explicit Server(Backend *be) : be_(be) {}
  ~Server() {
    if (!stuck_) stop();
  }

  // opens the backend + both models, checks the buffer shapes, one warm-up run each, binds the socket; then the worker
  // thread serves. Returns false (and err) when anything fails: the caller must then run without the feature.
  int start_timeout_ms = 10000;  // 0.6.31
  bool stuck() const { return stuck_; }  // start timed out or stop() could not join: never destroy this object
  bool start(const Cfg &c, std::string &err) {
    c_ = c;
    if (!c_.log_path.empty()) log_ = std::fopen(c_.log_path.c_str(), "w");
    // start handshake through members (the worker outlives this call; no reference to locals of start())
    th_ = std::thread([this] {
      std::string e;
      const bool ok = init_worker(e);
      {
        std::lock_guard<std::mutex> l(st_m_);
        st_state_ = ok ? 1 : 2;
        st_err_ = e;
        st_cv_.notify_all();
      }
      if (ok && !stop_) serve();
      teardown();
      done_ = true;
    });
    {
      std::unique_lock<std::mutex> l(st_m_);
      if (!st_cv_.wait_for(l, std::chrono::milliseconds(start_timeout_ms), [this] { return st_state_ != 0; })) {
        // 0.6.31: worker stuck in ENN init / model open / warm-up -> refuse; the worker keeps this object -> caller must leak it
        err = "NPU start timeout " + std::to_string(start_timeout_ms) + " ms (ENN init / model open / warm-up)";
        stuck_ = true;
        stop_ = true;
        l.unlock();
        th_.detach();
        return false;
      }
    }
    if (st_state_ != 1) {
      err = st_err_;
      th_.join();
      return false;
    }
    running_ = true;
    return true;
  }
  // returns false when the worker did not finish within timeout_ms (e.g. stuck inside EnnExecuteModel): the thread is then
  // detached and this object must NOT be destroyed (the JNI layer leaks it on purpose)
  bool stop(int timeout_ms = 3000) {
    stop_ = true;
    int fd = cli_fd_.exchange(-1);
    if (fd >= 0) ::shutdown(fd, SHUT_RDWR);
    if (th_.joinable()) {
      for (int i = 0; i < timeout_ms / 10 && !done_; i++) usleep(10000);
      if (!done_) {
        L("{\"op\":\"stop\",\"error\":\"worker still busy after %d ms, detached\"}", timeout_ms);
        stuck_ = true;
        fail("worker still busy at stop (" + std::to_string(timeout_ms) + " ms)");
        th_.detach();
        running_ = false;
        return false;
      }
      th_.join();
    }
    running_ = false;
    if (log_) {
      std::fclose(log_);
      log_ = nullptr;
    }
    return true;
  }
  bool running() const { return running_; }
  void set_temps(float skin, float ap, float bat, int status) {
    skin_ = skin;
    ap_ = ap;
    bat_ = bat;
    tstat_ = status;
  }
  std::string start_log() const { return start_log_; }
  // 0.6.31: first NPU failure during the run ("" = none); the app marks the run INVALID
  std::string failure() {
    std::lock_guard<std::mutex> l(fail_m_);
    return fail_;
  }

private:
  struct Entry {
    int W = 0, H = 0, nq = 0, n = 0;
    std::vector<float> pts;     // 2n
    std::vector<uint16_t> d;    // 64n fp16
  };

  void L(const char *fmt, ...) {
    char b[4096];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    std::lock_guard<std::mutex> l(logm_);
    if (log_) {
      std::fputs(b, log_);
      std::fputc('\n', log_);
      std::fflush(log_);
    }
  }
  void S(const char *fmt, ...) {  // start log (returned to the app, also into the log file)
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    start_log_ += b;
    start_log_ += "\n";
    std::string j = "{\"op\":\"start\",\"msg\":\"";
    for (const char *p = b; *p; p++) {
      if (*p == '"' || *p == '\\') j += '\\';
      j += (*p == '\n' ? ' ' : *p);
    }
    j += "\"}";
    L("%s", j.c_str());
  }

  bool init_worker(std::string &err) {
    const pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
    if (!c_.cpus.empty()) {
      cpu_set_t set;
      CPU_ZERO(&set);
      for (int c : c_.cpus) CPU_SET(c, &set);
      const int r = sched_setaffinity(tid, sizeof set, &set);
      S("worker tid %d affinity %s -> %d", static_cast<int>(tid), cpus_str().c_str(), r);
    } else S("worker tid %d not pinned", static_cast<int>(tid));
    if (c_.nice) S("worker nice %d -> %d", c_.nice, setpriority(PRIO_PROCESS, static_cast<id_t>(tid), c_.nice));
    S("opts nn_fast %d lg_post %d", c_.nn_fast, c_.lg_post);
    if (!be_->init(err)) return false;
    S("backend %s", be_->describe().c_str());
    xf_ = be_->load(c_.xf_model, err);
    if (xf_ < 0) return false;
    lg_ = be_->load(c_.lg_model, err);
    if (lg_ < 0) return false;
    // XFeat shapes: input 1 x 544 x 960, output (129 | 130) x 68 x 120; float32 or fp16 buffers (fp16 = the model has no
    // CPU Cast at that end, e.g. nncrun xfpo16), told apart by the buffer size
    const size_t xin = be_->in_bytes(xf_), xout = be_->out_bytes(xf_);
    Wm_ = 960;
    Hm_ = 544;
    const size_t npx = static_cast<size_t>(Wm_) * Hm_, cell = static_cast<size_t>(Wm_ / 8) * (Hm_ / 8);
    if (xin == npx * 4) xf_in16_ = false;
    else if (xin == npx * 2) xf_in16_ = true;
    else {
      err = "XFeat model input " + std::to_string(xin) + " B is not 1x1x544x960 float32 / fp16";
      return false;
    }
    if (xout == 129 * cell * 4 || xout == 129 * cell * 2) xf_soft_ = true;
    else if (xout == 130 * cell * 4 || xout == 130 * cell * 2) xf_soft_ = false;
    else {
      err = "XFeat model output " + std::to_string(xout) + " B is neither 129 nor 130 x 68x120 float32 / fp16";
      return false;
    }
    xf_out16_ = xout == (xf_soft_ ? 129 : 130) * cell * 2;
    // LighterGlue shapes
    const size_t lin = be_->in_bytes(lg_), lout = be_->out_bytes(lg_);
    if (lin % (512 * 4) == 0 && (lout == (lin / 2048) * 3 * 4 || lout == (lin / 2048) * (2 * (lin / 2048) + 2) * 4)) {
      N_ = static_cast<int>(lin / 2048);
      lg_mask_ = false;
    } else if (lin % (513 * 4) == 0 && (lout == (lin / 2052) * 3 * 4 || lout == (lin / 2052) * (2 * (lin / 2052) + 2) * 4)) {
      N_ = static_cast<int>(lin / 2052);
      lg_mask_ = true;
    } else {
      err = "LighterGlue model buffers in " + std::to_string(lin) + " / out " + std::to_string(lout) + " B not recognised";
      return false;
    }
    lg_inmodel_ = lout == static_cast<size_t>(N_) * 3 * 4;
    if (c_.lg_in == "trk768" && !lg_mask_) {
      err = "npu_lg_in trk768 needs a masked LighterGlue model (input N x 513); this one is N x 512";
      return false;
    }
    if (c_.lg_in != "x512" && c_.lg_in != "trk768") {
      err = "npu_lg_in must be x512 or trk768";
      return false;
    }
    // positional encoding weights W [2,48] f32 (lg_wr.npy of the model, raw 384 B or .npy)
    {
      std::vector<char> d;
      FILE *f = std::fopen(c_.wr_path.c_str(), "rb");
      if (!f) {
        err = "cannot open " + c_.wr_path;
        return false;
      }
      char b[4096];
      size_t k;
      while ((k = std::fread(b, 1, sizeof b, f)) > 0) d.insert(d.end(), b, b + k);
      std::fclose(f);
      if (d.size() < 384) {
        err = "lg_wr too small";
        return false;
      }
      std::memcpy(wr_, d.data() + d.size() - 384, 384);  // .npy: data is the last 384 bytes (header first)
    }
    S("xfeat %s: in 1x%dx%d %s, out %d ch %s (%s) | lighterglue %s: N %d%s, out %s | lg_in %s pad %d | topk %d min_cos %.2f "
      "lg_thr %.2f cache %.0f MB", c_.xf_model.c_str(), Hm_, Wm_, xf_in16_ ? "fp16" : "f32", xf_soft_ ? 129 : 130,
      xf_out16_ ? "fp16" : "f32", xf_soft_ ? "post_soft" : "post_off",
      c_.lg_model.c_str(), N_, lg_mask_ ? " masked (513)" : "", lg_inmodel_ ? "N x 3 (in-model decision)" : "A|BT|z0|z1",
      c_.lg_in.c_str(), c_.lg_pad, c_.topk, c_.min_cos, c_.lg_thr, c_.cache_mb);
    // warm-up (zero input): first ENN runs are slower; also proves both models execute
    for (int m : {xf_, lg_}) {
      std::memset(be_->in(m), 0, be_->in_bytes(m));
      const double a = now_us();
      if (!be_->run(m, err)) return false;
      S("warm-up %s %.0f us", m == xf_ ? "xfeat" : "lighterglue", now_us() - a);
    }
    // socket
    lfd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (lfd_ < 0) {
      err = "socket() failed";
      return false;
    }
    sockaddr_un sa;
    std::memset(&sa, 0, sizeof sa);
    sa.sun_family = AF_UNIX;
    if (c_.sock.empty() || c_.sock.size() + 1 >= sizeof sa.sun_path) {
      err = "bad socket name";
      return false;
    }
    std::memcpy(sa.sun_path + 1, c_.sock.data(), c_.sock.size());  // abstract: leading NUL
    const socklen_t len = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + c_.sock.size());
    if (::bind(lfd_, reinterpret_cast<sockaddr *>(&sa), len) != 0 || ::listen(lfd_, 2) != 0) {
      err = std::string("bind/listen @") + c_.sock + ": " + std::strerror(errno);
      return false;
    }
    S("listening on @%s", c_.sock.c_str());
    return true;
  }

  std::string cpus_str() const {
    std::string s;
    for (int c : c_.cpus) s += (s.empty() ? "" : ",") + std::to_string(c);
    return s.empty() ? "-" : s;
  }

  void teardown() {
    if (lfd_ >= 0) ::close(lfd_);
    lfd_ = -1;
    be_->close_all();
    L("{\"op\":\"end\",\"n_feat\":%llu,\"n_match\":%llu,\"n_lg\":%llu,\"n_err\":%llu,\"evicted\":%llu}",
      (unsigned long long)n_feat_, (unsigned long long)n_match_, (unsigned long long)n_lg_, (unsigned long long)n_err_,
      (unsigned long long)n_evict_);
  }

  static bool rd(int fd, void *p, size_t n) {
    char *c = static_cast<char *>(p);
    while (n) {
      const ssize_t k = ::read(fd, c, n);
      if (k <= 0) {
        if (k < 0 && errno == EINTR) continue;
        return false;
      }
      c += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }
  static bool wr(int fd, const void *p, size_t n) {
    const char *c = static_cast<const char *>(p);
    while (n) {
      const ssize_t k = ::send(fd, c, n, MSG_NOSIGNAL);
      if (k <= 0) {
        if (k < 0 && errno == EINTR) continue;
        return false;
      }
      c += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }

  void serve() {
    while (!stop_) {
      pollfd p{lfd_, POLLIN, 0};
      const int r = ::poll(&p, 1, 200);
      if (r <= 0) continue;
      const int fd = ::accept4(lfd_, nullptr, nullptr, SOCK_CLOEXEC);
      if (fd < 0) continue;
      cli_fd_ = fd;
      n_sess_++;
      L("{\"op\":\"session\",\"n\":%d,\"t_wall_ms\":%.0f}", n_sess_, wall_ms());
      session(fd);
      int f2 = cli_fd_.exchange(-1);
      if (f2 >= 0) ::close(f2);
      cache_.clear();
      order_.clear();
      cache_bytes_ = 0;
    }
  }

  void session(int fd) {
    std::vector<uint8_t> img;
    std::vector<float> q;
    while (!stop_) {
      char c;
      if (!rd(fd, &c, 1)) {  // 0.6.33: the bridge closed without 'Q' (its xf_down shuts the socket, or it crashed)
        if (!stop_) fail("bridge closed the NPU connection without Q (bridge lost the server / exited)");
        break;
      }
      const double t_req = now_us();
      if (c == 'Q') break;
      if (c == 'F') {
        int64_t id;
        int32_t W, H, nq;
        if (!rd(fd, &id, 8) || !rd(fd, &W, 4) || !rd(fd, &H, 4) || W <= 0 || H <= 0 || W > 4096 || H > 4096) break;
        img.resize(static_cast<size_t>(W) * H);
        if (!rd(fd, img.data(), img.size()) || !rd(fd, &nq, 4) || nq < 0 || nq > 100000) break;
        q.resize(2 * static_cast<size_t>(nq));
        if (nq && !rd(fd, q.data(), q.size() * 4)) break;
        std::vector<float> kp;
        int32_t code = 0;
        const bool ok = feat(id, W, H, img.data(), nq, q.data(), kp, t_req, code);
        const int32_t nk = ok ? static_cast<int32_t>(kp.size() / 2) : (code < 0 ? code : -1);
        const float ms = static_cast<float>((now_us() - t_req) / 1000.0);
        if (!wr(fd, &nk, 4) || (nk > 0 && !wr(fd, kp.data(), kp.size() * 4)) || !wr(fd, &ms, 4)) break;
        if (nk < 0) break;  // 0.6.31: error answered explicitly, then the session ends
      } else if (c == 'M') {
        int64_t a, b;
        int32_t mode;
        if (!rd(fd, &a, 8) || !rd(fd, &b, 8) || !rd(fd, &mode, 4)) break;
        std::vector<char> rec;
        const int32_t n = match(a, b, mode, rec, t_req);
        const float ms = static_cast<float>((now_us() - t_req) / 1000.0);
        if (!wr(fd, &n, 4) || (n > 0 && !wr(fd, rec.data(), rec.size())) || !wr(fd, &ms, 4)) break;
        if (n < 0) break;  // 0.6.31: explicit error, session ends
      } else {
        L("{\"op\":\"error\",\"msg\":\"bad request %d\"}", static_cast<int>(c));
        n_err_++;
        fail("bad request byte " + std::to_string(static_cast<int>(c)));
        break;
      }
    }
  }

  // bicubic descriptor at a (sub-pixel) model-input position, same weights as xfpost_*.h desc (A = -0.75, zero padding)
  template <typename T>
  void sample_desc(const T *o, float x, float y, float *dd) const {
    const int Hc = Hm_ / 8, Wc = Wm_ / 8, hw = Hc * Wc;
    const float sxs = static_cast<float>(Wc) / static_cast<float>(Wm_ - 1), sys = static_cast<float>(Hc) / static_cast<float>(Hm_ - 1);
    const float ix = x * sxs - 0.5f, iy = y * sys - 0.5f;
    const int x0 = static_cast<int>(std::floor(ix)), y0 = static_cast<int>(std::floor(iy));
    const float tx = ix - x0, ty = iy - y0;
    const float wx[4] = {off_cub2(tx + 1.f), off_cub1(tx), off_cub1(1.f - tx), off_cub2(2.f - tx)};
    const float wy[4] = {off_cub2(ty + 1.f), off_cub1(ty), off_cub1(1.f - ty), off_cub2(2.f - ty)};
    for (int c = 0; c < 64; c++) dd[c] = 0.f;
    for (int j = 0; j < 4; j++) {
      const int yy = y0 - 1 + j;
      if (yy < 0 || yy >= Hc) continue;
      for (int k = 0; k < 4; k++) {
        const int xx = x0 - 1 + k;
        if (xx < 0 || xx >= Wc) continue;
        const int c0 = yy * Wc + xx;
        const float w = wy[j] * wx[k] * B_.inv[c0];
        for (int c = 0; c < 64; c++) dd[c] += w * static_cast<float>(o[c * hw + c0]);
      }
    }
    float ss = 0.f;
    for (int c = 0; c < 64; c++) ss += dd[c] * dd[c];
    const float r = 1.f / std::max(std::sqrt(ss), 1e-12f);
    for (int c = 0; c < 64; c++) dd[c] *= r;
  }

  bool feat(int64_t id, int W, int H, const uint8_t *g, int nq, const float *q, std::vector<float> &kp_out, double t_req,
            int32_t &code) {
    kp_out.clear();
    code = 0;
    std::string err;
    if (W != Wm_ || H > Hm_ || H <= Hm_ - 32) {
      n_err_++;
      code = -2;
      L("{\"op\":\"F\",\"id\":%lld,\"error\":\"image %dx%d does not fit the model %dx%d\"}", (long long)id, W, H, Wm_, Hm_);
      fail("image " + std::to_string(W) + "x" + std::to_string(H) + " does not fit the XFeat model");
      return false;
    }
    const double a = now_us();
    // pad (replicate the last row) + /255 + whole-image mean / var normalisation (double sums), into the ENN input buffer
    float *dst = be_->in(xf_);
    double s = 0, s2 = 0;
    for (int y = 0; y < Hm_; y++) {
      const uint8_t *row = g + static_cast<size_t>(std::min(y, H - 1)) * W;
      double rs = 0, rs2 = 0;
      for (int x = 0; x < W; x++) {
        const double v = row[x] / 255.0;
        rs += v;
        rs2 += v * v;
      }
      s += rs;
      s2 += rs2;
    }
    const double npx = static_cast<double>(Hm_) * Wm_, mean = s / npx, var = std::max(0.0, s2 / npx - mean * mean);
    const float r = static_cast<float>(1.0 / std::sqrt(var + 1e-5)), mf = static_cast<float>(mean);
    uint16_t *dst16 = reinterpret_cast<uint16_t *>(dst);
    for (int y = 0; y < Hm_; y++) {
      const uint8_t *row = g + static_cast<size_t>(std::min(y, H - 1)) * W;
      const size_t o0 = static_cast<size_t>(y) * Wm_;
      if (xf_in16_)
        for (int x = 0; x < W; x++) dst16[o0 + x] = f2h((row[x] * (1.0f / 255.0f) - mf) * r);
      else
        for (int x = 0; x < W; x++) dst[o0 + x] = (row[x] * (1.0f / 255.0f) - mf) * r;
    }
    const double b = now_us();
    if (!be_->run(xf_, err)) {
      n_err_++;
      L("{\"op\":\"F\",\"id\":%lld,\"error\":\"npu: %s\"}", (long long)id, err.c_str());
      fail("xfeat EnnExecuteModel: " + err);
      code = -1;
      return false;
    }
    const double c = now_us();
    std::vector<Kp> kp;
    std::vector<float> desc;
    OffT t;
    Entry e;
    e.W = W;
    e.H = H;
    e.nq = nq;
    float dq[64];
    // post-processing straight on the ENN output buffer (float32, or fp16 read through Half)
    auto post = [&](const auto *o) {
      if (xf_soft_) post_soft(o, Hm_, Wm_, B_, kp, desc, c_.topk, t, H);
      else post_off(o, Hm_, Wm_, B_, kp, desc, c_.topk, t, H);
      e.n = nq + static_cast<int>(kp.size());
      e.pts.resize(2 * static_cast<size_t>(e.n));
      e.d.resize(64 * static_cast<size_t>(e.n));
      for (int i = 0; i < nq; i++) {
        e.pts[2 * i] = q[2 * i];
        e.pts[2 * i + 1] = q[2 * i + 1];
        sample_desc(o, q[2 * i], q[2 * i + 1], dq);
        for (int k = 0; k < 64; k++) e.d[64 * static_cast<size_t>(i) + k] = f2h(dq[k]);
      }
    };
    if (xf_out16_) post(reinterpret_cast<const Half *>(be_->out(xf_)));
    else post(be_->out(xf_));
    const double d = now_us();
    kp_out.resize(2 * kp.size());
    for (size_t i = 0; i < kp.size(); i++) {
      const size_t j = nq + i;
      e.pts[2 * j] = kp_out[2 * i] = static_cast<float>(kp[i].x);
      e.pts[2 * j + 1] = kp_out[2 * i + 1] = static_cast<float>(kp[i].y);
      for (int k = 0; k < 64; k++) e.d[64 * j + k] = f2h(desc[64 * i + k]);
    }
    const double f = now_us();
    put(id, std::move(e));
    n_feat_++;
    // us: prep = pad + normalise + input write, npu = EnnExecuteModel, post_q = XFeat post + query descriptors,
    // store = cache copy (fp16), tot = request byte 1 -> answer ready, recv = request byte 1 -> body received; post_us = post stages
    // (soft/shuffle, nms+score, top-k, descriptors)
    L("{\"op\":\"F\",\"id\":%lld,\"t_wall_ms\":%.0f,\"nq\":%d,\"nk\":%d,\"us\":{\"prep\":%.0f,\"npu\":%.0f,\"post_q\":%.0f,"
      "\"store\":%.0f,\"tot\":%.0f,\"recv\":%.0f},\"post_us\":[%.0f,%.0f,%.0f,%.0f],\"skin\":%s,\"ap\":%s,\"bat\":%s,"
      "\"tstat\":%d,\"cache_n\":%zu,\"cache_mb\":%.1f,\"evicted\":%llu}",
      (long long)id, wall_ms(), nq, static_cast<int>(kp.size()), b - a, c - b, d - c, f - d, f - t_req, a - t_req, t.soft, t.nms,
      t.topk, t.desc, jn(skin_.load()).c_str(), jn(ap_.load()).c_str(), jn(bat_.load()).c_str(), tstat_.load(), cache_.size(), cache_bytes_ / 1048576.0,
      (unsigned long long)n_evict_);
    return true;
  }

  void put(int64_t id, Entry &&e) {
    auto it = cache_.find(id);
    if (it != cache_.end()) {
      cache_bytes_ -= bytes(it->second);
      cache_.erase(it);
      order_.erase(std::remove(order_.begin(), order_.end(), id), order_.end());
    }
    cache_bytes_ += bytes(e);
    cache_[id] = std::move(e);
    order_.push_back(id);
    while (cache_bytes_ > c_.cache_mb * 1048576.0 && order_.size() > 1) {
      const int64_t old = order_.front();
      order_.pop_front();
      auto jt = cache_.find(old);
      if (jt != cache_.end()) {
        cache_bytes_ -= bytes(jt->second);
        cache_.erase(jt);
        n_evict_++;
      }
    }
  }
  static double bytes(const Entry &e) { return e.pts.size() * 4.0 + e.d.size() * 2.0 + 64.0; }

  // returns the number of matches; rec = n x (i32 ia, i32 ib, f32 s)
  int32_t match(int64_t a, int64_t b, int mode, std::vector<char> &rec, double t_req) {
    rec.clear();
    auto ia = cache_.find(a), ib = cache_.find(b);
    if (ia == cache_.end() || ib == cache_.end() || !ia->second.n || !ib->second.n) {
      L("{\"op\":\"M\",\"a\":%lld,\"b\":%lld,\"mode\":%d,\"miss\":[%d,%d],\"evicted\":%llu}", (long long)a, (long long)b, mode,
        ia == cache_.end() ? 1 : 0, ib == cache_.end() ? 1 : 0, (unsigned long long)n_evict_);
      n_err_++;
      fail(std::string("keyframe not in the descriptor cache (") + (n_evict_ ? "evicted?" : "never sent") + ")");
      return -3;
    }
    const Entry &A = ia->second, &B = ib->second;
    std::vector<std::array<float, 3>> out;  // (ia, ib, s) as floats then packed
    const double t0 = now_us();
    double t_pack = 0, t_npu = 0, t_post = 0;
    int nv0 = 0, nv1 = 0;
    std::string err;
    if (mode == 1) {
      nn(A, B, out);
      t_post = now_us() - t0;
    } else if (mode == 2) {
      if (!lg(A, B, out, t_pack, t_npu, t_post, nv0, nv1, err)) {
        n_err_++;
        L("{\"op\":\"M\",\"a\":%lld,\"b\":%lld,\"mode\":2,\"error\":\"%s\"}", (long long)a, (long long)b, err.c_str());
        fail("lighterglue EnnExecuteModel: " + err);
        return -1;
      }
      n_lg_++;
    } else {
      L("{\"op\":\"M\",\"a\":%lld,\"b\":%lld,\"mode\":%d,\"error\":\"bad mode\"}", (long long)a, (long long)b, mode);
      n_err_++;
      fail("bad match mode " + std::to_string(mode));
      return -4;
    }
    rec.resize(out.size() * 12);
    for (size_t i = 0; i < out.size(); i++) {
      const int32_t x = static_cast<int32_t>(out[i][0]), y = static_cast<int32_t>(out[i][1]);
      const float s = out[i][2];
      std::memcpy(rec.data() + 12 * i, &x, 4);
      std::memcpy(rec.data() + 12 * i + 4, &y, 4);
      std::memcpy(rec.data() + 12 * i + 8, &s, 4);
    }
    n_match_++;
    L("{\"op\":\"M\",\"a\":%lld,\"b\":%lld,\"mode\":%d,\"t_wall_ms\":%.0f,\"nA\":%d,\"nB\":%d,\"nv\":[%d,%d],\"n\":%zu,"
      "\"us\":{\"pack\":%.0f,\"npu\":%.0f,\"post\":%.0f,\"tot\":%.0f,\"recv\":%.0f},\"skin\":%s,\"ap\":%s}",
      (long long)a, (long long)b, mode, wall_ms(), A.n, B.n, nv0, nv1, out.size(), t_pack, t_npu, t_post, now_us() - t_req,
      t0 - t_req, jn(skin_.load()).c_str(), jn(ap_.load()).c_str());
    return static_cast<int32_t>(out.size());
  }

  // 0.6.35 npunight nn_fast: 64-float dot product, 16 partial sums (4 x float32x4 fused multiply-add), then
  // (q0+q1)+(q2+q3) per lane and a pairwise lane sum (vaddvq). The scalar branch does the same operations in the same order.
  static inline float dot64f(const float *x, const float *y) {
#if defined(__aarch64__)
    float32x4_t a0 = vdupq_n_f32(0.f), a1 = a0, a2 = a0, a3 = a0;
    for (int k = 0; k < 64; k += 16) {
      a0 = vfmaq_f32(a0, vld1q_f32(x + k), vld1q_f32(y + k));
      a1 = vfmaq_f32(a1, vld1q_f32(x + k + 4), vld1q_f32(y + k + 4));
      a2 = vfmaq_f32(a2, vld1q_f32(x + k + 8), vld1q_f32(y + k + 8));
      a3 = vfmaq_f32(a3, vld1q_f32(x + k + 12), vld1q_f32(y + k + 12));
    }
    return vaddvq_f32(vaddq_f32(vaddq_f32(a0, a1), vaddq_f32(a2, a3)));
#else
    float a[4][4] = {};
    for (int k = 0; k < 64; k += 16)
      for (int q = 0; q < 4; q++)
        for (int l = 0; l < 4; l++) a[q][l] = std::fma(x[k + 4 * q + l], y[k + 4 * q + l], a[q][l]);
    float s[4];
    for (int l = 0; l < 4; l++) s[l] = (a[0][l] + a[1][l]) + (a[2][l] + a[3][l]);
    return (s[0] + s[1]) + (s[2] + s[3]);
#endif
  }

  // mutual nearest neighbour on all points (xf_server mode 1), cos >= min_cos
  void nn(const Entry &A, const Entry &B, std::vector<std::array<float, 3>> &out) {
    const int na = A.n, nb = B.n;
    std::vector<float> fa(64 * static_cast<size_t>(na)), fb(64 * static_cast<size_t>(nb));
    for (size_t i = 0; i < fa.size(); i++) fa[i] = h2f(A.d[i]);
    for (size_t i = 0; i < fb.size(); i++) fb[i] = h2f(B.d[i]);
    std::vector<int> m12(na, 0), m21(nb, 0);
    std::vector<float> b12(na, -1e30f), b21(nb, -1e30f);
    for (int i = 0; i < na; i++) {
      const float *x = &fa[64 * static_cast<size_t>(i)];
      for (int j = 0; j < nb; j++) {
        const float *y = &fb[64 * static_cast<size_t>(j)];
        float s;
        if (c_.nn_fast) s = dot64f(x, y);
        else {
          s = 0.f;
          for (int k = 0; k < 64; k++) s += x[k] * y[k];
        }
        if (s > b12[i]) { b12[i] = s; m12[i] = j; }  // first max (torch argmax: first index on ties)
        if (s > b21[j]) { b21[j] = s; m21[j] = i; }
      }
    }
    for (int i = 0; i < na; i++)
      if (m21[m12[i]] == i && b12[i] >= c_.min_cos) out.push_back({static_cast<float>(i), static_cast<float>(m12[i]), b12[i]});
  }

  bool lg(const Entry &A, const Entry &B, std::vector<std::array<float, 3>> &out, double &t_pack, double &t_npu,
          double &t_post, int &nv0, int &nv1, std::string &err) {
    const double a = now_us();
    // point index lists (into the entry lists)
    auto pick = [&](const Entry &E, std::vector<int> &ix) {
      ix.clear();
      if (c_.lg_in == "x512") {
        for (int i = E.nq; i < E.n && static_cast<int>(ix.size()) < N_; i++) ix.push_back(i);
      } else {  // trk768: tracks first, then XFeat
        for (int i = 0; i < E.n && static_cast<int>(ix.size()) < N_; i++) ix.push_back(i);
      }
    };
    std::vector<int> i0, i1;
    pick(A, i0);
    pick(B, i1);
    if (lg_mask_) {  // one mask for both sides: both use the first min(n0, n1) points
      const size_t m = std::min(i0.size(), i1.size());
      i0.resize(m);
      i1.resize(m);
    }
    nv0 = static_cast<int>(i0.size());
    nv1 = static_cast<int>(i1.size());
    if (!nv0 || !nv1) return true;
    const int C = lg_mask_ ? 513 : 512;
    // lg_pad 1 / 2 (unmasked model): fill the N rows by repeating the real points. Copies get identical inputs, so they stay
    // identical through every layer (permutation-equivariant network); lg_pad 2 then sums the dual-softmax probabilities
    // over the copies -> with N = c * n exactly the network on the n points (PC fp32: pad_check.json), otherwise the
    // multiplicities differ by one. The in-model-decision output cannot sum copies -> lg_pad 2 acts as 1 there.
    const bool rep = !lg_mask_ && (c_.lg_pad == 1 || c_.lg_pad == 2);
    const bool csum = rep && c_.lg_pad == 2 && !lg_inmodel_;
    float *x = be_->in(lg_);
    const float sx0 = A.W / 2.f, sy0 = A.H / 2.f, sc0 = std::max(A.W, A.H) / 2.f;
    const float sx1 = B.W / 2.f, sy1 = B.H / 2.f, sc1 = std::max(B.W, B.H) / 2.f;
    for (int r = 0; r < N_; r++) {
      float *row = x + static_cast<size_t>(r) * C;
      for (int side = 0; side < 2; side++) {
        const Entry &E = side ? B : A;
        const std::vector<int> &ix = side ? i1 : i0;
        const int nv = static_cast<int>(ix.size());
        float kx = 0.f, ky = 0.f;  // padding: lg_pad 0 zero descriptor at the centre (cos 1, sin 0); 1 / 2 repeat cyclically
        float *dd = row + 64 * side;
        int src = -1;
        if (r < nv) src = ix[r];
        else if (rep) src = ix[r % nv];
        if (src >= 0) {
          const float sx = side ? sx1 : sx0, sy = side ? sy1 : sy0, sc = side ? sc1 : sc0;
          kx = (E.pts[2 * src] - sx) / sc;
          ky = (E.pts[2 * src + 1] - sy) / sc;
          for (int k = 0; k < 64; k++) dd[k] = h2f(E.d[64 * static_cast<size_t>(src) + k]);
        } else {
          for (int k = 0; k < 64; k++) dd[k] = 0.f;
        }
        float *cs = row + 128 + 192 * side, *sn = cs + 96;
        for (int j = 0; j < 48; j++) {
          const float p = kx * wr_[j] + ky * wr_[48 + j];
          const float cv = std::cos(p), sv = std::sin(p);
          cs[j] = cs[48 + j] = cv;
          sn[j] = sn[48 + j] = sv;
        }
      }
      if (lg_mask_) row[512] = (r < nv0) ? 1.f : 0.f;  // nv0 == nv1 here
    }
    const double b = now_us();
    if (!be_->run(lg_, err)) return false;
    const double c = now_us();
    const float *o = be_->out(lg_);
    const int N = N_;
    if (lg_inmodel_) {  // [N,3]: m0_i, m1_j (at row j), rmax_i
      const double lt = std::log(c_.lg_thr);
      for (int i = 0; i < nv0; i++) {
        const int j = static_cast<int>(std::lround(o[3 * i]));
        if (j < 0 || j >= nv1) continue;
        const int back = static_cast<int>(std::lround(o[3 * j + 1]));
        if (back != i || !(o[3 * i + 2] > lt)) continue;
        out.push_back({static_cast<float>(i0[i]), static_cast<float>(i1[j]), static_cast<float>(std::exp(o[3 * i + 2]))});
      }
    } else if (c_.lg_post == 3) {  // 0.6.35 npunight: product post (nncrun lgpost cpu_prod_bench.cpp postp, PC 13/13 same
      // pairs as lgpost 2): sc_ij = log(p_ij s0_i s1_j) is monotone in the product -> argmax on products, log only per match
      const int R = 2 * N + 2;
      if (csum) {
        a2_.assign(static_cast<size_t>(nv0) * nv1, 0.f);
        b2_.assign(static_cast<size_t>(nv1) * nv0, 0.f);
        for (int i = 0; i < nv0; i++) {
          const float *ar = o + static_cast<size_t>(i) * R;
          float *dst = &a2_[static_cast<size_t>(i) * nv1];
          for (int r = 0; r < N; r++) dst[r % nv1] += ar[r];
        }
        for (int j = 0; j < nv1; j++) {
          const float *br = o + static_cast<size_t>(j) * R + N;
          float *dst = &b2_[static_cast<size_t>(j) * nv0];
          for (int r = 0; r < N; r++) dst[r % nv0] += br[r];
        }
      }
      p3_.resize(static_cast<size_t>(nv0) * nv1);
      s0v_.resize(nv0);
      s1v_.resize(nv1);
      for (int i = 0; i < nv0; i++) s0v_[i] = 1.f / (1.f + std::exp(-o[static_cast<size_t>(i) * R + 2 * N]));
      for (int j = 0; j < nv1; j++) s1v_[j] = 1.f / (1.f + std::exp(-o[static_cast<size_t>(j) * R + 2 * N + 1]));
      std::vector<int> m0(nv0, 0), m1(nv1, 0);
      std::vector<float> cb(nv1, -1.f);
      for (int i = 0; i < nv0; i++) {
        float best = -1.f;
        int bj = 0;
        float *pr = &p3_[static_cast<size_t>(i) * nv1];
        for (int j = 0; j < nv1; j++) {
          const float pp = csum ? a2_[static_cast<size_t>(i) * nv1 + j] * b2_[static_cast<size_t>(j) * nv0 + i]
                                : o[static_cast<size_t>(i) * R + j] * o[static_cast<size_t>(j) * R + N + i];
          pr[j] = pp;
          const float q = pp * s1v_[j];
          if (q > best) { best = q; bj = j; }
        }
        m0[i] = bj;
      }
      for (int i = 0; i < nv0; i++) {
        const float s = s0v_[i];
        const float *pr = &p3_[static_cast<size_t>(i) * nv1];
        for (int j = 0; j < nv1; j++) {
          const float q = pr[j] * s;
          if (q > cb[j]) { cb[j] = q; m1[j] = i; }
        }
      }
      const double lt = std::log(c_.lg_thr);
      for (int i = 0; i < nv0; i++) {
        const int j = m0[i];
        if (m1[j] != i) continue;
        const double r = std::log(std::max(static_cast<double>(p3_[static_cast<size_t>(i) * nv1 + j] * s1v_[j]), 1e-30)) +
                         std::log(static_cast<double>(s0v_[i]));
        if (r > lt) out.push_back({static_cast<float>(i0[i]), static_cast<float>(i1[j]), static_cast<float>(std::exp(r))});
      }
    } else {  // [N, 2N+2] = A | BT | z0 | z1 (lgpost 2), argmax over the real points only
      const int R = 2 * N + 2;
      sc_.resize(static_cast<size_t>(nv0) * nv1);
      l1_.resize(nv1);
      for (int j = 0; j < nv1; j++) l1_[j] = logsig(o[static_cast<size_t>(j) * R + 2 * N + 1]);
      if (csum) {  // A2[i][j] = sum over copies r = j (mod nv1) of A[i][r]; BT2[j][i] = sum over copies r = i (mod nv0) of BT[j][r]
        a2_.assign(static_cast<size_t>(nv0) * nv1, 0.f);
        b2_.assign(static_cast<size_t>(nv1) * nv0, 0.f);
        for (int i = 0; i < nv0; i++) {
          const float *ar = o + static_cast<size_t>(i) * R;
          float *dst = &a2_[static_cast<size_t>(i) * nv1];
          for (int r = 0; r < N; r++) dst[r % nv1] += ar[r];
        }
        for (int j = 0; j < nv1; j++) {
          const float *br = o + static_cast<size_t>(j) * R + N;
          float *dst = &b2_[static_cast<size_t>(j) * nv0];
          for (int r = 0; r < N; r++) dst[r % nv0] += br[r];
        }
      }
      for (int i = 0; i < nv0; i++) {
        const double l0 = logsig(o[static_cast<size_t>(i) * R + 2 * N]);
        for (int j = 0; j < nv1; j++) {
          const float pp = csum ? a2_[static_cast<size_t>(i) * nv1 + j] * b2_[static_cast<size_t>(j) * nv0 + i]
                                : o[static_cast<size_t>(i) * R + j] * o[static_cast<size_t>(j) * R + N + i];
          sc_[static_cast<size_t>(i) * nv1 + j] = std::log(std::max(static_cast<double>(pp), 1e-30)) + l0 + l1_[j];
        }
      }
      std::vector<int> m0(nv0), m1(nv1);
      std::vector<double> rmax(nv0);
      for (int i = 0; i < nv0; i++) {
        int bj = 0;
        const double *s = &sc_[static_cast<size_t>(i) * nv1];
        for (int j = 1; j < nv1; j++)
          if (s[j] > s[bj]) bj = j;
        m0[i] = bj;
        rmax[i] = s[bj];
      }
      for (int j = 0; j < nv1; j++) {
        int bi = 0;
        for (int i = 1; i < nv0; i++)
          if (sc_[static_cast<size_t>(i) * nv1 + j] > sc_[static_cast<size_t>(bi) * nv1 + j]) bi = i;
        m1[j] = bi;
      }
      for (int i = 0; i < nv0; i++)
        if (m1[m0[i]] == i && std::exp(rmax[i]) > c_.lg_thr)
          out.push_back({static_cast<float>(i0[i]), static_cast<float>(i1[m0[i]]), static_cast<float>(std::exp(rmax[i]))});
    }
    const double d = now_us();
    t_pack = b - a;
    t_npu = c - b;
    t_post = d - c;
    return true;
  }

  Backend *be_;
  Cfg c_;
  FILE *log_ = nullptr;
  std::mutex logm_;
  std::string start_log_;
  std::thread th_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> done_{false};
  std::atomic<bool> stuck_{false};
  std::mutex fail_m_;
  std::string fail_;
  void fail(const std::string &w) {  // 0.6.31: first server failure of the run (the app marks the run INVALID)
    std::lock_guard<std::mutex> l(fail_m_);
    if (fail_.empty()) fail_ = w + " at t_wall_ms " + std::to_string(static_cast<long long>(wall_ms()));
  }
  std::mutex st_m_;
  std::condition_variable st_cv_;
  int st_state_ = 0;  // start handshake: 0 starting, 1 ok, 2 failed
  std::string st_err_;
  bool running_ = false;
  std::atomic<int> cli_fd_{-1};
  int lfd_ = -1;
  int n_sess_ = 0;
  int xf_ = -1, lg_ = -1;
  int Wm_ = 960, Hm_ = 544, N_ = 512;
  bool xf_soft_ = true, lg_mask_ = false, lg_inmodel_ = false;
  bool xf_in16_ = false, xf_out16_ = false;  // XFeat ENN buffers fp16 instead of float32
  float wr_[96];
  OffBuf B_;
  std::map<int64_t, Entry> cache_;
  std::deque<int64_t> order_;
  double cache_bytes_ = 0;
  std::vector<double> sc_, l1_;
  std::vector<float> a2_, b2_;
  std::vector<float> p3_, s0v_, s1v_;  // 0.6.35 lg_post 3
  uint64_t n_feat_ = 0, n_match_ = 0, n_lg_ = 0, n_err_ = 0, n_evict_ = 0;
  std::atomic<float> skin_{NAN}, ap_{NAN}, bat_{NAN};
  std::atomic<int> tstat_{-1};
};

}  // namespace xfnpu
