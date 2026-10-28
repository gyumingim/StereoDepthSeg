// xfnpu_pc.cpp (appnpu 2026-10-28): PC TEST build of the same xfnpu::Server (xfnpu_core.h), the NPU replaced by a stand-in
// process (npu_emu.py: official XFeat + lgnpu L6 in torch fp32) over two pipes. Only for checking the protocol, the
// pre / post-processing and the bridge integration on a PC replay - NOT the phone accuracy or timing.
//   xfnpu_pc SOCKNAME XF_MODEL LG_MODEL WR LOG "EMU CMD" [topk] [lg_in] [lg_pad]
//   model names: pc:xfsoft (out 129), pc:xfoff (out 130), pc:lg512 (A|BT|z0|z1)
// stand-in protocol: 'L' u32 len, path -> u32 in_bytes, u32 out_bytes ;  'X' i32 model, in bytes -> out bytes
#include <csignal>
#include <iostream>
#include <sys/wait.h>

#include "xfnpu_core.h"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void on_sig(int) { g_stop = 1; }

class PipeBackend : public xfnpu::Backend {
public:
  explicit PipeBackend(std::string cmd) : cmd_(std::move(cmd)) {}
  bool init(std::string &err) override {
    int a[2], b[2];
    if (pipe(a) || pipe(b)) {
      err = "pipe";
      return false;
    }
    pid_ = fork();
    if (pid_ == 0) {
      dup2(a[0], 0);
      dup2(b[1], 1);
      close(a[0]); close(a[1]); close(b[0]); close(b[1]);
      execl("/bin/sh", "sh", "-c", cmd_.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    close(a[0]);
    close(b[1]);
    to_ = a[1];
    from_ = b[0];
    return true;
  }
  int load(const std::string &path, std::string &err) override {
    const char c = 'L';
    const uint32_t n = static_cast<uint32_t>(path.size());
    uint32_t sz[2];
    if (!w(&c, 1) || !w(&n, 4) || !w(path.data(), n) || !r(sz, 8) || !sz[0] || !sz[1]) {
      err = "stand-in load " + path + " failed";
      return -1;
    }
    M m;
    m.in.assign(sz[0] / 4, 0.f);
    m.out.assign(sz[1] / 4, 0.f);
    ms_.push_back(std::move(m));
    return static_cast<int>(ms_.size()) - 1;
  }
  float *in(int m) override { return ms_[m].in.data(); }
  size_t in_bytes(int m) override { return ms_[m].in.size() * 4; }
  const float *out(int m) override { return ms_[m].out.data(); }
  size_t out_bytes(int m) override { return ms_[m].out.size() * 4; }
  bool run(int m, std::string &err) override {
    const char c = 'X';
    const int32_t id = m;
    if (!w(&c, 1) || !w(&id, 4) || !w(ms_[m].in.data(), in_bytes(m)) || !r(ms_[m].out.data(), out_bytes(m))) {
      err = "stand-in run failed";
      return false;
    }
    return true;
  }
  void close_all() override {
    if (to_ >= 0) {
      const char q = 'Q';
      (void)!::write(to_, &q, 1);
      close(to_);
      to_ = -1;
    }
    if (from_ >= 0) close(from_);
    from_ = -1;
    if (pid_ > 0) waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }
  std::string describe() override { return "PC torch stand-in: " + cmd_; }

private:
  bool w(const void *p, size_t n) {
    const char *c = static_cast<const char *>(p);
    while (n) {
      ssize_t k = ::write(to_, c, n);
      if (k <= 0) return false;
      c += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }
  bool r(void *p, size_t n) {
    char *c = static_cast<char *>(p);
    while (n) {
      ssize_t k = ::read(from_, c, n);
      if (k <= 0) return false;
      c += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }
  struct M {
    std::vector<float> in, out;
  };
  std::string cmd_;
  int to_ = -1, from_ = -1;
  pid_t pid_ = -1;
  std::vector<M> ms_;
};
}  // namespace

int main(int argc, char **argv) {
  if (argc < 7) {
    std::fprintf(stderr, "usage: xfnpu_pc SOCK XF LG WR LOG \"EMU CMD\" [topk] [lg_in] [lg_pad]\n");
    return 2;
  }
  std::signal(SIGINT, on_sig);
  std::signal(SIGTERM, on_sig);
  std::signal(SIGPIPE, SIG_IGN);
  xfnpu::Cfg c;
  c.sock = argv[1];
  c.xf_model = argv[2];
  c.lg_model = argv[3];
  c.wr_path = argv[4];
  c.log_path = argv[5];
  if (argc > 7) c.topk = std::atoi(argv[7]);
  if (argc > 8) c.lg_in = argv[8];
  if (argc > 9) c.lg_pad = std::atoi(argv[9]);
  if (const char *o = std::getenv("XFNPU_OPTS")) {  // 0.6.35 npunight: nn_fast=0|1,lg_post=2|3
    std::string s(o);
    size_t a = 0;
    while (a < s.size()) {
      size_t b = s.find(',', a);
      if (b == std::string::npos) b = s.size();
      const std::string t = s.substr(a, b - a);
      const size_t e = t.find('=');
      if (e != std::string::npos) {
        if (t.compare(0, e, "nn_fast") == 0) c.nn_fast = std::atoi(t.c_str() + e + 1);
        if (t.compare(0, e, "lg_post") == 0) c.lg_post = std::atoi(t.c_str() + e + 1);
        if (t.compare(0, e, "prep_fast") == 0) c.prep_fast = std::atoi(t.c_str() + e + 1);
      }
      a = b + 1;
    }
  }
  c.nice = 0;
  PipeBackend be(argv[6]);
  xfnpu::Server s(&be);
  std::string err;
  if (!s.start(c, err)) {
    std::fprintf(stderr, "[xfnpu_pc] start failed: %s\n%s", err.c_str(), s.start_log().c_str());
    return 1;
  }
  std::fprintf(stderr, "[xfnpu_pc] ready\n%s", s.start_log().c_str());
  std::fflush(stderr);
  while (!g_stop) usleep(100000);
  s.stop();
  std::fprintf(stderr, "[xfnpu_pc] stopped\n");
  return 0;
}
