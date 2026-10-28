// enn_backend.h (appnpu 2026-10-28): xfnpu::Backend on the phone NPU through the vendor public library
// libenn_public_api_cpp.so (listed in /vendor/etc/public.libraries.txt; the app declares it with <uses-native-library
// required="false">), dlopen-ed inside the app process - the same calls as nncrun (ovxfeat runs/npu_20261027/nncrun,
// own code: function names = the C++ symbols the phone library exports, argument lists after the public ExecuTorch (BSD) /
// LiteRT (Apache-2.0) ENN loaders; no Samsung header or binary copied). No CPU / GPU fallback: the only compute call is
// EnnExecuteModel on a precompiled .nnc; any failure -> the caller turns the feature off.
#pragma once
#include <dlfcn.h>

#include <string>
#include <vector>

#include "xfnpu_core.h"

namespace xfnpu {

class EnnBackend : public Backend {
public:
  explicit EnnBackend(int perf = -1) : perf_(perf) {}
  ~EnnBackend() override { close_all(); }

  bool init(std::string &err) override {
    h_ = dlopen("libenn_public_api_cpp.so", RTLD_NOW | RTLD_LOCAL);
    if (!h_) {
      err = std::string("dlopen libenn_public_api_cpp.so: ") + dlerror();
      return false;
    }
#define XFNPU_SYM(var, name)                                   \
  var = reinterpret_cast<decltype(var)>(dlsym(h_, name));      \
  if (!var) {                                                  \
    err = std::string("dlsym ") + name;                        \
    return false;                                              \
  }
    XFNPU_SYM(eInit, "_ZN3enn3api13EnnInitializeEv");
    XFNPU_SYM(eDeinit, "_ZN3enn3api15EnnDeinitializeEv");
    XFNPU_SYM(eOpen, "_ZN3enn3api12EnnOpenModelEPKcPm");
    XFNPU_SYM(eClose, "_ZN3enn3api13EnnCloseModelEm");
    XFNPU_SYM(eAlloc, "_ZN3enn3api21EnnAllocateAllBuffersEmPPP10_ennBufferP20_NumberOfBuffersInfoib");
    XFNPU_SYM(eRelBufs, "_ZN3enn3api17EnnReleaseBuffersEPP10_ennBufferi");
    XFNPU_SYM(eExec, "_ZN3enn3api15EnnExecuteModelEmi");
    XFNPU_SYM(eSetPerf, "_ZN3enn3api24EnnSetPreferencePerfModeEj");
#undef XFNPU_SYM
    const int r = eInit();
    if (r != 0) {
      err = "EnnInitialize -> " + std::to_string(r);
      return false;
    }
    inited_ = true;
    if (perf_ >= 0) desc_ += "perf " + std::to_string(perf_) + " -> " + std::to_string(eSetPerf(static_cast<uint32_t>(perf_))) + "; ";
    return true;
  }

  int load(const std::string &path, std::string &err) override {
    M m;
    const double t0 = now_us();
    int r = eOpen(path.c_str(), &m.id);
    if (r != 0) {
      err = "EnnOpenModel(" + path + ") -> " + std::to_string(r);
      return -1;
    }
    unsigned char nbi[32];
    std::memset(nbi, 0, sizeof nbi);
    r = eAlloc(m.id, &m.bufs, nbi, 0, true);
    uint32_t a0, a1, a2;
    std::memcpy(&a0, nbi, 4);
    std::memcpy(&a1, nbi + 4, 4);
    std::memcpy(&a2, nbi + 8, 4);
    m.nin = a0;
    m.nout = a1 ? a1 : a2;  // NumberOfBuffersInfo read defensively (as nncrun)
    if (r != 0 || !m.bufs || m.nin != 1 || m.nout != 1) {
      err = "EnnAllocateAllBuffers(" + path + ") -> " + std::to_string(r) + " (in " + std::to_string(m.nin) + " out " +
            std::to_string(m.nout) + ", want 1 / 1)";
      if (r == 0 && m.bufs) eRelBufs(m.bufs, static_cast<int>(m.nin + m.nout));
      eClose(m.id);
      return -1;
    }
    m.in = m.bufs[0];
    m.out = m.bufs[m.nin];
    desc_ += path + ": open+alloc " + std::to_string(static_cast<int>((now_us() - t0) / 1000)) + " ms, in " +
             std::to_string(m.in->size) + " B, out " + std::to_string(m.out->size) + " B; ";
    ms_.push_back(m);
    return static_cast<int>(ms_.size()) - 1;
  }
  float *in(int m) override { return static_cast<float *>(ms_[m].in->va); }
  size_t in_bytes(int m) override { return ms_[m].in->size; }
  const float *out(int m) override { return static_cast<const float *>(ms_[m].out->va); }
  size_t out_bytes(int m) override { return ms_[m].out->size; }
  bool run(int m, std::string &err) override {
    const int r = eExec(ms_[m].id, 0);
    if (r != 0) {
      err = "EnnExecuteModel -> " + std::to_string(r);
      return false;
    }
    return true;
  }
  void close_all() override {
    for (auto &m : ms_) {
      if (m.bufs) eRelBufs(m.bufs, static_cast<int>(m.nin + m.nout));
      eClose(m.id);
    }
    ms_.clear();
    if (inited_) eDeinit();
    inited_ = false;
    // the library stays loaded (dlclose of vendor code with live threads is not safe)
  }
  std::string describe() override { return "ENN (libenn_public_api_cpp.so) " + desc_; }

private:
  struct EnnBuf {
    void *va;
    uint32_t size;
    uint32_t offset;
  };
  struct M {
    uint64_t id = 0;
    EnnBuf **bufs = nullptr;
    uint32_t nin = 0, nout = 0;
    EnnBuf *in = nullptr, *out = nullptr;
  };
  int perf_;
  void *h_ = nullptr;
  bool inited_ = false;
  std::string desc_;
  std::vector<M> ms_;
  int (*eInit)() = nullptr;
  int (*eDeinit)() = nullptr;
  int (*eOpen)(const char *, uint64_t *) = nullptr;
  int (*eClose)(uint64_t) = nullptr;
  int (*eAlloc)(uint64_t, EnnBuf ***, void *, int, bool) = nullptr;
  int (*eRelBufs)(EnnBuf **, int) = nullptr;
  int (*eExec)(uint64_t, int) = nullptr;
  int (*eSetPerf)(uint32_t) = nullptr;
};

}  // namespace xfnpu
