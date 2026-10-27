// nncrun 2026-10-27: run an Exynos AI Studio .nnc model on the phone through the device's public vendor library
// libenn_public_api_cpp.so (listed in /vendor/etc/public.libraries.txt), loaded with dlopen inside an app process
// (untrusted_app). Own code: the function names are the C++ symbols exported by the phone library (llvm-nm -D,
// namespace enn::api); argument lists follow the public ExecuTorch (BSD) / LiteRT (Apache-2.0) ENN loaders and the
// mangled names. No Samsung header or binary is copied into this project. No CPU fallback path exists in this code:
// the only compute call is EnnExecuteModel on the precompiled NNC.
// args (space separated key=value): model=PATH in=PATH (N frames concatenated, raw input-buffer bytes) out=PATH
//   warm=N iters=N reps=N perf=MODE(-1 = do not set) open=file|mem msg=0|1 frames=N(max frames to run)
#include <android/log.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#ifdef NNCRUN_JNI
#include <jni.h>
#endif

static std::string g_out;
static FILE *g_f = nullptr;
static void L(const char *fmt, ...) {
  char b[2048];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  __android_log_print(ANDROID_LOG_INFO, "nncrun", "%s", b);
#ifndef NNCRUN_JNI
  std::printf("%s\n", b);
#endif
  if (g_f) {
    std::fprintf(g_f, "%s\n", b);
    std::fflush(g_f);
  }
  g_out += b;
  g_out += "\n";
}

// minimal own declarations (layouts as in the public loaders; NumberOfBuffersInfo / EnnBufferInfo read defensively)
struct EnnBuf {
  void *va;
  uint32_t size;
  uint32_t offset;
};
typedef int (*fn_v)();
typedef int (*fn_open)(const char *, uint64_t *);
typedef int (*fn_openmem)(const char *, uint32_t, uint64_t *);
typedef int (*fn_id)(uint64_t);
typedef int (*fn_alloc)(uint64_t, EnnBuf ***, void *, int, bool);
typedef int (*fn_relbufs)(EnnBuf **, int);
typedef int (*fn_getbufsinfo)(void *, uint64_t);
typedef int (*fn_getbufinfo)(void *, uint64_t, int, uint32_t);
typedef int (*fn_exec)(uint64_t, int);
typedef int (*fn_u32)(uint32_t);
typedef int (*fn_pu32)(uint32_t *);
typedef int (*fn_meta)(int, uint64_t, char *);

static double now_us() {
  return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static bool read_file(const std::string &p, std::vector<char> &d) {
  FILE *f = std::fopen(p.c_str(), "rb");
  if (!f) return false;
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  d.resize(n);
  size_t r = std::fread(d.data(), 1, n, f);
  std::fclose(f);
  return (long)r == n;
}
static double pct(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = (size_t)std::min<double>(v.size() - 1, q * (v.size() - 1) + 0.5);
  return v[i];
}

// ---- keyframe pipeline (pipe=1): gray [0,1] float 352x640 -> per-image normalise -> 3ch into ENN input buffer ->
// EnnExecuteModel -> memcpy output -> XFeat post on CPU (same as acc_lib.py: softmax 65, drop dustbin, pixel-shuffle 8,
// 5x5 NMS, thr 0.05, score = K * sigmoid(heat logit) at the cell, top 1024, 64-d desc at the nearest 1/8 cell, L2 norm)
static const int PH = 352, PW = 640, Ph = 44, Pw = 80;
struct Kp {
  float s;
  int x, y;
};
static void xfeat_post(const float *o, std::vector<float> &K, std::vector<Kp> &kp, std::vector<float> &desc, int topn) {
  const int hw = Ph * Pw;
  K.assign(PH * PW, 0.f);
  for (int cy = 0; cy < Ph; cy++)
    for (int cx = 0; cx < Pw; cx++) {
      int c0 = cy * Pw + cx;
      float m = -1e30f;
      for (int c = 0; c < 65; c++) m = std::max(m, o[(64 + c) * hw + c0]);
      float e[65], s = 0;
      for (int c = 0; c < 65; c++) {
        e[c] = std::exp(o[(64 + c) * hw + c0] - m);
        s += e[c];
      }
      for (int c = 0; c < 64; c++) K[(cy * 8 + c / 8) * PW + cx * 8 + c % 8] = e[c] / s;
    }
  kp.clear();
  for (int y = 0; y < PH; y++)
    for (int x = 0; x < PW; x++) {
      float v = K[y * PW + x];
      if (v <= 0.05f) continue;
      bool mx = true;
      for (int dy = -2; dy <= 2 && mx; dy++)
        for (int dx = -2; dx <= 2; dx++) {
          int yy = y + dy, xx = x + dx;
          if (yy < 0 || xx < 0 || yy >= PH || xx >= PW) continue;
          if (K[yy * PW + xx] > v) {
            mx = false;
            break;
          }
        }
      if (!mx) continue;
      float hl = o[129 * hw + (y / 8) * Pw + x / 8];
      kp.push_back({v / (1.f + std::exp(-hl)), x, y});
    }
  size_t n = std::min<size_t>(topn, kp.size());
  std::partial_sort(kp.begin(), kp.begin() + n, kp.end(), [](const Kp &a, const Kp &b) { return a.s > b.s; });
  kp.resize(n);
  desc.assign(n * 64, 0.f);
  for (size_t i = 0; i < n; i++) {
    int c0 = (kp[i].y / 8) * Pw + kp[i].x / 8;
    float ss = 0;
    for (int c = 0; c < 64; c++) {
      float v = o[c * hw + c0];
      desc[i * 64 + c] = v;
      ss += v * v;
    }
    float r = 1.f / std::sqrt(std::max(ss, 1e-12f));
    for (int c = 0; c < 64; c++) desc[i * 64 + c] *= r;
  }
}

#include "xfpost.h"
#include "lgpipe.h"

static void run_main(const std::string &args, const std::string &dir) {
  std::map<std::string, std::string> A;
  {
    std::istringstream is(args);
    std::string t;
    while (is >> t) {
      auto e = t.find('=');
      if (e != std::string::npos) A[t.substr(0, e)] = t.substr(e + 1);
    }
  }
  auto S = [&](const char *k, const char *d) { return A.count(k) ? A[k] : std::string(d); };
  auto I = [&](const char *k, int d) { return A.count(k) ? std::atoi(A[k].c_str()) : d; };
  auto P = [&](const std::string &p) { return (p.empty() || p[0] == '/') ? p : dir + "/" + p; };
  std::string model = P(S("model", "model.nnc")), in = P(S("in", "")), out = P(S("out", "")),
              logp = P(S("log", "nncrun_log.txt"));
  int warm = I("warm", 30), iters = I("iters", 200), reps = I("reps", 2), perf = I("perf", -1),
      msg = I("msg", 0), maxfr = I("frames", 1000);
  std::string openm = S("open", "file");
  g_f = std::fopen(logp.c_str(), "w");
  L("nncrun start args='%s' dir=%s pid=%d uid=%d", args.c_str(), dir.c_str(), getpid(), getuid());
  {
    char ctx[256] = {0};
    int fd = open("/proc/self/attr/current", O_RDONLY);
    if (fd >= 0) {
      read(fd, ctx, sizeof ctx - 1);
      close(fd);
    }
    L("selinux context: %s", ctx);
  }
  void *h = dlopen("libenn_public_api_cpp.so", RTLD_NOW | RTLD_LOCAL);
  if (!h) {
    L("ERROR dlopen libenn_public_api_cpp.so: %s", dlerror());
    return;
  }
  L("dlopen libenn_public_api_cpp.so OK");
#define SYM(var, type, name)                            \
  type var = (type)dlsym(h, name);                      \
  if (!var) {                                           \
    L("ERROR dlsym %s: %s", name, dlerror());          \
    return;                                             \
  }
  SYM(eInit, fn_v, "_ZN3enn3api13EnnInitializeEv");
  SYM(eDeinit, fn_v, "_ZN3enn3api15EnnDeinitializeEv");
  SYM(eOpen, fn_open, "_ZN3enn3api12EnnOpenModelEPKcPm");
  SYM(eOpenMem, fn_openmem, "_ZN3enn3api22EnnOpenModelFromMemoryEPKcjPm");
  SYM(eClose, fn_id, "_ZN3enn3api13EnnCloseModelEm");
  SYM(eAlloc, fn_alloc, "_ZN3enn3api21EnnAllocateAllBuffersEmPPP10_ennBufferP20_NumberOfBuffersInfoib");
  SYM(eRelBufs, fn_relbufs, "_ZN3enn3api17EnnReleaseBuffersEPP10_ennBufferi");
  SYM(eGetBufsInfo, fn_getbufsinfo, "_ZN3enn3api17EnnGetBuffersInfoEP20_NumberOfBuffersInfom");
  SYM(eGetBufInfo, fn_getbufinfo, "_ZN3enn3api23EnnGetBufferInfoByIndexEP14_ennBufferInfom14_enn_buf_dir_ej");
  SYM(eExec, fn_exec, "_ZN3enn3api15EnnExecuteModelEmi");
  SYM(eSetPerf, fn_u32, "_ZN3enn3api24EnnSetPreferencePerfModeEj");
  SYM(eGetPerf, fn_pu32, "_ZN3enn3api24EnnGetPreferencePerfModeEPj");
  SYM(eMeta, fn_meta, "_ZN3enn3api14EnnGetMetaInfoE19_enn_meta_type_id_emPc");
  SYM(eMsgOn, fn_v, "_ZN3enn3api21EnnSetExecMsgAlwaysOnEv");
  int r = eInit();
  L("EnnInitialize -> %d", r);
  if (r != 0) return;
  char ms[512];
  for (int id : {110, 111}) {
    memset(ms, 0, sizeof ms);
    int rr = eMeta(id, 0, ms);
    L("meta %d -> %d '%s'", id, rr, ms);
  }
  if (msg) L("EnnSetExecMsgAlwaysOn -> %d", eMsgOn());
  if (perf >= 0) L("EnnSetPreferencePerfMode(%d) -> %d", perf, eSetPerf((uint32_t)perf));
  uint32_t pm = 999;
  L("EnnGetPreferencePerfMode -> %d val=%u", eGetPerf(&pm), pm);

  uint64_t mid = 0;
  std::vector<char> mdata;
  double t0 = now_us();
  if (openm == "mem") {
    if (!read_file(model, mdata)) {
      L("ERROR read model %s", model.c_str());
      eDeinit();
      return;
    }
    r = eOpenMem(mdata.data(), (uint32_t)mdata.size(), &mid);
  } else {
    r = eOpen(model.c_str(), &mid);
  }
  L("EnnOpenModel(%s, %s) -> %d id=0x%llx (%.1f ms)", openm.c_str(), model.c_str(), r, (unsigned long long)mid,
    (now_us() - t0) / 1000);
  if (r != 0) {
    eDeinit();
    return;
  }
  for (int id : {120, 121, 122, 123, 124, 130, 131, 132, 133}) {
    memset(ms, 0, sizeof ms);
    int rr = eMeta(id, mid, ms);
    L("meta %d -> %d '%s'", id, rr, ms);
  }
  unsigned char nbi[32];
  memset(nbi, 0, sizeof nbi);
  r = eGetBufsInfo(nbi, mid);
  uint32_t a0, a1, a2;
  memcpy(&a0, nbi, 4);
  memcpy(&a1, nbi + 4, 4);
  memcpy(&a2, nbi + 8, 4);
  L("EnnGetBuffersInfo -> %d raw u32[0..2]=%u %u %u", r, a0, a1, a2);
  uint32_t nin = a0, nout = a1 ? a1 : a2;
  for (int dirn = 0; dirn < 2; dirn++) {
    uint32_t n = dirn == 0 ? nin : nout;
    for (uint32_t i = 0; i < n; i++) {
      unsigned char bi[64];
      memset(bi, 0, sizeof bi);
      int rr = eGetBufInfo(bi, mid, dirn, i);
      uint32_t u[7];
      memcpy(u, bi + 4, sizeof u);
      const char *lab = nullptr;
      memcpy(&lab, bi + 32, sizeof lab);
      L("BufferInfo %s[%u] -> %d upd=%d n=%u w=%u h=%u c=%u size=%u type=%u label=%s", dirn ? "out" : "in", i, rr,
        (int)bi[0], u[0], u[1], u[2], u[3], u[4], u[5], lab ? lab : "(null)");
    }
  }
  EnnBuf **bufs = nullptr;
  unsigned char nbi2[32];
  memset(nbi2, 0, sizeof nbi2);
  t0 = now_us();
  r = eAlloc(mid, &bufs, nbi2, 0, true);
  memcpy(&a0, nbi2, 4);
  memcpy(&a1, nbi2 + 4, 4);
  memcpy(&a2, nbi2 + 8, 4);
  L("EnnAllocateAllBuffers -> %d raw %u %u %u (%.1f ms)", r, a0, a1, a2, (now_us() - t0) / 1000);
  if (r != 0 || !bufs) {
    eClose(mid);
    eDeinit();
    return;
  }
  int ntot = (int)(nin + nout);
  for (int i = 0; i < ntot; i++) L("buf[%d] va=%p size=%u offset=%u", i, bufs[i]->va, bufs[i]->size, bufs[i]->offset);
  EnnBuf *bi0 = bufs[0], *bo0 = bufs[nin];
  if (I("lgpipe", 0)) {
    lg_pipe(I, S, P, in, out, bi0, bo0, mid, eExec);
    L("EnnReleaseBuffers -> %d", eRelBufs(bufs, ntot));
    L("EnnCloseModel -> %d", eClose(mid));
    L("EnnDeinitialize -> %d", eDeinit());
    L("nncrun end");
    return;
  }
  if (I("pipe", 0)) {
    // keyframe cost: in = gray float [0,1] frames 1x352x640; passes over all frames; pass 0 = first exec per frame
    std::vector<char> gd;
    if (!read_file(in, gd)) {
      L("ERROR read %s", in.c_str());
    } else {
      size_t fs = PH * PW * 4, nf = gd.size() / fs;
      int passes = I("passes", 10);
      L("pipe: %zu gray frames, passes=%d (frames=%zu each), in buf %u out buf %u", nf, passes, nf, bi0->size, bo0->size);
      // ch=1|3 from the input buffer size; fp16 output (2 bytes/elem) converted to float in the "out" stage
      const int ch = bi0->size == (uint32_t)(PH * PW * 4) ? 1 : 3;
      const size_t nout_el = (size_t)I("oc", 130) * Ph * Pw;  // oc=129 for the post-in-model xpsoft output
      const bool o16 = bo0->size == nout_el * 2;
      L("pipe: input channels %d, output %s", ch, o16 ? "fp16" : "fp32");
      std::vector<float> outv(nout_el), K, desc, Pb;
      std::vector<Kp> kp;
      std::vector<Cand> cand;
      std::vector<std::vector<Cand>> cands;
      // post=0: original xfeat_post (after memcpy/convert), 1: post_v1 stage-timed (after memcpy/convert),
      // 2: post_v2 on the ENN output buffer directly (no memcpy), threads=N, cpus=a,b (affinity, optional)
      const int postv = I("post", 0), nthr = I("threads", 1);
      std::vector<int> cpus;
      {
        std::string cs = S("cpus", "");
        std::istringstream is(cs);
        std::string t;
        while (std::getline(is, t, ',')) cpus.push_back(std::atoi(t.c_str()));
      }
      Pool pool;
      g_skip = I("skip", 0) != 0;
      pool.start(postv == 2 ? nthr : 1, cpus);
      L("pipe: post=%d threads=%d cpus=%s", postv, pool.n, S("cpus", "-").c_str());
      FILE *fk = out.empty() ? nullptr : std::fopen(out.c_str(), "wb");
      for (int ps = 0; ps < passes; ps++) {
        std::vector<double> tn, tx, to, tp, tt;
        PostT pt;
        for (size_t k = 0; k < nf; k++) {
          const float *g = (const float *)(gd.data() + k * fs);
          double a = now_us();
          double s = 0, s2 = 0;
          for (int i = 0; i < PH * PW; i++) {
            s += g[i];
            s2 += (double)g[i] * g[i];
          }
          double mean = s / (PH * PW), var = s2 / (PH * PW) - mean * mean;
          float r = (float)(1.0 / std::sqrt(var + 1e-5)), mf = (float)mean;
          float *dst = (float *)bi0->va;
          for (int i = 0; i < PH * PW; i++) dst[i] = (g[i] - mf) * r;
          if (ch == 3) {
            memcpy(dst + PH * PW, dst, PH * PW * 4);
            memcpy(dst + 2 * PH * PW, dst, PH * PW * 4);
          }
          double b = now_us();
          int rr = eExec(mid, 0);
          double c = now_us();
          if (postv != 2 && postv != 4) {
            if (o16) {
              const __fp16 *h16 = (const __fp16 *)bo0->va;
              for (size_t j = 0; j < nout_el; j++) outv[j] = (float)h16[j];
            } else {
              memcpy(outv.data(), bo0->va, nout_el * 4);
            }
          }
          double d = now_us();
          if (postv == 4) {  // xpsoft layout: feats 0-63 | softmax probs 64-127 | sigmoid heat 128, read in place
            if (o16) post_soft((const __fp16 *)bo0->va, K, cand, kp, desc, 1024, pt);
            else post_soft((const float *)bo0->va, K, cand, kp, desc, 1024, pt);
          } else if (postv == 0) {
            xfeat_post(outv.data(), K, kp, desc, 1024);
          } else if (postv == 1) {
            post_v1(outv.data(), Pb, K, cand, kp, desc, 1024, pt);
          } else if (o16) {
            post_v2((const __fp16 *)bo0->va, K, cands, cand, kp, desc, 1024, pool, pt);
          } else {
            post_v2((const float *)bo0->va, K, cands, cand, kp, desc, 1024, pool, pt);
          }
          double e = now_us();
          if (rr) L("pipe exec fail %d", rr);
          tn.push_back(b - a);
          tx.push_back(c - b);
          to.push_back(d - c);
          tp.push_back(e - d);
          tt.push_back(e - a);
          if (ps == 0) {
            L("pipe pass0 frame %zu: norm+in %.0f exec %.0f out %.0f post %.0f total %.0f us, kp=%zu", k, b - a, c - b,
              d - c, e - d, e - a, kp.size());
            if (fk) {
              int32_t n = (int32_t)kp.size();
              std::fwrite(&n, 4, 1, fk);
              for (auto &q : kp) {
                int32_t xy[2] = {q.x, q.y};
                std::fwrite(xy, 4, 2, fk);
              }
              std::fwrite(desc.data(), 4, desc.size(), fk);
            }
          }
        }
        L("PIPE pass %d n=%zu total p50=%.0f p95=%.0f | norm+in p50 %.0f | exec p50 %.0f p95 %.0f | out memcpy p50 %.0f | "
          "post p50 %.0f p95 %.0f us",
          ps, tt.size(), pct(tt, .5), pct(tt, .95), pct(tn, .5), pct(tx, .5), pct(tx, .95), pct(to, .5), pct(tp, .5),
          pct(tp, .95));
        if (postv) {
          double nn = (double)nf;
          L("POSTSTAGES pass %d mean us: softmax %.0f shuffle %.0f nms+score %.0f topk %.0f desc %.0f%s", ps, pt.soft / nn,
            pt.shuf / nn, pt.nms / nn, pt.topk / nn, pt.desc / nn, postv == 2 ? " (v2: softmax = fused softmax+shuffle)" : "");
          if (g_skip) L("SKIP pass %d skipped cells per frame %.0f of %d", ps, g_skipped.load() / nn, Ph * Pw);
          g_skipped = 0;
        }
      }
      if (fk) std::fclose(fk);
    }
    L("EnnReleaseBuffers -> %d", eRelBufs(bufs, ntot));
    L("EnnCloseModel -> %d", eClose(mid));
    L("EnnDeinitialize -> %d", eDeinit());
    L("nncrun end");
    return;
  }
  std::vector<char> ind;
  if (!in.empty() && read_file(in, ind)) {
    size_t fs = bi0->size;
    size_t nf = ind.size() / fs;
    L("input file %s: %zu bytes = %zu frames of %zu (rest %zu)", in.c_str(), ind.size(), nf, fs, ind.size() % fs);
    FILE *fo = out.empty() ? nullptr : std::fopen(out.c_str(), "wb");
    for (size_t k = 0; k < nf && (int)k < maxfr; k++) {
      memcpy(bi0->va, ind.data() + k * fs, fs);
      double a = now_us();
      int rr = eExec(mid, 0);
      double b = now_us();
      if (fo) std::fwrite(bo0->va, 1, bo0->size, fo);
      const float *o = (const float *)bo0->va;
      double s = 0;
      for (uint32_t j = 0; j < bo0->size / 4; j++) s += o[j];
      L("frame %zu exec -> %d %.0f us out[0..2]=%g %g %g sum=%g", k, rr, b - a, o[0], o[1], o[2], s);
      if (rr != 0) break;
    }
    if (fo) std::fclose(fo);
    memcpy(bi0->va, ind.data(), fs);  // timing runs use frame 0
  } else {
    L("no input file (%s) -> timing on zero-filled buffer", in.c_str());
    memset(bi0->va, 0, bi0->size);
  }
  // copy cost (what an app pays for float in/out, measured once each)
  {
    std::vector<char> tmp(bi0->size);
    double a = now_us();
    memcpy(bi0->va, ind.empty() ? tmp.data() : ind.data(), bi0->size);
    double b = now_us();
    std::vector<char> tmp2(bo0->size);
    memcpy(tmp2.data(), bo0->va, bo0->size);
    double c = now_us();
    L("memcpy in %u B %.0f us, out %u B %.0f us", bi0->size, b - a, bo0->size, c - b);
  }
  for (int w = 0; w < warm; w++) {
    int rr = eExec(mid, 0);
    if (rr) {
      L("warmup exec fail %d", rr);
      break;
    }
  }
  for (int rp = 0; rp < reps; rp++) {
    std::vector<double> t;
    int fails = 0;
    double a0t = now_us();
    for (int it = 0; it < iters; it++) {
      double a = now_us();
      int rr = eExec(mid, 0);
      double b = now_us();
      if (rr) fails++;
      t.push_back(b - a);
    }
    double tot = now_us() - a0t;
    double mean = 0;
    for (double x : t) mean += x;
    mean /= std::max<size_t>(1, t.size());
    L("TIMING rep %d: n=%d fails=%d p50=%.0f p95=%.0f min=%.0f max=%.0f mean=%.0f us (wall %.1f ms)", rp, iters, fails,
      pct(t, 0.5), pct(t, 0.95), pct(t, 0.0), pct(t, 1.0), mean, tot / 1000);
  }
  L("EnnReleaseBuffers -> %d", eRelBufs(bufs, ntot));
  L("EnnCloseModel -> %d", eClose(mid));
  L("EnnDeinitialize -> %d", eDeinit());
  L("nncrun end");
}

#ifdef NNCRUN_JNI
extern "C" JNIEXPORT jstring JNICALL Java_com_scan3d_nncprobe_Probe_run(JNIEnv *env, jclass, jstring a, jstring d) {
  const char *as = env->GetStringUTFChars(a, nullptr);
  const char *ds = env->GetStringUTFChars(d, nullptr);
  g_out.clear();
  run_main(as, ds);
  if (g_f) {
    std::fclose(g_f);
    g_f = nullptr;
  }
  env->ReleaseStringUTFChars(a, as);
  env->ReleaseStringUTFChars(d, ds);
  return env->NewStringUTF(g_out.c_str());
}
#else
int main(int argc, char **argv) {
  std::string a;
  for (int i = 1; i < argc; i++) {
    a += argv[i];
    a += " ";
  }
  run_main(a, ".");
  return 0;
}
#endif
