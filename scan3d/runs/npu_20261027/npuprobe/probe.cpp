// npu 2026-10-27: NNAPI single-op probe, built twice: as JNI lib for the probe app (untrusted_app) and as an adb-shell
// executable (same code, same models) so the two contexts can be compared directly.
// For each probe model: getSupportedOperationsForDevices on the chosen device, compile for that device only
// (createForDevices = no CPU fallback), run 30 computes, log p50/p95. Device list + ENN library dlopen check logged.
// args: space separated tokens: probe names (conv3_f32 conv3_f16 conv3_s8 conv3_u8 conv3_s8pc conv1_* fc_* add_*,
//       big_conv3_*), "pref=fast|sustained|low", "dev=NAME" (default enn), "all"
#include <android/NeuralNetworks.h>
#include <android/log.h>
#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>
#ifdef NPUPROBE_JNI
#include <jni.h>
#endif

static std::string g_out;
static FILE *g_f = nullptr;
static void L(const char *fmt, ...) {
  char b[1024];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof b, fmt, ap);
  va_end(ap);
  __android_log_print(ANDROID_LOG_INFO, "npuprobe", "%s", b);
#ifndef NPUPROBE_JNI
  std::printf("%s\n", b);
#endif
  if (g_f) {
    std::fprintf(g_f, "%s\n", b);
    std::fflush(g_f);
  }
  g_out += b;
  g_out += "\n";
}

struct Probe {
  ANeuralNetworksModel *m = nullptr;
  std::vector<std::vector<uint8_t>> inbufs, outbufs, consts;
};

static int elemSize(int t) {
  switch (t) {
    case ANEURALNETWORKS_TENSOR_FLOAT32:
    case ANEURALNETWORKS_TENSOR_INT32: return 4;
    case ANEURALNETWORKS_TENSOR_FLOAT16: return 2;
    default: return 1;
  }
}

struct B {
  Probe &p;
  uint32_t n = 0;
  explicit B(Probe &pp) : p(pp) {}
  uint32_t tensor(int type, std::vector<uint32_t> dims, float scale = 0.f, int zp = 0) {
    ANeuralNetworksOperandType t{type, (uint32_t)dims.size(), dims.data(), scale, zp};
    ANeuralNetworksModel_addOperand(p.m, &t);
    return n++;
  }
  uint32_t scalarI(int32_t v) {
    ANeuralNetworksOperandType t{ANEURALNETWORKS_INT32, 0, nullptr, 0.f, 0};
    ANeuralNetworksModel_addOperand(p.m, &t);
    ANeuralNetworksModel_setOperandValue(p.m, n, &v, sizeof v);
    return n++;
  }
  void bytes(uint32_t idx, size_t nb, unsigned seed) {
    p.consts.emplace_back(nb);
    auto &c = p.consts.back();
    for (size_t i = 0; i < nb; i++) c[i] = (uint8_t)((i * 2654435761u + seed) >> 13);
    ANeuralNetworksModel_setOperandValue(p.m, idx, c.data(), nb);
  }
  void zeros(uint32_t idx, size_t nb) {
    p.consts.emplace_back(nb, (uint8_t)0);
    ANeuralNetworksModel_setOperandValue(p.m, idx, p.consts.back().data(), nb);
  }
  void f32(uint32_t idx, size_t count, float amp) {
    p.consts.emplace_back(count * 4);
    float *f = (float *)p.consts.back().data();
    for (size_t i = 0; i < count; i++) f[i] = amp * (float)((int)((i * 2654435761u) >> 20) % 200 - 100) / 100.f;
    ANeuralNetworksModel_setOperandValue(p.m, idx, f, count * 4);
  }
  void f16(uint32_t idx, size_t count) {
    p.consts.emplace_back(count * 2);
    uint16_t *h = (uint16_t *)p.consts.back().data();
    for (size_t i = 0; i < count; i++) h[i] = 0x2000 + (uint16_t)((i * 37) % 0x0800);
    ANeuralNetworksModel_setOperandValue(p.m, idx, h, count * 2);
  }
};

// q: f32 | f16 | s8 (QUANT8_ASYMM_SIGNED per-tensor) | u8 (QUANT8_ASYMM) | s8pc (signed act, per-channel symm weights)
static bool build(Probe &p, const std::string &kind, const std::string &q, int H, int W, int Cin, int Cout, int k) {
  ANeuralNetworksModel_create(&p.m);
  B b(p);
  const int act = q == "f32" ? ANEURALNETWORKS_TENSOR_FLOAT32
                  : q == "f16" ? ANEURALNETWORKS_TENSOR_FLOAT16
                  : q == "u8" ? ANEURALNETWORKS_TENSOR_QUANT8_ASYMM
                              : ANEURALNETWORKS_TENSOR_QUANT8_ASYMM_SIGNED;
  const int zp = q == "u8" ? 128 : 0;
  const bool quant = q != "f32" && q != "f16";
  const float si = quant ? 0.05f : 0.f, sw = quant ? 0.01f : 0.f, so = quant ? 0.1f : 0.f;
  auto biasOp = [&](uint32_t cout) {
    uint32_t bias = b.tensor(quant ? ANEURALNETWORKS_TENSOR_INT32 : act, {cout}, (quant && q != "s8pc") ? 0.05f * 0.01f : 0.f, 0);  // per-channel: bias scale 0 (NNAPI spec)
    if (quant) b.zeros(bias, cout * 4);
    else if (q == "f32") b.f32(bias, cout, 0.1f);
    else b.f16(bias, cout);
    return bias;
  };
  auto weights = [&](uint32_t idx, size_t cnt) {
    if (q == "f32") b.f32(idx, cnt, 0.05f);
    else if (q == "f16") b.f16(idx, cnt);
    else b.bytes(idx, cnt, 7);
  };
  std::vector<uint32_t> ins, outs;
  size_t inN, outN;
  if (kind == "conv") {
    uint32_t x = b.tensor(act, {1, (uint32_t)H, (uint32_t)W, (uint32_t)Cin}, si, zp);
    const bool pc = q == "s8pc";
    uint32_t w = b.tensor(pc ? ANEURALNETWORKS_TENSOR_QUANT8_SYMM_PER_CHANNEL : act,
                          {(uint32_t)Cout, (uint32_t)k, (uint32_t)k, (uint32_t)Cin}, pc ? 0.f : sw, pc ? 0 : zp);
    std::vector<float> scales(Cout, 0.01f);
    if (pc) {
      ANeuralNetworksSymmPerChannelQuantParams pq{0, (uint32_t)Cout, scales.data()};
      ANeuralNetworksModel_setOperandSymmPerChannelQuantParams(p.m, w, &pq);
    }
    weights(w, (size_t)Cout * k * k * Cin);
    uint32_t bias = biasOp(Cout);
    uint32_t pad = b.scalarI(ANEURALNETWORKS_PADDING_SAME), s1 = b.scalarI(1), s2 = b.scalarI(1),
             fa = b.scalarI(ANEURALNETWORKS_FUSED_RELU);
    uint32_t y = b.tensor(act, {1, (uint32_t)H, (uint32_t)W, (uint32_t)Cout}, so, zp);
    uint32_t oi[] = {x, w, bias, pad, s1, s2, fa};
    if (ANeuralNetworksModel_addOperation(p.m, ANEURALNETWORKS_CONV_2D, 7, oi, 1, &y) != ANEURALNETWORKS_NO_ERROR) return false;
    ins = {x};
    outs = {y};
    inN = (size_t)H * W * Cin;
    outN = (size_t)H * W * Cout;
  } else if (kind == "fc") {
    uint32_t x = b.tensor(act, {(uint32_t)H, (uint32_t)Cin}, si, zp);
    uint32_t w = b.tensor(act, {(uint32_t)Cout, (uint32_t)Cin}, sw, zp);
    weights(w, (size_t)Cout * Cin);
    uint32_t bias = biasOp(Cout);
    uint32_t fa = b.scalarI(ANEURALNETWORKS_FUSED_NONE);
    uint32_t y = b.tensor(act, {(uint32_t)H, (uint32_t)Cout}, so, zp);
    uint32_t oi[] = {x, w, bias, fa};
    if (ANeuralNetworksModel_addOperation(p.m, ANEURALNETWORKS_FULLY_CONNECTED, 4, oi, 1, &y) != ANEURALNETWORKS_NO_ERROR) return false;
    ins = {x};
    outs = {y};
    inN = (size_t)H * Cin;
    outN = (size_t)H * Cout;
  } else if (kind == "add") {
    uint32_t a = b.tensor(act, {1, (uint32_t)H, (uint32_t)W, (uint32_t)Cin}, si, zp);
    uint32_t c = b.tensor(act, {1, (uint32_t)H, (uint32_t)W, (uint32_t)Cin}, si, zp);
    uint32_t fa = b.scalarI(ANEURALNETWORKS_FUSED_NONE);
    uint32_t y = b.tensor(act, {1, (uint32_t)H, (uint32_t)W, (uint32_t)Cin}, so, zp);
    uint32_t oi[] = {a, c, fa};
    if (ANeuralNetworksModel_addOperation(p.m, ANEURALNETWORKS_ADD, 3, oi, 1, &y) != ANEURALNETWORKS_NO_ERROR) return false;
    ins = {a, c};
    outs = {y};
    inN = outN = (size_t)H * W * Cin;
  } else if (kind == "chain" || kind == "rgb") {
    // supervisor 21:10 test 1: several convs in sequence (single-op models may fail the NPU in/out layer-name matching).
    // chain: k x conv3x3 Cin->Cin stride 1 at HxW; rgb: [1,224,224,3] conv3x3 s2 -> 32, then (k-1) conv3x3 32->32
    const bool rgb = kind == "rgb";
    int h = rgb ? 224 : H, w_ = rgb ? 224 : W, c = rgb ? 3 : Cin;
    uint32_t x = b.tensor(act, {1, (uint32_t)h, (uint32_t)w_, (uint32_t)c}, quant ? 0.0078125f : 0.f, zp);
    ins = {x};
    inN = (size_t)h * w_ * c;
    uint32_t cur = x;
    for (int i = 0; i < k; i++) {
      const int co = rgb ? 32 : Cin, st = (rgb && i == 0) ? 2 : 1;
      uint32_t wt = b.tensor(act, {(uint32_t)co, 3, 3, (uint32_t)c}, sw, zp);
      weights(wt, (size_t)co * 9 * c);
      uint32_t bias = b.tensor(quant ? ANEURALNETWORKS_TENSOR_INT32 : act, {(uint32_t)co},
                               quant ? (i == 0 && rgb ? 0.0078125f : (i == 0 ? si : so)) * sw : 0.f, 0);
      if (quant) b.zeros(bias, co * 4);
      else if (q == "f32") b.f32(bias, co, 0.1f);
      else b.f16(bias, co);
      uint32_t pad = b.scalarI(ANEURALNETWORKS_PADDING_SAME), s1 = b.scalarI(st), s2 = b.scalarI(st),
               fa = b.scalarI(ANEURALNETWORKS_FUSED_RELU);
      h = (h + st - 1) / st; w_ = (w_ + st - 1) / st; c = co;
      uint32_t y = b.tensor(act, {1, (uint32_t)h, (uint32_t)w_, (uint32_t)c}, so, zp);
      uint32_t oi[] = {cur, wt, bias, pad, s1, s2, fa};
      if (ANeuralNetworksModel_addOperation(p.m, ANEURALNETWORKS_CONV_2D, 7, oi, 1, &y) != ANEURALNETWORKS_NO_ERROR) return false;
      cur = y;
    }
    outs = {cur};
    outN = (size_t)h * w_ * c;
  } else {
    return false;
  }
  ANeuralNetworksModel_identifyInputsAndOutputs(p.m, ins.size(), ins.data(), outs.size(), outs.data());
  if (ANeuralNetworksModel_finish(p.m) != ANEURALNETWORKS_NO_ERROR) return false;
  const size_t es = elemSize(act);
  for (size_t i = 0; i < ins.size(); i++) p.inbufs.emplace_back(inN * es, (uint8_t)1);
  p.outbufs.emplace_back(outN * es);
  return true;
}

static void runOne(const std::string &name, ANeuralNetworksDevice *dev, int pref) {
  std::string n = name;
  int H = 44, W = 80, Cin = 64, Cout = 64, k = 3;
  if (n.rfind("big_", 0) == 0) {
    n = n.substr(4);
    H = 176; W = 320; Cin = 32; Cout = 32;
  }
  const size_t us = n.find('_');
  if (us == std::string::npos) { L("%s BAD_NAME", name.c_str()); return; }
  std::string kind = n.substr(0, us), q = n.substr(us + 1);
  if (kind == "conv1") { kind = "conv"; k = 1; }
  else if (kind == "conv3") kind = "conv";
  else if (kind.rfind("chain", 0) == 0 && kind.size() > 5) { k = std::atoi(kind.c_str() + 5); kind = "chain"; }  // chainK
  else if (kind.rfind("rgb", 0) == 0 && kind.size() > 3) { k = std::atoi(kind.c_str() + 3); kind = "rgb"; }        // rgbK
  if (kind == "fc") { H = 512; Cin = 96; Cout = 96; }
  Probe p;
  if (!build(p, kind, q, H, W, Cin, Cout, k)) { L("RESULT %s BUILD_FAIL", name.c_str()); if (p.m) ANeuralNetworksModel_free(p.m); return; }
  bool sup[4] = {false, false, false, false};
  int r = ANeuralNetworksModel_getSupportedOperationsForDevices(p.m, &dev, 1, sup);
  ANeuralNetworksCompilation *c = nullptr;
  int rc = ANeuralNetworksCompilation_createForDevices(p.m, &dev, 1, &c);
  if (rc == ANEURALNETWORKS_NO_ERROR) ANeuralNetworksCompilation_setPreference(c, pref);
  auto t0 = std::chrono::steady_clock::now();
  int rf = rc == ANEURALNETWORKS_NO_ERROR ? ANeuralNetworksCompilation_finish(c) : rc;
  double comp_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::vector<double> ms;
  int re = -1;
  if (rf == ANEURALNETWORKS_NO_ERROR) {
    for (int it = 0; it < 35; it++) {
      ANeuralNetworksExecution *e = nullptr;
      ANeuralNetworksExecution_create(c, &e);
      for (size_t i = 0; i < p.inbufs.size(); i++) ANeuralNetworksExecution_setInput(e, i, nullptr, p.inbufs[i].data(), p.inbufs[i].size());
      ANeuralNetworksExecution_setOutput(e, 0, nullptr, p.outbufs[0].data(), p.outbufs[0].size());
      auto a = std::chrono::steady_clock::now();
      re = ANeuralNetworksExecution_compute(e);
      double d = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
      ANeuralNetworksExecution_free(e);
      if (re != ANEURALNETWORKS_NO_ERROR) break;
      if (it >= 5) ms.push_back(d);
    }
  }
  std::sort(ms.begin(), ms.end());
  double p50 = ms.empty() ? -1 : ms[ms.size() / 2];
  double p95 = ms.empty() ? -1 : ms[std::min(ms.size() - 1, (size_t)(ms.size() * 0.95))];
  L("RESULT %s supported=%d (rc %d) compile_rc=%d finish_rc=%d compile_ms=%.1f exec_rc=%d p50_ms=%.3f p95_ms=%.3f",
    name.c_str(), (int)sup[0], r, rc, rf, comp_ms, re, p50, p95);
  if (c) ANeuralNetworksCompilation_free(c);
  ANeuralNetworksModel_free(p.m);
}

static std::string mainImpl(const std::string &args, const std::string &dir) {
  g_out.clear();
  std::string fn = dir + "/npuprobe_out.txt";
  g_f = std::fopen(fn.c_str(), "a");
  L("npuprobe start args='%s' out=%s", args.c_str(), fn.c_str());
  uint32_t nd = 0;
  ANeuralNetworks_getDeviceCount(&nd);
  std::string want = "enn";
  int pref = ANEURALNETWORKS_PREFER_SUSTAINED_SPEED;
  std::vector<std::string> probes;
  std::istringstream ss(args);
  std::string t;
  while (ss >> t) {
    if (t.rfind("dev=", 0) == 0) want = t.substr(4);
    else if (t == "pref=fast") pref = ANEURALNETWORKS_PREFER_FAST_SINGLE_ANSWER;
    else if (t == "pref=low") pref = ANEURALNETWORKS_PREFER_LOW_POWER;
    else if (t == "pref=sustained") pref = ANEURALNETWORKS_PREFER_SUSTAINED_SPEED;
    else if (t == "all") {
      for (std::string k : {"conv3", "conv1", "fc", "add", "big_conv3"})
        for (std::string q : {"f32", "f16", "s8", "u8", "s8pc"})
          if (!((k == "fc" || k == "add") && q == "s8pc")) probes.push_back(k + "_" + q);
    } else probes.push_back(t);
  }
  ANeuralNetworksDevice *dev = nullptr;
  for (uint32_t i = 0; i < nd; i++) {
    ANeuralNetworksDevice *d = nullptr;
    ANeuralNetworks_getDevice(i, &d);
    const char *nm = nullptr, *ver = nullptr;
    int32_t type = 0;
    int64_t fl = 0;
    ANeuralNetworksDevice_getName(d, &nm);
    ANeuralNetworksDevice_getVersion(d, &ver);
    ANeuralNetworksDevice_getType(d, &type);
    ANeuralNetworksDevice_getFeatureLevel(d, &fl);
    L("DEVICE %u name=%s version=%s type=%d (1 other 2 cpu 3 gpu 4 accelerator) feature_level=%lld", i, nm ? nm : "?",
      ver ? ver : "?", type, (long long)fl);
    if (nm && want == nm) dev = d;
  }
  for (const char *lib : {"libenn_public_api_cpp.so", "libenn_user.samsung_slsi.so", "libeden_rt_stub.edensdk.samsung.so"}) {
    void *h = dlopen(lib, RTLD_NOW | RTLD_LOCAL);
    const char *er = h ? "" : dlerror();
    L("DLOPEN %s -> %s %s", lib, h ? "OK" : "FAIL", er ? er : "");
    if (h) {
      for (const char *s : {"EnnInitialize", "EnnDeinitialize", "EnnOpenModel", "EnnExecuteModel", "EnnGetVersion"})
        L("  dlsym %s %s", s, dlsym(h, s) ? "found" : "-");
      dlclose(h);  // presence check only: no ENN calls (no .nnc model)
    }
  }
  if (!dev) {
    L("device %s not found", want.c_str());
  } else {
    L("pref=%d probes=%zu", pref, probes.size());
    for (auto &pn : probes) runOne(pn, dev, pref);
  }
  L("npuprobe end");
  if (g_f) std::fclose(g_f);
  g_f = nullptr;
  return g_out;
}

#ifdef NPUPROBE_JNI
extern "C" JNIEXPORT jstring JNICALL Java_com_scan3d_npuprobe_Probe_run(JNIEnv *env, jclass, jstring a, jstring d) {
  const char *s = env->GetStringUTFChars(a, nullptr);
  const char *ds = env->GetStringUTFChars(d, nullptr);
  std::string r = mainImpl(s, ds);
  env->ReleaseStringUTFChars(a, s);
  env->ReleaseStringUTFChars(d, ds);
  if (r.size() > 60000) r.resize(60000);
  return env->NewStringUTF(r.c_str());
}
#else
int main(int argc, char **argv) {
  std::string a;
  for (int i = 1; i < argc; i++) { a += argv[i]; a += " "; }
  mainImpl(a, ".");
  return 0;
}
#endif
