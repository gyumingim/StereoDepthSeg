// xfnpu_jni.cpp (appnpu 2026-10-28): JNI glue for com.scan3d.arlat.NpuLoop -> libxfnpu.so (loaded only when intent
// npu_loop 1). One server per process (the OvRunner run).
#include <jni.h>
#include <android/log.h>

#include <memory>
#include <mutex>
#include <sstream>

#include "enn_backend.h"

namespace {
std::mutex g_m;
std::unique_ptr<xfnpu::EnnBackend> g_be;
std::unique_ptr<xfnpu::Server> g_srv;
std::string g_start_log;
std::string g_stuck;  // 0.6.31: a previous server is stuck inside ENN (leaked) -> refuse further starts in this process

std::string js(JNIEnv *env, jstring s) {
  if (!s) return "";
  const char *c = env->GetStringUTFChars(s, nullptr);
  std::string r(c ? c : "");
  if (c) env->ReleaseStringUTFChars(s, c);
  return r;
}
}  // namespace

extern "C" JNIEXPORT jstring JNICALL Java_com_scan3d_arlat_NpuLoop_nStart(JNIEnv *env, jclass, jstring sock, jstring xf,
                                                                           jstring lg, jstring wr, jstring log, jint topk,
                                                                           jstring lgIn, jint lgPad, jstring cpus, jint nice,
                                                                           jint perf, jdouble cacheMb, jstring opts) {
  std::lock_guard<std::mutex> l(g_m);
  if (g_srv) return env->NewStringUTF("already running");
  if (!g_stuck.empty()) return env->NewStringUTF(("previous NPU server stuck (" + g_stuck + "); restart the app").c_str());
  xfnpu::Cfg c;
  c.sock = js(env, sock);
  c.xf_model = js(env, xf);
  c.lg_model = js(env, lg);
  c.wr_path = js(env, wr);
  c.log_path = js(env, log);
  c.topk = topk;
  c.lg_in = js(env, lgIn);
  c.lg_pad = lgPad;
  c.nice = nice;
  c.cache_mb = cacheMb;
  {  // 0.6.35 npunight: npu_opts K=V,... (nn_fast, lg_post)
    std::stringstream ss(js(env, opts));
    std::string t;
    while (std::getline(ss, t, ',')) {
      const size_t e = t.find('=');
      if (e == std::string::npos) continue;
      const std::string k = t.substr(0, e);
      const int v = std::atoi(t.c_str() + e + 1);
      if (k == "nn_fast") c.nn_fast = v;
      else if (k == "lg_post") c.lg_post = v;
      else if (k == "prep_fast") c.prep_fast = v;
      else return env->NewStringUTF(("bad npu_opts key " + k).c_str());
    }
  }
  {
    std::stringstream ss(js(env, cpus));
    std::string t;
    while (std::getline(ss, t, ','))
      if (!t.empty()) c.cpus.push_back(std::atoi(t.c_str()));
  }
  g_be.reset(new xfnpu::EnnBackend(perf));
  g_srv.reset(new xfnpu::Server(g_be.get()));
  std::string err;
  const bool ok = g_srv->start(c, err);
  g_start_log = g_srv->start_log();
  if (!ok) {
    __android_log_print(ANDROID_LOG_WARN, "xfnpu", "start failed: %s", err.c_str());
    if (g_srv->stuck()) {  // worker still inside ENN: keep both objects alive (leak), block later starts
      g_stuck = err;
      g_srv.release();
      g_be.release();
      return env->NewStringUTF(err.c_str());
    }
    g_srv.reset();
    g_be.reset();
    return env->NewStringUTF(err.empty() ? "start failed" : err.c_str());
  }
  __android_log_print(ANDROID_LOG_INFO, "xfnpu", "server started on @%s", c.sock.c_str());
  return env->NewStringUTF("");
}

extern "C" JNIEXPORT jstring JNICALL Java_com_scan3d_arlat_NpuLoop_nStartLog(JNIEnv *env, jclass) {
  std::lock_guard<std::mutex> l(g_m);
  return env->NewStringUTF(g_start_log.c_str());
}

extern "C" JNIEXPORT jstring JNICALL Java_com_scan3d_arlat_NpuLoop_nStop(JNIEnv *env, jclass) {
  std::lock_guard<std::mutex> l(g_m);
  if (!g_srv) return env->NewStringUTF("");
  if (!g_srv->stop(3000)) {
    // worker stuck (e.g. inside EnnExecuteModel): it still uses both objects -> leak them on purpose, never free under it
    __android_log_print(ANDROID_LOG_WARN, "xfnpu", "stop: worker busy after 3 s, server object leaked");
    std::string f = g_srv->failure();
    g_stuck = "stuck at stop";
    g_srv.release();
    g_be.release();
    return env->NewStringUTF(("worker stuck at stop (3 s)" + (f.empty() ? std::string() : "; " + f)).c_str());
  }
  std::string f = g_srv->failure();
  g_srv.reset();
  g_be.reset();
  return env->NewStringUTF(f.c_str());
}

extern "C" JNIEXPORT void JNICALL Java_com_scan3d_arlat_NpuLoop_nTemps(JNIEnv *, jclass, jfloat skin, jfloat ap, jfloat bat,
                                                                        jint status) {
  std::lock_guard<std::mutex> l(g_m);
  if (g_srv) g_srv->set_temps(skin, ap, bat, status);
}

extern "C" JNIEXPORT jboolean JNICALL Java_com_scan3d_arlat_NpuLoop_nFailed(JNIEnv *, jclass) {  // 0.6.33 RunCue
  std::lock_guard<std::mutex> l(g_m);
  return (g_srv && !g_srv->failure().empty()) ? JNI_TRUE : JNI_FALSE;
}
