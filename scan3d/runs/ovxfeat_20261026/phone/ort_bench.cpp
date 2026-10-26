// ovxfeat 2026-10-26: tiny ONNX Runtime benchmark for the phone (adb shell, no camera).
//   ort_bench MODEL.onnx BACKEND [iters 30] [warmup 5] [threads 2]
//   BACKEND: cpu | xnnpack | nnapi (NPU/GPU/CPU via Android NNAPI, Samsung driver) | nnapi_fp16 | nnapi_nocpu (NNAPI CPU device off)
// Inputs = random (float: uniform -1..1 / image 0..1), fixed shapes from the model. Prints one JSON line.
#include <onnxruntime_cxx_api.h>
#include <nnapi_provider_factory.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s MODEL BACKEND [iters] [warmup] [threads]\n", argv[0]);
    return 2;
  }
  const std::string model = argv[1], be = argv[2];
  const int iters = argc > 3 ? std::atoi(argv[3]) : 30, warm = argc > 4 ? std::atoi(argv[4]) : 5;
  const int thr = argc > 5 ? std::atoi(argv[5]) : 2;
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ovxfeat");
  Ort::SessionOptions so;
  so.SetIntraOpNumThreads(thr);
  so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  if (be == "xnnpack") {
    so.AppendExecutionProvider("XNNPACK", {{"intra_op_num_threads", std::to_string(thr)}});
  } else if (be.rfind("nnapi", 0) == 0) {
    uint32_t fl = 0;
    if (be == "nnapi_fp16") fl |= NNAPI_FLAG_USE_FP16;
    if (be == "nnapi_nocpu") fl |= NNAPI_FLAG_CPU_DISABLED;
    Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Nnapi(so, fl));
  } else if (be != "cpu") {
    std::fprintf(stderr, "unknown backend %s\n", be.c_str());
    return 2;
  }
  auto t0 = std::chrono::steady_clock::now();
  Ort::Session s(env, model.c_str(), so);
  const double load_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  Ort::AllocatorWithDefaultOptions al;
  std::vector<std::string> in_names, out_names;
  std::vector<std::vector<int64_t>> shapes;
  std::vector<std::vector<float>> bufs;
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> U(-1.f, 1.f);
  const std::string in_file = argc > 6 ? argv[6] : "", out_pref = argc > 7 ? argv[7] : "";  // accuracy check (raw f32)
  for (size_t i = 0; i < s.GetInputCount(); i++) {
    in_names.push_back(s.GetInputNameAllocated(i, al).get());
    auto sh = s.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    size_t n = 1;
    for (auto &d : sh) {
      if (d < 0) d = 1;
      n *= static_cast<size_t>(d);
    }
    std::vector<float> b(n);
    for (auto &x : b) x = U(rng);
    if (i == 0 && !in_file.empty()) {
      FILE *f = std::fopen(in_file.c_str(), "rb");
      if (!f || std::fread(b.data(), 4, n, f) != n) {
        std::fprintf(stderr, "cannot read %zu floats from %s\n", n, in_file.c_str());
        return 3;
      }
      std::fclose(f);
    }
    shapes.push_back(sh);
    bufs.push_back(std::move(b));
  }
  for (size_t i = 0; i < s.GetOutputCount(); i++) out_names.push_back(s.GetOutputNameAllocated(i, al).get());
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<Ort::Value> ins;
  for (size_t i = 0; i < bufs.size(); i++)
    ins.push_back(Ort::Value::CreateTensor<float>(mem, bufs[i].data(), bufs[i].size(), shapes[i].data(), shapes[i].size()));
  std::vector<const char *> inp, outp;
  for (auto &n : in_names) inp.push_back(n.c_str());
  for (auto &n : out_names) outp.push_back(n.c_str());
  std::vector<double> ms;
  for (int it = 0; it < warm + iters; it++) {
    auto a = std::chrono::steady_clock::now();
    auto out = s.Run(Ort::RunOptions{nullptr}, inp.data(), ins.data(), ins.size(), outp.data(), outp.size());
    const double d = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
    if (it >= warm) ms.push_back(d);
    if (it == warm + iters - 1 && !out_pref.empty())
      for (size_t k = 0; k < out.size(); k++) {
        auto ti = out[k].GetTensorTypeAndShapeInfo();
        if (ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) continue;
        FILE *f = std::fopen((out_pref + "_" + out_names[k] + ".f32").c_str(), "wb");
        if (f) {
          std::fwrite(out[k].GetTensorData<float>(), 4, ti.GetElementCount(), f);
          std::fclose(f);
        }
      }
  }
  std::sort(ms.begin(), ms.end());
  auto pc = [&](double p) { return ms[std::min(ms.size() - 1, static_cast<size_t>(p / 100.0 * (ms.size() - 1) + 0.5))]; };
  std::printf("{\"model\":\"%s\",\"backend\":\"%s\",\"threads\":%d,\"iters\":%d,\"load_ms\":%.1f,\"ms_min\":%.2f,\"ms_p50\":%.2f,\"ms_p95\":%.2f}\n",
              model.c_str(), be.c_str(), thr, iters, load_ms, ms.front(), pc(50), pc(95));
  return 0;
}
