// ovxfeat 2026-10-26: per-keyframe CPU work on the phone, measured in one process (no camera):
//   XFeat backbone (ONNX Runtime, XNNPACK or CPU EP; the GPU number comes from TFLite benchmark_model separately)
//   + post-processing in C++ (65-ch softmax -> 8x8 heatmap, 5x5 NMS th 0.05, score = K * H, top-k, bilinear descriptor
//     sampling at keypoints + 200 track points, L2 norm)
//   + n_match x mutual-NN matcher (ONNX nnmatch_N, fixed N, zero padded) + mutual / cos >= 0.82 check.
// usage: kf_bench XFEAT.onnx NN.onnx IMG.f32 W H TOPK N_MATCH ITERS THREADS [skip_xfeat 0|1]
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point a) { return std::chrono::duration<double, std::milli>(Clock::now() - a).count(); }

int main(int argc, char **argv) {
  if (argc < 10) {
    std::fprintf(stderr, "usage: %s XFEAT.onnx NN.onnx IMG.f32 W H TOPK N_MATCH ITERS THREADS [skip_xfeat]\n", argv[0]);
    return 2;
  }
  const std::string xm = argv[1], nm = argv[2], imf = argv[3];
  const int W = std::atoi(argv[4]), H = std::atoi(argv[5]), topk = std::atoi(argv[6]), nmatch = std::atoi(argv[7]);
  const int iters = std::atoi(argv[8]), thr = std::atoi(argv[9]);
  const bool skip_x = argc > 10 && std::atoi(argv[10]) == 1;
  const int h = H / 8, w = W / 8;
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "kf");
  Ort::SessionOptions sx, sn;
  sx.SetIntraOpNumThreads(thr);
  sx.AppendExecutionProvider("XNNPACK", {{"intra_op_num_threads", std::to_string(thr)}});
  sn.SetIntraOpNumThreads(thr);
  Ort::Session X(env, xm.c_str(), sx), N(env, nm.c_str(), sn);
  const int NN = static_cast<int>(N.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()[1]);
  std::vector<float> img(static_cast<size_t>(W) * H);
  FILE *f = std::fopen(imf.c_str(), "rb");
  if (!f || std::fread(img.data(), 4, img.size(), f) != img.size()) return 3;
  std::fclose(f);
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<int64_t> ish{1, 1, H, W};
  const char *xin[] = {"image"};
  const char *xout[] = {"feats", "keypoints", "heatmap"};
  const char *nin[] = {"desc0", "desc1"};
  const char *nout[] = {"m0", "m1", "c0"};
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> ux(8.f, W - 8.f), uy(8.f, H - 8.f);
  std::vector<double> t_x, t_post, t_match, t_tot;
  int last_nk = 0, last_nm = 0;
  for (int it = 0; it < iters + 2; it++) {
    auto T0 = Clock::now();
    // 1. backbone
    auto in = Ort::Value::CreateTensor<float>(mem, img.data(), img.size(), ish.data(), 4);
    auto t0 = Clock::now();
    auto out = X.Run(Ort::RunOptions{nullptr}, xin, &in, 1, xout, 3);
    const double dx = ms(t0);
    const float *M = out[0].GetTensorData<float>(), *K = out[1].GetTensorData<float>(), *Hm = out[2].GetTensorData<float>();
    // 2. post-processing
    t0 = Clock::now();
    std::vector<float> heat(static_cast<size_t>(W) * H);
    for (int y = 0; y < h; y++)
      for (int x = 0; x < w; x++) {
        float mx = -1e30f;
        for (int c = 0; c < 65; c++) mx = std::max(mx, K[(c * h + y) * w + x]);
        float s = 0, e[65];
        for (int c = 0; c < 65; c++) s += (e[c] = std::exp(K[(c * h + y) * w + x] - mx));
        for (int c = 0; c < 64; c++) heat[(y * 8 + c / 8) * W + x * 8 + c % 8] = e[c] / s;
      }
    std::vector<std::pair<float, int>> cand;
    for (int y = 0; y < H; y++)
      for (int x = 0; x < W; x++) {
        const float v = heat[y * W + x];
        if (v <= 0.05f) continue;
        bool mxp = true;
        for (int dy = -2; dy <= 2 && mxp; dy++)
          for (int dx2 = -2; dx2 <= 2; dx2++) {
            const int yy = y + dy, xx = x + dx2;
            if (yy < 0 || yy >= H || xx < 0 || xx >= W) continue;
            if (heat[yy * W + xx] > v) { mxp = false; break; }
          }
        if (!mxp) continue;
        // reliability: bilinear heatmap at 1/8 (align_corners false)
        const float fx = std::min(std::max((x + 0.5f) / 8.f - 0.5f, 0.f), w - 1.f), fy = std::min(std::max((y + 0.5f) / 8.f - 0.5f, 0.f), h - 1.f);
        const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy), x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
        const float ax = fx - x0, ay = fy - y0;
        const float r = (1 - ay) * ((1 - ax) * Hm[y0 * w + x0] + ax * Hm[y0 * w + x1]) + ay * ((1 - ax) * Hm[y1 * w + x0] + ax * Hm[y1 * w + x1]);
        cand.emplace_back(v * r, y * W + x);
      }
    const int nk = std::min<int>(topk, static_cast<int>(cand.size()));
    std::partial_sort(cand.begin(), cand.begin() + nk, cand.end(), [](auto &a, auto &b) { return a.first > b.first; });
    std::vector<float> pts;
    for (int i = 0; i < nk; i++) {
      pts.push_back(static_cast<float>(cand[i].second % W));
      pts.push_back(static_cast<float>(cand[i].second / W));
    }
    for (int i = 0; i < 200; i++) {  // OpenVINS track points
      pts.push_back(ux(rng));
      pts.push_back(uy(rng));
    }
    const int np = static_cast<int>(pts.size() / 2);
    std::vector<float> D(static_cast<size_t>(NN) * 64, 0.f);
    for (int i = 0; i < std::min(np, NN); i++) {
      const float fx = std::min(std::max((pts[2 * i] + 0.5f) / 8.f - 0.5f, 0.f), w - 1.f);
      const float fy = std::min(std::max((pts[2 * i + 1] + 0.5f) / 8.f - 0.5f, 0.f), h - 1.f);
      const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy), x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
      const float ax = fx - x0, ay = fy - y0;
      float n2 = 0;
      for (int c = 0; c < 64; c++) {
        const float *m = M + static_cast<size_t>(c) * h * w;
        const float v = (1 - ay) * ((1 - ax) * m[y0 * w + x0] + ax * m[y0 * w + x1]) + ay * ((1 - ax) * m[y1 * w + x0] + ax * m[y1 * w + x1]);
        D[static_cast<size_t>(i) * 64 + c] = v;
        n2 += v * v;
      }
      const float inv = 1.f / std::sqrt(std::max(n2, 1e-12f));
      for (int c = 0; c < 64; c++) D[static_cast<size_t>(i) * 64 + c] *= inv;
    }
    const double dpost = ms(t0);
    // 3. matcher calls (same descriptor sets: timing only)
    t0 = Clock::now();
    std::vector<int64_t> dsh{1, NN, 64};
    for (int k = 0; k < nmatch; k++) {
      Ort::Value ins[2] = {Ort::Value::CreateTensor<float>(mem, D.data(), D.size(), dsh.data(), 3),
                           Ort::Value::CreateTensor<float>(mem, D.data(), D.size(), dsh.data(), 3)};
      auto o = N.Run(Ort::RunOptions{nullptr}, nin, ins, 2, nout, 3);
      const int64_t *m0 = o[0].GetTensorData<int64_t>(), *m1 = o[1].GetTensorData<int64_t>();
      const float *c0 = o[2].GetTensorData<float>();
      int good = 0;
      for (int i = 0; i < std::min(np, NN); i++)
        if (m1[m0[i]] == i && c0[i] >= 0.82f) good++;
      last_nm = good;
    }
    const double dm = ms(t0);
    last_nk = nk;
    if (it >= 2) {
      t_x.push_back(skip_x ? 0.0 : dx);
      t_post.push_back(dpost);
      t_match.push_back(dm);
      t_tot.push_back(ms(T0) - (skip_x ? dx : 0.0));
    }
  }
  auto med = [](std::vector<double> v) { std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
  auto p95 = [](std::vector<double> v) { std::sort(v.begin(), v.end()); return v[std::min(v.size() - 1, static_cast<size_t>(0.95 * (v.size() - 1) + 0.5))]; };
  std::printf("{\"kf_bench\":1,\"W\":%d,\"H\":%d,\"topk\":%d,\"nk\":%d,\"N\":%d,\"n_match\":%d,\"threads\":%d,\"iters\":%d,\"xfeat_cpu_ms_p50\":%.2f,"
              "\"post_ms_p50\":%.2f,\"match_ms_p50\":%.2f,\"total_ms_p50\":%.2f,\"total_ms_p95\":%.2f,\"self_matches\":%d}\n",
              W, H, topk, last_nk, NN, nmatch, thr, iters, med(t_x), med(t_post), med(t_match), med(t_tot), p95(t_tot), last_nm);
  return 0;
}
