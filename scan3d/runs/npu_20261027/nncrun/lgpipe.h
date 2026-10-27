// nncrun 2026-10-28: LighterGlue NPU (lgnpu n3t256: x [1,256,512] -> out [1,256,514]) phone pipeline.
// in = pairs file (per pair: kpts0 N x2, kpts1 N x2, desc0 N x64, desc1 N x64, f32, kpts normalised), wr = W [2,48] f32.
// pack (positional encoding on CPU: p = k @ W, cos/sin each duplicated to 96) -> EnnExecuteModel -> post on CPU:
// P_ij = A_ij * BT_ji (f32), sc = log(max(P,1e-30)) + logsigmoid(z0_i) + logsigmoid(z1_j) (f64), mutual argmax, exp(max) > 0.1
#pragma once
static inline double logsig(double z) { return z >= 0 ? -std::log1p(std::exp(-z)) : z - std::log1p(std::exp(z)); }
template <class FI, class FS, class FP, class FE>
static void lg_pipe(FI &I, FS &S, FP &P, const std::string &in, const std::string &out, EnnBuf *bi, EnnBuf *bo,
                    uint64_t mid, FE eExec) {
  const int N = I("lgn", 256);
  std::vector<char> pd, wd;
  if (!read_file(in, pd) || !read_file(P(S("wr", "lg_wr.bin")), wd) || wd.size() != 96 * 4) {
    L("ERROR lg_pipe inputs (%s, wr %zu B)", in.c_str(), wd.size());
    return;
  }
  const float *W = (const float *)wd.data();
  const size_t ps = (size_t)N * (2 + 2 + 64 + 64) * 4, np_ = pd.size() / ps;
  L("lg_pipe: N=%d pairs=%zu in buf %u out buf %u", N, np_, bi->size, bo->size);
  if (bi->size != (uint32_t)(N * 512 * 4) || bo->size != (uint32_t)(N * (2 * N + 2) * 4)) {
    L("ERROR buffer sizes");
    return;
  }
  FILE *fm = out.empty() ? nullptr : std::fopen(out.c_str(), "wb");
  int passes = I("passes", 10);
  std::vector<double> sc(N * N), rmax(N);
  std::vector<int> m0(N), m1(N);
  for (int pss = 0; pss < passes; pss++) {
    std::vector<double> tk, tx, tp, tt;
    for (size_t k = 0; k < np_; k++) {
      const float *k0 = (const float *)(pd.data() + k * ps), *k1 = k0 + N * 2, *d0 = k1 + N * 2, *d1 = d0 + N * 64;
      double a = now_us();
      float *x = (float *)bi->va;
      for (int i = 0; i < N; i++) {
        float *r = x + i * 512;
        memcpy(r, d0 + i * 64, 256);
        memcpy(r + 64, d1 + i * 64, 256);
        for (int j = 0; j < 48; j++) {
          float p0 = k0[i * 2] * W[j] + k0[i * 2 + 1] * W[48 + j], p1 = k1[i * 2] * W[j] + k1[i * 2 + 1] * W[48 + j];
          float c0 = std::cos(p0), s0 = std::sin(p0), c1 = std::cos(p1), s1 = std::sin(p1);
          r[128 + j] = r[176 + j] = c0;
          r[224 + j] = r[272 + j] = s0;
          r[320 + j] = r[368 + j] = c1;
          r[416 + j] = r[464 + j] = s1;
        }
      }
      double b = now_us();
      int rr = eExec(mid, 0);
      double c = now_us();
      const float *o = (const float *)bo->va;
      const int R = 2 * N + 2;
      for (int i = 0; i < N; i++) {
        double l0 = logsig(o[i * R + 2 * N]);
        for (int j = 0; j < N; j++) {
          float pp = o[i * R + j] * o[j * R + N + i];
          sc[i * N + j] = std::log(std::max((double)pp, 1e-30)) + l0 + logsig(o[j * R + 2 * N + 1]);
        }
      }
      for (int i = 0; i < N; i++) {
        int bj = 0;
        for (int j = 1; j < N; j++)
          if (sc[i * N + j] > sc[i * N + bj]) bj = j;
        m0[i] = bj;
        rmax[i] = sc[i * N + bj];
      }
      for (int j = 0; j < N; j++) {
        int bi_ = 0;
        for (int i = 1; i < N; i++)
          if (sc[i * N + j] > sc[bi_ * N + j]) bi_ = i;
        m1[j] = bi_;
      }
      std::vector<int32_t> mm;
      for (int i = 0; i < N; i++)
        if (m1[m0[i]] == i && std::exp(rmax[i]) > 0.1) {
          mm.push_back(i);
          mm.push_back(m0[i]);
        }
      double e = now_us();
      if (rr) L("lg exec fail %d", rr);
      tk.push_back(b - a);
      tx.push_back(c - b);
      tp.push_back(e - c);
      tt.push_back(e - a);
      if (pss == 0) {
        L("lg pass0 pair %zu: pack %.0f exec %.0f post %.0f total %.0f us matches %zu", k, b - a, c - b, e - c, e - a,
          mm.size() / 2);
        if (fm) {
          int32_t n = (int32_t)(mm.size() / 2);
          std::fwrite(&n, 4, 1, fm);
          std::fwrite(mm.data(), 4, mm.size(), fm);
        }
      }
    }
    L("LGPIPE pass %d n=%zu total p50=%.0f p95=%.0f | pack p50 %.0f | exec p50 %.0f p95 %.0f | post p50 %.0f p95 %.0f us", pss,
      tt.size(), pct(tt, .5), pct(tt, .95), pct(tk, .5), pct(tx, .5), pct(tx, .95), pct(tp, .5), pct(tp, .95));
  }
  if (fm) std::fclose(fm);
}
