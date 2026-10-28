// npunight 2026-10-29 unpack2 = ovheat unpack.cpp + optional IDX file:
//   unpack2 SIDE.bin VIDEO.mp4 OUT.bin [IDX.txt|-] [nolut]
// nolut: no LUT (the ovheat LUT was fitted for an x264 re-encode; the phone original cam5.mp4 may need none)
// SIDE.bin = bridge stream records with the camera pixel payload removed (20 B camera header kept). Each camera record gets
// the next wanted decoded frame (OpenCV VideoCapture, Android MediaNDK) -> gray -> LUT (ovheat lut.h, empirical inverse of
// the decoder's limited-range stretch). IDX = one source frame index (decode order of VIDEO) per camera record, rising;
// frames not listed are decoded and skipped -> the phone's original cam5.mp4 can be used instead of a re-encoded copy
// (adb over wifi is ~1 MB/s). Without IDX every decoded frame is used in order (= unpack).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include "lut.h"
int main(int argc, char **argv) {
  if (argc < 4) return 1;
  FILE *s = fopen(argv[1], "rb");
  FILE *o = fopen(argv[3], "wb");
  if (!s || !o) return 2;
  std::vector<long> idx;
  const bool nolut = argc > 5 && std::strcmp(argv[5], "nolut") == 0;
  if (argc > 4 && std::strcmp(argv[4], "-") != 0) {
    FILE *fi = fopen(argv[4], "r");
    if (!fi) return 6;
    long v;
    while (fscanf(fi, "%ld", &v) == 1) idx.push_back(v);
    fclose(fi);
  }
  cv::VideoCapture cap(argv[2], cv::CAP_ANDROID);
  if (!cap.isOpened()) { fprintf(stderr, "cannot open video\n"); return 3; }
  long ncam = 0, nrec = 0, ndec = 0;
  cv::Mat bgr, g, g2, lut(1, 256, CV_8U);
  for (int v = 0; v < 256; v++) lut.at<uint8_t>(v) = v <= 235 ? LUT[v] : (uint8_t)std::min(255.0, LUT[235] + (v - 235) * 0.875 + 0.5);
  for (;;) {
    uint8_t h[12];
    if (fread(h, 1, 12, s) != 12) break;
    uint32_t t, n;
    memcpy(&t, h + 4, 4);
    memcpy(&n, h + 8, 4);
    std::vector<uint8_t> p(n);
    if (n && fread(p.data(), 1, n, s) != n) break;
    if (t == 2) {
      uint32_t w, hh;
      memcpy(&w, p.data() + 8, 4);
      memcpy(&hh, p.data() + 12, 4);
      const long want = idx.empty() ? ndec : (ncam < (long)idx.size() ? idx[ncam] : -1);
      if (want < ndec) { fprintf(stderr, "bad idx at camera record %ld (want %ld, decoded %ld)\n", ncam, want, ndec); return 7; }
      for (;;) {
        if (!cap.read(bgr)) { fprintf(stderr, "video ended at camera record %ld (decoded %ld)\n", ncam, ndec); return 4; }
        ndec++;
        if (ndec - 1 == want) break;
      }
      if (bgr.channels() == 3) cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
      else if (bgr.channels() == 4) cv::cvtColor(bgr, g, cv::COLOR_BGRA2GRAY);
      else g = bgr;
      if (!nolut) {
        cv::LUT(g, lut, g2);
        g = g2;
      }
      if (g.cols != (int)w || g.rows != (int)hh || !g.isContinuous()) {
        fprintf(stderr, "size mismatch %dx%d vs %ux%u\n", g.cols, g.rows, w, hh);
        return 5;
      }
      uint32_t len = 20 + w * hh;
      memcpy(h + 8, &len, 4);
      fwrite(h, 1, 12, o);
      fwrite(p.data(), 1, 20, o);
      fwrite(g.data, 1, (size_t)w * hh, o);
      ncam++;
      if (ncam % 300 == 0) fprintf(stderr, "frames %ld (decoded %ld)\n", ncam, ndec);
    } else {
      fwrite(h, 1, 12, o);
      fwrite(p.data(), 1, n, o);
    }
    nrec++;
  }
  fclose(o);
  fprintf(stderr, "unpack2: %ld records, %ld frames, decoded %ld\n", nrec, ncam, ndec);
  return 0;
}
