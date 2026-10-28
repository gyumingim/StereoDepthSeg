// Rebuild the raw bridge stream on the phone: unpack SIDE.bin FRAMES.mp4 OUT.bin
// side.bin = stream records with camera pixel payload removed (20 B header kept); frames decoded in order with
// OpenCV VideoCapture (Android MediaNDK backend) -> gray (cvtColor BGR2GRAY) -> appended after each camera header.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include "lut.h"  // empirical inverse of the decoder YUV->BGR->gray limited-range stretch (cmpf.py, frame 100)
int main(int argc, char **argv) {
  if (argc < 4) return 1;
  FILE *s = fopen(argv[1], "rb");
  FILE *o = fopen(argv[3], "wb");
  if (!s || !o) return 2;
  cv::VideoCapture cap(argv[2], cv::CAP_ANDROID);
  if (!cap.isOpened()) { fprintf(stderr, "cannot open video\n"); return 3; }
  long ncam = 0, nrec = 0;
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
      if (!cap.read(bgr)) { fprintf(stderr, "video ended at frame %ld\n", ncam); return 4; }
      if (bgr.channels() == 3) cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
      else if (bgr.channels() == 4) cv::cvtColor(bgr, g, cv::COLOR_BGRA2GRAY);
      else g = bgr;
      cv::LUT(g, lut, g2);
      g = g2;
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
      if (ncam % 300 == 0) fprintf(stderr, "frames %ld\n", ncam);
    } else {
      fwrite(h, 1, 12, o);
      fwrite(p.data(), 1, n, o);
    }
    nrec++;
  }
  fclose(o);
  fprintf(stderr, "unpack: %ld records, %ld frames\n", nrec, ncam);
  return 0;
}
