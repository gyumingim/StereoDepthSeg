// ovloop lc_features.h (2026-10-26): keypoints + 256-bit box-BRIEF descriptors for loop closure, without any OpenCV
// parallel_for_ call (see ovloop.h: an OpenCV parallel region in the loop thread could serialise the VIO thread's KLT).
// Pyramid = own bilinear downscale (factor 1.4 per level), keypoints = cv::FAST (serial) + grid bucketing,
// descriptor = 256 intensity comparisons of 5x5 box sums (own integral image) at a fixed random Gaussian pattern
// (seed fixed, = the vocabulary's descriptor), steered by a given angle (ovloop: the image direction of gravity from the
// VIO attitude, so the descriptor is the same however the phone is rolled; 32 angle bins).
#pragma once
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace ovloop {

struct Pyramid {
  std::vector<cv::Mat> img;   // 8U
  std::vector<cv::Mat> integ;  // 32S (H+1) x (W+1)
  std::vector<double> scale;   // level -> level-0 pixel factor
  void build(const cv::Mat &gray, int levels);
};

struct KeyPt {
  float x = 0, y = 0;  // level-0 pixel
  int level = 0;
  float lx = 0, ly = 0;  // level pixel
  float response = 0;
  float angle = 0;  // rad, steering
};

class Brief {
public:
  Brief();
  static constexpr int kMargin = 22;  // px from the level border needed for a descriptor
  // FAST keypoints on all levels, at most n_feat (bucketed). Angles are left 0 (caller sets them).
  void detect(const Pyramid &P, int n_feat, int fast_th, std::vector<KeyPt> &out) const;
  // descriptors (rows = 32 bytes) of keypoints; keypoints too close to a border get valid[i] = 0 and a zero row
  void describe(const Pyramid &P, const std::vector<KeyPt> &kps, cv::Mat &desc, std::vector<uint8_t> &valid) const;

private:
  static constexpr int kBins = 32;
  std::vector<int8_t> pat_[kBins];  // 256 pairs x 4 (x1 y1 x2 y2), rotated per bin
};

}  // namespace ovloop
