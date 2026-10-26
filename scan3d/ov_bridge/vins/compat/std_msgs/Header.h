// ovswba 2026-10-26: stand-in for std_msgs::Header (VINS-Mono estimator port): only header.stamp.toSec() is used.
#pragma once
#include <string>

namespace std_msgs {
struct Header {
  struct Stamp {
    double t = 0.0;
    double toSec() const { return t; }
  } stamp;
  std::string frame_id;
};
}  // namespace std_msgs
