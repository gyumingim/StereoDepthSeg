// ovswba 2026-10-26: minimal stand-in for ROS console macros used by the VINS-Mono estimator port (no ROS).
// Messages go to stderr when vins_log_level >= level (0 = quiet, 1 = warn/error, 2 = + info, 3 = + debug).
#pragma once
#include <cstdio>
#include <iostream>
#include <sstream>

extern int vins_log_level;

#define VINS_LOG_(lvl, tag, ...)                                   \
  do {                                                             \
    if (vins_log_level >= (lvl)) {                                 \
      std::fprintf(stderr, "[vins " tag "] ");                     \
      std::fprintf(stderr, __VA_ARGS__);                           \
      std::fprintf(stderr, "\n");                                  \
    }                                                              \
  } while (0)
#define VINS_LOGS_(lvl, tag, args)                                 \
  do {                                                             \
    if (vins_log_level >= (lvl)) {                                 \
      std::ostringstream vins_os_;                                 \
      vins_os_ << args;                                            \
      std::fprintf(stderr, "[vins " tag "] %s\n", vins_os_.str().c_str()); \
    }                                                              \
  } while (0)

#define ROS_DEBUG(...) VINS_LOG_(3, "debug", __VA_ARGS__)
#define ROS_INFO(...) VINS_LOG_(2, "info", __VA_ARGS__)
#define ROS_WARN(...) VINS_LOG_(1, "warn", __VA_ARGS__)
#define ROS_ERROR(...) VINS_LOG_(1, "error", __VA_ARGS__)
#define ROS_DEBUG_STREAM(args) VINS_LOGS_(3, "debug", args)
#define ROS_INFO_STREAM(args) VINS_LOGS_(2, "info", args)
#define ROS_WARN_STREAM(args) VINS_LOGS_(1, "warn", args)
#define ROS_ERROR_STREAM(args) VINS_LOGS_(1, "error", args)
