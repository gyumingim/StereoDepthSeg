// ovswba 2026-10-26: minimal stand-in for ROS assert macros (VINS-Mono estimator port).
// VINS-Mono is built with CMAKE_BUILD_TYPE Release (-DNDEBUG) -> in ROS, ROS_ASSERT is compiled out. Same behaviour
// here (the checked expression is NOT acted on), but a failed check is counted (vins_assert_fail) and printed once
// per site at log level >= 1, so a violated invariant is visible in the --swba report instead of silent.
// ROS_BREAK (unconditional in ROS too) aborts; it is only reached on pthread_create failure in marginalization and in
// FeatureManager::removeOutlier (never called).
#pragma once
#include <atomic>
#include <cstdio>
#include <cstdlib>

extern std::atomic<long> vins_assert_fail;
extern int vins_log_level;

#define ROS_ASSERT(cond)                                                                         \
  do {                                                                                           \
    if (!(cond)) {                                                                               \
      static bool vins_once_ = false;                                                            \
      vins_assert_fail++;                                                                        \
      if (!vins_once_ && vins_log_level >= 1)                                                    \
        std::fprintf(stderr, "[vins] assert failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__);   \
      vins_once_ = true;                                                                         \
    }                                                                                            \
  } while (0)
#define ROS_ASSERT_MSG(cond, ...) ROS_ASSERT(cond)
#define ROS_BREAK()                                                                              \
  do {                                                                                           \
    std::fprintf(stderr, "[vins] BREAK (%s:%d)\n", __FILE__, __LINE__);                          \
    std::abort();                                                                                \
  } while (0)
