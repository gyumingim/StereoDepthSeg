#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

// ovswba 2026-10-26: readParam / readParameters(ros::NodeHandle&) removed (ROS + yaml config). The bridge sets the
// globals above directly (vins_bridge.cpp SwbaRunner::set_params, same values / meaning as the VINS config keys).
int vins_log_level = 1;                 // ovswba: compat/ros/console.h verbosity
std::atomic<long> vins_assert_fail{0};  // ovswba: compat/ros/assert.h failed-check counter
