# OpenVINS + ov_bridge for Android arm64 (S24) — phone-side timing benchmark (build_android.sh).
# One executable: OpenVINS sources compiled in, Ceres static, OpenCV shared (libopencv_java4.so), libc++_shared.
cmake_minimum_required(VERSION 3.10)
project(ov_bridge_android CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()
set(OV_SRC "$ENV{HOME}/scanner/src/open_vins" CACHE PATH "open_vins source root")
set(DEPS "$ENV{HOME}/scanner/android" CACHE PATH "android deps root (OpenCV SDK, ceres install, headers)")

set(OpenCV_DIR "${DEPS}/OpenCV-android-sdk/sdk/native/jni")
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d video calib3d highgui)
set(Ceres_DIR "${DEPS}/ceres-install/lib/cmake/Ceres")
find_package(Ceres REQUIRED)

set(S "${OV_SRC}")
add_executable(ov_bridge
  ../main.cpp
  ${S}/ov_core/src/cpi/CpiV1.cpp ${S}/ov_core/src/cpi/CpiV2.cpp ${S}/ov_core/src/sim/BsplineSE3.cpp
  ${S}/ov_core/src/track/TrackBase.cpp ${S}/ov_core/src/track/TrackAruco.cpp ${S}/ov_core/src/track/TrackDescriptor.cpp
  ${S}/ov_core/src/track/TrackKLT.cpp ${S}/ov_core/src/track/TrackSIM.cpp ${S}/ov_core/src/types/Landmark.cpp
  ${S}/ov_core/src/feat/Feature.cpp ${S}/ov_core/src/feat/FeatureDatabase.cpp ${S}/ov_core/src/feat/FeatureInitializer.cpp
  ${S}/ov_core/src/utils/print.cpp
  ${S}/ov_init/src/ceres/Factor_GenericPrior.cpp ${S}/ov_init/src/ceres/Factor_ImageReprojCalib.cpp
  ${S}/ov_init/src/ceres/Factor_ImuCPIv1.cpp ${S}/ov_init/src/ceres/State_JPLQuatLocal.cpp
  ${S}/ov_init/src/init/InertialInitializer.cpp ${S}/ov_init/src/dynamic/DynamicInitializer.cpp
  ${S}/ov_init/src/static/StaticInitializer.cpp
  ${S}/ov_msckf/src/state/State.cpp ${S}/ov_msckf/src/state/StateHelper.cpp ${S}/ov_msckf/src/state/Propagator.cpp
  ${S}/ov_msckf/src/core/VioManager.cpp ${S}/ov_msckf/src/core/VioManagerHelper.cpp
  ${S}/ov_msckf/src/update/UpdaterHelper.cpp ${S}/ov_msckf/src/update/UpdaterMSCKF.cpp
  ${S}/ov_msckf/src/update/UpdaterSLAM.cpp ${S}/ov_msckf/src/update/UpdaterZeroVelocity.cpp)
# ARUCO off on the phone (OpenCV SDK has no contrib aruco); bridge and library see the same macro since both compile here.
target_compile_definitions(ov_bridge PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0)
target_include_directories(ov_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/shim
  ${S}/ov_msckf/src ${S}/ov_core/src ${S}/ov_init/src
  ${DEPS}/include/eigen3 ${DEPS}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ov_bridge PRIVATE ${OpenCV_LIBS} Ceres::ceres log)
target_compile_options(ov_bridge PRIVATE -O3 -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-deprecated-declarations)
# run on the phone with LD_LIBRARY_PATH=. (libopencv_java4.so, libc++_shared.so next to the binary)
