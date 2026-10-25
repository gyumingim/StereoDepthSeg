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
# OVOPT_FLAGS (2026-10-25, ovopt): extra compile flags for ov_bridge only (e.g. "-march=armv8.2-a+fp16+dotprod" or
# "-flto=thin"); empty (default) = the flags below unchanged. build_android.sh passes $OVOPT_FLAGS.
set(OVOPT_FLAGS "" CACHE STRING "extra compile+link flags for ov_bridge (empty = unchanged)")

set(OpenCV_DIR "${DEPS}/OpenCV-android-sdk/sdk/native/jni")
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d video calib3d highgui)
set(Ceres_DIR "${DEPS}/ceres-install/lib/cmake/Ceres")
find_package(Ceres REQUIRED)

set(S "${OV_SRC}")
# ovpatch/ copies are based on upstream open_vins 69488123: warn if OV_SRC holds other versions of the replaced files
foreach (pair "ov_core/src/track/TrackKLT.cpp=ec5b2cd169935ff1df1537cdfec6d62e"
              "ov_msckf/src/core/VioManager.cpp=0c38ccd6284bee46b559d1e366600fd1"
              "ov_msckf/src/state/StateHelper.cpp=69b3289a5061a008537d777558c3280c"
              "ov_msckf/src/update/UpdaterMSCKF.cpp=9c640902a78f3f7f82a763a6ed6043bc"
              "ov_msckf/src/update/UpdaterSLAM.cpp=411be47759278c9c7b4dcd531279bb06")
  string(REPLACE "=" ";" kv "${pair}")
  list(GET kv 0 f)
  list(GET kv 1 h)
  file(MD5 "${S}/${f}" got)
  if (NOT got STREQUAL h)
    message(WARNING "ovpatch: ${S}/${f} is not the upstream 69488123 version the ovpatch copy is based on (md5 ${got})")
  endif ()
endforeach ()
# ovpatch/ copies replace upstream files (each patch is a no-op unless its ov_bridge --ovopt flag is set):
#   TrackKLT.cpp (2026-10-24, arlat 0.6.15, KLT overlap, identical output),
#   VioManager.cpp / StateHelper.cpp / UpdaterMSCKF.cpp / UpdaterSLAM.cpp + ovopt.cpp (2026-10-25, ovopt).
add_executable(ov_bridge
  ../main.cpp ../ovpatch/ovopt.cpp
  ${S}/ov_core/src/cpi/CpiV1.cpp ${S}/ov_core/src/cpi/CpiV2.cpp ${S}/ov_core/src/sim/BsplineSE3.cpp
  ${S}/ov_core/src/track/TrackBase.cpp ${S}/ov_core/src/track/TrackAruco.cpp ${S}/ov_core/src/track/TrackDescriptor.cpp
  ../ovpatch/TrackKLT.cpp ${S}/ov_core/src/track/TrackSIM.cpp ${S}/ov_core/src/types/Landmark.cpp
  ${S}/ov_core/src/feat/Feature.cpp ${S}/ov_core/src/feat/FeatureDatabase.cpp ${S}/ov_core/src/feat/FeatureInitializer.cpp
  ${S}/ov_core/src/utils/print.cpp
  ${S}/ov_init/src/ceres/Factor_GenericPrior.cpp ${S}/ov_init/src/ceres/Factor_ImageReprojCalib.cpp
  ${S}/ov_init/src/ceres/Factor_ImuCPIv1.cpp ${S}/ov_init/src/ceres/State_JPLQuatLocal.cpp
  ${S}/ov_init/src/init/InertialInitializer.cpp ${S}/ov_init/src/dynamic/DynamicInitializer.cpp
  ${S}/ov_init/src/static/StaticInitializer.cpp
  ${S}/ov_msckf/src/state/State.cpp ../ovpatch/StateHelper.cpp ${S}/ov_msckf/src/state/Propagator.cpp
  ../ovpatch/VioManager.cpp ../ovpatch/VioManagerHelper.cpp
  ${S}/ov_msckf/src/update/UpdaterHelper.cpp ../ovpatch/UpdaterMSCKF.cpp
  ../ovpatch/UpdaterSLAM.cpp ${S}/ov_msckf/src/update/UpdaterZeroVelocity.cpp)
# ARUCO off on the phone (OpenCV SDK has no contrib aruco); bridge and library see the same macro since both compile here.
target_compile_definitions(ov_bridge PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0 OVOPT_FULL=1)
target_include_directories(ov_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../ovpatch
  ${CMAKE_CURRENT_SOURCE_DIR}/shim
  ${S}/ov_msckf/src ${S}/ov_core/src ${S}/ov_init/src ${S}/ov_core/src/track
  ${DEPS}/include/eigen3 ${DEPS}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ov_bridge PRIVATE ${OpenCV_LIBS} Ceres::ceres log)
target_compile_options(ov_bridge PRIVATE -O3 -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-deprecated-declarations)
if (OVOPT_FLAGS)
  separate_arguments(OVOPT_FLAGS_LIST UNIX_COMMAND "${OVOPT_FLAGS}")
  target_compile_options(ov_bridge PRIVATE ${OVOPT_FLAGS_LIST})
  target_link_options(ov_bridge PRIVATE ${OVOPT_FLAGS_LIST})
  message(STATUS "ov_bridge OVOPT_FLAGS: ${OVOPT_FLAGS}")
endif ()
# run on the phone with LD_LIBRARY_PATH=. (libopencv_java4.so, libc++_shared.so next to the binary)

# klt_bench (2026-10-24): TrackKLT / remap timing vs OpenCV thread count on the phone (adb shell, CPU only, no camera)
add_executable(klt_bench ../klt_bench.cpp
  ${S}/ov_core/src/track/TrackBase.cpp ../ovpatch/TrackKLT.cpp ${S}/ov_core/src/feat/Feature.cpp
  ${S}/ov_core/src/feat/FeatureDatabase.cpp ${S}/ov_core/src/utils/print.cpp)
target_compile_definitions(klt_bench PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0)
target_include_directories(klt_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${S}/ov_core/src ${S}/ov_core/src/track ${DEPS}/include/eigen3 ${DEPS}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(klt_bench PRIVATE ${OpenCV_LIBS} log)
target_compile_options(klt_bench PRIVATE -O3 -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-deprecated-declarations)
