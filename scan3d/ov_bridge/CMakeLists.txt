cmake_minimum_required(VERSION 3.10)
project(ov_bridge CXX)

# OpenVINS (ROS 없이 빌드된 ov_msckf_lib) 위치. 필요하면 -DOV_SRC=... -DOV_BUILD=... 로 바꾼다.
set(OV_SRC "$ENV{HOME}/scanner/src/open_vins" CACHE PATH "open_vins source root")
set(OV_BUILD "$ENV{HOME}/scanner/src/ovbuild" CACHE PATH "ov_msckf build dir (libov_msckf_lib.so)")
set(OV_LOCAL "$ENV{HOME}/scanner/local" CACHE PATH "Ceres install prefix")
list(APPEND CMAKE_PREFIX_PATH "${OV_LOCAL}")
# OV_FULL_SOURCE (2026-10-25, ovopt): compile the OpenVINS sources + all ovpatch/ copies into ov_bridge (like the phone
# build android/CMakeLists.txt) instead of linking libov_msckf_lib.so. Needed for ov_bridge --ovopt. Default OFF = the
# build below is unchanged (libov_msckf_lib.so + ovpatch/TrackKLT.cpp).
option(OV_FULL_SOURCE "compile OpenVINS + ovpatch into ov_bridge (needed for --ovopt)" OFF)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

# ov_msckf_lib 가 C++14 로 빌드됨 -> 같게 맞춘다
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

if (OV_FULL_SOURCE)
  # Same source list as android/CMakeLists.txt; patched copies from ovpatch/ replace the upstream files. The OpenVINS
  # part gets the flags of the libov_msckf_lib.so build (ovbuild flags.make), the bridge keeps its own -O2.
  set(S "${OV_SRC}")
  # ovpatch/ copies are based on upstream open_vins 69488123: warn if OV_SRC holds other versions of the replaced files
  foreach (pair "ov_core/src/track/TrackKLT.cpp=ec5b2cd169935ff1df1537cdfec6d62e"
                "ov_msckf/src/core/VioManager.cpp=0c38ccd6284bee46b559d1e366600fd1"
                "ov_msckf/src/core/VioManagerHelper.cpp=690f3f2fae333f4737026b005f163058"
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
  add_library(ov_full STATIC
    ovpatch/ovopt.cpp
    ${S}/ov_core/src/cpi/CpiV1.cpp ${S}/ov_core/src/cpi/CpiV2.cpp ${S}/ov_core/src/sim/BsplineSE3.cpp
    ${S}/ov_core/src/track/TrackBase.cpp ${S}/ov_core/src/track/TrackAruco.cpp ${S}/ov_core/src/track/TrackDescriptor.cpp
    ovpatch/TrackKLT.cpp ${S}/ov_core/src/track/TrackSIM.cpp ${S}/ov_core/src/types/Landmark.cpp
    ${S}/ov_core/src/feat/Feature.cpp ${S}/ov_core/src/feat/FeatureDatabase.cpp ${S}/ov_core/src/feat/FeatureInitializer.cpp
    ${S}/ov_core/src/utils/print.cpp
    ${S}/ov_init/src/ceres/Factor_GenericPrior.cpp ${S}/ov_init/src/ceres/Factor_ImageReprojCalib.cpp
    ${S}/ov_init/src/ceres/Factor_ImuCPIv1.cpp ${S}/ov_init/src/ceres/State_JPLQuatLocal.cpp
    ${S}/ov_init/src/init/InertialInitializer.cpp ${S}/ov_init/src/dynamic/DynamicInitializer.cpp
    ${S}/ov_init/src/static/StaticInitializer.cpp
    ${S}/ov_msckf/src/state/State.cpp ovpatch/StateHelper.cpp ${S}/ov_msckf/src/state/Propagator.cpp
    ovpatch/VioManager.cpp ovpatch/VioManagerHelper.cpp
    ${S}/ov_msckf/src/update/UpdaterHelper.cpp ovpatch/UpdaterMSCKF.cpp
    ovpatch/UpdaterSLAM.cpp ${S}/ov_msckf/src/update/UpdaterZeroVelocity.cpp)
  set(OV_FULL_DEFS ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0 OVOPT_FULL=1)
  set(OV_FULL_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/ovpatch
    "${OV_SRC}/ov_msckf/src" "${OV_SRC}/ov_core/src" "${OV_SRC}/ov_core/src/track" "${OV_SRC}/ov_init/src"
    ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
  target_compile_definitions(ov_full PUBLIC ${OV_FULL_DEFS})
  target_include_directories(ov_full PUBLIC ${OV_FULL_INC})
  set_target_properties(ov_full PROPERTIES CXX_STANDARD 17)
  target_compile_options(ov_full PRIVATE -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops
    -fno-omit-frame-pointer -Wall -Wno-deprecated-declarations ${OVOPT_EXTRA_FLAGS})
  target_link_libraries(ov_full PUBLIC ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} Threads::Threads)

  add_executable(ov_bridge main.cpp)
  set_target_properties(ov_bridge PROPERTIES CXX_STANDARD 17)
  target_link_libraries(ov_bridge PRIVATE ov_full)
  target_compile_options(ov_bridge PRIVATE -O2 -Wall)
  set_target_properties(ov_bridge PROPERTIES BUILD_RPATH "${OV_LOCAL}/lib" INSTALL_RPATH "${OV_LOCAL}/lib")
  message(STATUS "ov_bridge: OV_FULL_SOURCE (OpenVINS ${OV_SRC} + ovpatch) | OpenCV ${OpenCV_VERSION} | Ceres ${Ceres_VERSION}")
  return()
endif ()

find_library(OV_MSCKF_LIB NAMES ov_msckf_lib PATHS "${OV_BUILD}" NO_DEFAULT_PATH REQUIRED)
message(STATUS "ov_msckf_lib: ${OV_MSCKF_LIB} | OpenCV ${OpenCV_VERSION} | Ceres ${Ceres_VERSION}")

# ovpatch/TrackKLT.cpp (2026-10-24, arlat 0.6.15): patched TrackKLT (KLT overlap, identical output) compiled into the
# executable; it overrides the copy in libov_msckf_lib.so (ENABLE_EXPORTS: the library's TrackKLT vtable binds to ours).
# ovpatch/ovopt.cpp (2026-10-25): --ovopt flag storage (only frame_nocopy / prof work in this build, main.cpp checks).
add_executable(ov_bridge main.cpp ovpatch/TrackKLT.cpp ovpatch/ovopt.cpp)
set_target_properties(ov_bridge PROPERTIES ENABLE_EXPORTS ON)
# 라이브러리 빌드와 같은 매크로여야 헤더(옵션 구조체)가 일치한다 (ovbuild: ROS 끔, ARUCO 켬)
target_compile_definitions(ov_bridge PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=1)
target_include_directories(ov_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/ovpatch
  "${OV_SRC}/ov_msckf/src"
  "${OV_SRC}/ov_core/src"
  "${OV_SRC}/ov_core/src/track"
  "${OV_SRC}/ov_init/src"
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})
target_link_libraries(ov_bridge PRIVATE
  ${OV_MSCKF_LIB} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} Threads::Threads)
target_compile_options(ov_bridge PRIVATE -O2 -Wall)
# 실행 때 LD_LIBRARY_PATH 없이 찾도록
set_target_properties(ov_bridge PROPERTIES
  BUILD_RPATH "${OV_BUILD};${OV_LOCAL}/lib"
  INSTALL_RPATH "${OV_BUILD};${OV_LOCAL}/lib")

# klt_bench (2026-10-24): TrackKLT / remap timing vs OpenCV thread count (CPU only)
add_executable(klt_bench klt_bench.cpp ovpatch/TrackKLT.cpp ovpatch/ovopt.cpp)  # ovopt.cpp: ovpipe symbols (2026-10-26)
target_compile_definitions(klt_bench PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=1)
target_include_directories(klt_bench PRIVATE "${OV_SRC}/ov_core/src" "${OV_SRC}/ov_core/src/track" ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(klt_bench PRIVATE ${OV_MSCKF_LIB} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} Threads::Threads)
target_compile_options(klt_bench PRIVATE -O2)
set_target_properties(klt_bench PROPERTIES BUILD_RPATH "${OV_BUILD};${OV_LOCAL}/lib")
