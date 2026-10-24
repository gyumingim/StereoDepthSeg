cmake_minimum_required(VERSION 3.10)
project(ov_bridge CXX)

# OpenVINS (ROS 없이 빌드된 ov_msckf_lib) 위치. 필요하면 -DOV_SRC=... -DOV_BUILD=... 로 바꾼다.
set(OV_SRC "$ENV{HOME}/scanner/src/open_vins" CACHE PATH "open_vins source root")
set(OV_BUILD "$ENV{HOME}/scanner/src/ovbuild" CACHE PATH "ov_msckf build dir (libov_msckf_lib.so)")
set(OV_LOCAL "$ENV{HOME}/scanner/local" CACHE PATH "Ceres install prefix")
list(APPEND CMAKE_PREFIX_PATH "${OV_LOCAL}")

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

find_library(OV_MSCKF_LIB NAMES ov_msckf_lib PATHS "${OV_BUILD}" NO_DEFAULT_PATH REQUIRED)
message(STATUS "ov_msckf_lib: ${OV_MSCKF_LIB} | OpenCV ${OpenCV_VERSION} | Ceres ${Ceres_VERSION}")

# ovpatch/TrackKLT.cpp (2026-10-24, arlat 0.6.15): patched TrackKLT (KLT overlap, identical output) compiled into the
# executable; it overrides the copy in libov_msckf_lib.so (ENABLE_EXPORTS: the library's TrackKLT vtable binds to ours).
add_executable(ov_bridge main.cpp ovpatch/TrackKLT.cpp)
set_target_properties(ov_bridge PROPERTIES ENABLE_EXPORTS ON)
# 라이브러리 빌드와 같은 매크로여야 헤더(옵션 구조체)가 일치한다 (ovbuild: ROS 끔, ARUCO 켬)
target_compile_definitions(ov_bridge PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=1)
target_include_directories(ov_bridge PRIVATE
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
add_executable(klt_bench klt_bench.cpp ovpatch/TrackKLT.cpp)
target_compile_definitions(klt_bench PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=1)
target_include_directories(klt_bench PRIVATE "${OV_SRC}/ov_core/src" "${OV_SRC}/ov_core/src/track" ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(klt_bench PRIVATE ${OV_MSCKF_LIB} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} Threads::Threads)
target_compile_options(klt_bench PRIVATE -O2)
set_target_properties(klt_bench PROPERTIES BUILD_RPATH "${OV_BUILD};${OV_LOCAL}/lib")
