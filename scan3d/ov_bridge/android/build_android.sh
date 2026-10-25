#!/usr/bin/env bash
# Build OpenVINS ov_bridge for the S24 (arm64-v8a, Android 34) with the NDK. Deps go to ~/scanner/android.
#   bash scan3d/ov_bridge/android/build_android.sh [deps|bridge|all]   (default all)
# 2026-10-25 (ovopt): BUILD_DIR (default <this dir>/build, unchanged) = bridge build directory, OVOPT_FLAGS (default
# empty, unchanged) = extra compile/link flags for ov_bridge (CMake OVOPT_FLAGS), e.g. "-march=armv8.2-a+fp16+dotprod".
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NDK=${NDK:-$HOME/android-ndk-r27c}
DEPS=$HOME/scanner/android
BUILD_DIR=${BUILD_DIR:-$HERE/build}
OVOPT_FLAGS=${OVOPT_FLAGS:-}
TC=(-DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-34
    -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release)
J=${J:-8}
what=${1:-all}
mkdir -p "$DEPS/include"
if [[ $what == deps || $what == all ]]; then
  # header-only deps copied out of the host so the cross build never sees /usr/include
  [[ -d $DEPS/include/eigen3 ]] || cp -r /usr/include/eigen3 "$DEPS/include/"
  [[ -d $DEPS/include/boost ]] || cp -r /usr/include/boost "$DEPS/include/"
  if [[ ! -f $DEPS/ceres-install/lib/libceres.a ]]; then
    cmake -S "$HOME/scanner/src/ceres-solver" -B "$DEPS/ceres-build" "${TC[@]}"       -DCMAKE_INSTALL_PREFIX="$DEPS/ceres-install" -DEigen3_DIR=/usr/share/eigen3/cmake       -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DLAPACK=OFF       -DUSE_CUDA=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_SHARED_LIBS=OFF -DPROVIDE_UNINSTALL_TARGET=OFF
    cmake --build "$DEPS/ceres-build" -j$J
    cmake --install "$DEPS/ceres-build"
  fi
fi
if [[ $what == bridge || $what == all ]]; then
  cmake -S "$HERE" -B "$BUILD_DIR" "${TC[@]}" -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH -DEigen3_DIR=/usr/share/eigen3/cmake \
    "-DOVOPT_FLAGS=$OVOPT_FLAGS"
  cmake --build "$BUILD_DIR" -j$J
  ls -la "$BUILD_DIR/ov_bridge"
fi
