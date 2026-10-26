#!/bin/bash
set -e
O=/home/system/scanner/tmp_ovxfeat/ort
mkdir -p $O && cd $O
[ -d headers ] || unzip -q -o /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ovxfeat_dl/ort/ort.aar 'headers/*' 'jni/arm64-v8a/*'
NDK=$HOME/android-ndk-r27c
CXX=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang++
$CXX -O2 -std=c++17 -I headers /home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/ovxfeat_20261026/phone/ort_bench.cpp \
  -L jni/arm64-v8a -lonnxruntime -o ort_bench -Wl,-rpath,'$ORIGIN'
ls -la ort_bench jni/arm64-v8a/
sha256sum ort_bench jni/arm64-v8a/libonnxruntime.so
$CXX -O2 -std=c++17 -I headers /home/system/scanner/StereoDepthSeg-ovxfeat/scan3d/runs/ovxfeat_20261026/phone/kf_bench.cpp \
  -L jni/arm64-v8a -lonnxruntime -o kf_bench -Wl,-rpath,'$ORIGIN'
sha256sum kf_bench
