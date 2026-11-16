#!/usr/bin/env bash
# rec960 2026-11-16: build arlat 0.6.41 (std960) — BUILD ONLY, no install (user starts the phone only by saying so).
# Same inputs as 0.6.39 (guidealpha 11-12): ARCore aar, main-worktree SDK + keystore, the SAME stripped ov_bridge as 0.6.39, no NPU assets.
set -euo pipefail
M=/home/system/scanner/StereoDepthSeg
H=/home/system/scanner/StereoDepthSeg-rec960
R=$H/scan3d/runs/rec960_20261116
mkdir -p /tmp/rec960
cp /home/system/scanner/StereoDepthSeg-guidealpha/phone_stereo/arlat/build/apk/lib/arm64-v8a/libov_bridge.so /tmp/rec960/ov_bridge_0639.so
SDK=$M/tools/android-sdk AAR=/home/system/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=/tmp/rec960/ov_bridge_0639.so B=/tmp/rec960/apk nice -n 10 bash $H/phone_stereo/arlat/build.sh > $R/apk_build.log 2>&1
mkdir -p ~/scanner/android/apk_backup
cp /tmp/rec960/apk/arlat.apk ~/scanner/android/apk_backup/arlat_0.6.41_rec960.apk
sha256sum /tmp/rec960/apk/arlat.apk /tmp/rec960/apk/apk/lib/arm64-v8a/libov_bridge.so \
  /home/system/scanner/StereoDepthSeg-guidealpha/phone_stereo/arlat/build/apk/lib/arm64-v8a/libov_bridge.so | tee -a $R/apk_build.log
echo "APK ~/scanner/android/apk_backup/arlat_0.6.41_rec960.apk (NOT installed)" | tee -a $R/apk_build.log
