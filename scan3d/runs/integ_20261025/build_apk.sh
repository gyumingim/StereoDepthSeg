# integ-20261025: phone ov_bridge (NDK) + arlat 0.6.20 APK from this worktree (tools / keystore / AAR of the main checkout).
#   bash build_apk.sh  -> /tmp/integ/apk/arlat.apk (+ aapt badging, apksigner verify, sha256 of the bridge)
set -euo pipefail
H=~/scanner/StereoDepthSeg-integ
M=~/scanner/StereoDepthSeg
BUILD_DIR=$H/scan3d/ov_bridge/android/build_integ bash $H/scan3d/ov_bridge/android/build_android.sh bridge > /tmp/integ/and_build.log 2>&1
tail -2 /tmp/integ/and_build.log
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$H/scan3d/ov_bridge/android/build_integ/ov_bridge B=/tmp/integ/apk bash $H/phone_stereo/arlat/build.sh > /tmp/integ/apk_build.log 2>&1
tail -1 /tmp/integ/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging /tmp/integ/apk/arlat.apk | head -3
$BT/apksigner verify --print-certs /tmp/integ/apk/arlat.apk | head -3
unzip -l /tmp/integ/apk/arlat.apk | grep -E 'libov_bridge|classes.dex'
sha256sum /tmp/integ/apk/arlat.apk $H/scan3d/ov_bridge/android/build_integ/ov_bridge /tmp/integ/apk/apk/lib/arm64-v8a/libov_bridge.so
