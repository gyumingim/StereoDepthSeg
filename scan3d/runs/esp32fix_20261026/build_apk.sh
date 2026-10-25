# esp32fix-20261026: compile check only (NOT installed): phone ov_bridge (NDK) + arlat APK from this worktree, same recipe
# as runs/integ_20261025/build_apk.sh, outputs /tmp/esp32fix/apk.
set -euo pipefail
H=~/scanner/StereoDepthSeg-esp32fix
M=~/scanner/StereoDepthSeg
mkdir -p /tmp/esp32fix
J=4 BUILD_DIR=$H/scan3d/ov_bridge/android/build_esp32fix nice -n 5 bash $H/scan3d/ov_bridge/android/build_android.sh bridge > /tmp/esp32fix/and_build.log 2>&1
tail -2 /tmp/esp32fix/and_build.log
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$H/scan3d/ov_bridge/android/build_esp32fix/ov_bridge B=/tmp/esp32fix/apk bash $H/phone_stereo/arlat/build.sh > /tmp/esp32fix/apk_build.log 2>&1
tail -1 /tmp/esp32fix/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging /tmp/esp32fix/apk/arlat.apk | head -1
unzip -l /tmp/esp32fix/apk/arlat.apk | grep -E 'libov_bridge|classes.dex'
grep -c "viop-hb-src" $H/scan3d/ov_bridge/android/build_esp32fix/ov_bridge || true
sha256sum /tmp/esp32fix/apk/arlat.apk $H/scan3d/ov_bridge/android/build_esp32fix/ov_bridge
