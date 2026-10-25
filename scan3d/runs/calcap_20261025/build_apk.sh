# calcap 0.6.25: arlat APK from this worktree, reusing the already built ov_bridge (/tmp/calcap/ov_bridge, unchanged).
#   bash build_apk.sh -> /tmp/calcap/apk/arlat.apk
set -euo pipefail
H=~/scanner/StereoDepthSeg-calcap
M=~/scanner/StereoDepthSeg
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=/tmp/calcap/ov_bridge B=/tmp/calcap/apk bash $H/phone_stereo/arlat/build.sh > /tmp/calcap/apk_build.log 2>&1
tail -1 /tmp/calcap/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging /tmp/calcap/apk/arlat.apk | head -1
$BT/apksigner verify /tmp/calcap/apk/arlat.apk && echo SIGN_OK
