# integ2 app 0.6.27 (vc 33): profiles + gyro_klt=5. Same build as ../build_check.sh (APK part only).
set -uo pipefail
H=~/scanner/StereoDepthSeg-integ2
M=~/scanner/StereoDepthSeg
B=/tmp/integ2g/apk
mkdir -p $B
echo "== APK build $(date)"; git -C $H log --oneline -1
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$H/scan3d/ov_bridge/android/build_integ2/ov_bridge B=$B nice -n 5 bash $H/phone_stereo/arlat/build.sh > /tmp/integ2g/apk_build.log 2>&1
echo "apk build rc $?"; tail -2 /tmp/integ2g/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging $B/arlat.apk | head -1
$BT/apksigner verify --print-certs $B/arlat.apk | head -2
$BT/apksigner verify --print-certs /tmp/integ2g/arlat_0.6.26.apk | head -1
unzip -l $B/arlat.apk | grep -E 'libov_bridge|classes.dex'
rm -rf /tmp/integ2g/apkx && mkdir -p /tmp/integ2g/apkx && unzip -q -o $B/arlat.apk 'lib/*' -d /tmp/integ2g/apkx
sha256sum $B/arlat.apk $H/scan3d/ov_bridge/android/build_integ2/ov_bridge /tmp/integ2g/apkx/lib/*/libov_bridge.so
strings /tmp/integ2g/apkx/lib/*/libov_bridge.so | grep -m1 "ekf_sparse,gyro_klt=5"
unzip -p $B/arlat.apk classes.dex | strings | grep -m1 "ekf_sparse,gyro_klt=5"
date
