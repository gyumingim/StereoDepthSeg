set -uo pipefail
H=~/scanner/StereoDepthSeg-integ2
M=~/scanner/StereoDepthSeg
R=$H/scan3d/runs/integ2_20261026
mkdir -p $R /tmp/integ2
echo "== ESP32 PC checks (esp32fc/test, make) $(date)"
cd $H/esp32fc/test && head -30 Makefile | grep -E '^[a-z_]+:|CC|CFLAGS' | head -8
( cd $H/esp32fc/test && make clean >/dev/null 2>&1; make 2>&1 | tail -4 ) > $R/esp32_test.log 2>&1
tail -3 $R/esp32_test.log
( cd $H/esp32fc/test && bash host_syntax.sh 2>&1 | tail -3 ) > $R/esp32_syntax.log 2>&1; tail -2 $R/esp32_syntax.log
echo "== APK build $(date)"
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$H/scan3d/ov_bridge/android/build_integ2/ov_bridge B=/tmp/integ2/apk nice -n 5 bash $H/phone_stereo/arlat/build.sh > /tmp/integ2/apk_build.log 2>&1
echo "apk build rc $?"; tail -2 /tmp/integ2/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging /tmp/integ2/apk/arlat.apk | head -1
$BT/apksigner verify --print-certs /tmp/integ2/apk/arlat.apk | head -2
unzip -l /tmp/integ2/apk/arlat.apk | grep -E 'libov_bridge|classes.dex'
sha256sum /tmp/integ2/apk/arlat.apk $H/scan3d/ov_bridge/android/build_integ2/ov_bridge
date
