set -uo pipefail
H=~/scanner/StereoDepthSeg-integ2
M=~/scanner/StereoDepthSeg
R=~/scanner/tmp_integ2l
B=$R/apk
mkdir -p $B
echo "== APK build $(date)"; git -C $H log --oneline -1
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$H/scan3d/ov_bridge/android/build_integ2/ov_bridge B=$B nice -n 5 bash $H/phone_stereo/arlat/build.sh > $R/apk_build.log 2>&1
echo "apk build rc $?"; tail -2 $R/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging $B/arlat.apk | head -1
$BT/apksigner verify --print-certs $B/arlat.apk | head -2
unzip -lv $B/arlat.apk | grep -E 'libov_bridge|classes.dex|voc_k10L4'
rm -rf $R/apkx && mkdir -p $R/apkx && unzip -q -o $B/arlat.apk 'lib/*' 'assets/*' -d $R/apkx
sha256sum $B/arlat.apk $H/scan3d/ov_bridge/android/build_integ2/ov_bridge $R/apkx/lib/*/libov_bridge.so $R/apkx/assets/voc_k10L4.voc $H/scan3d/ov_bridge/loop/voc_k10L4.voc
ls -la $B/arlat.apk
strings $R/apkx/lib/*/libov_bridge.so | grep -m1 "loop closure dropped"
unzip -p $B/arlat.apk classes.dex | strings | grep -m3 "voc_k10L4\|--loop-vocab"
date
