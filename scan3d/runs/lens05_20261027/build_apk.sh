set -uo pipefail
H=~/scanner/StereoDepthSeg-lens05
M=~/scanner/StereoDepthSeg
R=~/scanner/tmp_lens05
B=$R/apk
OVB=~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/android/build_integ2/ov_bridge
mkdir -p $B
echo "== APK build $(date)"; git -C $H log --oneline -1; git -C $H status --short phone_stereo | head
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$OVB B=$B nice -n 5 bash $H/phone_stereo/arlat/build.sh > $R/apk_build.log 2>&1
echo "apk build rc $?"; tail -2 $R/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging $B/arlat.apk | head -1
$BT/apksigner verify --print-certs $B/arlat.apk | head -1
rm -rf $R/apkx && mkdir -p $R/apkx && unzip -q -o $B/arlat.apk 'lib/*' 'assets/*' -d $R/apkx
# same native bridge as 0.6.28? (lib inside the installed 0.6.28 APK copy)
OLDAPK=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/integ2/apk/arlat_0.6.28.apk
rm -rf $R/apk_old && mkdir -p $R/apk_old && unzip -q -o $OLDAPK 'lib/*' 'assets/*' -d $R/apk_old
sha256sum $B/arlat.apk $OLDAPK $R/apkx/lib/*/*.so $R/apk_old/lib/*/*.so $R/apkx/assets/* $R/apk_old/assets/* | sed "s#$HOME#~#"
unzip -p $B/arlat.apk classes.dex | strings | grep -m2 "says \|camera frame"
date
