# appnpu: build arlat 0.6.30 (vc 36) from this worktree: bridge = appnpu Android build, NPU models as assets (not in git)
set -uo pipefail
H=~/scanner/StereoDepthSeg-appnpu
M=~/scanner/StereoDepthSeg
R=~/scanner/tmp_appnpu
B=$R/apk
OVB=$R/build_android_new/ov_bridge
echo "== APK build $(date)"; git -C $H log --oneline -1; git -C $H status --short phone_stereo scan3d/ov_bridge | grep -v "^??" | head
SDK=$M/tools/android-sdk AAR=~/scanner/android/arcore/aar KS=$M/phone_stereo/android/build/debug.keystore \
  OVB=$OVB B=$B NPU_ASSETS=$R/npu_assets nice -n 5 bash $H/phone_stereo/arlat/build.sh > $R/apk_build.log 2>&1
echo "apk build rc $?"; tail -2 $R/apk_build.log
BT=$M/tools/android-sdk/android-15
$BT/aapt dump badging $B/arlat.apk | head -1
$BT/aapt dump badging $B/arlat.apk | grep -i "native\|uses-library" | head
$BT/apksigner verify --print-certs $B/arlat.apk | head -1
OLDAPK=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/lens05/apk/arlat_0.6.29.apk
$BT/apksigner verify --print-certs $OLDAPK | head -1
rm -rf $R/apkx && mkdir -p $R/apkx && unzip -q -o $B/arlat.apk lib/* assets/* -d $R/apkx
rm -rf $R/apk_old && mkdir -p $R/apk_old && unzip -q -o $OLDAPK lib/* assets/* -d $R/apk_old
unzip -l $B/arlat.apk | grep -v "META-INF" | tail -20
sha256sum $B/arlat.apk $OLDAPK $R/apkx/lib/*/*.so $R/apk_old/lib/*/*.so $R/apkx/assets/* $R/apkx/assets/npu/* $R/apk_old/assets/* | sed "s#$HOME#~#"
date
