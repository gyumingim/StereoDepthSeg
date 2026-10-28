set -u
cd ~/scanner/tmp_appnpu
NDK=$HOME/android-ndk-r27c; STRIP=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip
date
# 1. reference: lens05 source (= the phone 0.6.29 bridge source) -> stripped sha must equal 6f29ce70 (reproducible build check)
BUILD_DIR=~/scanner/tmp_appnpu/build_android_ref nice -n 5 bash ~/scanner/StereoDepthSeg-lens05/scan3d/ov_bridge/android/build_android.sh bridge > and_ref.log 2>&1; echo "ref rc $?"
$STRIP -o and_ref_stripped build_android_ref/ov_bridge && sha256sum and_ref_stripped
# 2. appnpu source
BUILD_DIR=~/scanner/tmp_appnpu/build_android_new nice -n 5 bash ~/scanner/StereoDepthSeg-appnpu/scan3d/ov_bridge/android/build_android.sh bridge > and_new.log 2>&1; echo "new rc $?"
$STRIP -o and_new_stripped build_android_new/ov_bridge && sha256sum and_new_stripped
grep -i "warning" and_new.log | grep -i ovloop | head
date
