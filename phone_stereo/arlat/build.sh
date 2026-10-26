#!/usr/bin/env bash
# Build the ARCore latency probe APK without Gradle (same tools as phone_stereo/build_android.py).
#   AAR=<unzipped core-1.45.0.aar dir> bash phone_stereo/arlat/build.sh
# 2026-10-25 (ovopt): optional overrides, defaults unchanged: SDK (android sdk dir), OVB (ov_bridge binary),
# KS (signing keystore), B (build output dir) - e.g. to build from a git worktree that has no tools/ or keystore.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=${SDK:-$HERE/../../tools/android-sdk}
JAR=$SDK/android-35/android.jar
BT=$SDK/android-15
AAR=${AAR:?unzipped ARCore aar dir}
B=${B:-$HERE/build}
KS=${KS:-$HERE/../android/build/debug.keystore}
rm -rf "$B"; mkdir -p "$B/classes" "$B/apk/lib/arm64-v8a"
javac -source 8 -target 8 -classpath "$JAR:$AAR/classes.jar" -d "$B/classes" $(find "$HERE/src" -name '*.java')
"$BT/d8" --lib "$JAR" --min-api 28 --output "$B" $(find "$B/classes" -name '*.class') "$AAR/classes.jar"
# 0.6.28 (integ2, ovloop): loop-closure vocabulary as an asset (OvRunner copies it to files/ov/); VOC overrides the source
VOC=${VOC:-$HERE/../../scan3d/ov_bridge/loop/voc_k10L4.voc}
mkdir -p "$B/assets"; cp "$VOC" "$B/assets/voc_k10L4.voc"
"$BT/aapt" package -f -M "$HERE/AndroidManifest.xml" -A "$B/assets" -I "$JAR" -F "$B/unsigned.apk"
cp "$AAR"/jni/arm64-v8a/*.so "$B/apk/lib/arm64-v8a/"
# live OpenVINS (mode ov): the bridge executable ships as a "library" so Android extracts it executable (W^X)
NDK=${NDK:-$HOME/android-ndk-r27c}
OVB=${OVB:-$HERE/../../scan3d/ov_bridge/android/build/ov_bridge}
"$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip" -o "$B/apk/lib/arm64-v8a/libov_bridge.so" "$OVB"
cp "$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" "$B/apk/lib/arm64-v8a/"
# loop closure over ARCore (jni/arloop.*): OpenCV + Ceres static inside libarloop.so
JB=$HERE/jni/build-android
cmake -S "$HERE/jni" -B "$JB" -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-34 -DANDROID_STL=c++_shared -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH \
  -DEigen3_DIR=/usr/share/eigen3/cmake > /dev/null
cmake --build "$JB" -j8 > /dev/null
"$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip" -o "$B/apk/lib/arm64-v8a/libarloop.so" "$JB/libarloop.so"
cp "$B/classes.dex" "$B/apk/"
(cd "$B/apk" && zip -q -r "$B/unsigned.apk" classes.dex lib)
"$BT/zipalign" -f 4 "$B/unsigned.apk" "$B/aligned.apk"
"$BT/apksigner" sign --ks "$KS" --ks-pass pass:android --out "$B/arlat.apk" "$B/aligned.apk"
echo "$B/arlat.apk"
