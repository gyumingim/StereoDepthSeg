#!/bin/bash
# nncrun: build the headless ENN .nnc runner APK com.scan3d.nncprobe (own code only). Output in tmp_npu/nncprobe.
set -e
S=$(cd $(dirname $0) && pwd); O=/home/system/scanner/tmp_npu/nncprobe; mkdir -p $O && cd $O
NDK=$HOME/android-ndk-r27c; CXX=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang++
BT=/home/system/android-sdk/android-15; AJ=/home/system/android-sdk/android-35/android.jar
rm -rf apk cls && mkdir -p apk/lib/arm64-v8a cls
$CXX -O2 -std=c++17 -Wall -shared -fPIC -DNNCRUN_JNI $S/nncrun.cpp -o apk/lib/arm64-v8a/libnncrun.so -static-libstdc++ -llog -ldl -pthread
javac -source 8 -target 8 -bootclasspath $AJ -d cls $S/src/com/scan3d/nncprobe/Probe.java 2>&1 | grep -v warning || true
$BT/d8 --lib $AJ --min-api 29 --output apk $(find cls -name '*.class')
$BT/aapt2 link -I $AJ --manifest $S/AndroidManifest.xml -o base.apk
cp base.apk unsigned.apk && (cd apk && zip -q -r ../unsigned.apk classes.dex lib)
$BT/zipalign -f -p 4 unsigned.apk aligned.apk
[ -f debug.keystore ] || keytool -genkeypair -keystore debug.keystore -storepass android -keypass android -alias nnc -keyalg RSA -keysize 2048 -validity 3650 -dname "CN=nncprobe" >/dev/null 2>&1
$BT/apksigner sign --ks debug.keystore --ks-pass pass:android --key-pass pass:android --out nncprobe.apk aligned.apk
ls -la nncprobe.apk; sha256sum nncprobe.apk apk/lib/arm64-v8a/libnncrun.so
