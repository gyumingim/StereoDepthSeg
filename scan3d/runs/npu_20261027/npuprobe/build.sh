#!/bin/bash
# npu: build the NNAPI probe (own code only) as (1) adb-shell executable and (2) minimal APK com.scan3d.npuprobe.
set -e
S=$(cd $(dirname $0) && pwd); O=/home/system/scanner/tmp_npu/npuprobe; mkdir -p $O && cd $O
NDK=$HOME/android-ndk-r27c; CXX=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android34-clang++
BT=/home/system/android-sdk/android-15; AJ=/home/system/android-sdk/android-35/android.jar
$CXX -O2 -std=c++17 $S/probe.cpp -o npuprobe_shell -static-libstdc++ -lneuralnetworks -llog -ldl
rm -rf apk cls && mkdir -p apk/lib/arm64-v8a cls
$CXX -O2 -std=c++17 -shared -fPIC -DNPUPROBE_JNI $S/probe.cpp -o apk/lib/arm64-v8a/libnpuprobe.so -static-libstdc++ -lneuralnetworks -llog -ldl
javac -source 8 -target 8 -bootclasspath $AJ -d cls $S/src/com/scan3d/npuprobe/Probe.java 2>&1 | grep -v warning || true
$BT/d8 --lib $AJ --min-api 29 --output apk $(find cls -name '*.class')
$BT/aapt2 link -I $AJ --manifest $S/AndroidManifest.xml -o base.apk
cp base.apk unsigned.apk && (cd apk && zip -q -r ../unsigned.apk classes.dex lib)
$BT/zipalign -f -p 4 unsigned.apk aligned.apk
[ -f debug.keystore ] || keytool -genkeypair -keystore debug.keystore -storepass android -keypass android -alias npu -keyalg RSA -keysize 2048 -validity 3650 -dname "CN=npuprobe" >/dev/null 2>&1
$BT/apksigner sign --ks debug.keystore --ks-pass pass:android --key-pass pass:android --out npuprobe.apk aligned.apk
ls -la npuprobe.apk npuprobe_shell; sha256sum npuprobe.apk npuprobe_shell
