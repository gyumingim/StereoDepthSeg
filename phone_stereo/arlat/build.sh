#!/usr/bin/env bash
# Build the ARCore latency probe APK without Gradle (same tools as phone_stereo/build_android.py).
#   AAR=<unzipped core-1.45.0.aar dir> bash phone_stereo/arlat/build.sh
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$HERE/../../tools/android-sdk
JAR=$SDK/android-35/android.jar
BT=$SDK/android-15
AAR=${AAR:?unzipped ARCore aar dir}
B=$HERE/build
rm -rf "$B"; mkdir -p "$B/classes" "$B/apk/lib/arm64-v8a"
javac -source 8 -target 8 -classpath "$JAR:$AAR/classes.jar" -d "$B/classes" $(find "$HERE/src" -name '*.java')
"$BT/d8" --lib "$JAR" --min-api 28 --output "$B" $(find "$B/classes" -name '*.class') "$AAR/classes.jar"
"$BT/aapt" package -f -M "$HERE/AndroidManifest.xml" -I "$JAR" -F "$B/unsigned.apk"
cp "$AAR"/jni/arm64-v8a/*.so "$B/apk/lib/arm64-v8a/"
cp "$B/classes.dex" "$B/apk/"
(cd "$B/apk" && zip -q -r "$B/unsigned.apk" classes.dex lib)
"$BT/zipalign" -f 4 "$B/unsigned.apk" "$B/aligned.apk"
"$BT/apksigner" sign --ks "$HERE/../android/build/debug.keystore" --ks-pass pass:android --out "$B/arlat.apk" "$B/aligned.apk"
echo "$B/arlat.apk"
