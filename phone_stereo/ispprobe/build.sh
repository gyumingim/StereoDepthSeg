#!/usr/bin/env bash
# Build the camera HAL/ISP latency probe APK (com.scan3d.ispprobe) without Gradle. Separate app: the live VIO app
# (phone_stereo/arlat) is not touched. Same tools as phone_stereo/arlat/build.sh, no native code.
#   bash phone_stereo/ispprobe/build.sh      -> phone_stereo/ispprobe/build/ispprobe.apk
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=$HERE/../../tools/android-sdk
JAR=$SDK/android-35/android.jar
BT=$SDK/android-15
B=$HERE/build
rm -rf "$B"; mkdir -p "$B/classes" "$B/apk"
javac -source 8 -target 8 -Xlint:-options -classpath "$JAR" -d "$B/classes" $(find "$HERE/src" -name '*.java')
"$BT/d8" --lib "$JAR" --min-api 28 --output "$B" $(find "$B/classes" -name '*.class')
"$BT/aapt" package -f -M "$HERE/AndroidManifest.xml" -I "$JAR" -F "$B/unsigned.apk"
cp "$B/classes.dex" "$B/apk/"
(cd "$B/apk" && zip -q -r "$B/unsigned.apk" classes.dex)
"$BT/zipalign" -f 4 "$B/unsigned.apk" "$B/aligned.apk"
"$BT/apksigner" sign --ks "$HERE/../android/build/debug.keystore" --ks-pass pass:android --out "$B/ispprobe.apk" "$B/aligned.apk"
echo "$B/ispprobe.apk"
