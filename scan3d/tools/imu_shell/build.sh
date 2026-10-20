#!/usr/bin/env bash
# build imulog.dex (run with app_process as adb shell user; no app install)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SDK=/home/system/scanner/StereoDepthSeg/tools/android-sdk
rm -rf "$HERE/cls"; mkdir -p "$HERE/cls"
javac -nowarn -source 8 -target 8 -classpath "$SDK/android-35/android.jar" -d "$HERE/cls" "$HERE/src/ImuLog.java"
"$SDK/android-15/d8" --lib "$SDK/android-35/android.jar" --min-api 28 --output "$HERE" "$HERE"/cls/*.class
mv "$HERE/classes.dex" "$HERE/imulog.dex"
ls -la "$HERE/imulog.dex"
