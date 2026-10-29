#!/usr/bin/env bash
# usbrelay 2026-10-29: PC test of the app USB relay (ViopRelayCore) with a fake USB endpoint, then the FC's own stream
# parser over the produced bytes. No phone, no board. Usage: bash run_test.sh [secs per traffic case, default 10]
set -eu
H=$(cd "$(dirname "$0")/../../../.." && pwd)            # worktree root
T=$H/scan3d/runs/usbrelay_20261029/test
FC=${FC:-$HOME/scanner/StereoDepthSeg-fcarm/esp32fc/components/viop/include}   # FC viop_proto.h (stream parser)
JAR=${JAR:-$HOME/scanner/StereoDepthSeg/tools/android-sdk/android-35/android.jar}
W=${W:-/tmp/usbrelay_test}; rm -rf "$W"; mkdir -p "$W/classes" "$W/out"
S=$H/phone_stereo/arlat/src/com/scan3d/arlat
# pcjson first: real (tiny) org.json.JSONObject instead of the android.jar stub
javac -source 8 -target 8 -nowarn -Xlint:-options -cp "$JAR" -d "$W/classes" \
  "$T/pcjson/org/json/JSONObject.java" "$S/ViopRelayCore.java" "$S/ViopImuSender.java" "$T/RelayTest.java"
java -cp "$W/classes:$JAR" com.scan3d.arlat.RelayTest "$W/out" "${1:-10}"
gcc -std=c99 -O1 -Wall -Wextra -Werror -I"$FC" -o "$W/stream_check" "$T/stream_check.c" -lm
"$W/stream_check" "$W"/out/usb_out_*.bin --reset "$W/out/reset_fwd.hex" | tee -a "$W/out/results.txt"; test "${PIPESTATUS[0]}" = 0
cp "$W/out/results.txt" "$T/results.txt"
