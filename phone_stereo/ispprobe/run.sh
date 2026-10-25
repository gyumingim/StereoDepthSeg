#!/usr/bin/env bash
# Camera HAL/ISP latency probe (com.scan3d.ispprobe, separate from the live VIO app arlat). Run from WSL.
#   bash phone_stereo/ispprobe/run.sh install <ip:port>          # adb install -r build/ispprobe.apk
#   bash phone_stereo/ispprobe/run.sh dry     <ip:port>          # characteristics + case plan only, camera never opened
#   bash phone_stereo/ispprobe/run.sh run     <ip:port> [extras] # all cases (camera ON ~85 s), e.g. --es cases base,fps60 --ei secs 5
#   bash phone_stereo/ispprobe/run.sh clean   <ip:port>          # force-stop + camera client check
#   bash phone_stereo/ispprobe/run.sh report  <local run dir>
# run: phone_preflight (SKIN < 36 C, heavy apps force-stopped, same as ov_phone_run.sh), pm grant CAMERA (own probe app),
# wake, am start, poll done.txt (SKIN >= 40 C or timeout -> stop), force-stop, then REQUIRE
# dumpsys media.camera "Active Camera Clients" == [] (exit 3 otherwise), pull to scan3d/runs/isp_probe/<run>/, report.
set -uo pipefail
cmd=${1:?install|dry|run|clean|report}; S=${2:?<ip:port> or run dir}; shift 2
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
ADB=${ADB:-/mnt/c/Users/a3162/platform-tools/adb.exe}
PKG=com.scan3d.ispprobe
D=/sdcard/Android/data/$PKG/files/isp
OUT=${OUT:-$ROOT/scan3d/runs/isp_probe}
STAGE=${STAGE:-/mnt/c/Users/a3162/scan3d_stage/isp_probe}
MAXSKIN=${MAXSKIN:-40.0}
a() { MSYS_NO_PATHCONV=1 "$ADB" -s "$S" "$@"; }
sh_() { a shell "$@" | tr -d '\r'; }
skin() { sh_ "dumpsys thermalservice" | sed -n '/Current temperatures from HAL/,$p' | grep -E "mValue=[0-9.]+,.*mName=SKIN" | tail -1 | sed -E 's/.*mValue=([0-9.]+).*/\1/'; }
check_clean() {   # 0 = no active camera client
  local c; c=$(sh_ "dumpsys media.camera" | grep -A1 "Active Camera Clients" | tail -1)
  echo "-- active camera clients: $c"
  if [ "$c" = "[]" ]; then echo "CAMERA CLEAN OK"; return 0; fi
  echo "CAMERA NOT CLEAN: dumpsys media.camera follows"; sh_ "dumpsys media.camera" | sed -n '/Active Camera Clients/,/^$/p'; return 3
}
pull_run() {   # $1 = phone run dir -> local dir; prints the local dir
  local r=$1 name; name=$(basename "$r")
  rm -rf "$STAGE/$name"; mkdir -p "$STAGE" "$OUT"
  a pull "$r" "$(wslpath -w "$STAGE")" > /dev/null
  rm -rf "$OUT/$name"; cp -r "$STAGE/$name" "$OUT/$name"
  echo "$OUT/$name"
}
latest() { sh_ "ls -d $D/run_* 2>/dev/null" | tail -1; }
start_and_wait() {   # $1 = timeout s, rest = am extras
  local tmo=$1; shift
  local before; before=$(latest)
  a shell input keyevent KEYCODE_WAKEUP
  a shell am start -n $PKG/.ProbeActivity "$@"
  local t0 r="" el sk
  t0=$(date +%s)
  while :; do
    sleep 3
    el=$(( $(date +%s) - t0 ))
    [ -z "$r" ] && { r=$(latest); [ "$r" = "$before" ] && r=""; }
    if [ -n "$r" ] && a shell "[ -f $r/done.txt ]"; then echo "done.txt after $el s: $(sh_ "head -1 $r/done.txt")"; break; fi
    sk=$(skin)
    echo "  t=$el s  SKIN ${sk:-?} C  run ${r:-?}"
    if [ -n "$sk" ] && awk -v x="$sk" -v y="$MAXSKIN" 'BEGIN{exit !(x + 0 >= y + 0)}'; then echo "STOP: SKIN $sk >= $MAXSKIN"; break; fi
    [ "$el" -ge "$tmo" ] && { echo "STOP: timeout $tmo s"; break; }
  done
  a shell input keyevent KEYCODE_HOME   # onPause closes the camera if still open
  sleep 1
  a shell am force-stop $PKG
  RUN=$r
}

case $cmd in
install)
  a install -r "$(wslpath -w "$HERE/build/ispprobe.apk")"
  ;;
dry)
  start_and_wait 30 --ez dry true
  check_clean
  [ -n "$RUN" ] && { L=$(pull_run "$RUN"); echo "pulled -> $L"; python3 "$HERE/report.py" "$L"; }
  ;;
run)
  . "$ROOT/scan3d/tools/phone_preflight.sh"
  a shell am force-stop $PKG
  PF=$(mktemp)
  phone_preflight "$ADB" "$S" 36.0 | tee "$PF"; [ "${PIPESTATUS[0]}" = 0 ] || { echo "preflight refused"; exit 2; }
  check_clean || { echo "camera busy before start: not starting"; exit 3; }
  trap 'echo "interrupted -> stopping"; a shell input keyevent KEYCODE_HOME; a shell am force-stop $PKG; check_clean; exit 1' INT TERM
  a shell pm grant $PKG android.permission.CAMERA
  start_and_wait 200 "$@"
  check_clean; rc=$?
  sk=$(skin); echo "SKIN after: ${sk:-?} C"
  if [ -n "$RUN" ]; then
    L=$(pull_run "$RUN"); cp "$PF" "$L/preflight.txt"; echo "SKIN after: ${sk:-?}" >> "$L/preflight.txt"
    echo "pulled -> $L"
    python3 "$HERE/report.py" "$L" | tee "$L/report.txt"
  fi
  exit $rc
  ;;
clean)
  a shell input keyevent KEYCODE_HOME; a shell am force-stop $PKG; check_clean
  ;;
report)
  python3 "$HERE/report.py" "$S"
  ;;
*) echo "unknown $cmd"; exit 2 ;;
esac
