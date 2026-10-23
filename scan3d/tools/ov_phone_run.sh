#!/usr/bin/env bash
# On-phone OpenVINS timing (arlat 0.6.7, mode ov, ov_cam main). docs/phone_session_60fps_plan.md "On-phone OpenVINS timing".
# Run from WSL (uses Windows adb.exe; adb.exe reads/writes Windows paths, so files go through /mnt/c staging dirs).
#   bash scan3d/tools/ov_phone_run.sh push <ip:port>                       # config sets -> files/ov/ (once per new config)
#   bash scan3d/tools/ov_phone_run.sh run  <ip:port> <cfg> <secs> [am extras, e.g. --ei fps 30 | --es ov_size 640x360]
#   bash scan3d/tools/ov_phone_run.sh clean <ip:port>                      # stop the app + show camera clients / bridge
# run: refuses to start when SKIN >= 36 C; polls SKIN + thermal status every 5 s; stops (HOME -> onPause writes meta.json,
# then force-stop) at SKIN >= MAXSKIN (40), thermal status >= 2 (moderate), app finished (meta.json), or secs + 30 s;
# always prints the active camera clients and any ov_bridge process (both must be empty); pulls the run dir to
# scan3d/runs/phone_ov_timing/<run>/ (+ thermal_pc.txt) and prints the timing report (tools/ov_phone_timing.py).
set -uo pipefail
cmd=${1:?push|run|clean}; S=${2:?<ip:port>}; shift 2
ADB=${ADB:-/mnt/c/Users/a3162/platform-tools/adb.exe}
PKG=com.scan3d.arlat
OVD=/sdcard/Android/data/$PKG/files/ov
MAXSKIN=${MAXSKIN:-40.0}
SCAN3D=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$SCAN3D/runs/phone_ov_timing}
STAGE=${STAGE:-/mnt/c/Users/a3162/scan3d_stage}
SEG=${SEG:-}
a() { "$ADB" -s "$S" "$@"; }
sh_() { a shell "$@" | tr -d '\r'; }
stop_app() {   # HOME -> onPause closes the files and writes meta.json last; wait for it (<= 12 s), then force-stop
  a shell input keyevent KEYCODE_HOME
  local r; r=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
  for _ in 1 2 3 4 5 6; do sleep 2; [ -n "$r" ] && a shell "[ -f $r/meta.json ]" && break; done
  a shell am force-stop $PKG
}
check_clean() {
  echo "-- active camera clients (must be empty):"
  sh_ "dumpsys media.camera | sed -n '/Active Camera Clients/,/^\$/p'"
  echo "-- ov_bridge processes (must be empty):"
  sh_ "ps -A | grep -i ov_bridge || echo none"
}
skin() { sh_ "dumpsys thermalservice" | sed -n '/Current temperatures from HAL/,$p' | grep -E "mName=SKIN" | tail -1 | sed -E 's/.*mValue=([0-9.]+).*/\1/'; }
tstatus() { sh_ "dumpsys thermalservice" | grep -m1 "Thermal Status" | grep -oE "[0-9]+" | head -1; }
ge() { awk -v x="$1" -v y="$2" 'BEGIN{exit !(x != "" && x + 0 >= y + 0)}'; }

case $cmd in
push)
  src=${CFGSRC:-$SCAN3D/config/phone_ov_main}
  [ -f "$src/remap_1920x1080.bin" ] || { echo "no $src/remap_1920x1080.bin: run scan3d/ov_bridge/android/make_phone_ov_main.py"; exit 2; }
  rm -rf "$STAGE/phone_ov_main"; mkdir -p "$STAGE"; cp -r "$src" "$STAGE/phone_ov_main"
  w=$(wslpath -w "$STAGE/phone_ov_main")
  sh_ "mkdir -p $OVD"
  for f in "$STAGE"/phone_ov_main/*; do a push "$w\\$(basename "$f")" "$OVD/" > /dev/null && echo "pushed $(basename "$f")"; done
  sh_ "ls -la $OVD $OVD/base60"
  ;;
clean)
  stop_app; check_clean
  ;;
run)
  CFG=${1:?<cfg>}; SECS=${2:?<secs>}; shift 2
  trap 'echo "interrupted -> stopping the app"; stop_app; check_clean; exit 1' INT TERM
  s0=$(skin); st0=$(tstatus); echo "SKIN before: ${s0:-?} C, thermal status ${st0:-?}"
  if [ -z "$s0" ]; then echo "cannot read SKIN from dumpsys thermalservice: the SKIN stop rule would not work, not starting"; exit 2; fi
  if ge "$s0" 36.0; then echo "phone too warm (SKIN >= 36 C): cool it first"; exit 2; fi
  before=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
  a shell input keyevent KEYCODE_WAKEUP
  a shell am start -n $PKG/.MainActivity --es mode ov --es ov_cam main --es ov_cfg "$CFG" --ei secs "$SECS" "$@"
  LOG=$(mktemp); t0=$(date +%s); reason=""
  while :; do
    sleep 5
    el=$(( $(date +%s) - t0 )); sk=$(skin); ts=$(tstatus)
    echo "$(date +%s) t=${el}s SKIN ${sk:-?} status ${ts:-?}" | tee -a "$LOG"
    if ge "$sk" "$MAXSKIN"; then reason="SKIN $sk >= $MAXSKIN"; break; fi
    if ge "$ts" 2; then reason="thermal status $ts"; break; fi
    run=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
    if [ -n "$run" ] && [ "$run" != "$before" ] && a shell "[ -f $run/meta.json ]"; then reason="app finished"; break; fi
    if [ "$el" -gt $(( SECS + 30 )) ]; then reason="timeout"; break; fi
  done
  echo "stop: $reason"
  stop_app
  check_clean
  run=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
  if [ -z "$run" ] || [ "$run" = "$before" ]; then echo "no new run dir on the phone (app failed to start? see adb logcat -s arlat)"; exit 1; fi
  name=$(basename "$run")
  mkdir -p "$STAGE" "$OUT"
  a pull "$run" "$(wslpath -w "$STAGE")" > /dev/null
  cp -r "$STAGE/$name" "$OUT/"
  { cat "$LOG"; echo "stop_pc: $reason"; } > "$OUT/$name/thermal_pc.txt"
  echo "pulled -> $OUT/$name"
  py=$SCAN3D/../venv_ffs/bin/python
  [ -x "$py" ] || py=python3
  "$py" "$SCAN3D/tools/ov_phone_timing.py" "$OUT/$name" ${SEG:+--seg "$SEG"}
  ;;
*) echo "push|run|clean"; exit 2 ;;
esac
