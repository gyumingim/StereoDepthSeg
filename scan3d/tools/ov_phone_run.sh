#!/usr/bin/env bash
# On-phone OpenVINS timing (arlat 0.6.7, mode ov, ov_cam main). docs/phone_session_60fps_plan.md "On-phone OpenVINS timing".
# Run from WSL (uses Windows adb.exe; adb.exe reads/writes Windows paths, so files go through /mnt/c staging dirs).
#   bash scan3d/tools/ov_phone_run.sh push <ip:port>                       # config sets -> files/ov/ (once per new config)
#   bash scan3d/tools/ov_phone_run.sh run  <ip:port> <cfg> <secs> [am extras, e.g. --ei fps 30 | --es ov_size 640x360]
#   bash scan3d/tools/ov_phone_run.sh clean <ip:port>                      # stop the app + show camera clients / bridge
# run (2026-10-24): pre-run cleanup first (tools/phone_preflight.sh: force-stop our app, am kill-all, force-stop running
# heavy apps, MemAvailable, thermal status, SKIN; refuses when SKIN >= 36 C or unreadable), log -> <run>/preflight.txt
# (refused: $OUT/preflight_refused_<time>.txt). The camera fps follows the config: when the extras give no fps,
# --ei fps <track_frequency of config/phone_ov_main/<cfg>> is added (c2 / c4 / fps30*: 30). A config with its own
# remap (c4: 1920x1080 -> 960x540) needs the 1920x1080 stream: any other ov_size is refused.
#   30 fps VIO (user decision 2026-10-24): C4 = cfg c4 (30 fps, 15 clones, 960x540), fallback C2 = cfg c2 (30 fps, 11, 640x360):
#   bash scan3d/tools/ov_phone_run.sh push <ip:port>; SEG=still0:0:8,hand:8:68,still:68:128 bash scan3d/tools/ov_phone_run.sh run <ip:port> c4 130
# run: refuses to start when SKIN >= 36 C; polls SKIN + thermal status every 5 s; stops (HOME -> onPause writes meta.json,
# then force-stop) at SKIN >= MAXSKIN (40), thermal status >= 2 (moderate), app finished (meta.json), or secs + 30 s;
# always prints the active camera clients and any ov_bridge process (both must be empty); pulls the run dir to
# scan3d/runs/phone_ov_timing/<run>/ (+ thermal_pc.txt) and prints the timing report (tools/ov_phone_timing.py).
# On-screen guide (arlat 0.6.12): SEG=still0:0:8,hand:8:68,still:68:128 also sends a Korean step guide built from it
# (still0 -> 모서리에 세워 두고 손 떼기, hand -> 들고 방 안 움직이기, still -> 다시 세워 두기, other names as is; step length
# = t1 - previous end) with guide_secs = last t1; GUIDE="label:secs;..." overrides it (guide_secs = GUIDE_SECS or the sum).
# The phone counts down from the first frame fed to OpenVINS (after auto ISO) and stops itself at the end (meta.json).
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
GUIDE=${GUIDE:-}
GUIDE_SECS=${GUIDE_SECS:-}
seg_guide() {   # SEG "name:t0:t1,..." -> "label:secs;..." (Korean labels for the known names)
  awk -v s="$1" 'BEGIN{ n = split(s, a, ","); e = 0; out = ""
    for (i = 1; i <= n; i++) { split(a[i], f, ":"); nm = f[1]; d = f[3] - e; e = f[3]; if (d < 1) continue
      lb = nm; if (nm == "still0") lb = "모서리에 세워 두고 손 떼기"; else if (nm == "hand") lb = "들고 방 안 움직이기"; else if (nm == "still") lb = "다시 세워 두기"
      out = out (out == "" ? "" : ";") lb ":" d }
    print out }'
}
guide_sum() { awk -v s="$1" 'BEGIN{ n = split(s, a, ";"); t = 0; for (i = 1; i <= n; i++) { m = split(a[i], f, ":"); t += f[m] } print t }'; }
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
. "$SCAN3D/tools/phone_preflight.sh"
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
  # camera fps / stream size must match the config (track_frequency, remap size)
  cdir=${CFGSRC:-$SCAN3D/config/phone_ov_main}/$CFG
  ex=" $* "
  if [[ "$ex" != *" fps "* ]] && [ -f "$cdir/estimator_config.yaml" ]; then
    tf=$(awk -F: '/^track_frequency:/{gsub(/ /, "", $2); printf "%d", $2 + 0.5}' "$cdir/estimator_config.yaml")
    [ -n "$tf" ] && [ "$tf" -gt 0 ] && { set -- "$@" --ei fps "$tf"; echo "config $CFG: track_frequency $tf -> --ei fps $tf"; }
  fi
  if [ -f "$cdir/remap_1920x1080.bin" ] && [[ "$ex" == *" ov_size "* ]] && [[ "$ex" != *" ov_size 1920x1080 "* ]]; then
    echo "config $CFG has its own remap for the 1920x1080 stream only: drop the ov_size extra"; exit 2
  fi
  [ -f "$cdir/estimator_config.yaml" ] || echo "note: $cdir not found on the PC (fps / size not checked)"
  PF=$(mktemp)
  a shell am force-stop $PKG     # no stale instance of our app (the run starts it fresh)
  phone_preflight "$ADB" "$S" 36.0 | tee "$PF"; pfrc=${PIPESTATUS[0]}
  if [ "$pfrc" != 0 ]; then
    mkdir -p "$OUT"; cp "$PF" "$OUT/preflight_refused_$(date +%Y%m%d_%H%M%S).txt"; exit 2
  fi
  before=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
  a shell input keyevent KEYCODE_WAKEUP
  G=$GUIDE; [ -z "$G" ] && [ -n "$SEG" ] && G=$(seg_guide "$SEG")
  gx=()
  if [ -n "$G" ]; then
    GS=${GUIDE_SECS:-$(guide_sum "$G")}
    echo "guide ($GS s from the first fed frame): $G"
    [ "$GS" -gt "$SECS" ] && echo "note: guide $GS s > secs $SECS (PC timeout = secs + 30 s): give secs >= guide"
    # adb shell joins the args into one device shell command line: single-quote the label string (spaces)
    gx=(--es guide "'${G//\'/}'" --ei guide_secs "$GS")
  fi
  a shell am start -n $PKG/.MainActivity --es mode ov --es ov_cam main --es ov_cfg "$CFG" --ei secs "$SECS" "${gx[@]}" "$@"
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
  { cat "$PF"; echo "am extras: $*"; } > "$OUT/$name/preflight.txt"
  echo "pulled -> $OUT/$name"
  py=$SCAN3D/../venv_ffs/bin/python
  [ -x "$py" ] || py=python3
  "$py" "$SCAN3D/tools/ov_phone_timing.py" "$OUT/$name" ${SEG:+--seg "$SEG"}
  ;;
*) echo "push|run|clean"; exit 2 ;;
esac
