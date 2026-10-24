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
# Live path page (2026-10-24): LIVE=1 bash scan3d/tools/ov_phone_run.sh run <ip:port> c4 130  -> http://100.92.214.97:8798
# (tools/ov_live_view.py; starts it if needed, sets adb reverse tcp:8798 tcp:8794, copies the lines to <run>/ov_live.jsonl).
# Perf A/B knobs (arlat 0.6.14, default off): --ei hint_ms 15 (hint session target 15 ms instead of 1000/fps),
# --es cpus 7,8,9 (ov_bridge --cpus: all bridge threads pinned). The report prints "perf knobs" + "[ov_bridge] affinity" lines.
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
# only Temperature{mValue=..., mName=SKIN ..} lines: the TemperatureThreshold{.. mName=SKIN ..} line has no value (same fix as phone_preflight.sh)
skin() { sh_ "dumpsys thermalservice" | sed -n '/Current temperatures from HAL/,$p' | grep -E "mValue=[0-9.]+,.*mName=SKIN" | tail -1 | sed -E 's/.*mValue=([0-9.]+).*/\1/'; }
# LIVE=1: PC live path page (tools/ov_live_view.py, Windows Python) at http://$LIVE_HOST:$LIVE_PORT. Started if not
# answering; phone 127.0.0.1:8798 (Streamer) -> adb reverse -> PC 127.0.0.1:$LIVE_STREAM (8798 on the PC is
# vio_bench_server's); a new session per run (tag = cfg); after the pull the received lines -> <run>/ov_live.jsonl.
LIVE=${LIVE:-}
LIVE_HOST=${LIVE_HOST:-100.92.214.97}
LIVE_PORT=${LIVE_PORT:-8798}
LIVE_STREAM=${LIVE_STREAM:-8794}
# python.org install, not the Store alias (Store Python redirects AppData writes into its package folder)
LIVE_PY=${LIVE_PY:-'C:\Users\a3162\AppData\Local\Programs\Python\Python312\pythonw.exe'}
CURL=${CURL:-/mnt/c/Windows/System32/curl.exe}
PSH=${PSH:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}
live_get() { "$CURL" -s -m 3 "http://$LIVE_HOST:$LIVE_PORT$1" | tr -d '\r'; }
live_up() {   # $1 = session tag; never fails the run (the run works without the page)
  if ! live_get /api/ping | grep -q ov_live_view; then
    local wp; wp=$(wslpath -w "$SCAN3D/tools/ov_live_view.py")
    echo "live view: starting $wp"
    "$PSH" -NoProfile -Command "Start-Process -WindowStyle Hidden -FilePath '$LIVE_PY' -ArgumentList '\"$wp\"','--host','$LIVE_HOST','--port','$LIVE_PORT','--stream-port','$LIVE_STREAM','--serial','$S'"
    for _ in 1 2 3 4 5 6 7 8 9 10; do sleep 1; live_get /api/ping | grep -q ov_live_view && break; done
  fi
  live_get /api/ping | grep -q ov_live_view || { echo "live view: not answering at http://$LIVE_HOST:$LIVE_PORT (run continues)"; return 0; }
  a reverse tcp:8798 tcp:"$LIVE_STREAM" > /dev/null && echo "live view: adb reverse tcp:8798 tcp:$LIVE_STREAM"
  live_get "/api/new?tag=$1" > /dev/null
  echo "live view: http://$LIVE_HOST:$LIVE_PORT"
}
live_save() {   # $1 = local run folder
  local f; f=$(live_get /api/session | python3 -c 'import json,sys; print(json.load(sys.stdin).get("file") or "")' 2>/dev/null)
  if [ -n "$f" ] && [ -f "$(wslpath -u "$f")" ]; then cp "$(wslpath -u "$f")" "$1/ov_live.jsonl" && echo "live view lines -> $1/ov_live.jsonl ($(wc -l < "$1/ov_live.jsonl") lines, from $f)"
  else echo "live view: no saved lines (file: ${f:-none})"; fi
}
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
  # a config with its own remap needs a map for the stream size it runs on (c4: 1920x1080 only; c7: 1920x1080 or 1280x720)
  osz=$(echo "$ex" | grep -oE " ov_size [0-9]+x[0-9]+ " | awk '{print $2}')
  if [ -f "$cdir/remap_1920x1080.bin" ] && [ -n "$osz" ] && [ ! -f "$cdir/remap_$osz.bin" ]; then
    echo "config $CFG has no remap for the $osz stream ($(cd "$cdir" && ls remap_*.bin | tr '\n' ' ')): change ov_size"; exit 2
  fi
  [ -f "$cdir/estimator_config.yaml" ] || echo "note: $cdir not found on the PC (fps / size not checked)"
  PF=$(mktemp)
  a shell am force-stop $PKG     # no stale instance of our app (the run starts it fresh)
  phone_preflight "$ADB" "$S" 36.0 | tee "$PF"; pfrc=${PIPESTATUS[0]}
  if [ "$pfrc" != 0 ]; then
    mkdir -p "$OUT"; cp "$PF" "$OUT/preflight_refused_$(date +%Y%m%d_%H%M%S).txt"; exit 2
  fi
  before=$(sh_ "ls -d $OVD/run_* 2>/dev/null" | tail -1)
  [ -n "$LIVE" ] && live_up "$CFG"
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
  [ -n "$LIVE" ] && live_save "$OUT/$name"
  py=$SCAN3D/../venv_ffs/bin/python
  [ -x "$py" ] || py=python3
  "$py" "$SCAN3D/tools/ov_phone_timing.py" "$OUT/$name" ${SEG:+--seg "$SEG"}
  ;;
*) echo "push|run|clean"; exit 2 ;;
esac
