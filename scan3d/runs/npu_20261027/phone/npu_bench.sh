#!/bin/bash
# npu 2026-10-27 phone runner (Git Bash on Windows). LIST lines: "TAG<TAB>shell command run in /data/local/tmp/npu".
# Rules: phone lock dir (owner 'npu'), wait while the user uses the phone (foreground app not launcher + awake + last
# input < 5 min), start an item only at SKIN < 36 and AP < 45, stop if SKIN >= 45 or AP >= 50, battery < 20 % -> stop.
# Logs per item: SKIN/AP before+after, battery level, charging, power save, wakefulness. Restart-safe (OUT.done).
# usage: npu_bench.sh LIST OUT
export MSYS_NO_PATHCONV=1
ADB="/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:43487"
T=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team
LOCK=$T/phone_lock.d
LIST=$1; OUTF=$2; DONE=$OUTF.done; touch $DONE
log() { echo "$(date '+%F %T') $*" | tee -a "$OUTF"; }
temps() { $ADB shell "dumpsys thermalservice | grep -A8 'Current temperatures from HAL' | grep -E 'mName=(AP|SKIN)'" | sed -E 's/.*mValue=([0-9.]+).*mName=([A-Z]+).*/\2=\1/' | tr '\n' ' '; }
idle_ms() { $ADB shell "dumpsys power | grep -m1 ' lastUserActivityTime='" | sed -E 's/.*\(([0-9]+) ms ago\).*/\1/' | tr -dc '0-9'; }
user_busy() {
  [ -n "$NOUSER" ] && return 1  # coordinator 20:2x: user ordered "run now" -> no wait for user (logged as user-active condition)
  local top wake im
  top=$($ADB shell "dumpsys activity activities | grep topResumedActivity" | head -1)
  wake=$($ADB shell "dumpsys power | grep mWakefulness=" | head -1)
  case "$top" in *launcher*|*Launcher*|"") return 1;; esac
  echo "$wake" | grep -q Awake || return 1
  im=$(idle_ms); [ -n "$im" ] && [ "$im" -ge 300000 ] && return 1
  BUSY="top=${top##*u0 } last_input=${im}ms"; return 0
}
state() { echo "bat=$($ADB shell "dumpsys battery | grep -E '^  (AC powered|USB powered|level):'" | tr -d '\r ' | tr '\n' ',') lowpower=$($ADB shell settings get global low_power | tr -d '\r') $($ADB shell "dumpsys power | grep mWakefulness=" | tr -d '\r ')"; }
get_lock() {
  if mkdir $LOCK 2>/dev/null; then echo "npu $(date '+%F %T') NPU/GPU tflite bench (no camera)" > $LOCK/owner.txt; return 0; fi
  log "lock busy: $(cat $LOCK/owner.txt 2>/dev/null)"; return 1
}
release() { if grep -q '^npu' $LOCK/owner.txt 2>/dev/null; then rm -f $LOCK/owner.txt; rmdir $LOCK; log "lock released"; fi; }
trap release EXIT
HAVE=0
i=0
while IFS=$'\t' read -r tag cmd <&3; do
  i=$((i+1)); [ -z "$tag" ] && continue
  grep -qx "$i" $DONE && continue
  # wait for: user idle, lock, temps
  for k in $(seq 1 90); do
    if user_busy; then log "item $i: user active ($BUSY) -> wait 120 s"; [ $HAVE = 1 ] && { release; HAVE=0; }; sleep 120; continue; fi
    if [ $HAVE = 0 ]; then get_lock || { sleep 120; continue; }; HAVE=1; fi
    tt=$(temps); skin=$(echo "$tt" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$tt" | grep -o 'AP=[0-9.]*' | cut -d= -f2)
    [ -z "$skin" ] && { log "no temperature -> stop"; exit 7; }
    # supervisor 20:41: at SKIN >= 45 or AP >= 50 before an item -> stop the list (do not wait and continue)
    awk "BEGIN{exit !($skin >= 45 || $ap >= 50)}" && { log "STOP: too hot before item $i ($tt)"; exit 10; }
    if awk "BEGIN{exit !($skin >= ${GATE_SKIN:-36} || $ap >= ${GATE_AP:-45})}"; then log "item $i: hot ($tt) -> wait 60 s"; sleep 60; continue; fi
    break
  done
  awk "BEGIN{exit !($skin >= ${GATE_SKIN:-36} || $ap >= ${GATE_AP:-45})}" && { log "gave up waiting"; exit 9; }
  user_busy && { log "gave up waiting (user)"; exit 5; }
  lvl=$($ADB shell "dumpsys battery | grep -E '^  level:'" | tr -dc '0-9')
  [ -n "$lvl" ] && [ "$lvl" -lt 20 ] && { log "battery $lvl < 20 -> stop"; exit 8; }
  st="$(state) $( [ -n "$NOUSER" ] && echo "USER_ACTIVE_RULE_OFF top=$($ADB shell "dumpsys activity activities | grep topResumedActivity" | head -1 | sed "s/.*u0 //" | cut -c1-40) last_input=$(idle_ms)ms")"
  $ADB logcat -c
  r=$($ADB shell "cd /data/local/tmp/npu && LD_LIBRARY_PATH=. timeout 180 $cmd 2>&1" | tr -d '\r')
  [ -n "$FULLLOG" ] && { echo "== item $i [$tag] full logcat (read-only)"; $ADB logcat -d -v threadtime 2>/dev/null | tr -d ""; } >> "$OUTF.full"
  lc=$($ADB logcat -d -v brief 2>/dev/null | grep -iE 'tflite|nnapi|enn|neuralnetworks|gpu delegate|ExynosNN|eden' | grep -viE 'kenny|enable' | tail -40 | tr -d '\r')
  { echo "== item $i [$tag] $(date +%T)"; echo "$r"; echo "-- logcat"; echo "$lc"; } >> "$OUTF.raw"
  ta=$(temps)
  log "item $i [$tag] before[$tt] after[$ta] $st"
  echo "$r" | grep -E 'Inference timings|count=|delegate|Delegate|nodes|NNAPI|nnapi|accelerator|ERROR|Error|rror|Fail|fail|GPU|Replacing|enn|ms_p50|\{' | tail -25 | sed 's/^/    | /' | tee -a "$OUTF"
  echo "$lc" | sed 's/^/    L /' >> "$OUTF"
  echo $i >> $DONE
  skin=$(echo "$ta" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$ta" | grep -o 'AP=[0-9.]*' | cut -d= -f2)
  awk "BEGIN{exit !($skin >= 45 || $ap >= 50)}" && { log "ABORT: too hot ($ta)"; exit 10; }
  sleep ${COOL:-15}
done 3< $LIST
$ADB shell "pgrep -fl 'benchmark_model|ort_bench|npu_run'" && log "WARNING bench process still running"
log "camera: $($ADB shell "dumpsys media.camera | grep -A1 'Active Camera Clients'" | tr -d '\r' | tr '\n' ' ')"
log "list done"
