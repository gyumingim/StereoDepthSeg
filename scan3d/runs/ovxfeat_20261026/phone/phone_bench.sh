#!/bin/bash
# ovxfeat phone benchmark (Git Bash on Windows). Takes the phone lock, checks calcap/foreground/thermal, pushes the ONNX
# models + ort_bench to /data/local/tmp/ovxfeat, runs short benchmarks (no camera), writes raw output to OUTF.
# usage: phone_bench.sh LIST_FILE OUTF        (LIST_FILE lines: model backend iters warmup threads)
export MSYS_NO_PATHCONV=1
ADB="/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
T=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team
LOCK=$T/phone_lock.d
LIST=$1
OUTF=$2
W='\\wsl$\Ubuntu-22.04\home\system\scanner\tmp_ovxfeat'
log() { echo "$(date '+%F %T') $*" | tee -a "$OUTF"; }
temps() { $ADB shell "dumpsys thermalservice | grep -A8 'Current temperatures from HAL' | grep -E 'mName=(AP|SKIN)'" | sed -E 's/.*mValue=([0-9.]+).*mName=([A-Z]+).*/\2=\1/' | tr '\n' ' '; }
st=$(curl -s -m 5 http://100.92.214.97:8789/api/state | grep -o '"running": [a-z]*')
if [ "$st" != '"running": false' ]; then log "calcap state '$st' -> skip"; exit 3; fi
if ! mkdir $LOCK 2>/dev/null; then log "lock busy: $(cat $LOCK/owner.txt 2>/dev/null)"; exit 4; fi
echo "ovxfeat $(date '+%F %T') ONNX benchmark (no camera)" > $LOCK/owner.txt
release() { if grep -q '^ovxfeat' $LOCK/owner.txt 2>/dev/null; then rm -f $LOCK/owner.txt; rmdir $LOCK; log "lock released"; fi; }
trap release EXIT
top=$($ADB shell "dumpsys activity activities | grep topResumedActivity" | head -1)
wake=$($ADB shell "dumpsys power | grep mWakefulness=" | head -1)
bat=$($ADB shell "dumpsys battery | grep -E 'AC powered|USB powered|level'" | tr -d '\r' | tr '\n' ' ')
log "top: $top | $wake | battery: $bat"
case "$top" in *launcher*|*Launcher*|"") ;; *) if echo "$wake" | grep -q Awake; then log "user app in foreground and awake -> skip"; exit 5; fi;; esac
$ADB shell "mkdir -p /data/local/tmp/ovxfeat"
for f in ort/ort_bench ort/jni/arm64-v8a/libonnxruntime.so; do $ADB push "$W\\${f//\//\\}" /data/local/tmp/ovxfeat/ > /dev/null || { log "push $f failed"; exit 6; }; done
for m in $(awk '{print $1}' $LIST | sort -u); do
  $ADB push "$W\\onnx\\$m" /data/local/tmp/ovxfeat/ > /dev/null || { log "push $m failed"; exit 6; }
  [ -n "$(ls //wsl\$/Ubuntu-22.04/home/system/scanner/tmp_ovxfeat/onnx/$m.data 2>/dev/null)" ] && $ADB push "$W\\onnx\\$m.data" /data/local/tmp/ovxfeat/ > /dev/null
done
$ADB shell "chmod 755 /data/local/tmp/ovxfeat/ort_bench"
while read -r m be it wu th; do
  [ -z "$m" ] && continue
  tt=$(temps)
  skin=$(echo "$tt" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$tt" | grep -o 'AP=[0-9.]*' | cut -d= -f2)
  if awk "BEGIN{exit !($skin >= 36 || $ap >= 45)}"; then log "hot ($tt) -> wait 60 s"; sleep 60; tt=$(temps); skin=$(echo "$tt" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$tt" | grep -o 'AP=[0-9.]*' | cut -d= -f2); fi
  if awk "BEGIN{exit !($skin >= 36 || $ap >= 45)}"; then log "still hot ($tt) -> stop"; break; fi
  r=$($ADB shell "cd /data/local/tmp/ovxfeat && LD_LIBRARY_PATH=. timeout 120 ./ort_bench $m $be $it $wu $th 2>&1 | tail -2" | tr -d '\r' | tr '\n' ' ')
  log "before[$tt] after[$(temps)] $r"
done < $LIST
$ADB shell "pgrep -f ort_bench" && log "WARNING ort_bench still running"
log "camera: $($ADB shell "dumpsys media.camera | grep -A1 'Active Camera Clients'" | tr -d '\r' | tr '\n' ' ')"
