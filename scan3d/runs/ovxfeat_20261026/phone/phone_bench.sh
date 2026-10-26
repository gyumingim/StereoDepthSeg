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
bat=$($ADB shell "dumpsys battery | grep -E '^  (AC powered|USB powered|level):'" | tr -d '\r' | tr '\n' ' ')
log "top: $top | $wake | battery: $bat"
case "$top" in *launcher*|*Launcher*|"") ;; *) if echo "$wake" | grep -q Awake; then log "user app in foreground and awake -> skip"; exit 5; fi;; esac
$ADB shell "mkdir -p /data/local/tmp/ovxfeat"
for f in ort/ort_bench ort/jni/arm64-v8a/libonnxruntime.so; do $ADB push "$W\\${f//\//\\}" /data/local/tmp/ovxfeat/ > /dev/null || { log "push $f failed"; exit 6; }; done
for m in $(awk '$1 != "T" {print $1}' $LIST | sort -u); do
  $ADB push "$W\\onnx\\$m" /data/local/tmp/ovxfeat/ > /dev/null || { log "push $m failed"; exit 6; }
  [ -n "$(ls //wsl\$/Ubuntu-22.04/home/system/scanner/tmp_ovxfeat/onnx/$m.data 2>/dev/null)" ] && $ADB push "$W\\onnx\\$m.data" /data/local/tmp/ovxfeat/ > /dev/null
done
# TFLite lines: "T name_prec.tflite benchmark_model-args..." (file from tmp_ovxfeat/tflite/<name>/)
if grep -q '^T ' $LIST; then
  $ADB push 'C:\Users\a3162\AppData\Local\Temp\claude\c--Users-a3162-Desktop----\27548596-eef5-4a58-bd39-e92ba44d822c\scratchpad\ovxfeat_dl\bench\android_aarch64_benchmark_model' /data/local/tmp/ovxfeat/benchmark_model > /dev/null || { log "push benchmark_model failed"; exit 6; }
  for f in $(awk '$1 == "T" {print $2}' $LIST | sort -u); do
    d=$(echo $f | sed -E 's/_(float32|float16)\.tflite$//')
    $ADB push "$W\\tflite\\$d\\$f" /data/local/tmp/ovxfeat/ > /dev/null || { log "push $f failed"; exit 6; }
  done
  $ADB shell "chmod 755 /data/local/tmp/ovxfeat/benchmark_model"
fi
$ADB shell "chmod 755 /data/local/tmp/ovxfeat/ort_bench"
for f in in_640x352.f32 in_640x352_norm.f32 in_960x544.f32 in_960x544_norm.f32; do $ADB push "$W\\acc\\$f" /data/local/tmp/ovxfeat/ > /dev/null; done
log "power save: $($ADB shell settings get global low_power | tr -d '\r')"
DONE=$OUTF.done
touch $DONE
i=0
# per item (supervisor 21:03): skip items already done (restart-safe), user/battery check, 20 s cool-down
while read -r m be it wu th rest <&3; do
  i=$((i+1))
  [ -z "$m" ] && continue
  grep -qx "$i" $DONE && continue
  top=$($ADB shell "dumpsys activity activities | grep topResumedActivity" | head -1)
  wake=$($ADB shell "dumpsys power | grep mWakefulness=" | head -1)
  lvl=$($ADB shell "dumpsys battery | grep -E '^  level:'" | tr -dc '0-9')
  case "$top" in *launcher*|*Launcher*|"") ;; *) if echo "$wake" | grep -q Awake; then log "item $i: user app in foreground and awake -> pause"; exit 5; fi;; esac
  if [ -n "$lvl" ] && [ "$lvl" -lt 20 ]; then log "item $i: battery $lvl % < 20 -> stop"; exit 8; fi
  sleep 20
  tt=$(temps)
  skin=$(echo "$tt" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$tt" | grep -o 'AP=[0-9.]*' | cut -d= -f2)
  # gate (coordinator): start a run only at SKIN < 36 (AP < 45 here, stricter than the AP 50 abort); wait up to 10 min
  for k in 1 2 3 4 5 6 7 8 9 10; do
    if [ -z "$skin" ] || [ -z "$ap" ]; then log "no temperature reading -> stop"; exit 7; fi
    awk "BEGIN{exit !($skin >= 36 || $ap >= 45)}" || break
    log "hot ($tt) -> wait 60 s"; sleep 60
    tt=$(temps); skin=$(echo "$tt" | grep -o 'SKIN=[0-9.]*' | cut -d= -f2); ap=$(echo "$tt" | grep -o 'AP=[0-9.]*' | cut -d= -f2)
  done
  if awk "BEGIN{exit !($skin >= 36 || $ap >= 45)}"; then log "still hot ($tt) -> stop"; break; fi
  if [ "$m" = "T" ]; then
    r=$($ADB shell "cd /data/local/tmp/ovxfeat && timeout 120 ./benchmark_model --graph=$be $it $wu $th $rest 2>&1 | grep -E 'Inference timings|Delegate|delegate|nodes|ERROR|Error|Failed|count=' | tail -8" | tr -d '\r' | tr '\n' ' ')
    r="TFL $be $it $wu $th $rest :: $r"
  else
    r=$($ADB shell "cd /data/local/tmp/ovxfeat && LD_LIBRARY_PATH=. timeout 120 ./ort_bench $m $be $it $wu $th $rest 2>&1 | tail -2" | tr -d '\r' | tr '\n' ' ')
  fi
  log "item $i bat $lvl% $(echo $wake | tr -d ' ') before[$tt] after[$(temps)] $r"
  echo $i >> $DONE
done 3< $LIST
mkdir -p //wsl\$/Ubuntu-22.04/home/system/scanner/tmp_ovxfeat/acc/phone
for f in $($ADB shell "ls /data/local/tmp/ovxfeat/ | grep '^o_'" | tr -d '\r'); do $ADB pull /data/local/tmp/ovxfeat/$f "$W\\acc\\phone\\$f" > /dev/null; done
$ADB shell "pgrep -f 'ort_bench|benchmark_model'" && log "WARNING ort_bench still running"
log "camera: $($ADB shell "dumpsys media.camera | grep -A1 'Active Camera Clients'" | tr -d '\r' | tr '\n' ' ')"
