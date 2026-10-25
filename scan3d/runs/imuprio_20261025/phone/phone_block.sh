#!/usr/bin/env bash
# imuprio phone block (Git Bash): wait for the team phone lock, push own files, run Q.txt camera-free replays, install
# the 0.6.19 APK, release the lock. Usage: bash phone_block.sh
SP=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad
L=$SP/team/phone_lock.d
P=$SP/imuprio/progress.txt
A=/c/Users/a3162/platform-tools/adb.exe
D="-s 100.79.156.20:5555"
export MSYS_NO_PATHCONV=1
WP=C:/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad
log() { echo "$(date +%H:%M:%S) $*" | tee -a $P; }
until mkdir $L 2>/dev/null; do sleep 30; done
STAMP=$(date '+%Y-%m-%d %H:%M:%S'); echo "imuprio $STAMP camera-free replay R1/R3 (final binary) + install final 0.6.19" > $L/owner.txt
echo "$(date +%H:%M) imuprio: took phone lock" >> $SP/team/imuprio.progress.txt
release() { grep -q "^imuprio $STAMP" $L/owner.txt 2>/dev/null || { log "lock not mine any more - not removed"; return; }; rm -f $L/owner.txt; rmdir $L; log "lock released"; echo "$(date +%H:%M) imuprio: released phone lock" >> $SP/team/imuprio.progress.txt; }
trap release EXIT
log "lock taken"
T0=$(date +%s)
st() { $A $D shell "dumpsys thermalservice | grep -A40 'Current temperatures from HAL' | grep -E 'mName=SKIN|mName=AP,' | head -2 | sed 's/.*mValue=\([0-9.]*\).*mName=\([A-Z]*\).*/\2 \1/' | xargs; dumpsys power | grep -m1 mWakefulness=; dumpsys window | grep -m1 mCurrentFocus"; }
log "state: $(st | xargs)"
$A $D shell "mv /data/local/tmp/imuprio/out /data/local/tmp/imuprio/out_q 2>/dev/null; mkdir -p /data/local/tmp/imuprio/cfg && cp -r /data/local/tmp/ovopt/cfg/c4 /data/local/tmp/imuprio/cfg/ && cp /data/local/tmp/ovopt/feeder3_ovopt /data/local/tmp/imuprio/feeder3_imuprio"
$A $D push $WP/imuprio/phone/ov_bridge_imuprio $WP/imuprio/phone/phone_run.sh $WP/imuprio/phone/Q.txt /data/local/tmp/imuprio/ | tail -1
$A $D shell "chmod 755 /data/local/tmp/imuprio/ov_bridge_imuprio /data/local/tmp/imuprio/feeder3_imuprio; sha256sum /data/local/tmp/imuprio/ov_bridge_imuprio"
$A $D shell "cd /data/local/tmp/imuprio && MIN_COOL=30 nohup sh phone_run.sh Q.txt > out_nohup.txt 2>&1 < /dev/null &"
log "replay started"
while true; do
  sleep 20
  s=$($A $D shell "grep -h 'ALL DONE' /data/local/tmp/imuprio/out/status.txt; tail -1 /data/local/tmp/imuprio/out/status.txt" | tr '
' ' ')
  echo "$(date +%T) $s" >> $SP/imuprio/phone_status.log
  case "$s" in *"ALL DONE"*) break;; esac
  if [ $(( $(date +%s) - T0 )) -gt 1080 ]; then log "18 min reached -> stopping my replay"; $A $D shell "pkill feeder3_imuprio; pkill ov_bridge_imuprio"; break; fi
done
$A $D shell "cat /data/local/tmp/imuprio/out/status.txt" >> $SP/imuprio/phone_status.log
log "state after: $(st | xargs)"
$A $D shell "pidof feeder3_imuprio ov_bridge_imuprio || echo none-left; dumpsys media.camera | grep -A2 'Active Camera Clients'" >> $SP/imuprio/phone_status.log
mkdir -p $SP/imuprio/phone/out
$A $D pull /data/local/tmp/imuprio/out $WP/imuprio/phone/out_r | tail -1
log "install 0.6.19"
$A $D install -r $WP/arlat_0.6.19.apk 2>&1 | tail -2 | tee -a $P
$A $D shell "dumpsys package com.scan3d.arlat | grep -m2 -E 'versionCode|versionName'" | tee -a $P
