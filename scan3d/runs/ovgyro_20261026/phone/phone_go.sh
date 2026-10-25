#!/bin/bash
# usage: phone_go.sh LIST   (Git Bash) - waits until the user is not using the phone, takes the team phone lock, runs LIST
# on the phone (phone_run.sh), pulls results, releases the lock (owner-checked).
T=/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team
S=$T/ovgyro; L=$T/phone_lock.d; LIST=$1
WS=$(cygpath -m $S)
A="/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
export MSYS_NO_PATHCONV=1
log() { echo "$(date +%H:%M) ovgyro: $*" >> $T/ovgyro.progress.txt; echo "$*"; }
user_busy() { local w f; w=$($A shell 'dumpsys power | grep -m1 mWakefulness=' 2>/dev/null); f=$($A shell 'dumpsys window | grep -m1 mCurrentFocus' 2>/dev/null)
  echo "$w" | grep -q Awake || return 1; echo "$f" | grep -qi launcher && return 1; return 0; }
n=0
while true; do
  if user_busy; then n=$((n+1)); [ $((n % 20)) -eq 1 ] && log "waiting: user is using the phone"; sleep 30; continue; fi
  mkdir $L 2>/dev/null && break
  sleep 30
done
echo "ovgyro $(date '+%F %T') $LIST replay" > $L/owner.txt
log "took phone lock ($LIST)"
release() { if grep -q "^ovgyro " $L/owner.txt 2>/dev/null; then rm -f $L/owner.txt; rmdir $L; log "phone lock released"; fi; }
trap release EXIT
$A shell 'mkdir -p /data/local/tmp/ovgyro' || { log "adb failed"; exit 1; }
for f in ov_bridge_ovgyro phone_run.sh $LIST; do $A push "$WS/push/$f" /data/local/tmp/ovgyro/ > /dev/null || { log "push $f failed"; exit 1; }; done
$A shell 'cd /data/local/tmp/ovgyro && cp -n /data/local/tmp/ovopt/feeder3_ovopt ./feeder3_ovgyro; chmod 755 ov_bridge_ovgyro feeder3_ovgyro; sha256sum ov_bridge_ovgyro; dumpsys battery | grep -E "powered|temperature|level"'
$A shell "cd /data/local/tmp/ovgyro && MIN_COOL=45 nohup sh phone_run.sh $LIST > /dev/null 2>&1 &"
sleep 20
$A shell "cat /data/local/tmp/ovgyro/out/status.txt" | grep -q "list $LIST" || { log "runner did not start"; exit 1; }
for i in $(seq 1 120); do
  sleep 20
  st=$($A shell "cat /data/local/tmp/ovgyro/out/status.txt" 2>/dev/null)
  echo "$st" | grep -q "ALL DONE.*$LIST" && break
done
echo "$st" | grep -q "ALL DONE.*$LIST" || { log "runner not done after wait - NOT releasing early: waiting more"; until $A shell "cat /data/local/tmp/ovgyro/out/status.txt" | grep -q "ALL DONE.*$LIST"; do sleep 20; done; }
echo "$st" | tail -12
$A shell 'pidof feeder3_ovgyro ov_bridge_ovgyro; dumpsys media.camera | grep -A2 "Active Camera Clients"'
mkdir -p $S/pull; $A pull /data/local/tmp/ovgyro/out "$WS/pull" > /dev/null
log "phone list $LIST finished; pulled to scratchpad ovgyro/pull"
