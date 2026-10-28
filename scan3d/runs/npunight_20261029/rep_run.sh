#!/usr/bin/env bash
# npunight 2026-10-29: one camera-free in-app replay run (arlat 0.6.35d vc 44, intent replay), README pre-registered rules.
#   bash rep_run.sh <tag> <rec> <npu 0|1> [extra am args ...]     e.g. bash rep_run.sh 165506_A1 165506 1
# Start gate: SKIN < 38 and AP < 40 (checked every 10 s, wait at most 15 min, else SKIPPED). Stop: SKIN >= 45 / AP >= 60 /
# battery < 15 % while not charging (own 5 s monitor -> force-stop). The app ends by itself 3 s after the stream end
# (meta stop_reason "replay end"); hard cap secs 150. No camera: the replay mode never opens it (checked after the run).
# Output: runs/npunight_20261029/phone/<tag>/ (run folder, temps_pc.txt, rep_meta.json) + score (score_rep.py).
set -u
TAG=${1:?tag}; REC=${2:?rec}; NPU=${3:?npu}; shift 3
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad tag"; exit 2; }
A=/mnt/c/Users/a3162/platform-tools/adb.exe; S=${SERIAL:-100.79.156.20:33841}
H=$(cd "$(dirname "$0")" && pwd)
D=$H/phone/$TAG
[ -e "$D" ] && { echo "$D exists"; exit 2; }
mkdir -p "$D"
LOCK=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/phone_lock.d
mkdir "$LOCK" 2>/dev/null || { echo "phone locked by: $(cat "$LOCK"/owner.txt 2>/dev/null)"; rmdir "$D"; exit 3; }
echo "npunight rep_run $TAG $(date)" > "$LOCK/owner.txt"
unlock() { [ -f "$LOCK/owner.txt" ] && mv "$LOCK/owner.txt" "$D/lock_owner_released.txt"; rmdir "$LOCK" 2>/dev/null; }
trap unlock EXIT
$A connect "$S" > /dev/null 2>&1
vc=$($A -s "$S" shell "dumpsys package com.scan3d.arlat | grep -m1 versionCode" | tr -d '\r')
export VC=${VC:-44}; echo "$vc" | grep -q "versionCode=$VC" || { echo "phone app is not vc $VC: $vc"; exit 2; }
temps() {
  local o; o=$($A -s "$S" shell "dumpsys thermalservice | sed -n '/Current temperatures from HAL/,/^[^ \t]/p'; dumpsys battery | grep -E '^  level|AC powered|USB powered'" 2>/dev/null | tr -d '\r')
  sk=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=3, mName=SKIN" | grep -oE "[0-9.]+" | head -1)
  ap=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=0, mName=AP" | grep -oE "[0-9.]+" | head -1)
  bl=$(echo "$o" | grep -oE "level: [0-9]+" | grep -oE "[0-9]+")
  ch=$(echo "$o" | grep -cE "powered: true")
}
w0=$(date +%s)
while :; do
  temps
  awk -v s="${sk:-99}" -v a="${ap:-99}" 'BEGIN{exit !(s < 38 && a < 40)}' && break
  echo "$(date +%T) wait SKIN $sk AP $ap batt $bl charging $ch" >> "$D/wait.txt"
  if [ $(( $(date +%s) - w0 )) -gt 900 ]; then echo "SKIPPED: not cool after 15 min (SKIN $sk AP $ap)" | tee "$D/SKIPPED.txt"; exit 4; fi
  sleep 10
done
awk -v b="${bl:-0}" -v c="${ch:-0}" 'BEGIN{exit !(c > 0 || b >= 15)}' || { echo "REFUSED: battery $bl not charging" | tee "$D/REFUSED.txt"; exit 4; }
echo "start: SKIN $sk AP $ap battery $bl charging $ch waited $(( $(date +%s) - w0 )) s" | tee "$D/start.txt"
FLAG=/tmp/npunight_mon_${TAG}_$$
( while [ ! -f "$FLAG" ]; do
    temps; echo "$(date +%T) SKIN $sk AP $ap batt $bl ch $ch" >> "$D/temps_pc.txt"
    if awk -v s="${sk:-0}" -v a="${ap:-0}" -v b="${bl:-100}" -v c="${ch:-1}" 'BEGIN{exit !(s>=45||a>=60||(b<15&&c==0))}'; then
      echo "$(date +%T) SAFETY STOP SKIN $sk AP $ap batt $bl" | tee -a "$D/temps_pc.txt" "$D/SAFETY_STOP.txt"; $A -s "$S" shell am force-stop com.scan3d.arlat; fi
    sleep 5; done ) &
MP=$!
EX=(--es profile rec60 --ei loop 1 --es replay "replay/$REC.bin"); [ "$NPU" = 1 ] && EX+=(--ei npu_loop 1)
EX+=("$@")
T0=$(date "+%F %T")
$A -s "$S" shell am force-stop com.scan3d.arlat
before=$($A -s "$S" shell "ls /sdcard/Android/data/com.scan3d.arlat/files/ov | grep ^run_ | tail -1" | tr -d '\r')
$A -s "$S" shell input keyevent KEYCODE_WAKEUP; sleep 3   # screen off -> the activity is paused at once (165506_A1/B1 10-28 21:58)
$A -s "$S" shell am start -n com.scan3d.arlat/.MainActivity --es mode ov --es ov_cam main --es ov_cfg c4f60 --ei secs 150 "${EX[@]}" > "$D/am_start.txt" 2>&1
sleep 3
for i in $(seq 1 60); do   # up to 5 min: until the new run folder has meta.json (written last) or the app is gone
  pid=$($A -s "$S" shell pidof com.scan3d.arlat | tr -d '\r')
  [ -z "$pid" ] && break
  nr=$($A -s "$S" shell "ls /sdcard/Android/data/com.scan3d.arlat/files/ov | grep ^run_ | tail -1" | tr -d '\r')
  [ -n "$nr" ] && [ "$nr" != "$before" ] && $A -s "$S" shell "ls /sdcard/Android/data/com.scan3d.arlat/files/ov/$nr/meta.json" > /dev/null 2>&1 && { sleep 2; break; }
  sleep 5
done
touch "$FLAG"; wait $MP
run=$($A -s "$S" shell "ls /sdcard/Android/data/com.scan3d.arlat/files/ov | grep ^run_ | tail -1" | tr -d '\r')
cam=$($A -s "$S" shell "dumpsys media.camera | grep -A1 'Active Camera Clients'" | tr -d '\r' | tail -1)
br=$($A -s "$S" shell "pidof libov_bridge.so" | tr -d '\r')
echo "after: app pid '${pid:-none}' camera clients '$cam' bridge pid '${br:-none}' run '$run' (before '$before')" | tee "$D/after_check.txt"
[ -n "$pid" ] && $A -s "$S" shell am force-stop com.scan3d.arlat
if [ -n "$run" ] && [ "$run" != "$before" ]; then
  $A -s "$S" pull "/sdcard/Android/data/com.scan3d.arlat/files/ov/$run" "$(wslpath -w "$D")" > /dev/null
  echo "pulled $run"
else echo "NO NEW RUN FOLDER"; fi
python3 - "$D/rep_meta.json" "$TAG" "$REC" "$NPU" "$T0" "$run" "$cam" "${br:-}" "$*" <<'PY'
import json, sys, time
o, tag, rec, npu, t0, run, cam, br, extra = sys.argv[1:10]
json.dump(dict(made=time.strftime("%F %T"), tag=tag, rec=rec, npu_loop=int(npu), started=t0, run=run or None, app="vc %s" % __import__("os").environ.get("VC", "44"),
               extras="--es mode ov --es ov_cam main --es ov_cfg c4f60 --ei secs 150 --es profile rec60 --ei loop 1 --es replay replay/%s.bin%s %s"
               % (rec, " --ei npu_loop 1" if npu == "1" else "", extra), camera_after=cam, bridge_after=br or None), open(o, "w"), indent=1)
PY
echo "done $TAG"
