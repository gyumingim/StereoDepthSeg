#!/usr/bin/env bash
# appnpu error test (user approved 10-28): hand-held "return to start" run with arlat 0.6.32, live viewer on.
#   bash run_err.sh <tag> <npu 0|1> <secs>        e.g. bash run_err.sh r1_npu1 1 90
# = ov_phone_run.sh run <serial> c4f60 <secs> --es profile rec60 --ei loop 1 [--ei npu_loop 1], LIVE=1 (page + adb reverse,
#   lines -> <run>/ov_live.jsonl). Safety: start only SKIN < 40 (ov_phone_run preflight is stricter: SKIN < 36 + thermal 0),
#   stop at SKIN >= 45 / AP >= 50 / battery < 20 % (own 5 s monitor -> force-stop) and thermal status >= 2 (ov_phone_run).
#   The app stops itself after secs (meta.json). Afterwards: camera clients must be [] and no ov_bridge; pull; score.
# Output: runs/appnpu_20261028/err/<tag>/ (run folder, temps_pc.txt, err_meta.json, score.json) + one Korean summary line.
set -u
TAG=${1:?tag}; NPU=${2:?npu 0|1}; SECS=${3:?secs}
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "bad tag"; exit 2; }
[[ "$NPU" == 0 || "$NPU" == 1 ]] || { echo "npu must be 0 or 1"; exit 2; }
A=${ADB:-/mnt/c/Users/a3162/platform-tools/adb.exe}; S=${SERIAL:-100.79.156.20:33841}
H=$(cd "$(dirname "$0")" && pwd); W=$(cd "$H/../.." && pwd)            # W = scan3d of the appnpu worktree
D=$H/err/$TAG
[ -e "$D" ] && { echo "$D exists: pick a new tag"; exit 2; }
mkdir -p "$D"
LOCK=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/phone_lock.d
mkdir "$LOCK" 2>/dev/null || { echo "phone locked by: $(cat "$LOCK"/owner.txt 2>/dev/null)"; exit 3; }
echo "appnpu run_err $TAG $(date)" > "$LOCK/owner.txt"
unlock() { [ -f "$LOCK/owner.txt" ] && mv "$LOCK/owner.txt" "$D/lock_owner_released.txt"; rmdir "$LOCK" 2>/dev/null; }
trap unlock EXIT
$A connect "$S" > /dev/null 2>&1
vc=$($A -s "$S" shell "dumpsys package com.scan3d.arlat | grep -m1 versionCode" | tr -d '\r')
echo "$vc" | grep -q "versionCode=39" || { echo "phone app is not 0.6.33 (vc 39): $vc"; exit 2; }
temps() {
  local o; o=$($A -s "$S" shell "dumpsys thermalservice | sed -n '/Current temperatures from HAL/,/^[^ \t]/p'; dumpsys battery | grep -E '^  level'" 2>/dev/null | tr -d '\r')
  sk=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=3, mName=SKIN" | grep -oE "[0-9.]+" | head -1)
  ap=$(echo "$o" | grep -oE "mValue=[0-9.]+, mType=0, mName=AP" | grep -oE "[0-9.]+" | head -1)
  bl=$(echo "$o" | grep -oE "level: [0-9]+" | grep -oE "[0-9]+")
}
temps
echo "start check: SKIN $sk AP $ap battery $bl"
awk -v s="${sk:-99}" -v a="${ap:-99}" -v b="${bl:-0}" 'BEGIN{exit !(s < 40 && a < 50 && b >= 20)}' || { echo "REFUSED: SKIN $sk (need < 40) / AP $ap (need < 50, else the monitor stops at once) / battery $bl (need >= 20)"; exit 4; }
FLAG=/tmp/appnpu_err_mon_${TAG}_$$
( while [ ! -f "$FLAG" ]; do
    temps; echo "$(date +%T) SKIN $sk AP $ap batt $bl" >> "$D/temps_pc.txt"
    if awk -v s="${sk:-0}" -v a="${ap:-0}" -v b="${bl:-100}" 'BEGIN{exit !(s>=45||a>=50||b<20)}'; then
      echo "$(date +%T) SAFETY STOP SKIN $sk AP $ap batt $bl" | tee -a "$D/temps_pc.txt"; $A -s "$S" shell am force-stop com.scan3d.arlat; fi
    sleep 5; done ) &
MP=$!
EX=(--es profile rec60 --ei loop 1); [ "$NPU" = 1 ] && EX+=(--ei npu_loop 1)
T0=$(date "+%F %T")
LIVE=1 MAXSKIN=45 OUT="$D" ADB="$A" bash "$W/tools/ov_phone_run.sh" run "$S" c4f60 "$SECS" "${EX[@]}" 2>&1 | tee "$D/ov_phone_run.log"
touch "$FLAG"; wait $MP
cam=$($A -s "$S" shell "dumpsys media.camera | grep -A1 'Active Camera Clients'" | tr -d '\r' | tail -1)
br=$($A -s "$S" shell "pidof libov_bridge.so" | tr -d '\r')
echo "after: camera clients '$cam' bridge pid '${br:-none}'" | tee "$D/after_check.txt"
if [ "$cam" != "[]" ] || [ -n "$br" ]; then echo "WARNING camera/bridge not closed -> force-stop"; $A -s "$S" shell am force-stop com.scan3d.arlat; fi
RUN=$(ls -d "$D"/run_* 2>/dev/null | tail -1)
python3 - "$D/err_meta.json" "$TAG" "$NPU" "$SECS" "$T0" "${RUN:-}" "$cam" "${br:-}" <<'PY'
import json, sys, time
o, tag, npu, secs, t0, run, cam, br = sys.argv[1:9]
json.dump(dict(made=time.strftime("%F %T"), tag=tag, npu_loop=int(npu), secs=int(secs), started=t0, run=run or None,
               app="0.6.33 vc 39", cfg="c4f60", extras="--es profile rec60 --ei loop 1" + (" --ei npu_loop 1" if npu == "1" else ""),
               camera_after=cam, bridge_after=br or None), open(o, "w"), indent=1)
PY
[ -n "$RUN" ] && python3 "$H/score_err.py" "$D" || echo "no run folder pulled"
