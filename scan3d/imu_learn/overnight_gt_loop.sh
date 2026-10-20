#!/usr/bin/env bash
# Wait for each recording to be uploaded and verified ("DONE <name> sha256 OK" in runs/cf_upload_progress.log), then run
# imu_learn/overnight_gt.sh on it, in order. Logs "waiting"/"STALL" lines to runs/overnight_gt.log (no upload progress
# for > STALL_MIN minutes → STALL line once; the loop keeps waiting). Run from scan3d/:
#   (setsid nohup bash imu_learn/overnight_gt_loop.sh hs_20261020_214138 ... > runs/overnight_gt_loop.log 2>&1 &)
set -u
cd "$(dirname "$0")/.."
UP=runs/cf_upload_progress.log
LOG=runs/overnight_gt.log
STALL_MIN=${STALL_MIN:-40}
for N in "$@"; do
  stalled=0
  while ! grep -q "DONE $N sha256 OK" $UP 2>/dev/null || [ ! -f recordings/$N/meta.json ]; do
    age=$(( ($(date +%s) - $(stat -c %Y $UP)) / 60 ))
    if [ $age -ge $STALL_MIN ] && [ $stalled = 0 ]; then
      echo "$(date +%T) STALL upload: no progress for $age min while waiting for $N" >> $LOG; stalled=1
    fi
    sleep 60
  done
  echo "$(date +%T) $N uploaded+verified → GT" >> $LOG
  bash imu_learn/overnight_gt.sh recordings/$N
done
echo "$(date +%T) ALL DONE overnight GT loop ($*)" >> $LOG
