#!/usr/bin/env bash
# 2026-10-24: poll the coordinator's list of std60 board recordings (one name per line) every 60 s for up to 60 min and run
# the full chain (gt.sh -> std60_board_run.py run/score) on each new one once it is complete (meta.json "complete": true).
#   bash runs/std60_board_20261024/poll.sh <list file> [already-done names...]
set -u
cd "$(dirname "$0")/../.."
LIST=$1; shift
DONE=" $* "
PY=../venv_ffs/bin/python
LOG=runs/std60_board_20261024/poll.log
for i in $(seq 1 60); do
  for N in $(tr -d '\r' < "$LIST"); do
    case "$DONE" in *" $N "*) continue;; esac
    R=recordings/$N
    [ -f $R/meta.json ] && [ -f $R/cam5.mp4 ] && [ -f $R/imu.csv ] && grep -q '"complete": true' $R/meta.json || continue
    sleep 20   # let the upload settle
    echo "$(date +%T) $N start" >> $LOG
    if bash runs/std60_board_20261024/gt.sh $R >> $LOG 2>&1; then
      STD60_REC=$N $PY imu_learn/std60_board_run.py run >> $LOG 2>&1
      STD60_REC=$N $PY imu_learn/std60_board_run.py score > runs/std60_board_20261024/$(echo $N | cut -d_ -f3)/score.out 2>&1
      echo "$(date +%T) $N vio done" >> $LOG
    fi
    DONE="$DONE$N "
  done
  echo "$(date +%T) poll $i done:$DONE" >> $LOG
  sleep 60
done
echo "$(date +%T) poll end" >> $LOG
