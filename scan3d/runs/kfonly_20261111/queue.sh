#!/bin/bash
# kfonly 11-11: run job_rec.sh for each recording on the laptop worker, one after another (remote_laptop.sh takes the laptop lock).
# usage: queue.sh ID...   (short ids, e.g. 20261022_134204). Progress lines -> $PROG (if set).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W/scan3d"
D=scan3d/runs/kfonly_20261111
for id in "$@"; do
  if [ -f "$W/scan3d/runs/laptop/$id/kfonly/B/mesh_S/summary.json" ]; then echo "$id done already"; continue; fi
  echo "$(date '+%m-%d %H:%M') kfonly: job $id queued/started" | tee -a ${PROG:-/dev/null}
  tools/remote_laptop.sh job "$id" "cmd:bash $D/job_rec.sh \$R \$O $W/$D/vio/$id.poses.jsonl.gz 90" \
      --send scan3d/runs/kfonly_20261111/vio/$id.poses.jsonl.gz --tag kfonly > "$W/$D/q_$id.log" 2>&1
  rc=$?
  echo "$(date '+%m-%d %H:%M') kfonly: job $id rc $rc ($(grep -c . "$W/$D/q_$id.log") log lines)" | tee -a ${PROG:-/dev/null}
done
echo "$(date '+%m-%d %H:%M') kfonly: queue finished ($*)" | tee -a ${PROG:-/dev/null}
