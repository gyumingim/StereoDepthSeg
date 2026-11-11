#!/bin/bash
# kfonly 11-11 follow-up B95: waits for the main queue (pids given in WAIT_PIDS) to end, then one laptop job per recording.
export PROG=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/kfonly.progress.txt
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W/scan3d"; D=scan3d/runs/kfonly_20261111
for p in $WAIT_PIDS; do while kill -0 $p 2>/dev/null; do sleep 60; done; done
for id in "$@"; do
  [ -f "$W/scan3d/runs/laptop/$id/kfonly95/B95/mesh_S/summary.json" ] && continue
  echo "$(date "+%m-%d %H:%M") kfonly: B95 job $id started" >> $PROG
  tools/remote_laptop.sh job "$id" "cmd:bash $D/job_b95.sh \$R \$O $W/$D/vio/$id.poses.jsonl.gz" \
      --send scan3d/runs/kfonly_20261111/vio/$id.poses.jsonl.gz --tag kfonly95 > "$W/$D/q95_$id.log" 2>&1
  echo "$(date "+%m-%d %H:%M") kfonly: B95 job $id rc $?" >> $PROG
done
echo "$(date "+%m-%d %H:%M") kfonly: B95 queue finished" >> $PROG
