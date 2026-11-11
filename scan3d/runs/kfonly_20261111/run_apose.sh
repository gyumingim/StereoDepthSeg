#!/bin/bash
# kfonly 11-11 README 11: one laptop job (133820) for job_apose.sh; sends the reduced folders + A poses.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W/scan3d"
L=scan3d/runs/laptop/20261022_133820
tools/remote_laptop.sh job 20261022_133820 "cmd:bash scan3d/runs/kfonly_20261111/job_apose.sh \$O" --tag apose \
  --send $L/kfonly/red --send $L/kfonly1080/red1080 --send $L/kfonly/A/ba_S/poses_ba.jsonl.gz \
  > "$W/scan3d/runs/kfonly_20261111/q_apose.log" 2>&1
echo "rc $?" >> "$W/scan3d/runs/kfonly_20261111/q_apose.log"
