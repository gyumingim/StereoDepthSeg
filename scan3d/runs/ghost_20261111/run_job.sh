#!/bin/bash
# ghost 11-11: one laptop job from this PC (run under setsid). usage: run_job.sh NAME VARIANTS FUSE_TAG
cd /home/system/scanner/StereoDepthSeg-laptop/scan3d
PROG=/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/ghost.progress.txt
echo "$(date +%H:%M) $1 ($2 -> $3) queued" >> $PROG
WT=/home/system/scanner/StereoDepthSeg-ghost tools/remote_laptop.sh job 134550 \
   "cmd:bash scan3d/runs/ghost_20261111/job_ghost.sh \$R \$O \$P \$G \$WT $2 $3 $4" \
   --gt imu_gt_spl_cal1022 --tag ghost --send scan3d/runs/ghost_20261111/A_poses_ba_134550.jsonl.gz < /dev/null
rc=$?
echo "$(date +%H:%M) $1 rc $rc" >> $PROG
