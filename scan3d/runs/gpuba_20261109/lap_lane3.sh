#!/bin/bash
# gpuba 11-11 16:05: determinism fix candidates on 165506 from scratch, 3x each: CPU SIFT + sorted inliers; CPU SIFT + CPU matcher (+ sorted)
WT=/home/system/scanner/StereoDepthSeg-gpuba; RL=/home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh
FL=/home/system/scanner/StereoDepthSeg-featloop/scan3d/runs; V=$FL/scanA_20261105/step1/B_o0.poses.jsonl.gz
while pgrep -f lap_lane2.sh >/dev/null; do sleep 30; done
WT=$WT $RL job 165506 "cmd:bash scan3d/runs/gpuba_20261109/lap_full.sh \$R \$O imu_gt_spl $V \"fs1:cpu:cuda:s fs2:cpu:cuda:s fs3:cpu:cuda:s fm1:cpu:cuda:sm fm2:cpu:cuda:sm fm3:cpu:cuda:sm\"" \
  --tag gpuba_fix --send $V 2>&1 | grep --line-buffered -E "^==|TIME|sim3|failed|DONE|rc|pulled"
echo "LANE3 DONE $(date +%T)"
