#!/bin/bash
# gpuba 11-11 15:05: sequential queue after the lock incident: BA repeatability test (165506 db) -> remaining H comparisons -> 133820 from scratch
WT=/home/system/scanner/StereoDepthSeg-gpuba; RL=/home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh
FL=/home/system/scanner/StereoDepthSeg-featloop/scan3d/runs; D=$WT/scan3d/runs/gpuba_20261109
V=$FL/scanA_20261105/step1/B_o0.poses.jsonl.gz; SRC=$FL/featloop_20261105/rec/165506/ba_S
[ -f $WT/scan3d/runs/laptop/20261024_165506_std60/badet/c2/ba.json ] || WT=$WT $RL job 165506 "cmd:bash scan3d/runs/gpuba_20261109/lap_badet.sh \$R \$O imu_gt_spl $V $SRC" \
  --tag badet --send $V --send $SRC 2>&1 | grep -E "^==|sim3|POSEDIFF|failed|rc"
bash $D/lap_lane.sh
V=$FL/featloop_20261105/vio/20261022_133820.poses.jsonl.gz
[ -f $WT/scan3d/runs/laptop/20261022_133820/gpuba_full/fix1/ba.json ] || WT=$WT $RL job 20261022_133820 "cmd:bash scan3d/runs/gpuba_20261109/lap_full.sh \$R \$O imu_gt_spl_cal1022 $V \"old1:auto:cpu new1:auto:cuda fix1:cpu:cuda\" --kf-from-vio --match-ms 1" \
  --gt imu_gt_spl_cal1022 --tag gpuba_full --send $V 2>&1 | grep -E "^==|TIME|sim3|failed|DONE|rc|pulled"
echo "LANE2 DONE $(date +%T)"
