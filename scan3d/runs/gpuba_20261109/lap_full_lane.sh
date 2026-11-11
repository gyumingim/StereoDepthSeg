#!/bin/bash
# gpuba 11-11, on THIS PC: from-scratch laptop jobs. 165506: old x3 (CUDA SIFT, CPU BA), new x1 (CUDA SIFT, GPU BA), fix x3 (CPU SIFT, GPU BA)
# + sift_det base/cpu x3 on the laptop cache; 133820: old x1, new x1, fix x1 (timing).
WT=/home/system/scanner/StereoDepthSeg-gpuba; RL=/home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh
FL=/home/system/scanner/StereoDepthSeg-featloop/scan3d/runs
V=$FL/scanA_20261105/step1/B_o0.poses.jsonl.gz
WT=$WT $RL job 165506 "cmd:bash scan3d/runs/gpuba_20261109/lap_full.sh \$R \$O imu_gt_spl $V \"old1:auto:cpu old2:auto:cpu old3:auto:cpu new1:auto:cuda fix1:cpu:cuda fix2:cpu:cuda fix3:cpu:cuda\" && \$P scan3d/runs/gpuba_20261109/sift_det.py \$CACHE/hs_20261024_165506_std60/kf10_s0.5 \$O/sift_det_laptop.json --variants base,cpu" \
  --tag gpuba_full --send $V 2>&1 | grep -E "^==|TIME|sim3|failed|DONE|rc|pulled|base|cpu"
V=$FL/featloop_20261105/vio/20261022_133820.poses.jsonl.gz
WT=$WT $RL job 20261022_133820 "cmd:bash scan3d/runs/gpuba_20261109/lap_full.sh \$R \$O imu_gt_spl_cal1022 $V \"old1:auto:cpu new1:auto:cuda fix1:cpu:cuda\" --kf-from-vio --match-ms 1" \
  --gt imu_gt_spl_cal1022 --tag gpuba_full --send $V 2>&1 | grep -E "^==|TIME|sim3|failed|DONE|rc|pulled"
echo "FULLLANE DONE $(date +%T)"
