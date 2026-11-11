#!/bin/bash
# gpuba 11-11, runs on THIS PC: one laptop job per H recording (lap_rec.sh there), results -> WT/scan3d/runs/laptop/<rid>/gpuba
WT=/home/system/scanner/StereoDepthSeg-gpuba; RL=/home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh
FL=/home/system/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105
H=${H:-"20261022_134550 20261022_133820 20261022_134204 20261021_214550 20261021_213341 20261021_215348 20261020_221052 20261020_221925 20261020_222747 20261020_223815 20261020_224323"}
for id in $H; do
  [ -f $WT/scan3d/runs/laptop/$id/gpuba/mesh_cpu/summary.json ] && { echo "== $id done before"; continue; }
  SRC=$FL/rec/$id/ba_S; [ -f $SRC/db.db ] || SRC=$FL/rec/$id/ba_S_gpu
  V=$FL/vio/$id.poses.jsonl.gz
  echo "== $id $(date +%T) src $SRC"
  WT=$WT $RL job $id "cmd:bash scan3d/runs/gpuba_20261109/lap_rec.sh \$R \$O imu_gt_spl_cal1022 $V $SRC --kf-from-vio --match-ms 1" \
    --gt imu_gt_spl_cal1022 --tag gpuba --send $V --send $SRC 2>&1 | grep -E "^==|TIME|TRI_EQUAL|failed|DONE|rc|pulled" 
done
echo "LAPLANE DONE $(date +%T)"
