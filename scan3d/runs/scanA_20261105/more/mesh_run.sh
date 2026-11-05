#!/bin/bash
# scanA more-recordings (11-05): per HS recording, keyframe BA (seeded by the 2.22 dpt VIO replay) -> depth + TSDF (5 mm,
# 960x540) with BA poses at the VIO scale (= real scanner output), then the same keyframes with GT poses (comparison).
# GT = imu_gt_spl_cal1022 (new 2.22 dpt lens K fx 1303.9; never imu_gt_spl = old K 1267). Sequential GPU jobs (gpu_guard lock).
W=$(cd "$(dirname "$0")/../../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/scanA_20261105/more; G=imu_gt_spl_cal1022
export FFS_ROOT=$HOME/scanner/StereoDepthSeg
for r in "$@"; do
  R=$W/scan3d/recordings/hs_$r
  echo "== $r BA $(date +%T)"
  $P scan3d/scanA/kf_ba.py $R --vio $D/vio/$r.poses.jsonl.gz --out $D/ba/$r --gt $G --kf-from-vio --match-ms 1 --iters ${BA_ITERS:-40} > $D/ba/$r.log 2>&1 \
    || { echo "   BA failed rc $?"; continue; }
  rm -f $D/ba/$r/db.db
  grep -E "^\[sim3|^\[kf\]" $D/ba/$r.log | head -3
  echo "== $r mesh BA $(date +%T)"
  $P scan3d/scanA/scan_offline.py $R --out $D/mesh/${r}_ba --gt $G --vio $D/ba/$r/poses_ba.jsonl.gz --compress --kf-step 1 \
    > $D/mesh/${r}_ba.log 2>&1 || echo "   mesh BA failed rc $?"
  echo "== $r mesh GT $(date +%T)"
  $P scan3d/scanA/scan_offline.py $R --out $D/mesh/${r}_gt --gt $G --rows-from $D/ba/$r/poses_ba.jsonl.gz --compress --kf-step 1 \
    > $D/mesh/${r}_gt.log 2>&1 || echo "   mesh GT failed rc $?"
done
echo ALLDONE $(date +%T)
