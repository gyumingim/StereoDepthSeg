#!/usr/bin/env bash
# ffsviews 11-11 queue (desktop; laptop does the compute through remote_laptop.sh). Run under setsid; log -> queue.log
cd /home/system/scanner/StereoDepthSeg-defaults/scan3d
RL=tools/remote_laptop.sh
L=/home/system/scanner/StereoDepthSeg-defaults/scan3d/runs/laptop/20261022_133820
BASE=$L/fv_n3/scan
echo "$(date +%T) queue start"
$RL job 133820 scan --tag fv_n3 --keep; echo "$(date +%T) fv_n3 rc $?"
$RL job 133820 'cmd:$P scan3d/scanA/_ref_scan_offline_bcc2c75.py '$BASE'/red/hs_20261022_133820_kf --out $O/mesh --gt imu_gt_spl_cal1022 --vio '$BASE'/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense > $O/mesh.log 2>&1' --tag fv_ref --keep; echo "$(date +%T) fv_ref rc $?"
for v in 'n3b|' 'n2|--ffs-views 2' 'n4|--ffs-views 4' 'n5|--ffs-views 5' 'n3med|--ffs-views 3 --ffs-fuse median'          'a2|--ffs-views 2 --ffs-partners all' 'a3|--ffs-partners all' 'a4|--ffs-views 4 --ffs-partners all' 'a5|--ffs-views 5 --ffs-partners all'; do
  t=${v%%|*}; x=${v#*|}
  K=--keep; [ $t = a5 ] && K=
  $RL job 133820 scan --tag fv_$t $K --args "--reuse $BASE $x"; echo "$(date +%T) fv_$t rc $?"
done
echo "$(date +%T) queue done"
