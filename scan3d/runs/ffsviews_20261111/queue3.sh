#!/usr/bin/env bash
# ffsviews 11-11 queue 3: --ffs-partners all + --pair-check dy (auto) + default identity re-check
cd /home/system/scanner/StereoDepthSeg-defaults/scan3d
RL=tools/remote_laptop.sh
BASE=/home/system/scanner/StereoDepthSeg-defaults/scan3d/runs/laptop/20261022_133820/fv_n3/scan
echo "$(date +%T) queue3 start"
for v in 'n3d|' 'd2|--ffs-views 2 --ffs-partners all' 'd3|--ffs-partners all' 'd4|--ffs-views 4 --ffs-partners all' 'd5|--ffs-views 5 --ffs-partners all'; do
  t=${v%%|*}; x=${v#*|}
  K=--keep; [ $t = d5 ] && K=
  $RL job 133820 scan --tag fv_$t $K --args "--reuse $BASE $x"; echo "$(date +%T) fv_$t rc $?"
done
echo "$(date +%T) queue3 done"
