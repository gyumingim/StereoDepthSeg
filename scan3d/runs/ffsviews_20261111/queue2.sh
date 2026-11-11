#!/usr/bin/env bash
# ffsviews 11-11 queue 2: pair gate (--pair-min-valid 0.2 default with --ffs-partners all) + default identity re-check
cd /home/system/scanner/StereoDepthSeg-defaults/scan3d
RL=tools/remote_laptop.sh
BASE=/home/system/scanner/StereoDepthSeg-defaults/scan3d/runs/laptop/20261022_133820/fv_n3/scan
echo "$(date +%T) queue2 start"
for v in 'n3c|' 'g2|--ffs-views 2 --ffs-partners all' 'g3|--ffs-partners all' 'g4|--ffs-views 4 --ffs-partners all' 'g5|--ffs-views 5 --ffs-partners all'; do
  t=${v%%|*}; x=${v#*|}
  K=--keep; [ $t = g5 ] && K=
  $RL job 133820 scan --tag fv_$t $K --args "--reuse $BASE $x"; echo "$(date +%T) fv_$t rc $?"
done
echo "$(date +%T) queue2 done"
