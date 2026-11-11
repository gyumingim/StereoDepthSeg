#!/usr/bin/env bash
# ffsviews 11-11 queue 4: --ffs-partners all + dy check + dt cap 30 s (1964454 defaults) + default identity re-check
cd /home/system/scanner/StereoDepthSeg-defaults/scan3d
RL=tools/remote_laptop.sh
BASE=/home/system/scanner/StereoDepthSeg-defaults/scan3d/runs/laptop/20261022_133820/fv_n3/scan
echo "$(date +%T) queue4 start"
for v in 'n3e|' 'c2|--ffs-views 2 --ffs-partners all' 'c3|--ffs-partners all' 'c4|--ffs-views 4 --ffs-partners all' 'c5|--ffs-views 5 --ffs-partners all'; do
  t=${v%%|*}; x=${v#*|}
  K=--keep; [ $t = c5 ] && K=
  $RL job 133820 scan --tag fv_$t $K --args "--reuse $BASE $x"; echo "$(date +%T) fv_$t rc $?"
done
echo "$(date +%T) queue4 done"
