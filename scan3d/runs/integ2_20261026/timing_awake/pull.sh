#!/bin/bash
# pull.sh BLOCK TAG... : pull phone run dirs into worktree timing_awake/out_BLOCK
B=$1; shift
HERE=~/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026/timing_awake
ADB="/mnt/c/Users/a3162/platform-tools/adb.exe -s 100.79.156.20:5555"
P=/data/local/tmp/timing0626
mkdir -p $HERE/out_$B; WD=$(wslpath -w $HERE/out_$B)
for t in "$@"; do
  rm -rf "$HERE/out_$B/$t"
  $ADB pull $P/out/$t "$WD" | tail -1
  [ -f $HERE/out_$B/$t/po.jsonl ] && gzip -f $HERE/out_$B/$t/po.jsonl
done
$ADB pull $P/out/status.txt "$WD" > /dev/null
ls $HERE/out_$B
