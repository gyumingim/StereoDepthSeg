#!/bin/bash
# npunight: 0.6.36 defaults check (D1a npu 1, D0 npu 0, D1b npu 1), 165506
H=$(cd "$(dirname "$0")" && pwd); cd $H
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python; R=~/scanner/StereoDepthSeg/scan3d/recordings
P=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/npunight.progress.txt
export VC=45
for x in "D1a 1" "D0 0" "D1b 1"; do set -- $x
  [ -e phone/$1 ] && continue
  bash rep_run.sh $1 165506 $2 > phone_$1.log 2>&1
  echo "$(date +%H:%M) $1: $($PY score_rep.py phone phone/$1 $R/hs_20261024_165506_std60 ~/scanner/tmp_npunight/pc/165506.json 2>&1 | tail -1)" | tee -a $P
done
