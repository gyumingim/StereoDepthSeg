#!/bin/bash
# npunight S5: S4 opts + ENN perf mode 1 (after batch2)
H=$(cd "$(dirname "$0")" && pwd); cd $H
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python; R=~/scanner/StereoDepthSeg/scan3d/recordings
P=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/npunight.progress.txt
O4="--es npu_opts nn_fast=1,lg_post=3,prep_fast=1"
for t in S5a S5b; do
  [ -e phone/$t ] && continue
  bash rep_run.sh $t 165506 1 $O4 --es npu_cpus 7-8 --ei npu_perf 1 > phone_$t.log 2>&1
  echo "$(date +%H:%M) $t: $($PY score_rep.py phone phone/$t $R/hs_20261024_165506_std60 ~/scanner/tmp_npunight/pc/165506.json 2>&1 | tail -1)" | tee -a $P
done
