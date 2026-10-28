#!/bin/bash
# npunight: pre-registered order. ABBA per GT recording, then speed S1 S2 S3 S3 S2 S1 on 165506. Each run scored right after.
H=$(cd "$(dirname "$0")" && pwd); cd $H
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python; R=~/scanner/StereoDepthSeg/scan3d/recordings
declare -A RD=([165506]=$R/hs_20261024_165506_std60 [213341]=$R/hs_20261021_213341 [214550]=$R/hs_20261021_214550)
P=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/npunight.progress.txt
one() {  # tag rec npu extra...
  local t=$1 r=$2; shift 2
  [ -e phone/$t ] && return
  bash rep_run.sh $t $r "$@" > phone_$t.log 2>&1
  local s=$($PY score_rep.py phone phone/$t ${RD[$r]} ~/scanner/tmp_npunight/pc/$r.json 2>&1 | tail -1)
  echo "$(date +%H:%M) $t: $s" | tee -a $P
}
for r in ${RECS:-165506 213341 214550}; do
  [ -f ~/scanner/StereoDepthSeg-appnpu/scan3d/runs/npunight_20261029/STOP ] && exit 0
  one ${r}_A1 $r 1; one ${r}_B1 $r 0; one ${r}_B2 $r 0; one ${r}_A2 $r 1
done
O3="--es npu_opts nn_fast=1,lg_post=3"
one S1a 165506 1 $O3; one S2a 165506 1 $O3 --es npu_cpus 4-6; one S3a 165506 1 $O3 --es npu_cpus 7-8
one S3b 165506 1 $O3 --es npu_cpus 7-8; one S2b 165506 1 $O3 --es npu_cpus 4-6; one S1b 165506 1 $O3
O4="--es npu_opts nn_fast=1,lg_post=3,prep_fast=1"
one S4a 165506 1 $O4 --es npu_cpus 7-8; one S4b 165506 1 $O4 --es npu_cpus 7-8
echo "$(date +%H:%M) batch done" | tee -a $P
