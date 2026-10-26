#!/bin/bash
# dpvofast: split profile, then knob jobs one at a time (GPU alone). Log -> logs/sched.log
cd ~/scanner/StereoDepthSeg-dpvofast/scan3d/runs/dpvofast_20261026
PY=~/scanner/dpvo_ext/mamba/envs/dpvo/bin/python
mkdir -p logs
L=logs/sched.log
echo "$(date '+%F %T') start load $(cat /proc/loadavg) gpu $(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader)" >> $L
if [ "$1" != "nosplit" ]; then
for cell in s165506_o0_f60 221052_o0_f60; do
 for sc in 0 0.6667; do
  tag=split_sc${sc}
  $PY dpvofast.py split $cell --tag $tag $( [ "$sc" != 0 ] && echo --scale $sc ) > logs/split_${cell}_${tag}.log 2>&1
  echo "$(date '+%F %T') split $cell $tag rc $? load $(cut -d' ' -f1 /proc/loadavg)" >> $L
 done
done
fi
CELLS="s165506_o0_f60 s165506_o1_f60 s165506_o2_f60 l142148_o0_f60 l142610_o0_f60"
while read tag args; do
 [ -z "$tag" ] && continue
 for cell in $CELLS; do
  [ -f ~/scanner/tmp_dpvofast/out/$tag/${cell}_$tag.json ] && continue
  $PY dpvofast.py job $cell --tag $tag $args > logs/job_${cell}_${tag}.log 2>&1
  echo "$(date '+%F %T') job $tag $cell rc $? $(tail -1 logs/job_${cell}_${tag}.log | cut -c1-260)" >> $L
 done
done < ${2:-knobs.txt}
echo "$(date '+%F %T') end" >> $L
