#!/bin/bash
# usage: run_joint.sh BOOT
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
B=$1
mkdir -p logs
W=4
for g in focus rec; do
  $P joint.py --groups $g --model k5 --boot $B --workers $W > logs/joint_${g}_k5.log 2>&1 &
  $P joint.py --groups $g --model rational --boot $B --workers $W > logs/joint_${g}_rational.log 2>&1 &
  $P joint.py --groups $g --model k5 --k1-per-group --boot $B --workers $W > logs/joint_${g}_k5_k1g.log 2>&1 &
done
wait
for f in logs/joint_*.log; do echo "== $f"; cat $f; done
