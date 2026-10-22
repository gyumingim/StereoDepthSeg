#!/bin/bash
# bootstrap (1 s blocks within each recording) for the k5 joint variants
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
B=${1:-60}
for g in focus rec; do
  $P joint.py --groups $g --model k5 --boot $B --workers 4 > logs/jointb_${g}_k5.log 2>&1 &
  $P joint.py --groups $g --model k5 --k1-per-group --boot $B --workers 4 > logs/jointb_${g}_k5_k1g.log 2>&1 &
done
wait
for f in logs/jointb_*.log; do echo "== $f"; cat $f; done
