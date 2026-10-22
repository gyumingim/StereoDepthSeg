#!/bin/bash
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
for u in 162218 163644 171455 162754 172351 163008 173834 163228 174512 2.22 1.00 3.33 0.37; do
  $P calib_r2.py $u --boot 0 --corner-reject 4 --tag _cr > logs/sens_cr_$u.log 2>&1 &
done
wait
for u in 162218 163644 171455 162754 172351 163008 173834 163228 174512 2.22 1.00 3.33 0.37; do echo "== $u"; cat logs/sens_cr_$u.log; done
