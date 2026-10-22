#!/bin/bash
# sensitivity variants without bootstrap: max tilt 45 deg, w < 0.02 rad/s, alternative selection seeds via n
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
mkdir -p logs
UNITS="162218 163644 171455 162754 172351 163008 173834 163228 174512 2.22 1.00 3.33 0.37"
for u in $UNITS; do
  $P calib_r2.py $u --boot 0 --max-tilt 45 --tag _t45 > logs/sens_t45_$u.log 2>&1 &
  $P calib_r2.py $u --boot 0 --wmax 0.02 --tag _w02 > logs/sens_w02_$u.log 2>&1 &
  $P calib_r2.py $u --boot 0 --n 1000 --tag _all > logs/sens_all_$u.log 2>&1 &
  wait
done
for s in t45 w02 all; do for u in $UNITS; do echo "== $s $u"; grep -v '^$' logs/sens_${s}_$u.log; done; done
