#!/bin/bash
# usage: run_cal.sh BOOT [extra args]  (runs all units; parallel when BOOT=0)
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
BOOT=$1; shift
mkdir -p logs
UNITS="162218 163644 171455 162754 172351 163008 173113 173834 163228 174512 2.22 1.00 3.33 0.37"
if [ "$BOOT" = "0" ]; then
  for u in $UNITS; do $P calib_r2.py $u --boot 0 "$@" > logs/cal_$u.log 2>&1 & done
  wait
else
  for u in $UNITS; do $P calib_r2.py $u --boot $BOOT "$@" > logs/cal_$u.log 2>&1; done
fi
for u in $UNITS; do echo "== $u"; cat logs/cal_$u.log; done
