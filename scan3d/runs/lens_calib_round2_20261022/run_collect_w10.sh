#!/bin/bash
# 0.1 rad/s still-threshold sensitivity (judge round 2, item 17): detections with --wmax 0.1 --step 2 into views_w10/
# (2 at a time, nice 10; 173113 excluded as in the main analysis)
cd ~/scanner/StereoDepthSeg/scan3d
P=../venv_ffs/bin/python
D=runs/lens_calib_round2_20261022
mkdir -p $D/views_w10
RECS=${@:-162218 163644 171455 162754 172351 163008 173834 163228 174512}
one() { nice -n 10 $P $D/collect2.py recordings/hs_20261022_$1 $D/views_w10/$1.pkl --step 2 --wmax 0.1 > $D/views_w10/$1.log 2>&1; }
set -- $RECS
while [ $# -gt 0 ]; do
  one $1 &
  if [ $# -gt 1 ]; then one $2 & fi
  wait
  shift; [ $# -gt 0 ] && shift
done
cat $D/views_w10/*.log
