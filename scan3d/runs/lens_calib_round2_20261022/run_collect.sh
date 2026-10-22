#!/bin/bash
# usage: run_collect.sh STEP [recs...]
cd ~/scanner/StereoDepthSeg/scan3d
P=../venv_ffs/bin/python
D=runs/lens_calib_round2_20261022
mkdir -p $D/views
STEP=$1; shift
RECS=${@:-162218 163644 171455 162754 172351 163008 173113 173834 163228 174512}
for r in $RECS; do
  $P $D/collect2.py recordings/hs_20261022_$r $D/views/$r.pkl --step $STEP > $D/views/$r.log 2>&1 &
done
wait
cat $D/views/*.log
