#!/bin/bash
# 0.1 rad/s sensitivity fits (after run_collect_w10.sh): per-recording / pooled (no bootstrap) and joint k5 k1-per-group
# (rec and focus groups), all on views_w10 with w < 0.1; then the comparison table (sens_w10.py).
cd ~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_round2_20261022
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export LC_VIEWS=views_w10 LC_WMAX=0.1
UNITS="162218 163644 171455 162754 172351 163008 173834 163228 174512 2.22 1.00 3.33 0.37"
for u in $UNITS; do nice -n 5 $P calib_r2.py $u --boot 0 --wmax 0.1 --workers 2 --tag _w10 > logs/sens_w10_$u.log 2>&1; done
nice -n 5 $P joint.py --groups rec --model k5 --k1-per-group --boot 0 --tag _w10 > logs/joint_rec_k5_k1g_w10.log 2>&1 &
nice -n 5 $P joint.py --groups focus --model k5 --k1-per-group --boot 0 --tag _w10 > logs/joint_focus_k5_k1g_w10.log 2>&1 &
wait
unset LC_VIEWS LC_WMAX
$P sens_w10.py
