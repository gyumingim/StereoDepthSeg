#!/bin/bash
# std60 supplementary set, after round 1 (keeps the 6-job cap)
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
while ! grep -q ROUND1_DONE /tmp/ovacc/round1b.log; do sleep 30; done
$PY acc_std.py run B0,L0,LC,LS,FXp,FXm,DTp2,DTm2 --par 6
echo STD_DONE
