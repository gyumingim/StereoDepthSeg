#!/bin/bash
# round 1 of the ovacc offline GT matrix (par 6 = brief cap)
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
$PY acc_run.py run B0R --only 133820_o0,221925_o1,154320_o2 --par 3
$PY acc_run.py run L0,LC,DTp2,DTm2,FXp,FXm,F10,F20,G85,MD15,SG15,SG07,CH2,NC05,NC2,M60,S70,N300M80 --par 6
echo ROUND1_DONE
