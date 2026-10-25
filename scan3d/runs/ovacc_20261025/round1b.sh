#!/bin/bash
# round 1b (16:56): PC load ~30 from other agents -> jobs 150 s; full 45-cell matrix would take 5+ h.
# Screening on the o0 cells only (15 recordings), finish DTp2 (running) fully; full 45 cells later for the winners.
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
O0=133820_o0,134550_o0,134204_o0,214550_o0,213341_o0,213955_o0,215348_o0,214138_o0,221052_o0,221925_o0,222747_o0,223815_o0,224323_o0,131326_o0,154320_o0
$PY acc_run.py run DTp2 --par 6
$PY acc_run.py run FXp,FXm,F10,F20,G85,SG15,SG07,NC05,NC2,M60,S70,CH2,MD15,DTm2,N300M80 --only $O0 --par 6
echo ROUND1_DONE
