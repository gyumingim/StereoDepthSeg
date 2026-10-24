#!/bin/bash
# FAR two-marker set (2026-10-24 12:15-13:10): (A) far only, (B) far + old frontal ChArUco, (C) everything
cd "$(dirname "$0")/../.."
D=runs/lens_calib_f050_20261023
../venv_ffs/bin/python $D/steady.py > $D/logs/steady_far.log 2>&1
M="k1k2p_k3 rational k1_prior k123_notan k12_notan"
../venv_ffs/bin/python $D/calib.py --models $M --tag farA --exclude-file $D/logs/ex_A_nonfar.txt > $D/logs/calib_farA.log 2>&1 &
../venv_ffs/bin/python $D/calib.py --models $M --tag farB --exclude-file $D/logs/ex_B_oldtilted.txt > $D/logs/calib_farB.log 2>&1 &
../venv_ffs/bin/python $D/calib.py --models $M --tag farC > $D/logs/calib_farC.log 2>&1 &
wait
for f in farA farB farC; do echo "== $f"; grep -E "^views|^\[" $D/logs/calib_$f.log | cut -c1-300; done
