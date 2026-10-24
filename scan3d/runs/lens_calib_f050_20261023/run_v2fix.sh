#!/bin/bash
cd "$(dirname "$0")/../.."
for r in hs_20261024_121536_std60 hs_20261024_121711_std60; do ../venv_ffs/bin/python runs/lens_calib_f050_20261023/collect.py $r; done
../venv_ffs/bin/python runs/lens_calib_f050_20261023/steady.py > runs/lens_calib_f050_20261023/logs/steady_far.log 2>&1
../venv_ffs/bin/python runs/lens_calib_f050_20261023/far_diag.py
