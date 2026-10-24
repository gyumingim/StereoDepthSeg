#!/bin/bash
cd "$(dirname "$0")"
../../../venv_ffs/bin/python glass_check.py
cd ../..
../venv_ffs/bin/python runs/lens_calib_f050_20261023/glass.py --tmax_mm 20 > runs/lens_calib_f050_20261023/logs/glass_tmax20.log 2>&1; tail -4 runs/lens_calib_f050_20261023/logs/glass_tmax20.log
