#!/bin/bash
# refit without poses tilted >= 55 deg (list from calib_all1024b k12_notan perpose)
cd "$(dirname "$0")/../.."
../venv_ffs/bin/python runs/lens_calib_f050_20261023/calib.py --models k12_notan k1_prior k123_notan --tag lt55_1024b --exclude $(cat runs/lens_calib_f050_20261023/logs/ex_tilt55.txt) > runs/lens_calib_f050_20261023/logs/calib_lt55_1024b.log 2>&1
