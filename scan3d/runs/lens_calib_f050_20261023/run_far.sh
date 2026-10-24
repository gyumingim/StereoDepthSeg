#!/bin/bash
# refit without tilted poses closer than 0.75 m (|tvec|, board units from json)
cd "$(dirname "$0")/../.."
../venv_ffs/bin/python runs/lens_calib_f050_20261023/calib.py --models k12_notan k1_prior --tag far_1024b --exclude $(cat runs/lens_calib_f050_20261023/logs/ex_near_tilted.txt) > runs/lens_calib_f050_20261023/logs/calib_far_1024b.log 2>&1
