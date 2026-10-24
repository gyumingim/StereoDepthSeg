#!/bin/bash
cd "$(dirname "$0")/../.."
../venv_ffs/bin/python runs/lens_calib_f050_20261023/glass.py --halves > runs/lens_calib_f050_20261023/logs/glass_halves.log 2>&1
