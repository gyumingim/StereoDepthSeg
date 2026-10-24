#!/bin/bash
cd "$(dirname "$0")/../.."
sed -i "s///" runs/lens_calib_f050_20261023/recs.txt
grep -E "_1[23][0-9]{4}_" runs/lens_calib_f050_20261023/recs.txt | xargs -P 8 -I{} sh -c "../venv_ffs/bin/python runs/lens_calib_f050_20261023/collect.py {} > runs/lens_calib_f050_20261023/logs/collect_{}.log 2>&1"
cat runs/lens_calib_f050_20261023/logs/collect_hs_20261024_1[23]*.log
