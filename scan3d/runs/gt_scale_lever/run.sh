#!/usr/bin/env bash
# 2026-10-22 recompute the GT scale check (s, s_lever, s_fixedlever) for every recording with GT. Old files: before/.
cd $(dirname $0)/../..
PY=../venv_ffs/bin/python
OD=runs/overnight_gt
M19=$(ls -d recordings/hs_20261019_* | while read r; do [ -f $r/imu_gt_rs/gt.npz ] && echo $r; done)
for gd in rs spl; do $PY imu_learn/scale_ls.py $M19 --gt-dir imu_gt_$gd --out $OD/scale_ref_monitor_imu_gt_$gd.json; done
for r in recordings/hs_2026102[0-2]_*; do N=$(basename $r)
  for gd in rs spl; do [ -f $r/imu_gt_$gd/gt.npz ] && $PY imu_learn/scale_ls.py $r --gt-dir imu_gt_$gd --out $OD/${N}_scale_$gd.json; done
done
