#!/usr/bin/env bash
# Physical calibration only (no bias stage, no CNN), held out = given recordings -> runs/on_<TAG>_<rec>/result.json
#   GT=imu_gt_rs SMOOTH=--gt-smooth TAG=rs  bash imu_learn/calib_folds.sh hs_20261019_131326 ...   (default, as before)
#   GT=imu_gt_spl SMOOTH= TAG=spl           bash imu_learn/calib_folds.sh ...                        (spline GT, already smooth)
#   EXTRA="--seed 1" TAG=rs_s1 ...  (repeat with another training seed)
# Recordings = only folders with imu_gt_rs/gt.npz AND $GT/gt.npz (hs_20261019_163324 has no GT: crashed the 10-20 folds).
cd "$(dirname "$0")/.."
GT=${GT:-imu_gt_rs}
SMOOTH=${SMOOTH---gt-smooth}
TAG=${TAG:-rs}
ALL=$(for d in recordings/hs_20261019_*; do [ -f "$d/imu_gt_rs/gt.npz" ] && [ -f "$d/$GT/gt.npz" ] && echo "$d"; done)
for v in "$@"; do
  tr=$(echo "$ALL" | grep -v "$v" | tr '\n' ' ')
  (IMU_CACHE_DIR=runs/train_cache IMU_VRAM_GB=${IMU_VRAM_GB:-3} setsid nohup ../venv_ffs/bin/python -u imu_learn/train_imu.py --train $tr \
    --val recordings/$v --win 3 --epochs 40 --stride 1.5 --batch 128 $SMOOTH --gt-dir $GT --eval-only --skip-bias $EXTRA \
    --out runs/on_${TAG}_$v > runs/calibonly_${TAG}_$v.log 2>&1 &)
done
