#!/usr/bin/env bash
# Physical calibration only (no bias stage, no CNN), held out = given recordings, rolling-shutter GT -> runs/on_rs_<rec>/result.json
cd "$(dirname "$0")/.."
ALL=$(ls -d recordings/hs_20261019_* | grep -v _junk)
for v in "$@"; do
  tr=$(echo "$ALL" | grep -v "$v" | tr '\n' ' ')
  IMU_VRAM_GB=3 nohup ../venv_ffs/bin/python -u imu_learn/train_imu.py --train $tr --val recordings/$v --win 3 --epochs 40 --stride 1.5     --batch 128 --gt-smooth --gt-dir imu_gt_rs --eval-only --skip-bias --out runs/on_rs_$v > runs/calibonly_$v.log 2>&1 &
done
