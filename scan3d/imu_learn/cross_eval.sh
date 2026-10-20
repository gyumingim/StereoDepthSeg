#!/usr/bin/env bash
# 2x2: every calibration (trained on GT A) scored on the held-out recording with GT B — same held-out recording, two rulers.
#   SEEDS="0 1 2" bash imu_learn/cross_eval.sh "rs spl" hs_20261019_131326 ...   → runs/x_<calib>_on_<gt>_<rec>_s<seed>/result.json
# rulers: rs = imu_gt_rs + --gt-smooth (as in the calibration runs), spl = imu_gt_spl (spline, no extra smoothing),
#         rsraw = imu_gt_rs without smoothing. Seeds: the per-recording gravity fit is stochastic (±0.07 cm seen) → average.
cd "$(dirname "$0")/.."
TAGS=$1; shift
SEEDS=${SEEDS:-0 1 2}
gtargs(){ case $1 in rs) echo "--gt-dir imu_gt_rs --gt-smooth";; spl) echo "--gt-dir imu_gt_spl";; rsraw) echo "--gt-dir imu_gt_rs";; esac; }
for s in $SEEDS; do
for v in "$@"; do
  for c in $TAGS; do
    for g in rs spl rsraw; do
      [ -f runs/on_${c}_$v/result.json ] || { echo "missing runs/on_${c}_$v"; continue; }
      IMU_CACHE_DIR=runs/train_cache IMU_VRAM_GB=${IMU_VRAM_GB:-2} ../venv_ffs/bin/python -u imu_learn/train_imu.py \
        --train recordings/$v --val recordings/$v --win 3 --stride 1.5 --batch 128 $(gtargs $g) --seed $s \
        --calib-json runs/on_${c}_$v/result.json --out runs/x_${c}_on_${g}_${v}_s$s 2>&1 | grep -E '^(보정|물리)' | sed "s/^/$c on $g $v s$s: /"
    done
  done
done
done
