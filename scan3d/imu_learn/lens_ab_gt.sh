#!/usr/bin/env bash
# Lens A/B GT sensitivity (docs/lens_ab_plan.md): rebuild the scoring GT of hs_20261022_134204 with lens B.
# Same steps as imu_learn/overnight_gt.sh from imu_gt_rs on (imu_gt_rs -> spline knot 0.0125 camera-only -> scale_ls),
# but into NEW folders (imu_gt_rs_lensB, imu_gt_spl_lensB); the existing imu_gt / imu_gt_rs / imu_gt_spl (lens A) are not touched.
# Kept from lens A: imu_gt/align.json (batch_sync dt, R_ci, gyro bias) used by the rolling-shutter step (mixed, noted in the plan).
set -eu
cd "$(dirname "$0")/.."
PY=${PY:-../venv_ffs/bin/python}
R=recordings/hs_20261022_134204
LB=config/lens/hs_1920x1080_f2.22_cal20261022.json
BOARD=config/charuco/board_monitor_6x6.json
OD=runs/lens_ab
mkdir -p $OD
export PYTHONIOENCODING=utf-8
echo "$(date +%T) imu_gt_rs (lens B) start"
$PY -u imu_learn/imu_gt.py $R --board $BOARD --rs --keep-corners --out-dir imu_gt_rs_lensB --lens $LB > $OD/gt_134204_imu_gt_rs_lensB.log 2>&1
echo "$(date +%T) spline start"
$PY -u imu_learn/spline_gt.py $R --gt-dir imu_gt_rs_lensB --out-dir imu_gt_spl_lensB --knot 0.0125 > $OD/gt_134204_spline_lensB.log 2>&1
echo "$(date +%T) scale_ls (A existing GT and B GT, same script)"
$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl --out $OD/gt_134204_scale_spl_lensA.json > $OD/gt_134204_scale_lensA.log 2>&1
$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl_lensB --out $OD/gt_134204_scale_spl_lensB.json > $OD/gt_134204_scale_lensB.log 2>&1
echo "$(date +%T) done"
