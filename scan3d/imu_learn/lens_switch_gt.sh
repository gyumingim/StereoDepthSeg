#!/usr/bin/env bash
# Lens switch GT rebuild (docs/lens_switch_20261023.md section 7): rebuild the scoring GT of one recording with the NEW
# 2.22 lens (config/lens/hs_1920x1080_f2.22_cal20261022.json), same steps as imu_learn/lens_ab_gt.sh / overnight_gt.sh from
# imu_gt_rs on: imu_gt --rs --keep-corners -> spline_gt knot 0.0125 camera-only -> scale_ls (old and new GT, same script).
# Output goes to NEW folders imu_gt_rs_cal1022 / imu_gt_spl_cal1022; imu_gt, imu_gt_rs, imu_gt_spl (old K) are not touched.
# Kept from the old K: imu_gt/align.json (batch_sync dt, R_ci, gyro bias) used by the rolling-shutter step (mixed).
# Board = the board recorded in the old imu_gt_rs/gt.npz; 10-19 recordings have none -> board_monitor_6x6.json
# (2026-10-23: the imu_gt default board_user_A4 found 0 poses there; their old GT = monitor board at square 63.403 mm,
# PnP distance ratio 1.00172 on 3000 frames; the current file has 63.294 mm -> new 10-19 GT is 0.17 % smaller from that).
#   bash imu_learn/lens_switch_gt.sh recordings/hs_...      (one recording; the driver runs several in parallel)
set -eu
cd "$(dirname "$0")/.."
PY=${PY:-../venv_ffs/bin/python}
R=${1%/}
N=$(basename $R)
LENS=config/lens/hs_1920x1080_f2.22_cal20261022.json
OD=runs/lens_switch/gt
mkdir -p $OD
export PYTHONIOENCODING=utf-8 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENCV_FOR_THREADS_NUM=1
# (2026-10-23: the first 12 parallel jobs ran without the OpenCV cap -> load ~98 on 16 cores, all pinned to cores 0-11)
BOARD=$($PY -c "import numpy as np,sys; z=np.load(sys.argv[1],allow_pickle=True); print(z['board_json'].item() if 'board_json' in z.files else 'config/charuco/board_monitor_6x6.json')" $R/imu_gt_rs/gt.npz)
echo "$(date +%T) $N start board $BOARD"
$PY -u imu_learn/imu_gt.py $R --board $BOARD --rs --keep-corners --out-dir imu_gt_rs_cal1022 --lens $LENS \
  ${CHUNKS:+--chunks $CHUNKS} > $OD/${N}_imu_gt_rs.log 2>&1      # CHUNKS=N: parallel board detection (same output)
echo "$(date +%T) $N rs done"
$PY -u imu_learn/spline_gt.py $R --gt-dir imu_gt_rs_cal1022 --out-dir imu_gt_spl_cal1022 --knot 0.0125 > $OD/${N}_spline.log 2>&1
echo "$(date +%T) $N spline done"
$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl --out $OD/${N}_scale_old.json > $OD/${N}_scale_old.log 2>&1 || echo "$N scale old FAILED"
$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl_cal1022 --out $OD/${N}_scale_new.json > $OD/${N}_scale_new.log 2>&1 || echo "$N scale new FAILED"
echo "$(date +%T) $N done"
