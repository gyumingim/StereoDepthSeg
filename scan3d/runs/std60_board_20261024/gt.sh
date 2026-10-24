#!/usr/bin/env bash
# 2026-10-24 GT for std60 laptop-board recordings, same chain as imu_learn/overnight_gt.sh, lens passed explicitly
# (std60 0.5 dpt default lens) and board_laptop_5x3 (square 60.40 mm from panel EDID since commit 7cafe2d; v1 used 60.3,
# its logs are the files directly in runs/std60_board_20261024/).
#   bash runs/std60_board_20261024/gt.sh recordings/hs_..._std60      -> logs in runs/std60_board_20261024/<HHMMSS>/
set -u
cd "$(dirname "$0")/../.."
PY=../venv_ffs/bin/python
R=${1%/}
N=$(basename $R)
B=config/charuco/board_laptop_5x3.json
L=config/lens/std_1920x1080_f0.50_cal20261024d.json
OD=runs/std60_board_20261024/$(echo $N | cut -d_ -f3)
mkdir -p $OD
export PYTHONIOENCODING=utf-8
RAW=--raw-imu
s() { echo "$(date +%T) $N $*"; }
s imu_gt;  $PY -u imu_learn/imu_gt.py $R --board $B --lens $L $RAW --chunks 8 > $OD/imu_gt.log 2>&1 || { s FAIL imu_gt; exit 1; }
s board_shift; $PY imu_learn/board_shift.py $R --max-gap-ms 500 --raw-jump > $OD/board_shift.log 2>&1
s batch_sync; $PY -u imu_learn/batch_sync.py $R --write --json-out $OD/batch_sync.json > $OD/batch_sync.log 2>&1 || { s FAIL batch_sync; exit 1; }
s imu_gt_rs; $PY -u imu_learn/imu_gt.py $R --board $B --lens $L --rs --keep-corners --out-dir imu_gt_rs --chunks 8 > $OD/imu_gt_rs.log 2>&1 || { s FAIL rs; exit 1; }
s spline; $PY -u imu_learn/spline_gt.py $R --knot 0.0125 > $OD/spline.log 2>&1 || s FAIL spline
s scale; $PY imu_learn/scale_ls.py $R --gt-dir imu_gt_rs --out $OD/scale_rs.json > $OD/scale_rs.log 2>&1
$PY imu_learn/scale_ls.py $R --gt-dir imu_gt_spl --out $OD/scale_spl.json > $OD/scale_spl.log 2>&1
$PY imu_learn/gt_status.py $R > $OD/status.log 2>&1
s gt done
