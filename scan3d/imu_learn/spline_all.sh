#!/usr/bin/env bash
# Spline GT for every recording with corners in imu_gt_rs: camera only -> imu_gt_spl, then with IMU -> imu_gt_spl_imu
# (knot 0.0125 s, chosen by held-out corners + gyro consistency: runs/spline_score_a.json, spline_score_b.json)
cd "$(dirname "$0")/.."
KNOT=${KNOT:-0.0125}
RECS=$(for d in recordings/hs_20261019_*; do [ -f "$d/imu_gt_rs/gt.npz" ] && echo "$d"; done)
for mode in cam imu; do
  for r in $RECS; do
    if [ $mode = cam ]; then
      ../venv_ffs/bin/python -u imu_learn/spline_gt.py $r --knot $KNOT 2>&1 | grep -v Warn | grep -E '^(hs_|→)'
    else
      ../venv_ffs/bin/python -u imu_learn/spline_gt.py $r --knot $KNOT --imu 2>&1 | grep -v Warn | grep -E '^(hs_|→)'
    fi
  done
  echo "$(date +%T) step 3: spline GT ($mode) done for all recordings" >> runs/spline_progress.log
done
