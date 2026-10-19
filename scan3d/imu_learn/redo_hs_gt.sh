#!/usr/bin/env bash
# Redo ground truth of the 240 fps recordings with lens_calib (slow, spread frames; shared lens for fixed-focus ones):
# timestamps rematch first (hs_fix_ts), then imu_gt (lens), overlay, precise sync. Run from scan3d/ with Windows python.
set -u
export PYTHONIOENCODING=utf-8
B=config/charuco/board_monitor_6x6.json
RECS=("$@")
for R in "${RECS[@]}"; do
  echo "== $R $(date +%T)"
  python imu_learn/hs_fix_ts.py "$R"
  rm -f "$R/imu_gt/gt.orig.npz" "$R/imu_gt/gt.npz" "$R/imu_gt/lens.json"
  python -u imu_learn/imu_gt.py "$R" --board $B 2>&1 | grep -v "^\[" | tail -12
  python imu_learn/gt_overlay.py "$R" --board $B
done
echo "== sync $(date +%T)"
python imu_learn/check_sync.py "${RECS[@]}" --write 2>&1 | tail -8
echo "== done $(date +%T)"
