#!/usr/bin/env bash
# Redo ground truth of 240 fps recordings with lens_calib (slow, spread frames; shared lens per fixed focus):
# timestamps rematch first (hs_fix_ts), then imu_gt (lens), overlay, traj_compare. Run from scan3d/.
# PY defaults to the WSL venv (native file access; Windows python over \wsl.localhost was 3-4x slower).
set -u
PY=${PY:-../venv_ffs/bin/python}
export PYTHONIOENCODING=utf-8
B=config/charuco/board_monitor_6x6.json
for R in "$@"; do
  echo "== $R $(date +%T)"
  $PY imu_learn/hs_fix_ts.py "$R"
  rm -f "$R/imu_gt/gt.orig.npz" "$R/imu_gt/gt.npz" "$R/imu_gt/lens.json"   # lens_views.pkl (corner cache) is kept
  $PY -u imu_learn/imu_gt.py "$R" --board $B 2>&1 | grep -v "^\[" | tail -12
  $PY imu_learn/gt_overlay.py "$R" --board $B
  $PY imu_learn/traj_compare.py "$R"
  echo "== $R done $(date +%T)"
done
