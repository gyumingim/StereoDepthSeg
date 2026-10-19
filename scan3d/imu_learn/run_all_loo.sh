#!/usr/bin/env bash
# All 240 fps recordings (old focus 1-3 + fixed 45 cm focus 4-9): make sure every recording has the whole-recording
# sync (batch_sync), then leave-one-recording-out trainings in parallel, held out = each fixed-focus recording.
set -u
cd "$(dirname "$0")/.."
PY=../venv_ffs/bin/python
export PYTHONIOENCODING=utf-8
OLD=(recordings/hs_20261019_114000 recordings/hs_20261019_115620 recordings/hs_20261019_125714)
FIX=(recordings/hs_20261019_131326 recordings/hs_20261019_132803 recordings/hs_20261019_145816 recordings/hs_20261019_151358 recordings/hs_20261019_152631 recordings/hs_20261019_154320 recordings/hs_20261019_161035)
ALL=("${OLD[@]}" "${FIX[@]}")
for R in "${ALL[@]}"; do
  [ -f "$R/imu_gt/gt.npz" ] || bash imu_learn/redo_hs_gt.sh "$R" > "runs/redo_$(basename $R).log" 2>&1
  if ! grep -q batch_sync "$R/imu_gt/align.json"; then
    echo "== sync $R $(date +%T)"
    $PY -u imu_learn/batch_sync.py "$R" --iters 1500 --write 2>&1 | grep -v Warning | tail -2
  fi
done
echo "== training $(date +%T)"
export IMU_VRAM_GB=1.6          # 1.25 GB desktop + 7 x (1.6 + ~0.4 context) = 15.25 GB < 15.5 GB limit
# 8 training recordings ≈ 2x the windows of the 4-recording runs: stride 1.5 s (windows still overlap 2x), 40 epochs
COMMON="--win 3 --epochs 40 --stride 1.5 --batch 128 --gt-smooth"
for val in "${FIX[@]}"; do
  train=(); for r in "${ALL[@]}"; do [ "$r" != "$val" ] && train+=("$r"); done
  n=$(basename "$val")
  $PY -u imu_learn/train_imu.py --train "${train[@]}" --val "$val" $COMMON --out runs/imu_all_$n > runs/imu_all_$n.log 2>&1 &
done
wait
echo "== done $(date +%T)"
for f in runs/imu_all_*.log; do echo "-- $f"; grep -E "^(보정 없음|바이어스만|물리 보정|CNN)" "$f" | cut -c1-220; done
