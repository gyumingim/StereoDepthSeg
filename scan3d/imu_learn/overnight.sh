#!/usr/bin/env bash
# Overnight 2026-10-19: finish GT for all recordings, pooled lens, rolling-shutter GT, then trainings
# (old GT vs rolling-shutter GT, same held-out recordings) — every step logs to runs/overnight_*.log.
set -u
cd "$(dirname "$0")/.."
PY=../venv_ffs/bin/python
export PYTHONIOENCODING=utf-8
B=config/charuco/board_monitor_6x6.json
OLD=(recordings/hs_20261019_114000 recordings/hs_20261019_115620 recordings/hs_20261019_125714)
FIX=(recordings/hs_20261019_131326 recordings/hs_20261019_132803 recordings/hs_20261019_145816 recordings/hs_20261019_151358 recordings/hs_20261019_152631 recordings/hs_20261019_154320 recordings/hs_20261019_161035 recordings/hs_20261019_163345)
ALL=("${OLD[@]}" "${FIX[@]}")
log(){ echo "== $* $(date +%T)"; }

log 'step 1: GT for recordings without one + corner caches for the pooled lens'
for R in "${ALL[@]}"; do
  [ -f "$R/imu_gt/gt.npz" ] || bash imu_learn/redo_hs_gt.sh "$R" > runs/overnight_gt_$(basename $R).log 2>&1 &
done
for R in "${FIX[@]}"; do
  [ -f "$R/imu_gt/lens_views.pkl" ] || $PY imu_learn/lens_pool.py --collect "$R" > runs/overnight_lens_$(basename $R).log 2>&1 &
done
wait
while pgrep -f 'imu_learn/imu_gt.py' > /dev/null; do sleep 20; done
for R in "${ALL[@]}"; do [ -f "$R/imu_gt/gt.npz" ] || echo "!! no GT: $R"; done

log 'step 2: whole-recording sync where missing'
for R in "${ALL[@]}"; do
  grep -q batch_sync "$R/imu_gt/align.json" 2>/dev/null || $PY -u imu_learn/batch_sync.py "$R" --iters 1500 --write 2>&1 | grep -v Warning | tail -2
done

log 'step 3: pooled lens from all fixed-focus recordings (replaces the shared lens only if better on held-out views)'
$PY -u imu_learn/lens_pool.py "${FIX[@]}" --n 200 --write 2>&1 | grep -v Warning

log 'step 4: rolling-shutter GT -> imu_gt_rs (keeps imu_gt as the baseline)'
for R in "${ALL[@]}"; do
  ( $PY -u imu_learn/imu_gt.py "$R" --board $B --rs --out-dir imu_gt_rs --keep-corners 2>&1 | grep -E '정답|렌즈|롤링' ) > runs/overnight_rs_$(basename $R).log 2>&1 &
  while [ $(pgrep -fc 'imu_learn/imu_gt.py') -ge 6 ]; do sleep 10; done
done
wait
while pgrep -f 'imu_learn/imu_gt.py' > /dev/null; do sleep 20; done
cat runs/overnight_rs_*.log

log 'step 5: trainings, old GT vs rolling-shutter GT, held out = 4 fixed-focus recordings'
export IMU_VRAM_GB=1.3       # 1.25 GB desktop + 8 x (1.3 + ~0.4 context) = 14.85 GB < 15.5 GB
COMMON='--win 3 --epochs 40 --stride 1.5 --batch 96 --gt-smooth'
for val in recordings/hs_20261019_131326 recordings/hs_20261019_145816 recordings/hs_20261019_154320 recordings/hs_20261019_161035; do
  train=(); for r in "${ALL[@]}"; do [ "$r" != "$val" ] && train+=("$r"); done
  n=$(basename $val)
  $PY -u imu_learn/train_imu.py --train "${train[@]}" --val $val $COMMON --gt-dir imu_gt --out runs/on_old_$n > runs/overnight_train_old_$n.log 2>&1 &
  $PY -u imu_learn/train_imu.py --train "${train[@]}" --val $val $COMMON --gt-dir imu_gt_rs --out runs/on_rs_$n > runs/overnight_train_rs_$n.log 2>&1 &
done
wait
log 'done'
for f in runs/overnight_train_*.log; do echo "-- $f"; grep -E '^(보정 없음|바이어스만|물리 보정|CNN)' $f | cut -c1-220; tail -1 $f | cut -c1-200; done
