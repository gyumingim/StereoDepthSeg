#!/usr/bin/env bash
# After redo_hs_gt.sh: whole-recording sync fit for every 240 fps recording, then training runs in parallel on the GPU
# (user set a GPU power limit, 2026-10-19): leave-one-recording-out for each recording + one split (old focus → fixed focus).
set -u
cd "$(dirname "$0")/.."
PY=../venv_ffs/bin/python
export PYTHONIOENCODING=utf-8
RECS=(recordings/hs_20261019_114000 recordings/hs_20261019_115620 recordings/hs_20261019_125714 recordings/hs_20261019_131326 recordings/hs_20261019_132803)
echo "== batch sync $(date +%T)"
$PY -u imu_learn/batch_sync.py "${RECS[@]}" --iters 600 --write 2>&1 | grep --line-buffered -v Warning
echo "== training $(date +%T)"
export IMU_VRAM_GB=2.5          # 6 runs x 2.5 = 15 GB < 15.5 GB limit (user, 2026-10-19)
COMMON="--win 3 --epochs 120 --gt-smooth"
for k in "${!RECS[@]}"; do
  val=${RECS[$k]}; train=()
  for j in "${!RECS[@]}"; do [ "$j" != "$k" ] && train+=("${RECS[$j]}"); done
  n=$(basename "$val")
  $PY -u imu_learn/train_imu.py --train "${train[@]}" --val "$val" $COMMON --out runs/imu_loo_$n > runs/imu_loo_$n.log 2>&1 &
done
$PY -u imu_learn/train_imu.py --train "${RECS[@]:0:3}" --val "${RECS[@]:3:2}" $COMMON --out runs/imu_split_fixedfocus \
  > runs/imu_split_fixedfocus.log 2>&1 &
wait
echo "== done $(date +%T)"
for f in runs/imu_split_fixedfocus.log runs/imu_loo_*.log; do echo "-- $f"; grep -E "^(보정 없음|바이어스만|물리 보정|CNN)" "$f" | cut -c1-200; done
