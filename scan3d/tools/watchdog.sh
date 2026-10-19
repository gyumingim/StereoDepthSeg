#!/usr/bin/env bash
# Progress snapshot for long unattended runs (Claude re-arms "sleep 600; watchdog.sh" every 10 min).
cd "$(dirname "$0")/.."
echo "== $(date +%T)  load $(cut -d' ' -f1-3 /proc/loadavg)"
nvidia-smi --query-gpu=memory.used,utilization.gpu,power.draw --format=csv,noheader 2>/dev/null | sed 's/^/gpu: /'
echo "procs: train $(pgrep -fc 'imu_learn/train_imu.py')  imu_gt $(pgrep -fc 'imu_learn/imu_gt.py')  vio $(pgrep -fc 'imu_learn/vio_bench_hs.py')"
[ -d runs/vio ] && echo "vio results: $(ls runs/vio/*.json 2>/dev/null | wc -l)  newest: $(ls -t runs/vio/*.json 2>/dev/null | head -1 | xargs -r basename)"
for f in runs/vio_morning.log runs/overnight.log; do [ -f $f ] && echo "$(basename $f): $(grep -E '^==' $f | tail -1)"; done
for f in $(ls -t runs/overnight_train_*.log 2>/dev/null | head -8); do
  echo "$(basename $f .log): $(grep -E '^(보정 없음|바이어스만|물리 보정|CNN)|ep +[0-9]+' $f | tail -1 | cut -c1-70)"
done
