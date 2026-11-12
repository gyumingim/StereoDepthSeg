#!/usr/bin/env bash
# Wait for the far-pairs agent (unit 14) to finish with the GPU, then run sweep_ffs_job.sh.
# Start when interim_far.txt has a final / 완료 line AND no GPU compute process for >= 5 min,
# or (fail-safe) no GPU compute process for >= 45 min. Polls every 60 s, logs every ~5 min.
set -uo pipefail
R=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/desktop/sweep_ffs_20261112
F=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_far.txt
LOGW=/mnt/c/Users/a3162/scan3d_handoff/review_20261112/interim_sweep.txt
mkdir -p "$R"
idle0=""; k=0
echo "$(date '+%m-%d %T') FFS sweep: prepared, waiting for the far-pairs agent to finish (GPU free + final line in interim_far.txt)" >> "$LOGW"
while :; do
  nproc_gpu=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -c .)
  busy=$(pgrep -f 'scan_offline.py|ffs_trt.py|far_eval.py|far14_job|run_scanner.sh' | wc -l)
  fin=$(grep -Eic 'final|완료' "$F" 2>/dev/null); fin=${fin:-0}
  now=$(date +%s)
  if [ "$nproc_gpu" = 0 ] && [ "$busy" = 0 ]; then [ -n "$idle0" ] || idle0=$now; else idle0=""; fi
  idle=$([ -n "$idle0" ] && echo $((now - idle0)) || echo 0)
  [ $((k % 5)) = 0 ] && echo "$(date '+%m-%d %T') gpu procs $nproc_gpu, other jobs $busy, final lines $fin, idle $idle s" >> "$R/wait.log"
  k=$((k + 1))
  if { [ "$fin" -gt 0 ] && [ "$idle" -ge 300 ]; } || [ "$idle" -ge 2700 ]; then
    echo "$(date '+%m-%d %T') start: final lines $fin, idle $idle s" >> "$R/wait.log"
    echo "$(date '+%m-%d %T') FFS sweep: GPU free (final lines $fin, idle $idle s), starting" >> "$LOGW"
    break
  fi
  sleep 60
done
exec bash /home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112/sweep_ffs_job.sh "$@"
