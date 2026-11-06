#!/bin/bash
# hybrid 11-07: one recording, all runs (README 3). GPU steps only: gpu.lock via gpu_guard (TSDF and FFS use CUDA), frac 0.6,
# no torch.compile, threads 8, wait for >= 8 GB free RAM, watchdog kills MY job if MemAvailable < 4 GB.
# usage: hy_rec.sh ID     inputs (read only): ~/scanner/StereoDepthSeg-mvggt/scan3d/runs/mvggt_20261106/rec/ID
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
PY=$HOME/scanner/mvggt_env/venv/bin/python
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=5
id=$1
M=$HOME/scanner/StereoDepthSeg-mvggt/scan3d/runs/mvggt_20261106/rec/$id
D=scan3d/runs/hybrid_20261107/rec/$id; mkdir -p $D
if [ "$id" = 165506 ]; then R=scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; SOO=""
else R=scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; SOO="--compress --kf-step 1"; fi
EX=""; [ "$id" = 20261022_134204 ] && EX="--exclude-chunks 6"      # README 2: known broken chunk (mvggt 7.2), H1 only
BA=$M/poses_ba.jsonl.gz; SUB=$M/frames/kf_subset.txt
FILL="--fill-depth $M/al/V2_depth.npz --fill-raw $M/raw_vggt.npz"
( while sleep 5; do a=$(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo)
    if [ "$a" -lt 4 ]; then for p in $(pgrep -f "hybrid_20261107/rec/$id/"); do
      r=$(awk "/VmRSS/{print int(\$2/1048576)}" /proc/$p/status 2>/dev/null)
      [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) MemAvailable ${a} GB, my pid $p ${r} GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
run() { n=$1; shift; [ -f $D/mesh_$n/summary.json ] && { echo "== $n exists"; return; }
  while [ $(awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo) -lt 8 ]; do sleep 20; done
  echo "== $id $n start $(date "+%F %T")"
  $PY scan3d/hybrid/hy_offline.py $R --out $D/mesh_$n --gt $G $SOO --vio $BA "$@" > $D/mesh_$n.log 2>&1
  echo "== $id $n rc $? $(date "+%F %T")"; tail -n 1 $D/mesh_$n.log | cut -c1-200; }
for fs in "" _fs; do
  F=${fs:+--free-scale}; C=$D/ffs${fs:-_native}.npz; OV=$M/overlapV${fs:-_native}.txt
  run B0r$fs --kf-subset $SUB $F --save-ffs $C
  [ -f $C ] || { echo "no cache $C"; continue; }
  run B0c$fs --kf-subset $SUB $F --ffs-cache $C
  run H1$fs  --kf-subset $SUB $F --ffs-cache $C $FILL --hybrid H1 $EX
  run H2$fs  --kf-subset $SUB $F --ffs-cache $C $FILL --hybrid H2
  run H3$fs  --kf-subset $SUB $F --ffs-cache $C $FILL --hybrid H3
  run B0c_ov$fs --kf-subset $OV $F --ffs-cache $C
  run H3_ov$fs  --kf-subset $OV $F --ffs-cache $C $FILL --hybrid H3
  [ "$id" = 165506 ] && run H1_ov$fs --kf-subset $OV $F --ffs-cache $C $FILL --hybrid H1 $EX
done
kill $WD
echo "DONE $id $(date "+%F %T")"
