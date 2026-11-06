#!/bin/bash
# mvggt (coordinator 11-06): re-fuse + score each variant on the overlap keyframes only (rec/<id>/overlap_{native,fs}.txt from
# overlap.py --write-lists) -> rec/<id>/mesh_<V>_ov[_fs]. GPU (gpu.lock via gpu_guard), same env rules as run_rec.sh.
# usage: ov_meshes.sh ID VARS...
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
PY=$HOME/scanner/mvggt_env/venv/bin/python
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=5
id=$1; shift
D=scan3d/runs/mvggt_20261106/rec/$id
if [ "$id" = 165506 ]; then R=scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; SOO=""
else R=scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; SOO="--compress --kf-step 1"; fi
BA=$D/poses_ba.jsonl.gz
for v in "$@"; do for fs in "" _fs; do
  n=${v}_ov$fs; [ -f $D/mesh_$n/summary.json ] && continue
  while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt 8 ]; do sleep 20; done
  P=$BA; E=""; case $v in V1|M1) P=$D/al/${v}_poses.jsonl.gz;; esac
  [ $v != B0 ] && E="--ext-depth $D/al/${v}_depth.npz"
  L=$D/overlap${fs:-_native}.txt
  echo "== $id $n start $(date '+%F %T')"
  $PY scan3d/mvggt/mv_offline.py $R --out $D/mesh_$n --gt $G $SOO --kf-subset $L --vio $P $E ${fs:+--free-scale} > $D/mesh_$n.log 2>&1
  echo "== $id $n rc $? $(date '+%F %T')"
done; done
echo "OV DONE $id $(date '+%F %T')"
