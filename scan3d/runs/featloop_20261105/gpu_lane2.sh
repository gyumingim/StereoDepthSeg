#!/bin/bash
# featloop GPU lane 2 (11-06 17:10, coordinator: GPU idle -> meshes in a lane that takes only gpu.lock; CPU lane under heavy.lock):
# meshes for every complete BA without a mesh; repeats passes until the
# CPU lane (restart_order.sh) is done and nothing is left. GPU rules: gpu.lock, fraction 0.6, TORCH_COMPILE_DISABLE=1.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
H="20261021_213341 20261021_215348 20261020_224323 20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261022_133820 20261022_134550 20261022_134204 20261021_214550"
mem() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
memwait() { while [ $(mem) -lt 8 ]; do sleep 30; done; }
run() { for t in 1 2 3 4; do memwait; "$@"; rc=$?; [ $rc = 143 ] || return $rc; echo "   rc 143, retry $t"; sleep 120; done; return 143; }
last=0
while true; do
  grep -q "CPULANE DONE" $D/rec/cpu_lane.out 2>/dev/null && last=1
  for v in S X SL XL; do for id in $H; do
    O=$D/rec/$id; R=$W/scan3d/recordings/hs_$id
    [ -f $O/mesh_$v/summary.json ] && continue
    [ -f $O/ba_$v/ba.json ] && [ -f $O/ba_$v/poses_ba.jsonl.gz ] || continue
    echo "== $id $v mesh $(date +%T)"; rm -rf $O/mesh_$v
    run $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt imu_gt_spl_cal1022 --vio $O/ba_$v/poses_ba.jsonl.gz --compress --kf-step 1 \
      > $O/mesh_$v.log 2>&1 || echo "   mesh failed rc $?"
  done; done
  [ $last = 1 ] && break
  sleep 120
done
echo "GPULANE DONE $(date +%T)"
