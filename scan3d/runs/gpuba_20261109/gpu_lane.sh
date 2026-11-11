#!/bin/bash
# gpuba 11-09 GPU lane (README 2): new version (--ba-device cuda --fast-tracks) db-from runs on the 11 H (heavy.lock via time_rec.sh,
# so they interleave with cpu_lane.sh jobs and never overlap them), then scan_offline meshes (GPU lock only) for cpu and cuda poses,
# repeated until cpu_lane.sh is done and every pose file has its mesh.
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/gpuba_20261109; FLR=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
H=${H:-"20261022_133820 20261022_134550 20261022_134204 20261021_214550 20261021_213341 20261021_215348 20261020_221052 20261020_221925 20261020_222747 20261020_223815 20261020_224323"}
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
for id in $H; do
  SRC=$FLR/$id/ba_S; [ -f $SRC/db.db ] || SRC=$FLR/$id/ba_S_gpu
  [ -f $D/rec/$id/cuda_db/poses_ba.jsonl.gz ] || $D/time_rec.sh $id cuda cuda_db --db-from $SRC --fast-tracks
done
last=0
while true; do
  grep -q "CPULANE DONE" $D/cpu_lane.out && last=1
  for id in $H; do
    R=$W/scan3d/recordings/hs_$id; O=$D/rec/$id
    for dev in cuda cpu; do
      [ -f $O/mesh_$dev/summary.json ] && continue
      [ -f $O/${dev}_db/poses_ba.jsonl.gz ] || continue
      while [ $(mem) -lt 8 ]; do sleep 30; done
      echo "== $id mesh $dev $(date +%T)"; rm -rf $O/mesh_$dev
      $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$dev --gt imu_gt_spl_cal1022 --vio $O/${dev}_db/poses_ba.jsonl.gz --compress --kf-step 1 \
        > $O/mesh_$dev.log 2>&1 || echo "   mesh failed rc $?"
      tail -n 1 $O/mesh_$dev.log | cut -c1-200
    done
  done
  [ $last = 1 ] && break
  sleep 60
done
echo "GPULANE DONE $(date +%T)"
