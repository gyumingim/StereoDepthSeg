#!/bin/bash
# gpuba 11-09 lane (README 2): sequential, one heavy job at a time.
# 1) fresh full timing on 20261022_133820 (cpu, cuda)  2) db-from cpu / cuda BA on the 11 H  3) scan_offline meshes for both
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/gpuba_20261109; FLR=$HOME/scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
H=${H:-"20261022_133820 20261022_134550 20261022_134204 20261021_214550 20261021_213341 20261021_215348 20261020_221052 20261020_221925 20261020_222747 20261020_223815 20261020_224323"}
mem() { awk "/MemAvailable/{print int(\$2/1048576)}" /proc/meminfo; }
if [ -z "$SKIP_FULL" ]; then
  for dev in cpu cuda; do [ -f $D/rec/20261022_133820/${dev}_full/ba.json ] || $D/time_rec.sh 20261022_133820 $dev ${dev}_full; done
fi
for id in $H; do
  SRC=$FLR/$id/ba_S; [ -f $SRC/db.db ] || SRC=$FLR/$id/ba_S_gpu
  for dev in cpu cuda; do [ -f $D/rec/$id/${dev}_db/poses_ba.jsonl.gz ] || $D/time_rec.sh $id $dev ${dev}_db --db-from $SRC; done
done
for id in $H; do
  R=$W/scan3d/recordings/hs_$id; O=$D/rec/$id
  for dev in cpu cuda; do
    [ -f $O/mesh_$dev/summary.json ] && continue
    [ -f $O/${dev}_db/poses_ba.jsonl.gz ] || continue
    while [ $(mem) -lt 8 ]; do sleep 30; done
    echo "== $id mesh $dev $(date +%T)"
    $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$dev --gt imu_gt_spl_cal1022 --vio $O/${dev}_db/poses_ba.jsonl.gz --compress --kf-step 1 \
      > $O/mesh_$dev.log 2>&1 || echo "   mesh failed rc $?"
  done
done
echo "LANE DONE $(date +%T)"
