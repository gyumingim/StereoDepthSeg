#!/bin/bash
# mvggt 11-06: one recording, all steps (README 2-4). Every heavy step: flock ~/scanner/heavy.lock (one heavy job machine-wide),
# threads 8, GPU frac 0.6 + gpu.lock (gpu_guard), no torch.compile, wait for >= 12 GB free RAM, watchdog kills MY job < 8 GB.
# usage: run_rec.sh ID STEPS...   ID = 165506 | 20261022_134204 | 20261020_222747 ; STEPS from: dump b0 vggt v mast3r m  (default all)
# env: CHUNK (VGGT chunk, default 64), WIN (MASt3R swin window, default 8)
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
PY=$HOME/scanner/mvggt_env/venv/bin/python
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 \
       TORCHINDUCTOR_COMPILE_THREADS=1 FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=5
id=$1; shift; STEPS=${@:-dump b0 vggt v mast3r m}
D=scan3d/runs/mvggt_20261106/rec/$id
if [ "$id" = 165506 ]; then R=scan3d/recordings/hs_20261024_165506_std60; G=imu_gt_spl; SOO=""
else R=scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; SOO="--compress --kf-step 1"; fi
BA=$D/poses_ba.jsonl.gz; SUB=$D/frames/kf_subset.txt
memwait() { while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt 12 ]; do sleep 20; done; }
( while sleep 5; do a=$(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo)
    if [ "$a" -lt 8 ]; then for p in $(pgrep -f "mvggt_20261106/rec/$id"); do
      r=$(awk '/VmRSS/{print int($2/1048576)}' /proc/$p/status 2>/dev/null)
      [ "${r:-0}" -ge 3 ] && { echo "WATCHDOG $(date +%T) MemAvailable ${a} GB, my pid $p ${r} GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
heavy() { name=$1; shift; memwait; echo "== $id $name start $(date '+%F %T')"; flock $HOME/scanner/heavy.lock "$@" > $D/$name.log 2>&1;
          rc=$?; echo "== $id $name rc $rc $(date '+%F %T')"; tail -n 2 $D/$name.log | cut -c1-300; return $rc; }
mesh() { n=$1; shift; [ -f $D/mesh_$n/summary.json ] && { echo "== $n exists"; return; }
         heavy mesh_$n $PY scan3d/mvggt/mv_offline.py $R --out $D/mesh_$n --gt $G $SOO --kf-subset $SUB "$@"; }
for st in $STEPS; do case $st in
  dump) [ -f $D/frames/frames.npz ] || heavy dump $PY scan3d/mvggt/mv_dump.py $R --gt $G --ba $BA --out $D/frames ;;
  probe) for c in 96 64 48 32; do heavy probe_c$c $PY scan3d/mvggt/vggt_run.py $D/frames --out $D/probe_c$c/raw.npz --chunk $c --probe \
           && { echo $c > $D/vggt_chunk.txt; break; }; done ;;
  b0) mesh B0 --vio $BA; mesh B0_fs --vio $BA --free-scale ;;
  vggt) [ -f $D/raw_vggt.npz ] || heavy vggt $PY scan3d/mvggt/vggt_run.py $D/frames --out $D/raw_vggt.npz --chunk ${CHUNK:-$(cat scan3d/runs/mvggt_20261106/rec/165506/vggt_chunk.txt)} --overlap 8
        [ -f $D/raw_vggt.npz ] && heavy align_vggt $PY scan3d/mvggt/mv_align.py vggt $D/frames $D/raw_vggt.npz --out $D/al --vio-kf $D/poses_vio_kf.jsonl.gz ;;
  v) for v in V1 V2; do P=$BA; [ $v = V1 ] && P=$D/al/V1_poses.jsonl.gz
       mesh $v --vio $P --ext-depth $D/al/${v}_depth.npz; mesh ${v}_fs --vio $P --ext-depth $D/al/${v}_depth.npz --free-scale; done ;;
  mast3r) [ -f $D/raw_mast3r.npz ] || heavy mast3r $PY scan3d/mvggt/mast3r_run.py $D/frames --out $D/raw_mast3r.npz --win ${WIN:-8}
        [ -f $D/raw_mast3r.npz ] && heavy align_mast3r $PY scan3d/mvggt/mv_align.py mast3r $D/frames $D/raw_mast3r.npz --out $D/al --vio-kf $D/poses_vio_kf.jsonl.gz ;;
  m) for v in M1 M2; do P=$BA; [ $v = M1 ] && P=$D/al/M1_poses.jsonl.gz
       mesh $v --vio $P --ext-depth $D/al/${v}_depth.npz; mesh ${v}_fs --vio $P --ext-depth $D/al/${v}_depth.npz --free-scale; done ;;
esac; done
kill $WD
echo "DONE $id $(date '+%F %T')"
