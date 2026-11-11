#!/bin/bash
# gpuba 11-11, runs ON THE LAPTOP (via remote_laptop.sh job REC "cmd:bash scan3d/runs/gpuba_20261109/lap_rec.sh ..."):
# README 2 comparison for one recording, same machine: new (cuda BA + --fast-tracks) and CPU (default) kf_ba from the same
# featloop db (--db-from, seed 0), tri dumps of both, then scan_offline meshes of both pose sets. Sent db dir removed at the end.
# usage: lap_rec.sh R O G VIO SRC [KFO...]     (R, O, G as set by remote_laptop.sh)
R=$1; O=$2; G=$3; V=$4; SRC=$5; shift 5; KFO="$@"
P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python; CACHE=/home/system/scanner/laptop_cache
export FFS_ROOT=/home/system/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
mkdir -p $O
# image cache first (both BA runs then start from the same cached images; not part of either timing)
$P scan3d/scanA/kf_ba.py $R --vio $V --out $O/warm --gt $G $KFO --feat sift --loop none --cache-root $CACHE --seed 0 --db-from $SRC --dump-tri > $O/warm.log 2>&1
rm -rf $O/warm
for v in cuda cpu; do
  F="--no-fast-tracks --sift-device auto"; [ $v = cuda ] && F="--fast-tracks --sift-device auto"
  echo "$(date +%T) $v start"
  /usr/bin/time -f "TIME $v %e s maxrss %M kB" $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/${v}_db --gt $G $KFO --feat sift --loop none \
    --cache-root $CACHE --seed 0 --db-from $SRC --ba-device $v --save-tri $F > $O/${v}_db.log 2>&1 || echo "$v failed rc $?"
  tail -n 1 $O/${v}_db.log | cut -c1-80; grep TIME $O/${v}_db.log
done
$P - $O <<"PY"
import sys, json, numpy as np
o = sys.argv[1]
try:
    a = np.load(o + "/cpu_db/tri_dump.npz"); b = np.load(o + "/cuda_db/tri_dump.npz")
    r = {k: bool(a[k].shape == b[k].shape and (a[k] == b[k]).all()) for k in a.files}
except Exception as e:
    r = {"error": str(e)}
print("TRI_EQUAL", json.dumps(r)); open(o + "/tri_equal.json", "w").write(json.dumps(r))
PY
rm -f $O/cpu_db/tri_dump.npz $O/cuda_db/tri_dump.npz
for v in cuda cpu; do
  [ -f $O/${v}_db/poses_ba.jsonl.gz ] || continue
  echo "$(date +%T) mesh $v"
  SCAN3D_GPU_FRAC=0.75 /usr/bin/time -f "TIME mesh_$v %e s maxrss %M kB" $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt $G --vio $O/${v}_db/poses_ba.jsonl.gz --compress --kf-step 1 \
    > $O/mesh_$v.log 2>&1 || echo "mesh $v failed rc $?"
  rm -f $O/mesh_$v/mesh.ply $O/mesh_$v/mesh.obj
done
rm -rf $SRC
echo "$(date +%T) LAPREC DONE"
