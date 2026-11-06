#!/bin/bash
# featloop after the 3rd crash (11-06 12:46): ONE heavy job machine-wide, every heavy job under flock ~/scanner/heavy.lock
# (shared with viba), threads 8, GPU fraction 0.6, no torch.compile, free RAM >= 8 GB before each job. No parallel lanes.
# Scope: pre-registered S X SL XL on the H recordings that already have features; per recording: CPU redo of missing BAs from
# their complete db (seed 0) -> corner ruler -> meshes for the dZ metric. 214138 skipped (XL has no features).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache; HL=$HOME/scanner/heavy.lock
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
export OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8
H=${H:-"20261021_213341 20261021_214550 20261022_133820 20261022_134204 20261022_134550 20261020_221052 20261020_224323 20261020_222747 20261021_215348 20261020_223815 20261020_221925"}
VARS="S X SL XL"
mem() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
heavy() { while [ $(mem) -lt 8 ]; do sleep 30; done; flock $HL "$@"; }
dbok() {
  [ -f $1/db.db ] && [ -f $1/pairs.txt ] && $P - "$1" <<'EOF'
import sys, pycolmap
d = sys.argv[1]; db = pycolmap.Database.open(d + "/db.db"); n = db.num_matched_image_pairs(); db.close()
sys.exit(0 if n >= sum(1 for l in open(d + "/pairs.txt") if l.strip()) else 1)
EOF
}
for id in $H; do
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; GR=imu_gt_rs_cal1022; V=$D/vio/$id.poses.jsonl.gz; O=$D/rec/$id
  for v in $VARS; do
    case $v in S*) F=sift;; X*) F=xfeat_lg;; esac; case $v in *L) L=salad;; *) L=none;; esac
    [ -f $O/ba_$v/ba.json ] && [ -f $O/ba_$v/poses_ba.jsonl.gz ] && continue
    SRC=""; dbok $O/ba_${v}_gpu && SRC=ba_${v}_gpu
    if [ -z "$SRC" ] && dbok $O/ba_$v; then rm -rf $O/ba_${v}_gpu; mv $O/ba_$v $O/ba_${v}_gpu; SRC=ba_${v}_gpu; fi
    [ -z "$SRC" ] && { echo "== $id $v no complete db -> skipped"; continue; }
    echo "== $id $v BA redo from $SRC $(date +%T)"; rm -rf $O/ba_$v
    heavy $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/ba_$v --gt $G --kf-from-vio --match-ms 1 --feat $F --loop $L --cache-root $CACHE \
      --seed 0 --db-from $O/$SRC > $O/ba_$v.log 2>&1 || echo "   failed rc $?"
    grep -E "^\[sim3\]" $O/ba_$v.log
  done
  if [ ! -f $O/corner_main.json ]; then
    PO="--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz"; n=0
    for v in $VARS; do [ -f $O/ba_$v/ba.json ] && { PO="$PO --pose $v=$O/ba_$v/poses_ba.jsonl.gz"; n=$((n+1)); }; done
    [ -f $O/ba_S/poses_vio_kf.jsonl.gz ] || PO=${PO#--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz}
    echo "== $id corner ($n variants) $(date +%T)"
    heavy $P scan3d/scanA/corner_eval.py $R $PO --gt $G --gt-rs $GR --kf-step 1 --tol-ms 1 --max-gap-rows 1200 --out $O/corner_main.json \
      > $O/corner_main.log 2>&1 || echo "   corner failed"
  fi
  for v in $VARS; do
    [ -f $O/mesh_$v/summary.json ] && continue; [ -f $O/ba_$v/ba.json ] || continue
    echo "== $id $v mesh $(date +%T)"; rm -rf $O/mesh_$v
    heavy $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt $G --vio $O/ba_$v/poses_ba.jsonl.gz --compress --kf-step 1 \
      > $O/mesh_$v.log 2>&1 || echo "   mesh failed rc $?"
  done
  echo "-- $id complete $(date +%T)"
done
echo "FINALQ3 DONE $(date +%T)"
