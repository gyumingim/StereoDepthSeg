#!/bin/bash
# featloop after the 11-06 10:50 PC crash: ONE job at a time (GPU or CPU), free RAM >= 12 GB before each job,
# torch fraction 0.6, no torch.compile. Only the pre-registered variants S X SL XL on H (greedy / seed extras dropped).
# A result counts as complete only with ba.json + poses_ba.jsonl.gz (BA) or summary.json (mesh); half-written dirs are redone.
# 1) BA per H recording: complete -> keep; else complete db (all pairs matched) -> CPU redo of verification (seed 0) + BA;
#    else full run (GPU features/matching under gpu.lock, then BA). 2) corner ruler per recording (CPU). 3) meshes (GPU).
W=$(cd "$(dirname "$0")/../../.." && pwd); cd "$W"
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
D=scan3d/runs/featloop_20261105; CACHE=$HOME/scanner/featloop_ext/cache
export FFS_ROOT=$HOME/scanner/StereoDepthSeg SCAN3D_GPU_POLL_S=3 SCAN3D_GPU_FRAC=0.6 TORCH_COMPILE_DISABLE=1 TORCHINDUCTOR_COMPILE_THREADS=1
H=${H:-"20261021_213341 20261021_215348 20261020_224323 20261020_223815 20261020_221925 20261020_221052 20261020_222747 20261020_214138 20261022_133820 20261022_134550 20261022_134204 20261021_214550"}
VARS="S X SL XL"
mem() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
memwait() { while [ $(mem) -lt 12 ]; do sleep 30; done; }
( while sleep 5; do a=$(mem); if [ "$a" -lt 10 ]; then for p in $(pgrep -f "featloop_20261105/rec/"); do
    r=$(awk '/VmRSS/{print int($2/1048576)}' /proc/$p/status 2>/dev/null); [ "${r:-0}" -ge 2 ] && { echo "WATCHDOG $(date +%T) avail $a GB, pid $p $r GB -> kill"; kill $p; }; done; fi; done ) &
WD=$!
dbok() {  # db.db + pairs.txt, every pair has a matches row
  [ -f $1/db.db ] && [ -f $1/pairs.txt ] && $P - "$1" <<'EOF'
import sys, pycolmap
d = sys.argv[1]; db = pycolmap.Database.open(d + "/db.db")
n = db.num_matched_image_pairs(); db.close()
need = sum(1 for l in open(d + "/pairs.txt") if l.strip())
sys.exit(0 if n >= need else 1)
EOF
}
run() { # retry a job killed by the watchdog (rc 143) up to 4 times
  for t in 1 2 3 4; do memwait; "$@"; rc=$?; [ $rc = 143 ] || return $rc; echo "   rc 143, retry $t"; sleep 120; done; return 143; }
for id in $H; do
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; GR=imu_gt_rs_cal1022; V=$D/vio/$id.poses.jsonl.gz; O=$D/rec/$id
  for v in $VARS; do
    case $v in S*) F=sift;; X*) F=xfeat_lg;; esac; case $v in *L) L=salad;; *) L=none;; esac
    [ -f $O/ba_$v/ba.json ] && [ -f $O/ba_$v/poses_ba.jsonl.gz ] && continue
    SRC=""; dbok $O/ba_${v}_gpu && SRC=ba_${v}_gpu
    if [ -z "$SRC" ] && dbok $O/ba_$v; then rm -rf $O/ba_${v}_gpu; mv $O/ba_$v $O/ba_${v}_gpu; SRC=ba_${v}_gpu; fi
    if [ -n "$SRC" ]; then
      echo "== $id $v BA redo from $SRC $(date +%T)"; rm -rf $O/ba_$v
      run $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/ba_$v --gt $G --kf-from-vio --match-ms 1 --feat $F --loop $L --cache-root $CACHE \
        --seed 0 --db-from $O/$SRC > $O/ba_$v.log 2>&1 || echo "   failed rc $?"
    else
      echo "== $id $v BA full $(date +%T)"; rm -rf $O/ba_$v
      run $P scan3d/scanA/kf_ba.py $R --vio $V --out $O/ba_$v --gt $G --kf-from-vio --match-ms 1 --feat $F --loop $L --cache-root $CACHE \
        --seed 0 --keep-db > $O/ba_$v.log 2>&1 || echo "   failed rc $?"
    fi
    grep -E "^\[sim3\]" $O/ba_$v.log
  done
  if [ ! -f $O/corner_main.json ]; then
    PO="--pose VIO=$O/ba_S/poses_vio_kf.jsonl.gz"; for v in $VARS; do [ -f $O/ba_$v/poses_ba.jsonl.gz ] && PO="$PO --pose $v=$O/ba_$v/poses_ba.jsonl.gz"; done
    echo "== $id corner $(date +%T)"
    run $P scan3d/scanA/corner_eval.py $R $PO --gt $G --gt-rs $GR --kf-step 1 --tol-ms 1 --max-gap-rows 1200 --out $O/corner_main.json > $O/corner_main.log 2>&1 || echo "   corner failed"
  fi
done
echo "BA+CORNER DONE $(date +%T)"
for id in $H; do
  R=$W/scan3d/recordings/hs_$id; G=imu_gt_spl_cal1022; O=$D/rec/$id
  for v in $VARS; do
    [ -f $O/mesh_$v/summary.json ] && continue; [ -f $O/ba_$v/poses_ba.jsonl.gz ] || continue
    echo "== $id $v mesh $(date +%T)"; rm -rf $O/mesh_$v
    run $P scan3d/scanA/scan_offline.py $R --out $O/mesh_$v --gt $G --vio $O/ba_$v/poses_ba.jsonl.gz --compress --kf-step 1 > $O/mesh_$v.log 2>&1 || echo "   mesh failed rc $?"
  done
done
kill $WD; echo "FINALQ DONE $(date +%T)"
