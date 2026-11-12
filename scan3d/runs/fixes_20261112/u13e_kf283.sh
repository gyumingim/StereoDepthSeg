#!/usr/bin/env bash
# review 11-12 finding 2: kf_0283-0290 of 121801, same inputs (fix_u8 keyframes + BA poses, the trt13d_full.sh command):
# PyTorch fp16 twice, PyTorch fp32, TRT (fresh engines_v3) -> fused depth dumps, pairwise trt_compare.
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
unset SCAN3D_FFS_BACKEND
P=$FFS_ROOT/venv_ffs/bin/python
TR=/home/system/scanner/ffs_trt
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u13e_kf283
DD=$TR/dump13e_kf283
mkdir -p "$O" "$DD"
echo '[283, 284, 285, 286, 287, 288, 289, 290]' > "$O/kf_only.json"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --kf-only $O/kf_only.json"
echo "$(date '+%F %T') start, commit $(git rev-parse HEAD)" > "$O/times.txt"
for v in ${VARIANTS:-pt_a pt_b pt_fp32 trt}; do
  case $v in pt_a|pt_b) bk=pytorch;; pt_fp32) bk=pytorch_fp32;; trt) bk=trt;; esac
  rm -rf "$DD/$v"; t0=$SECONDS
  $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/$v" $ARGS --ffs-backend $bk \
    --dump-depth "$DD/$v" > "$O/$v.log" 2>&1 || echo "$v FAILED" >> "$O/times.txt"
  echo "$v ($bk) $((SECONDS - t0)) s" >> "$O/times.txt"
  rm -f "$O/$v/mesh.obj" "$O/$v/mesh.ply"
done
for pr in "pt_a pt_b" "pt_a pt_fp32" "pt_b pt_fp32" "pt_a trt" "pt_b trt" "pt_fp32 trt"; do
  set -- $pr
  $P scan3d/runs/fixes_20261112/trt_compare.py --a "$DD/$1" --b "$DD/$2" --sa "$O/$1/summary.json" --sb "$O/$2/summary.json" \
    --out "$O/cmp_${1}_vs_${2}.json" > /dev/null 2>&1 || echo "cmp $1 $2 failed" >> "$O/times.txt"
done
echo "$(date '+%F %T') done" >> "$O/times.txt"
