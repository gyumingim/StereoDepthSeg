#!/usr/bin/env bash
# fixes 11-12 unit 13b (laptop job): padding vs fp16 on the 40 downscaled (s<1) keyframes of 121801 (fix_u8 keyframes + BA poses).
# Variants (all --kf-only those 40, depth dumps): pt = PyTorch fp16 (as default, sanity vs the u13 full run), pt_bucket = PyTorch fp16
# with the TRT bucket padding, pt_fp32 = PyTorch without autocast, trt_dyn = ONE dynamic-shape TRT fp16 engine padded to /32 like
# lib_ffs (no extra padding). trt bucket = the u13 full-run dump. Compares: every variant vs pt fp16 (u13 full) and vs pt_fp32.
# Env from the job: WT O P.
set -euo pipefail
cd "$WT"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
IN=$U8/red/hs_20261112_121801_std60_kf
POSES=$U8/ba/poses_ba.jsonl.gz
KF=$WT/scan3d/runs/fixes_20261112/u13b_kf_s_lt1.json
TR=/home/system/scanner/ffs_trt
ENGD=$TR/engines_dyn
DYN=${DYN:-256x256,448x768,960x960}
mkdir -p "$O"
export PYTHONUNBUFFERED=1
ARGS="--gt lensonly --vio $POSES --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto --ffs-partners all \
 --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --kf-only $KF"
gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }

if [ ! -f "$ENGD/manifest.json" ]; then
  m=$(gpumon "$O/gpu_build_dyn.csv"); t0=$SECONDS
  /usr/bin/time -f 'TIME build_dyn %e s maxrss %M kB' $P scan3d/ffs_trt.py build --out "$ENGD" --dynamic "$DYN" > "$O/build_dyn.log" 2>&1 \
    || { kill $m; echo "dynamic build FAILED (see build_dyn.log)" | tee -a "$O/times.txt"; tail -30 "$O/build_dyn.log"; }
  kill $m 2>/dev/null || true; echo "build_dyn $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_build_dyn.csv") MiB" | tee -a "$O/times.txt"
fi
[ -f "$ENGD/manifest.json" ] && cp "$ENGD/manifest.json" "$O/engine_dyn_manifest.json"

run() {  # run TAG extra-args
  local tag=$1; shift
  rm -rf "$TR/dump13b_$tag"
  m=$(gpumon "$O/gpu_$tag.csv"); t0=$SECONDS
  if /usr/bin/time -f "TIME $tag %e s maxrss %M kB" $P scan3d/scanA/scan_offline.py "$IN" --out "$O/mesh_$tag" $ARGS \
       --dump-depth "$TR/dump13b_$tag" "$@" > "$O/mesh_$tag.log" 2>&1; then
    echo "$tag $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_$tag.csv") MiB" | tee -a "$O/times.txt"
  else
    echo "$tag FAILED" | tee -a "$O/times.txt"; tail -20 "$O/mesh_$tag.log"
  fi
  kill $m 2>/dev/null || true
  rm -f "$O/mesh_$tag/mesh.obj" "$O/mesh_$tag/mesh.ply"
}
run pt
run pt_bucket --ffs-backend pytorch_bucket
run pt_fp32 --ffs-backend pytorch_fp32
[ -f "$ENGD/manifest.json" ] && run trt_dyn --ffs-backend trt --ffs-engines "$ENGD"

cmp() {  # cmp NAME DUMP_A SUMM_A DUMP_B SUMM_B
  [ -d "$2" ] && [ -d "$4" ] || return 0
  $P scan3d/runs/fixes_20261112/trt_compare.py --a "$2" --b "$4" --sa "$3" --sb "$5" --out "$O/cmp_$1.json" > "$O/cmp_$1.log" 2>&1 || echo "cmp $1 failed"
}
S13=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u11trt
for v in pt pt_bucket pt_fp32 trt_dyn; do
  cmp "ptfull_vs_$v" "$TR/dump_pt" "$S13/mesh_pt/summary.json" "$TR/dump13b_$v" "$O/mesh_$v/summary.json"
done
cmp ptfull_vs_trtbucket "$TR/dump_pt" "$S13/mesh_pt/summary.json" "$TR/dump_trt" "$S13/mesh_trt/summary.json"   # all 351 kf (40 subset in summary script)
for v in pt pt_bucket trt_dyn; do
  cmp "fp32_vs_$v" "$TR/dump13b_pt_fp32" "$O/mesh_pt_fp32/summary.json" "$TR/dump13b_$v" "$O/mesh_$v/summary.json"
done
cmp fp32_vs_trtbucket "$TR/dump13b_pt_fp32" "$O/mesh_pt_fp32/summary.json" "$TR/dump_trt" "$S13/mesh_trt/summary.json"
cmp ptbucket_vs_trtbucket "$TR/dump13b_pt_bucket" "$O/mesh_pt_bucket/summary.json" "$TR/dump_trt" "$S13/mesh_trt/summary.json"
echo "$(date +%T) trt13b_job done"
