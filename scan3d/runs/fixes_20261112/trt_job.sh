#!/usr/bin/env bash
# fixes 11-12 unit 11 (laptop job, run via remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/trt_job.sh'):
# build FFS TensorRT engines (if missing) -> scan_offline PyTorch vs TRT on the fix_u8 keyframes + BA poses -> trt_compare.json
# Desktop (11-12 user: desktop RTX 5080 free too): same script, run directly there with WT O P set + FFS_ROOT exported;
# BUILD_ONLY=1 stops after the engines. Engines are GPU-specific (manifest gpu name checked), so each machine builds its own.
# Env from the job: WT O P. Outputs in $O (pulled back); depth dumps + engines stay on the laptop (/home/system/scanner/ffs_trt).
set -euo pipefail
cd "$WT"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
IN=$U8/red/hs_20261112_121801_std60_kf
POSES=$U8/ba/poses_ba.jsonl.gz
TR=/home/system/scanner/ffs_trt
ENG=$TR/engines
SIZES=${SIZES:-544x960,448x768,352x576,256x448,960x544,768x448,576x352,448x256}
mkdir -p "$O" "$TR"
export PYTHONUNBUFFERED=1
# same mesh command line as run_scanner.sh for fix_u8 (summary.json args): ffs-views 2, partners all, depth-sigma ba, cap 250k
ARGS="--gt lensonly --vio $POSES --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto --ffs-partners all \
 --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000"

gpumon() { nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader,nounits -lms 500 > "$1" 2>/dev/null & echo $!; }
peak() { awk -F', ' 'NR==1||$2>m{m=$2} END{print m}' "$1"; }

echo "$(date +%T) idle GPU memory MiB: $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits)" | tee "$O/gpu_idle.txt"
$P -c "import sys; sys.path.append('$TR/site'); import tensorrt, torch; print('tensorrt', tensorrt.__version__, 'torch', torch.__version__, torch.version.cuda)" | tee "$O/versions.txt"

if [ ! -f "$ENG/manifest.json" ]; then
  m=$(gpumon "$O/gpu_build.csv"); t0=$SECONDS
  /usr/bin/time -f 'TIME build %e s maxrss %M kB' $P scan3d/ffs_trt.py build --out "$ENG" --sizes "$SIZES" > "$O/build.log" 2>&1 || { kill $m; tail -30 "$O/build.log"; exit 1; }
  kill $m; echo "build $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_build.csv") MiB" | tee -a "$O/build.log"
fi
cp "$ENG/manifest.json" "$O/engine_manifest.json"

run() {  # run TAG extra-args
  local tag=$1; shift
  rm -rf "$TR/dump_$tag"
  m=$(gpumon "$O/gpu_$tag.csv"); t0=$SECONDS
  /usr/bin/time -f "TIME mesh_$tag %e s maxrss %M kB" $P scan3d/scanA/scan_offline.py "$IN" --out "$O/mesh_$tag" $ARGS \
    --dump-depth "$TR/dump_$tag" "$@" > "$O/mesh_$tag.log" 2>&1 || { kill $m; tail -30 "$O/mesh_$tag.log"; exit 1; }
  kill $m; echo "mesh_$tag $((SECONDS - t0)) s, GPU peak $(peak "$O/gpu_$tag.csv") MiB" | tee -a "$O/times.txt"
}
[ "${BUILD_ONLY:-0}" = 1 ] && { echo "$(date +%T) BUILD_ONLY: engines done"; exit 0; }
run pt
run trt --ffs-backend trt --ffs-engines "$ENG"
$P scan3d/runs/fixes_20261112/trt_compare.py --a "$TR/dump_pt" --b "$TR/dump_trt" --sa "$O/mesh_pt/summary.json" \
  --sb "$O/mesh_trt/summary.json" --ka "$O/mesh_pt/keyframes.json" --kb "$O/mesh_trt/keyframes.json" --out "$O/trt_compare.json" \
  > "$O/compare.log" 2>&1
# no mesh files back (2 x 230 MB): keep them on the laptop
mkdir -p "$TR/mesh_keep"; for t in pt trt; do mv "$O/mesh_$t/mesh.obj" "$TR/mesh_keep/mesh_$t.obj"; mv "$O/mesh_$t/mesh.ply" "$TR/mesh_keep/mesh_$t.ply"; done
echo "$(date +%T) trt_job done"
