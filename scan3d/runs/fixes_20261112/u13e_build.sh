#!/usr/bin/env bash
# review 11-12 step 2: rebuild all 4 export-v3 engines into an EMPTY engines_v3 in one logged build (old dir kept aside)
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
unset FFS_TRT_DECOMP_IN FFS_TRT_FP32 FFS_TRT_ALLOW_BUCKET
P=$FFS_ROOT/venv_ffs/bin/python
TR=/home/system/scanner/ffs_trt
O=$WT/scan3d/runs/fixes_20261112/u13e_build
mkdir -p "$O"
ENGD=$TR/engines_v3
if [ -d "$ENGD" ] && [ ! -d "$TR/engines_v3_old_unlogged" ]; then mv "$ENGD" "$TR/engines_v3_old_unlogged"; fi
[ -e "$ENGD" ] && { echo "engines_v3 exists, refusing"; exit 2; }
echo "$(date '+%F %T') start, commit $(git rev-parse HEAD) (ffs_trt.py dirty: $(git diff --stat scan3d/ffs_trt.py | tail -1)), gpu $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader)" > "$O/times.txt"
t0=$SECONDS
/usr/bin/time -f 'TIME build %e s maxrss %M kB' $P scan3d/ffs_trt.py build --out "$ENGD" --sizes 544x960,960x544 \
  --dynamic "128x128,384x672,544x960;128x128,672x384,960x544" > "$O/build.log" 2>&1
rc=$?
echo "$(date '+%F %T') build rc $rc $((SECONDS - t0)) s" >> "$O/times.txt"
[ $rc = 0 ] || exit $rc
cp "$ENGD/manifest.json" "$O/engine_manifest.json"
$P scan3d/ffs_trt.py check > "$O/check.log" 2>&1; echo "check rc $?" >> "$O/times.txt"
echo "$(date '+%F %T') done" >> "$O/times.txt"
