#!/usr/bin/env bash
# step 3b: all-fp32 TRT engines (diagnostic, static 544x960 + 960x544 only) on kf283-290 -> does the TRT graph equal PyTorch fp32?
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
export FFS_ROOT=/home/system/scanner/StereoDepthSeg PATH=/usr/local/cuda/bin:$PATH PYTHONUNBUFFERED=1
unset SCAN3D_FFS_BACKEND FFS_TRT_DECOMP_IN
P=$FFS_ROOT/venv_ffs/bin/python
TR=/home/system/scanner/ffs_trt
ENGD=$TR/engines_v3_fp32diag
O=$WT/scan3d/runs/desktop/20261112_121801_std60/fix_u13e_kf283
DD=$TR/dump13e_kf283
t0=$SECONDS
if [ ! -f $ENGD/manifest.json ]; then
  FFS_TRT_FP32=1 $P scan3d/ffs_trt.py build --out "$ENGD" --sizes 544x960,960x544 > "$O/build_fp32diag.log" 2>&1 || { echo "fp32 build FAILED" >> "$O/times.txt"; exit 1; }
fi
echo "trt32 build $((SECONDS - t0)) s" >> "$O/times.txt"
U8=$WT/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan
ARGS="--gt lensonly --vio $U8/ba/poses_ba.jsonl.gz --compress --kf-step 1 --tsdf-alloc dense --ffs-views 2 --ffs-fuse auto \
 --ffs-partners all --sigma-rot-deg 0.1 --sigma-b-rel 0.02 --tsdf-max-blocks 250000 --kf-only $O/kf_only.json"
rm -rf "$DD/trt32"; t0=$SECONDS
FFS_TRT_PRECISION=fp32 $P scan3d/scanA/scan_offline.py "$U8/red/hs_20261112_121801_std60_kf" --out "$O/trt32" $ARGS --ffs-backend trt \
  --ffs-engines "$ENGD" --dump-depth "$DD/trt32" > "$O/trt32.log" 2>&1 || echo "trt32 FAILED" >> "$O/times.txt"
echo "trt32 $((SECONDS - t0)) s" >> "$O/times.txt"
rm -f "$O/trt32/mesh.obj" "$O/trt32/mesh.ply"
$P scan3d/runs/fixes_20261112/kf283_table.py $DD $O/kf283_table_fp32.json pt_fp32:trt32 pt_a:trt32 trt:trt32
