#!/usr/bin/env bash
# fixes 11-13 unit 15: 2nd laptop office job = the far variants with TSDF eviction (fits 8 GB), results fix_u15far; then CPU evals here.
cd /home/system/scanner/StereoDepthSeg-fixes
VARIANTS="farnone farguard farp2" OFFICE_EXTRA="--tsdf-evict-gpu-blocks 60000" OTAG=fix_u15far \
  bash scan3d/runs/fixes_20261112/u15_laptop_launch.sh office
L=scan3d/runs/laptop/20261112_121801_std60
REC=$L/fix_u8/scan/red/hs_20261112_121801_std60_kf
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
ev() {  # ev BASE_DIR OTHER_DIR OUT_PREFIX
  $PY scan3d/runs/fixes_20261112/far_eval.py "$1" "$2" "$REC" "$3" > "$3_eval.log" 2>&1; echo "eval $3 rc $?"
}
E=$L/fix_u15far
ev $L/fix_u15/mesh_base $L/fix_u15/mesh_sig $E/u15_sig_vs_base
for v in farnone farguard farp2; do [ -f $E/mesh_$v/summary.json ] && ev $L/fix_u15/mesh_sig $E/mesh_$v $E/u15_${v}_vs_sig; done
for v in farguard farp2; do [ -f $E/mesh_$v/summary.json ] && ev $E/mesh_farnone $E/mesh_$v $E/u15_${v}_vs_farnone; done
echo "u15 laptop far + evals done"
