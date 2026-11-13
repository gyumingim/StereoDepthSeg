#!/usr/bin/env bash
# fixes 11-13 unit 15: far_eval.py (CPU) on two 121801 unit-15 meshes. Usage: bash u15_eval.sh BASE_TAG OTHER_TAG
set -uo pipefail
WT=/home/system/scanner/StereoDepthSeg-fixes
cd "$WT"
O=scan3d/runs/desktop/20261112_121801_std60/fix_u15
REC=scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/red/hs_20261112_121801_std60_kf
/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/runs/fixes_20261112/far_eval.py "$O/mesh_$1" "$O/mesh_$2" "$REC" "$O/u15_$2_vs_$1" \
  > "$O/u15_$2_vs_$1_eval.log" 2>&1
echo "eval $2 vs $1 rc $?"
