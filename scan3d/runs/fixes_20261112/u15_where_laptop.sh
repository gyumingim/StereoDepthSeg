#!/usr/bin/env bash
# fixes 11-13 unit 15: u15_guard_where.py on the laptop PyTorch meshes (CPU, desktop): guard and guard+partner2 vs no guard
cd /home/system/scanner/StereoDepthSeg-fixes
L=scan3d/runs/laptop/20261112_121801_std60
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
for v in farguard farp2; do
  $PY scan3d/runs/fixes_20261112/u15_guard_where.py $L/fix_u15/mesh_sig $L/fix_u15far/mesh_farnone $L/fix_u15far/mesh_$v $L/fix_u15far/u15_where_$v.json
done
