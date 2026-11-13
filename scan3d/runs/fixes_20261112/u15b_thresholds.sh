#!/usr/bin/env bash
# fixes 11-13 unit 15b: run u15b_thresholds.py on the laptop PyTorch far meshes (CPU, desktop, read only)
cd /home/system/scanner/StereoDepthSeg-fixes
L=scan3d/runs/laptop/20261112_121801_std60
/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/runs/fixes_20261112/u15b_thresholds.py $L/fix_u15b/u15b_thresholds.json \
  sig=$L/fix_u15/mesh_sig farnone_sigoff=$L/fix_u15b/mesh_farnone_sigoff farnone=$L/fix_u15far/mesh_farnone \
  farguard=$L/fix_u15far/mesh_farguard farp2dir=$L/fix_u15b/mesh_farp2dir farlr15=$L/fix_u15b/mesh_farlr15
