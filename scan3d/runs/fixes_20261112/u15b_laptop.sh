#!/usr/bin/env bash
# fixes 11-13 unit 15b (audit u15_verify.md): 3 laptop PyTorch far runs on 121801 in one laptop job (evict 60 000 like fix_u15far),
# results fix_u15b, then CPU evals here vs fix_u15/mesh_sig and fix_u15far/mesh_farnone. Checks the laptop is free first.
cd /home/system/scanner/StereoDepthSeg-fixes
st=$(bash scan3d/tools/remote_laptop.sh status 2>&1)
echo "$st" | head -3
VARIANTS="farnone_sigoff farp2dir farlr15" OFFICE_EXTRA="--tsdf-evict-gpu-blocks 60000" OTAG=fix_u15b \
  bash scan3d/runs/fixes_20261112/u15_laptop_launch.sh office
L=scan3d/runs/laptop/20261112_121801_std60
REC=$L/fix_u8/scan/red/hs_20261112_121801_std60_kf
PY=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
E=$L/fix_u15b
ev() { $PY scan3d/runs/fixes_20261112/far_eval.py "$1" "$2" "$REC" "$3" > "$3_eval.log" 2>&1; echo "eval $3 rc $?"; }
[ -f $E/mesh_farnone_sigoff/summary.json ] && ev $L/fix_u15/mesh_base $E/mesh_farnone_sigoff $E/u15_farnone_sigoff_vs_base
for v in farp2dir farlr15; do [ -f $E/mesh_$v/summary.json ] && { ev $L/fix_u15/mesh_sig $E/mesh_$v $E/u15_${v}_vs_sig; ev $L/fix_u15far/mesh_farnone $E/mesh_$v $E/u15_${v}_vs_farnone; }; done
for v in farp2dir farlr15; do [ -f $E/mesh_$v/summary.json ] && \
  $PY scan3d/runs/fixes_20261112/u15_guard_where.py $L/fix_u15/mesh_sig $L/fix_u15far/mesh_farnone $E/mesh_$v $E/u15_where_$v.json > /dev/null; done
echo "u15b done"
