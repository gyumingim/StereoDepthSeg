#!/usr/bin/env bash
# Corner-rms reference for docs/lens_switch_20261023.md section 7: re-fit the camera-only spline (knot 0.0125) on the OLD-K
# imu_gt_rs with the SAME spline code as the new GT, into imu_gt_spl_oldK_refit (scoring GT imu_gt_spl is not touched).
# Sequential, pinned to cores 12-15, 4 threads.
set -u
cd "$(dirname "$0")/.."
PY=${PY:-../venv_ffs/bin/python}
export PYTHONIOENCODING=utf-8 OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENCV_FOR_THREADS_NUM=4
OD=runs/lens_switch/gt
for R in "$@"; do
  N=$(basename ${R%/})
  taskset -c 12-15 $PY -u imu_learn/spline_gt.py $R --gt-dir imu_gt_rs --out-dir imu_gt_spl_oldK_refit --knot 0.0125 \
    > $OD/${N}_spline_oldK_refit.log 2>&1 && echo "$(date +%T) $N refit ok" || echo "$(date +%T) $N refit FAILED"
done
echo "REFIT ALL DONE $(date +%T)"
