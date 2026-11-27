#!/bin/bash
# 2x2 BA x fps rehearsal + stage-B positive control. One GPU job at a time; waits for a free GPU.
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/bafps; G=$HOME/scanner/StereoDepthSeg-gsmap; N=$G/scan3d/gsmap/newrec
PY="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"; export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
L=$D/run.log; log() { echo "$(date '+%F %T') $*" >> $L; }
log START
[ -e $D/arms.json ] || { $PY $D/mk.py >> $L 2>&1 || { log "FAIL mk"; exit 1; }; }
[ -e $D/control_official.json ] || { $PY $N/postrec_check.py ffs --pairs $G/scan3d/runs/gsmap_20261125_121801_v2/ba_partners/ffs_pairs2/pairs --out $D/control_official.json > $D/control_official.log 2>&1; log "control official rc $?"; }
for arm in A60_BA A60_VIO A30_BA A30_VIO; do
  REC=$(python3 -c "import json; print(json.load(open('$D/arms.json'))['arms']['$arm']['rec'])"); VIO=$(python3 -c "import json; print(json.load(open('$D/arms.json'))['arms']['$arm']['vio'])")
  A="$D/$arm/run --vio $VIO --rec $REC --out $D/$arm/ffs --min-perp 40 --dir-deg 10 --n-pairs 4 --k 2 --profile new"
  if [ ! -e $D/$arm/ffs/s1/ffs_info.json ]; then
    while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -c .)" != 0 ]; do sleep 20; done
    log "ffs $arm"
    (cd $G && source $HOME/scanner/host.env 2>/dev/null; FFS_ROOT=$HOME/scanner/StereoDepthSeg nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore scan3d/gsmap/gsm_ffs_pairs2.py ffs $A > $D/$arm/ffs.log 2>&1) || { log "FAIL ffs $arm"; continue; }
  fi
  $PY $N/postrec_check.py ffs --pairs $D/$arm/ffs/pairs --out $D/$arm/check_b.json > $D/$arm/check_b.log 2>&1; log "check_b $arm rc $? $(python3 -c "import json; d=json.load(open('$D/$arm/check_b.json')); print('kf_med', round(d['kf_med_px_median'],3), 'over', round(d['share_kf_over_line'],3), 'n', d['n_kf_multi'])")"
done
log DONE
