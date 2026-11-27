#!/bin/bash
# generic partner-frame BA (newrec/ba_partners_generic.py) on the same 20 kf, 60 and 30 fps -> FFS with BA poses -> stage B.
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/bafps; G=$HOME/scanner/StereoDepthSeg-gsmap; N=$G/scan3d/gsmap/newrec
PY="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"; FF="nice -n 19 $HOME/scanner/StereoDepthSeg/venv_ffs/bin/python -W ignore"
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8 PYTHONUNBUFFERED=1
L=$D/run.log; log() { echo "$(date '+%F %T') $*" >> $L; }
for r in 60 30; do
  arm=A${r}_VIO; O=$D/BAG$r; mkdir -p $O
  REC=$(python3 -c "import json; print(json.load(open('$D/arms.json'))['arms']['$arm']['rec'])"); VIO=$(python3 -c "import json; print(json.load(open('$D/arms.json'))['arms']['$arm']['vio'])")
  [ -e $O/plan/plan.json ] || (cd $G && $PY scan3d/gsmap/gsm_ffs_pairs2.py plan $D/$arm/run --vio $VIO --rec $REC --out $O/plan --min-perp 40 --dir-deg 10 --n-pairs 4 --k 2 --profile new > $O/plan.log 2>&1) || { log "FAIL BAG$r plan"; continue; }
  X="--run $D/$arm/run --plan $O/plan/plan.json --rec $REC --vio $VIO --out $O"
  for st in frames ba post ffs; do
    case $st in frames) [ -e $O/manifest.json ] && continue;; ba) [ -e $O/ba/ba_state.npz ] && continue;; post) [ -e $O/poses_all.npz ] && continue;; ffs) [ -e $O/ffs/s1/ffs_info.json ] && continue;; esac
    while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -c .)" != 0 ]; do sleep 20; done
    log "BAG$r $st"
    (cd $G && FFS_ROOT=$HOME/scanner/StereoDepthSeg source $HOME/scanner/host.env 2>/dev/null; FFS_ROOT=$HOME/scanner/StereoDepthSeg $FF $N/ba_partners_generic.py $st $X > $O/$st.log 2>&1) || { log "FAIL BAG$r $st"; continue 2; }
  done
  $PY $N/postrec_check.py ffs --pairs $O/ffs/pairs --out $O/check_b.json > $O/check_b.log 2>&1; log "check_b BAG$r rc $? $(python3 -c "import json; d=json.load(open('$O/check_b.json')); print('kf_med', round(d['kf_med_px_median'],3), 'over', round(d['share_kf_over_line'],3), 'n', d['n_kf_multi'])") post $(cat $O/post.json | tr -d '\n' | cut -c1-300)"
done
log DONE BAG
