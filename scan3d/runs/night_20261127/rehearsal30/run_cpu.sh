#!/bin/bash
# night 11-27 item 4 (CPU part): validate the check on 121801, then the 30 fps rehearsal up to stage A with both VIO configs.
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/rehearsal30; N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/newrec
REC=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/rec_hs_20261112_121801_std30
PY="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"; export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
L=$D/run_cpu.log; log() { echo "$(date '+%F %T') $*" >> $L; }
log START
[ -e $D/validate121801.json ] || { $PY $N/postrec_check.py validate121801 --out $D/validate121801.json > $D/validate121801.log 2>&1; log "validate rc $?"; }
for c in c4f60 fps30; do
  bash $N/newrec.sh $REC $D/$c --cfg $c --stop-after check > $D/newrec_$c.out 2>&1; log "newrec $c rc $?"
done
log DONE
