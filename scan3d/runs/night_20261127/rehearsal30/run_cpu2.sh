#!/bin/bash
# night 11-27 item 4 CPU part, round 2: validate (tag bug fixed) + VIO arms fps30 (own cfg copy with the 640x360 remap) and c4 (= c4f60 at 30 Hz)
D=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/rehearsal30; N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/newrec
REC=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127/rec_hs_20261112_121801_std30
PY="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"; export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
L=$D/run_cpu.log; log() { echo "$(date '+%F %T') $*" >> $L; }
log START round2
[ -e $D/validate121801.json ] || { $PY $N/postrec_check.py validate121801 --out $D/validate121801.json > $D/validate121801.log 2>&1; log "validate rc $?"; }
bash $N/newrec.sh $REC $D/fps30 --cfg $D/cfg_fps30 --stop-after check > $D/newrec_fps30.out 2>&1; log "newrec fps30 rc $?"
bash $N/newrec.sh $REC $D/c4 --cfg c4 --stop-after check > $D/newrec_c4.out 2>&1; log "newrec c4 rc $?"
log DONE round2
