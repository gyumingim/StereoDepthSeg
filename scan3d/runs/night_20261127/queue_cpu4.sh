#!/bin/bash
N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127
while pgrep -f "queue_cpu3.sh" > /dev/null; do sleep 10; done
cd $N/unseen && mv checks.log checks_fail1.log 2>/dev/null; OMP_NUM_THREADS=8 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore checks.py > checks.log 2>&1
$N/log.sh "item 2 checks rerun rc $? (unseen/checks.json; first try failed: GSMAP_RUN env needed by gsm_ffsgate import)"
cd $N/rehearsal30 && $HOME/scanner/gsmap_env/venv/bin/python vio_choice.py > vio_choice.log 2>&1
$N/log.sh "VIO choice rc $?: $(python3 -c "import json; d=json.load(open('$N/rehearsal30/vio_choice.json')); print('pick', d.get('pick'), {k: {x: v.get(x) for x in ('tracked_frac','ate_med_cm','sim3_scale','n_kf','strict_ge2_pct','strict_zero_pct')} for k, v in d['arms'].items()})")"
