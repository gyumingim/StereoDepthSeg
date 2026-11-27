#!/bin/bash
N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127
while pgrep -f "queue_cpu1.sh" > /dev/null; do sleep 10; done
cd $N/unseen && OMP_NUM_THREADS=8 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore checks.py > checks.log 2>&1
$N/log.sh "item 2 checks done rc $? (unseen/checks.json)"
