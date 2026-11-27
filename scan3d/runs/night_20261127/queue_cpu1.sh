#!/bin/bash
# CPU queue (one heavy job at a time): wait for plabels, then item-4 CPU part, then W-fill advice stats.
N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127
while pgrep -f "plabels.py" > /dev/null; do sleep 10; done
$N/log.sh "ptie labels done: $(tail -1 $N/ptie/plabels.log | cut -c1-200)"
bash $N/rehearsal30/run_cpu.sh
$N/log.sh "item 4 CPU part done (validate121801 + rehearsal c4f60/fps30 to stage A): $(tail -3 $N/rehearsal30/run_cpu.log | tr '\n' ' ')"
cd $N/wfill && OMP_NUM_THREADS=4 nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore advice.py > advice.log 2>&1
$N/log.sh "W-fill advice stats done (wfill/advice.json) rc $?"
