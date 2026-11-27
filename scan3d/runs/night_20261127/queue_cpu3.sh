#!/bin/bash
N=$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127
while pgrep -f "queue_cpu2.sh" > /dev/null; do sleep 10; done
bash $N/rehearsal30/run_cpu2.sh
$N/log.sh "item 4 CPU round 2 done: $(tail -4 $N/rehearsal30/run_cpu.log | tr '\n' ' ')"
