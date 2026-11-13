#!/usr/bin/env bash
# fixes 11-13 unit 15: 121801 --far-pairs wide variants on TRT, sigma-scale-aware on (new default), one after another, then CPU evals.
cd /home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112
F="--sigma-scale-aware=on --far-pairs=wide --far-baseline=0.35"
bash u15_job.sh office farnone "$F --far-guard=none --far-repl-min-z=off"
bash u15_job.sh office farguard "$F --far-guard=lr+sat --far-repl-min-z=on"
bash u15_job.sh office farp2 "$F --far-guard=lr+sat --far-repl-min-z=on --far-partner2=on"
for v in farnone farguard farp2; do bash u15_eval.sh sig $v; done
bash u15_eval.sh farnone farguard
bash u15_eval.sh farnone farp2
echo "u15_far done"
