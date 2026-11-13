#!/usr/bin/env bash
# fixes 11-13 unit 15: 121801 --far-pairs wide variants on TRT, sigma-scale-aware on (new default), one after another, then CPU evals.
cd /home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112
F="--sigma-scale-aware=on --far-pairs=wide --far-baseline=0.35"
# --far-partner2=off explicit (audit 11-13: the runs had it off only by the then default; the default changed later)
bash u15_job.sh office farnone "$F --far-guard=none --far-repl-min-z=off --far-partner2=off"
bash u15_job.sh office farguard "$F --far-guard=lr+sat --far-lr-tol=1.0 --far-repl-min-z=on --far-partner2=off"
# farp2 never finished on the desktop; with the current code it would run the unit 15b partner rule
bash u15_job.sh office farp2 "$F --far-guard=lr+sat --far-lr-tol=1.0 --far-repl-min-z=on --far-partner2=on"
for v in farnone farguard farp2; do bash u15_eval.sh sig $v; done
bash u15_eval.sh farnone farguard
bash u15_eval.sh farnone farp2
echo "u15_far done"
