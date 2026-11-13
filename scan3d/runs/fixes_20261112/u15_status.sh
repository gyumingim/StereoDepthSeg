#!/usr/bin/env bash
# fixes 11-13 unit 15: show orchestrator processes + tails of the laptop job logs (read only)
L=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112/u15_laptop_logs
pgrep -af 'remote_laptop|u15_laptop' | grep -v pgrep | cut -c1-160
for f in board office; do echo "== $f"; tail -n ${N:-4} $L/$f.log 2>/dev/null; done
cat $L/launch.txt 2>/dev/null
