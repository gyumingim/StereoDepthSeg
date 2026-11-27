#!/bin/bash
# append one line "- MM-DD HH:MM msg" to the night progress log
echo "- $(date '+%m-%d %H:%M') $*" >> "$HOME/scanner/StereoDepthSeg-gsmap/scan3d/runs/night_20261127/progress.md"
