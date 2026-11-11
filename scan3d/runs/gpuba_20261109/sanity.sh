#!/bin/bash
# gpuba 11-11: sanity run of the new kf_ba defaults through the pipeline entry point (remote_laptop.sh kfba step, WT = this worktree)
WT=/home/system/scanner/StereoDepthSeg-gpuba WT=/home/system/scanner/StereoDepthSeg-gpuba /home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh job 165506 kfba \
  --vio /home/system/scanner/StereoDepthSeg-featloop/scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz --tag sanity_defaults
echo "SANITY DONE rc $? $(date +%T)"
