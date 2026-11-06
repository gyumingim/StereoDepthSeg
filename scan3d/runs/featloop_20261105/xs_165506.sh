#!/bin/bash
# exploratory (README 6.3): XFeat with 3x3 heatmap-centroid sub-pixel keypoints (xfopt --subpix 1), then greedy tracks on the CPU
cd /home/system/scanner/StereoDepthSeg-featloop; export SCAN3D_GPU_POLL_S=3
P=$HOME/scanner/StereoDepthSeg/venv_ffs/bin/python
while [ $(awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo) -lt 10 ]; do sleep 30; done
$P scan3d/scanA/kf_ba.py /home/system/scanner/StereoDepthSeg-featloop/scan3d/recordings/hs_20261024_165506_std60 --vio scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz --out scan3d/runs/featloop_20261105/rec/165506/ba_Xs --gt imu_gt_spl --feat xfeat_lg --xf-subpix 1 --loop none --cache-root $HOME/scanner/featloop_ext/cache --seed 0 --keep-db > scan3d/runs/featloop_20261105/rec/165506/ba_Xs.log 2>&1; echo Xs rc $?
flock $HOME/scanner/featloop_ext/cpu.lock $P scan3d/scanA/kf_ba.py /home/system/scanner/StereoDepthSeg-featloop/scan3d/recordings/hs_20261024_165506_std60 --vio scan3d/runs/scanA_20261105/step1/B_o0.poses.jsonl.gz --out scan3d/runs/featloop_20261105/rec/165506/ba_Xsg --gt imu_gt_spl --feat xfeat_lg --xf-subpix 1 --loop none --cache-root $HOME/scanner/featloop_ext/cache --seed 0 --tracks greedy --db-from scan3d/runs/featloop_20261105/rec/165506/ba_Xs > scan3d/runs/featloop_20261105/rec/165506/ba_Xsg.log 2>&1; echo Xsg rc $?
echo XSDONE
