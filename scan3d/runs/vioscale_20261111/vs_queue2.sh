#!/usr/bin/env bash
# vioscale queue 2 (after vs_queue.sh). 11-12 fixes: (1) vs_batch out path was relative while lp_replay runs with cwd = livepose dir
# -> the replays of queue 1 landed in scan3d/runs/livepose_20261030/scan3d/runs/laptop/... on the laptop and no scoring happened:
# move them into place (replays are deterministic, not rerun). (2) vs_prov onset (adds S_* null floor, A_meas, GR_meas, I_*_post).
cd "$HOME/scanner/StereoDepthSeg-vioscale/scan3d"
export WT=$HOME/scanner/StereoDepthSeg-vioscale
D=scan3d/runs/vioscale_20261111
L=runs/vioscale_20261111/logs
until grep -q "165506 rc" $L/queue.log 2>/dev/null; do sleep 30; done
tools/remote_laptop.sh sh "for r in 20261111_202906_std60 20261024_165506_std60; do s=runs/livepose_20261030/scan3d/runs/laptop/\$r/vs1/runs; d=runs/laptop/\$r/vs1/runs; mkdir -p \$d; [ -d \$s ] && cp -n \$s/* \$d/ ; ls \$d | wc -l; done" >> $L/queue.log 2>&1
KF=$D/inputs/poses_vio_kf.jsonl.gz
C="cmd:\$P $D/vs_batch.py \$R \$O -j 4 --views \$O/views.pkl --kfmap $KF > \$O/batch2.log 2>&1; \$P $D/vs_rel.py \$O/runs $KF > \$O/rel.json"
tools/remote_laptop.sh job 202906 "$C" --tag vs1 > $L/job202906b.out 2>&1
echo "$(date +%T) 202906b rc $?" >> $L/queue.log
C="cmd:\$P $D/vs_batch.py \$R \$O -j 4 --rulers board,imu > \$O/batch2.log 2>&1"
tools/remote_laptop.sh job 165506 "$C" --tag vs1 > $L/job165506b.out 2>&1
echo "$(date +%T) 165506b rc $?" >> $L/queue.log
