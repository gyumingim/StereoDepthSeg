#!/usr/bin/env bash
# vioscale queue (run on THIS PC under setsid): 202906 all conditions (+ repro of the redscreen poses), then 165506 (board guard).
cd "$HOME/scanner/StereoDepthSeg-vioscale/scan3d"
export WT=$HOME/scanner/StereoDepthSeg-vioscale
D=scan3d/runs/vioscale_20261111
L=runs/vioscale_20261111/logs; mkdir -p $L
LENS=scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json
KF=$D/inputs/poses_vio_kf.jsonl.gz
if [ "${SKIP1:-0}" != 1 ]; then
C1="cmd:\$P $D/vs_score.py views $D/inputs/frames_kf $LENS \$O/views.pkl > \$O/views.log 2>&1; "
C1+="for f in B_o0.poses.jsonl.gz poses_ba.jsonl.gz poses_vio_kf.jsonl.gz; do \$P $D/vs_score.py red \$O/views.pkl $KF $D/inputs/\$f; done > \$O/repro.log 2>&1; "
C1+="\$P $D/vs_batch.py \$R \$O -j 4 --views \$O/views.pkl --kfmap $KF > \$O/batch.log 2>&1"
tools/remote_laptop.sh job 202906 "$C1" --send $D/inputs --tag vs1 > $L/job202906.out 2>&1
echo "$(date +%T) 202906 rc $?" >> $L/queue.log
fi
C2="cmd:\$P $D/vs_batch.py \$R \$O -j 4 --rulers board,imu > \$O/batch.log 2>&1"
tools/remote_laptop.sh job 165506 "$C2" --tag vs1 > $L/job165506.out 2>&1
echo "$(date +%T) 165506 rc $?" >> $L/queue.log
