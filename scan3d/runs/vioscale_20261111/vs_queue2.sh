#!/usr/bin/env bash
# vioscale queue 2 (after vs_queue.sh): 202906 again with the fixed onset (vs_prov 11-12): adds S_* null floor, A_meas, GR_meas,
# I_*_post; existing conditions are skipped (laptop keeps runs/laptop/.../vs1). Waits for queue 1 to finish.
cd "$HOME/scanner/StereoDepthSeg-vioscale/scan3d"
export WT=$HOME/scanner/StereoDepthSeg-vioscale
D=scan3d/runs/vioscale_20261111
L=runs/vioscale_20261111/logs
until grep -q "165506 rc" $L/queue.log 2>/dev/null; do sleep 30; done
KF=$D/inputs/poses_vio_kf.jsonl.gz
C="cmd:\$P $D/vs_batch.py \$R \$O -j 4 --views \$O/views.pkl --kfmap $KF > \$O/batch2.log 2>&1"
tools/remote_laptop.sh job 202906 "$C" --tag vs1 > $L/job202906b.out 2>&1
echo "$(date +%T) 202906b rc $?" >> $L/queue.log
