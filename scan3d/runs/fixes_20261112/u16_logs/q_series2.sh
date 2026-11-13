Q=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112/u16_logs
while ! grep -q "SERIES DONE" $Q/series.log; do sleep 30; done
bash $Q/q_full.sh shift fix_u16_shift1m '--align-args "--test-offset-m 1.0"' > $Q/job_shift1m.log 2>&1
echo "SERIES2 DONE $(date +%T)"
