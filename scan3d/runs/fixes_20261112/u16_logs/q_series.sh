# runs the u16 full tests one after another (each = one laptop job, waits for the lock)
Q=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/fixes_20261112/u16_logs
bash $Q/q_full.sh shift fix_u16_shift '--align-args "--test-offset-m 0.2"' > $Q/job_shift.log 2>&1
bash $Q/q_full.sh u11 fix_u16_u11b '' > $Q/job_u11b.log 2>&1
bash $Q/q_full.sh x105 fix_u16_x105 '' > $Q/job_x105.log 2>&1
bash $Q/q_full.sh u10 fix_u16_u10 '' > $Q/job_u10.log 2>&1
bash $Q/q_full.sh neg fix_u16_neg '' > $Q/job_neg.log 2>&1
bash $Q/q_full.sh x095 fix_u16_x095 '' > $Q/job_x095.log 2>&1
echo "SERIES DONE $(date +%T)"
