B=~/scanner/StereoDepthSeg-ovgyro/scan3d/ov_bridge/build_ovgyro_full/ov_bridge
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
for c in 30 60 100 200; do
 ( OV_GK_CAP_PX=$c $B /tmp/ovopt/cfg/c4/estimator_config.yaml $BASE --remap /tmp/ovopt/remap_1920x1080.bin --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=5 < /tmp/ovopt/raw80.bin 2>&1 >/dev/null | grep "LK result" | sed "s/^/raw80 cap $c: /" ) &
 ( OV_GK_CAP_PX=$c $B ~/scanner/StereoDepthSeg-ovgyro/scan3d/runs/ovopt_20261025/cfg_identity/estimator_config.yaml $BASE --klt-stats 1 --ovopt gyro_klt=5 < /tmp/imuprio/h_L0.4nc.bin 2>&1 >/dev/null | grep "LK result" | sed "s/^/harness cap $c: /" ) &
done; wait
