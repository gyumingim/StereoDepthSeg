# PC check of the Phase 3 idle-spin variants (2026-10-25): --idle-spin 3 (pool WFE wait; on x86 = mode 2) and
# --idle-spin-ms W (spin only W ms before the expected frame). Real-time paced feed of the C4 stream (paced_feed.py of
# the 0.6.16 check) -> pose identity vs the 0.6.16 P0 reference + the bridge's idle-spin stats (spin / blocked time).
# Heat and the phone clock behaviour can only be measured on the phone (see README phone plan).
set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
OUT=/tmp/ovopt/spin; mkdir -p $OUT
FULL=$H/ov_bridge/build_ovopt_full/ov_bridge
CFG=$RD/cfg_identity/estimator_config.yaml
STREAM=/tmp/idlespin/pose_ab/stream.bin
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date
for n in ${RUNS:-S2 S3 S2W15 S3W15 S1W10}; do
  case $n in
    S2) X="--idle-spin 2" ;;
    S3) X="--idle-spin 3" ;;
    S2W15) X="--idle-spin 2 --idle-spin-ms 15" ;;
    S3W15) X="--idle-spin 3 --idle-spin-ms 15" ;;
    S1W10) X="--idle-spin 1 --idle-spin-ms 10" ;;
  esac
  echo "== $n $(date +%T) ($X)"
  python3 $H/runs/idle_spin_check_20261025/paced_feed.py $STREAM $OUT/$n.jsonl -- $FULL $ARGS $X --timing-log $OUT/$n.tlog 2> $OUT/$n.err
  grep -E "idle spin (end|:)" $OUT/$n.err | tail -2
done
echo "== identity vs 0.6.16 paced P0"
python3 $RD/cmp.py /tmp/idlespin/pose_ab/P0.jsonl $OUT/*.jsonl
date
