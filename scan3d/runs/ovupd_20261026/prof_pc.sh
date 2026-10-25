# ovupd 2026-10-26: PC stage timing (raw80 + phone remap, unpaced, --threads 2, sequential A/B interleaved), --prof-log.
# PC is shared with other agents -> timing is indicative only (phone replay is the ruler).
set -u
MINE=$HOME/scanner/StereoDepthSeg-ovupd/scan3d/ov_bridge/build_ovupd_full/ov_bridge
OUT=/tmp/ovupd/prof; mkdir -p $OUT
W3=all_exact,ekf_gemm,qr_compress,ekf_chol
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --remap /tmp/ovopt/remap_1920x1080.bin"
date; sha256sum $MINE; uptime
for rep in 1 2; do for cfg in c4 c4s75 c4s100; do for v in A B; do
  o=$W3; [ $v = B ] && o=$W3,sym_inplace,ekf_sparse
  n=${cfg}_${v}_r$rep
  $MINE /tmp/ovupd/cfg/$cfg/estimator_config.yaml $BASE --ovopt $o --prof-log $OUT/$n.prof.csv < /tmp/ovopt/raw80.bin > $OUT/$n.jsonl 2> $OUT/$n.err
  echo "== $n rc $? $(uptime | sed 's/.*load/load/')"
done; done; done
date
