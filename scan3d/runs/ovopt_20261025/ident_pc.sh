# PC identity check of ov_bridge --ovopt (2026-10-25, branch ovopt-20261025).
# Input = the captured C4 bridge input stream of hs_20261024_165506_std60 used by the 0.6.16 / 0.6.17 checks
# (/tmp/idlespin/pose_ab/stream.bin, 845 frames, 960x540), config cfg_identity/ (= runs/vio_v3w4kbl8 of those checks).
# Unpaced feed (stdin from the file): the record order and so every VIO result do not depend on timing (0.6.16 check:
# paced == fast feed). Compared: all pose-line fields except ms / remap_ms / wait_ms (cmp.py).
#   bash ident_pc.sh [RUNS...]      outputs /tmp/ovopt/ident/<run>.{jsonl,err}, log to stdout
set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
OUT=/tmp/ovopt/ident; mkdir -p $OUT
STREAM=${STREAM:-/tmp/idlespin/pose_ab/stream.bin}
CFG=$RD/cfg_identity/estimator_config.yaml
BP=~/scanner/StereoDepthSeg/scan3d/ov_bridge/build_par/ov_bridge     # main checkout build (= 0.6.17 bridge), run read-only
DEF=$H/ov_bridge/build_ovopt_def/ov_bridge                            # this branch, default PC build (libov_msckf_lib.so)
FULL=$H/ov_bridge/build_ovopt_full/ov_bridge                          # this branch, -DOV_FULL_SOURCE=ON
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date
echo "stream $STREAM ($(stat -c %s $STREAM) bytes) cfg $CFG"
sha256sum $BP $DEF $FULL | sed 's/  / /'
run() {  # name binary extra...
  local n=$1 b=$2; shift 2
  local t0=$(date +%s.%N)
  $b $ARGS "$@" < $STREAM > $OUT/$n.jsonl 2> $OUT/$n.err
  local rc=$?
  echo "== $n rc $rc $(echo "$(date +%s.%N) - $t0" | bc | cut -c1-6) s poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"
}
for r in ${@:-BP DEF FULL0 skip_retri par_tri par_jac par_ma cov_move all_exact ekf_gemm qr_compress}; do
  case $r in
    BP) run BP $BP ;;
    DEF) run DEF $DEF ;;
    FULL0) run FULL0 $FULL ;;
    *) run $r $FULL --ovopt $r ;;
  esac
done
echo "== identity vs BP (main checkout build_par = 0.6.17 bridge)"
ls $OUT/*.jsonl | grep -v "/BP.jsonl" | xargs python3 $RD/cmp.py $OUT/BP.jsonl
echo "== BP vs 0.6.16 paced reference (idle_spin_check P0)"
python3 $RD/cmp.py /tmp/idlespin/pose_ab/P0.jsonl $OUT/BP.jsonl
date
