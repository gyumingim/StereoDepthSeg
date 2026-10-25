# PC identity check of the ovrobust flags (2026-10-25, branch ovrobust-20261025): health / recover must not change any
# pose while nothing is lost; init_still changes the init moment (shown, not identical). Same harness as
# runs/ovopt_20261025/ident_pc.sh (stream /tmp/idlespin/pose_ab/stream.bin 845 frames, cfg_identity, unpaced).
#   bash ident_pc.sh     -> /tmp/ovrobust/ident/<run>.{jsonl,err,hlog}
set -u
H=~/scanner/StereoDepthSeg-ovrobust/scan3d
RD=$H/runs/ovrobust_20261025
OUT=/tmp/ovrobust/ident; mkdir -p $OUT
STREAM=/tmp/idlespin/pose_ab/stream.bin
CFG=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
BP=/tmp/ovopt/ident/BP.jsonl     # ovopt run of the main checkout build_par (= 0.6.17 bridge), hash d7958f8247fbc180
FULL=$H/ov_bridge/build_ovrobust_full/ov_bridge
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date; sha256sum $FULL
run() { local n=$1; shift; $FULL $ARGS "$@" < $STREAM > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
run R0
run RH --health 1 --health-log $OUT/RH.hlog
run RHW --health 1 --health-log $OUT/RHW.hlog --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol
run RW --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol
run RREC --health 1 --health-log $OUT/RREC.hlog --recover 15
run RSI --init-still 1 --health 1 --health-log $OUT/RSI.hlog
echo "== vs BP"; python3 $RD/cmp.py $BP $OUT/R0.jsonl $OUT/RH.jsonl $OUT/RREC.jsonl $OUT/RSI.jsonl
echo "== RHW vs RW (near-exact set, health on/off)"; python3 $RD/cmp.py $OUT/RW.jsonl $OUT/RHW.jsonl
grep -h "health:\|recover" $OUT/*.err
for n in R0 RSI; do echo "$n first tracking: $(grep -m1 '"tracking"' $OUT/$n.jsonl | python3 -c 'import json,sys; print(json.load(sys.stdin)["t"])')"; done
date
