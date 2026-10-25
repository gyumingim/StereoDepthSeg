# integ2-20261026 PC identity: integ2 build (flags off / old profile) vs integ-20261025 build (14de3b8, build_integ_full).
# H = C4 harness stream (845 frames, cfg_identity), R = raw80 (2327 frames, c4 + remap), pose-precision 17, single run each
# (deterministic, single-threaded OpenVINS per run). Compare = integ cmp2.py (pose lines without timing keys).
set -u
OUT=/tmp/integ2/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge
MINE=~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge
HC=~/scanner/StereoDepthSeg-integ/scan3d/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
OLDOPT="--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol"
date; sha256sum $REF $MINE $HS $R80 $C4 $HC
cd ~/scanner/StereoDepthSeg-integ2 && git log --oneline -1 && git status --short scan3d/ov_bridge | grep -v build_ | head
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  nice -n 5 $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
run h_ref $REF $HC $HS &
run h_mine $MINE $HC $HS &
run hv_ref $REF $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14781 &
run hv_mine0 $MINE $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14782 &
run hv_mine1 $MINE $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14783 --viop-hb-src 1 &
wait
run r_ref $REF $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine_old $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 $OLDOPT &
run r_mine_new $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run rp_ref $REF $C4 $R80 $RM &
run rp_mine $MINE $C4 $R80 $RM &
run r60_ref $REF $C4 $R80 $RM --profile rec60 --drop-late 0 &
wait
run r60_mine_old $MINE $C4 $R80 $RM --profile rec60 --drop-late 0 $OLDOPT &
run r_mine_new_rep $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine_dlm0 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --drop-late-mode 0 $OLDOPT &
wait
C=~/scanner/StereoDepthSeg-integ/scan3d/runs/integ_20261025/cmp2.py
for p in "h_ref h_mine" "hv_ref hv_mine0" "hv_ref hv_mine1" "r_ref r_mine_old" "rp_ref rp_mine" "r60_ref r60_mine_old" "r_ref r_mine_dlm0" "r_ref r_mine_new" "r_mine_new r_mine_new_rep"; do
  set -- $p; echo "-- $1 vs $2"; python3 $C $OUT/$1.jsonl $OUT/$2.jsonl; done
grep -h "ovopt:" $OUT/r_mine_new.err $OUT/r_mine_old.err $OUT/r_ref.err | head -3
grep -h "viop heartbeat\|viop: udp" $OUT/hv_mine1.err | head -3
echo "-- refusal checks"
$MINE $HC --stdio --ovopt gyro_klt=5 --pipe 1 < /dev/null; echo "rc $?"
$MINE $HC --stdio --pipe 1 < /dev/null; echo "rc $?"
$MINE $HC --stdio --pipe 0 --ovopt gyro_klt=5 < /dev/null > /dev/null 2> $OUT/g5.err; echo "rc gyro_klt=5 alone (empty input) $?"; grep -m2 'ovopt' $OUT/g5.err
date
