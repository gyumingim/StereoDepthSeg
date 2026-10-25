# integ2 gyro-on-profile (2026-10-26 morning): PC identity of the new build (profile + gyro_klt=5) vs
#   REF  = integ-20261025 build (14de3b8) and OLD = integ2 0.6.26 build (a0311486, frozen copy /tmp/integ2g/ov_bridge_pc_0626).
# Same inputs / compare script as ident/integ2_ident.sh. pose-precision 17, single run each (deterministic).
set -u
OUT=/tmp/integ2g/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge
OLD=/tmp/integ2g/ov_bridge_pc_0626
MINE=~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge
HC=~/scanner/StereoDepthSeg-integ/scan3d/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
OLDOPT="--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol"
NOGY="--ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,sym_inplace,ekf_sparse"
date; sha256sum $REF $OLD $MINE
cd ~/scanner/StereoDepthSeg-integ2 && git log --oneline -1 && git status --short scan3d/ov_bridge phone_stereo | grep -v build_ | head
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  nice -n 5 $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
run h_ref $REF $HC $HS &
run h_mine $MINE $HC $HS &
run rp_ref $REF $C4 $R80 $RM &
run rp_mine $MINE $C4 $R80 $RM &
run r_ref $REF $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine_old $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 $OLDOPT &
wait
run r_old_new $OLD $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine_nogy $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 $NOGY &
run r60_old_new $OLD $C4 $R80 $RM --profile rec60 --drop-late 0 &
run r60_mine_nogy $MINE $C4 $R80 $RM --profile rec60 --drop-late 0 $NOGY &
run r_mine_gyro $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine_gyro_rep $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
wait
run r_mine_gyro_explicit $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 $NOGY,gyro_klt=5 &
run r_old_gyro_explicit $OLD $C4 $R80 $RM --profile rec20261025 --drop-late 0 $NOGY,gyro_klt=5 &
run r60_mine_gyro $MINE $C4 $R80 $RM --profile rec60 --drop-late 0 &
wait
C=~/scanner/StereoDepthSeg-integ/scan3d/runs/integ_20261025/cmp2.py
for p in "h_ref h_mine" "rp_ref rp_mine" "r_ref r_mine_old" "r_old_new r_mine_nogy" "r60_old_new r60_mine_nogy" "r_mine_gyro r_mine_gyro_rep" "r_mine_gyro r_mine_gyro_explicit" "r_mine_gyro r_old_gyro_explicit" "r_mine_nogy r_mine_gyro" "r60_mine_nogy r60_mine_gyro"; do
  set -- $p; echo "-- $1 vs $2"; python3 $C $OUT/$1.jsonl $OUT/$2.jsonl; done
grep -h "ovopt:" $OUT/r_mine_gyro.err $OUT/r60_mine_gyro.err $OUT/r_mine_nogy.err | head -3
grep -h "gyro_klt mode" $OUT/r_mine_gyro.err $OUT/r60_mine_gyro.err | head -4
echo "-- refusal checks"
$MINE $HC --stdio --profile rec20261025 --pipe 1 < /dev/null; echo "rc profile+pipe1 $?"
$MINE $HC --stdio --profile rec20261025 $NOGY --pipe 1 < /dev/null; echo "rc profile+nogyro+pipe1 $?"
$MINE $HC --stdio --pipe 1 < /dev/null; echo "rc pipe1 alone $?"
date
