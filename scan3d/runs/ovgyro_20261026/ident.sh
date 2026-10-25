# ovgyro-20261026 PC identity: this branch (gyro_klt off, --klt-stats on/off) vs integ-20261025 build (14de3b8), 17 digits.
set -u
H=~/scanner/StereoDepthSeg-ovgyro/scan3d
RD=$H/runs/ovgyro_20261026; mkdir -p $RD
OUT=/tmp/ovgyro/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge
MINE=$H/ov_bridge/build_ovgyro_full/ov_bridge
HC=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $REF $MINE; ls -la $HS $R80 $C4
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
job() {
  case $1 in
    h_ref) run $1 $REF $HC $HS ;;
    h_def) run $1 $MINE $HC $HS ;;
    h_g0) run $1 $MINE $HC $HS --ovopt gyro_klt=0 --klt-stats 1 ;;
    h_g1) run $1 $MINE $HC $HS --ovopt gyro_klt=1 --klt-stats 1 ;;
    h_g2) run $1 $MINE $HC $HS --ovopt gyro_klt=2 --klt-stats 1 ;;
    h_g3) run $1 $MINE $HC $HS --ovopt gyro_klt=3 --klt-stats 1 ;;
    h_g4) run $1 $MINE $HC $HS --ovopt gyro_klt=4 --klt-stats 1 ;;
    r_g3) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=3 ;;
    r_g4) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=4 ;;
    h_g5) run $1 $MINE $HC $HS --ovopt gyro_klt=5 --klt-stats 1 ;;
    r_g5) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=5 ;;
    r_ref) run $1 $REF $C4 $R80 $RM --profile rec20261025 --drop-late 0 ;;
    r_prof) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 ;;
    r_ks) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 ;;
    r_g1) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=1 ;;
    r_g2) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --klt-stats 1 --ovopt all_exact,ekf_gemm,qr_compress,ekf_chol,gyro_klt=2 ;;
  esac
}
JOBS=${@:-h_ref h_def h_g0 h_g1 h_g2 h_g3 h_g4 h_g5 r_ref r_prof r_ks r_g1 r_g2 r_g3 r_g4 r_g5}
P=${P:-5}; n=0
for j in $JOBS; do job $j & n=$((n+1)); if [ $((n % P)) -eq 0 ]; then wait; fi; done; wait
C=$H/runs/integ_20261025/cmp2.py
echo "== harness"
python3 $C $OUT/h_ref.jsonl $OUT/h_def.jsonl
python3 $C --ex klt $OUT/h_ref.jsonl $OUT/h_g0.jsonl $OUT/h_g1.jsonl $OUT/h_g2.jsonl $OUT/h_g3.jsonl $OUT/h_g4.jsonl $OUT/h_g5.jsonl
echo "== raw80 (profile rec20261025, drop-late 0)"
python3 $C --ex h $OUT/r_ref.jsonl $OUT/r_prof.jsonl
python3 $C --ex h,klt $OUT/r_ref.jsonl $OUT/r_ks.jsonl $OUT/r_g1.jsonl $OUT/r_g2.jsonl $OUT/r_g3.jsonl $OUT/r_g4.jsonl $OUT/r_g5.jsonl
grep -H "gyro_klt mode" $OUT/*.err
date
