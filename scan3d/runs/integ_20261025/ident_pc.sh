# integ-20261025 PC identity: integ build (all new flags off, and read-only flags on) vs the ovopt-20261025 base build
# (35cbf60 full-source Release, ~/scanner/StereoDepthSeg-ovopt/scan3d/ov_bridge/build_ovopt_full/ov_bridge), 17 digits.
# Inputs: H = C4 harness stream hs_20261024_165506_std60 with IMU lead 0.4 s, unchunked (imuprio /tmp/imuprio/h_L0.4nc.bin,
# 845 frames, cfg_identity), R = raw80 phone replay stream (hs_20261021_213341, 2327 frames, /tmp/ovopt/cfg/c4 + remap).
#   bash ident_pc.sh [JOBS...]   outputs /tmp/integ/ident/<job>.{jsonl,err}
set -u
H=~/scanner/StereoDepthSeg-integ/scan3d
RD=$H/runs/integ_20261025
OUT=/tmp/integ/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-ovopt/scan3d/ov_bridge/build_ovopt_full/ov_bridge
MINE=${MINE:-$H/ov_bridge/build_integ_full/ov_bridge}
HC=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
W3="all_exact,ekf_gemm,qr_compress,ekf_chol"
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $REF $MINE
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
job() {
  case $1 in
    h_ref) run $1 $REF $HC $HS ;;
    h_def) run $1 $MINE $HC $HS ;;
    h_ex_ref) run $1 $REF $HC $HS --ovopt all_exact ;;
    h_ex) run $1 $MINE $HC $HS --ovopt all_exact ;;
    h_pt_ref) run $1 $REF $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 ;;
    h_pt_viop) run $1 $MINE $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14699 ;;
    h_health) run $1 $MINE $HC $HS --health 1 ;;
    h_envpyr) OV_KLT_PYR=3 run $1 $MINE $HC $HS ;;
    r_ref) run $1 $REF $C4 $R80 $RM ;;
    r_def) run $1 $MINE $C4 $R80 $RM ;;
    r_w3_ref) run $1 $REF $C4 $R80 $RM --ovopt $W3 --idle-spin 2 --idle-spin-ms 20 ;;
    r_prof_nodl) run $1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 ;;
    r_prof) run $1 $MINE $C4 $R80 $RM --profile rec20261025 ;;
  esac
}
JOBS=${@:-h_ref h_def h_ex_ref h_ex h_pt_ref h_pt_viop h_health h_envpyr r_ref r_def r_w3_ref r_prof_nodl r_prof}
P=${P:-4}; n=0
for j in $JOBS; do job $j & n=$((n+1)); if [ $((n % P)) -eq 0 ]; then wait; fi; done; wait
C=$RD/cmp2.py
echo "== harness: integ vs ovopt base"
python3 $C $OUT/h_ref.jsonl $OUT/h_def.jsonl $OUT/h_envpyr.jsonl
python3 $C $OUT/h_ex_ref.jsonl $OUT/h_ex.jsonl
python3 $C $OUT/h_pt_ref.jsonl $OUT/h_pt_viop.jsonl
python3 $C --ex h $OUT/h_ref.jsonl $OUT/h_health.jsonl
echo "== raw80: integ vs ovopt base"
python3 $C $OUT/r_ref.jsonl $OUT/r_def.jsonl
python3 $C --ex h $OUT/r_w3_ref.jsonl $OUT/r_prof_nodl.jsonl $OUT/r_prof.jsonl
grep -h "profile\|KLT_PYR\|drop-late\|viop:\|pred thread:\|health" $OUT/*.err | sort | uniq -c | head -30
date
