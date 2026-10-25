# ovupd 2026-10-26: PC identity / near-exact check of the new EKFUpdate options (ekf_sparse, sym_inplace).
# REF = integ-20261025 full-source build (14de3b8 = this branch's base), MINE = this branch's full-source build.
# Inputs = integ ident_pc.sh: H = C4 harness (845 frames, cfg_identity), R = raw80 phone replay stream (2327 frames) with
# configs c4 / c4s75 / c4s100 (= /tmp/ovopt/cfg/c4 with max_slam 75 / 100; own copies so ov_timing.txt lands in /tmp/ovupd).
#   bash ident_pc.sh [JOBS...]   outputs /tmp/ovupd/ident/<job>.{jsonl,err}
set -u
H=~/scanner/StereoDepthSeg-ovupd/scan3d
RD=$H/runs/ovupd_20261026
OUT=/tmp/ovupd/ident; mkdir -p $OUT
REF=${REF:-$HOME/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge}
MINE=${MINE:-$H/ov_bridge/build_ovupd_full/ov_bridge}
HC=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
W3="all_exact,ekf_gemm,qr_compress,ekf_chol"
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
mkcfg() { local n=$1 s=$2; local d=/tmp/ovupd/cfg/$n; mkdir -p $d; cp /tmp/ovopt/cfg/c4/kalibr_*.yaml $d/
  sed -e "s#^max_slam: 50#max_slam: $s#" -e "s#/tmp/ovopt/cfg/c4/#$d/#" /tmp/ovopt/cfg/c4/estimator_config.yaml > $d/estimator_config.yaml; }
mkcfg c4 50; mkcfg c4s75 75; mkcfg c4s100 100
date; sha256sum $REF $MINE
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
C() { echo /tmp/ovupd/cfg/$1/estimator_config.yaml; }
job() {
  case $1 in
    h_ref) run $1 $REF $HC $HS ;;
    h_def) run $1 $MINE $HC $HS ;;
    h_sym) run $1 $MINE $HC $HS --ovopt sym_inplace ;;
    h_w3_ref) run $1 $REF $HC $HS --ovopt $W3 ;;
    h_w3) run $1 $MINE $HC $HS --ovopt $W3 ;;
    h_w3sym) run $1 $MINE $HC $HS --ovopt $W3,sym_inplace ;;
    h_w3sp) run $1 $MINE $HC $HS --ovopt $W3,sym_inplace,ekf_sparse ;;
    r_ref) run $1 $REF $(C c4) $R80 $RM ;;
    r_def) run $1 $MINE $(C c4) $R80 $RM ;;
    r_sym) run $1 $MINE $(C c4) $R80 $RM --ovopt sym_inplace ;;
    r_w3_ref) run $1 $REF $(C c4) $R80 $RM --ovopt $W3 ;;
    r_w3) run $1 $MINE $(C c4) $R80 $RM --ovopt $W3 ;;
    r_w3sym) run $1 $MINE $(C c4) $R80 $RM --ovopt $W3,sym_inplace ;;
    r_w3sp) run $1 $MINE $(C c4) $R80 $RM --ovopt $W3,sym_inplace,ekf_sparse ;;
    r_sp_only) run $1 $MINE $(C c4) $R80 $RM --ovopt ekf_sparse ;;
    s75_w3) run $1 $MINE $(C c4s75) $R80 $RM --ovopt $W3 ;;
    s75_w3sp) run $1 $MINE $(C c4s75) $R80 $RM --ovopt $W3,sym_inplace,ekf_sparse ;;
    s100_w3) run $1 $MINE $(C c4s100) $R80 $RM --ovopt $W3 ;;
    s100_w3sp) run $1 $MINE $(C c4s100) $R80 $RM --ovopt $W3,sym_inplace,ekf_sparse ;;
  esac
}
JOBS=${@:-h_ref h_def h_sym h_w3_ref h_w3 h_w3sym h_w3sp r_ref r_def r_sym r_w3_ref r_w3 r_w3sym r_w3sp r_sp_only s75_w3 s75_w3sp s100_w3 s100_w3sp}
P=${P:-5}; n=0
for j in $JOBS; do job $j & n=$((n+1)); if [ $((n % P)) -eq 0 ]; then wait; fi; done; wait
CMP="python3 $H/runs/integ_20261025/cmp2.py"
echo "== exact: default / sym_inplace vs integ base"
$CMP $OUT/h_ref.jsonl $OUT/h_def.jsonl $OUT/h_sym.jsonl
$CMP $OUT/h_w3_ref.jsonl $OUT/h_w3.jsonl $OUT/h_w3sym.jsonl
$CMP $OUT/r_ref.jsonl $OUT/r_def.jsonl $OUT/r_sym.jsonl
$CMP $OUT/r_w3_ref.jsonl $OUT/r_w3.jsonl $OUT/r_w3sym.jsonl
echo "== near-exact: ekf_sparse vs W3 (pose differences)"
python3 $RD/near.py $OUT/h_w3.jsonl $OUT/h_w3sp.jsonl
python3 $RD/near.py $OUT/r_w3.jsonl $OUT/r_w3sp.jsonl
python3 $RD/near.py $OUT/r_def.jsonl $OUT/r_sp_only.jsonl
python3 $RD/near.py $OUT/s75_w3.jsonl $OUT/s75_w3sp.jsonl
python3 $RD/near.py $OUT/s100_w3.jsonl $OUT/s100_w3sp.jsonl
date
