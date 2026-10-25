# ovdrone identity + RS sanity (2026-10-25). REF = frozen ovopt-tip full build 71bb15d9 (/tmp/ovopt/ate/bin).
set -u
H=~/scanner/StereoDepthSeg-ovdrone/scan3d
RD=$H/runs/ovdrone_20261025
CMP=$H/runs/ovopt_20261025/cmp.py
OUT=/tmp/ovdrone/ident; mkdir -p $OUT
REF=/tmp/ovopt/ate/bin/ov_bridge_71bb15d95c1f
MINE=${MINE:-$H/ov_bridge/build_ovdrone_full/ov_bridge}
sed -i "s#/tmp/ovopt/cfg/c4/#/tmp/ovdrone/cfg/c4/#" /tmp/ovdrone/cfg/c4/estimator_config.yaml
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $REF $MINE
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
HC=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/idlespin/pose_ab/stream.bin
C4=/tmp/ovdrone/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
for job in ${JOBS:-h_ref h_mine h_mine_ex r_ref r_mine r_mine_ex r_rs r_rsflip}; do
  case $job in
    h_ref) run $job $REF $HC $HS ;;
    h_mine) run $job $MINE $HC $HS ;;
    h_mine_ex) run $job $MINE $HC $HS --ovopt all_exact ;;
    r_ref) run $job $REF $C4 $R80 $RM ;;
    r_mine) run $job $MINE $C4 $R80 $RM ;;
    r_mine_ex) run $job $MINE $C4 $R80 $RM --ovopt all_exact ;;
    r_rs) run $job $MINE $C4 $R80 $RM --ovopt all_exact,rs_derot ;;
    r_rsflip) run $job $MINE $C4 $R80 $RM --ovopt all_exact,rs_derot=-6540 ;;
  esac
done
python3 $CMP $OUT/h_ref.jsonl $OUT/h_mine.jsonl $OUT/h_mine_ex.jsonl
python3 $CMP $OUT/r_ref.jsonl $OUT/r_mine.jsonl $OUT/r_mine_ex.jsonl $OUT/r_rs.jsonl $OUT/r_rsflip.jsonl
grep -h "rs_derot" $OUT/*.err
date
