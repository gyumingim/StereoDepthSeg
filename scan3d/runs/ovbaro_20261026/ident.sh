# ovbaro PC identity (2026-10-26): this branch's build without --baro (and with type-7 records in the stream, ignored)
# vs the integ2 build (build_integ2_full, integ2-20261026 tip). raw80 phone stream (2327 frames, c4 + remap), live
# profile rec20261025 --drop-late 0, pose-precision 17. Compare = integ cmp2.py (pose lines without timing keys).
set -u
OUT=/tmp/ovbaro/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge
MINE=~/scanner/StereoDepthSeg-ovbaro/scan3d/ov_bridge/build_ovbaro_full/ov_bridge
R=~/scanner/StereoDepthSeg-ovbaro/scan3d/runs/ovbaro_20261026
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17 --profile rec20261025 --drop-late 0"
date; sha256sum $REF $MINE $C4
run() { local n=$1 b=$2; shift 2
  nice -n 5 $b $C4 $BASE $RM "$@" < $R80 > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
run r_ref $REF &
run r_mine $MINE &
( ~/scanner/StereoDepthSeg/venv_ffs/bin/python $R/inject_baro.py $R80 2> $OUT/inj.err | nice -n 5 $MINE $C4 $BASE $RM > $OUT/r_mine_t7.jsonl 2> $OUT/r_mine_t7.err; echo "== r_mine_t7 rc $? poses $(grep -c '"pose"' $OUT/r_mine_t7.jsonl)"; cat $OUT/inj.err ) &
wait
C=~/scanner/StereoDepthSeg-integ/scan3d/runs/integ_20261025/cmp2.py
for p in "r_ref r_mine" "r_ref r_mine_t7"; do set -- $p; echo "-- $1 vs $2"; python3 $C $OUT/$1.jsonl $OUT/$2.jsonl; done
grep -h "baro" $OUT/r_mine_t7.err | tail -2
echo "-- bad cfg refusal"; $MINE $C4 --stdio --baro 1 --baro-cfg foo=1 < /dev/null > /dev/null 2> $OUT/badcfg.err; echo "rc $?"; grep baro $OUT/badcfg.err
date
