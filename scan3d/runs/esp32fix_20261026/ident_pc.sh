# esp32fix-20261026 PC identity: esp32fix build vs integ-20261025 build (14de3b8, build_integ_full), pose lines 17 digits.
# H = C4 harness stream (845 frames, cfg_identity), R = raw80 (2327 frames, c4 + remap). NEW flag runs (--viop-hb-src 1)
# must give the same poses too (heartbeat only sends UDP).
set -u
OUT=/tmp/esp32fix/ident; mkdir -p $OUT
REF=~/scanner/StereoDepthSeg-integ/scan3d/ov_bridge/build_integ_full/ov_bridge
MINE=~/scanner/StereoDepthSeg-esp32fix/scan3d/ov_bridge/build_esp32fix_full/ov_bridge
HC=~/scanner/StereoDepthSeg-integ/scan3d/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
HS=/tmp/imuprio/h_L0.4nc.bin
C4=/tmp/ovopt/cfg/c4/estimator_config.yaml
RM="--remap /tmp/ovopt/remap_1920x1080.bin"
R80=/tmp/ovopt/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $REF $MINE
run() { local n=$1 b=$2 cfg=$3 in=$4; shift 4
  $b $cfg $BASE "$@" < $in > $OUT/$n.jsonl 2> $OUT/$n.err; grep '"pose"' $OUT/$n.jsonl > $OUT/$n.pose; echo "== $n rc $? poses $(wc -l < $OUT/$n.pose) | $*"; }
run h_ref $REF $HC $HS &
run h_mine $MINE $HC $HS &
run hv_ref $REF $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14771 &
run hv_mine0 $MINE $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14772 &
run hv_mine1 $MINE $HC $HS --predict 2 --predict-hz 200 --pred-thread 1 --viop-udp 127.0.0.1:14773 --viop-hb-src 1 &
wait
run r_ref $REF $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run r_mine $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 &
run rv_mine1 $MINE $C4 $R80 $RM --profile rec20261025 --drop-late 0 --viop-udp 127.0.0.1:14774 --viop-hb-src 1 &
wait
C=~/scanner/StereoDepthSeg-integ/scan3d/runs/integ_20261025/cmp2.py
# pose lines without timing keys (ms, remap_ms, wait_ms); drop-late 0 on raw80 (drop-late is timing dependent)
for p in "h_ref h_mine" "hv_ref hv_mine0" "hv_ref hv_mine1" "r_ref r_mine" "r_ref rv_mine1"; do
  set -- $p; echo "-- $1 vs $2"; python3 $C $OUT/$1.jsonl $OUT/$2.jsonl; done
grep -h "viop heartbeat\|viop: udp" $OUT/hv_mine1.err $OUT/rv_mine1.err
date
