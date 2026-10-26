set -u
# identity after the WSL crash: raw80 (pulled back from the phone, sha256 checked) + phone c4 config + phone remap
W=~/scanner/StereoDepthSeg-ovloop/scan3d
T=~/scanner/tmp_ovloop
OUT=$T/ident; mkdir -p $OUT $T/bin
cp $W/ov_bridge/build_ovloop_ref/ov_bridge $T/bin/ov_bridge_ref
cp $W/ov_bridge/build_ovloop_full/ov_bridge $T/bin/ov_bridge_mine
REF=$T/bin/ov_bridge_ref; MINE=$T/bin/ov_bridge_mine
VOC=$W/ov_bridge/loop/voc_k10L4.voc
C4=$T/cfg_c4/estimator_config.yaml
RM="--remap $HOME/scanner/StereoDepthSeg/scan3d/config/phone_ov_main/c4/remap_1920x1080.bin"
R80=$T/raw80.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $REF $MINE $VOC $R80 $C4 ~/scanner/StereoDepthSeg-integ2/scan3d/ov_bridge/build_integ2_full/ov_bridge
cd $W && git log --oneline -1 && git status --short ov_bridge | grep -v build_
run() { local n=$1 b=$2; shift 2
  nice -n 5 $b $C4 $BASE "$@" < $R80 > $OUT/$n.jsonl 2> $OUT/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $OUT/$n.jsonl) | $*"; }
run r_ref $REF $RM --profile rec20261025 --drop-late 0 &
run r_mine $MINE $RM --profile rec20261025 --drop-late 0 &
run r_loop $MINE $RM --profile rec20261025 --drop-late 0 --loop 1 --loop-vocab $VOC --loop-sync 1 --loop-log $OUT/r_loop.loop.jsonl &
run r_loopw $MINE $RM --profile rec20261025 --drop-late 0 --loop 1 --loop-vocab $VOC --loop-sync 1 --loop-cfg weight_mode=1 --loop-log $OUT/r_loopw.loop.jsonl &
wait
C=~/scanner/StereoDepthSeg-integ/scan3d/runs/integ_20261025/cmp2.py
echo "-- off vs ref (full lines)"; python3 $C $OUT/r_ref.jsonl $OUT/r_mine.jsonl
echo "-- loop on vs ref (map key excluded)"; python3 $C --ex map $OUT/r_ref.jsonl $OUT/r_loop.jsonl; python3 $C --ex map $OUT/r_ref.jsonl $OUT/r_loopw.jsonl
echo "-- map smoothness"; python3 $W/runs/ovloop_20261026/check_smooth.py $OUT/r_loop.jsonl --ref $OUT/r_ref.jsonl; python3 $W/runs/ovloop_20261026/check_smooth.py $OUT/r_loopw.jsonl --ref $OUT/r_ref.jsonl
grep "loop:" $OUT/r_loop.err $OUT/r_loopw.err | tail -4
date
