set -u
H=~/scanner/StereoDepthSeg-ovopt/scan3d
RD=$H/runs/ovopt_20261025
SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/imuprio
O=/tmp/imuprio/h; mkdir -p $O
NEW=$H/ov_bridge/build_ovopt_full/ov_bridge
OLD=/tmp/imuprio/ov_bridge_0618_full
cp $NEW /tmp/imuprio/ov_bridge_new_full
NEW=/tmp/imuprio/ov_bridge_new_full
CFG=$RD/cfg_identity/estimator_config.yaml
IN=/tmp/idlespin/pose_ab/stream.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
date; sha256sum $OLD $NEW
for L in 0 0.1 0.18; do python3 $SP/reorder.py $IN /tmp/imuprio/h_L$L.bin --lead $L & done
python3 $SP/reorder.py $IN /tmp/imuprio/h_L0.4nc.bin --lead 0.4 --no-chunk &
wait
run() { n=$1; bin=$2; in=$3; shift 3; /usr/bin/time -f "%U %S %e" -o $O/$n.time $bin $CFG $BASE "$@" < $in > $O/$n.jsonl 2> $O/$n.err; echo "== $n rc $? poses $(grep -c '"pose"' $O/$n.jsonl) pred $(grep -c '"pred"' $O/$n.jsonl) time $(cat $O/$n.time)"; }
run old $OLD $IN &
run new $NEW $IN &
run p2old $NEW $IN --predict 2 --predict-hz 200 &
run pt200 $NEW $IN --predict 2 --predict-hz 200 --pred-thread 1 --imu-log $O/pt200_imu.csv &
run pt1 $NEW $IN --predict 1 --pred-thread 1 &
wait
run L0 $NEW /tmp/imuprio/h_L0.bin &
run L0.1 $NEW /tmp/imuprio/h_L0.1.bin &
run L0.18 $NEW /tmp/imuprio/h_L0.18.bin &
run L0.18pt $NEW /tmp/imuprio/h_L0.18.bin --predict 2 --predict-hz 200 --pred-thread 1 &
run L0.4nc $NEW /tmp/imuprio/h_L0.4nc.bin &
wait
echo "--- default new vs 0.6.18 binary (all fields)"; python3 $SP/cmp2.py $O/old.jsonl $O/new.jsonl
echo "--- predict modes vs new default (all non-timing fields incl imu_held)"; python3 $SP/cmp2.py $O/new.jsonl $O/p2old.jsonl $O/pt200.jsonl $O/pt1.jsonl
echo "--- reordered/chunked vs new default (excluding imu_held)"; python3 $SP/cmp2.py --ex imu_held $O/new.jsonl $O/L0.jsonl $O/L0.1.jsonl $O/L0.18.jsonl $O/L0.18pt.jsonl $O/L0.4nc.jsonl
grep -h "pred thread\|predict:" $O/*.err
date
