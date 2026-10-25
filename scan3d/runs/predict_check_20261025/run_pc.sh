# PC check of ov_bridge --predict (2026-10-25, arlat 0.6.17): identity of the pose lines for --predict 0/1/2 (paced
# real-time feed of the captured C4 input stream of hs_20261024_165506_std60) + accuracy of the predicted poses with the
# camera records delayed 40 / 60 ms behind the IMU (= phone camera HAL latency).
set -u
date
H=~/scanner/StereoDepthSeg/scan3d
RD=$H/runs/predict_check_20261025
OUT=/tmp/predict; mkdir -p $OUT
BR=$H/ov_bridge/build_par/ov_bridge
CFG=$H/runs/vio_v3w4kbl8/estimator_config.yaml      # config of the 0.6.16 idle-spin paced check (P0..P2)
STREAM=/tmp/idlespin/pose_ab/stream.bin
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
cd $H
run() {  # name stream extra
  echo "== $1 $(date +%T) ($3)"
  python3 $H/runs/idle_spin_check_20261025/paced_feed.py $2 $OUT/$1.jsonl -- $BR $ARGS $3 --timing-log $OUT/$1.tlog 2> $OUT/$1.err
  echo "pose $(grep -c '"pose"' $OUT/$1.jsonl) pred $(grep -c '"pred"' $OUT/$1.jsonl)"; grep "predict:" $OUT/$1.err
}
for D in 40 60; do [ -f $OUT/delay$D.bin ] || python3 $RD/make_delayed.py $STREAM $OUT/delay$D.bin $D; done
for n in ${RUNS:-R0 R1 R2 D40_0 D40_1 D40_2 D60_1}; do
  case $n in
    R0) run R0 $STREAM "--predict 0" ;;
    R1) run R1 $STREAM "--predict 1" ;;
    R2) run R2 $STREAM "--predict 2" ;;
    D40_0) run D40_0 $OUT/delay40.bin "--predict 0" ;;
    D40_1) run D40_1 $OUT/delay40.bin "--predict 1" ;;
    D40_2) run D40_2 $OUT/delay40.bin "--predict 2 --predict-hz 200" ;;
    D60_1) run D60_1 $OUT/delay60.bin "--predict 1" ;;
  esac
done
echo "== identity (ref = 0.6.16 paced P0 = --idle-spin 0 bridge before this change)"
python3 $H/runs/idle_spin_check_20261025/cmp.py /tmp/idlespin/pose_ab/P0.jsonl $OUT/R0.jsonl $OUT/R1.jsonl $OUT/R2.jsonl $OUT/D40_0.jsonl $OUT/D40_1.jsonl $OUT/D40_2.jsonl $OUT/D60_1.jsonl
echo "== accuracy"
python3 $RD/pred_acc.py $OUT/R1.jsonl $OUT/R2.jsonl $OUT/D40_1.jsonl $OUT/D40_2.jsonl $OUT/D60_1.jsonl
echo DONE $(date +%T)
