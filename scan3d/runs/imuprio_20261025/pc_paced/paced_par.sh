set -u
SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/imuprio
O=/tmp/imuprio/acc; mkdir -p $O
BR=/tmp/imuprio/ov_bridge_new_full
CFG=~/scanner/StereoDepthSeg/scan3d/runs/vio_v3w4kbl8/estimator_config.yaml
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date
[ -f /tmp/imuprio/d40c.bin ] || python3 $SP/reorder.py /tmp/idlespin/pose_ab/stream.bin /tmp/imuprio/d40c.bin --lead 0.04 --imu-per-chunk 16
run() { n=$1; in=$2; shift 2; /usr/bin/time -f "%U %S %e" -o $O/$n.time python3 $SP/pfeed.py $in $O/$n.jsonl $O/$n.rx.json -- $BR $ARGS "$@" --timing-log $O/$n.tlog 2> $O/$n.err > $O/$n.out; echo "== $n $(date +%T) poses $(grep -c '"pose"' $O/$n.jsonl) pred $(grep -c '"pred"' $O/$n.jsonl) time(feeder+bridge) $(cat $O/$n.time)"; }
for rep in 1; do
run P0_$rep /tmp/predict/delay40.bin --predict 0 &
run P1_$rep /tmp/predict/delay40.bin --predict 1 &
run P2o_$rep /tmp/predict/delay40.bin --predict 2 --predict-hz 200 &
wait
run P2t_$rep /tmp/predict/delay40.bin --predict 2 --predict-hz 200 --pred-thread 1 &
run P2tc_$rep /tmp/imuprio/d40c.bin --predict 2 --predict-hz 200 --pred-thread 1 &
run P1c_$rep /tmp/imuprio/d40c.bin --predict 1 &
wait
done
echo "--- identity vs P0_1 (excluding imu_held)"
python3 $SP/cmp2.py --ex imu_held $O/P0_1.jsonl $O/P1_1.jsonl $O/P2o_1.jsonl $O/P2t_1.jsonl $O/P2tc_1.jsonl $O/P1c_1.jsonl
echo "--- accuracy"
python3 ~/scanner/StereoDepthSeg/scan3d/runs/predict_check_20261025/pred_acc.py $O/P1_1.jsonl $O/P2o_1.jsonl $O/P2t_1.jsonl $O/P2tc_1.jsonl $O/P1c_1.jsonl
echo "--- latency / age"
for n in P1_1 P2o_1 P2t_1 P2tc_1 P1c_1; do python3 -c "
import json;r=json.load(open('$O/$n.rx.json'))
f=lambda k:(lambda d:'%s/%s/%s'%(d.get('p50'),d.get('p95'),d.get('max')))(r.get(k,{}))
print('$n','rate',r.get('rate_hz'),'lat all',f('latency_ms_all'),'frame',f('latency_ms_frame'),'rate',f('latency_ms_rate'),'| age@1kHz p50/p95/max',f('age_at_consumer_ms_1khz'),'mean',r.get('age_at_consumer_ms_1khz',{}).get('mean'),'| gap',f('gap_between_lines_ms'),'| imu late',r['imu_write_late_ms'])"; done
grep -h "pred thread:" $O/*.err
date
