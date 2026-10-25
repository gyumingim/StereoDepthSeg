set -u
SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/imuprio
O=/tmp/imuprio/acc2; mkdir -p $O
BR=/tmp/imuprio/ov_bridge_new_full
CFG=~/scanner/StereoDepthSeg/scan3d/runs/vio_v3w4kbl8/estimator_config.yaml
ARGS="$CFG --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date; uptime
run() { n=$1; in=$2; shift 2; /usr/bin/time -f "%U %S %e" -o $O/$n.time python3 $SP/pfeed.py $in $O/$n.jsonl $O/$n.rx.json -- $BR $ARGS "$@" --timing-log $O/$n.tlog 2> $O/$n.err > $O/$n.out; echo "== $n $(date +%T) poses $(grep -c '"pose"' $O/$n.jsonl) pred $(grep -c '"pred"' $O/$n.jsonl) time $(cat $O/$n.time) load $(cut -d' ' -f1 /proc/loadavg)"; }
for rep in 1 2; do
run P1_$rep /tmp/predict/delay40.bin --predict 1
run P2o_$rep /tmp/predict/delay40.bin --predict 2 --predict-hz 200
run P2t_$rep /tmp/predict/delay40.bin --predict 2 --predict-hz 200 --pred-thread 1
run P2tc_$rep /tmp/imuprio/d40c.bin --predict 2 --predict-hz 200 --pred-thread 1
done
python3 $SP/cmp2.py --ex imu_held /tmp/imuprio/acc/P0_1.jsonl $O/*_?.jsonl
python3 ~/scanner/StereoDepthSeg/scan3d/runs/predict_check_20261025/pred_acc.py $O/P1_1.jsonl $O/P2o_1.jsonl $O/P2t_1.jsonl $O/P2tc_1.jsonl $O/P2t_2.jsonl $O/P2tc_2.jsonl
for n in P1_1 P2o_1 P2t_1 P2tc_1 P1_2 P2o_2 P2t_2 P2tc_2; do python3 -c "
import json;r=json.load(open('$O/$n.rx.json'))
f=lambda k:(lambda d:'%s/%s/%s'%(d.get('p50'),d.get('p95'),d.get('max')))(r.get(k,{}))
print('$n','rate',r.get('rate_hz'),'lat all',f('latency_ms_all'),'frame',f('latency_ms_frame'),'rate',f('latency_ms_rate'),'| age@1kHz p50/p95/max',f('age_at_consumer_ms_1khz'),'mean',r.get('age_at_consumer_ms_1khz',{}).get('mean'),'| gap',f('gap_between_lines_ms'),'| imu late p95 %.1f'%r['imu_write_late_ms']['p95'])"; done
grep -h "pred thread:" $O/*.err
python3 - <<'PY'
import csv,glob,numpy as np
for f in sorted(glob.glob('/tmp/imuprio/acc2/*_1.tlog')):
    r=list(csv.DictReader(open(f))); x=np.array([float(a['feed_ms'])+float(a['remap_ms']) for a in r if a['init']=='1'])
    print(f, 'proc ms p50 %.1f p95 %.1f'%(np.percentile(x,50),np.percentile(x,95)))
PY
date
