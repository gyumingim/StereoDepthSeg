set -u
SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/imuprio
W=~/scanner/StereoDepthSeg-ovopt/scan3d/ov_bridge
date
cmake --build $W/build_ovopt_full -j6 2>&1 | grep -E "error|Built target ov_bridge"
B=/tmp/imuprio/ov_bridge_new3_full; cp $W/build_ovopt_full/ov_bridge $B
H=~/scanner/StereoDepthSeg-ovopt/scan3d; O=/tmp/imuprio/h3; mkdir -p $O
CFG=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml; IN=/tmp/idlespin/pose_ab/stream.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
$B $CFG $BASE < $IN > $O/new.jsonl 2>$O/new.err &
OVB_PRED_CHECK=1 $B $CFG $BASE --predict 2 --predict-hz 200 --pred-thread 1 < $IN > $O/pt200.jsonl 2>$O/pt200.err &
$B $CFG $BASE < /tmp/imuprio/h_L0.18.bin > $O/L0.18.jsonl 2>$O/L0.18.err &
OVB_PRED_CHECK=1 $B $CFG $BASE --predict 2 --predict-hz 200 --pred-thread 1 < /tmp/imuprio/h_L0.bin > $O/L0pt.jsonl 2>$O/L0pt.err &
wait
python3 $SP/cmp2.py /tmp/imuprio/h/old.jsonl $O/new.jsonl $O/pt200.jsonl
python3 $SP/cmp2.py --ex imu_held /tmp/imuprio/h/old.jsonl $O/L0.18.jsonl $O/L0pt.jsonl
grep -h "pred thread" $O/*.err
CFG2=~/scanner/StereoDepthSeg/scan3d/runs/vio_v3w4kbl8/estimator_config.yaml
ARGS="$CFG2 --stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
O2=/tmp/imuprio/acc3; mkdir -p $O2
OVB_PRED_CHECK=1 python3 $SP/pfeed.py /tmp/imuprio/d40c.bin $O2/P2tc.jsonl $O2/P2tc.rx.json -- $B $ARGS --predict 2 --predict-hz 200 --pred-thread 1 2> $O2/P2tc.err > /dev/null
python3 $SP/cmp2.py --ex imu_held /tmp/imuprio/acc/P0_1.jsonl $O2/P2tc.jsonl
grep -h "pred thread" $O2/P2tc.err
python3 ~/scanner/StereoDepthSeg/scan3d/runs/predict_check_20261025/pred_acc.py $O2/P2tc.jsonl
python3 -c "import json;r=json.load(open('$O2/P2tc.rx.json'));print('age',r['age_at_consumer_ms_1khz'],'rate',r['rate_hz'])"
date
