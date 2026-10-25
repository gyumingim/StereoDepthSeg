SP=/mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/imuprio
H=~/scanner/StereoDepthSeg-ovopt/scan3d; O=/tmp/imuprio/h3
CFG=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml; IN=/tmp/imuprio/h_L0.4nc.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --pose-precision 17"
/tmp/imuprio/ov_bridge_0618_full $CFG $BASE < $IN > $O/x_old.jsonl 2>$O/x_old.err &
/tmp/imuprio/ov_bridge_new3_full $CFG $BASE < $IN > $O/x_new.jsonl 2>$O/x_new.err &
OVB_PRED_CHECK=1 /tmp/imuprio/ov_bridge_new3_full $CFG $BASE --predict 2 --predict-hz 200 --pred-thread 1 < $IN > $O/x_pt.jsonl 2>$O/x_pt.err &
wait
python3 $SP/cmp2.py $O/x_old.jsonl $O/x_new.jsonl $O/x_pt.jsonl
grep -h "pred thread" $O/x_pt.err
