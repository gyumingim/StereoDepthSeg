# integ-20261025: VIOP POSE rate at --predict-hz 200 on the PC, paced (real-time) feed of the C4 harness stream
# (/tmp/imuprio/h_L0.4nc.bin, 845 frames), UDP loopback to viop_rx.py. OLD = main-loop prediction (no --pred-thread,
# = the path measured at 127 Hz on the phone by ovdrone), NEW = --pred-thread 1 (integ: link driven by the prediction
# thread). PC is shared (ATE jobs running) -> timing indicative only. Outputs /tmp/integ/vrate/<run>/ + log on stdout.
set -u
H=~/scanner/StereoDepthSeg-integ/scan3d
RD=$H/runs/integ_20261025
O=/tmp/integ/vrate; mkdir -p $O
B=$H/ov_bridge/build_integ_full/ov_bridge
CFG=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
IN=/tmp/imuprio/h_L0.4nc.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
for rep in 1 2; do
for m in OLD NEW; do
  n=${m}_$rep; rm -rf $O/$n; port=$((14680 + rep * 2))
  [ $m = NEW ] && { X="--pred-thread 1"; port=$((port + 1)); } || X=""
  python3 $H/runs/ovdrone_20261025/viop_rx.py listen --port $port --bind 127.0.0.1 --secs 45 --ping-period 1000 --out $O/$n > $O/$n.rx.log 2>&1 &
  sleep 0.5
  python3 $H/runs/imuprio_20261025/pfeed.py $IN $O/$n.jsonl $O/$n.rxlines.json -- $B $CFG $BASE --predict 2 --predict-hz 200 $X \
    --viop-udp 127.0.0.1:$port --viop-log $O/$n.viop_log.csv 2> $O/$n.err > /dev/null
  wait
  python3 $H/runs/ovdrone_20261025/viop_rx.py check $O/$n > /dev/null 2>&1
  python3 - $O/$n/check.json $n <<'PY'
import json, sys
d = json.load(open(sys.argv[1])); r = d.get("rate", {}); h = d.get("heartbeat", {})
print(sys.argv[2], "pose frames", d.get("pose_frames"), "| rate", r.get("frames_per_s"), "Hz | t_pred gap ms p50/p95/max",
      r.get("t_pred_gap_ms_p50"), r.get("t_pred_gap_ms_p95"), r.get("t_pred_gap_ms_max"), "| arrival gap p95/max",
      r.get("arrival_gap_ms_p95"), r.get("arrival_gap_ms_max"), "| lost", r.get("lost_by_seq"), "| hb", h.get("per_s"), "Hz gap max", h.get("gap_ms_max"))
PY
  grep -h "viop: udp.*sent\|pred thread:" $O/$n.err
done
done
grep -c '"pose"' $O/*.jsonl
