# esp32fix-20261026: VIOP heartbeat timing on the PC, OLD (--viop-hb-src 0 = main loop, default) vs NEW (--viop-hb-src 1 =
# steady thread), same esp32fix binary, --pred-thread 1 --predict 2 --predict-hz 200 (= profile link path), paced
# (real-time) feed of the C4 harness stream /tmp/imuprio/h_L0.4nc.bin (845 frames), UDP loopback to ovdrone viop_rx.py.
# Same method as runs/integ_20261025/viop_rate_pc.sh. PC is shared with other agents -> timing indicative only.
#   bash hb_rate_pc.sh   -> /tmp/esp32fix/hb/<run>/check.json, log on stdout
set -u
H=~/scanner/StereoDepthSeg-esp32fix/scan3d
O=/tmp/esp32fix/hb; mkdir -p $O
B=$H/ov_bridge/build_esp32fix_full/ov_bridge
CFG=$H/runs/ovopt_20261025/cfg_identity/estimator_config.yaml
IN=/tmp/imuprio/h_L0.4nc.bin
BASE="--stdio --max-speed 8 --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2"
date; sha256sum $B; uptime
for rep in 1 2; do
for m in OLD NEW; do
  n=${m}_$rep; rm -rf $O/$n; port=$((14760 + rep * 2))
  [ $m = NEW ] && { X="--viop-hb-src 1"; port=$((port + 1)); } || X=""
  python3 $H/runs/ovdrone_20261025/viop_rx.py listen --port $port --bind 127.0.0.1 --secs 45 --ping-period 1000 --out $O/$n > $O/$n.rx.log 2>&1 &
  sleep 0.5
  python3 $H/runs/imuprio_20261025/pfeed.py $IN $O/$n.jsonl $O/$n.rxlines.json -- $B $CFG $BASE --predict 2 --predict-hz 200 --pred-thread 1 $X \
    --viop-udp 127.0.0.1:$port --viop-log $O/$n.viop_log.csv 2> $O/$n.err > /dev/null
  wait
  python3 $H/runs/ovdrone_20261025/viop_rx.py check $O/$n > /dev/null 2>&1
  python3 - $O/$n/check.json $n <<'PY'
import json, sys
d = json.load(open(sys.argv[1])); r = d.get("rate", {}); h = d.get("heartbeat", {})
print(sys.argv[2], "pose frames", d.get("pose_frames"), "| pose rate", r.get("frames_per_s"), "Hz arrival gap p95/max",
      r.get("arrival_gap_ms_p95"), r.get("arrival_gap_ms_max"), "| hb n", h.get("n"), "rate", h.get("per_s"), "Hz gap ms p50/p95/max",
      h.get("gap_ms_p50"), h.get("gap_ms_p95"), h.get("gap_ms_max"))
PY
  python3 /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/esp32fix/hbstat.py $O/$n/hb.csv $n
  grep -h "viop: udp.*sent\|viop heartbeat" $O/$n.err
done
done
