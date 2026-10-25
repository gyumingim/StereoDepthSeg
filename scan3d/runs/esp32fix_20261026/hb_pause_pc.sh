# esp32fix-20261026: heartbeat liveness with --viop-hb-src 1: the feeder is paused 2 s at 25 s (as ovdrone pc_viop_test.sh)
# -> heartbeats must stop during the pause (gap ~2 s + <= 100 ms feed liveness) and resume. raw80 c4 stream, feeder3 paced,
# --pred-thread 1 --predict 2 --predict-hz 200. MODE=0 runs the same with --viop-hb-src 0 for reference.
set -u
H=~/scanner/StereoDepthSeg-esp32fix/scan3d
M=${MODE:-1}
O=/tmp/esp32fix/hbpause_$M; rm -rf $O; mkdir -p $O
B=$H/ov_bridge/build_esp32fix_full/ov_bridge
SECS=45
python3 $H/runs/ovdrone_20261025/viop_rx.py listen --port $((14790 + M)) --bind 127.0.0.1 --secs $((SECS+12)) --out $O/rx > $O/rx.log 2>&1 &
RX=$!
sleep 1
( sleep 25; pkill -STOP -f "/tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1 esp32fix$M" ; echo "feed paused $(date +%T.%N)"; sleep 2; pkill -CONT -f "/tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1 esp32fix$M"; echo "feed resumed $(date +%T.%N)" ) &
timeout $((SECS+5)) bash -c "exec -a '/tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1 esp32fix$M' /tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1" | $B /tmp/ovopt/cfg/c4/estimator_config.yaml --stdio --max-speed 8 \
  --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --remap /tmp/ovopt/remap_1920x1080.bin --ovopt all_exact \
  --predict 2 --predict-hz 200 --pred-thread 1 --viop-hb-src $M --viop-udp 127.0.0.1:$((14790 + M)) --viop-log $O/viop_log.csv > $O/out.jsonl 2> $O/err.txt
echo "bridge rc $?"
wait $RX
grep "viop" $O/err.txt | tail -3
python3 $H/runs/ovdrone_20261025/viop_rx.py check $O/rx > /dev/null 2>&1
python3 - $O/rx <<'PY'
import json, sys, os
d = json.load(open(os.path.join(sys.argv[1], "check.json"))); h = d.get("heartbeat", {})
print("hb", {k: h.get(k) for k in ("n", "per_s", "gap_ms_p50", "gap_ms_p95", "gap_ms_max")}, "states", h.get("states"))
PY
python3 /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/esp32fix/hbstat.py $O/rx/hb.csv pause_mode$M
