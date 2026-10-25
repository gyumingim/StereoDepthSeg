# PC functional test of --viop-udp on raw80 (paced by feeder3 at real time, first SECS s), WSL loopback, + C header test.
set -u
H=~/scanner/StereoDepthSeg-ovdrone/scan3d
RD=$H/runs/ovdrone_20261025
O=/tmp/ovdrone/viop_pc; mkdir -p $O
B=$H/ov_bridge/build_ovdrone_full/ov_bridge
SECS=${SECS:-45}
python3 $RD/viop_rx.py listen --port 14556 --secs $((SECS+12)) --out $O/rx > $O/rx.log 2>&1 &
RX=$!
sleep 1
timeout $((SECS+5)) /tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1 | $B /tmp/ovdrone/cfg/c4/estimator_config.yaml --stdio --max-speed 8 \
  --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --remap /tmp/ovopt/remap_1920x1080.bin --ovopt all_exact \
  --predict 2 --predict-hz 200 --viop-udp 127.0.0.1:14556 --viop-replay-time 1 --viop-log $O/viop_log.csv > $O/out.jsonl 2> $O/err.txt
echo "bridge rc $?"
wait $RX
grep "viop" $O/err.txt | tail -3
python3 $RD/viop_rx.py check $O/rx --bridge-log $O/viop_log.csv
gcc -std=c99 -O2 -Wall -Wextra -I$H/ov_bridge $RD/viop_c_test.c -o $O/viop_c_test -lm && $O/viop_c_test $O/rx/raw.bin
