# PC functional test of --mav-udp on raw80 (paced by feeder3 at real time, first SECS s). WSL loopback.
set -u
H=~/scanner/StereoDepthSeg-ovdrone/scan3d
RD=$H/runs/ovdrone_20261025
O=/tmp/ovdrone/mavpc; mkdir -p $O
B=$H/ov_bridge/build_ovdrone_full/ov_bridge
[ -x /tmp/ovdrone/feeder3 ] || g++ -O2 -pthread $H/runs/ovopt_20261025/feeder3.cpp -o /tmp/ovdrone/feeder3
SECS=${SECS:-45}
MOUNT=${MOUNT:-cam_fwd}
cp $RD/mav_rx.py $O/
python3 $O/mav_rx.py listen --port 14555 --secs $((SECS+15)) --out $O/rx > $O/rx.log 2>&1 &
RX=$!
sleep 1
timeout $((SECS+5)) /tmp/ovdrone/feeder3 /tmp/ovopt/raw80.bin 1 | $B /tmp/ovdrone/cfg/c4/estimator_config.yaml --stdio --max-speed 8 \
  --lost-frames 1000000 --startup-lost-frames 1000000 --threads 2 --remap /tmp/ovopt/remap_1920x1080.bin --ovopt all_exact \
  --predict 2 --predict-hz 100 --mav-udp 127.0.0.1:14555 --mav-msg both --mav-mount $MOUNT --mav-replay-time 1 \
  --mav-log $O/mav_log.csv > $O/out.jsonl 2> $O/err.txt
echo "bridge rc $?"
wait $RX
grep "mav" $O/err.txt | tail -5
python3 $O/mav_rx.py check $O/rx --bridge-log $O/mav_log.csv
python3 $O/mav_rx.py pymavlink-check $O/rx 2>&1 | grep -v "deprecated\|msgname\|compatib\|^$" | tail -3
