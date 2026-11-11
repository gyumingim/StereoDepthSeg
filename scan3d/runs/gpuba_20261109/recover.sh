#!/bin/bash
# gpuba 11-11 14:55: desktop WSL restarted while laptop job 214550 ran -> wait for its rc there, pull, clean, unlock, restart lanes
RL=/home/system/scanner/StereoDepthSeg-laptop/scan3d/tools/remote_laptop.sh; WT=/home/system/scanner/StereoDepthSeg-gpuba; J=/home/system/scanner/laptop_jobs/20261111_143335_20261021_214550_cmd
cd $WT/scan3d/runs/gpuba_20261109
until r=$(WT=$WT $RL sh "cat $J/rc 2>/dev/null" 2>/dev/null) && [[ "$r" =~ ^[0-9]+$ ]]; do sleep 30; done
echo "214550 laptop rc $r $(date +%T)"
WT=$WT $RL pull 20261021_214550/gpuba
WT=$WT $RL sh "rm -rf /home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261021_214550 /home/system/scanner/laptop_cache"
WT=$WT $RL unlock
cat lap_lane.out >> lap_lane_part1.out
setsid -f bash lap_lane.sh > lap_lane.out 2>&1 < /dev/null
sleep 90
setsid -f bash lap_full_lane.sh > lap_full_lane.out 2>&1 < /dev/null
echo "lanes restarted $(date +%T)"
