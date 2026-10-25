#!/bin/bash
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
pgrep -f 'round_std[.]sh' && pkill -f 'round_std[.]sh'
sed 's#/tmp/ovacc/round1.log#/tmp/ovacc/round1b.log#' round_std.sh > round_std.tmp && mv round_std.tmp round_std.sh
nohup bash round_std.sh > /tmp/ovacc/round_std.log 2>&1 &
sleep 1; pgrep -fa 'round_std[.]sh'; grep round1 round_std.sh
