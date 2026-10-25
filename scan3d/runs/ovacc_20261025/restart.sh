#!/bin/bash
# stop the round1 scheduler (running child jobs finish on their own), start round1b
pkill -f 'bash round1.sh'
pkill -f 'acc_run.py run L0,LC'
sleep 1
ps aux | grep -c '[a]cc_run.py job'
cd /home/system/scanner/StereoDepthSeg-ovacc/scan3d/runs/ovacc_20261025
echo "round1 stopped $(date +%T) (switch to round1b)" >> /tmp/ovacc/round1.log
nohup bash round1b.sh > /tmp/ovacc/round1b.log 2>&1 &
sleep 2; cat /tmp/ovacc/round1b.log
