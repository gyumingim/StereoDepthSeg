cd ~/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026/loop60
mkdir -p ~/scanner/tmp_integ2l/ate
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
$P loop60_run.py freeze
$P loop60_run.py run "std:R60,N60" --par 4 > ~/scanner/tmp_integ2l/ate/sched.log 2>&1
tail -3 ~/scanner/tmp_integ2l/ate/sched.log
$P loop60_run.py score > score_print.txt 2>&1; tail -20 score_print.txt
cp ~/scanner/tmp_integ2l/ate/sched.log ~/scanner/tmp_integ2l/ate/bin/bridge.json .
