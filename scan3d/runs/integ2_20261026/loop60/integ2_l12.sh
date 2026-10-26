python3 /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/integ2_mk60b.py || exit 1
cd ~/scanner/StereoDepthSeg-integ2/scan3d/runs/integ2_20261026/loop60
export OVLOOP_OUT=$HOME/scanner/tmp_integ2l/ate_id LOOP60_PP="--pose-precision 17"
mkdir -p $OVLOOP_OUT
P=~/scanner/StereoDepthSeg/venv_ffs/bin/python
$P loop60_run.py freeze; cat $OVLOOP_OUT/bin/br.sh $OVLOOP_OUT/bin/br_old.sh
$P loop60_run.py run "std:R60,N60,O60" --par 5 > $OVLOOP_OUT/sched.log 2>&1; tail -2 $OVLOOP_OUT/sched.log
