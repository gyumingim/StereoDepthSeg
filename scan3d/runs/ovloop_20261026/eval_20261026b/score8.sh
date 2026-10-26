cd ~/scanner/StereoDepthSeg-ovloop/scan3d/runs/ovloop_20261026
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export OVLOOP_OUT=/home/system/scanner/tmp_ovloop/ate
cp /mnt/c/Users/a3162/AppData/Local/Temp/claude/c--Users-a3162-Desktop----/27548596-eef5-4a58-bd39-e92ba44d822c/scratchpad/team/ovloop/new/sum.py ./sum_score.py
$PY ovloop_run.py score > $OVLOOP_OUT/score_print.txt 2>&1; tail -2 $OVLOOP_OUT/score_print.txt | cut -c1-200
$PY sum_score.py | tee $OVLOOP_OUT/sum.txt
$PY recall.py $OVLOOP_OUT 2>&1 | tail -12 | tee $OVLOOP_OUT/recall.txt
