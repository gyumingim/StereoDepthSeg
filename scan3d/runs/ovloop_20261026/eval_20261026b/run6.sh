cd ~/scanner/StereoDepthSeg-ovloop/scan3d/runs/ovloop_20261026
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export OVLOOP_OUT=/home/system/scanner/tmp_ovloop/ate
mkdir -p $OVLOOP_OUT
$PY ovloop_run.py freeze && $PY ovloop_run.py run "std:B30,B30w;loop:B30,B30w;hs:B30,B30w" --par 5 > $OVLOOP_OUT/run6.log 2>&1
tail -1 $OVLOOP_OUT/run6.log; grep -c FAILED $OVLOOP_OUT/run6.log
echo run6 done
