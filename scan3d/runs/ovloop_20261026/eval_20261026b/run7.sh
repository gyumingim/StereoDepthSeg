cd ~/scanner/StereoDepthSeg-ovloop/scan3d/runs/ovloop_20261026
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
export OVLOOP_OUT=/home/system/scanner/tmp_ovloop/ate
$PY ovloop_run.py run "std:Q30,S30;loop:Q30,S30;hs:Q30,S30" --par 4 > $OVLOOP_OUT/run7.log 2>&1
tail -1 $OVLOOP_OUT/run7.log; grep -c FAILED $OVLOOP_OUT/run7.log
echo run7 done
