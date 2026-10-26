cd ~/scanner/StereoDepthSeg-ovloop/scan3d/runs/ovloop_20261026
PY=~/scanner/StereoDepthSeg/venv_ffs/bin/python
A=~/scanner/tmp_ovloop/ate
ls $A/kf | grep -vc _B30
$PY vpr_batch.py --model boq --kfroot $A/kf --outdir $A/vpr/boq --fp16 2>&1 | grep -v Warning | tail -3
$PY vpr_batch.py --model salad --kfroot $A/kf --outdir $A/vpr/salad --fp16 2>&1 | grep -v Warning | tail -3
echo vpr done
