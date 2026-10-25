cd /tmp
for w in 2 5; do
  echo "=== window $w s"
  SEG_WIN=$w ~/scanner/StereoDepthSeg/venv_ffs/bin/python /home/system/scanner/StereoDepthSeg-ovdrone/scan3d/runs/ovdrone_20261025/seg_ate.py B0 R294 R294F 2>&1 | tail -14
done
