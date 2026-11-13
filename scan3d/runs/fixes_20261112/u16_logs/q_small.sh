cd /home/system/scanner/StereoDepthSeg-fixes/scan3d
JOB_TIMEOUT_S=1800 bash tools/remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/u16_small.sh $WT/$O' --tag fix_u16_small --send /home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u11merge/merge/part1/red --send /home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u11merge/merge/part2/red
echo "EXIT $?"
