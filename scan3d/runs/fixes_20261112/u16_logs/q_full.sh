# q_full.sh NAME TAG 'run_merge args'  (one laptop job; sends the red folders it needs)
cd /home/system/scanner/StereoDepthSeg-fixes/scan3d
L=$PWD/runs/laptop/20261112_121801_std60
N=$1; TAG=$2; XA=${3:-}
S=(--send $L/fix_u11merge/merge/part1/red)
case $N in u11|x105|x095|shift) S+=(--send $L/fix_u11merge/merge/part2/red);; u10) S=(--send $L/fix_u10split/merge/part1/red --send $L/fix_u10split/merge/part2/red --send $L/fix_u8/scan/ba/poses_ba.jsonl.gz);;
  neg) S+=(--send $PWD/runs/desktop/20261024_165506_std60/fix_u13c_pt/red);; esac
JOB_TIMEOUT_S=5400 bash tools/remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/u16_full.sh $WT/$O '"$N $XA" --tag $TAG "${S[@]}"
echo "EXIT $?"
