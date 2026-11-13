#!/usr/bin/env bash
# u16 (11-13): one full run_merge.sh on the laptop (cwd = WT). Inputs = reduced keyframe folders used as phone-sent folders
# (review 1-6 path: REC/frames_kf exists -> no VIO / reduce, lensonly made in place). The folders are copied first so the
# laptop copies of the sent originals stay untouched.
#   u16_full.sh OUT NAME [run_merge options]       NAME = u11 | u10 | neg | x105 | x095 | shift
set -u
O=$1; N=$2; shift 2; mkdir -p "$O"
WT=$(pwd)
L=$WT/scan3d/runs/laptop/20261112_121801_std60
U11A=$L/fix_u11merge/merge/part1/red/hs_20261112_121801_std60_kf; U11B=$L/fix_u11merge/merge/part2/red/hs_20261112_131328_std60_kf
U10A=$L/fix_u10split/merge/part1/red/hs_20261112_121801_std60_kf; U10B=$L/fix_u10split/merge/part2/red/hs_20261112_121801_std60_kf
NEG=$WT/scan3d/runs/desktop/20261024_165506_std60/fix_u13c_pt/red/hs_20261024_165506_std60_kf
cp_kf() { mkdir -p "$2"; rm -rf "$2/$(basename "$1")"; cp -al "$1" "$2/" && rm -rf "$2/$(basename "$1")/lensonly" && echo "$2/$(basename "$1")"; }
case $N in
  u11)  A=$(cp_kf "$U11A" "$O/in/a"); B=$(cp_kf "$U11B" "$O/in/b");;
  u10)  A=$(cp_kf "$U10A" "$O/in/a"); B=$(cp_kf "$U10B" "$O/in/b");;
  neg)  A=$(cp_kf "$U11A" "$O/in/a"); B=$(cp_kf "$NEG" "$O/in/b");;
  shift) A=$(cp_kf "$U11A" "$O/in/a"); B=$(cp_kf "$U11B" "$O/in/b");;     # with --align-args '--test-offset-m 0.2'
  x095) A=$(cp_kf "$U11A" "$O/in/a"); B=$O/in/b/hs_20261112_131328_std60_kf
        /home/system/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/runs/fixes_20261112/u16_scale_part.py "$U11B" "$B" 0.95238
        rm -rf "$B/lensonly";;
  x105) A=$(cp_kf "$U11A" "$O/in/a"); B=$O/in/b/hs_20261112_131328_std60_kf
        /home/system/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/runs/fixes_20261112/u16_scale_part.py "$U11B" "$B" 1.05
        rm -rf "$B/lensonly";;
  *) echo "bad NAME $N"; exit 1;;
esac
echo "$(date +%T) u16 $N: A $A B $B, args $*, gpu $(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader)"
bash scan3d/scanA/run_merge.sh "$O/merge" "$A" "$B" --depth-sigma ba --mesh-args "--tsdf-max-blocks 250000 --ffs-backend pytorch" "$@" > "$O/merge.log" 2>&1
rc=$?; echo $rc > "$O/merge_rc.txt"
echo "$(date +%T) u16 $N run_merge rc $rc"; tail -n 40 "$O/merge.log"
if [ $N = u10 ] && [ -f "$O/merge/scan/ba/poses_ba.jsonl.gz" ]; then   # ATE vs the single full-recording BA (u8), round 1 and round 2
  /home/system/scanner/StereoDepthSeg/venv_ffs/bin/python scan3d/runs/fixes_20261112/ate_merge.py "$O/merge" "$L/fix_u8/scan/ba/poses_ba.jsonl.gz" > "$O/ate_vs_fix_u8.json"
  head -c 1500 "$O/ate_vs_fix_u8.json"; fi
rm -rf "$O/in" "$O"/merge/scan/cache "$O"/merge/merged/*_align/img "$O"/merge/merged/*_align/*.db "$O"/merge/merged/*_merged_kf/frames_kf
exit 0
