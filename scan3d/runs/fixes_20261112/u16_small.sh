#!/usr/bin/env bash
# u16 (11-13): small refusal tests on the laptop (cwd = WT): lens metadata differ (exit 10), incomplete phone keyframe folder (exit 16),
# a second run into the same OUT does not pick up the earlier merge.json (_prev).
set -u
O=$1; mkdir -p "$O"; P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
L=$(pwd)/scan3d/runs/laptop/20261112_121801_std60
U11A=$L/fix_u11merge/merge/part1/red/hs_20261112_121801_std60_kf; U11B=$L/fix_u11merge/merge/part2/red/hs_20261112_131328_std60_kf
LENS=$(pwd)/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json
rm -rf "$O/in"; mkdir -p "$O/in/a" "$O/in/b" "$O/in/c"
cp -al "$U11A" "$O/in/a/"; cp -al "$U11B" "$O/in/b/"; cp -al "$U11B" "$O/in/c/"
B=$O/in/b/$(basename "$U11B"); C=$O/in/c/$(basename "$U11B"); A=$O/in/a/$(basename "$U11A")
rm "$B/meta.json"; $P -c "import json,sys;m=json.load(open('$U11B/meta.json'));m['focus_dpt_request']=0.6;json.dump(m,open('$B/meta.json','w'),indent=1)"
$P scan3d/scanA/merge_align.py "$A" "$B" --out "$O/lens/merged_kf" --lens "$LENS" > "$O/lens.log" 2>&1; echo "lens test rc $?"; grep -m1 "MERGE FAILED" "$O/lens.log"
rm "$C/meta.json"
bash scan3d/scanA/run_merge.sh "$O/m16" "$A" "$C" > "$O/m16.log" 2>&1; echo "incomplete folder test rc $?"; grep -m1 "lacks" "$O/m16.log"
bash scan3d/scanA/run_merge.sh "$O/m16" "$A" "$C" > "$O/m16b.log" 2>&1; echo "same OUT again rc $?"; grep "_prev" "$O/m16b.log"
$P -c "import json;s=json.load(open('$O/m16/merge_summary.json'));print('summary status', s['status'], 'exit', s['exit_code'], 'merge', s['merge'])"
rm -rf "$O/in"
exit 0
