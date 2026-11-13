#!/usr/bin/env bash
# u16 (11-13): laptop post-processing (cwd = WT): top / slice renders of the u11 merges (old u11 vs final u11b) and the keyframe-centre
# difference between merged results (x105 vs u11b, u11b vs old u11). Light numpy only.
#   remote_laptop.sh job 121801 'cmd:bash scan3d/runs/fixes_20261112/u16_post.sh $WT/$O' --tag fix_u16_post --send <mesh/pose dirs>
set -u
O=$1; mkdir -p "$O"; P=/home/system/scanner/StereoDepthSeg/venv_ffs/bin/python
L=$(pwd)/scan3d/runs/laptop/20261112_121801_std60
for t in fix_u11merge fix_u16_u11b; do
  $P scan3d/runs/fixes_20261112/render_merge.py "$L/$t/merge" "$O/${t}" > "$O/render_$t.log" 2>&1; echo "render $t rc $?"; tail -n 2 "$O/render_$t.log"
done
$P - "$L" > "$O/pose_diff.json" <<'EOF'
import gzip, json, sys
import numpy as np
L = sys.argv[1]
def ld(t):
    d = {}
    for l in gzip.open(f"{L}/{t}/merge/scan/ba/poses_ba.jsonl.gz", "rt"):
        m = json.loads(l); d[int(m["frame"])] = np.array(m["T_world_cam0"]).reshape(4, 4)[:3, 3]
    return d
def cmp(a, b):
    A, B = ld(a), ld(b); f = sorted(set(A) & set(B)); X = np.array([A[i] for i in f]); Y = np.array([B[i] for i in f])
    e = np.linalg.norm(X - Y, axis=1) * 100
    return dict(a=a, b=b, n=len(f), raw_rms_cm=float(np.sqrt((e ** 2).mean())), raw_median_cm=float(np.median(e)), raw_max_cm=float(e.max()))
print(json.dumps([cmp("fix_u16_x105", "fix_u16_u11b"), cmp("fix_u11merge", "fix_u16_u11b"), cmp("fix_u16_u11", "fix_u16_u11b")], indent=1))
EOF
cat "$O/pose_diff.json"
exit 0
