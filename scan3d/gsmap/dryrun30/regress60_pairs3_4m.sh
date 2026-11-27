#!/bin/bash
# regression 2026-11-27: pairs3 + 4m plans on 121801 @ 60 fps with the OLD args after the fps change
cd "$HOME/scanner/StereoDepthSeg-gsmap"
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
R=scan3d/runs/gsmap_20261125_121801; V2=scan3d/runs/gsmap_20261125_121801_v2; D=scan3d/runs/dryrun30_121801_20261127
VIO=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz
P="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
mkdir -p $D/regress60_pairs3 $D/regress60_4m
$P scan3d/gsmap/gsm_ffs_pairs3.py plan $R --vio $VIO --rec $R/rec --p2 $V2/ffs_pairs2 --out $D/regress60_pairs3 --min-perp 40 --fb-perp 30 --dir-deg 10 --n-pairs 4 --k 2 > $D/regress60_pairs3/plan.log 2>&1; echo rc3=$?
$P scan3d/gsmap/gsm_ffs_4m.py plan $R --vio $VIO --rec $R/rec --p2 $V2/ffs_pairs2 --p3 $V2/ffs_pairs3 --out $D/regress60_4m --min-perp 40 --fb-perp 30 --dir-deg 10 --n-pairs 4 --k 2 > $D/regress60_4m/plan.log 2>&1; echo rc4=$?
$P - <<'EOF'
import json
D = "scan3d/runs/dryrun30_121801_20261127"; V2 = "scan3d/runs/gsmap_20261125_121801_v2"
out = {}
for name, ref, new in (("pairs3", f"{V2}/ffs_pairs3/plan3.json", f"{D}/regress60_pairs3/plan3.json"), ("4m", f"{V2}/ffs_4m/plan_4m.json", f"{D}/regress60_4m/plan_4m.json")):
    a, b = json.load(open(ref)), json.load(open(new))
    keys = [k for k in a if k not in ("made", "args")]
    out[name] = dict(made_ref=a["made"], same={k: a[k] == b.get(k) for k in keys})
print(json.dumps(out, indent=1))
json.dump(out, open(f"{D}/regress60_pairs3_4m_compare.json", "w"), indent=1)
EOF
