#!/bin/bash
# regression 2026-11-27: pairs2 plan on 121801 @ 60 fps with the OLD args (run_ffs_pairs2.sh plan) after the fps change
cd "$HOME/scanner/StereoDepthSeg-gsmap"
export OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=8 MKL_NUM_THREADS=8
R=scan3d/runs/gsmap_20261125_121801; O=scan3d/runs/dryrun30_121801_20261127/regress60_pairs2
mkdir -p $O
VIO=/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz
P="nice -n 19 $HOME/scanner/gsmap_env/venv/bin/python -W ignore"
date +"start %T"
$P scan3d/gsmap/gsm_ffs_pairs2.py plan $R --vio $VIO --rec $R/rec --out $O --min-perp 40 --fb-perp 30 --dir-deg 10 --n-pairs 4 --k 2 > $O/plan.log 2>&1; echo rc=$?
date +"end %T"
$P - <<'EOF'
import json
a = json.load(open("scan3d/runs/gsmap_20261125_121801_v2/ffs_pairs2/plan.json"))
b = json.load(open("scan3d/runs/dryrun30_121801_20261127/regress60_pairs2/plan.json"))
out = {"made_ref": a["made"], "made_new": b["made"], "rate": b.get("rate")}
for k in ("old_rule", "whole_video", "variants"):
    out[k + "_same"] = a[k] == b[k]
ca, cb = a["chosen"], b["chosen"]
out["chosen_same_all_keys"] = {k: ca[k] == cb.get(k) for k in ca}
out["new"] = {k: cb[k] for k in ("n_kf", "ge1", "ge2", "n_pairs_total")} | {"fallback": len(cb["fallback_kf"])}
out["ref"] = {k: ca[k] for k in ("n_kf", "ge1", "ge2", "n_pairs_total")} | {"fallback": len(ca["fallback_kf"])}
ka = set(a.keys()); kb = set(b.keys()); out["top_keys_added"] = sorted(kb - ka); out["top_keys_missing"] = sorted(ka - kb)
print(json.dumps(out, indent=1))
json.dump(out, open("scan3d/runs/dryrun30_121801_20261127/regress60_pairs2/compare.json", "w"), indent=1)
EOF
