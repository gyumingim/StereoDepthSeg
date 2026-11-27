#!/bin/bash
# dry run 30 fps extra (2026-11-27): same 30 fps VIO, keyframe window 2 frames (= 67 ms like 4 frames @ 60 fps), then pairs2 plans
set -uo pipefail
D=/home/system/scanner/StereoDepthSeg-gsmap/scan3d/runs/dryrun30_121801_20261127
RS=/home/system/scanner/StereoDepthSeg-fixes/scan3d/scanA/run_scanner.sh
R30=$D/rec_hs_20261112_121801_std30
nice -n 19 bash $RS $R30 $D/scan30_kfwin2 --vio $D/scan30/vio/B_o0.poses.jsonl.gz --stop-after reduce --kf-win 2 > $D/scan30_kfwin2.log 2>&1; echo "reduce rc=$?"
cd /home/system/scanner/StereoDepthSeg-gsmap
/home/system/scanner/gsmap_env/venv/bin/python - <<'EOF'
import json, sys, importlib.util
from pathlib import Path
D = Path("scan3d/runs/dryrun30_121801_20261127").resolve()
spec = importlib.util.spec_from_file_location("dr", "/home/system/scanner/StereoDepthSeg-gsmap/scan3d/gsmap/dryrun30/dryrun30.py")
sys.argv = ["x", str(D)]; dr = importlib.util.module_from_spec(spec); spec.loader.exec_module(dr)
R30 = D / "rec_hs_20261112_121801_std30"; V30 = D / "scan30/vio/B_o0.poses.jsonl.gz"
red = D / f"scan30_kfwin2/red/{R30.name}_kf"
run, k = dr.make_cams("30_kfwin2", red, R30, V30)
out = dict(made=__import__("time").strftime("%F %T"), note="30 fps, --kf-win 2 (window 67 ms as 4 frames @ 60 fps), same 30 fps VIO", keyframes=k, plans={})
for name, ex in (("A30w2_vio_legacy", dr.LEG), ("A30w2_vio_new", dr.LEG + ["--profile", "new"])):
    o = D / "plans" / name; dt, notes = dr.plan(run, R30, V30, o, ex); s = dr.summ(o / "plan.json"); s["notes"] = notes; out["plans"][name] = s
(D / "dryrun30_kfwin2.json").write_text(json.dumps(out, indent=1, default=float))
print(json.dumps({"n_kf": k["n_kf"], "moved": k["kf_select"]["moved_m_median"], "rot": k["kf_select"]["rot_deg_median"],
                  **{n: {x: v[x] for x in ("ge1_pct", "ge2_pct", "zero_pct", "fallback_kf", "pairs", "ge1", "ge2", "zero")} for n, v in out["plans"].items()}}, indent=1))
EOF
