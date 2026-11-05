#!/usr/bin/env python3
"""scalehunt: 165506 (the only live-equivalent GT recording) across all conditions: R2/R3 + R5 corner ruler -> s165506_conds.json"""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sh_score import score_cell, ROOT, ORDER  # noqa: E402

out = {}
print("cond      status  ATE    k_fwd   k_rev  | corner mv %  pair2v med % | mv reproj px")
for c in ORDER:
    f = ROOT / "runs" / c / "165506s.json"
    if not f.exists():
        continue
    v = score_cell(f, "hs_20261024_165506_std60")
    cj = ROOT / "corner" / f"{c}.json"
    cr = json.load(open(cj))["sources"][c] if cj.exists() else {}
    b = v.get("best", {})
    o = dict(status=v["status"], ate=b.get("ate_se3_cm"), k_fwd=b.get("k_fwd"), k_rev=b.get("k_rev"),
             corner_mv=round(cr.get("mv_scale_err_pct", float("nan")), 2),
             corner_pair_med=round(cr.get("pair2v_scale_err_pct", {}).get("median", float("nan")), 2),
             mv_reproj_med=round(cr.get("mv_reproj_px", {}).get("median", float("nan")), 2))
    out[c] = o
    print(f"{c:9s} {o['status']:6s} {o['ate']}  {o['k_fwd']}  {o['k_rev']} | {o['corner_mv']}  {o['corner_pair_med']} | {o['mv_reproj_med']}")
json.dump(out, open(HERE / "s165506_conds.json", "w"), indent=1)
