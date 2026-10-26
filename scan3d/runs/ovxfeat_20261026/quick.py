#!/usr/bin/env python3
"""ovxfeat: quick per-cell summary from the loop logs (+ score.json if present).
  python3 quick.py [OUT] [cond,cond..] [set-prefix]"""
import collections
import json
import statistics as st
import sys
from pathlib import Path

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1] != "-" else Path("/home/system/scanner/tmp_ovxfeat/ate")
conds = sys.argv[2].split(",") if len(sys.argv) > 2 else sorted(p.name for p in OUT.iterdir() if p.is_dir() and p.name not in ("bin", "tmp"))
pref = sys.argv[3] if len(sys.argv) > 3 else ""
S = json.load(open(OUT / "score.json"))["rows"] if (OUT / "score.json").exists() else {}
for c in conds:
    for lj in sorted((OUT / c).glob(f"{pref}*.loop.jsonl")):
        name = lj.name.replace(".loop.jsonl", "")
        L = [json.loads(x) for x in open(lj)]
        K = [x for x in L if x.get("type") == "kf"]
        why = collections.Counter()
        nm = []
        xfn = []
        late = []  # candidates from the last 20 % of keyframes pointing to the first 20 %
        n = len(K)
        for k in K:
            for cd in k["cand"]:
                why[cd[4]] += 1
                nm.append(cd[2])
                if len(cd) > 10:
                    xfn.append(cd[10])
                if n and k["idx"] >= 0.8 * n and cd[0] <= 0.2 * n:
                    late.append((k["idx"], cd[0], cd[2], cd[3], cd[4], cd[10] if len(cd) > 10 else None))
        nl = sum(1 for x in L if x.get("type") == "loop")
        xs = [k["xf"] for k in K if "xf" in k]
        r = S.get(f"{c}/{name}", {})
        extra = ""
        if r.get("set") == "loop":
            extra = f"loop_err vio {r.get('vio_loop_cm')} -> map {r.get('map_loop_cm')} cm path {r.get('path_m')}"
        elif r:
            extra = (f"ate vio {r.get('vio_ate_cm')} -> map {r.get('map_ate_cm')} edges {r.get('edges_checked')} false {r.get('edges_false')} "
                     f"errmax {r.get('edge_err_cm_max')}")
        xinfo = ""
        if xs:
            xinfo = (f" xf kp med {st.median(x[0] for x in xs):.0f} 3d med {st.median(x[1] for x in xs):.0f} tri med {st.median(x[2] for x in xs):.0f}"
                     f" ms feat {st.median(x[3] for x in xs):.1f} trimatch {st.median(x[4] for x in xs):.1f}")
        print(f"{c:6s} {name:22s} kf {n} loops {nl} cand {sum(why.values())} {dict(why)} match med {st.median(nm) if nm else 0} "
              f"max {max(nm) if nm else 0}" + (f" allmatch med {st.median(xfn)}" if xfn else "") + xinfo + " | " + extra)
        if late and pref.startswith("l"):
            print("   end->start cands (cur, old, n3dmatch, inl, why, allmatch):", late[:12])
