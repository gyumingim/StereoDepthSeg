#!/usr/bin/env python3
"""xfopt test verdict (PLAN 4, R1-R3; R4 = phone, separate). Run once after the test run; writes verdict.json.
Input: XFOPT_OUT=/home/system/scanner/tmp_xfopt/test score.json (xfopt_run.py score)."""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
os.environ.setdefault("XFOPT_OUT", "/home/system/scanner/tmp_xfopt/test")
import xfopt_run as XR  # noqa: E402

RD = Path(__file__).resolve().parent
TC = json.load(open(RD / "test_conds.json"))["conds"]
S = json.load(open(XR.OUT / "score.json"))["rows"]


def rows(cond, which):
    return {r["name"].rsplit("_", 1)[0]: r for k, r in S.items() if r["cond"] == cond and r["set"] == which}


def edge_counts(rs, deg=3.0, corner=True):
    n_chk = n_false = 0
    for r in rs.values():
        for e in r.get("edges", []):
            if not e.get("gt") or (corner and not e["gt_good"]):
                continue
            n_chk += 1
            n_false += e["err_cm"] > 10 or e["err_deg"] > deg
    return n_chk, n_false


out = dict(made=time.strftime("%F %T"), out=str(XR.OUT), plan_commits=sorted({r.get("plan_commit") for r in S.values()}),
           bridges=sorted({r.get("bridge") for r in S.values()}), conds={})
B = {w: rows("B60", w) for w in ("test", "std", "loop")}
for c in [x for x in TC if x != "B60"]:
    R = {w: rows(c, w) for w in ("test", "std", "loop")}
    d = dict(n_cells={w: len(R[w]) for w in R}, n_cells_B60={w: len(B[w]) for w in B})
    pool_c = dict(R["test"], **{"s" + k: v for k, v in R["std"].items()})
    pool_b = dict(B["test"], **{"s" + k: v for k, v in B["std"].items()})
    common = sorted(set(pool_c) & set(pool_b))
    d["cells_common"] = len(common)
    pc = {k: pool_c[k] for k in common}
    pb = {k: pool_b[k] for k in common}
    chk, fl = edge_counts(pc)
    chkb, flb = edge_counts(pb)
    d["R1"] = dict(checked=chk, false=fl, B60_checked=chkb, B60_false=flb, rate=fl / max(chk, 1), B60_rate=flb / max(chkb, 1),
                   passed=fl <= flb)
    d["sens"] = {f"{deg}deg": [edge_counts(pc, deg)[1], edge_counts(pb, deg)[1]] for deg in (2.0, 2.5, 3.5, 4.0)}
    d["sens"]["no_corner_rule"] = [edge_counts(pc, corner=False)[1], edge_counts(pb, corner=False)[1]]
    byrec = {}
    for k in common:
        rec = pc[k]["rec"]
        a = edge_counts({k: pc[k]})
        b = edge_counts({k: pb[k]})
        x = byrec.setdefault(rec, [0, 0, 0, 0])
        x[0] += a[0]; x[1] += a[1]; x[2] += b[0]; x[3] += b[1]
    d["by_rec_checked_false_vs_B60"] = byrec
    # R2 hand loops
    r2 = {}
    for k, r in R["loop"].items():
        b = B["loop"].get(k, {})
        r2[k] = dict(loops=r.get("n_loop"), B60_loops=b.get("n_loop"), map_cm=r.get("map_loop_cm"), B60_map_cm=b.get("map_loop_cm"),
                     vio_cm=r.get("vio_loop_cm"))
    app = [k for k, v in r2.items() if (v["B60_loops"] or 0) == 0]
    d["R2"] = dict(per=r2, applicable=app, passed=(any((r2[k]["loops"] or 0) >= 1 for k in app) if app else None))
    # R3 map ATE paired difference
    diffs = [pc[k]["map_ate_cm"] - pb[k]["map_ate_cm"] for k in common
             if pc[k].get("map_ate_cm") is not None and pb[k].get("map_ate_cm") is not None]
    d["R3"] = dict(n=len(diffs), median_diff_cm=float(np.median(diffs)) if diffs else None,
                   map_ate_median=float(np.median([pc[k]["map_ate_cm"] for k in common if pc[k].get("map_ate_cm") is not None])),
                   B60_map_ate_median=float(np.median([pb[k]["map_ate_cm"] for k in common if pb[k].get("map_ate_cm") is not None])),
                   passed=bool(diffs) and float(np.median(diffs)) <= 0.02)
    # repetition: offsets with the same init time / first pose = duplicate cells
    seen = {}
    for k in common:
        key = (pb[k]["rec"], pb[k].get("init_t_ns"), pb[k].get("pose0_sha"))
        seen.setdefault(key, []).append(k)
    d["duplicate_offset_cells"] = [v for v in seen.values() if len(v) > 1]
    # PLAN 4: duplicate offset cells (same init time + first pose) count once -> the verdict uses the deduplicated pool
    keep = sorted(v[0] for v in seen.values())
    pcd = {k: pc[k] for k in keep}
    pbd = {k: pb[k] for k in keep}
    c1, f1 = edge_counts(pcd)
    c2, f2 = edge_counts(pbd)
    dd = [pcd[k]["map_ate_cm"] - pbd[k]["map_ate_cm"] for k in keep
          if pcd[k].get("map_ate_cm") is not None and pbd[k].get("map_ate_cm") is not None]
    d["dedup"] = dict(cells=len(keep), R1=dict(checked=c1, false=f1, B60_checked=c2, B60_false=f2, passed=f1 <= f2),
                      R3=dict(n=len(dd), median_diff_cm=float(np.median(dd)) if dd else None,
                              passed=bool(dd) and float(np.median(dd)) <= 0.02),
                      sens={f"{deg}deg": [edge_counts(pcd, deg)[1], edge_counts(pbd, deg)[1]] for deg in (2.0, 2.5, 3.5)})
    kms = [r["kf_ms_p50_p95"][0] for r in pc.values() if r.get("kf_ms_p50_p95")]
    d["pc_kf_ms_p50_median"] = float(np.median(kms)) if kms else None
    d["R1_R3_pass"] = bool(d["dedup"]["R1"]["passed"] and d["dedup"]["R3"]["passed"] and d["R2"]["passed"] is not False)
    out["conds"][c] = d
json.dump(out, open(RD / "verdict.json", "w"), indent=1)
for c, d in out["conds"].items():
    print(c, "PASS R1-R3:", d["R1_R3_pass"], "dedup", d["dedup"], "\n  R1", d["R1"], "\n  R2", d["R2"]["passed"], d["R2"]["per"], "\n  R3", d["R3"], "\n  sens", d["sens"], "\n  dup", d["duplicate_offset_cells"])
