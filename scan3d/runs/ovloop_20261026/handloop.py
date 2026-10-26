#!/usr/bin/env python3
"""ovloop 2026-10-26: (1) HS summary without the DROP cells (= the 42-cell ruler of ov60eval), (2) what each place
recognition proposes on the hand loops (no GT): candidates, verification outcome counts, candidates that point back to
the first 20 % of the recording (= the start area the loop returns to) from the last 20 %.
  python3 handloop.py [OUT] -> OUT/handloop.json + RD/handloop.json"""
import collections
import json
import statistics as st
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ovloop_run as R  # noqa: E402

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else R.OUT
S = json.load(open(OUT / "score.json"))["rows"]
_, DROP, _ = R.hs_cells()
res = dict(made=time.strftime("%F %T"), out=str(OUT), drop=sorted(DROP), hs42={}, hand={})
for cond in ("B30", "B30w", "Q30", "S30"):
    X = [v for v in S.values() if v["cond"] == cond and v["set"] == "hs" and v["rec"].split("/")[-1] not in DROP]
    ok = [v for v in X if v.get("vio_ate_cm") is not None and v.get("map_ate_cm") is not None]
    d = [v["map_ate_cm"] - v["vio_ate_cm"] for v in ok]
    res["hs42"][cond] = dict(cells=len(X), scored=len(ok), vio_med=st.median(v["vio_ate_cm"] for v in ok),
                             map_med=st.median(v["map_ate_cm"] for v in ok), diff_med=st.median(d), diff_mean=round(st.mean(d), 4),
                             better=sum(x < -0.005 for x in d), worse=sum(x > 0.005 for x in d),
                             loops=sum(v["n_loop"] for v in X), checked=sum(v.get("edges_checked", 0) for v in X),
                             false=sum(v.get("edges_false", 0) for v in X),
                             edge_err_max=max((v.get("edge_err_cm_max") or 0) for v in X),
                             kf_ms_p50_med=st.median(v["kf_ms_p50_p95"][0] for v in X if v.get("kf_ms_p50_p95")))
    for lj in sorted((OUT / cond).glob("l*.loop.jsonl")):
        L = [json.loads(x) for x in open(lj)]
        kfs = [x for x in L if x.get("type") == "kf"]
        if not kfs:
            continue
        n = len(kfs)
        t0, t1 = kfs[0]["t"], kfs[-1]["t"]
        early = {k["idx"] for k in kfs if k["t"] <= t0 + 0.2 * (t1 - t0)}
        why = collections.Counter()
        back, back_ok, best_inl = 0, 0, 0
        for k in kfs:
            late = k["t"] >= t1 - 0.2 * (t1 - t0)
            for c in k["cand"]:
                why[c[4]] += 1
                if late and c[0] in early:
                    back += 1
                    back_ok += c[4] == "ok"
                    best_inl = max(best_inl, c[3])
        res["hand"][f"{cond}/{lj.name.split('.')[0]}"] = dict(n_kf=n, n_cand=sum(why.values()), why=dict(why),
                                                              late_to_start_cands=back, late_to_start_ok=back_ok,
                                                              late_to_start_max_inliers=best_inl,
                                                              start_kf_n3d=[k["n3d"] for k in kfs[:5]],
                                                              loops=sum(1 for x in L if x.get("type") == "loop"))
json.dump(res, open(OUT / "handloop.json", "w"), indent=1)
(R.RD / "handloop.json").write_text(json.dumps(res, indent=1))
for k, v in res["hs42"].items():
    print("hs42", k, v)
for k, v in res["hand"].items():
    print("hand", k, v)
