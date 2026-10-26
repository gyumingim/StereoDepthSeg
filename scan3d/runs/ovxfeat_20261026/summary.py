#!/usr/bin/env python3
"""ovxfeat: one table per condition from OUT/score.json (+ xf server logs): hand loops, std60 165506 (GT), HS 42 (GT).
False loop = edge vs GT > 10 cm or > 3 deg (ovloop rule); borderline = position 3-10 cm (reported separately).
  python3 summary.py [OUT] -> OUT/summary.json + RD/summary.json + printed table"""
import json
import statistics as st
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ovxfeat_run as R  # noqa: E402

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else R.OUT
S = json.load(open(OUT / "score.json"))["rows"]
_, DROP, _ = R.hs_cells()


def edges(rows):
    e = [x for r in rows for x in r.get("edges", []) if x.get("gt")]
    return dict(loops=sum(r.get("n_loop", 0) for r in rows), checked=len(e), false=sum(x["false"] for x in e),
                f_pos=sum(x["err_cm"] > 10 for x in e), f_rot=sum(x["err_deg"] > 3 for x in e),
                border=sum(3 < x["err_cm"] <= 10 for x in e),
                err_cm_p50=round(st.median([x["err_cm"] for x in e]), 2) if e else None,
                err_deg_p50=round(st.median([x["err_deg"] for x in e]), 2) if e else None,
                err_cm_max=max((x["err_cm"] for x in e), default=None), err_deg_max=max((x["err_deg"] for x in e), default=None))


def xf_ms(cond):
    fs, ms = [], []
    for f in (OUT / cond).glob("*.xf.jsonl"):
        for l in open(f):
            d = json.loads(l)
            (fs if d["op"] == "F" else ms).append(d["ms"])
    return dict(feat_ms_p50=round(st.median(fs), 1) if fs else None, match_ms_p50=round(st.median(ms), 1) if ms else None)


res = dict(made=time.strftime("%F %T"), out=str(OUT), conds={})
conds = sorted({v["cond"] for v in S.values()})
for c in conds:
    rows = {k: v for k, v in S.items() if v["cond"] == c}
    d = {}
    for k, v in rows.items():
        if v["set"] == "loop":
            d[v["name"].split("_")[0]] = dict(vio=v.get("vio_loop_cm"), map=v.get("map_loop_cm"), loops=v.get("n_loop"))
    std = [v for v in rows.values() if v["set"] == "std" and "165506" in v["rec"]]
    if std:
        d["std"] = dict(vio=[v.get("vio_ate_cm") for v in sorted(std, key=lambda v: v["name"])],
                        map=[v.get("map_ate_cm") for v in sorted(std, key=lambda v: v["name"])], **edges(std))
    hs = [v for v in rows.values() if v["set"] == "hs" and v["rec"].split("/")[-1] not in DROP]
    if hs:
        ok = [v for v in hs if v.get("vio_ate_cm") is not None and v.get("map_ate_cm") is not None]
        df = [v["map_ate_cm"] - v["vio_ate_cm"] for v in ok]
        d["hs"] = dict(cells=len(hs), scored=len(ok), vio_med=st.median(v["vio_ate_cm"] for v in ok),
                       map_med=st.median(v["map_ate_cm"] for v in ok), diff_med=round(st.median(df), 4),
                       diff_mean=round(st.mean(df), 4), better=sum(x < -0.005 for x in df), worse=sum(x > 0.005 for x in df), **edges(hs))
    d.update(xf_ms(c))
    res["conds"][c] = d
json.dump(res, open(OUT / "summary.json", "w"), indent=1)
(R.RD / ("summary.json" if OUT.name == "ate" else f"summary_{OUT.name}.json")).write_text(json.dumps(res, indent=1))
for c, d in res["conds"].items():
    a, b = d.get("l142148", {}), d.get("l142610", {})
    s = d.get("std", {})
    line = (f"{c:7s} | hand 142148 {a.get('map')} ({a.get('loops')}) 142610 {b.get('map')} ({b.get('loops')}) | std map {s.get('map')} "
            f"loops {s.get('loops')} false {s.get('false')} (rot {s.get('f_rot')}) border {s.get('border')} errmax {s.get('err_cm_max')}cm/{s.get('err_deg_max')}deg")
    h = d.get("hs")
    if h:
        line += (f" | HS map med {h['map_med']:.3f} (vio {h['vio_med']:.3f}) d_med {h['diff_med']:+.3f} {h['better']}/{h['worse']} loops {h['loops']} "
                 f"false {h['false']} (pos {h['f_pos']} rot {h['f_rot']}) border {h['border']} max {h['err_cm_max']}cm/{h['err_deg_max']}deg")
    line += f" | PC ms feat {d.get('feat_ms_p50')} match {d.get('match_ms_p50')}"
    print(line)
