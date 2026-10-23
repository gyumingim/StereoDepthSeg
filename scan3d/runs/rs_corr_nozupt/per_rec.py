"""Per-recording table from score.json (RS comparison). Usage: python per_rec.py runs/rs_corr_nozupt/score.json"""
import json, sys
import numpy as np
s = json.load(open(sys.argv[1]))
R = s["runs"]
recs = sorted({r["rec"] for r in R.values()})
print("| rec | base ATE o0/o1/o2 | rs294 | rs221 | div base/294/221 |")
for rec in recs:
    row = []
    divs = []
    for c in ("base", "rs294", "rs221"):
        v = [R[f"{rec[-6:]}_o{o}_{c}"] for o in (0, 1, 2)]
        row.append(" / ".join("div" if x["div"] else f"{x['ate_w']:.2f}" for x in v))
        divs.append(sum(x["div"] for x in v))
    print(f"| {rec[-6:]} | {row[0]} | {row[1]} | {row[2]} | {divs[0]}/{divs[1]}/{divs[2]} |")
# paired ratio over normal cells (excl drop)
for c in ("rs294", "rs221"):
    ks = sorted({r["key"] for r in R.values() if "213955" not in r["key"]})
    ks = [k for k in ks if not R[f"{k}_base"]["div"] and not R[f"{k}_{c}"]["div"]]
    d = np.array([R[f"{k}_{c}"]["ate_w"] - R[f"{k}_base"]["ate_w"] for k in ks])
    print(c, "paired diff cm: median %.3f mean %.3f p10 %.3f p90 %.3f n %d" % (np.median(d), d.mean(), np.percentile(d, 10), np.percentile(d, 90), len(d)))
    flips = [k for k in {r["key"] for r in R.values()} if R[f"{k}_base"]["div"] != R[f"{k}_{c}"]["div"]]
    print(c, "div flips:", sorted(flips))
