"""review 11-12 finding 2: per-keyframe table of fused-depth differences between runs (same keyframes, same poses).
python kf283_table.py DUMP_ROOT OUT.json name_a:name_b [name_a:name_b ...]   (DUMP_ROOT/<name>/kf_XXXX.npz from scan_offline --dump-depth)
Per keyframe and pair: px valid in both, frac |dZ| > 10 / 50 mm, p95 |dZ| mm, median depth of the > 50 mm px, only-A / only-B px,
same partners (y/n)."""
import json
import sys
import time
from pathlib import Path

import numpy as np

root, outp, pairs = Path(sys.argv[1]), Path(sys.argv[2]), [p.split(":") for p in sys.argv[3:]]
res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), root=str(root), pairs={})
for a, b in pairs:
    rows = []
    for fa in sorted((root / a).glob("kf_*.npz")):
        fb = root / b / fa.name
        if not fb.exists():
            continue
        A, B = np.load(fa), np.load(fb)
        za, zb = A["Z01mm"].astype(np.float64) * 1e-4, B["Z01mm"].astype(np.float64) * 1e-4
        va, vb = za > 0, zb > 0
        both = va & vb
        d = np.abs(za[both] - zb[both]) * 1e3
        r = dict(kf=fa.stem, n_both=int(both.sum()), only_a=int((va & ~vb).sum()), only_b=int((vb & ~va).sum()),
                 same_partners=bool(np.array_equal(A["partners"], B["partners"])), partners_a=A["partners"].tolist(),
                 partners_b=B["partners"].tolist())
        if len(d) > 100:
            r.update(frac_gt10=float(np.mean(d > 10)), frac_gt50=float(np.mean(d > 50)), p95_mm=float(np.percentile(d, 95)),
                     med_mm=float(np.median(d)), z_med_gt50_m=float(np.median(za[both][d > 50])) if (d > 50).any() else None,
                     bias_mm=float(np.median((za[both] - zb[both]) * 1e3)))
        rows.append(r)
    res["pairs"][f"{a}_vs_{b}"] = rows
    print(f"== {a} vs {b}")
    print(" kf       n_both   >10mm  >50mm  p95mm  med_mm  z(>50)  onlyA  onlyB  same_partners")
    for r in rows:
        if "p95_mm" in r:
            print(f" {r['kf']}  {r['n_both']:7d}  {r['frac_gt10']:5.1%}  {r['frac_gt50']:5.1%}  {r['p95_mm']:6.1f}  {r['med_mm']:5.2f}  "
                  f"{r['z_med_gt50_m'] or 0:5.2f}  {r['only_a']:6d} {r['only_b']:6d}  {r['same_partners']}")
outp.write_text(json.dumps(res, indent=1))
