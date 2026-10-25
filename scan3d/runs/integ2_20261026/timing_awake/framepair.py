#!/usr/bin/env python3
"""Frame-matched N-O difference (same raw80 frames, matched by stream t) for one run pair; bootstrap 95% CI over frames
(within-pair precision only - says nothing about run-to-run noise)."""
import csv, sys, numpy as np
def load(p): return {r["t"]: float(r["feed_ms"]) for r in csv.DictReader(open(p)) if r["init"] == "1"}
a, b = load(sys.argv[1]), load(sys.argv[2]); k = sorted(set(a) & set(b))
d = np.array([a[x] - b[x] for x in k]); rng = np.random.default_rng(0)
bs = [np.median(rng.choice(d, len(d))) for _ in range(2000)]; bm = [rng.choice(d, len(d)).mean() for _ in range(2000)]
print(f"{sys.argv[1]} - {sys.argv[2]}: frames {len(k)}, median diff {np.median(d):+.3f} ms CI [{np.percentile(bs,2.5):+.3f},{np.percentile(bs,97.5):+.3f}], mean diff {d.mean():+.3f} CI [{np.percentile(bm,2.5):+.3f},{np.percentile(bm,97.5):+.3f}], frames N faster {100*(d<0).mean():.0f}%")
