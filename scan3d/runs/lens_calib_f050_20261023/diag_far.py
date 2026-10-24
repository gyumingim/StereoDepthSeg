"""Per-corner jitter of the two-marker FAR recordings (which corners jump?) (2026-10-24)."""
import pickle, sys
import numpy as np
for rec in sys.argv[1:]:
    d = pickle.load(open(f"views/{rec}.pkl", "rb"))
    fr = [f for f in d["frames"] if len(f[2]) == 8]
    P = np.stack([f[3] for f in fr])
    dif = np.linalg.norm(np.diff(P, axis=0), axis=2)
    med = np.median(P, 0)
    dev = np.linalg.norm(P - med, axis=2)
    print(rec[12:18], "n", len(fr), "per-corner motion p50", np.round(np.median(dif, 0), 2), "p90", np.round(np.quantile(dif, .9, 0), 2),
          "dev>2px frac", np.round((dev > 2).mean(0), 2), "marker side px", round(float(np.linalg.norm(med[0] - med[1])), 1))
