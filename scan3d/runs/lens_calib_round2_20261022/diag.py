"""Quick diagnostics of collected views: board scale distribution (mirror check), corner-ID counts, sharpness."""
import pickle, sys
from pathlib import Path
import numpy as np
D = Path(__file__).resolve().parent
for p in sorted((D / "views").glob("*.pkl")):
    P = pickle.load(open(p, "rb"))
    V = P["views"]
    if not V:
        print(p.stem, "no views"); continue
    sq = []
    for v in V:
        o = v["obj"][:, :2].astype(np.float64); im = v["img"].reshape(-1, 2).astype(np.float64)
        # scale = median px per m between neighbouring corners
        d_o = np.linalg.norm(o[:, None] - o[None], axis=2); d_i = np.linalg.norm(im[:, None] - im[None], axis=2)
        m = (d_o > 0.06) & (d_o < 0.066)
        sq.append(np.median(d_i[m] / d_o[m]) * 0.063294 if m.any() else np.nan)
    sq = np.array(sq)
    sh = np.array([v["sharp"] for v in V]); nc = np.array([len(v["ids"]) for v in V])
    tl = np.array([v["tilt"] for v in V])
    print(f"{p.stem}: views {len(V)} sq_px p1/p10/p50/p90 {np.nanpercentile(sq,[1,10,50,90]).round(0)} "
          f"min {np.nanmin(sq):.0f}; sharp p10/50 {np.percentile(sh,[10,50]).round(0)}; corners med {np.median(nc)}; "
          f"tilt p50/p90/max {np.percentile(tl,[50,90]).round(1)} {tl.max():.1f}; dist0 med {np.median([v['dist_m0'] for v in V]):.2f}")
    print("   corner id counts:", P["corner_id_count"])
    small = np.where(sq < 0.5 * np.nanmedian(sq))[0]
    if len(small):
        print("   small-scale views:", [(V[k]["fi"], round(sq[k]), round(V[k]["cx"]), round(V[k]["cy"])) for k in small[:20]])
