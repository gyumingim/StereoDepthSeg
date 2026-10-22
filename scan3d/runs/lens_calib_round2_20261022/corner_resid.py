"""Mirror-window check: per-corner-ID reprojection residual (own k1k2p_k3 fit, selected views) for each recording.
A corner hidden/corrupted by the mirror window would show a larger residual or a mean offset."""
import json, sys
import numpy as np, cv2
from common import D, S3, load_views, spread, SIZE
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import lens_calib as lc
for r in sys.argv[1:]:
    v, _ = load_views(r)
    pick, _ = spread(v, 160)
    sel = [v[k] for k in pick]
    rms, K, d, rv, tv = lc.fit([(x["obj"], x["img"], 0, 0, 0) for x in sel], SIZE, lc.MODELS["k1k2p_k3"])
    e = {}
    for x, a, b in zip(sel, rv, tv):
        p, _ = cv2.projectPoints(x["obj"], a, b, K, d)
        for cid, q in zip(x["ids"], (p - x["img"]).reshape(-1, 2)):
            e.setdefault(int(cid), []).append(q)
    rows = []
    for cid in range(32):
        q = np.array(e.get(cid, np.zeros((0, 2))))
        rows.append((cid, len(q), float(np.sqrt((q ** 2).sum(1).mean())) if len(q) else np.nan,
                     q.mean(0).round(2).tolist() if len(q) else None))
    worst = sorted(rows, key=lambda t: -np.nan_to_num(t[2]))[:4]
    print(r, f"rms {rms:.3f}", "corner 24:", rows[24], "corner 16:", rows[16], "worst:", worst)
