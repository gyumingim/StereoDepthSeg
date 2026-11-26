#!/usr/bin/env python3
"""gsmap 2026-11-27 W8: paired per-frame differences from the R-ruler photo run (photo_perframe.npz) vs two baselines
(nC = the published GLB, B0 = the same chain re-baked by gsm_texture3 with no option). se = std(diff)/sqrt(n) over frames, n = 1 recording.
  python gsm_w8table.py RJUDGE_DIR OUT.json"""
import json, sys, time
from pathlib import Path
import numpy as np
d = Path(sys.argv[1]); z = np.load(d / "photo_perframe.npz"); R, maps, cols, kind = z["R"], list(z["maps"]), list(z["cols"]), z["kind"]
res = dict(made=time.strftime("%F %T"), inp=str(d / "photo_perframe.npz"), photo_json=json.loads((d / "photo.json").read_text())["inputs"], diffs={})
for k in ("partner", "kf"):
    sel = kind == k; res["diffs"][k] = {}
    for base in ("nC", "B0"):
        for m in maps:
            if m == base:
                continue
            e = {}
            for c in ("psnr_common", "psnr_common_colourfit", "ssim_common", "psnr_hit", "hit_share"):
                ci = cols.index(c); x = R[sel, maps.index(m), ci] - R[sel, maps.index(base), ci]; x = x[np.isfinite(x)]
                e[c] = dict(diff=round(float(x.mean()), 4), se=round(float(x.std(ddof=1) / np.sqrt(len(x))), 4), n=int(len(x)),
                            halves=[round(float(x[0::2].mean()), 4), round(float(x[1::2].mean()), 4)])
            res["diffs"][k][f"{m}-{base}"] = e
    res.setdefault("abs", {})[k] = {m: {c: round(float(np.nanmean(R[sel, maps.index(m), cols.index(c)])), 4) for c in cols} for m in maps}
Path(sys.argv[2]).write_text(json.dumps(res, indent=1))
for k in ("partner",):
    for p, e in res["diffs"][k].items():
        print(k, p, {c: (v["diff"], v["se"]) for c, v in e.items() if c in ("psnr_common", "ssim_common", "psnr_common_colourfit")})
