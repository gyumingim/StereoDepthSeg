# featloop: why few tracks with XFeat+LG? component sizes of the union-find over verified inlier matches (CPU, reads db.db)
import sys, numpy as np, pycolmap
from collections import Counter
sys.path.insert(0, "/home/system/scanner/StereoDepthSeg-featloop/scan3d/scanA")
from kf_ba import UF
for d in sys.argv[1:]:
    db = pycolmap.Database.open(d + "/db.db"); ids, g = db.read_two_view_geometries(); db.close()
    uf = UF()
    for pid, gm in zip(ids, g):
        i1, i2 = pid if isinstance(pid, tuple) else pycolmap.pair_id_to_image_pair(pid)
        mm = np.asarray(gm.inlier_matches)
        if len(mm) < 15: continue
        for x, y in mm: uf.u((i1, int(x)), (i2, int(y)))
    comp = {}
    for n in list(uf.p): comp.setdefault(uf.f(n), []).append(n)
    sz = np.array([len(v) for v in comp.values()]); bad = np.array([len(set(c[0] for c in v)) != len(v) for v in comp.values()])
    print(d.split("/")[-1], "components", len(sz), "kept", int((~bad & (sz >= 2)).sum()), "dropped(conflict)", int(bad.sum()),
          "nodes in dropped %.1f%%" % (100 * sz[bad].sum() / sz.sum()), "largest", sz.max(), "size p50/p90/p99 of dropped",
          np.percentile(sz[bad], [50, 90, 99]).round() if bad.any() else None)
