#!/usr/bin/env python3
"""fixes 11-12 unit 14: baseline vs --far-pairs wide mesh (same keyframes + BA poses). No ground truth: far range = coverage only,
near range = agreement with the baseline. Metric rules copied from mvs-20261112 scan3d/mvs/mvs_eval.py + mvs_floaters.py (no import):
  * camera depth of a vertex = nearest integrated keyframe camera (BASE/kf_poses.npz) that has it inside its 960x540 view
    (no occlusion test -> shorter than the real viewing depth); counts > 1.6 / > 2.5 m
  * top-view area = 5 cm cells (z up) holding vertices; also only cells of vertices > 1.6 / > 2.5 m, and cells new vs the baseline
  * near agreement: vertices with camera depth < 1.2 m -> nearest vertex of the other mesh (both directions), median / p90 mm
  * floaters = vertices in small connected components (< 500 / < 5000 welded vertices); a small real object also counts
Renders (headless numpy, no window): OUT_top.png, OUT_kfviews.jpg (photo | base | far for the 3 keyframes with most added far
pixels), OUT_oblique.jpg (small components red).
Usage: far_eval.py BASE_DIR FAR_DIR REC OUT_PREFIX"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_mesh import look  # noqa: E402


def read_mesh(p):
    with open(p, "rb") as f:
        hdr = b""
        while not hdr.endswith(b"end_header\n"):
            hdr += f.readline()
        h = hdr.decode()
        nv = int(h.split("element vertex ")[1].split()[0])
        nf = int(h.split("element face ")[1].split()[0]) if "element face" in h else 0
        vb = h.split("element vertex")[1].split("element")[0].splitlines()
        props = [l.split()[-1] for l in vb if l.startswith("property")]
        types = [l.split()[1] for l in vb if l.startswith("property")]
        tmap = {"float": "<f4", "uchar": "u1", "double": "<f8", "int": "<i4"}
        dt = np.dtype([(n, tmap[t]) for n, t in zip(props, types)])
        v = np.frombuffer(f.read(nv * dt.itemsize), dt, nv)
        fr = np.frombuffer(f.read(nf * 13), np.dtype([("n", "u1"), ("i", "<i4", 3)]), nf)
    X = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    C = np.stack([v["red"], v["green"], v["blue"]], 1)
    return X, C, fr["i"].astype(np.int64)


def cam_depth(X, T, f, cx, cy, W, H):
    best = np.full(len(X), np.inf, np.float32)
    Xf = X.astype(np.float32)
    for Tw in T:
        R = Tw[:3, :3].astype(np.float32); t = Tw[:3, 3].astype(np.float32)
        pc = (Xf - t) @ R
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pc[:, 0] / z * f + cx; v = pc[:, 1] / z * f + cy
        inn = (z > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        best = np.where(inn, np.minimum(best, z), best)
    return best


def comp_size(X, F):
    _, wid = np.unique(np.round(X / 1e-4).astype(np.int64), axis=0, return_inverse=True)
    wid = wid.ravel()
    Fw = wid[F]
    nw = int(wid.max()) + 1
    e = np.concatenate([Fw[:, [0, 1]], Fw[:, [1, 2]]])
    g = coo_matrix((np.ones(len(e), np.int8), (e[:, 0], e[:, 1])), shape=(nw, nw))
    nc, lab_w = connected_components(g, directed=False)
    sz = np.bincount(lab_w, minlength=nc)
    return sz[lab_w[wid]], nc, sz


def cells(X):
    return set(map(tuple, np.unique(np.floor(X[:, :2] / 0.05).astype(np.int64), axis=0))) if len(X) else set()


def top(X, C, lo, hi, res, cams, zsel=None, gray=False, hl=None):
    W, H = int((hi[0] - lo[0]) / res) + 1, int((hi[1] - lo[1]) / res) + 1
    img = np.full((H, W, 3), 255, np.uint8)
    m = np.ones(len(X), bool) if zsel is None else (X[:, 2] >= zsel[0]) & (X[:, 2] <= zsel[1])
    x, c = X[m], C[m]
    u = ((x[:, 0] - lo[0]) / res).astype(int); v = (H - 1 - (x[:, 1] - lo[1]) / res).astype(int)
    k = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if gray:
        cnt = np.zeros((H, W)); np.add.at(cnt, (v[k], u[k]), 1)
        g = np.clip(255 - cnt * 25, 0, 255).astype(np.uint8); img = np.dstack([g, g, g])
        if hl is not None:      # vertices with camera depth > 1.6 m in this slice: blue
            hm = hl[m] & k
            img[v[hm], u[hm]] = (200, 80, 0)
    else:
        o = np.argsort(x[k, 2])
        img[v[k][o], u[k][o]] = c[k][o][:, ::-1]
    for cxy in cams:
        cv2.circle(img, (int((cxy[0] - lo[0]) / res), int(H - 1 - (cxy[1] - lo[1]) / res)), 2, (0, 0, 255), -1)
    return img


def splat(X, C, Tw, f, cx, cy, W, H, zmax=12.0):
    pc = (X - Tw[:3, 3]) @ Tw[:3, :3]
    z = pc[:, 2]
    ok = (z > 0.1) & (z < zmax)
    u = (pc[ok, 0] / z[ok] * f + cx).astype(int); v = (pc[ok, 1] / z[ok] * f + cy).astype(int)
    zz, cc = z[ok], C[ok][:, ::-1]
    img = np.full((H, W, 3), 255, np.uint8)
    o = np.argsort(-zz)                              # far first, near overwrite (painter's z-buffer)
    for du in (0, 1):
        for dv in (0, 1):
            uu, vv = u[o] + du, v[o] + dv
            m = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
            img[vv[m], uu[m]] = cc[o][m]
    return img


def render(X, C, R, eye, f, W, H):
    pc = (X - eye) @ R.T
    ok = pc[:, 2] > 0.05
    pc, Cc = pc[ok], C[ok]
    u = (pc[:, 0] / pc[:, 2] * f + W / 2).astype(int); v = (pc[:, 1] / pc[:, 2] * f + H / 2).astype(int)
    img = np.full((H, W, 3), 255, np.uint8)
    o = np.argsort(-pc[:, 2])
    for du in (0, 1):
        for dv in (0, 1):
            uu, vv = u[o] + du, v[o] + dv
            m = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
            img[vv[m], uu[m]] = Cc[o][m][:, ::-1]
    return img


def main():
    base, far_, rec, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), sys.argv[4]
    P = np.load(base / "kf_poses.npz")
    T = P["T"]; f, cx, cy = float(P["f"]), float(P["cx"]), float(P["cy"]); W, H = (int(x) for x in P["size"])
    res, M = {}, {}
    for lab, d in (("base", base), ("far", far_)):
        X, C, F = read_mesh(d / "mesh.ply")
        cd = cam_depth(X, T, f, cx, cy, W, H)
        fin = np.isfinite(cd)
        vsz, nc, sz = comp_size(X, F)
        g16, g25 = fin & (cd > 1.6), fin & (cd > 2.5)
        S = json.loads((d / "summary.json").read_text())
        r = dict(path=str(d / "mesh.ply"), vertices=int(len(X)), faces=int(len(F)),
                 cam_depth_m=dict(p50=float(np.median(cd[fin])), p90=float(np.percentile(cd[fin], 90)), p99=float(np.percentile(cd[fin], 99)),
                                  max=float(cd[fin].max()), frac_not_in_view=float(np.mean(~fin))),
                 v_gt_1p6=int(g16.sum()), v_gt_2p0=int((fin & (cd > 2.0)).sum()), v_gt_2p5=int(g25.sum()),
                 top_area_m2=round(len(cells(X)) * 0.0025, 2), top_area_gt_1p6_m2=round(len(cells(X[g16])) * 0.0025, 2),
                 top_area_gt_2p5_m2=round(len(cells(X[g25])) * 0.0025, 2),
                 components=int(nc), floater_frac_lt500=float(np.mean(vsz < 500)), floater_frac_lt5000=float(np.mean(vsz < 5000)),
                 floater_v_lt500=int((vsz < 500).sum()), floater_v_lt5000=int((vsz < 5000).sum()),
                 components_lt500=int((sz < 500).sum()), components_lt5000=int((sz < 5000).sum()),
                 gt_1p6_floater_frac_lt500=float(np.mean(vsz[g16] < 500)) if g16.any() else None,
                 gt_1p6_floater_frac_lt5000=float(np.mean(vsz[g16] < 5000)) if g16.any() else None,
                 runtime_s=S.get("runtime_s"), ffs_s_total=S.get("ffs_s_total"), far_pairs=S.get("far_pairs"),
                 tsdf_blocks=S.get("tsdf_blocks"), tsdf_full=S.get("tsdf_full"), ffs_backend=S.get("ffs_backend"))
        res[lab] = r
        M[lab] = (X, C, cd, vsz)
        print(lab, json.dumps(r), flush=True)
    Xb, _, cdb, _ = M["base"]; Xf, _, cdf, _ = M["far"]
    res["far"]["top_area_new_vs_base_m2"] = round(len(cells(Xf) - cells(Xb)) * 0.0025, 2)
    nb, nf = np.isfinite(cdb) & (cdb < 1.2), np.isfinite(cdf) & (cdf < 1.2)
    tb, tf = cKDTree(Xb), cKDTree(Xf)
    d_fb, _ = tb.query(Xf[nf], k=1, workers=-1)
    d_bf, _ = tf.query(Xb[nb], k=1, workers=-1)
    res["near_lt_1p2m"] = dict(base_vertices=int(nb.sum()), far_vertices=int(nf.sum()),
                               far_to_base_median_mm=float(np.median(d_fb) * 1000), far_to_base_p90_mm=float(np.percentile(d_fb, 90) * 1000),
                               far_to_base_frac_gt_2cm=float(np.mean(d_fb > 0.02)),
                               base_to_far_median_mm=float(np.median(d_bf) * 1000), base_to_far_p90_mm=float(np.percentile(d_bf, 90) * 1000),
                               base_covered_frac_2cm=float(np.mean(d_bf <= 0.02)), base_to_far_exact_frac_0p1mm=float(np.mean(d_bf < 1e-4)))
    # 1.2-1.6 m band: the wide pair can replace near-pair pixels here (only where the two agree within 3 sigma)
    mb = np.isfinite(cdb) & (cdb >= 1.2) & (cdb < 1.6)
    if mb.any():
        d_m, _ = tf.query(Xb[mb], k=1, workers=-1)
        res["band_1p2_1p6m"] = dict(base_vertices=int(mb.sum()), base_to_far_median_mm=float(np.median(d_m) * 1000),
                                    base_to_far_p90_mm=float(np.percentile(d_m, 90) * 1000))
    print("near", json.dumps(res["near_lt_1p2m"]), json.dumps(res.get("band_1p2_1p6m")), flush=True)
    # renders
    allX = np.concatenate([Xb[::4], Xf[::4]])
    lo = np.percentile(allX[:, :2], 0.5, 0) - 0.3; hi = np.percentile(allX[:, :2], 99.5, 0) + 0.3
    floor = np.percentile(Xb[:, 2], 2); ceil = np.percentile(Xb[:, 2], 97)
    camxy = T[:, :3, 3][:, :2]
    rows = []
    for lab in ("base", "far"):
        X, C, cd, _ = M[lab]
        m = X[:, 2] < ceil + 0.3
        t1 = top(X[m], C[m], lo, hi, 0.01, camxy)
        t2 = top(X, C, lo, hi, 0.01, camxy, zsel=(floor + 0.4, floor + 1.2), gray=True, hl=np.isfinite(cd) & (cd > 1.6))
        cv2.putText(t1, f"{lab}: top colour ({len(X)} v)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
        cv2.putText(t2, "slice 0.4-1.2 m above floor, blue = cam depth > 1.6 m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 200), 2)
        rows.append(np.hstack([t1, np.zeros((t1.shape[0], 8, 3), np.uint8), t2]))
    img = np.vstack([np.vstack([r_, np.zeros((8, r_.shape[1], 3), np.uint8)]) for r_ in rows])
    sc = min(1.0, 2400 / img.shape[1])
    cv2.imwrite(out + "_top.png", cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
    # keyframe views: 3 keyframes (>= 40 rows apart) with the most added far pixels
    kfs = json.loads((far_ / "keyframes.json").read_text())
    kfs = sorted([r for r in kfs if r.get("far_add")], key=lambda r: -r["far_add"])
    picks, used = [], []
    for r in kfs:
        if all(abs(r["i"] - u) >= 40 for u in used):
            picks.append(r); used.append(r["i"])
        if len(picks) >= 3:
            break
    g = np.load(rec / "lensonly" / "gt.npz")
    K = g["K"].copy(); K[:2] *= 0.5
    dist = g["dist"].astype(np.float64)
    Kr = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1.0]])
    und = cv2.initUndistortRectifyMap(K, dist, np.eye(3), Kr, (W, H), cv2.CV_32FC1)
    ii = [int(x) for x in P["i"]]
    kv = []
    for r in picks:
        Tw = T[ii.index(r["i"])]
        ph = cv2.imread(str(rec / "frames_kf" / f"{r['frame']:05d}.jpg"))
        ph = cv2.remap(cv2.resize(ph, (W, H), interpolation=cv2.INTER_AREA), *und, cv2.INTER_LINEAR)
        cv2.putText(ph, f"kf row {r['i']} frame {r['frame']} photo", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        tiles = [ph]
        for lab in ("base", "far"):
            X, C, _, _ = M[lab]
            t_ = splat(X, C, Tw, f, cx, cy, W, H)
            cv2.putText(t_, lab, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            tiles.append(t_)
        kv.append(np.hstack(tiles))
    if kv:
        cv2.imwrite(out + "_kfviews.jpg", cv2.resize(np.vstack(kv), None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 85])
    # oblique, small components red
    lo3, hi3 = np.percentile(Xb, 1, 0), np.percentile(Xb, 99, 0)
    ctr = (lo3 + hi3) / 2; rad = float(np.linalg.norm(hi3 - lo3)) / 2
    tiles = []
    for lab in ("base", "far"):
        X, C, _, vsz = M[lab]
        Cr = C.copy(); fl = vsz < 5000; Cr[fl] = (230, 0, 0)
        row = []
        for name, d in (("oblique SW", np.array([-0.6, -0.6, 0.8])), ("oblique NE", np.array([0.6, 0.6, 0.8]))):
            d = d / np.linalg.norm(d)
            eye = ctr + d * rad * 1.8
            t = render(X, Cr, look(eye, ctr), eye, 700, 1000, 700)
            cv2.putText(t, f"{lab} | {name} | red = comp < 5000 v ({fl.mean() * 100:.1f} %)", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 200), 2)
            row.append(t)
        tiles.append(np.hstack(row))
    cv2.imwrite(out + "_oblique.jpg", cv2.resize(np.vstack(tiles), None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 85])
    res["kfviews_rows"] = [dict(i=r["i"], frame=r["frame"], far_add=r["far_add"], far_partner=r["far_partner"], far_B_cm=r["far_B_cm"]) for r in picks]
    Path(out + "_eval.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), base=str(base), far=str(far_), rec=str(rec),
                                                        results=res), indent=1))
    print("wrote", out + "_eval.json", out + "_top.png", out + "_kfviews.jpg", out + "_oblique.jpg")


if __name__ == "__main__":
    main()
