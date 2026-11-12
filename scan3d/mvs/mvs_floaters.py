#!/usr/bin/env python3
"""mvs 11-12: floater estimate + oblique headless render for TSDF meshes (no window, no GPU).
Floaters = vertices in small connected mesh components (face connectivity, scipy csgraph). No ground truth: a small component can
also be a real small object (cable, cup); a large wrong surface is NOT caught. Reported at two sizes: < 500 and < 5000 vertices
(at 5 mm voxels about < 0.01 m2 and < 0.1 m2 of surface).
Render: one oblique view from above (45 deg, looking at the 1..99 % box centre of the first mesh), small components drawn red.
Usage: mvs_floaters.py OUT_PREFIX label=mesh.ply ...  -> OUT_floaters.json, OUT_oblique.jpg"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "runs" / "fixes_20261112"))
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
    assert (fr["n"] == 3).all(), "triangles only"
    X = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    C = np.stack([v["red"], v["green"], v["blue"]], 1)
    return X, C, fr["i"].astype(np.int64)


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
    out = sys.argv[1]
    res, ms = {}, []
    for s in sys.argv[2:]:
        lab, _, p = s.partition("=")
        X, C, F = read_mesh(p)
        n = len(X)
        # weld: the TSDF mesher duplicates vertices at block seams -> same position (0.1 mm grid) = same node
        _, wid = np.unique(np.round(X / 1e-4).astype(np.int64), axis=0, return_inverse=True)
        wid = wid.ravel()
        Fw = wid[F]
        nw = int(wid.max()) + 1
        e = np.concatenate([Fw[:, [0, 1]], Fw[:, [1, 2]]])
        g = coo_matrix((np.ones(len(e), np.int8), (e[:, 0], e[:, 1])), shape=(nw, nw))
        nc, lab_w = connected_components(g, directed=False)
        lab_v = lab_w[wid]
        sz = np.bincount(lab_v, minlength=nc)
        vsz = sz[lab_v]
        r = dict(path=p, vertices=int(n), welded_vertices=nw, faces=int(len(F)), components=int(nc),
                 largest_component_frac=float(sz.max() / n),
                 floater_frac_lt500=float(np.mean(vsz < 500)), floater_frac_lt5000=float(np.mean(vsz < 5000)),
                 components_lt500=int((sz < 500).sum()), components_lt5000=int((sz < 5000).sum()))
        res[lab] = r
        print(lab, json.dumps(r), flush=True)
        ms.append((lab, X, C, vsz < 5000))
    X0 = ms[0][1]
    lo, hi = np.percentile(X0, 1, 0), np.percentile(X0, 99, 0)
    ctr = (lo + hi) / 2; rad = float(np.linalg.norm(hi - lo)) / 2
    tiles = []
    for lab, X, C, fl in ms:
        Cr = C.copy(); Cr[fl] = (230, 0, 0)
        row = []
        for name, d in (("oblique SW", np.array([-0.6, -0.6, 0.8])), ("oblique NE", np.array([0.6, 0.6, 0.8]))):
            d = d / np.linalg.norm(d)
            eye = ctr + d * rad * 1.6
            R = look(eye, ctr)
            t = render(X, Cr, R, eye, 700, 1000, 700)
            cv2.putText(t, f"{lab} | {name} | red = comp < 5000 v ({fl.mean() * 100:.1f} %)", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 200), 2)
            row.append(t)
        tiles.append(np.hstack(row))
    cv2.imwrite(out + "_oblique.jpg", cv2.resize(np.vstack(tiles), None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 85])
    Path(out + "_floaters.json").write_text(json.dumps(dict(made=__import__("time").strftime("%Y-%m-%d %H:%M:%S"), inputs=sys.argv[2:], results=res), indent=1))
    print("wrote", out + "_oblique.jpg", out + "_floaters.json")


if __name__ == "__main__":
    main()
