#!/usr/bin/env python3
"""redscreen 11-11 (b): red rectangle on the scanner mesh (TSDF 5 mm, vertex colours).
red faces (all 3 vertices red in HSV) -> largest connected red patch -> plane fit (SVD, 2-pass trim) -> faces rasterised in the plane
at 1 mm -> holes filled -> orientation from minAreaRect -> each edge from the MIDDLE part only (exclude EXCL at both ends, rounded
corners) as the outermost red pixel per row/column, line fit -> W/H = distance between opposite lines.
Sensitivity: red thresholds (loose / base / strict) x EXCL 0.10/0.15/0.20.
usage: measure_mesh.py MESH.ply OUT_DIR"""
import json, sys, time
from pathlib import Path
import cv2
import numpy as np
from scipy import ndimage

REF_W, REF_H = 1920 * 0.15729e-3, 1200 * 0.15729e-3


def read_ply(p):
    b = Path(p).read_bytes()
    h_end = b.index(b"end_header\n") + len(b"end_header\n")
    hdr = b[:h_end].decode()
    assert "binary_little_endian" in hdr, hdr[:200]
    nv = int(hdr.split("element vertex ")[1].split()[0]); nf = int(hdr.split("element face ")[1].split()[0])
    vt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    V = np.frombuffer(b, vt, nv, h_end)
    ft = np.dtype([("n", "u1"), ("i", "<i4", 3)])
    F = np.frombuffer(b, ft, nf, h_end + nv * vt.itemsize)
    assert (F["n"] == 3).all()
    return np.stack([V["x"], V["y"], V["z"]], 1).astype(float), np.stack([V["r"], V["g"], V["b"]], 1), F["i"].astype(np.int64)


def is_red(C, smin, vmin):
    hsv = cv2.cvtColor(C[None, :, ::-1].astype(np.uint8), cv2.COLOR_BGR2HSV)[0]
    return (((hsv[:, 0] <= 20) | (hsv[:, 0] >= 172)) & (hsv[:, 1] >= smin) & (hsv[:, 2] >= vmin))


def measure(X, C, F, smin, vmin, excl, res=1e-3):
    red = is_red(C, smin, vmin)
    fr = F[red[F].all(1)]
    # TSDF meshes are written per block (vertices not shared across blocks) -> no connectivity; select by the dominant red plane instead
    P = X[np.unique(fr)]
    m = np.median(P, 0); keep = np.ones(len(P), bool); nrm = None
    for tol in (0.10, 0.05, 0.03, 0.02):
        if keep.sum() < 50:
            break
        m = P[keep].mean(0); _, _, vt = np.linalg.svd(P[keep] - m)
        keep = np.abs((P - m) @ vt[2]) < tol
    dd = np.abs((X[fr] - m) @ vt[2])
    fr = fr[(dd < 0.02).all(1)]
    P = X[np.unique(fr)]
    keep = np.ones(len(P), bool)
    for _ in range(3):
        m = P[keep].mean(0); _, _, vt = np.linalg.svd(P[keep] - m); nrm = vt[2]
        d = (P - m) @ nrm; sd = d[keep].std(); keep = np.abs(d) < max(3 * sd, 0.003)
    plane_rms = float(np.sqrt(np.mean(d[keep] ** 2)))
    e1, e2 = vt[0], vt[1]
    T2 = lambda Y: np.stack([(Y - m) @ e1, (Y - m) @ e2], 1)
    tri = T2(X[fr].reshape(-1, 3)).reshape(-1, 3, 2)
    lo = tri.reshape(-1, 2).min(0) - 0.02
    sz = np.ceil((tri.reshape(-1, 2).max(0) + 0.02 - lo) / res).astype(int)
    img = np.zeros((sz[1], sz[0]), np.uint8)
    pts = np.round((tri - lo) / res * 16).astype(np.int32)
    cv2.fillPoly(img, list(pts), 1, lineType=cv2.LINE_8, shift=4)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    img = ndimage.binary_fill_holes(img).astype(np.uint8)
    nl, labi, st, _ = cv2.connectedComponentsWithStats(img, 8)
    img = (labi == 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))).astype(np.uint8)
    cs, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    (cx, cy), (w, h), ang = cv2.minAreaRect(max(cs, key=cv2.contourArea))
    # rotate so that the rectangle is axis-aligned
    M = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
    big_sz = int(np.hypot(*img.shape)) + 10
    M[:, 2] += [big_sz / 2 - cx, big_sz / 2 - cy]
    R = cv2.warpAffine(img * 255, M, (big_sz, big_sz), flags=cv2.INTER_LINEAR) > 127
    ys, xs = np.nonzero(R)
    x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    edges = {}
    rows = np.arange(int(y0 + excl * (y1 - y0)), int(y1 - excl * (y1 - y0)) + 1)
    cols = np.arange(int(x0 + excl * (x1 - x0)), int(x1 - excl * (x1 - x0)) + 1)
    L = np.array([np.flatnonzero(R[r]).min() if R[r].any() else np.nan for r in rows]); Rr = np.array([np.flatnonzero(R[r]).max() + 1 if R[r].any() else np.nan for r in rows])
    T_ = np.array([np.flatnonzero(R[:, c]).min() if R[:, c].any() else np.nan for c in cols]); B_ = np.array([np.flatnonzero(R[:, c]).max() + 1 if R[:, c].any() else np.nan for c in cols])

    def fitl(t, v):
        ok = np.isfinite(v); p = np.polyfit(t[ok], v[ok], 1); r = v[ok] - np.polyval(p, t[ok]); return p, float(np.sqrt(np.mean(r ** 2)))
    pl, rl = fitl(rows, L); pr, rr = fitl(rows, Rr); pt, rt = fitl(cols, T_); pb, rb = fitl(cols, B_)
    ym, xm = (y0 + y1) / 2, (x0 + x1) / 2
    a = (np.polyval(pr, ym) - np.polyval(pl, ym)) * res
    b = (np.polyval(pb, xm) - np.polyval(pt, xm)) * res
    W, H = (a, b) if a >= b else (b, a)
    return dict(W_mm=W * 1e3, H_mm=H * 1e3, diag_mm=float(np.hypot(W, H) * 1e3), sW=W / REF_W, sH=H / REF_H,
                s_mean=(W / REF_W + H / REF_H) / 2, plane_rms_mm=plane_rms * 1e3, n_red_faces=int(len(fr)),
                edge_rms_mm=dict(l=rl * res * 1e3, r=rr * res * 1e3, t=rt * res * 1e3, b=rb * res * 1e3),
                minAreaRect_mm=[w * res * 1e3, h * res * 1e3], smin=smin, vmin=vmin, excl=excl), (R, img)


def main():
    mesh, out = sys.argv[1], Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    X, C, F = read_ply(mesh)
    hsv = cv2.cvtColor(C[None, :, ::-1].astype(np.uint8), cv2.COLOR_BGR2HSV)[0]
    res = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), mesh=mesh, n_vertices=len(X), n_faces=len(F), variants=[])
    for smin, vmin, tag in ((120, 120, "loose"), (160, 160, "base"), (200, 190, "strict")):
        for excl in (0.10, 0.15, 0.20):
            r, (R, img) = measure(X, C, F, smin, vmin, excl)
            r["tag"] = tag; res["variants"].append(r)
            print(f"{tag} excl {excl}: W {r['W_mm']:.1f} H {r['H_mm']:.1f} s_mean {r['s_mean']:.4f} plane rms {r['plane_rms_mm']:.2f} mm "
                  f"edge rms {r['edge_rms_mm']}", flush=True)
            if tag == "base" and excl == 0.15:
                cv2.imwrite(str(out / "mesh_red_plane_1mm.png"), (R * 255).astype(np.uint8))
    s = [v["s_mean"] for v in res["variants"]]
    res["s_mean_range"] = [min(s), max(s)]
    (out / "mesh_measure.json").write_text(json.dumps(res, indent=1, default=float))


if __name__ == "__main__":
    main()
