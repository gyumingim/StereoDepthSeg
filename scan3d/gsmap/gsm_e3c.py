#!/usr/bin/env python3
"""gsmap 2026-11-25 E3c (plan section 6): floor (and ceiling) plane fill with a see-through guard. CPU.

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_e3c.py IN.ply OUT.ply [--cell 0.04] [--no-ceiling]
* plane normal = gravity = world y (reframed VIO frame, y down). Floor height = median y of the floor inliers over the whole room:
  near-horizontal vertices (normal within 25 deg of y) below the cameras (y > max camera y + 0.3); 1 cm histogram mode, inliers
  within +-3 cm of the mode, median. Ceiling the same above the cameras (y < min camera y - 0.2), topmost mode with >= 20 % of the
  largest peak.
* fill: 4 cm cells of the room polygon (eval2 ref) without mesh within 5 cm of the plane get 2 flat triangles ONLY if fewer than 3
  keyframes see through the cell centre (eval2 reference depth = gold FFS else 2DGS, > 5 cm behind the cell). No vertex snapping.
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_eval2 import cams, ref_depth  # noqa: E402

R = Path(os.environ.get("GSMAP_RUN", ""))


def mode_height(y, top=False):
    h, e = np.histogram(y, bins=np.arange(y.min(), y.max() + 0.01, 0.01))
    if top:
        ok = np.nonzero(h >= 0.2 * h.max())[0]; j = ok.min()
        while j + 1 < len(h) and h[j + 1] >= h[j]:
            j += 1
    else:
        j = int(np.argmax(h))
    m = (e[j] + e[j + 1]) / 2; inl = y[np.abs(y - m) < 0.03]
    return float(np.median(inl)), int(len(inl))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--cell", type=float, default=0.04)
    ap.add_argument("--no-ceiling", action="store_true"); ap.add_argument("--see-max", type=int, default=3)
    ap.add_argument("--ceil-y", type=float, default=None)
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time(); info = {}
    m = o3d.io.read_triangle_mesh(a.inp); m.compute_vertex_normals()
    V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64); VN = np.asarray(m.vertex_normals)
    kf, T, K, w, h = cams(); gold, ZM, AL = ref_depth()
    camy = T[:, 1, 3]; hz = np.abs(VN[:, 1]) > np.cos(np.radians(25))
    planes = {}
    yf, nf = mode_height(V[hz & (V[:, 1] > camy.max() + 0.3), 1]); planes["floor"] = yf; info["floor"] = dict(y_m=round(yf, 3), inliers=nf)
    if not a.no_ceiling:
        yc, nc = mode_height(V[hz & (V[:, 1] < camy.min() - 0.2), 1], top=True); planes["ceiling"] = yc; info["ceiling"] = dict(y_m=round(yc, 3), inliers=nc)
        if a.ceil_y is not None:   # fixed ceiling height (audit 2: DA3 normal histogram + photo-consistency sweep)
            planes["ceiling"] = a.ceil_y; info["ceiling"] = dict(y_m=a.ceil_y, inliers=nc, source="--ceil-y", mode_y=round(yc, 3))
    rp = json.loads((R / "eval2" / "ref" / "room_poly.json").read_text()); poly = np.array(rp["poly_xz"]); lo = poly.min(0)
    gsz = np.ceil((poly.max(0) - lo) / a.cell).astype(int) + 1; pm = np.zeros((gsz[1], gsz[0]), np.uint8)
    cv2.fillPoly(pm, [np.round((poly - lo) / a.cell).astype(np.int32)], 1)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; pts = np.concatenate([(P0 + P1 + P2) / 3, P0, (P0 + P1) / 2])
    newV, newF = [], []
    for nm, y0 in planes.items():
        q = pts[np.abs(pts[:, 1] - y0) < 0.05][:, [0, 2]]; g = np.floor((q - lo) / a.cell).astype(int)
        ok = (g[:, 0] >= 0) & (g[:, 0] < gsz[0]) & (g[:, 1] >= 0) & (g[:, 1] < gsz[1]); cov = np.zeros_like(pm); cov[g[ok, 1], g[ok, 0]] = 1
        ys, xs = np.nonzero((pm > 0) & (cov == 0))
        C = np.stack([lo[0] + (xs + 0.5) * a.cell, np.full(len(xs), y0), lo[1] + (ys + 0.5) * a.cell], 1).astype(np.float32)
        see = np.zeros(len(C), np.int32)
        for j in range(len(T)):
            Rm, t = T[j][:3, :3].astype(np.float32), T[j][:3, 3].astype(np.float32); Xc = (C - t) @ Rm; z = Xc[:, 2]; okz = z > 0.1
            u = np.floor(K[0, 0] * Xc[:, 0] / np.where(okz, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(okz, z, 1) + K[1, 2]).astype(np.int32)
            okz &= (u >= 0) & (u < w) & (v >= 0) & (v < h); idx = np.nonzero(okz)[0]
            gz = gold[j][v[idx], u[idx]].astype(np.float32); gm = ZM[j][v[idx], u[idx]].astype(np.float32); ga = AL[j][v[idx], u[idx]]
            r = np.where(gz > 0, gz, np.where(ga > 127, gm, np.nan))
            see[idx] += (r > z[idx] + 0.05).astype(np.int32)
        fill = see < a.see_max
        info[nm].update(coverage_before_pct=round(float((cov & pm).sum() / pm.sum() * 100), 2), empty_cells=int(len(xs)),
                        filled_cells=int(fill.sum()), filled_m2=round(float(fill.sum() * a.cell ** 2), 2), blocked_by_see_through=int((~fill).sum()))
        b0 = len(V) + sum(len(x) for x in newV); k_ = 0
        for x_, y_ in zip(xs[fill], ys[fill]):
            x0, z0 = lo[0] + x_ * a.cell, lo[1] + y_ * a.cell
            cs = np.array([[x0, y0, z0], [x0 + a.cell, y0, z0], [x0 + a.cell, y0, z0 + a.cell], [x0, y0, z0 + a.cell]])
            newV.append(cs); bb = b0 + 4 * k_; k_ += 1
            tri = [[bb, bb + 1, bb + 2], [bb, bb + 2, bb + 3]]; nt = np.cross(cs[1] - cs[0], cs[2] - cs[0])
            towards = np.array([0, -1.0, 0]) if nm == "floor" else np.array([0, 1.0, 0])
            newF += tri if nt @ towards > 0 else [t_[::-1] for t_ in tri]
    nf0 = len(F)
    if newV:
        V = np.concatenate([V] + newV); F = np.concatenate([F, np.array(newF, np.int64)])
    mo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    o3d.io.write_triangle_mesh(a.out, mo, write_ascii=False, write_vertex_normals=False)
    info.update(made=time.strftime("%F %T"), inp=a.inp, faces_in=int(nf0), generated_faces=int(len(F) - nf0), args=vars(a), runtime_s=round(time.time() - t0, 1))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
