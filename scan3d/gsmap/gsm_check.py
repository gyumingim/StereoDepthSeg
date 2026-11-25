#!/usr/bin/env python3
"""gsmap 2026-11-25: checks for a textured GLB on a run folder.

  python scan3d/gsmap/gsm_check.py RUN_DIR GLB OUT.jpg [--views 5]
* render the GLB (Open3D ray cast, nearest texel) at evenly spaced keyframe poses next to the photo (960 dump images);
  held-out keyframes (index % 8 == 0) are preferred.
* double layer: for each face, ray from the nearest keyframe camera to the face centre; share of faces 1-10 cm behind the
  first hit; also the mesh area. Printed as JSON.
"""
import json, struct, sys
from pathlib import Path

import cv2
import numpy as np


def load_glb(p):
    b = open(p, "rb").read(); jl = struct.unpack("<I", b[12:16])[0]; j = json.loads(b[20:20 + jl]); bl = b[20 + jl + 8:]
    bv = j["bufferViews"]; g = lambda i: bl[bv[i]["byteOffset"]:bv[i]["byteOffset"] + bv[i]["byteLength"]]  # noqa: E731
    return (np.frombuffer(g(0), np.float32).reshape(-1, 3), np.frombuffer(g(1), np.float32).reshape(-1, 2),
            np.frombuffer(g(2), np.uint32).reshape(-1, 3), cv2.imdecode(np.frombuffer(g(3), np.uint8), 1))


def main():
    import open3d as o3d
    from scipy.spatial import cKDTree
    run, glb, outp = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
    nviews = int(sys.argv[sys.argv.index("--views") + 1]) if "--views" in sys.argv else 5
    c = np.load(run / "dump_ba" / "cams.npz"); T, K, kf = c["T"], c["Kr"], c["kf"]; w, h = [int(v) for v in c["size"]]
    V, UV, F, tex = load_glb(glb)
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F))
    uu, vv = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    test = np.arange(len(kf))[np.arange(len(kf)) % 8 == 0]
    pick = test[np.linspace(0, len(test) - 1, nviews).astype(int)]
    rows = []
    for j in pick:
        R, t = T[j][:3, :3], T[j][:3, 3]
        r = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, d.shape), d @ R.T], 1).astype(np.float32)))
        pid = r["primitive_ids"].numpy(); bu = r["primitive_uvs"].numpy(); hit = pid != 4294967295
        img = np.zeros((h * w, 3), np.uint8); p = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
        uv = b0[:, None] * UV[F[p, 0]] + b1[:, None] * UV[F[p, 1]] + b2[:, None] * UV[F[p, 2]]
        Ah, A = tex.shape[:2]
        img[hit] = tex[np.clip((uv[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uv[:, 0] * A).astype(int), 0, A - 1)]
        ph = cv2.imread(str(run / "dump_ba" / "img" / f"{kf[j]:05d}.png"))
        row = np.hstack([cv2.resize(ph, (480, 270)), cv2.resize(img.reshape(h, w, 3), (480, 270))])
        cv2.putText(row, f"photo kf {kf[j]} (held out from 2DGS)", (6, 18), 0, 0.5, (0, 255, 255), 1)
        cv2.putText(row, "textured mesh", (486, 18), 0, 0.5, (0, 255, 255), 1)
        rows.append(row)
    cv2.imwrite(outp, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
    P0, P1, P2 = V[F[:, 0]].astype(np.float64), V[F[:, 1]].astype(np.float64), V[F[:, 2]].astype(np.float64)
    ar = 0.5 * np.linalg.norm(np.cross(P1 - P0, P2 - P0), axis=1); cen = (P0 + P1 + P2) / 3
    cc = T[:, :3, 3]; _, jn = cKDTree(cc).query(cen); o = cc[jn]; dv = cen - o; L = np.linalg.norm(dv, axis=1); dv /= L[:, None]
    th = sc.cast_rays(o3d.core.Tensor(np.concatenate([o, dv], 1).astype(np.float32)))["t_hit"].numpy(); gap = L - th
    print(json.dumps(dict(glb=glb, faces=int(len(F)), area_m2=round(float(ar.sum()), 3),
                          double_layer_1_10cm_pct=round(float(((gap > 0.01) & (gap < 0.10)).mean() * 100), 2),
                          occluded_pct=round(float((gap > 0.01).mean() * 100), 2), views=[int(kf[j]) for j in pick])), flush=True)


if __name__ == "__main__":
    main()
