#!/usr/bin/env python3
"""gsmap 2026-11-26 W3: wall-fill picture. Spots = keyframes that see the most filled wall cells (z < 5 m, spread >= 40 keyframes apart).
Per spot: photo | NAME=GLB ... | W3 mesh overlay (grey shading, red = new wall-fill faces = face index >= nf0).
  python gsm_w3pic.py REF_RUN(with dump_ba) OUT.jpg W3.ply W3_FILL.npz NAME=GLB [NAME=GLB ...]   [env GSMAP_NSPOTS=4]
"""
import os, sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_check import load_glb  # noqa: E402


def main():
    import open3d as o3d
    ref, outp, w3p, fz = sys.argv[1:5]; specs = [s.split("=", 1) for s in sys.argv[5:]]
    c = np.load(ref + "/dump_ba/cams.npz"); T, K, kf = c["T"], c["Kr"], c["kf"]; w, h = 960, 540
    fd = np.load(fz); C = fd["C"][fd["fill"]]; nf0 = int(fd["nf0"])
    cnt = np.zeros(len(T), int)
    for j in range(len(T)):
        Xc = (C - T[j][:3, 3]) @ T[j][:3, :3]; z = Xc[:, 2]; ok = (z > 0.3) & (z < 5)
        u = K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]; v = K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]
        cnt[j] = int((ok & (u >= 0) & (u < w) & (v >= 0) & (v < h)).sum())
    spots = []
    for j in np.argsort(-cnt):
        if cnt[j] == 0 or len(spots) >= int(os.environ.get("GSMAP_NSPOTS", 4)):
            break
        if all(abs(int(j) - s) >= 40 for s in spots):
            spots.append(int(j))
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    M = []
    for n, f in specs:
        V, UV, F, tex = load_glb(f); s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F)); M.append((n, UV, F, tex, s))
    m = o3d.io.read_triangle_mesh(w3p); Vp = np.asarray(m.vertices, np.float32); Fp = np.asarray(m.triangles, np.uint32)
    nrm = np.cross(Vp[Fp[:, 1]] - Vp[Fp[:, 0]], Vp[Fp[:, 2]] - Vp[Fp[:, 0]]); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    sp = o3d.t.geometry.RaycastingScene(); sp.add_triangles(o3d.core.Tensor(Vp), o3d.core.Tensor(Fp))

    def cast(s, Tv):
        return s.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tv[:3, 3], d.shape), d @ Tv[:3, :3].T], 1).astype(np.float32)))

    def shade(mm, Tv):
        n, UV, F, tex, s = mm; r = cast(s, Tv); pid = r["primitive_ids"].numpy(); bu = r["primitive_uvs"].numpy(); hit = pid != 4294967295
        img = np.zeros((h * w, 3), np.uint8); img[:] = (60, 0, 60); p = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
        uv = b0[:, None] * UV[F[p, 0]] + b1[:, None] * UV[F[p, 1]] + b2[:, None] * UV[F[p, 2]]
        Ah, A = tex.shape[:2]; img[hit] = tex[np.clip((uv[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uv[:, 0] * A).astype(int), 0, A - 1)]
        return img.reshape(h, w, 3)

    def overlay(Tv):
        r = cast(sp, Tv); pid = r["primitive_ids"].numpy(); hit = pid != 4294967295; p = pid[hit]
        dw = d[hit] @ Tv[:3, :3].T; dw /= np.linalg.norm(dw, axis=1, keepdims=True)
        g = (60 + 170 * np.abs((nrm[p] * dw).sum(1))).astype(np.uint8)
        img = np.zeros((h * w, 3), np.uint8); img[:] = (60, 0, 60); img[hit] = g[:, None]
        hh = np.nonzero(hit)[0][p >= nf0]; img[hh] = (0.35 * img[hh] + 0.65 * np.array((0, 0, 255))).astype(np.uint8)
        return img.reshape(h, w, 3)

    rows = []
    for j in spots:
        Tv = T[j]; ph = cv2.imread(ref + "/dump_ba/img/%05d.png" % kf[j])
        tiles = [ph] + [shade(mm, Tv) for mm in M] + [overlay(Tv)]
        names = [f"photo kf {kf[j]} ({cnt[j]} fill cells)"] + [mm[0] for mm in M] + ["W3 mesh, red = new wall fill"]
        tl = []
        for x, nm in zip(tiles, names):
            x = cv2.resize(x, (480, 270)); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 0, 0), 3); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 255, 255), 1); tl.append(x)
        rows.append(np.hstack(tl))
    cv2.imwrite(outp, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85]); print("spots", [int(kf[j]) for j in spots], [int(cnt[j]) for j in spots])


if __name__ == "__main__":
    main()
