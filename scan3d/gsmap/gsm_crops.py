#!/usr/bin/env python3
"""gsmap 2026-11-25: before/after renders of textured GLBs at fixed spots (CPU ray cast, nearest texel; dark purple = no surface).

  python scan3d/gsmap/gsm_crops.py REF_RUN OUT.jpg NAME=GLB [NAME=GLB ...]
Spots (121801, keyframe index): 72 sticker cabinet, 72 rotated 25 deg (side view), 24 white bin, 48 wavy wall, 52 ceiling chunks
(camera pitched up 20 deg).
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_check import load_glb  # noqa: E402


def rot(Tv, yaw=0.0, pitch=0.0, piv_d=1.2):
    piv = Tv[:3, 3] + Tv[:3, 2] * piv_d; a, b = np.radians(yaw), np.radians(pitch)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]]); Rx = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)], [0, np.sin(b), np.cos(b)]])
    Rl = Ry @ Rx; Rw = Tv[:3, :3] @ Rl @ Tv[:3, :3].T; Tn = np.eye(4); Tn[:3, :3] = Rw @ Tv[:3, :3]; Tn[:3, 3] = piv + Rw @ (Tv[:3, 3] - piv); return Tn


def main():
    import open3d as o3d
    ref, outp = sys.argv[1], sys.argv[2]; specs = [s.split("=", 1) for s in sys.argv[3:]]
    c = np.load(ref + "/dump_ba/cams.npz"); T, K, kf = c["T"], c["Kr"], c["kf"]; w, h = 960, 540
    uu, vv = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    d = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    M = []
    for n, f in specs:
        V, UV, F, tex = load_glb(f); s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F)); M.append((n, V, UV, F, tex, s))

    def shade(m, Tv):
        n, V, UV, F, tex, s = m; R, t = Tv[:3, :3], Tv[:3, 3]
        r = s.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, d.shape), d @ R.T], 1).astype(np.float32)))
        pid = r["primitive_ids"].numpy(); bu = r["primitive_uvs"].numpy(); hit = pid != 4294967295
        img = np.zeros((h * w, 3), np.uint8); img[:] = (60, 0, 60); p = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
        uv = b0[:, None] * UV[F[p, 0]] + b1[:, None] * UV[F[p, 1]] + b2[:, None] * UV[F[p, 2]]
        Ah, A = tex.shape[:2]; img[hit] = tex[np.clip((uv[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uv[:, 0] * A).astype(int), 0, A - 1)]
        return img.reshape(h, w, 3)
    spots = [(72, 0, 0, "sticker cabinet"), (72, 25, 0, "cabinet, 25 deg side view"), (24, 0, 0, "white bin"), (48, 0, 0, "wavy wall"),
             (52, 0, 20, "ceiling chunks (pitched up 20 deg)")]
    rows = []
    for j, yaw, pitch, name in spots:
        Tv = rot(T[j], yaw, pitch); ph = cv2.imread(ref + "/dump_ba/img/%05d.png" % kf[j])
        tiles = [ph if (yaw == 0 and pitch == 0) else np.full_like(ph, 30)] + [shade(m, Tv) for m in M]
        names = [f"photo kf {kf[j]}: {name}" if (yaw == 0 and pitch == 0) else f"{name} (no photo)"] + [m[0] for m in M]
        tl = []
        for x, nm in zip(tiles, names):
            x = cv2.resize(x, (480, 270)); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 0, 0), 3); cv2.putText(x, nm, (6, 18), 0, 0.5, (0, 255, 255), 1); tl.append(x)
        rows.append(np.hstack(tl))
    cv2.imwrite(outp, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])


if __name__ == "__main__":
    main()
