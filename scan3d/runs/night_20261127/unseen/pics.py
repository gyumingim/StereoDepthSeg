#!/usr/bin/env python3
"""Pictures for item 2 (CPU ray-cast, headless): measured ffsonly2 mesh (grey shading by normal) without / with generated walls (purple).
Views: 3 virtual cameras inside the room looking at the two structure walls. -> unseen_pics.jpg (+ holdout picture)."""
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

import unseen as U

HERE = Path(__file__).resolve().parent


def look(eye, tgt, W=960, H=540, f=520.0):
    z = tgt - eye; z /= np.linalg.norm(z); down0 = np.array([0, 1.0, 0])             # world +y = down (camera convention x right, y down)
    x = np.cross(down0, z); x /= np.linalg.norm(x); y = np.cross(z, x)                 # y = image down
    vv, uu = np.mgrid[0:H, 0:W]; d = ((uu - W / 2) / f)[..., None] * x + ((vv - H / 2) / f)[..., None] * y + z
    return d.reshape(-1, 3), W, H


def render(scenes, eye, tgt):
    D, W, H = look(eye, tgt); rays = o3d.core.Tensor(np.concatenate([np.broadcast_to(eye, D.shape), D], 1).astype(np.float32))
    best_t = np.full(len(D), np.inf); col = np.zeros((len(D), 3))
    for sc, kind in scenes:
        h = sc.cast_rays(rays); t = h["t_hit"].numpy(); n = h["primitive_normals"].numpy()
        sh = np.abs((n * (-D / np.linalg.norm(D, axis=1, keepdims=True))).sum(1)) * 0.7 + 0.3
        c = (np.c_[sh, sh, sh] * 200) if kind == "m" else (np.c_[sh * 217, sh * 138, sh * 185])
        nearer = t < best_t; best_t[nearer] = t[nearer]; col[nearer] = c[nearer]
    return col.reshape(H, W, 3).astype(np.uint8)


def main():
    V, F, fin, fy = U.load_mesh()
    sm = o3d.t.geometry.RaycastingScene(); sm.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    import os; GEN = Path(os.environ.get("UNSEEN_GEN", str(HERE / "gen_final.ply"))); g = o3d.io.read_triangle_mesh(str(GEN))
    sg = o3d.t.geometry.RaycastingScene(); sg.add_triangles(o3d.core.Tensor(np.asarray(g.vertices, np.float32)), o3d.core.Tensor(np.asarray(g.triangles, np.uint32)))
    W_ = [w for w in json.loads((HERE / "walls.json").read_text())["walls"] if w["structure"]]
    kf, T, Kr, _ = U.cams(); P = T[:, :3, 3]; c = P.mean(0)
    rows = []
    for w in W_:
        ax, t_ = np.array(w["ax"]), np.array(w["t"])
        for um in (0.3, 0.7):
            u = w["u"][0] + um * (w["u"][1] - w["u"][0]); tgt = ax * w["off"] + t_ * u; tgt[1] = -0.5
            eye = tgt - ax * 3.0; eye[1] = -0.6
            a = render([(sm, "m")], eye, tgt); b = render([(sm, "m"), (sg, "g")], eye, tgt)
            for im, lab in ((a, "measured only"), (b, "+ generated (purple)")):
                cv2.putText(im, lab, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            rows.append(np.hstack([a, b])[:, :, ::-1])
    out = np.vstack(rows); cv2.imwrite(os.environ.get("UNSEEN_PIC", str(HERE / "unseen_pics.jpg")), cv2.resize(out, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("ok", out.shape)


if __name__ == "__main__":
    main()
