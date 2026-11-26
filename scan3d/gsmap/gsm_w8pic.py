#!/usr/bin/env python3
"""gsmap 2026-11-27 W8: texture close-up sheet. Spots = held-out partner frames (BA poses bap, NOT keyframes = never texture sources)
nearest in time to keyframes 72 (cabinet front / stickers), 80 (panel), 48 (door / labels), 36 (wall). Rendered at 1920x1080 (Kr x2,
ray cast, nearest texel) from the partner pose; photo = cam5.mp4 frame undistorted the same way as gsm_texture.load_frames.
  python gsm_w8pic.py OUT.json OUT.jpg NAME=GLB [...]"""
import json, sys, time
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_texture import load_frames  # noqa: E402
from gsm_eval2 import load_mesh  # noqa: E402
V2 = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
DUMP = V2 / "ba_partners/run/dump_ba"
SPOTS = [(72, [520, 760, 200, 520], "cabinet front / stickers"), (80, [700, 945, 100, 520], "panel"),
         (48, [420, 660, 90, 490], "door / labels"), (36, [200, 520, 30, 250], "wall")]


def main():
    import open3d as o3d
    outj, outp = sys.argv[1], sys.argv[2]; specs = [s.split("=", 1) for s in sys.argv[3:]]
    c = np.load(DUMP / "cams.npz"); kf = c["kf"]; pa = np.load(V2 / "ba_partners/poses_all.npz")
    TF = {int(f): pa["T"][k] for k, f in enumerate(pa["frames"]) if pa["src"][k] == "ba"}; kfs = set(int(x) for x in kf)
    part = np.array(sorted(set(TF) - kfs))
    spots = []
    for idx, rect, nm in SPOTS:
        f = int(part[np.argmin(np.abs(part - int(kf[idx])))]); spots.append((f, rect, nm, int(kf[idx])))
    imgs, Kr1, _ = load_frames(DUMP, V2 / "rec", [s[0] for s in spots], "lensonly")
    W, H = 1920, 1080; uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    ray = np.stack([(uu - Kr1[0, 2]) / Kr1[0, 0], (vv - Kr1[1, 2]) / Kr1[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    rows = []; info = dict(made=time.strftime("%F %T"), maps={n: g for n, g in specs}, poses=str(V2 / "ba_partners/poses_all.npz"),
                          spots=[dict(frame=f, near_kf=k, rect960=r, name=n) for f, r, n, k in spots])
    cells = {i: [] for i in range(len(spots))}
    for si, (f, r, nm, k) in enumerate(spots):
        x0, x1, y0, y1 = [2 * v for v in r]; cells[si].append((f"photo f{f} ({nm})", imgs[si][y0:y1, x0:x1]))
    for n, g in specs:
        V, F, UV, tex = load_mesh(g); sc = o3d.t.geometry.RaycastingScene(nthreads=8)
        sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32))); Ah, Aw = tex.shape[:2]
        for si, (f, r, nm, k) in enumerate(spots):
            Tc = TF[f]; rr = sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], ray.shape), ray @ Tc[:3, :3].T], 1).astype(np.float32)), nthreads=8)
            pid = rr["primitive_ids"].numpy(); hit = pid != 4294967295; bu = rr["primitive_uvs"].numpy()
            img = np.full((H * W, 3), (60, 20, 60), np.uint8); q = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
            uvp = b0[:, None] * UV[F[q, 0]] + b1[:, None] * UV[F[q, 1]] + b2[:, None] * UV[F[q, 2]]
            img[hit] = tex[np.clip((uvp[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uvp[:, 0] * Aw).astype(int), 0, Aw - 1)]
            x0, x1, y0, y1 = [2 * v for v in r]; cells[si].append((n, img.reshape(H, W, 3)[y0:y1, x0:x1]))
        del sc
    for si in cells:
        hh = 420; row = []
        for lab, im in cells[si]:
            im = cv2.resize(im, (int(im.shape[1] * hh / im.shape[0]), hh), interpolation=cv2.INTER_AREA)
            im = cv2.copyMakeBorder(im, 30, 4, 2, 2, cv2.BORDER_CONSTANT, value=(255, 255, 255))
            cv2.putText(im, lab, (4, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA); row.append(im)
        rows.append(np.concatenate(row, 1))
    wmax = max(r_.shape[1] for r_ in rows); rows = [cv2.copyMakeBorder(r_, 0, 0, 0, wmax - r_.shape[1], cv2.BORDER_CONSTANT, value=(255, 255, 255)) for r_ in rows]
    cv2.imwrite(outp, np.concatenate(rows, 0), [cv2.IMWRITE_JPEG_QUALITY, 90]); Path(outj).write_text(json.dumps(info, indent=1)); print(json.dumps(info))


if __name__ == "__main__":
    main()
