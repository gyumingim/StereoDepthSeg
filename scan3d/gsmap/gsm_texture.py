#!/usr/bin/env python3
"""gsmap 2026-11-25: photo-texture a mesh from the 1920x1080 undistorted keyframes + BA poses -> GLB (mesh + JPEG atlas). CPU only.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_texture.py MESH.ply OUT.glb [--atlas 4096]

Method (own, simple; mvs-texturing not built):
* frames: cam5.mp4 frames at the keyframe ids, undistorted at 1920x1080 with the GT-calibration K/dist -> pinhole Kr x 2
  (same as the 960 dump, scale 2). Poses = dump cams.npz T (BA, board frame) -> same world frame as the splats/meshes.
* visibility: Open3D RaycastingScene depth at 960x540 per view; a face is visible in a view if its centroid projects inside
  the valid (non-border) area, depth matches the ray-cast depth (1 cm + 1 %) and the face faces the camera (cos > 0.15).
* view choice per face: max of cos(view angle) / depth * sharpness(view) (Laplacian variance, clipped 0.5..1.5). No MRF.
* colour leveling: per-view RGB gain from face-centroid colours seen in several views (3 rounds, median ratio).
* atlas: per-face packing (2 triangles per square cell, 1.5 px gap on the diagonal, barycentrics clamped -> edge padding);
  marching-cubes faces are all about voxel size, so a fixed texel budget per face is fine.
* faces seen by no view -> magenta (255, 0, 255), not filled.
* GLB written by hand: POSITION, TEXCOORD_0, uint32 indices, one JPEG image (q90). Positions as in the input (y down).
"""
import argparse, json, struct, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
RUN = HERE.parent / "runs" / "gsmap_20261125"


def load_frames(dump, rec, need):
    g = np.load(rec / "imu_gt_spl" / "gt.npz")
    K1 = g["K"].astype(np.float64); dist = g["dist"].astype(np.float64)
    c = np.load(dump / "cams.npz"); Kr1 = c["Kr"].copy(); Kr1[:2] *= 2
    mx, my = cv2.initUndistortRectifyMap(K1, dist, np.eye(3), Kr1, (1920, 1080), cv2.CV_32FC1)
    valid = cv2.erode(((mx >= 0) & (mx <= 1919) & (my >= 0) & (my <= 1079)).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    cap = cv2.VideoCapture(str(rec / "cam5.mp4")); out = {}; i = 0; want = set(int(k) for k in need)
    while len(out) < len(want):
        r, f = cap.read()
        if not r:
            break
        if i in want:
            out[i] = cv2.remap(f, mx, my, cv2.INTER_LINEAR)
        i += 1
    cap.release()
    return [out[int(k)] for k in need], Kr1, valid


def write_glb(path, V, UV, F, jpg):
    V = V.astype(np.float32); UV = UV.astype(np.float32); F = F.astype(np.uint32).ravel()
    bufs = [V.tobytes(), UV.tobytes(), F.tobytes(), jpg]
    offs, blob = [], b""
    for b in bufs:
        offs.append(len(blob)); blob += b + b"\0" * ((-len(b)) % 4)
    j = {"asset": {"version": "2.0", "generator": "gsmap gsm_texture.py"},
         "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
         "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "TEXCOORD_0": 1}, "indices": 2, "material": 0}]}],
         "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}, "metallicFactor": 0.0, "roughnessFactor": 1.0},
                        "doubleSided": True, "extensions": {"KHR_materials_unlit": {}}}],
         "extensionsUsed": ["KHR_materials_unlit"],
         "textures": [{"source": 0, "sampler": 0}], "samplers": [{"magFilter": 9729, "minFilter": 9987}],
         "images": [{"bufferView": 3, "mimeType": "image/jpeg"}],
         "buffers": [{"byteLength": len(blob)}],
         "bufferViews": [{"buffer": 0, "byteOffset": offs[0], "byteLength": len(bufs[0]), "target": 34962},
                         {"buffer": 0, "byteOffset": offs[1], "byteLength": len(bufs[1]), "target": 34962},
                         {"buffer": 0, "byteOffset": offs[2], "byteLength": len(bufs[2]), "target": 34963},
                         {"buffer": 0, "byteOffset": offs[3], "byteLength": len(bufs[3])}],
         "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(V), "type": "VEC3",
                        "min": V.min(0).tolist(), "max": V.max(0).tolist()},
                       {"bufferView": 1, "componentType": 5126, "count": len(UV), "type": "VEC2"},
                       {"bufferView": 2, "componentType": 5125, "count": len(F), "type": "SCALAR"}]}
    js = json.dumps(j).encode(); js += b" " * ((-len(js)) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob)))
        f.write(struct.pack("<II", len(js), 0x4E4F534A)); f.write(js)
        f.write(struct.pack("<II", len(blob), 0x004E4942)); f.write(blob)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh"); ap.add_argument("out"); ap.add_argument("--atlas", type=int, default=4096)
    ap.add_argument("--dump", default=str(RUN / "dump_ba"))
    ap.add_argument("--rec", default="/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60")
    a = ap.parse_args()
    import open3d as o3d
    t0 = time.time()
    m = o3d.io.read_triangle_mesh(a.mesh)
    V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    nf = len(F)
    c = np.load(Path(a.dump) / "cams.npz"); kf, T = c["kf"], c["T"]
    imgs, Kr1, valid1 = load_frames(Path(a.dump), Path(a.rec), kf)
    t_dec = time.time() - t0
    nv_ = len(kf)
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cen = (P0 + P1 + P2) / 3
    nrm = np.cross(P1 - P0, P2 - P0); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    # raycast depth at 960x540
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    w, h = 960, 540; K = Kr1.copy(); K[:2] /= 2
    uu, vv = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    dcam = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    valid = cv2.resize(valid1.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
    sharp = np.array([cv2.Laplacian(cv2.cvtColor(cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY), cv2.CV_64F).var() for im in imgs])
    sharp = np.clip(sharp / np.median(sharp), 0.5, 1.5)
    best = np.full(nf, -1); bscore = np.zeros(nf)
    samp_f, samp_v, samp_c = [], [], []
    small = [cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA) for im in imgs]
    for j in range(nv_):
        R, t = T[j][:3, :3], T[j][:3, 3]
        rays = np.concatenate([np.broadcast_to(t, dcam.shape), dcam @ R.T], 1).astype(np.float32)
        zb = sc.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().reshape(h, w)   # = camera z (direction has z = 1)
        Xc = (cen - t) @ R
        z = Xc[:, 2]
        ok = z > 0.05
        u = np.where(ok, K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2], -1); v = np.where(ok, K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2], -1)
        ui, vi = np.round(u).astype(int), np.round(v).astype(int)
        ok &= (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        ok[ok] &= valid[vi[ok], ui[ok]]
        zr = np.full(nf, np.inf); zr[ok] = zb[vi[ok], ui[ok]]
        ok &= np.abs(z - zr) < 0.01 + 0.01 * z
        vd = t - cen; vd /= np.linalg.norm(vd, axis=1, keepdims=True)
        cs = np.abs((nrm * vd).sum(1))        # TSDF/2DGS normals may point either way -> |cos|
        ok &= cs > 0.15
        s = np.where(ok, cs / np.maximum(z, 1e-3) * sharp[j], 0)
        b = s > bscore; best[b] = j; bscore[b] = s[b]
        idx = np.nonzero(ok)[0]
        samp_f.append(idx); samp_v.append(np.full(len(idx), j))
        samp_c.append(small[j][vi[idx], ui[idx]].astype(np.float64) + 1.0)
    sf, sv, scol = np.concatenate(samp_f), np.concatenate(samp_v), np.concatenate(samp_c)
    t_vis = time.time() - t0 - t_dec
    # per-view gain (RGB)
    gain = np.ones((nv_, 3))
    multi = np.bincount(sf, minlength=nf)[sf] >= 2
    sf2, sv2, sc2 = sf[multi], sv[multi], scol[multi]
    for _ in range(3):
        adj = sc2 * gain[sv2]
        fm = np.zeros((nf, 3)); np.add.at(fm, sf2, adj); fm /= np.maximum(np.bincount(sf2, minlength=nf), 1)[:, None]
        r = fm[sf2] / sc2
        for j in range(nv_):
            mm = sv2 == j
            if mm.sum() > 50:
                gain[j] = np.median(r[mm], 0)
        gain /= np.exp(np.log(gain).mean(0))
    gain = np.clip(gain, 0.6, 1.6)
    # atlas: per-face cells, 2 faces per cell
    A = a.atlas; ncell = (nf + 1) // 2; ncol = int(np.ceil(np.sqrt(ncell))); cs_ = A // ncol
    if cs_ < 4:
        sys.exit(f"atlas {A} too small for {nf} faces (cell {cs_} px)")
    e = 1.5 / cs_; aa = 1 - e
    fi = np.arange(nf); cell = fi // 2; half = fi % 2
    ox = (cell % ncol) * cs_; oy = (cell // ncol) * cs_
    loc = np.array([[0, 0], [aa, 0], [0, aa]])
    locB = 1 - loc
    L = np.where(half[:, None, None] == 0, loc[None], locB[None])           # (nf, 3, 2) cell-normalised
    UVpx = np.stack([ox, oy], 1)[:, None, :] + L * cs_                       # texel coordinates (pixel edges)
    UV = UVpx / A
    # fill texels: every texel of every cell, both halves
    tex = np.zeros((A, A, 3), np.uint8); tex[:] = (255, 0, 255)[::-1]        # BGR magenta background
    gy, gx = np.mgrid[0:cs_, 0:cs_]
    lu = (gx.ravel() + 0.5) / cs_; lv = (gy.ravel() + 0.5) / cs_
    for hf in (0, 1):
        su, sv_ = (lu, lv) if hf == 0 else (1 - lu, 1 - lv)
        sel = (su + sv_ < 1) if hf == 0 else (su + sv_ <= 1)
        b1 = np.clip(su[sel] / aa, 0, 1); b2 = np.clip(sv_[sel] / aa, 0, 1)
        s_ = b1 + b2; b1 = np.where(s_ > 1, b1 / s_, b1); b2 = np.where(s_ > 1, b2 / s_, b2); b0 = 1 - b1 - b2
        px, py = gx.ravel()[sel], gy.ravel()[sel]
        faces = fi[half == hf]
        for j in range(nv_):
            fj = faces[best[faces] == j]
            if len(fj) == 0:
                continue
            X = (b0[None, :, None] * P0[fj][:, None] + b1[None, :, None] * P1[fj][:, None] + b2[None, :, None] * P2[fj][:, None]).reshape(-1, 3)
            R, t = T[j][:3, :3], T[j][:3, 3]
            Xc = (X - t) @ R
            mxp = (Kr1[0, 0] * Xc[:, 0] / Xc[:, 2] + Kr1[0, 2]).astype(np.float32); myp = (Kr1[1, 1] * Xc[:, 1] / Xc[:, 2] + Kr1[1, 2]).astype(np.float32)
            nn_ = len(mxp); pad = (-nn_) % 4096
            mxq = np.pad(mxp, (0, pad)).reshape(-1, 4096); myq = np.pad(myp, (0, pad)).reshape(-1, 4096)
            col = cv2.remap(imgs[j], mxq, myq, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3)[:nn_].astype(np.float64)
            col = np.clip(col * gain[j], 0, 255).astype(np.uint8)
            tx = (ox[fj][:, None] + px[None]).ravel(); ty = (oy[fj][:, None] + py[None]).ravel()
            tex[ty, tx] = col
    ok_, jpg = cv2.imencode(".jpg", tex, [cv2.IMWRITE_JPEG_QUALITY, 90])
    Vo = np.concatenate([P0[:, None], P1[:, None], P2[:, None]], 1).reshape(-1, 3)
    write_glb(a.out, Vo, UV.reshape(-1, 2), np.arange(3 * nf).reshape(-1, 3), jpg.tobytes())
    cv2.imwrite(str(Path(a.out).with_suffix(".atlas_preview.jpg")), cv2.resize(tex, (1024, 1024), interpolation=cv2.INTER_AREA))
    info = dict(made=time.strftime("%F %T"), mesh=a.mesh, faces=int(nf), atlas=A, cell_px=int(cs_), unseen_face_pct=float((best < 0).mean() * 100),
                views_used=int(len(np.unique(best[best >= 0]))), gain_min=gain.min(0).round(3).tolist(), gain_max=gain.max(0).round(3).tolist(),
                decode_s=round(t_dec, 1), vis_s=round(t_vis, 1), total_s=round(time.time() - t0, 1),
                glb_mb=round(Path(a.out).stat().st_size / 2 ** 20, 2), jpg_mb=round(len(jpg) / 2 ** 20, 2))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
