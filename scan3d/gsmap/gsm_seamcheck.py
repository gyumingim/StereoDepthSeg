#!/usr/bin/env python3
"""gsmap 2026-11-26: render GLBs like three.js (bilinear + trilinear mipmaps, LOD from per-triangle UV derivatives = GPU quad
derivatives) and count seam artefacts before/after gsm_seampad.
usage: GSMAP_SPOTS="idx:yaw:pitch:name;..." gsm_seamcheck.py dump_dir out.jpg out.json name=orig.glb,pad.glb,cover.png [...]
Shading emulates three.js antialias:true = MSAA 4x (shader at pixel centre, UV extrapolated past triangle edges).
Seam pixels = pixels where a sample's texture footprint reaches an empty atlas texel (distance to nearest empty texel < 1 + 2^lod, level-0 texels).
Counts on seam pixels: near-white (min RGB >= 215), magenta (R,B >= 150, G <= 0.6 min(R,B)), bright outlier (luma > 5x5 median luma + 35)."""
import json, os, sys, time
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gsm_check import load_glb  # noqa: E402
from gsm_crops import rot  # noqa: E402


MSAA = [(-0.125, -0.375), (0.375, -0.125), (-0.375, 0.125), (0.125, 0.375)]   # standard 4x sample pattern (D3D/GL)
if os.environ.get("GSMAP_NOMSAA"):   # diagnosis only: single centre sample
    MSAA = [(0.0, 0.0)]


def mips(tex):
    L = [tex.astype(np.float32)]
    while min(L[-1].shape[:2]) > 1:
        a = L[-1]; L.append(cv2.resize(a, (max(1, a.shape[1] // 2), max(1, a.shape[0] // 2)), interpolation=cv2.INTER_AREA))
    return L


def sample(L, u, v, lod):
    out = np.zeros((len(u), 3), np.float32); lod = np.clip(lod, 0, len(L) - 1); l0 = np.floor(lod).astype(int); fr = lod - l0
    for l in range(len(L)):
        for m, w in ((l0 == l, 1 - fr), (l0 + 1 == l, fr)):
            if not m.any():
                continue
            h_, w_ = L[l].shape[:2]; n = int(m.sum()); pad = (-n) % 4096
            mx = np.pad((u[m] * w_ - 0.5).astype(np.float32), (0, pad)).reshape(-1, 4096); my = np.pad((v[m] * h_ - 0.5).astype(np.float32), (0, pad)).reshape(-1, 4096)
            s = cv2.remap(L[l], mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3)[:n]
            out[m] += s * w[m, None]
    return out


def main():
    import open3d as o3d
    from scipy import ndimage
    dump, outp, outj = sys.argv[1], sys.argv[2], sys.argv[3]; specs = [s.split("=", 1) for s in sys.argv[4:]]
    c = np.load(dump + "/cams.npz"); T, K = c["T"], c["Kr"]; W0, H0 = 960, 540
    spots = []   # "idx:yaw:pitch:name" or "idx:yaw:pitch:dolly_m:name" (dolly = move the camera forward along its view axis)
    for x in os.environ["GSMAP_SPOTS"].split(";"):
        q = x.split(":"); dl = 0.0
        if len(q) >= 5:
            try:
                dl = float(q[3]); q = q[:3] + [":".join(q[4:])]
            except ValueError:
                q = q[:3] + [":".join(q[3:])]
        spots.append((int(q[0]), float(q[1]), float(q[2]), dl, q[3]))
    res = dict(made=time.strftime("%F %T"), dump=dump, rule=__doc__.split("\n")[3:6], maps={}); rows = []
    for name, files in specs:
        f_orig, f_pad, f_cov = files.split(",")
        V, UV, F, tex0 = load_glb(f_orig); V1, UV1, F1, tex1 = load_glb(f_pad)
        assert np.array_equal(UV, UV1) and np.array_equal(F, F1)
        Ht, Wt = tex0.shape[:2]; cov = cv2.imread(f_cov, 0) > 0
        dist = ndimage.distance_transform_edt(cov).astype(np.float32)           # level-0 texels to nearest empty texel
        L0, L1 = mips(tex0), mips(tex1)
        s = o3d.t.geometry.RaycastingScene(); s.add_triangles(o3d.core.Tensor(V), o3d.core.Tensor(F))
        Fi = F.astype(np.int64); P0, P1, P2 = V[Fi[:, 0]].astype(np.float64), V[Fi[:, 1]].astype(np.float64), V[Fi[:, 2]].astype(np.float64)
        Nn = np.cross(P1 - P0, P2 - P0)
        for j, yaw, pitch, dl, sname in spots:
            Tv = rot(T[j], yaw, pitch); R, t = Tv[:3, :3], Tv[:3, 3] + Tv[:3, 2] * dl
            for view, sc in (("close", 1), ("far (1/8 res = minified)", 8)):
                w, h = W0 // sc, H0 // sc; Kv = K.astype(np.float64).copy(); Kv[:2] /= sc
                uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
                def rays(du, dv):
                    return np.stack([(uu + du - Kv[0, 2]) / Kv[0, 0], (vv + dv - Kv[1, 2]) / Kv[1, 1], np.ones_like(uu)], -1).reshape(-1, 3) @ R.T
                def uv_at(p, dd):                                                # ray / plane of triangle p (extrapolated beyond its edges like the GPU)
                    n_ = Nn[p]; dn = (dd * n_).sum(1); tt = ((P0[p] - t) * n_).sum(1) / np.where(np.abs(dn) < 1e-12, 1e-12, dn)
                    X = t + dd * tt[:, None]; e1, e2, e0 = P1[p] - P0[p], P2[p] - P0[p], X - P0[p]
                    d00 = (e1 * e1).sum(1); d01 = (e1 * e2).sum(1); d11 = (e2 * e2).sum(1); d20 = (e0 * e1).sum(1); d21 = (e0 * e2).sum(1)
                    den = d00 * d11 - d01 * d01; den = np.where(np.abs(den) < 1e-30, 1e-30, den)
                    b1 = (d11 * d20 - d01 * d21) / den; b2 = (d00 * d21 - d01 * d20) / den; b0 = 1 - b1 - b2
                    return b0[:, None] * UV[Fi[p, 0]] + b1[:, None] * UV[Fi[p, 1]] + b2[:, None] * UV[Fi[p, 2]]
                dc, dx, dy = rays(0, 0), rays(1, 0), rays(0, 1); sz = np.array([Wt, Ht])
                # MSAA 4x (three.js antialias:true): coverage per sample, but the shader runs at the pixel CENTRE with the UV of the covering
                # triangle extrapolated (no 'centroid' qualifier) -> at every triangle edge the texture is read outside the triangle.
                samp = []
                for ox, oy in MSAA:
                    d_ = rays(ox, oy); r = s.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(t, d_.shape), d_], 1).astype(np.float32)))
                    pid = r["primitive_ids"].numpy().astype(np.int64); hit = pid != 4294967295; p = pid[hit]
                    uv = uv_at(p, dc[hit]); ux = uv_at(p, dx[hit]); uy = uv_at(p, dy[hit])
                    lod = np.log2(np.maximum(np.maximum(np.linalg.norm((ux - uv) * sz, axis=1), np.linalg.norm((uy - uv) * sz, axis=1)), 1e-6))
                    if os.environ.get("GSMAP_NOMIP"):   # diagnosis only: level 0 always
                        lod = np.minimum(lod, 0)
                    dd_ = dist[np.clip(np.floor(uv[:, 1] * Ht).astype(int), 0, Ht - 1), np.clip(np.floor(uv[:, 0] * Wt).astype(int), 0, Wt - 1)]
                    samp.append((hit, pid, uv, lod, dd_ < 1 + 2 ** np.clip(lod, 0, None)))
                hit_any = np.any([q[0] for q in samp], 0); seam = np.zeros(h * w, bool)
                for hit, pid, uv, lod, sm in samp:
                    seam[np.flatnonzero(hit)[sm]] = True
                edge = np.zeros(h * w, bool)
                for hit, pid, *_ in samp[1:]:
                    edge |= pid != samp[0][1]
                ent = dict(spot=sname, view=view, size=[w, h], hit_px=int(hit_any.sum()), seam_px=int(seam.sum()), tri_edge_px=int(edge.sum()),
                           lod_median=round(float(np.median(samp[0][3])), 2))
                tiles = []
                for tag, Ls in (("before", L0), ("pad", L1)):
                    acc = np.zeros((h * w, 3), np.float32)
                    for hit, pid, uv, lod, sm in samp:
                        cs_ = np.full((h * w, 3), 17.0, np.float32); cs_[hit] = sample(Ls, uv[:, 0], uv[:, 1], lod); acc += cs_ / len(samp)
                    img = np.clip(acc + 0.5, 0, 255).astype(np.uint8); c_ = img[seam].astype(np.int16); bb, gg, rr = c_[:, 0], c_[:, 1], c_[:, 2]
                    lum = cv2.cvtColor(img.reshape(h, w, 3), cv2.COLOR_BGR2GRAY).astype(np.int16); med = cv2.medianBlur(lum.astype(np.uint8), 5).astype(np.int16)
                    ent[tag] = dict(near_white=int((c_.min(1) >= 215).sum()), magenta=int(((rr >= 150) & (bb >= 150) & (gg <= 0.6 * np.minimum(rr, bb))).sum()),
                                    bright_outlier=int(((lum - med).ravel()[seam] > 35).sum()), dark_outlier=int(((med - lum).ravel()[seam] > 35).sum()))
                    tl = cv2.resize(img.reshape(h, w, 3), (W0, H0), interpolation=cv2.INTER_NEAREST)
                    lab = f"{name} {sname} {view} {tag}: white {ent[tag]['near_white']} mag {ent[tag]['magenta']} bright {ent[tag]['bright_outlier']}"
                    cv2.putText(tl, lab, (8, 24), 0, 0.6, (0, 0, 0), 4); cv2.putText(tl, lab, (8, 24), 0, 0.6, (0, 255, 255), 1); tiles.append(tl)
                rows.append(np.hstack(tiles)); res["maps"].setdefault(name, []).append(ent); print(json.dumps(ent), flush=True)
    cv2.imwrite(outp, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88]); Path(outj).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
