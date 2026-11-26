#!/usr/bin/env python3
"""gsmap 2026-11-27 W8: texture v3 = gsm_texture2.py (copied unchanged) + four opt-in texture-quality options (all off = texture2):
  --sharp-local 1   view score uses LOCAL sharpness (blurred Laplacian energy at the face's pixel, relative to the other views of the
                    same face, sqrt, clip 0.5..2) instead of the per-image global Laplacian variance.
  --robust-gain 1   per-keyframe scalar luminance gain: log-domain, per-view median of (face mean - sample), 10 rounds, samples with
                    lum 20..235 only, temporal median over +-3 keyframes, clip 0.8..1.25 (texture2 gains: RGB, mean, clip 0.6..1.6 -> hurt PSNR).
  --seam-level 1    local seam levelling: at vertices shared by faces with different view labels, each label's colour is pulled to
                    the mean; the correction is diffused into each label patch (Jacobi, --seam-iters) and added per texel (clamp +-40).
  --seam-soft W     (with --seam-level) seam corrections are a soft prior of weight W, the field is smoothed everywhere (W8 SM2).
  --cap N           downscale the atlas to <= N px after baking (0 = off; the old chain uses gsm_atlas8k.py = 8192).
texture2 doc: xatlas charts (8192 atlas, padded/dilated), view-label smoothing,
small-hole fill (boundary loop < --hole-max m), small unseen patches (< --unseen-max m^2) filled from neighbour colours.

  ~/scanner/gsmap_env/venv/bin/python scan3d/gsmap/gsm_texture3.py MESH.ply OUT.glb
"""
import argparse, json, sys, time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from gsm_texture import load_frames, write_glb, RUN  # noqa: E402


def fill_small_holes(V, F, max_perim):
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
    Ed = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]])          # directed, keeps face orientation
    key = E[:, 0] * len(V) + E[:, 1]
    u, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    bd = Ed[cnt[inv] == 1]                                                  # boundary half-edges a->b
    nxt = defaultdict(list)
    for a, b in bd:
        nxt[b].append(a)                                                     # walk reversed so the fill faces match orientation
    used = set(); newV = []; newF = []; nfilled = 0
    for a0, b0 in bd:
        if (a0, b0) in used:
            continue
        loop = [b0, a0]; used.add((a0, b0)); ok = True
        while True:
            cand = nxt.get(loop[-1], [])
            if len(cand) != 1:
                ok = False; break
            n_ = cand[0]
            used.add((n_, loop[-1]))
            if n_ == loop[0]:
                break
            loop.append(n_)
            if len(loop) > 400:
                ok = False; break
        if not ok or len(loop) < 3:
            continue
        L = np.array(loop); P = V[L]
        per = np.linalg.norm(P - np.roll(P, -1, 0), axis=1).sum()
        if per > max_perim:
            continue
        ci = len(V) + len(newV); newV.append(P.mean(0))
        for i in range(len(L)):
            newF.append([L[i], L[(i + 1) % len(L)], ci])
        nfilled += 1
    if newV:
        V = np.concatenate([V, np.array(newV)]); F = np.concatenate([F, np.array(newF)])
    return V, F, nfilled


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh"); ap.add_argument("out"); ap.add_argument("--atlas", type=int, default=8192)
    ap.add_argument("--no-gain", type=int, default=0)
    ap.add_argument("--drop-views", default="", help="comma list of keyframe indices never used (121801: 0 = mis-posed)")
    ap.add_argument("--skip-held", type=int, default=0, help="1: do not use keyframes with index mod 8 == 0 (held-out views for eval2 P4)")
    ap.add_argument("--fill-all", type=int, default=0, help="1: every unseen face gets the nearest textured colour (no magenta)")
    ap.add_argument("--chunk", type=float, default=0.0, help=">0: xatlas per spatial chunk of this size (m), packed together")
    ap.add_argument("--consistency", type=float, default=0.0, help=">0: down-weight views whose face colour is far from the median over views (glare); scale in grey levels")
    ap.add_argument("--hole-max", type=float, default=0.10); ap.add_argument("--unseen-max", type=float, default=0.0025)
    ap.add_argument("--dump", default=str(RUN / "dump_ba")); ap.add_argument("--gt", default="imu_gt_spl")
    ap.add_argument("--rec", default="/home/system/scanner/StereoDepthSeg/scan3d/recordings/hs_20261024_165506_std60")
    ap.add_argument("--sharp-local", type=int, default=0); ap.add_argument("--robust-gain", type=int, default=0)
    ap.add_argument("--seam-level", type=int, default=0); ap.add_argument("--seam-iters", type=int, default=300)
    ap.add_argument("--seam-soft", type=float, default=0.0, help=">0: seam corrections become a soft prior of this weight (vs 1 per mesh-edge neighbour), field smoothed everywhere")
    ap.add_argument("--cap", type=int, default=0)
    a = ap.parse_args()
    import open3d as o3d, xatlas
    assert not Path(a.out).exists(), "never overwrite an existing result"
    t0 = time.time(); tm = {}
    m = o3d.io.read_triangle_mesh(a.mesh)
    V = np.asarray(m.vertices, np.float64); F = np.asarray(m.triangles, np.int64)
    nf0 = len(F)
    V, F, nholes = fill_small_holes(V, F, a.hole_max)
    nf = len(F); tm["holes"] = time.time() - t0
    c = np.load(Path(a.dump) / "cams.npz"); kf, T = c["kf"], c["T"]
    imgs, Kr1, valid1 = load_frames(Path(a.dump), Path(a.rec), kf, a.gt); nv_ = len(kf)
    tm["decode"] = time.time() - t0
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cen = (P0 + P1 + P2) / 3
    nrm = np.cross(P1 - P0, P2 - P0); area = 0.5 * np.linalg.norm(nrm, axis=1); nrm /= 2 * area[:, None] + 1e-12
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    w, h = 960, 540; K = Kr1.copy(); K[:2] /= 2
    uu, vv = np.meshgrid(np.arange(w) + 0.0, np.arange(h) + 0.0)
    dcam = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1).reshape(-1, 3)
    valid = cv2.resize(valid1.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
    small = [cv2.resize(im, (w, h), interpolation=cv2.INTER_AREA) for im in imgs]
    sharp = np.array([cv2.Laplacian(cv2.cvtColor(s, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var() for s in small])
    sharp = np.clip(sharp / np.median(sharp), 0.5, 1.5)
    score = np.zeros((nf, nv_), np.float32)
    drop_v = set(int(x) for x in a.drop_views.split(",") if x.strip())
    sf, sv, scol, sen = [], [], [], []
    for j in range(nv_):
        if (a.skip_held and j % 8 == 0) or j in drop_v:   # E0 P4 held-out views; judge round 1: drop mis-posed views (121801: index 0)
            continue
        R, t = T[j][:3, :3], T[j][:3, 3]
        rays = np.concatenate([np.broadcast_to(t, dcam.shape), dcam @ R.T], 1).astype(np.float32)
        zb = sc.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().reshape(h, w)
        Xc = (cen - t) @ R; z = Xc[:, 2]; ok = z > 0.05
        u = np.where(ok, K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2], -1); v = np.where(ok, K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2], -1)
        ui, vi = np.round(u).astype(int), np.round(v).astype(int)
        ok &= (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
        ok[ok] &= valid[vi[ok], ui[ok]]
        zr = np.full(nf, np.inf); zr[ok] = zb[vi[ok], ui[ok]]
        ok &= np.abs(z - zr) < 0.01 + 0.01 * z
        vd = t - cen; vd /= np.linalg.norm(vd, axis=1, keepdims=True)
        cs = np.abs((nrm * vd).sum(1)); ok &= cs > 0.15
        score[:, j] = np.where(ok, cs / np.maximum(z, 1e-3) * (1.0 if a.sharp_local else sharp[j]), 0)
        idx = np.nonzero(ok)[0]; sf.append(idx); sv.append(np.full(len(idx), j)); scol.append(small[j][vi[idx], ui[idx]].astype(np.float64) + 1)
        if a.sharp_local:
            lap = cv2.Laplacian(cv2.cvtColor(small[j], cv2.COLOR_BGR2GRAY).astype(np.float32), cv2.CV_32F)
            en = cv2.GaussianBlur(lap * lap, (0, 0), 6.0); sen.append(np.log(en[vi[idx], ui[idx]].astype(np.float64) + 1.0))
    sf, sv, scol = np.concatenate(sf), np.concatenate(sv), np.concatenate(scol)
    if a.sharp_local:   # relative local sharpness of this view vs the other views of the same face (log mean over views)
        sen = np.concatenate(sen); cntf = np.maximum(np.bincount(sf, minlength=nf), 1)
        mlog = np.bincount(sf, weights=sen, minlength=nf) / cntf
        fac = np.exp(np.clip(0.5 * (sen - mlog[sf]), np.log(0.5), np.log(2.0)))
        score[sf, sv] *= fac.astype(np.float32)
    if a.consistency > 0:   # glare: weight exp(-|lum - median lum over the views that see the face| / k), plus a penalty for brighter
        lum = scol.mean(1); o_ = np.argsort(sf, kind="stable"); sfo, lo = sf[o_], lum[o_]
        st_ = np.searchsorted(sfo, np.arange(nf)); en_ = np.searchsorted(sfo, np.arange(nf), side="right")
        med = np.zeros(nf)
        for f_ in np.nonzero(en_ > st_)[0]:
            med[f_] = np.median(lo[st_[f_]:en_[f_]])
        dev = lum - med[sf]
        wgt = np.exp(-np.abs(dev) / a.consistency) * np.where(dev > 0, np.exp(-dev / a.consistency), 1.0)
        score[sf, sv] *= wgt.astype(np.float32)
    tm["vis"] = time.time() - t0
    # face adjacency (shared edges)
    E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1); fid = np.tile(np.arange(nf), 3)
    o = np.lexsort((E[:, 1], E[:, 0])); Es, fs = E[o], fid[o]
    same = (Es[1:] == Es[:-1]).all(1); adj = np.stack([fs[:-1][same], fs[1:][same]], 1)
    adj = np.concatenate([adj, adj[:, ::-1]])
    # view label: best score, then 2 rounds of neighbour-majority smoothing (only to a label the face can see well)
    best = np.where(score.max(1) > 0, score.argmax(1), -1)
    for _ in range(2):
        votes = np.zeros((nf, nv_), np.float32)
        nb = best[adj[:, 1]]; okn = nb >= 0
        np.add.at(votes, (adj[okn, 0], nb[okn]), 1.0)
        cand = votes.argmax(1)
        good = (best >= 0) & (votes.max(1) >= 2) & (score[np.arange(nf), cand] >= 0.7 * score.max(1))
        best = np.where(good, cand, best)
    seen = best >= 0
    # gains
    gain = np.ones((nv_, 3))
    multi = np.bincount(sf, minlength=nf)[sf] >= 2
    sf2, sv2, sc2 = sf[multi], sv[multi], scol[multi]
    for _ in range(3):
        fm = np.zeros((nf, 3)); np.add.at(fm, sf2, sc2 * gain[sv2]); fm /= np.maximum(np.bincount(sf2, minlength=nf), 1)[:, None]
        r = fm[sf2] / sc2
        for j in range(nv_):
            mm = sv2 == j
            if mm.sum() > 50:
                gain[j] = np.median(r[mm], 0)
        gain /= np.exp(np.log(gain).mean(0))
    gain = np.clip(gain, 0.6, 1.6)
    if a.no_gain:   # diagnostic: no per-view colour levelling
        gain = np.ones_like(gain)
    if a.robust_gain:   # W8 (b): scalar luminance gain, log domain, medians, temporal smoothing, tight clip
        lum = scol.mean(1) - 1; okl = multi & (lum > 20) & (lum < 235)
        f3, v3, l3 = sf[okl], sv[okl], np.log(lum[okl]); g_ = np.zeros(nv_)
        for _ in range(10):
            c_ = np.bincount(f3, minlength=nf); fm = np.bincount(f3, weights=l3 + g_[v3], minlength=nf) / np.maximum(c_, 1)
            r_ = fm[f3] - l3; o_ = np.argsort(v3, kind="stable"); vs_, rs_ = v3[o_], r_[o_]
            b_ = np.searchsorted(vs_, np.arange(nv_ + 1))
            for j in range(nv_):
                if b_[j + 1] - b_[j] > 50:
                    g_[j] = np.median(rs_[b_[j]:b_[j + 1]])
            g_ -= g_.mean()
        gs_ = np.array([np.median(g_[max(0, j - 3):j + 4]) for j in range(nv_)])
        gain = np.repeat(np.clip(np.exp(gs_), 0.8, 1.25)[:, None], 3, 1)
    # unseen components -> small ones get neighbour colour
    import scipy.sparse as sp
    from scipy.sparse.csgraph import connected_components
    un = ~seen; ua = adj[un[adj[:, 0]] & un[adj[:, 1]]]
    G = sp.coo_matrix((np.ones(len(ua)), (ua[:, 0], ua[:, 1])), shape=(nf, nf))
    _, lab = connected_components(G, directed=False)
    comp_area = np.bincount(lab[un], weights=area[un], minlength=lab.max() + 1)
    fill_face = un & ((comp_area[lab] < a.unseen_max) | bool(a.fill_all)); magenta_face = un & ~fill_face
    # vertex colours (for the filled faces): from seen faces' chosen view at the vertex, gain applied, then diffusion
    vc = np.zeros((len(V), 3)); vn = np.zeros(len(V))
    for j in range(nv_):
        fj = np.nonzero(best == j)[0]
        if len(fj) == 0:
            continue
        vi_ = F[fj].ravel(); X = V[vi_]; R, t = T[j][:3, :3], T[j][:3, 3]; Xc = (X - t) @ R
        mxp = (K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2]).astype(np.float32); myp = (K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]).astype(np.float32)
        pad = (-len(mxp)) % 4096
        col = cv2.remap(small[j], np.pad(mxp, (0, pad)).reshape(-1, 4096), np.pad(myp, (0, pad)).reshape(-1, 4096), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3)[:len(mxp)] * gain[j]
        np.add.at(vc, vi_, col); np.add.at(vn, vi_, 1)
    has = vn > 0; vc[has] /= vn[has, None]
    Ev = np.concatenate([E, E[:, ::-1]])
    for _ in range(60):
        miss = ~has
        if not miss.any():
            break
        s_ = np.zeros((len(V), 3)); n_ = np.zeros(len(V))
        ok = has[Ev[:, 1]] & miss[Ev[:, 0]]
        np.add.at(s_, Ev[ok, 0], vc[Ev[ok, 1]]); np.add.at(n_, Ev[ok, 0], 1)
        nw = n_ > 0; vc[nw] = s_[nw] / n_[nw, None]; has |= nw
    n_nearest_v = 0
    if a.fill_all and (~has).any() and has.any():   # vertices the edge diffusion did not reach (e.g. separate patches): nearest coloured vertex
        from scipy.spatial import cKDTree
        miss = ~has; _, nn_ = cKDTree(V[has]).query(V[miss]); vc[miss] = vc[has][nn_]; n_nearest_v = int(miss.sum()); has[:] = True
    seam_info = {}
    if a.seam_level:   # W8 (d): local seam levelling on (vertex, label) pairs
        fs_ = np.nonzero(best >= 0)[0]; key = F[fs_] * nv_ + best[fs_][:, None]            # (ns, 3)
        uk, pinv = np.unique(key.ravel(), return_inverse=True); pinv = pinv.reshape(-1, 3); npair = len(uk)
        pv, pl = uk // nv_, uk % nv_
        pc = np.zeros((npair, 3))
        for j in np.unique(pl):
            mm = np.nonzero(pl == j)[0]; R, t = T[j][:3, :3], T[j][:3, 3]; Xc = (V[pv[mm]] - t) @ R
            mxp = (K[0, 0] * Xc[:, 0] / Xc[:, 2] + K[0, 2]).astype(np.float32); myp = (K[1, 1] * Xc[:, 1] / Xc[:, 2] + K[1, 2]).astype(np.float32)
            pad = (-len(mxp)) % 4096; bl_ = cv2.GaussianBlur(small[j], (0, 0), 1.5)
            pc[mm] = cv2.remap(bl_, np.pad(mxp, (0, pad)).reshape(-1, 4096), np.pad(myp, (0, pad)).reshape(-1, 4096), cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3)[:len(mm)].astype(np.float64) * gain[j]
        nlab = np.bincount(pv, minlength=len(V)); seamp = nlab[pv] >= 2
        tgt = np.stack([np.bincount(pv, weights=pc[:, k], minlength=len(V)) for k in range(3)], 1) / np.maximum(nlab, 1)[:, None]
        d = np.zeros((npair, 3)); d[seamp] = np.clip(tgt[pv[seamp]] - pc[seamp], -40, 40)
        pe = np.concatenate([pinv[:, [0, 1]], pinv[:, [1, 2]], pinv[:, [2, 0]]]); pe = np.unique(np.sort(pe, 1), axis=0)
        pe = np.concatenate([pe, pe[:, ::-1]]); deg = np.bincount(pe[:, 0], minlength=npair).astype(np.float64)
        fixed = seamp.copy(); dfix = d[fixed].copy()
        wfix = np.where(fixed, a.seam_soft, 0.0)
        for _ in range(a.seam_iters):
            s_ = np.stack([np.bincount(pe[:, 0], weights=d[pe[:, 1], k], minlength=npair) for k in range(3)], 1)
            if a.seam_soft > 0:   # soft: seam targets are a weighted prior, the field is smoothed everywhere (less per-vertex noise)
                tgt0 = np.zeros_like(d); tgt0[fixed] = dfix
                d = (s_ + wfix[:, None] * tgt0) / np.maximum(deg + wfix, 1e-9)[:, None]
            else:
                d = s_ / np.maximum(deg, 1)[:, None]; d[fixed] = dfix
        cp = np.full((nf, 3), -1, np.int64); cp[fs_] = pinv
        seam_info = dict(pairs=int(npair), seam_pairs=int(seamp.sum()), seam_vertices=int((nlab >= 2).sum()),
                         abs_corr_seam_mean=round(float(np.abs(dfix).mean()), 2), abs_corr_all_mean=round(float(np.abs(d).mean()), 2))
    tm["labels"] = time.time() - t0
    # xatlas charts
    atl = xatlas.Atlas()
    if a.chunk > 0:   # xatlas charting is very slow on big welded meshes (70+ min): chart per spatial chunk, pack jointly
        cen_ = (V[F[:, 0]] + V[F[:, 1]] + V[F[:, 2]]) / 3; ck = np.floor(cen_ / a.chunk).astype(np.int64)
        _, cid = np.unique(ck, axis=0, return_inverse=True); cid = cid.ravel(); perm = np.argsort(cid, kind="stable")
        F = F[perm]; best = best[perm]; fill_face = fill_face[perm]; magenta_face = magenta_face[perm]; cid = cid[perm]
        if a.seam_level:
            cp = cp[perm]
        bounds = np.flatnonzero(np.diff(np.r_[-1, cid, cid.max() + 1]))
        chunks = []
        for b0, b1 in zip(bounds[:-1], bounds[1:]):
            Fc = F[b0:b1]; uv_, inv_ = np.unique(Fc.ravel(), return_inverse=True)
            atl.add_mesh(V[uv_].astype(np.float32), inv_.reshape(-1, 3).astype(np.uint32)); chunks.append(uv_)
    else:
        atl.add_mesh(V.astype(np.float32), F.astype(np.uint32)); chunks = [np.arange(len(V))]
    co = xatlas.ChartOptions(); po = xatlas.PackOptions(); po.resolution = a.atlas; po.padding = 4; po.bilinear = True
    atl.generate(co, po)
    Vout, Fis, UVs, off = [], [], [], 0
    for k_, uv_ in enumerate(chunks):
        vm_, Fi_, UV_ = atl[k_]; Vout.append(V[uv_[vm_]]); Fis.append(Fi_.astype(np.int64) + off); UVs.append(UV_); off += len(vm_)
    Vout = np.concatenate(Vout); Fi = np.concatenate(Fis); UV = np.concatenate(UVs)
    A = atl.width; Ah = atl.height
    tm["xatlas"] = time.time() - t0
    # rasterise triangles in UV space -> texel face id + barycentrics
    UVp = UV * np.array([A, Ah]); T2 = UVp[Fi]                                  # (nf, 3, 2)
    texf = np.full((Ah, A), -1, np.int32); texb = np.zeros((Ah, A, 2), np.float32)
    x0 = np.floor(T2[:, :, 0].min(1)).astype(int); x1 = np.ceil(T2[:, :, 0].max(1)).astype(int)
    y0 = np.floor(T2[:, :, 1].min(1)).astype(int); y1 = np.ceil(T2[:, :, 1].max(1)).astype(int)
    bw, bh = x1 - x0 + 1, y1 - y0 + 1
    for s0 in range(0, nf, 20000):
        ids = np.arange(s0, min(nf, s0 + 20000)); cnt = bw[ids] * bh[ids]
        rep = np.repeat(ids, cnt); off = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        px = x0[rep] + off % bw[rep]; py = y0[rep] + off // bw[rep]
        ok = (px >= 0) & (px < A) & (py >= 0) & (py < Ah); rep, px, py = rep[ok], px[ok], py[ok]
        p = np.stack([px + 0.5, py + 0.5], 1); t_ = T2[rep]
        v0, v1, v2 = t_[:, 1] - t_[:, 0], t_[:, 2] - t_[:, 0], p - t_[:, 0]
        d00 = (v0 * v0).sum(1); d01 = (v0 * v1).sum(1); d11 = (v1 * v1).sum(1); d20 = (v2 * v0).sum(1); d21 = (v2 * v1).sum(1)
        den = d00 * d11 - d01 * d01; den = np.where(np.abs(den) < 1e-12, 1e-12, den)
        b1 = (d11 * d20 - d01 * d21) / den; b2 = (d00 * d21 - d01 * d20) / den; b0 = 1 - b1 - b2
        ins = (b0 >= -0.02) & (b1 >= -0.02) & (b2 >= -0.02)
        texf[py[ins], px[ins]] = rep[ins]; texb[py[ins], px[ins]] = np.stack([b1[ins], b2[ins]], 1)
    tm["raster"] = time.time() - t0
    tex = np.zeros((Ah, A, 3), np.uint8)
    yy, xx = np.nonzero(texf >= 0); ff = texf[yy, xx]; b1 = texb[yy, xx, 0].astype(np.float64); b2 = texb[yy, xx, 1].astype(np.float64); b0 = 1 - b1 - b2
    tri = F[ff]
    X = b0[:, None] * V[tri[:, 0]] + b1[:, None] * V[tri[:, 1]] + b2[:, None] * V[tri[:, 2]]
    lab_t = best[ff]
    for j in range(nv_):
        mm = np.nonzero(lab_t == j)[0]
        if len(mm) == 0:
            continue
        R, t = T[j][:3, :3], T[j][:3, 3]; Xc = (X[mm] - t) @ R
        mxp = (Kr1[0, 0] * Xc[:, 0] / Xc[:, 2] + Kr1[0, 2]).astype(np.float32); myp = (Kr1[1, 1] * Xc[:, 1] / Xc[:, 2] + Kr1[1, 2]).astype(np.float32)
        pad = (-len(mxp)) % 4096
        col = cv2.remap(imgs[j], np.pad(mxp, (0, pad)).reshape(-1, 4096), np.pad(myp, (0, pad)).reshape(-1, 4096), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_REPLICATE).reshape(-1, 3)[:len(mm)].astype(np.float64)
        col = col * gain[j]
        if a.seam_level:
            q_ = cp[ff[mm]]; col += b0[mm, None] * d[q_[:, 0]] + b1[mm, None] * d[q_[:, 1]] + b2[mm, None] * d[q_[:, 2]]
        tex[yy[mm], xx[mm]] = np.clip(col, 0, 255).astype(np.uint8)
    mf = fill_face[ff]
    if mf.any():
        cc = b0[mf, None] * vc[tri[mf, 0]] + b1[mf, None] * vc[tri[mf, 1]] + b2[mf, None] * vc[tri[mf, 2]]
        tex[yy[mf], xx[mf]] = np.clip(cc, 0, 255).astype(np.uint8)
    mg = magenta_face[ff]; tex[yy[mg], xx[mg]] = (255, 0, 255)
    # dilate into the gutter (8 px)
    filled = texf >= 0
    for _ in range(8):
        d = cv2.dilate(tex, np.ones((3, 3), np.uint8)); dm = cv2.dilate(filled.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        new = dm & ~filled; tex[new] = d[new]; filled |= new
    tm["texels"] = time.time() - t0
    A0 = [int(A), int(Ah)]
    if a.cap and max(A, Ah) > a.cap:   # same rule as gsm_atlas8k.py (square resize, INTER_AREA), cap configurable
        tex = cv2.resize(tex, (a.cap, a.cap), interpolation=cv2.INTER_AREA); A = Ah = a.cap
    ok_, jpg = cv2.imencode(".jpg", tex, [cv2.IMWRITE_JPEG_QUALITY, 88])
    write_glb(a.out, Vout, UV, Fi, jpg.tobytes())
    cv2.imwrite(str(Path(a.out).with_suffix(".atlas_preview.jpg")), cv2.resize(tex, (2048, int(2048 * Ah / A)), interpolation=cv2.INTER_AREA))
    info = dict(made=time.strftime("%F %T"), mesh=a.mesh, faces_in=int(nf0), faces=int(nf), holes_filled=int(nholes),
                atlas=[int(A), int(Ah)], unseen_pct=float(un.mean() * 100), filled_from_neighbours_pct=float(fill_face.mean() * 100),
                magenta_pct=float(magenta_face.mean() * 100), filled_faces=int(fill_face.sum()), nearest_colour_vertices=n_nearest_v, gain_min=gain.min(0).round(3).tolist(), gain_max=gain.max(0).round(3).tolist(),
                times_s={k: round(v, 1) for k, v in tm.items()}, total_s=round(time.time() - t0, 1),
                script="gsm_texture3.py", args=vars(a), dump_cams=str(Path(a.dump) / "cams.npz"), atlas_baked=A0, seam=seam_info,
                gain_lum=[round(float(x), 3) for x in gain[:, 0]],
                glb_mb=round(Path(a.out).stat().st_size / 2 ** 20, 2), jpg_mb=round(len(jpg) / 2 ** 20, 2))
    Path(a.out).with_suffix(".json").write_text(json.dumps(info, indent=1)); print(json.dumps(info), flush=True)


if __name__ == "__main__":
    main()
