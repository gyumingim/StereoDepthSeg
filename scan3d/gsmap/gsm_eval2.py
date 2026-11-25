#!/usr/bin/env python3
"""gsmap 2026-11-25 E0: eval2 ruler (plan gsmap_perfect_plan_20261125.md section 2), CPU only.

  GSMAP_RUN=<v2 run> python scan3d/gsmap/gsm_eval2.py MESH(.ply|.glb) OUT.json [--heldout-glb GLB] [--tex-json JSON] [--sheet OUT.jpg]

References (fixed, eval2/ref, built once by gsm_eval2_ref.py): gold FFS (480x270), 2DGS depth of all keyframes, room polygon +
floor/ceiling planes, 6 ROIs, 24 inside poses. Reference depth per keyframe pixel = gold FFS if present else 2DGS (alpha > 0.5).
P1 floater area: face centroid inside a keyframe frustum (z > 0.1, |cos| > 0.15); violated if ref depth > z + max(5 cm, 4 %),
   agreeing if |ref - z| <= max(5 cm, 4 %); floater = violated in >= N keyframes AND violated > 2 x agreeing; share of mesh area
   for N = 3, 5, 10.
P2 mid-air islands: welded components < 0.25 m2 whose median face distance to the vertices of components >= 1 m2 is > 5 cm.
P3 correct surface: first hit within 3 cm of gold FFS (every 6th keyframe, 480x270). S1 = front (> 3 cm in front) / hole.
P4 (with --heldout-glb, a texture built WITHOUT the held-out views): ray cast at the 62 held-out keyframes (index % 8 == 0),
   480x270, nearest texel, no hit = black; PSNR / SSIM / LPIPS(alex) on valid (non-border) pixels; hit share; hit-only PSNR.
S2 second layer on ROIs: second hit 2..15 cm behind the first. S3 ROI plane RMS / p95 (SVD, refit on |d| < 5 cm, hits within 30 cm).
S4 floor / ceiling coverage of the room polygon (4 cm cells with surface within 5 cm of the plane).
S5 inside poses: empty (no hit within 8 m) and back-face (hit normal facing away from the ray) pixel shares.
S6 from --tex-json (texture2 json): unseen / magenta face %. S7 area, faces, welded components.
"""
import json, os, struct, sys, time
from pathlib import Path

import cv2
import numpy as np

R = Path(os.environ.get("GSMAP_RUN", ""))
REF = R / "eval2" / "ref"


def load_mesh(p):
    import open3d as o3d
    if str(p).endswith(".glb"):
        b = open(p, "rb").read(); jl = struct.unpack("<I", b[12:16])[0]; j = json.loads(b[20:20 + jl]); bl = b[20 + jl + 8:]
        bv = j["bufferViews"]; g = lambda i: bl[bv[i]["byteOffset"]:bv[i]["byteOffset"] + bv[i]["byteLength"]]  # noqa: E731
        return (np.frombuffer(g(0), np.float32).reshape(-1, 3).astype(np.float64), np.frombuffer(g(2), np.uint32).reshape(-1, 3).astype(np.int64),
                np.frombuffer(g(1), np.float32).reshape(-1, 2), cv2.imdecode(np.frombuffer(g(3), np.uint8), 1))
    m = o3d.io.read_triangle_mesh(str(p)); return np.asarray(m.vertices, np.float64), np.asarray(m.triangles, np.int64), None, None


def cams():
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, K = c["kf"], c["T"], c["Kr"].copy(); W, H = [int(v) for v in c["size"]]
    K2 = K.copy(); K2[:2] /= 2
    return kf, T, K2, W // 2, H // 2


def ref_depth():
    g = np.load(REF / "gold_ffs.npz")["Z"]; d = np.load(REF / "gs_depth_all.npz"); ZM, AL = d["ZM"], d["AL"]
    return g, ZM, AL


def face_votes(V, F, T, K, w, h, gold, ZM, AL):
    """per face: (violated, agreeing) keyframe counts (P1 rule)."""
    P0, P1, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cen = (P0 + P1 + P2) / 3; nrm = np.cross(P1 - P0, P2 - P0); nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    viol = np.zeros(len(F), np.int16); agr = np.zeros(len(F), np.int16)
    cen32 = cen.astype(np.float32)
    for j in range(len(T)):
        Rm, t = T[j][:3, :3].astype(np.float32), T[j][:3, 3].astype(np.float32)
        Xc = (cen32 - t) @ Rm; z = Xc[:, 2]; ok = z > 0.1
        u = np.floor(K[0, 0] * Xc[:, 0] / np.where(ok, z, 1) + K[0, 2]).astype(np.int32); v = np.floor(K[1, 1] * Xc[:, 1] / np.where(ok, z, 1) + K[1, 2]).astype(np.int32)
        ok &= (u >= 0) & (u < w) & (v >= 0) & (v < h)
        idx = np.nonzero(ok)[0]
        if len(idx) == 0:
            continue
        vd = t - cen32[idx]; vd /= np.linalg.norm(vd, axis=1, keepdims=True)
        ng = np.abs((nrm[idx] * vd).sum(1)) > 0.15; idx = idx[ng]
        uu, vv = u[idx], v[idx]; zz = z[idx]
        gz = gold[j][vv, uu].astype(np.float32); gm = ZM[j][vv, uu].astype(np.float32); ga = AL[j][vv, uu]
        r = np.where(gz > 0, gz, np.where(ga > 127, gm, np.nan))
        tol = np.maximum(0.05, 0.04 * zz)
        viol[idx] += (r > zz + tol).astype(np.int16); agr[idx] += (np.abs(r - zz) <= tol).astype(np.int16)
    return viol, agr, cen


def components(V, F):
    import open3d as o3d
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F.astype(np.int32)))
    m.merge_close_vertices(1e-5)
    cl, nt, ar = m.cluster_connected_triangles()
    return np.asarray(cl), np.asarray(nt), np.asarray(ar)


def islands(V, F, cl, ar, cen, small=0.25, big=1.0, gap=0.05):
    from scipy.spatial import cKDTree
    bigf = ar[cl] >= big
    if not bigf.any():
        return np.zeros(len(ar), bool)
    Pb = cen[bigf][:: max(1, int(bigf.sum() // 2_000_000))]
    smf = np.nonzero(ar[cl] < small)[0]
    d, _ = cKDTree(Pb).query(cen[smf], k=1, workers=8)
    o = np.argsort(cl[smf], kind="stable"); cs = cl[smf][o]; ds = d[o]
    bounds = np.flatnonzero(np.diff(np.r_[-1, cs, -2]))
    isl = np.zeros(len(ar), bool)
    for b0, b1 in zip(bounds[:-1], bounds[1:]):
        if np.median(ds[b0:b1]) > gap:
            isl[cs[b0]] = True
    return isl


def ssim(x, y):
    import torch, torch.nn.functional as Fn
    g = torch.exp(-((torch.arange(11) - 5.0) ** 2) / (2 * 1.5 ** 2)); g = g / g.sum(); wk = (g[:, None] * g[None, :])[None, None].repeat(3, 1, 1, 1)
    cv_ = lambda t: Fn.conv2d(t, wk, padding=5, groups=3)  # noqa: E731
    mx, my = cv_(x), cv_(y); sxx, syy, sxy = cv_(x * x) - mx * mx, cv_(y * y) - my * my, cv_(x * y) - mx * my
    return float((((2 * mx * my + 1e-4) * (2 * sxy + 9e-4)) / ((mx * mx + my * my + 1e-4) * (sxx + syy + 9e-4))).mean())


def main():
    import open3d as o3d
    mp, outp = sys.argv[1], sys.argv[2]
    arg = lambda k: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else None  # noqa: E731
    t0 = time.time(); res = dict(mesh=mp, made=time.strftime("%F %T"))
    V, F, _, _ = load_mesh(mp)
    kf, T, K, w, h = cams(); gold, ZM, AL = ref_depth()
    P0, P1_, P2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]; area = 0.5 * np.linalg.norm(np.cross(P1_ - P0, P2 - P0), axis=1); A = area.sum()
    # P1
    viol, agr, cen = face_votes(V, F, T, K, w, h, gold, ZM, AL)
    res["S9_floater_area_pct"] = res_p1 = {f"N{N}": round(float(area[(viol >= N) & (viol > 2 * agr)].sum() / A * 100), 3) for N in (3, 5, 10)}
    # P2 + S7
    cl, nt, ar = components(V, F); isl = islands(V, F, cl, ar, cen)
    res["S8_islands"] = dict(count=int(isl.sum()), area_pct=round(float(ar[isl].sum() / A * 100), 3))
    small_face = (ar < 0.25)[cl] if len(cl) == len(F) else np.zeros(len(F), bool)
    res["S7"] = dict(area_m2=round(float(A), 2), faces=int(len(F)), components=int(len(nt)), comp_lt_0p05m2=int((ar < 0.05).sum()))
    tm1 = time.time() - t0
    # ray-cast helpers
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    ray = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu)], -1)
    cast = lambda Tc, d: sc.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(Tc[:3, 3], d.shape), d @ Tc[:3, :3].T], 1).astype(np.float32)))  # noqa: E731
    # P3 / S1
    tt, zz = [], []
    for ci in range(2, len(kf), 6):
        m = gold[ci] > 0
        if m.any():
            tt.append(cast(T[ci], ray[m])["t_hit"].numpy()); zz.append(gold[ci][m].astype(np.float32))
    t_, z_ = np.concatenate(tt), np.concatenate(zz)
    res["P3_correct_pct"] = round(float((np.abs(t_ - z_) <= 0.03).mean() * 100), 2)
    res["S1"] = dict(front_pct=round(float((t_ < z_ - 0.03).mean() * 100), 2), hole_pct=round(float((~(t_ <= z_ + 0.03)).mean() * 100), 2), n_px=int(len(z_)))
    # S2 / S3 ROIs
    rois = json.loads((REF / "rois.json").read_text()); res["S2_S3_rois"] = {}
    for r in rois:
        x0, x1, y0, y1 = [v // 2 for v in r["rect"]]; Tc = T[r["idx"]]; d = ray[y0:y1:2, x0:x1:2].reshape(-1, 3)
        h1 = cast(Tc, d); t1 = h1["t_hit"].numpy(); okh = np.isfinite(t1)
        dw = d @ Tc[:3, :3].T; o2 = Tc[:3, 3] + dw * (np.where(okh, t1, 0) + 0.005)[:, None]
        t2 = sc.cast_rays(o3d.core.Tensor(np.concatenate([o2, dw], 1).astype(np.float32)))["t_hit"].numpy()
        sec = okh & np.isfinite(t2) & (t2 * np.linalg.norm(dw, axis=1) > 0.015) & (t2 * np.linalg.norm(dw, axis=1) < 0.145)
        X = (d[okh] * t1[okh, None])
        e = dict(hit=round(float(okh.mean()), 3), second_layer_pct=round(float(sec[okh].mean() * 100), 2) if okh.any() else None)
        if len(X) > 50:
            med = np.median(X, 0); X = X[np.linalg.norm(X - med, axis=1) < 0.6]
            c0 = X.mean(0); nn = np.linalg.svd(X - c0, full_matrices=False)[2][2]; dd = (X - c0) @ nn
            for _ in range(2):
                k = np.abs(dd) < 0.05; c0 = X[k].mean(0); nn = np.linalg.svd(X[k] - c0, full_matrices=False)[2][2]; dd = (X - c0) @ nn
            dd = np.abs(dd[np.abs(dd) < 0.30]); e.update(rms_cm=round(float(np.sqrt((dd ** 2).mean()) * 100), 2), p95_cm=round(float(np.percentile(dd, 95) * 100), 2))
        res["S2_S3_rois"][r["name"]] = e
    # S4
    rp = json.loads((REF / "room_poly.json").read_text()); poly = np.array(rp["poly_xz"]); lo = poly.min(0); gsz = np.ceil((poly.max(0) - lo) / 0.04).astype(int) + 1
    pm = np.zeros((gsz[1], gsz[0]), np.uint8); cv2.fillPoly(pm, [np.round((poly - lo) / 0.04).astype(np.int32)], 1)
    pts = np.concatenate([cen, P0, (P0 + P1_) / 2]); res["S4"] = dict(room_poly_m2=rp["area_m2"])
    for nm in ("floor", "ceiling"):
        n_ = np.array(rp[nm]["normal"]); y_ = rp[nm]["y_m"]; dd = np.abs((pts - np.array([0, y_, 0])) @ n_)
        q = pts[dd < 0.05][:, [0, 2]]; g = np.floor((q - lo) / 0.04).astype(int); ok = (g[:, 0] >= 0) & (g[:, 0] < gsz[0]) & (g[:, 1] >= 0) & (g[:, 1] < gsz[1])
        cov = np.zeros_like(pm); cov[g[ok, 1], g[ok, 0]] = 1
        res["S4"][nm + "_coverage_pct"] = round(float((cov & pm).sum() / pm.sum() * 100), 2)
    # P2 (judge round 1): inside-view defects at 48 poses = the 24 novel poses + the same positions at pitch 0
    nov = json.loads((REF / "novel_poses.json").read_text())
    sys.path.insert(0, str(Path(__file__).resolve().parent)); from gsm_eval2_ref import rot
    poses = [np.array(nv["T"]) for nv in nov] + [rot(T[nv["idx"]], nv["yaw"], 0) for nv in nov]
    emp = bf = smf = tot = 0
    for Tc in poses:
        d = ray.reshape(-1, 3); r = cast(Tc, d); t1 = r["t_hit"].numpy(); hit = np.isfinite(t1) & (t1 < 8.0)
        pid = r["primitive_ids"].numpy()[hit]; nr = np.cross(P1_[pid] - P0[pid], P2[pid] - P0[pid]); dw = (d @ Tc[:3, :3].T)[hit]
        back = (nr * dw).sum(1) > 0
        bf += int(back.sum()); smf += int((~back & small_face[pid]).sum()); emp += int((~hit).sum()); tot += len(t1)
    res["P2_inside"] = dict(P2a_backface_pct=round(bf / tot * 100, 2), P2b_small_component_front_pct=round(smf / tot * 100, 2),
                            P2c_empty_pct=round(emp / tot * 100, 2), n_poses=len(poses), res="480x270", small="welded component < 0.25 m2")
    # S6
    tj = arg("--tex-json")
    if tj:
        x = json.loads(Path(tj).read_text()); res["S6"] = dict(unseen_face_pct=x.get("unseen_pct", x.get("unseen_face_pct")), magenta_face_pct=x.get("magenta_pct"), tex_json=tj)
    # P4
    hg = arg("--heldout-glb")
    if hg:
        import torch, lpips
        Vg, Fg, UV, tex = load_mesh(hg)
        s2 = o3d.t.geometry.RaycastingScene(); s2.add_triangles(o3d.core.Tensor(Vg.astype(np.float32)), o3d.core.Tensor(Fg.astype(np.uint32)))
        c = np.load(R / "dump_ba" / "cams.npz"); Kf = c["Kr"]
        mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kf, (w * 2, h * 2), cv2.CV_32FC1)
        vm = cv2.erode(((mx >= 0) & (mx <= w * 2 - 1) & (my >= 0) & (my <= h * 2 - 1)).astype(np.uint8), np.ones((7, 7), np.uint8))
        vm = cv2.resize(vm, (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)
        lp = lpips.LPIPS(net="alex", verbose=False); ps, ph, hs, ss, ls, pl, pc = [], [], [], [], [], [], []
        Ah, Aw = tex.shape[:2]; W2, H2 = w * 2, h * 2
        uf, vf = np.meshgrid(np.arange(W2) + 0.5, np.arange(H2) + 0.5)
        rayf = np.stack([(uf - Kf[0, 2]) / Kf[0, 0], (vf - Kf[1, 2]) / Kf[1, 1], np.ones_like(uf)], -1).reshape(-1, 3)
        for ci in range(8, len(kf), 8):   # judge round 1: kf index 0 dropped (mis-posed)
            r = s2.cast_rays(o3d.core.Tensor(np.concatenate([np.broadcast_to(T[ci][:3, 3], (H2 * W2, 3)), rayf @ T[ci][:3, :3].T], 1).astype(np.float32)))
            pid = r["primitive_ids"].numpy(); hit = pid != 4294967295; bu = r["primitive_uvs"].numpy()
            img = np.zeros((H2 * W2, 3), np.uint8); p = pid[hit]; b1, b2 = bu[hit, 0], bu[hit, 1]; b0 = 1 - b1 - b2
            uvp = b0[:, None] * UV[Fg[p, 0]] + b1[:, None] * UV[Fg[p, 1]] + b2[:, None] * UV[Fg[p, 2]]
            img[hit] = tex[np.clip((uvp[:, 1] * Ah).astype(int), 0, Ah - 1), np.clip((uvp[:, 0] * Aw).astype(int), 0, Aw - 1)]
            img = cv2.resize(img.reshape(H2, W2, 3), (w, h), interpolation=cv2.INTER_AREA)   # 960x540 render -> area downsample
            hit = cv2.resize(hit.reshape(H2, W2).astype(np.float32), (w, h), interpolation=cv2.INTER_AREA) > 0.99
            gt = cv2.resize(cv2.imread(str(R / "dump_ba" / "img" / f"{kf[ci]:05d}.png")), (w, h), interpolation=cv2.INTER_AREA)
            a_, b_ = img.astype(np.float64) / 255, gt.astype(np.float64) / 255
            mse = ((a_ - b_) ** 2)[vm].mean(); ps.append(-10 * np.log10(mse)); mh = vm & hit
            ph.append(-10 * np.log10(((a_ - b_) ** 2)[mh].mean()) if mh.any() else np.nan); hs.append(mh.sum() / vm.sum())
            if mh.sum() > 100:   # per-image colour fit (3x3 + bias, least squares on hit pixels), hit-only
                Xa = np.c_[a_[mh], np.ones(mh.sum())]; M_ = np.linalg.lstsq(Xa, b_[mh], rcond=None)[0]
                pc.append(-10 * np.log10(((Xa @ M_ - b_[mh]) ** 2).mean()))
                a4 = cv2.resize(a_, (w // 2, h // 2), interpolation=cv2.INTER_AREA); b4 = cv2.resize(b_, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
                m4 = cv2.resize(mh.astype(np.float32), (w // 2, h // 2), interpolation=cv2.INTER_AREA) > 0.99
                pl.append(-10 * np.log10(((a4 - b4) ** 2)[m4].mean()) if m4.any() else np.nan)
            am = np.where(vm[..., None], a_, b_); xa = torch.from_numpy(am[..., ::-1].copy()).permute(2, 0, 1)[None].float(); xb = torch.from_numpy(b_[..., ::-1].copy()).permute(2, 0, 1)[None].float()
            ss.append(ssim(xa, xb))
            with torch.no_grad():
                ls.append(float(lp(xa * 2 - 1, xb * 2 - 1)))
        res["P4_heldout"] = dict(glb=hg, n_views=len(ps), psnr=round(float(np.mean(ps)), 3), ssim=round(float(np.mean(ss)), 4), lpips=round(float(np.mean(ls)), 4),
                                 hit_share=round(float(np.mean(hs)), 4), psnr_hit_only=round(float(np.nanmean(ph)), 3),
                                 psnr_hit_colourfit=round(float(np.nanmean(pc)), 3), psnr_hit_lowres=round(float(np.nanmean(pl)), 3),
                                 res="render 960x540 -> area 480x270; lowres = 240x135 (4x fewer px); kf index 0 dropped")
    res["runtime_s"] = round(time.time() - t0, 1); res["p1p2_s"] = round(tm1, 1)
    Path(outp).write_text(json.dumps(res, indent=1)); print(json.dumps(res), flush=True)


if __name__ == "__main__":
    main()
