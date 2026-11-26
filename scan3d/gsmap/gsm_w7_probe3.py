#!/usr/bin/env python3
"""gsmap W7 2026-11-27 probe 3 (= probe 2 + confirmation count >= 2 and a split of "none" into epipole cone / FFS no output): sub-classes of the rejected aisle-object FFS depth (same keyframes/regions/pair stack as gsm_w7_probe.py).

single sub-classes: sigma/Z bins (<= 1 %, 1-2 %, 2-3 %) and cross-keyframe confirmation: projected into the ODD-offset keyframes within
+-12 idx, confirmed if >= 1 of them has its own depth (fused, else min-sigma stored pair) within k=2 * hypot(sig_i, sig_j) (the fix3
neighbour rule, extended to keyframes with >= 2 pairs). Cross-view error (ref b) is then measured on the EVEN-offset keyframes only
(disjoint from the confirming views); BA error (ref a) is fully independent of the rule.
no_stored sub-classes from w7/uncut (wide/relax pairs recomputed without sigma cap/zmax): sigma_cut (uncut finite, best sigma/Z > 3 %,
Z <= 4 m; bins 3-5, 5-10, > 10 %), range_cut (uncut Z > 4 m only), none (no uncut value: epipole cone, no overlap, FFS uncertainty mask,
or the keyframe has no wide/relax pair).
"""
import json, sys, time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/gsmap"
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import gsm_f_ruler as fr  # noqa: E402
import gsm_ffs_4m as M  # noqa: E402
import gsm_ba_partners as BP  # noqa: E402

V2 = fr.V2; BAP = V2 / "ba_partners"; P2D = BAP / "ffs_pairs2"; P3D = BAP / "ffs_pairs3"; SRC = V2 / "ffs_relax/src"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"; MESH = V2 / "ffsonly_ba2/b/carve.ply"; UNC = V2 / "w7/uncut"
A8 = Path("/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/audit8")
OUT = V2 / "w7"; CAP, ZMAX, K2, ZSPLIT = 0.03, 4.0, 2.0, 1.5


def main():
    t0 = time.time()
    c = np.load(DUMP / "cams.npz"); kf, T, Kr = c["kf"], c["T"].astype(np.float64), c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    base_c = json.loads((SRC / "plan_4m.json").read_text())["base_c"]
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    sel = json.loads((OUT / "probe.json").read_text())["kf_idx"]
    b1 = np.load(A8 / "b1_faces.npz"); sidx = b1["sidx"]
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(MESH)); Vm = np.asarray(m.vertices, np.float32); Fm = np.asarray(m.triangles, np.uint32)
    isobj = np.zeros(len(Fm), bool); isobj[sidx] = True
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(o3d.core.Tensor(Vm), o3d.core.Tensor(Fm))
    vv, uu = np.mgrid[0:H, 0:W]; rc = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, dtype=np.float64)], -1).reshape(-1, 3)

    def pairs(ci):
        fi = int(kf[ci]); L = []
        fb = P2D / "pairs" / f"{fi:05d}.npz"
        if fb.exists():
            d = np.load(fb); Z = d["Z"].astype(np.float32); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
            for p in range(len(d["B"])):
                L.append((Z[p], float(d["f"][p]), float(d["B"][p]), M.sin_map(Kfull, cmap[int(d["partner"][p])], W, H)))
        for tag, fe in (("low", P3D / "extra" / f"{fi:05d}.npz"), ("wide", SRC / "wide" / f"{fi:05d}.npz")):
            if fe.exists():
                d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
                for p in range(len(d["B"])):
                    if tag == "low" and str(d["tag"][p]) != "low":
                        continue
                    L.append((Z[p], float(d["f"][p]), float(d["B"][p]), sn[p]))
        out = []
        for Z, f, B, sn in L:
            sn = np.maximum(sn, 1e-3); sig = (Z ** 2 / (f * B * sn)).astype(np.float32)
            with np.errstate(invalid="ignore"):
                bad = ~((Z / (f * B * sn)) <= CAP) | (Z > ZMAX) | (Z < 0.05)
            out.append((np.where(bad, np.nan, Z).astype(np.float32), np.where(bad, np.inf, sig).astype(np.float32)))
        return out
    cache = {}

    def kfdata(ci):
        if ci in cache:
            return cache[ci]
        Ls = pairs(ci); d = np.load(FUSED / f"{int(kf[ci]):05d}.npz"); Ff = d["F"] == 1
        Zf = np.where(Ff, d["Z"], np.nan).astype(np.float32); Sf = np.where(Ff, d["S"], np.inf).astype(np.float32)
        if Ls:
            Zp = np.stack([q[0] for q in Ls]); Sp = np.stack([q[1] for q in Ls]); nv = np.isfinite(Zp).sum(0)
            bi = np.argmin(Sp, 0); Zb = np.take_along_axis(Zp, bi[None], 0)[0]; Sb = np.take_along_axis(Sp, bi[None], 0)[0]
        else:
            nv = np.zeros((H, W), int); Zb = np.full((H, W), np.nan, np.float32); Sb = np.full((H, W), np.inf, np.float32)
        r = dict(F=Ff, nv=nv, Zb=Zb, Sb=Sb, ref=np.where(Ff, Zf, Zb).astype(np.float32), refS=np.where(Ff, Sf, Sb).astype(np.float32))
        if len(cache) > 40:
            cache.pop(next(iter(cache)))
        cache[ci] = r
        return r

    def project(ci, j, v, u, z):
        X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
        Xj = (X - T[j, :3, 3]) @ T[j, :3, :3]; zj = Xj[:, 2]
        pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
        ok = (zj > 0.1) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
        D = kfdata(j); dj = np.full(len(z), np.nan, np.float32); sj = np.full(len(z), np.inf, np.float32)
        dj[ok] = D["ref"][pv[ok], pu[ok]]; sj[ok] = D["refS"][pv[ok], pu[ok]]
        return zj, dj, sj

    tr = fr.tracks(SimpleNamespace(min_obs=3, reproj=1.0, min_tri=2.0, min_kf=0)); P = np.load(fr.POSES["ba2"] / "poses_all.npz"); fobs = P["frames"][tr["cam"]]
    names = ["fused", "single_le1", "single_1to2", "single_2to3", "single_conf", "single_conf_le2", "single_unconf", "sigcut_3to5", "sigcut_5to10", "sigcut_gt10",
             "range_cut", "none", "other"]
    px = {}; ba = {}; xv = {}
    for ci in sel:
        fi = int(kf[ci]); Tc = T[ci]
        d = rc @ Tc[:3, :3].T; o = np.broadcast_to(Tc[:3, 3], d.shape)
        hit = sc.cast_rays(o3d.core.Tensor(np.concatenate([o, d], 1).astype(np.float32)))
        tt = hit["t_hit"].numpy().reshape(H, W); fid = hit["primitive_ids"].numpy().reshape(H, W).astype(np.int64); hm = np.isfinite(tt)
        Xw = (o + d * np.where(hm, tt, 0).reshape(-1, 1)).reshape(H, W, 3)
        roi = hm & (Xw[..., 0] > -0.3) & (Xw[..., 0] < 3.3) & (Xw[..., 2] > -5.0) & (Xw[..., 2] < -3.2); obj = roi & isobj[np.where(hm, fid, 0)]
        im = cv2.imread(str(DUMP / "img" / f"{fi:05d}.png"), cv2.IMREAD_GRAYSCALE); dark = cv2.GaussianBlur(im.astype(np.float32), (0, 0), 1.0) < 45
        D = kfdata(ci); Ff, nv, Zb, Sb = D["F"], D["nv"], D["Zb"], D["Sb"]
        lab = np.full((H, W), 12, np.int8); cand = np.full((H, W), np.nan, np.float32)
        lab[Ff] = 0; cand[Ff] = D["ref"][Ff]
        single = ~Ff & (nv == 1) & (Zb > ZSPLIT)
        with np.errstate(invalid="ignore", divide="ignore"):
            rz = Sb / Zb
        lab[single & (rz <= 0.01)] = 1; lab[single & (rz > 0.01) & (rz <= 0.02)] = 2; lab[single & (rz > 0.02)] = 3; cand[single] = Zb[single]
        # confirmation (odd offsets) for single pixels inside the roi
        conf = np.zeros((H, W), bool); conf2 = np.zeros((H, W), bool); q = single & roi
        if q.any():
            v, u = np.nonzero(q); z = Zb[v, u].astype(np.float64); s0 = Sb[v, u]; okc = np.zeros(len(z), np.int32)
            for j in range(max(0, ci - 12), min(n, ci + 13)):
                if (j - ci) % 2 == 0:
                    continue
                zj, dj, sj = project(ci, j, v, u, z)
                with np.errstate(invalid="ignore"):
                    okc += np.isfinite(dj) & (np.abs(zj - dj) <= K2 * np.hypot(s0, np.where(np.isfinite(sj), sj, 1e9)))
            conf[v, u] = okc >= 1; conf2[v, u] = okc >= 2
        # no stored value -> uncut sub-classes
        ns = ~Ff & (nv == 0) & ~np.isfinite(Zb)
        fu = UNC / f"{fi:05d}.npz"
        if fu.exists():
            du = np.load(fu); Zu = du["Z"].astype(np.float32); snu = np.maximum(du["sin"].astype(np.float32), 1e-3)
            with np.errstate(invalid="ignore", divide="ignore"):
                ru = Zu / (du["f"][:, None, None] * du["B"][:, None, None] * snu); ru = np.where(np.isfinite(Zu), ru, np.inf)
            bi = np.argmin(ru, 0); Zub = np.take_along_axis(Zu, bi[None], 0)[0]; rub = np.take_along_axis(ru, bi[None], 0)[0]
            inr = np.isfinite(Zu) & (Zu <= ZMAX); ru2 = np.where(inr, ru, np.inf); bi2 = np.argmin(ru2, 0)
            Zin = np.take_along_axis(Zu, bi2[None], 0)[0]; rin = np.take_along_axis(ru2, bi2[None], 0)[0]
            sc_ = ns & np.isfinite(rin)
            lab[sc_ & (rin <= 0.05)] = 7; lab[sc_ & (rin > 0.05) & (rin <= 0.10)] = 8; lab[sc_ & (rin > 0.10)] = 9; cand[sc_] = Zin[sc_]
            rg = ns & ~np.isfinite(rin) & np.isfinite(Zub); lab[rg] = 10; cand[rg] = Zub[rg]
            lab[ns & ~np.isfinite(Zub)] = 11; cone = (snu < np.sin(np.radians(25.0))).all(0); none_cone = ns & ~np.isfinite(Zub) & cone
        else:
            lab[ns] = 11; none_cone = np.zeros((H, W), bool)
        regs = dict(aisle=roi, obj=obj, obj_dark=obj & dark)
        for rn, rm in regs.items():
            px.setdefault(rn, np.zeros(len(names) + 6, np.int64))
            px[rn][:13] += np.bincount(lab[rm], minlength=13)[:13]
            px[rn][13] += int((conf & rm).sum()); px[rn][14] += int((conf & rm & (rz <= 0.02)).sum()); px[rn][15] += int((single & ~conf & rm).sum()); px[rn][16] += int((conf2 & rm).sum()); px[rn][17] += int((none_cone & rm).sum()); px[rn][18] += int((~np.isfinite(D['Zb']) & ~Ff & (nv == 0) & (not fu.exists()) & rm).sum())
        sub = {k: (lab == i) for i, k in enumerate(names) if i not in (4, 5, 6)}
        sub["single_conf"] = conf; sub["single_conf_le2"] = conf & (rz <= 0.02); sub["single_unconf"] = single & roi & ~conf; sub["single_conf2"] = conf2
        mk = fobs == fi
        if mk.any():
            X = tr["Xm"][tr["pt"][mk]]; Xc = (X - Tc[:3, 3]) @ Tc[:3, :3]; z = Xc[:, 2]
            u = np.round(fx * Xc[:, 0] / z + cx).astype(int); v = np.round(fy * Xc[:, 1] / z + cy).astype(int)
            ok = (z > 0.1) & (u >= 0) & (u < W) & (v >= 0) & (v < H); u, v, z = u[ok], v[ok], z[ok]
            for rn, rm in regs.items():
                for k, sm in sub.items():
                    qq = rm[v, u] & sm[v, u]; zc = cand[v[qq], u[qq]]; f_ = np.isfinite(zc)
                    if f_.any():
                        ba.setdefault(rn, {}).setdefault(k, []).append(((zc[f_] - z[qq][f_]) / z[qq][f_]).astype(np.float32))
        g2 = np.zeros((H, W), bool); g2[::2, ::2] = True
        for k, sm in sub.items():
            qq = g2 & roi & sm & np.isfinite(cand)
            if not qq.any():
                continue
            v, u = np.nonzero(qq); z = cand[v, u].astype(np.float64); rels = []
            for j in range(max(0, ci - 12), min(n, ci + 13)):
                if j == ci or (j - ci) % 2 == 1:
                    continue
                zj, dj, _ = project(ci, j, v, u, z)
                with np.errstate(invalid="ignore"):
                    rels.append(np.abs(zj - dj) / dj)
            rels = np.array(rels); cnt = np.isfinite(rels).sum(0); has = cnt >= 2
            if not has.any():
                continue
            med = np.nanmedian(rels[:, has], 0)
            for rn, rm in regs.items():
                s = rm[v, u][has]
                if s.any():
                    xv.setdefault(rn, {}).setdefault(k, []).append(med[s].astype(np.float32))
        print(f"kf idx {ci} obj {np.bincount(lab[obj], minlength=13).tolist()} conf {int((conf & obj).sum())} {time.time() - t0:.0f}s", flush=True)
    allnames = names[:4] + ["(placeholder)"] * 3 + names[7:] + ["single_conf", "single_conf_le2", "single_unconf", "single_conf2", "none_epipole_cone", "none_kf_without_wide_pair"]
    res = dict(made=time.strftime("%F %T"), script="gsm_w7_probe3.py", note=__doc__, kf_idx=sel,
               inputs=dict(probe=str(OUT / "probe.json"), uncut=str(UNC), fused=str(FUSED), mesh=str(MESH), ba=str(fr.POSES["ba2"] / "ba/ba_state.npz")),
               pixels={rn: {k: int(x) for k, x in zip(allnames, v) if k != "(placeholder)"} for rn, v in px.items()})
    def st(dd, isba):
        o = {}
        for k, arrs in dd.items():
            a = np.concatenate(arrs); ab = np.abs(a)
            o[k] = dict(n=int(len(a)), med_pct=round(float(np.median(ab) * 100), 2), p90_pct=round(float(np.percentile(ab, 90) * 100), 2))
            if isba:
                o[k]["signed_med_pct"] = round(float(np.median(a) * 100), 2)
        return o
    res["ba"] = {rn: st(dd, True) for rn, dd in ba.items()}; res["xv_even"] = {rn: st(dd, False) for rn, dd in xv.items()}; res["runtime_s"] = round(time.time() - t0, 1)
    (OUT / "probe3.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: res[k] for k in ("pixels", "ba", "xv_even")}, indent=1))


if __name__ == "__main__":
    main()
