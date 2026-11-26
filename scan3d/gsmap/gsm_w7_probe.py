#!/usr/bin/env python3
"""gsmap W7 2026-11-27 probe: is the REJECTED stored per-pair FFS depth on the aisle objects accurate, and which fusion rule threw it away?

Keyframes: idx 314-331 + the 10 others with the largest visible area of the audit8 b1 unsupported aisle clusters.
Regions (per keyframe, raycast of ffsonly_ba2/b/carve.ply with the ba2 cams): aisle = hit point in ROI x -0.3..3.3, z -5..-3.2;
obj = hit face in audit8 b1 sidx (C2-kept, FFS-unsupported aisle faces = the dark real objects); dark = blurred grey < 45.
Pair stack = exactly gsm_ffs_fix3.pairs_of inputs (base ffs_pairs2/pairs + sin_map, low ffs_pairs3/extra, wide ffs_relax/src/wide incl relax),
stored Z of these files is ALREADY cut at sigma/Z <= 3 %, Z 0.3-4 m (pairs3.partial_depth) -> pixels cut there show up as "no_stored".
Classes (pixel not in fused -> reason): fused (ffsonly_ba2/fused F==1); near (any pair Z <= 1.5, L domain); single (exactly 1 pair with
fuse-cut Z, keyframe has >= 2 pairs -> never admitted, or 1 pair failing the neighbour check); disagree (>= 2 pairs valid, no k=2 agreement);
cut_at_fuse (a stored value exists but fails the fuse cap/zmax); no_stored (no pair value).
Candidate depth: fused Z / single pair Z / min-sigma pair Z.
Ref a: BA2 tracks (gsm_f_ruler.tracks filter, >= 3 obs, reproj < 1 px, tri > 2 deg), keyframe observations, rel err at the projected pixel.
Ref b: cross-view: back-project candidate, project into the other keyframes within +-12 idx; their depth = fused else min-sigma stored pair;
per pixel median |zj - Dj| / Dj over views (occlusion not removed -> conservative), and share of views agreeing within 3 %.
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

V2 = fr.V2; BAP = V2 / "ba_partners"; P2D = BAP / "ffs_pairs2"; P3D = BAP / "ffs_pairs3"; SRC = V2 / "ffs_relax/src"
DUMP = V2 / "ba_partners2/run/dump_ba"; FUSED = V2 / "ffsonly_ba2/fused"; MESH = V2 / "ffsonly_ba2/b/carve.ply"
A8 = Path("/mnt/c/Users/a3162/AppData/Local/Temp/claude/b2ff6dea35ed22b8/92940a45-822d-4f2e-9a4d-643a3bbb683f/scratchpad/audit8")
OUT = V2 / "w7"; CAP, ZMAX, K2, ZSPLIT = 0.03, 4.0, 2.0, 1.5
CLS = ["fused", "near", "single", "disagree", "cut_at_fuse", "no_stored"]


def main():
    t0 = time.time()
    c = np.load(DUMP / "cams.npz"); kf, T, Kr = c["kf"], c["T"].astype(np.float64), c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    pl = json.loads((SRC / "plan_4m.json").read_text()); base_c = pl["base_c"]
    import gsm_ba_partners as BP
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    b1 = np.load(A8 / "b1_faces.npz"); sidx = b1["sidx"]; ckf = b1["ckf_area"]
    aisle = list(range(314, 332)); score = ckf.sum(0); order = [int(j) for j in np.argsort(-score) if int(j) not in aisle]
    extra = order[:10]; sel = aisle + extra
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
                L.append((Z[p], float(d["f"][p]), float(d["B"][p]), M.sin_map(Kfull, cmap[int(d["partner"][p])], W, H), "base"))
        for tag, fe in (("low", P3D / "extra" / f"{fi:05d}.npz"), ("wide", SRC / "wide" / f"{fi:05d}.npz")):
            if fe.exists():
                d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
                for p in range(len(d["B"])):
                    if tag == "low" and str(d["tag"][p]) != "low":
                        continue
                    L.append((Z[p], float(d["f"][p]), float(d["B"][p]), sn[p], tag))
        out = []
        for Z, f, B, sn, tag in L:
            sn = np.maximum(sn, 1e-3); sig = (Z ** 2 / (f * B * sn)).astype(np.float32)
            with np.errstate(invalid="ignore"):
                bad = ~((Z / (f * B * sn)) <= CAP) | (Z > ZMAX) | (Z < 0.05)
            out.append(dict(raw=Z, Z=np.where(bad, np.nan, Z).astype(np.float32), sig=sig, tag=tag))
        return out

    cache = {}

    def kfdata(ci):
        if ci in cache:
            return cache[ci]
        Ls = pairs(ci); d = np.load(FUSED / f"{int(kf[ci]):05d}.npz"); Ff = d["F"] == 1; Zf = np.where(Ff, d["Z"], np.nan).astype(np.float32)
        if Ls:
            Zp = np.stack([q["Z"] for q in Ls]); Sp = np.stack([np.where(np.isfinite(q["Z"]), q["sig"], np.inf) for q in Ls]); raw = np.stack([q["raw"] for q in Ls])
            Sr = np.stack([np.where(np.isfinite(q["raw"]), q["sig"], np.inf) for q in Ls])
        else:
            Zp = np.full((0, H, W), np.nan, np.float32); Sp = Zp.copy(); raw = Zp.copy(); Sr = Zp.copy()
        nv = np.isfinite(Zp).sum(0); m_ = len(Ls)
        agree = np.zeros((H, W), bool)
        for p in range(m_):
            for q in range(p + 1, m_):
                with np.errstate(invalid="ignore"):
                    agree |= np.isfinite(Zp[p]) & np.isfinite(Zp[q]) & (np.abs(Zp[p] - Zp[q]) <= K2 * np.hypot(Sp[p], Sp[q]))
        if m_:
            bi = np.argmin(Sp, 0); Zb = np.take_along_axis(Zp, bi[None], 0)[0]
            bir = np.argmin(Sr, 0); Zr = np.take_along_axis(raw, bir[None], 0)[0]
        else:
            Zb = np.full((H, W), np.nan, np.float32); Zr = Zb.copy()
        anyraw = np.isfinite(raw).any(0) if m_ else np.zeros((H, W), bool)
        cls = np.full((H, W), 5, np.int8)
        cls[anyraw] = 4
        cls[(nv >= 2)] = 3
        cls[(nv == 1)] = 2
        with np.errstate(invalid="ignore"):
            cls[(cls != 5) & ~Ff & np.isfinite(Zb) & (Zb <= ZSPLIT)] = 1
        cls[Ff] = 0
        cand = np.where(Ff, Zf, np.where(np.isfinite(Zb), Zb, np.nan)).astype(np.float32)   # cut_at_fuse/no_stored: no candidate
        cand_raw = np.where(np.isfinite(cand), cand, Zr).astype(np.float32)                 # cut_at_fuse: raw stored value
        r = dict(cls=cls, cand=cand, cand_raw=cand_raw, m=m_, agree_any=agree, ref=np.where(Ff, Zf, Zb).astype(np.float32))
        if len(cache) > 40:
            cache.pop(next(iter(cache)))
        cache[ci] = r
        return r

    # BA tracks
    tr = fr.tracks(SimpleNamespace(min_obs=3, reproj=1.0, min_tri=2.0, min_kf=0)); P = np.load(fr.POSES["ba2"] / "poses_all.npz")
    frames = P["frames"]; fobs = frames[tr["cam"]]; kfpos = {int(f): i for i, f in enumerate(kf)}
    res = dict(made=time.strftime("%F %T"), script="gsm_w7_probe.py", inputs=dict(cams=str(DUMP / "cams.npz"), fused=str(FUSED), mesh=str(MESH), base=str(P2D / "pairs"),
               low=str(P3D / "extra"), wide=str(SRC / "wide"), ba=str(fr.POSES["ba2"] / "ba/ba_state.npz"), b1=str(A8 / "b1_faces.npz")),
               kf_idx=sel, kf_frames=[int(kf[j]) for j in sel], ba_filter=tr["filt"], note=__doc__)
    acc = {}

    def add(key, cl, arr):
        acc.setdefault(key, {}).setdefault(cl, []).append(arr)
    px = {}
    for ci in sel:
        fi = int(kf[ci]); Tc = T[ci]
        d = rc @ Tc[:3, :3].T; o = np.broadcast_to(Tc[:3, 3], d.shape)
        rays = o3d.core.Tensor(np.concatenate([o, d], 1).astype(np.float32)); hit = sc.cast_rays(rays)
        tt = hit["t_hit"].numpy().reshape(H, W); fid = hit["primitive_ids"].numpy().reshape(H, W).astype(np.int64); hitm = np.isfinite(tt)
        Xw = (o + d * np.where(np.isfinite(tt), tt, 0).reshape(-1, 1)).reshape(H, W, 3)
        roi = hitm & (Xw[..., 0] > -0.3) & (Xw[..., 0] < 3.3) & (Xw[..., 2] > -5.0) & (Xw[..., 2] < -3.2)
        obj = hitm & isobj[np.where(hitm, fid, 0)] & roi
        im = cv2.imread(str(DUMP / "img" / f"{fi:05d}.png"), cv2.IMREAD_GRAYSCALE)
        if im.shape != (H, W):
            im = cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA)
        dark = cv2.GaussianBlur(im.astype(np.float32), (0, 0), 1.0) < 45
        D = kfdata(ci); cls = D["cls"]
        regs = dict(aisle=roi, obj=obj, obj_dark=obj & dark, aisle_dark=roi & dark)
        for rn, rm in regs.items():
            px.setdefault(rn, np.zeros(6, np.int64)); px[rn] += np.bincount(cls[rm], minlength=6)
        # (a) BA
        mk = fobs == fi
        if mk.any():
            X = tr["Xm"][tr["pt"][mk]]; Rw = Tc[:3, :3]; Xc = (X - Tc[:3, 3]) @ Rw; z = Xc[:, 2]
            u = np.round(fx * Xc[:, 0] / z + cx).astype(int); v = np.round(fy * Xc[:, 1] / z + cy).astype(int)
            ok = (z > 0.1) & (u >= 0) & (u < W) & (v >= 0) & (v < H); u, v, z = u[ok], v[ok], z[ok]
            for rn, rm in regs.items():
                s = rm[v, u]
                for k in range(6):
                    q = s & (cls[v, u] == k)
                    zc = D["cand_raw"][v[q], u[q]]; f_ = np.isfinite(zc)
                    if f_.any():
                        add("ba_" + rn, CLS[k], ((zc[f_] - z[q][f_]) / z[q][f_]).astype(np.float32))
        # (b) cross-view on a 2-px grid
        g2 = np.zeros((H, W), bool); g2[::2, ::2] = True
        nbs = [j for j in range(max(0, ci - 12), min(n, ci + 13)) if j != ci]
        for k in range(6):
            q = g2 & roi & (cls == k) & np.isfinite(D["cand_raw"])
            if not q.any():
                continue
            v, u = np.nonzero(q); z = D["cand_raw"][v, u].astype(np.float64)
            X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ Tc[:3, :3].T + Tc[:3, 3]
            rels = np.full((len(nbs), len(z)), np.nan)
            for a_, j in enumerate(nbs):
                Dj = kfdata(j)["ref"]; Tj = T[j]; Xj = (X - Tj[:3, 3]) @ Tj[:3, :3]; zj = Xj[:, 2]
                pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
                okj = (zj > 0.1) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                dj = np.full(len(z), np.nan); dj[okj] = Dj[pv[okj], pu[okj]]
                with np.errstate(invalid="ignore"):
                    rels[a_] = np.abs(zj - dj) / dj
            with np.errstate(invalid="ignore"):
                cnt = np.isfinite(rels).sum(0); has = cnt >= 2
                med = np.nanmedian(np.where(has[None], rels, np.nan)[:, has], 0) if has.any() else np.zeros(0)
                agr = (np.nan_to_num(rels, nan=9) <= 0.03).sum(0)[has] / cnt[has]
            for rn, rm in (("aisle", roi), ("obj", obj), ("obj_dark", obj & dark), ("aisle_dark", roi & dark)):
                s = rm[v, u][has]
                if s.any():
                    add("xv_" + rn, CLS[k], np.stack([med[s], agr[s]], 1).astype(np.float32))
        print(f"kf idx {ci} frame {fi} pairs {D['m']} obj px {int(obj.sum())} cls {np.bincount(cls[obj], minlength=6).tolist()} {time.time() - t0:.0f}s", flush=True)
    res["pixels"] = {rn: dict(zip(CLS, [int(x) for x in v])) for rn, v in px.items()}
    res["pixels_pct"] = {rn: {c_: round(100 * float(x) / max(1, v.sum()), 2) for c_, x in zip(CLS, v)} for rn, v in px.items()}
    st = {}
    for key, dd in acc.items():
        st[key] = {}
        for cl_, arrs in dd.items():
            a = np.concatenate(arrs)
            if key.startswith("ba_"):
                ab = np.abs(a); st[key][cl_] = dict(n=int(len(a)), med_pct=round(float(np.median(ab) * 100), 2), p90_pct=round(float(np.percentile(ab, 90) * 100), 2),
                                                    signed_med_pct=round(float(np.median(a) * 100), 2), within3pct=round(float((ab <= 0.03).mean() * 100), 1))
            else:
                st[key][cl_] = dict(n=int(len(a)), med_pct=round(float(np.median(a[:, 0]) * 100), 2), p90_pct=round(float(np.percentile(a[:, 0], 90) * 100), 2),
                                    view_agree3pct_mean=round(float(a[:, 1].mean() * 100), 1))
    res["err"] = st; res["runtime_s"] = round(time.time() - t0, 1)
    OUT.mkdir(parents=True, exist_ok=True); (OUT / "probe.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: res[k] for k in ("pixels_pct", "err")}, indent=1))


if __name__ == "__main__":
    main()
