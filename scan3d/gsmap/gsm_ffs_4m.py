#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS depth limit 2 m -> 4 m on top of the adopted pairs3 "L" set, with an error cut instead of a distance cut.

  python gsm_ffs_4m.py plan    RUN --vio VIO --rec REC --p2 P2 --p3 P3 --out OUT   (CPU) -> OUT/plan_4m.json
  python gsm_ffs_4m.py ffs     ...                                                  (GPU) -> OUT/wide/<kf>.npz   (new wide pairs only)
  python gsm_ffs_4m.py fuse    ...                                                  (CPU) -> OUT/s4m/<kf>.npz
  python gsm_ffs_4m.py measure ... --gsmap-run V2                                   (GPU) -> OUT/measure_4m.json (+ fig)

Wide tier: per keyframe up to --n-wide (2) partners in +-win s (VIO relative motion anchored on the keyframe BA pose), B --wb-min..--wb-max
(15-35 cm), perp >= --min-perp (40), epipole outside the image in both frames, rot <= --max-rot (25), dt >= 0.15 s, partner not already a
short/low partner; ranked by B*sin(perp) (desc); diversity as pairs3 (direction >= 10 deg or B ratio >= 1.5, >= 0.2 s apart = 12 frames @ 60 fps, --min-gap-s).
FFS: motion_stereo.pair_depth (full rectification, as pairs2) with cfg.z_max = --zmax (4 m), z_near --wide-znear (1 m, keeps far pixels
near full resolution; pixels nearer than ~1 m fall out).
Error cut (replaces the 2 m cut) per pair, per pixel: sigma_Z/Z = Z*1px/(f*B*sin theta) <= --cap (3 %), theta = pixel ray vs baseline.
Re-used pair outputs: P2/pairs (short, saved with the old 2 m cut) and P3/extra (low tier, 2 m cut) -> they can only supply <= 2 m.
Fusion (far domain): all pairs of the keyframe (short + low + wide) after the error cut, sigma = Z^2/(f B sin theta), pairs3/pairs2 rule
(>= 2 pairs agree within k*hypot(sigma), inverse-variance mean of pairs agreeing with the best-supported pair; single-pair keyframes:
neighbour check). Output: L where L <= --z-split (1.5 m, adopted set unchanged); else the far fusion where it lies in (z-split, zmax].
L pixels > z-split are replaced by / dropped in favour of the error-cut far fusion. T bits: 2 = far fusion pixel, 4 = a wide pair was an inlier.
"""
import argparse, json, os, sys, time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import gsm_ffs_pairs2 as P2
import gsm_ffs_pairs3 as P3
import rec_profile as RP
import motion_stereo as ms


def sin_map(K, c, W, H):
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    r = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1); r /= np.linalg.norm(r, axis=-1, keepdims=True)
    e = np.asarray(c, float) / np.linalg.norm(c); cs = r @ e
    return np.sqrt(np.clip(1 - cs ** 2, 0, 1)).astype(np.float32)


def build4(a):
    b = P3.build(a); kf, Tk, K, W0, H0, t_ns, Tv = b["kf"], b["Tk"], b["K"], b["W0"], b["H0"], b["t_ns"], b["Tv"]
    win = RP.frames(a.win_s, a.fps_used); wide = {}
    for ci in range(len(kf)):
        fi = int(kf[ci])
        if fi not in Tv:
            continue
        used = {q[1] for q in b["base"].get(ci, [])} | {q[1] for q in b["low"].get(ci, [])}
        A = Tk[ci] @ np.linalg.inv(Tv[fi]); cands = []
        for j in range(max(0, fi - win), min(len(t_ns), fi + win + 1)):
            if j == fi or j not in Tv or j in used or abs(t_ns[fi] - t_ns[j]) * 1e-9 < 0.15:
                continue
            Tj = A @ Tv[j]; R, t = ms.rel_pose(Tk[ci], Tj); g = ms.pair_geometry(R, t)
            if not (a.wb_min <= g["B"] <= a.wb_max and g["perp_deg"] >= a.min_perp and g["rot_deg"] <= a.max_rot):
                continue
            if P2.epi_inside(K, g["c"], W0, H0) or P2.epi_inside(K, t, W0, H0):
                continue
            g["j"] = j; cands.append((g["B"] * np.sin(np.radians(g["perp_deg"])), j, g, Tj))
        cands.sort(key=lambda x: -x[0]); ch = []
        for c_ in cands:
            if all(P3.dirdiv(c_[2], q, a.gap_used) for q in ch):
                ch.append(c_)
            if len(ch) >= a.n_wide:
                break
        if ch:
            wide[ci] = ch
    b["wide"] = wide
    return b


def cmd_plan(a):
    b = build4(a); kf = b["kf"]; wide = b["wide"]; out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    pp = lambda v: [[int(q[1]), round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1), round(q[2]["rot_deg"], 1), round(float(q[0]) * 100, 1)] for q in v]
    base_c = {int(kf[ci]): [[int(q[1])] + [float(x) for x in q[2]["c"]] for q in v] for ci, v in b["base"].items()}
    from collections import Counter
    hist = Counter(len(v) for v in wide.values())
    res = dict(made=time.strftime("%F %T"), args=vars(a), base_plan_matches_p2=b["same"], n_kf=len(kf), kf_with_wide=len(wide),
               hist={int(k): int(v) for k, v in hist.items()}, n_wide_pairs=int(sum(len(v) for v in wide.values())),
               B_cm_med=float(np.median([q[2]["B"] * 100 for v in wide.values() for q in v])) if wide else 0,
               perp_med=float(np.median([q[2]["perp_deg"] for v in wide.values() for q in v])) if wide else 0,
               plan={int(kf[ci]): pp(v) for ci, v in sorted(wide.items())}, base_c=base_c)
    (out / "plan_4m.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k not in ("plan", "base_c", "args")}, indent=1, default=float))


def cmd_ffs(a):
    import lib_ffs
    t0 = time.time(); b = build4(a); kf = b["kf"]; K, dist, W0, H0 = b["K"], b["dist"], b["W0"], b["H0"]
    out = Path(a.out); (out / "wide").mkdir(parents=True, exist_ok=True); plan = b["wide"]
    cfg = ms.MSConfig(); cfg.z_max = a.zmax
    rp = dict(size=(W0, H0), f=float(K[0, 0]), cx=float(K[0, 2]), cy=float(K[1, 2]), R1=np.eye(3).tolist())
    need = {}
    for ci, ch in plan.items():
        for q in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    ready = {ci: max([int(kf[ci])] + [q[1] for q in plan[ci]]) for ci in plan}
    last_use = {f_: max(ready[ci] for ci in cis) for f_, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "wide" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"wide plan {len(plan)} kf {sum(len(v) for v in plan.values())} pairs todo {len(todo)} t {time.time() - t0:.0f}s", flush=True)
    model = lib_ffs.load(valid_iters=a.iters)
    cap = cv2.VideoCapture(str(Path(a.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    while ti < len(todo):
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); res = []
            for q in plan[ci]:
                R, t = ms.rel_pose(b["Tk"][ci], q[3])
                res.append(ms.pair_depth(model, lib_ffs.infer, buf[fi], buf[q[1]], R, t, K, dist, rp, cfg, z_near=a.wide_znear, iters=a.iters))
            np.savez_compressed(out / "wide" / f"{fi:05d}.npz", Z=np.stack([np.where(r["valid"], r["Z"], np.nan) for r in res]).astype(np.float16),
                                sin=np.stack([sin_map(K, q[2]["c"], W0, H0) for q in plan[ci]]).astype(np.float16),
                                B=np.array([r["B"] for r in res]), f=np.array([r["f"] for r in res]), partner=np.array([q[1] for q in plan[ci]]),
                                perp=np.array([r["perp_deg"] for r in res]), scale=np.array([r["scale"] for r in res]))
            stats[fi] = dict(valid=[round(float(r["valid"].mean()), 4) for r in res], B_cm=[round(r["B"] * 100, 1) for r in res], scale=[r["scale"] for r in res])
            if ti % 25 == 0:
                print(f"{ti}/{len(todo)} kf {fi} {stats[fi]['valid']} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "wide" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), total_s=round(time.time() - t0, 1), per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    t0 = time.time(); run, out, p2, p3 = Path(a.run), Path(a.out), Path(a.p2), Path(a.p3)
    od = out / a.name; od.mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    pl = json.loads((out / "plan_4m.json").read_text()); base_c = pl["base_c"]
    g = np.load(Path(a.rec) / a.gt / "gt.npz"); K = g["K"].copy(); K[:2] *= W / 1920.0
    k = a.k; Tv = P2.load_common(a)[-1] if a.rescale else None; SC = []

    def bscale(ci, j):
        """audit4 an3s.py: local BA/VIO baseline ratio from the keyframe on the partner's side nearest the partner (<= 30 frames), clip 0.7-1.4."""
        fi = int(kf[ci]); cand = [cj for cj in range(n) if cj != ci and int(kf[cj]) in Tv and (int(kf[cj]) - fi) * (j - fi) > 0]
        if not cand or fi not in Tv:
            return 1.0
        cj = min(cand, key=lambda q: abs(int(kf[q]) - j)); bb = np.linalg.norm(T[cj][:3, 3] - T[ci][:3, 3]); bv = np.linalg.norm(Tv[int(kf[cj])][:3, 3] - Tv[fi][:3, 3])
        return float(np.clip(bb / bv, 0.7, 1.4)) if (bb > 0.02 and bv > 0.02 and abs(int(kf[cj]) - j) <= 30) else 1.0
    PC = {}

    def pairs_of(ci):
        """list of dict(Z error-cut, raw, sig (with sin theta), sigL (L-style: base Z^2/(fB), low with sin), tag), after the optional rescale"""
        if ci in PC:
            return PC[ci]
        fi = int(kf[ci]); L = []

        def add(Z, f, B, sn, tag, j):
            sc = bscale(ci, j) if a.rescale else 1.0; SC.append(sc); Z = Z * sc; B = B * sc
            sn = np.maximum(sn, 1e-3); sig = (Z ** 2 / (f * B * sn)).astype(np.float32)
            sigL = (Z ** 2 / (f * B * (sn if tag == "low" else 1.0))).astype(np.float32)
            with np.errstate(invalid="ignore"):
                bad = ~((Z / (f * B * sn)) <= a.cap) | (Z > a.zmax) | (Z < 0.05)
            L.append(dict(Z=np.where(bad, np.nan, Z).astype(np.float32), raw=Z.astype(np.float32), sig=sig, sigL=sigL, tag=tag))
        fb = p2 / "pairs" / f"{fi:05d}.npz"
        if fb.exists():
            d = np.load(fb); Z = d["Z"].astype(np.float32); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
            for p in range(len(d["B"])):
                j = int(d["partner"][p]); add(Z[p], float(d["f"][p]), float(d["B"][p]), sin_map(K, cmap[j], W, H), "base", j)
        for tag, fe in (("low", p3 / "extra" / f"{fi:05d}.npz"), ("wide", out / "wide" / f"{fi:05d}.npz")):
            if fe.exists():
                d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
                for p in range(len(d["B"])):
                    if tag == "low" and str(d["tag"][p]) != "low":
                        continue
                    add(Z[p], float(d["f"][p]), float(d["B"][p]), sn[p], tag, int(d["partner"][p]))
        if len(PC) > 12:
            PC.pop(next(iter(PC)))
        PC[ci] = L
        return L
    cnt = {}
    for ci in range(n):
        fi = int(kf[ci])
        cnt[ci] = sum(len(np.load(f)["B"]) for f in (p2 / "pairs" / f"{fi:05d}.npz", p3 / "extra" / f"{fi:05d}.npz", out / "wide" / f"{fi:05d}.npz") if f.exists())
    has = [cnt[ci] > 0 for ci in range(n)]; C1 = {}

    def comp(ci, zk, sk, sel):       # neighbour composite (inverse variance) for the single-pair check
        key = (ci, zk, sk)
        if key not in C1:
            Ls = [q for q in pairs_of(ci) if sel(q)]
            if not Ls:
                C1[key] = (np.full((H, W), np.nan, np.float32), np.full((H, W), np.inf, np.float32))
            else:
                Z = np.stack([q[zk] for q in Ls]); w = np.where(np.isfinite(Z), 1 / np.maximum(np.nan_to_num(np.stack([q[sk] for q in Ls]), nan=1), 1e-6) ** 2, 0); den = w.sum(0)
                with np.errstate(invalid="ignore", divide="ignore"):
                    C1[key] = (np.where(den > 0, (np.nan_to_num(Z) * w).sum(0) / den, np.nan).astype(np.float32), np.where(den > 0, 1 / np.sqrt(den), np.inf).astype(np.float32))
        return C1[key]
    vv, uu = np.mgrid[0:H, 0:W]

    def fuse_stack(ci, Ls, zk, sk, sel, weight):
        """pairs2/pairs3 rule: >= 2 pairs agree within k*hypot(sigma); weighted mean of the pairs agreeing with the best-supported pair
        (ivw: w = 1/sigma^2, S = 1/sqrt(sum w); plain: equal weights, S = sqrt(sum sigma^2)/n); single-pair keyframes: neighbour check."""
        Zp = np.stack([q[zk] for q in Ls]); sig = np.stack([q[sk] for q in Ls]); m = len(Ls); val = np.isfinite(Zp); nv = val.sum(0)
        isw = np.array([q["tag"] == "wide" for q in Ls])
        Zf = np.full((H, W), np.nan, np.float32); Sf = np.full((H, W), np.inf, np.float32); Tw = np.zeros((H, W), bool); n_nb = 0
        if m >= 2:
            sup = np.zeros((m, H, W), np.int32)
            for p in range(m):
                for q in range(p + 1, m):
                    with np.errstate(invalid="ignore"):
                        x = val[p] & val[q] & (np.abs(Zp[p] - Zp[q]) <= k * np.hypot(sig[p], sig[q]))
                    sup[p] += x; sup[q] += x
            best = np.argmax(sup - 1e-3 * np.where(val, sig, 1e9).clip(0, 999), 0)
            ok2 = np.take_along_axis(sup, best[None], 0)[0] >= 1
            Zb = np.take_along_axis(Zp, best[None], 0)[0]; sb = np.take_along_axis(sig, best[None], 0)[0]
            num = np.zeros((H, W)); den = np.zeros((H, W)); ssq = np.zeros((H, W))
            for p in range(m):
                with np.errstate(invalid="ignore"):
                    inn = ok2 & val[p] & (np.abs(Zp[p] - Zb) <= k * np.hypot(sig[p], sb))
                if weight == "ivw":
                    w = np.where(inn, 1.0 / np.maximum(np.nan_to_num(sig[p], nan=1.0), 1e-6) ** 2, 0)
                else:
                    w = inn.astype(np.float64); ssq += np.where(inn, np.nan_to_num(sig[p]) ** 2, 0)
                num += np.where(inn, np.nan_to_num(Zp[p]), 0) * w; den += w
                if isw[p]:
                    Tw |= inn
            with np.errstate(invalid="ignore", divide="ignore"):
                Zf[ok2] = (num / den)[ok2]
                Sf[ok2] = (1.0 / np.sqrt(den))[ok2] if weight == "ivw" else (np.sqrt(ssq) / den)[ok2]
        single = (nv == 1) & (m == 1)
        if single.any():
            z = Zp[0][single]; s0 = sig[0][single]; u = uu[single]; v = vv[single]
            X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
            nbs = [j for j in range(ci - 1, -1, -1) if has[j]][:2] + [j for j in range(ci + 1, n) if has[j]][:2]
            okn = np.zeros(len(z), bool)
            for j in nbs:
                Tinv = np.linalg.inv(T[j]); Xj = X @ Tinv[:3, :3].T + Tinv[:3, 3]; zj = Xj[:, 2]
                with np.errstate(invalid="ignore", divide="ignore"):
                    pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                Zn, Sn = comp(j, zk, sk, sel); zr = np.full(len(z), np.nan, np.float32); sr = np.full(len(z), np.inf, np.float32)
                zr[ok] = Zn[pv[ok], pu[ok]]; sr[ok] = Sn[pv[ok], pu[ok]]
                with np.errstate(invalid="ignore"):
                    okn |= np.isfinite(zr) & (np.abs(zj - zr) <= k * np.hypot(s0, sr))
            Zs = Zf[single]; Zs[okn] = z[okn]; Zf[single] = Zs; n_nb = int(okn.sum())
            Ss = Sf[single]; Ss[okn] = s0[okn]; Sf[single] = Ss
            if isw[0]:
                Ts = Tw[single]; Ts[okn] = True; Tw[single] = Ts
        return Zf, Sf, Tw, n_nb
    info = {}; allp = lambda q: True; shortp = lambda q: q["tag"] != "wide"
    for ci in range(n):
        name = f"{int(kf[ci]):05d}.npz"; fL = p3 / "s12_L" / name
        ZL = np.load(fL)["Z"].astype(np.float32) if fL.exists() else np.full((H, W), np.nan, np.float32)
        if not has[ci]:
            if fL.exists():
                Zo = np.where(ZL <= a.z_split, ZL, np.nan).astype(np.float32); V = np.isfinite(Zo)
                dL = np.load(fL); SL = dL["S"] if "S" in dL else np.full((H, W), np.inf, np.float32)
                np.savez_compressed(od / name, Z=Zo, V=V, T=np.zeros((H, W), np.uint8), S=np.where(V, SL, np.inf).astype(np.float32)); info[int(kf[ci])] = dict(pairs=0, valid_frac=float(V.mean()))
            continue
        Ls = pairs_of(ci); m = len(Ls)
        Zf, Sf, Tw, n_nb = fuse_stack(ci, Ls, "Z", "sig", allp, a.weight)          # far domain: error-cut pairs incl. wide
        if a.near == "refuse":     # near domain re-fused from the (optionally rescaled) short + low pairs, L rule (uncut, L-style sigma, 2 m)
            Lsh = [q for q in Ls if shortp(q)]
            if Lsh:
                Zn_, Sn_, _, _ = fuse_stack(ci, Lsh, "raw", "sigL", shortp, a.weight)
            else:
                Zn_ = np.full((H, W), np.nan, np.float32); Sn_ = np.full((H, W), np.inf, np.float32)
            Zn_ = np.where(Zn_ <= 2.0, Zn_, np.nan)
        else:
            Zn_ = ZL; Sn_ = None
        nearL = np.isfinite(Zn_) & (Zn_ <= a.z_split)
        far = np.isfinite(Zf) & (Zf > a.z_split) & (Zf <= a.zmax) & ~nearL
        Zo = np.where(nearL, Zn_, np.where(far, Zf, np.nan)).astype(np.float32); V = np.isfinite(Zo)
        Tg = np.where(far, 2 | np.where(Tw, 4, 0), 0).astype(np.uint8)
        if Sn_ is None:
            # sigma of the kept L pixels: inverse variance over the pairs (uncut, sigma with sin theta) agreeing with L within k*sigma;
            # none agree -> smallest pair sigma at that pixel
            wsum = np.zeros((H, W)); best_s = np.full((H, W), np.inf)
            for q in Ls:
                with np.errstate(invalid="ignore"):
                    agr = nearL & np.isfinite(q["raw"]) & (np.abs(q["raw"] - ZL) <= k * q["sig"])
                wsum += np.where(agr, 1.0 / np.maximum(np.nan_to_num(q["sig"], nan=1.0), 1e-6) ** 2, 0)
                best_s = np.where(nearL & np.isfinite(q["raw"]), np.minimum(best_s, np.nan_to_num(q["sig"], nan=np.inf)), best_s)
            with np.errstate(divide="ignore"):
                Sn_ = np.where(wsum > 0, 1.0 / np.sqrt(wsum), best_s)
        So = np.where(nearL, Sn_, np.where(far, Sf, np.inf)).astype(np.float32)
        np.savez_compressed(od / name, Z=Zo, V=V, T=Tg, S=So)
        info[int(kf[ci])] = dict(pairs=m, n_wide=int(sum(q["tag"] == "wide" for q in Ls)), valid_frac=round(float(V.mean()), 4), px_far=int(far.sum()),
                                 px_far_wide=int((far & Tw).sum()), px_L_far_dropped=int((np.isfinite(ZL) & (ZL > a.z_split) & ~far).sum()), px_single_nb_ok=n_nb)
        if ci % 50 == 0:
            print(ci, info[int(kf[ci])], f"{time.time() - t0:.0f}s", flush=True)
    scs = np.array(SC) if SC else np.ones(1)
    rs = dict(med=float(np.median(scs)), p10=float(np.percentile(scs, 10)), p90=float(np.percentile(scs, 90)), frac_ne1=float(np.mean(scs != 1)))
    (od / "fuse_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), rescale=rs, per_kf=info), indent=1))
    print(f"fuse {a.name}: {len(info)} kf, rescale {rs}, {time.time() - t0:.0f}s", flush=True)


def cmd_measure(a):
    import torch
    t0 = time.time(); R = Path(a.gsmap_run); out, p3 = Path(a.out), Path(a.p3)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)

    SS = {}

    def load(d, cap=None, key=None):
        Z = np.full((n, H, W), np.nan, np.float32); S = np.full((n, H // 2, W // 2), np.nan, np.float32)
        for ci, i in enumerate(kf):
            f = d / f"{int(i):05d}.npz"
            if f.exists():
                dd = np.load(f); Z[ci] = dd["Z"]
                if "S" in dd:
                    S[ci] = dd["S"][::2, ::2]
        if cap:
            Z[Z > cap] = np.nan
        if key:
            SS[key] = S
        return Z
    names = a.name.split(",")
    sets = {"L": load(p3 / "s12_L", key="L")}
    for nm_ in names:
        sets[nm_] = load(out / nm_, a.zcap or None, key=nm_)
    has = {k_: np.isfinite(Z).reshape(n, -1).any(1) for k_, Z in sets.items()}
    dev = "cuda"; Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    gold = np.load(R / "eval2" / "ref" / "gold_ffs.npz")["Z"].astype(np.float32)

    def mvcheck(Z, href):      # pairs3 measure rule; returns per-kf (z, fail) on the 2x grid
        rt = torch.from_numpy(Z).to(dev); outl = [None] * n
        js = [j for j in range(n) if href[j]]; jt = torch.tensor(js, device=dev)
        for ci in range(n):
            Zs = Z[ci, ::2, ::2]; val = np.isfinite(Zs) & vm[::2, ::2]
            if not val.any():
                continue
            v, u = np.nonzero(val); z = torch.from_numpy(Zs[v, u]).float().to(dev)
            ut = torch.from_numpy(u * 2).float().to(dev); vt = torch.from_numpy(v * 2).float().to(dev)
            X = torch.stack([(ut - cx) / fx * z, (vt - cy) / fy * z, z], 1) @ Tc[ci, :3, :3].T + Tc[ci, :3, 3]
            viol = torch.zeros(len(z), dtype=torch.int32, device=dev); chk = torch.zeros_like(viol)
            for k0 in range(0, len(js), 48):
                jj = jt[k0:k0 + 48]; jj = jj[jj != ci]
                if len(jj) == 0:
                    continue
                Xj = torch.einsum("vij,mj->vmi", Tw[jj, :3, :3], X) + Tw[jj, None, :3, 3]; zj = Xj[..., 2]
                pu = torch.round(fx * Xj[..., 0] / zj.clamp(min=1e-3) + cx).long(); pv = torch.round(fy * Xj[..., 1] / zj.clamp(min=1e-3) + cy).long()
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                zr = rt[jj[:, None].expand_as(pu), pv.clamp(0, H - 1), pu.clamp(0, W - 1)]; zr = torch.where(ok, zr, torch.full_like(zr, float("nan")))
                fin = torch.isfinite(zr)
                chk += fin.sum(0).int(); viol += (fin & (zj < zr - torch.clamp(0.03 * zr, min=0.03))).sum(0).int()
            outl[ci] = (v, u, Zs[v, u], ((viol >= 2) & (viol >= 0.3 * chk)).cpu().numpy())
        return outl
    bins = [(0, 1), (1, 2), (2, 3), (3, 4)]
    res = dict(made=time.strftime("%F %T"), inputs=dict(L=str(p3 / "s12_L"), new=[str(out / x) for x in names], gold=str(R / "eval2" / "ref" / "gold_ffs.npz"), zcap=a.zcap),
               note="mv = pairs3 measure rule (gsm_filtmesh), 2x subsampled, each set its own reference; kf_L = the keyframes with L FFS (measure3.json set)",
               adopted_L_ref_fail=19.83, sets={})
    for k_, Z in sets.items():
        mv = mvcheck(Z, has[k_]); V = np.isfinite(Z); vf = V.reshape(n, -1).mean(1)
        rows = [(ci, *mv[ci]) for ci in range(n) if mv[ci] is not None]

        def pct(sel):
            f_ = sum(int(r[4].sum()) for r in rows if sel(r[0])); t_ = sum(len(r[4]) for r in rows if sel(r[0]))
            return round(100 * f_ / max(1, t_), 2)
        zall = Z[V]
        e = dict(kf_with_ffs=int(has[k_].sum()), mv_pct_all=pct(lambda ci: has[k_][ci]), mv_pct_kf_L=pct(lambda ci: has["L"][ci]),
                 valid_med_with_ffs=round(float(np.median(vf[has[k_]])), 4), valid_med_kf_L=round(float(np.median(vf[has["L"]])), 4),
                 depth_p2_p98=[round(float(x), 3) for x in np.percentile(zall, [2, 98])], depth_max=round(float(zall.max()), 3), bins={})
        zz = np.concatenate([r[3] for r in rows]); ff = np.concatenate([r[4] for r in rows])
        cis = np.concatenate([np.full(len(r[3]), r[0]) for r in rows]); vs = np.concatenate([r[1] for r in rows]); us = np.concatenate([r[2] for r in rows])
        gz = gold[cis, vs, us]; sz = SS[k_][cis, vs, us]
        for lo, hi in bins:
            sb = (zz >= lo) & (zz < hi) if hi < 4 else (zz >= lo) & (zz <= hi)
            gm = sb & (gz > 0); ent = dict(px_2x=int(sb.sum()), px_full=int(((Z >= lo) & ((Z < hi) if hi < 4 else (Z <= hi))).sum()),
                                           mv_pct=round(100 * float(ff[sb].mean()), 2) if sb.any() else None, gold_n=int(gm.sum()))
            sm = sb & np.isfinite(sz)
            if sm.any():
                ent["sigma_cm_p10_med_p90"] = [round(float(x) * 100, 2) for x in np.percentile(sz[sm], [10, 50, 90])]
                ent["sigma_rel_med_pct"] = round(float(np.median(sz[sm] / zz[sm])) * 100, 2); ent["sigma_frac_px"] = round(float(sm.sum() / sb.sum()), 3)
            if gm.any():
                er = np.abs(zz[gm] - gz[gm]); ent["gold_within_2cm_1pct"] = round(float((er < 0.02 + 0.01 * gz[gm]).mean()), 4); ent["gold_med_abs_rel"] = round(float(np.median(er / gz[gm])), 4)
            e["bins"][f"{lo}-{hi}m"] = ent
        gm = gz > 0
        if gm.any():
            er = np.abs(zz[gm] - gz[gm]); e["gold"] = dict(n_2x=int(gm.sum()), within_2cm_1pct=round(float((er < 0.02 + 0.01 * gz[gm]).mean()), 4), med_abs_rel=round(float(np.median(er / gz[gm])), 4))
        res["sets"][k_] = e; print(k_, json.dumps(e), f"{time.time() - t0:.0f}s", flush=True)
    zc = a.zcap or a.zmax; res["keep_rule"] = {}
    for nm_ in names:
        nw = res["sets"][nm_]
        b_ok = {b_: (nw["bins"][b_]["mv_pct"] is None or nw["bins"][b_]["mv_pct"] <= 30.0) for b_ in ("2-3m", "3-4m") if float(b_[0]) < zc}
        res["keep_rule"][nm_] = dict(overall_ok=nw["mv_pct_kf_L"] <= 19.83 + 2.0, bins_ok=b_ok, keep=bool(nw["mv_pct_kf_L"] <= 21.83 and all(b_ok.values())))
    jp = out / (a.json or "measure_4m.json"); jp.write_text(json.dumps(res, indent=1)); print("wrote", jp, json.dumps(res["keep_rule"]))
    if a.fig:
        fname = a.fig_set or names[0]; ZN = sets[fname]; far_n = ((ZN > 2.5) & np.isfinite(ZN)).reshape(n, -1).sum(1)
        fk = [int(x) for x in a.fig_kf.split(",") if x]
        far_kf = int(kf[int(np.argmax(far_n))]); fk = [far_kf] + [f_ for f_ in fk if f_ != far_kf]
        rows = []
        for f_ in fk[:4]:
            ci = int(np.nonzero(kf == f_)[0][0]); img = cv2.imread(str(R / "dump_ba" / "img" / f"{f_:05d}.png"))
            tiles = [cv2.resize(img, (480, 270))]
            cv2.putText(tiles[0], f"kf {f_}" + (" (far wall)" if f_ == far_kf else ""), (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            for k_, nm in (("L", "L (2 m)"), (fname, f"new {fname} ({zc:g} m)")):
                Z = sets[k_][ci]; d = np.clip((Z - 0.3) / 3.7, 0, 1); col = cv2.applyColorMap((255 * np.nan_to_num(d)).astype(np.uint8), cv2.COLORMAP_TURBO)
                col[~np.isfinite(Z)] = 0; tl = cv2.resize(col, (480, 270), interpolation=cv2.INTER_NEAREST)
                cv2.putText(tl, f"{nm} valid {np.isfinite(Z).mean():.2f} >2m {(Z > 2).mean():.2f}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2); tiles.append(tl)
            rows.append(np.hstack(tiles))
        fig = np.vstack(rows); cv2.putText(fig, "turbo 0.3-4.0 m (same scale both), black = none", (6, fig.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(a.fig, fig, [cv2.IMWRITE_JPEG_QUALITY, 88]); print("fig", a.fig, fk[:4])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["plan", "ffs", "fuse", "measure"]); ap.add_argument("run")
    ap.add_argument("--vio", default=""); ap.add_argument("--rec", default=""); ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--p2", required=True); ap.add_argument("--p3", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--win-s", type=float, default=2.0); ap.add_argument("--n-pairs", type=int, default=4)
    ap.add_argument("--b-min", type=float, default=0.03); ap.add_argument("--b-max", type=float, default=0.25)
    ap.add_argument("--min-perp", type=float, default=40.0); ap.add_argument("--max-rot", type=float, default=25.0)
    ap.add_argument("--dir-deg", type=float, default=10.0); ap.add_argument("--fb-perp", type=float, default=30.0)
    ap.add_argument("--low-perp", type=float, default=10.0)
    ap.add_argument("--n-wide", type=int, default=2); ap.add_argument("--wb-min", type=float, default=0.15); ap.add_argument("--wb-max", type=float, default=0.35)
    ap.add_argument("--wide-znear", type=float, default=1.0); ap.add_argument("--z-split", type=float, default=1.5)
    ap.add_argument("--zmax", type=float, default=4.0); ap.add_argument("--cap", type=float, default=0.03)
    ap.add_argument("--iters", type=int, default=8); ap.add_argument("--k", type=float, default=2.0)
    ap.add_argument("--weight", default="ivw", choices=["ivw", "plain"]); ap.add_argument("--rescale", type=int, default=0); ap.add_argument("--near", default="L", choices=["L", "refuse"]); ap.add_argument("--name", default="s4m"); ap.add_argument("--zcap", type=float, default=0.0); ap.add_argument("--json", default="")
    ap.add_argument("--gsmap-run", default=""); ap.add_argument("--fig", default=""); ap.add_argument("--fig-set", default=""); ap.add_argument("--fig-kf", default="642,649,656")
    RP.add_args(ap)
    a = ap.parse_args()
    if a.cmd in ("ffs", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap ffs_4m " + a.cmd, torch_cap=True):
            (cmd_ffs if a.cmd == "ffs" else cmd_measure)(a)
    else:
        {"plan": cmd_plan, "fuse": cmd_fuse}[a.cmd](a)
