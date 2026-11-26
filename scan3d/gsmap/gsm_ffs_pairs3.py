#!/usr/bin/env python3
"""gsmap 2026-11-26: FFS pairs3 = pairs2 (strict + fallback, unchanged) + two extra tiers, partial rectification.

  python gsm_ffs_pairs3.py plan  RUN --vio VIO --rec REC --p2 P2 --out OUT      (CPU)  -> OUT/plan3.json
  python gsm_ffs_pairs3.py ffs   ...                                             (GPU)  -> OUT/extra/<kf>.npz
  python gsm_ffs_pairs3.py fuse  ...                                             (CPU)  -> OUT/s12_L/<kf>.npz
  python gsm_ffs_pairs3.py measure ...                                           (GPU)  -> OUT/measure3.json + fig

Tiers (on top of the adopted pairs2 plan, which is rebuilt with the same args and checked against P2/plan.json):
  low     : keyframes with < 2 partners. +-win s window, VIO relative motion anchored on the keyframe BA pose (as fallback).
            perp >= 10, B 3-25 cm, rot <= 25, dt >= 0.15 s. Rank by B*sin(perp) (desc). Fill to 2 partners.
  Diversity inside a tier: >= 12 frames from every chosen partner and (baseline direction >= 10 deg apart or B ratio >= 1.5).

Partial rectification (low tier): the rectified pair is a set of virtual cameras whose x axis is the baseline e1 (j at +x) and
whose z axis is perpendicular to e1, rotated about e1 to one of 4 azimuth sectors (90 deg each, centre 0 = the optical-axis side).
Any rotation about e1 keeps epipolar lines horizontal. Each keyframe pixel goes to the sector of its azimuth around e1; pixels
with angle theta to the baseline line < --theta-min (25 deg) are excluded (cone around both epipoles). Each sector grid is the
bbox of its pixels (f reduced so the grid stays <= 1.0 Mpx). FFS per sector, disparity -> virtual z -> keyframe depth by
rotation (motion_stereo.warp_depth). Per-pixel cap: sigma_Z/Z = Z*1px/(f*B*sin theta) <= --cap (3 %).
Per-pixel sigma for fusion: Z^2/(f*B*sin theta) (pairs2 pairs: Z^2/(f*B), as gsm_ffs_pairs2 fuse).
Fusion = gsm_ffs_pairs2 cmd_fuse rule (k = 2 agreement, single-pair neighbour check), only for keyframes whose pair set changed;
others are symlinks to P2/s12. Output T = uint8 tag: 1 = a low pair contributed.
"""
import argparse, gzip, json, os, sys, time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
if os.environ.get("FFS_ROOT"):
    sys.path.insert(0, os.environ["FFS_ROOT"])
import gsm_ffs_pairs2 as P2
import motion_stereo as ms


def dirdiv(gg, q):
    return (ms.dir_differs(gg, q[2], 10.0) or max(gg["B"], q[2]["B"]) / min(gg["B"], q[2]["B"]) >= 1.5) and abs(gg["j"] - q[1]) >= 12


def geom(Ti, Tj):
    R, t = ms.rel_pose(Ti, Tj); g = ms.pair_geometry(R, t); return R, t, g


def build(a):
    kf, Tk, Kr, W0, H0, t_ns, K, dist, Tv = P2.load_common(a)
    base, cfg, _ = P2.new_plan(a, kf, Tk, K, W0, H0, t_ns, Tv, Kr)
    ref = json.loads((Path(a.p2) / "plan.json").read_text())["chosen"]["plan"]
    mine = {str(int(kf[c])): [[int(q[1]), q[4], round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1)] for q in v] for c, v in base.items()}
    same = mine == ref
    win = int(round(a.win_s * 60)); n = len(kf)
    low, why = {}, Counter()
    # ---- low tier
    for ci in range(n):
        fi = int(kf[ci]); have = [(q[0], q[1], q[2]) for q in base.get(ci, [])]
        if len(have) >= 2 or fi not in Tv:
            continue
        A = Tk[ci] @ np.linalg.inv(Tv[fi]); cands = []
        for j in range(max(0, fi - win), min(len(t_ns), fi + win + 1)):
            if j == fi or j not in Tv or abs(t_ns[fi] - t_ns[j]) * 1e-9 < 0.15:
                continue
            Tj = A @ Tv[j]; R, t, g = geom(Tk[ci], Tj)
            if not (a.b_min <= g["B"] <= a.b_max and g["perp_deg"] >= a.low_perp and g["rot_deg"] <= a.max_rot):
                continue
            g["j"] = j; cands.append((g["B"] * np.sin(np.radians(g["perp_deg"])), j, g, Tj))
        cands.sort(key=lambda x: -x[0]); ch = []
        for c_ in cands:
            if all(dirdiv(c_[2], q) for q in have + ch):
                ch.append(c_)
            if len(have) + len(ch) >= 2:
                break
        why["low_cands"] += len(cands)
        if ch:
            low[ci] = ch
    return dict(kf=kf, Tk=Tk, Kr=Kr, W0=W0, H0=H0, t_ns=t_ns, K=K, dist=dist, Tv=Tv, base=base, low=low, same=same, why=why)


def cmd_plan(a):
    b = build(a); kf = b["kf"]; base, low = b["base"], b["low"]; n = len(kf)
    fwd = [ci for ci in range(n) if ci not in base]; one = [ci for ci in range(n) if len(base.get(ci, [])) == 1]
    tot = lambda ci: len(base.get(ci, [])) + len(low.get(ci, []))
    pp = lambda v, tag: [[int(q[1]), tag, round(q[2]["B"] * 100, 1), round(q[2]["perp_deg"], 1), round(q[2]["rot_deg"], 1)] for q in v]
    res = dict(made=time.strftime("%F %T"), args=vars(a), base_plan_matches_p2=b["same"], why=dict(b["why"]),
               n_kf=n, n_no_pair_base=len(fwd), n_one_pair_base=len(one),
               low=dict(kf=len(low), pairs=sum(len(v) for v in low.values()), fwd_ge1=sum(len(low.get(c, [])) >= 1 for c in fwd),
                        fwd_ge2=sum(len(low.get(c, [])) >= 2 for c in fwd), one_gain=sum(c in low for c in one),
                        perp_med=float(np.median([q[2]["perp_deg"] for v in low.values() for q in v])) if low else None),
               fwd_total_ge1=sum(tot(c) >= 1 for c in fwd), fwd_total_ge2=sum(tot(c) >= 2 for c in fwd),
               fwd_kf=[int(kf[c]) for c in fwd],
               plan={int(kf[ci]): pp(low.get(ci, []), "low") for ci in sorted(low)})
    Path(a.out).mkdir(parents=True, exist_ok=True)
    (Path(a.out) / "plan3.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({k: v for k, v in res.items() if k not in ("plan", "fwd_kf", "args")}, indent=1, default=float))


# ---------------------------------------------------------------- partial rectification
def partial_depth(model, infer, bgr_i, bgr_j, R, t, K, dist, W0, H0, a, iters=8):
    c = -R.T @ t; B = float(np.linalg.norm(c)); e1 = c / B
    f0 = float(K[0, 0])
    u, v = np.meshgrid(np.arange(W0, dtype=np.float64), np.arange(H0, dtype=np.float64))
    r = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)
    r /= np.linalg.norm(r, axis=-1, keepdims=True)
    cos_t = r @ e1; sin_t = np.sqrt(np.clip(1 - cos_t ** 2, 0, 1))
    zm = np.array([0, 0, 1.0]) + R.T @ np.array([0, 0, 1.0]); e3b = zm - (zm @ e1) * e1; e3b /= np.linalg.norm(e3b); e2b = np.cross(e3b, e1)
    psi = np.degrees(np.arctan2(r @ e2b, r @ e3b))
    sec = (np.round(psi / 90.0).astype(int)) % 4
    keep = sin_t >= np.sin(np.radians(a.theta_min))
    Zout = np.full((H0, W0), np.nan, np.float32); info = []
    for s_ in range(4):
        m = keep & (sec == s_)
        if m.mean() < 0.01:
            continue
        pc = np.radians(90.0 * s_); e3 = np.cos(pc) * e3b + np.sin(pc) * e2b; e2 = np.cross(e3, e1)
        Rm = np.stack([e1, e2, e3])
        rr = r[m] @ Rm.T
        X = rr[:, 0] / rr[:, 2]; Y = rr[:, 1] / rr[:, 2]
        x0, x1 = np.percentile(X, [0.2, 99.8]); y0, y1 = np.percentile(Y, [0.2, 99.8])
        f = f0 * a.f_scale
        if f * f * (x1 - x0) * (y1 - y0) > a.max_px:
            f = np.sqrt(a.max_px / ((x1 - x0) * (y1 - y0)))
        W = int(np.ceil(f * (x1 - x0) / 2) * 2) + 2; H = int(np.ceil(f * (y1 - y0) / 2) * 2) + 2
        Km = ms.K_of(f, -f * x0 + 1, -f * y0 + 1)
        mi = cv2.initUndistortRectifyMap(K, dist, Rm, Km, (W, H), cv2.CV_32FC1)
        mj = cv2.initUndistortRectifyMap(K, dist, Rm @ R.T, Km, (W, H), cv2.CV_32FC1)
        ri = cv2.remap(bgr_i, *mi, cv2.INTER_LINEAR); rj = cv2.remap(bgr_j, *mj, cv2.INTER_LINEAR)
        one = np.full(bgr_i.shape[:2], 255, np.uint8)
        vi = cv2.remap(one, *mi, cv2.INTER_NEAREST, borderValue=0) > 0; vj = cv2.remap(one, *mj, cv2.INTER_NEAREST, borderValue=0) > 0
        if vj.mean() < 0.05:
            info.append(dict(sec=s_, skip="no overlap")); continue
        d_need = f * B / a.z_near; sc = round(float(np.clip(0.92 * 192 / max(d_need, 1.0), 0.35, 1.0)), 3); d_cap = 0.95 * 192 / sc
        disp = infer(model, cv2.cvtColor(ri, cv2.COLOR_BGR2RGB), cv2.cvtColor(rj, cv2.COLOR_BGR2RGB), valid_iters=iters, scale=sc)
        uu = np.arange(W, dtype=np.float32)[None, :]; vv = np.repeat(np.arange(H, dtype=np.int32)[:, None], W, 1)
        uj = uu - disp; uji = np.clip(np.round(uj).astype(np.int32), 0, W - 1)
        ok = np.isfinite(disp) & (disp > 0.5) & (disp < d_cap) & (uj >= 0) & (uj <= W - 1) & vi
        ok &= vj[vv, uji]; ok &= ~ms._uncertain(disp)
        with np.errstate(divide="ignore", invalid="ignore"):
            Zm = (f * B / disp).astype(np.float32)
        ok &= np.isfinite(Zm) & (Zm > 0.05)
        Zk, Vk, _ = ms.warp_depth(Zm, ok, Km, Rm, ms.K_of(f0, K[0, 2], K[1, 2]), (W0, H0))
        # warp_depth uses one focal for both axes of rect0; K has fx == fy here
        put = m & Vk
        Zout[put] = Zk[put]
        info.append(dict(sec=s_, W=W, H=H, f=round(f, 1), scale=sc, px=int(m.sum()), valid=int(put.sum())))
    with np.errstate(invalid="ignore"):
        bad = ~((Zout * 1.0 / (f0 * B * np.maximum(sin_t, 1e-6))) <= a.cap) | (Zout < 0.3) | (Zout > a.zmax)
    Zout[bad] = np.nan
    return Zout, sin_t.astype(np.float32), B, f0, info


def cmd_ffs(a):
    import lib_ffs
    t0 = time.time(); b = build(a); kf = b["kf"]; out = Path(a.out); (out / "extra").mkdir(parents=True, exist_ok=True)
    plan = {ci: [(q, "low") for q in b["low"][ci]] for ci in b["low"]}
    need = {}
    for ci, ch in plan.items():
        for q, _ in ch:
            need.setdefault(q[1], []).append(ci)
        need.setdefault(int(kf[ci]), []).append(ci)
    ready = {ci: max([int(kf[ci])] + [q[1] for q, _ in plan[ci]]) for ci in plan}
    last_use = {f_: max(ready[ci] for ci in cis) for f_, cis in need.items()}
    todo = sorted([ci for ci in plan if not (out / "extra" / f"{int(kf[ci]):05d}.npz").exists()], key=lambda ci: ready[ci])
    print(f"plan {len(plan)} kf {sum(len(v) for v in plan.values())} pairs todo {len(todo)}", flush=True)
    model = lib_ffs.load(valid_iters=a.iters)
    cap = cv2.VideoCapture(str(Path(a.rec) / "cam5.mp4")); buf = {}; fidx = -1; ti = 0; stats = {}
    W0, H0 = b["W0"], b["H0"]
    while ti < len(todo):
        r_, fr = cap.read(); fidx += 1
        if not r_:
            break
        if fidx in need:
            buf[fidx] = cv2.resize(fr, (W0, H0), interpolation=cv2.INTER_AREA) if fr.shape[1] != W0 else fr
        while ti < len(todo) and ready[todo[ti]] <= fidx:
            ci = todo[ti]; ti += 1; fi = int(kf[ci]); Zs, Ss, Bs, inf = [], [], [], []
            for q, tag in plan[ci]:
                R, t = ms.rel_pose(b["Tk"][ci], q[3])
                Z, st, B, f0, info = partial_depth(model, lib_ffs.infer, buf[fi], buf[q[1]], R, t, b["K"], b["dist"], W0, H0, a, a.iters)
                Zs.append(Z); Ss.append(st); Bs.append(B); inf.append(dict(tag=tag, j=q[1], valid=round(float(np.isfinite(Z).mean()), 4), sec=info))
            np.savez_compressed(out / "extra" / f"{fi:05d}.npz", Z=np.stack(Zs).astype(np.float16), sin=np.stack(Ss).astype(np.float16), B=np.array(Bs),
                                f=np.full(len(Bs), float(b["K"][0, 0])), partner=np.array([q[1] for q, _ in plan[ci]]), tag=np.array([t_ for _, t_ in plan[ci]]),
                                perp=np.array([q[2]["perp_deg"] for q, _ in plan[ci]]))
            stats[fi] = inf
            if ti % 20 == 0:
                print(f"{ti}/{len(todo)} kf {fi} {[x['valid'] for x in inf]} t {time.time() - t0:.0f}s", flush=True)
        for fdel in [k for k in buf if last_use[k] < fidx]:
            del buf[fdel]
    cap.release()
    (out / "extra" / "ffs_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), total_s=round(time.time() - t0, 1), per_kf=stats), indent=1, default=float))
    print(f"done {len(stats)} kf {time.time() - t0:.0f}s", flush=True)


def cmd_fuse(a):
    """pairs2 cmd_fuse rule with per-pixel sigma; low tier added (variant L)."""
    t0 = time.time(); run, out, p2 = Path(a.run), Path(a.out), Path(a.p2)
    tiers = {"low"}; a.variant = "L"
    od = out / f"s12_{a.variant}"; od.mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]

    def pairs_of(ci):
        Zl, Sl, tg = [], [], []
        fb = p2 / "pairs" / f"{int(kf[ci]):05d}.npz"
        if fb.exists():
            d = np.load(fb); Z = d["Z"].astype(np.float32)
            for p in range(len(d["B"])):
                Zl.append(Z[p]); Sl.append(Z[p] ** 2 / (d["f"][p] * d["B"][p])); tg.append("base")
        fe = out / "extra" / f"{int(kf[ci]):05d}.npz"
        if fe.exists():
            d = np.load(fe); Z = d["Z"].astype(np.float32); sn = d["sin"].astype(np.float32)
            for p in range(len(d["B"])):
                if str(d["tag"][p]) in tiers:
                    Zl.append(Z[p]); Sl.append(Z[p] ** 2 / (d["f"][p] * d["B"][p] * np.maximum(sn[p], 1e-3))); tg.append(str(d["tag"][p]))
        return Zl, Sl, tg
    cnt = [len(pairs_of(ci)[0]) for ci in range(n)]
    changed = [any(t_ != "base" for t_ in pairs_of(ci)[2]) or (cnt[ci] == 1) for ci in range(n)]
    has = [x > 0 for x in cnt]; S1 = {}

    def s1(ci):
        if ci not in S1:
            f1 = p2 / "s1" / f"{int(kf[ci]):05d}.npz"
            if f1.exists():
                S1[ci] = np.load(f1)["Z"]
            else:
                Zl, Sl, _ = pairs_of(ci); Z = np.stack(Zl); w = np.where(np.isfinite(Z), 1 / np.nan_to_num(np.stack(Sl), nan=1) ** 2, 0); den = w.sum(0)
                with np.errstate(invalid="ignore", divide="ignore"):
                    S1[ci] = np.where(den > 0, (np.nan_to_num(Z) * w).sum(0) / den, np.nan).astype(np.float32)
        return S1[ci]
    vv, uu = np.mgrid[0:H, 0:W]; info = {}
    for ci in range(n):
        name = f"{int(kf[ci]):05d}.npz"
        if not has[ci]:
            continue
        if not changed[ci]:
            if not (od / name).exists():
                os.symlink(os.path.relpath(p2 / "s12" / name, od), od / name)
            continue
        Zl, Sl, tg = pairs_of(ci); Zp = np.stack(Zl); sig = np.stack(Sl); m = len(Zl); val = np.isfinite(Zp); nv = val.sum(0)
        isx = np.array([t_ != "base" for t_ in tg]); isl = np.array([t_ == "low" for t_ in tg])
        Zo = np.full((H, W), np.nan, np.float32); So = np.full((H, W), np.inf, np.float32); Tg = np.zeros((H, W), np.uint8)
        if m >= 2:
            sup = np.zeros((m, H, W), np.int32)
            for p in range(m):
                for q in range(m):
                    if p != q:
                        with np.errstate(invalid="ignore"):
                            sup[p] += (val[p] & val[q] & (np.abs(Zp[p] - Zp[q]) <= a.k * np.hypot(sig[p], sig[q])))
            best = np.argmax(sup - 1e-3 * np.nan_to_num(sig, nan=1e9).clip(0, 999), 0)
            ok2 = np.take_along_axis(sup, best[None], 0)[0] >= 1
            Zb = np.take_along_axis(Zp, best[None], 0)[0]; sb = np.take_along_axis(sig, best[None], 0)[0]
            num = np.zeros((H, W)); den = np.zeros((H, W))
            for p in range(m):
                with np.errstate(invalid="ignore"):
                    inn = ok2 & val[p] & (np.abs(Zp[p] - Zb) <= a.k * np.hypot(sig[p], sb))
                w = np.where(inn, 1.0 / np.maximum(np.nan_to_num(sig[p], nan=1.0), 1e-6) ** 2, 0)
                num += np.where(inn, np.nan_to_num(Zp[p]), 0) * w; den += w
                if isl[p]:
                    Tg[inn] |= 1
            with np.errstate(invalid="ignore", divide="ignore"):
                Zo[ok2] = (num / den)[ok2]; So[ok2] = (1.0 / np.sqrt(den))[ok2]
        else:
            ok2 = np.zeros((H, W), bool)
        single = (nv == 1) & (m == 1); n_nb = 0
        if single.any():
            z = Zp[0][single]; s0 = sig[0][single]; u = uu[single]; v = vv[single]
            X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
            nbs = [j for j in range(ci - 1, -1, -1) if has[j]][:2] + [j for j in range(ci + 1, n) if has[j]][:2]
            okn = np.zeros(len(z), bool)
            for j in nbs:
                Tw = np.linalg.inv(T[j]); Xj = X @ Tw[:3, :3].T + Tw[:3, 3]; zj = Xj[:, 2]
                with np.errstate(invalid="ignore", divide="ignore"):
                    pu = np.round(fx * Xj[:, 0] / np.maximum(zj, 1e-3) + cx).astype(int); pv = np.round(fy * Xj[:, 1] / np.maximum(zj, 1e-3) + cy).astype(int)
                ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H)
                Zj = s1(j); zr = np.full(len(z), np.nan, np.float32); zr[ok] = Zj[pv[ok], pu[ok]]
                sr = zr ** 2 / (fx * 0.08)
                with np.errstate(invalid="ignore"):
                    okn |= np.isfinite(zr) & (np.abs(zj - zr) <= a.k * np.hypot(s0, sr))
            Zs = Zo[single]; Zs[okn] = z[okn]; Zo[single] = Zs
            Ss = So[single]; Ss[okn] = s0[okn]; So[single] = Ss; n_nb = int(okn.sum())
            if isx[0]:
                Ts = Tg[single]; Ts[okn] |= 1; Tg[single] = Ts
        V = np.isfinite(Zo)
        np.savez_compressed(od / name, Z=Zo, V=V, S=np.where(V, So, np.inf).astype(np.float32), T=Tg)
        info[int(kf[ci])] = dict(pairs=m, tags=tg, valid_frac=float(V.mean()), px_low=int((Tg & 1).astype(bool).sum()), px_single_nb_ok=n_nb)
    (od / "fuse_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), args=vars(a), recomputed=len(info), per_kf=info), indent=1))
    print(f"fuse {a.variant}: recomputed {len(info)} kf, {time.time() - t0:.0f}s", flush=True)


def cmd_measure(a):
    import torch
    t0 = time.time(); R = Path(a.gsmap_run); out, p2 = Path(a.out), Path(a.p2)
    c = np.load(R / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W, H), cv2.CV_32FC1)
    vm = cv2.erode(((mx >= 0) & (mx <= W - 1) & (my >= 0) & (my <= H - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    pl = json.loads((out / "plan3.json").read_text()); fwd = set(pl["fwd_kf"])
    p2plan = json.loads((p2 / "plan.json").read_text())["chosen"]["plan"]; nbase = np.array([len(p2plan.get(str(int(i)), [])) for i in kf])

    def load(d, key="Z"):
        Z = np.full((n, H, W), np.nan if key == "Z" else 0, np.float32 if key == "Z" else np.uint8)
        for ci, i in enumerate(kf):
            f = d / f"{int(i):05d}.npz"
            if f.exists():
                dd = np.load(f)
                if key in dd:
                    Z[ci] = dd[key]
        return Z
    sets = {"s12": load(p2 / "s12")}; tags = {}
    for v in ("L",):
        if (out / f"s12_{v}").exists():
            sets[v] = load(out / f"s12_{v}"); tags[v] = load(out / f"s12_{v}", "T")
    has = {k: np.isfinite(Z).reshape(n, -1).any(1) for k, Z in sets.items()}
    dev = "cuda"; Tw = torch.from_numpy(np.linalg.inv(T)).float().to(dev); Tc = torch.from_numpy(T).float().to(dev)
    gold = np.load(R / "eval2" / "ref" / "gold_ffs.npz")["Z"].astype(np.float32)

    def mvcheck(Z, ref, href):
        rt = torch.from_numpy(ref).to(dev); res = np.zeros((n, 2), np.int64)
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
            res[ci] = [int(((viol >= 2) & (viol >= 0.3 * chk)).sum()), len(z)]
        return res
    pct = lambda r, m: round(100 * r[m, 0].sum() / max(1, r[m, 1].sum()), 2)
    fm = np.array([int(i) in fwd for i in kf]); rr = np.hypot(*np.meshgrid((np.arange(W) - cx) / (W / 2), (np.arange(H) - cy) / (H / 2)))  # 1 = edge midpoints
    centre = rr < 0.5
    res = dict(made=time.strftime("%F %T"), inputs=dict(p2=str(p2), out=str(out), gold=str(R / "eval2" / "ref" / "gold_ffs.npz")),
               note="mv = gsm_filtmesh rule, 2x subsampled, each set its own reference (as measure.json mv_self); centre = normalised radius < 0.5",
               n_fwd=len(fwd), sets={})

    def gold_ag(Z):
        Zs = Z[:, ::2, ::2]; gm = gold > 0; both = gm & np.isfinite(Zs)
        if not both.any():
            return dict(n=0)
        e = np.abs(Zs[both] - gold[both]); return dict(n=int(both.sum()), within_2cm_1pct=round(float((e < 0.02 + 0.01 * gold[both]).mean()), 4), med_abs_rel=round(float(np.median(e / gold[both])), 4))
    base = sets["s12"]
    for k, Z in sets.items():
        mv = mvcheck(Z, Z, has[k]); V = np.isfinite(Z); vf = V.reshape(n, -1).mean(1)
        diff = np.array([not np.array_equal(np.nan_to_num(Z[ci], nan=-1), np.nan_to_num(base[ci], nan=-1)) for ci in range(n)])
        e = dict(kf_with_ffs=int(has[k].sum()), mv_pct_all=pct(mv, has[k]), mv_pct_old411=pct(mv, has["s12"]), valid_med_with_ffs=round(float(np.median(vf[has[k]])), 4),
                 fwd_kf_with_depth=int((has[k] & fm).sum()), fwd_valid_med=round(float(np.median(vf[fm])), 4), fwd_valid_mean=round(float(vf[fm].mean()), 4),
                 fwd_mv_pct=pct(mv, has[k] & fm), changed_kf=int(diff.sum()), changed_base_ge2=int((diff & (nbase >= 2)).sum()),
                 changed_base_lt2=int((diff & (nbase < 2)).sum()), changed_base_eq2=int((diff & (nbase == 2)).sum()), gold=gold_ag(Z))
        if k in tags:
            for bit, nm in ((1, "low"),):
                msk = (tags[k] & bit) > 0
                if not msk.any():
                    continue
                Zt = np.where(msk, Z, np.nan); mvt = mvcheck(Zt, Z, has[k]); ht = msk.reshape(n, -1).any(1)
                e[nm] = dict(kf=int(ht.sum()), fwd_kf=int((ht & fm).sum()), px=int(msk.sum()), mv_pct=pct(mvt, ht), mv_pct_fwd=pct(mvt, ht & fm),
                             centre_frac=round(float((msk & centre[None]).sum() / msk.sum()), 3), gold=gold_ag(Zt),
                             fwd_valid_med=round(float(np.median(msk[fm].reshape(int(fm.sum()), -1).mean(1))), 4))
        res["sets"][k] = e; print(k, json.dumps(e), f"{time.time() - t0:.0f}s", flush=True)
    res["s12_ref_fail"] = 20.64
    (out / "measure3.json").write_text(json.dumps(res, indent=1))
    if a.fig:
        vs = [v for v in ("L",) if v in sets]
        fk = [int(x) for x in a.fig_kf.split(",")] if a.fig_kf else []
        if len(fk) < 4:
            best = sorted([ci for ci in range(n) if fm[ci] and int(kf[ci]) not in fk], key=lambda ci: -np.isfinite(sets[vs[-1]][ci]).mean())
            fk += [int(kf[ci]) for ci in best[: 4 - len(fk)]]
        rows = []
        for f_ in fk:
            ci = int(np.nonzero(kf == f_)[0][0]); img = cv2.imread(str(R / "dump_ba" / "img" / f"{f_:05d}.png"))
            tiles = [cv2.resize(img, (480, 270))]
            cv2.putText(tiles[0], f"kf idx {ci} frame {f_}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            for k in ["s12"] + vs:
                Z = sets[k][ci]; d = np.clip((Z - 0.3) / 1.7, 0, 1); col = cv2.applyColorMap((255 * np.nan_to_num(d)).astype(np.uint8), cv2.COLORMAP_TURBO)
                col[~np.isfinite(Z)] = 0; tl = cv2.resize(col, (480, 270), interpolation=cv2.INTER_NEAREST)
                nm = {"s12": "s12", "L": "+low-angle"}[k]
                cv2.putText(tl, f"{nm} valid {np.isfinite(Z).mean():.2f}", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2); tiles.append(tl)
            rows.append(np.hstack(tiles))
        fig = np.vstack(rows); cv2.putText(fig, "turbo 0.3-2.0 m, black = none", (6, fig.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imwrite(a.fig, fig, [cv2.IMWRITE_JPEG_QUALITY, 88]); print("fig", a.fig, fk)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["plan", "ffs", "fuse", "measure"]); ap.add_argument("run")
    ap.add_argument("--vio", default=""); ap.add_argument("--rec", default=""); ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--p2", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--win-s", type=float, default=2.0); ap.add_argument("--n-pairs", type=int, default=4)
    ap.add_argument("--b-min", type=float, default=0.03); ap.add_argument("--b-max", type=float, default=0.25)
    ap.add_argument("--min-perp", type=float, default=40.0); ap.add_argument("--max-rot", type=float, default=25.0)
    ap.add_argument("--dir-deg", type=float, default=10.0); ap.add_argument("--fb-perp", type=float, default=30.0); ap.add_argument("--zmax", type=float, default=2.0)
    ap.add_argument("--iters", type=int, default=8); ap.add_argument("--k", type=float, default=2.0)
    ap.add_argument("--low-perp", type=float, default=10.0)
    ap.add_argument("--theta-min", type=float, default=25.0); ap.add_argument("--cap", type=float, default=0.03)
    ap.add_argument("--f-scale", type=float, default=1.0); ap.add_argument("--max-px", type=float, default=1.0e6); ap.add_argument("--z-near", type=float, default=0.4)
    ap.add_argument("--gsmap-run", default=""); ap.add_argument("--fig", default=""); ap.add_argument("--fig-kf", default="762")
    a = ap.parse_args()
    if a.cmd in ("ffs", "measure"):
        import gpu_guard
        with gpu_guard.job("gsmap ffs_pairs3 " + a.cmd, torch_cap=True):
            (cmd_ffs if a.cmd == "ffs" else cmd_measure)(a)
    else:
        {"plan": cmd_plan, "fuse": cmd_fuse}[a.cmd](a)
