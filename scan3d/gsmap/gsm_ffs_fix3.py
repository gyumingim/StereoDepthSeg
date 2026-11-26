#!/usr/bin/env python3
"""gsmap 2026-11-26: three FFS fusion fixes on top of gsm_ffs_4m (s4m), as reusable fusion options. CPU only, single thread.

  python gsm_ffs_fix3.py fuse  RUN --vio VIO --rec REC --p2 P2 --p3 P3 --src FFS4M --out OUT --name NAME [fix options]  -> OUT/NAME/<kf>.npz
  python gsm_ffs_fix3.py kfit  RUN --set DIR --kout FILE.npz [--parity all|odd]                                          -> per-kf scale file
  python gsm_ffs_fix3.py judge RUN --ref-set S4M --gold GOLD --sets name=dir,... --json OUT.json                         -> metrics (merged into json)

Fusion = gsm_ffs_4m.py fuse (same rule, same npz Z/V/T/S), reading plan_4m.json + wide/ from --src, plus:
 fix 1  --kscale FILE  per-keyframe scale s (from kfit) applied to every pair of the keyframe before fusion (Z*s, B*s -> sigma*s), and to
                       the L near pixels when --near L. kfit: half-res depth, query step 6; for each keyframe and each time neighbour (<= 300
                       frames) the median log depth ratio at overlapping pixels (|log| <= 0.25, >= 30 px), weight sqrt(n); Huber-IRLS least
                       squares on log s (prior 0.3 to 1.0), clipped to [--kmin, --kmax] (0.9-1.1). --parity odd fits only neighbours whose
                       keyframe index differs by an odd number (honest test: judge then uses even offsets only).
 fix 2  --rs-exp E     partial BA/VIO rescale: pair scale = (an3s local BA/VIO baseline ratio, clip 0.7-1.4) ** E  (E=1 == s4m_rs, 0 == off).
 fix 3  --dark-thr L --dark-mode drop|w --dark-w W   pixels with luminance (gray, Gaussian sigma 1) < L in the keyframe image: dropped from
                       every pair, or sigma * 1/sqrt(W) (weight W) in the fusion.
Judge = audit5 rule (gsm_filtmesh mvcheck at half res, query step 6, every keyframe with depth as a view; reproduces 19.83 % on s12_L);
fail counted on the reference set's keyframes; --views even = only views with an even keyframe-index offset (held-out for --parity odd).
Gold: eval2/ref/gold_ffs.npz on the 2x grid inside the undistortion mask (as measure_4m), within 2 cm + 1 %.
"""
import argparse, json, os, sys, time
for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "NUMBA_NUM_THREADS"):
    os.environ[_k] = "1"
os.environ["CUDA_VISIBLE_DEVICES"] = ""
from pathlib import Path

import cv2
import numpy as np
cv2.setNumThreads(1)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "gsplat"))
import gsm_ffs_4m as M
P2 = M.P2


def cmd_fuse(a):
    t0 = time.time(); run, out, src, p2, p3 = Path(a.run), Path(a.out), Path(a.src or a.out), Path(a.p2), Path(a.p3)
    od = out / a.name; od.mkdir(parents=True, exist_ok=True)
    c = np.load(run / "dump_ba" / "cams.npz"); kf, T, Kr = c["kf"], c["T"], c["Kr"]; W, H = [int(v) for v in c["size"]]; n = len(kf)
    fx, fy, cx, cy = [float(v) for v in (Kr[0, 0], Kr[1, 1], Kr[0, 2], Kr[1, 2])]
    pl = json.loads((src / "plan_4m.json").read_text()); base_c = pl["base_c"]
    g = np.load(Path(a.rec) / a.gt / "gt.npz"); K = g["K"].copy(); K[:2] *= W / 1920.0
    k = a.k; Tv = P2.load_common(a)[-1] if a.rs_exp > 0 else None; SC = []
    KS = np.ones(n)
    if a.kscale:
        d = np.load(a.kscale); mp = {int(f): float(s) for f, s in zip(d["kf"], d["s"])}; KS = np.array([mp.get(int(f), 1.0) for f in kf])
    dk_cache = {}

    def dark(ci):
        if not a.dark_thr:
            return None
        if ci not in dk_cache:
            im = cv2.imread(str(run / "dump_ba" / "img" / f"{int(kf[ci]):05d}.png"), cv2.IMREAD_GRAYSCALE)
            if im is None:
                raise FileNotFoundError(f"image for kf {int(kf[ci])}")
            if im.shape != (H, W):
                im = cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA)
            if len(dk_cache) > 16:
                dk_cache.pop(next(iter(dk_cache)))
            dk_cache[ci] = cv2.GaussianBlur(im.astype(np.float32), (0, 0), 1.0) < a.dark_thr
        return dk_cache[ci]

    def bscale(ci, j):
        """audit4 an3s.py: local BA/VIO baseline ratio from the keyframe on the partner's side nearest the partner (<= 30 frames), clip 0.7-1.4."""
        fi = int(kf[ci]); cand = [cj for cj in range(n) if cj != ci and int(kf[cj]) in Tv and (int(kf[cj]) - fi) * (j - fi) > 0]
        if not cand or fi not in Tv:
            return 1.0
        cj = min(cand, key=lambda q: abs(int(kf[q]) - j)); bb = np.linalg.norm(T[cj][:3, 3] - T[ci][:3, 3]); bv = np.linalg.norm(Tv[int(kf[cj])][:3, 3] - Tv[fi][:3, 3])
        return float(np.clip(bb / bv, 0.7, 1.4)) if (bb > 0.02 and bv > 0.02 and abs(int(kf[cj]) - j) <= 30) else 1.0
    PC = {}

    def pairs_of(ci):
        if ci in PC:
            return PC[ci]
        fi = int(kf[ci]); L = []; dk = dark(ci); inf = 1.0 / np.sqrt(a.dark_w)

        def add(Z, f, B, sn, tag, j):
            rs = bscale(ci, j) ** a.rs_exp if a.rs_exp > 0 else 1.0; SC.append(rs); sc = rs * KS[ci]; Z = Z * sc; B = B * sc
            if dk is not None and a.dark_mode == "drop":
                Z = np.where(dk, np.nan, Z)
            sn = np.maximum(sn, 1e-3); sig = (Z ** 2 / (f * B * sn)).astype(np.float32)
            sigL = (Z ** 2 / (f * B * (sn if tag == "low" else 1.0))).astype(np.float32)
            if dk is not None and a.dark_mode == "w":
                sig = np.where(dk, sig * inf, sig).astype(np.float32); sigL = np.where(dk, sigL * inf, sigL).astype(np.float32)
            with np.errstate(invalid="ignore"):
                bad = ~((Z / (f * B * sn)) <= a.cap) | (Z > a.zmax) | (Z < 0.05)
            L.append(dict(Z=np.where(bad, np.nan, Z).astype(np.float32), raw=Z.astype(np.float32), sig=sig, sigL=sigL, tag=tag))
        fb = p2 / "pairs" / f"{fi:05d}.npz"
        if fb.exists():
            d = np.load(fb); Z = d["Z"].astype(np.float32); cmap = {int(r[0]): r[1:] for r in base_c.get(str(fi), [])}
            for p in range(len(d["B"])):
                j = int(d["partner"][p]); add(Z[p], float(d["f"][p]), float(d["B"][p]), M.sin_map(K, cmap[j], W, H), "base", j)
        for tag, fe in (("low", p3 / "extra" / f"{fi:05d}.npz"), ("wide", src / "wide" / f"{fi:05d}.npz")):
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
        cnt[ci] = sum(len(np.load(f)["B"]) for f in (p2 / "pairs" / f"{fi:05d}.npz", p3 / "extra" / f"{fi:05d}.npz", src / "wide" / f"{fi:05d}.npz") if f.exists())
    has = [cnt[ci] > 0 for ci in range(n)]; C1 = {}

    def comp(ci, zk, sk, sel):
        key = (ci, zk, sk)
        if key not in C1:
            Ls = [q for q in pairs_of(ci) if sel(q)]
            if not Ls:
                C1[key] = (np.full((H, W), np.nan, np.float32), np.full((H, W), np.inf, np.float32))
            else:
                Z = np.stack([q[zk] for q in Ls]); w = np.where(np.isfinite(Z), 1 / np.maximum(np.nan_to_num(np.stack([q[sk] for q in Ls]), nan=1), 1e-6) ** 2, 0); den = w.sum(0)
                with np.errstate(invalid="ignore", divide="ignore"):
                    C1[key] = (np.where(den > 0, (np.nan_to_num(Z) * w).sum(0) / den, np.nan).astype(np.float32), np.where(den > 0, 1 / np.sqrt(den), np.inf).astype(np.float32))
            if len(C1) > 24:
                C1.pop(next(iter(C1)))
        return C1[key]
    vv, uu = np.mgrid[0:H, 0:W]

    def fuse_stack(ci, Ls, zk, sk, sel, weight):
        """verbatim gsm_ffs_4m.fuse_stack"""
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

    def loadL(ci):
        fL = p3 / "s12_L" / f"{int(kf[ci]):05d}.npz"
        if not fL.exists():
            return None, None
        dL = np.load(fL); ZL = dL["Z"].astype(np.float32) * np.float32(KS[ci]); SL = dL["S"] * np.float32(KS[ci]) if "S" in dL else np.full((H, W), np.inf, np.float32)
        dk = dark(ci)
        if dk is not None:
            if a.dark_mode == "drop":
                ZL = np.where(dk, np.nan, ZL).astype(np.float32)
            else:
                SL = np.where(dk, SL / np.sqrt(a.dark_w), SL)
        return ZL, SL
    info = {}; allp = lambda q: True; shortp = lambda q: q["tag"] != "wide"
    for ci in range(n):
        name = f"{int(kf[ci]):05d}.npz"; ZL, SL = loadL(ci)
        if ZL is None:
            ZL = np.full((H, W), np.nan, np.float32)
        if not has[ci]:
            if SL is not None:
                Zo = np.where(ZL <= a.z_split, ZL, np.nan).astype(np.float32); V = np.isfinite(Zo)
                np.savez_compressed(od / name, Z=Zo, V=V, T=np.zeros((H, W), np.uint8), S=np.where(V, SL, np.inf).astype(np.float32)); info[int(kf[ci])] = dict(pairs=0, valid_frac=float(V.mean()))
            continue
        Ls = pairs_of(ci); m = len(Ls)
        Zf, Sf, Tw, n_nb = fuse_stack(ci, Ls, "Z", "sig", allp, a.weight)
        if a.near == "refuse":
            Lsh = [q for q in Ls if shortp(q)]
            if Lsh:
                Zn_, Sn_, _, _ = fuse_stack(ci, Lsh, "raw", "sigL", shortp, a.weight)
            else:
                Zn_ = np.full((H, W), np.nan, np.float32); Sn_ = np.full((H, W), np.inf, np.float32)
            Zn_ = np.where(Zn_ <= 2.0 * KS[ci], Zn_, np.nan)
        else:
            Zn_ = ZL; Sn_ = None
        nearL = np.isfinite(Zn_) & (Zn_ <= a.z_split)
        far = np.isfinite(Zf) & (Zf > a.z_split) & (Zf <= a.zmax) & ~nearL
        Zo = np.where(nearL, Zn_, np.where(far, Zf, np.nan)).astype(np.float32); V = np.isfinite(Zo)
        Tg = np.where(far, 2 | np.where(Tw, 4, 0), 0).astype(np.uint8)
        if Sn_ is None:
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
        info[int(kf[ci])] = dict(pairs=m, kscale=round(float(KS[ci]), 4), valid_frac=round(float(V.mean()), 4), px_far=int(far.sum()), px_single_nb_ok=n_nb)
        if ci % 100 == 0:
            print(ci, info[int(kf[ci])], f"{time.time() - t0:.0f}s", flush=True)
    scs = np.array(SC) if SC else np.ones(1)
    rs = dict(med=float(np.median(scs)), p10=float(np.percentile(scs, 10)), p90=float(np.percentile(scs, 90)), frac_ne1=float(np.mean(scs != 1)))
    ks = dict(med=float(np.median(KS)), p5=float(np.percentile(KS, 5)), p95=float(np.percentile(KS, 95)), frac_ne1=float(np.mean(KS != 1)))
    (od / "fuse_info.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), script="gsm_ffs_fix3.py fuse", args=vars(a), bvio_rescale=rs, kscale=ks, per_kf=info), indent=1))
    print(f"fuse {a.name}: {len(info)} kf, bvio rescale {rs}, kscale {ks}, {time.time() - t0:.0f}s", flush=True)


# ---------------------------------------------------------------- half-res judge / kfit (audit5 an5a/an5d rule)
STEP = 6


class Cams:
    def __init__(self, run):
        c = np.load(Path(run) / "dump_ba" / "cams.npz"); self.kf = c["kf"]; self.T = c["T"].astype(np.float64); Kr = c["Kr"]; self.n = len(self.kf)
        self.kfn = np.array([int(i) for i in self.kf]); self.H, self.W = 270, 480
        self.fx, self.fy, self.cx, self.cy = Kr[0, 0] / 2, Kr[1, 1] / 2, Kr[0, 2] / 2, Kr[1, 2] / 2; self.Tw = np.linalg.inv(self.T)
        self.gv, self.gu = np.mgrid[0:self.H:STEP, 0:self.W:STEP]
        W0, H0 = [int(v) for v in c["size"]]
        mx, my = cv2.initUndistortRectifyMap(c["K"], c["dist"], None, Kr, (W0, H0), cv2.CV_32FC1)
        self.vm2 = cv2.erode(((mx >= 0) & (mx <= W0 - 1) & (my >= 0) & (my <= H0 - 1)).astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)[::2, ::2]

    def load(self, d):
        """-> half-res Z (n,270,480), full-res valid count per kf"""
        Z = np.full((self.n, self.H, self.W), np.nan, np.float32); vc = np.zeros(self.n, np.int64); tot = None
        for ci, i in enumerate(self.kf):
            f = Path(d) / f"{int(i):05d}.npz"
            if f.exists():
                z = np.load(f)["Z"]; vc[ci] = int(np.isfinite(z).sum()); tot = z.size; Z[ci] = z[::2, ::2]
        return Z, vc, tot

    def proj(self, Z, ci, j, z, u, v):
        T, Tw, fx, fy, cx, cy, H, W = self.T, self.Tw, self.fx, self.fy, self.cx, self.cy, self.H, self.W
        X = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], 1) @ T[ci, :3, :3].T + T[ci, :3, 3]
        Xj = np.einsum("vij,mj->vmi", Tw[j, :3, :3], X) + Tw[j, None, :3, 3]; zj = Xj[..., 2]; zc = np.maximum(zj, 1e-3)
        pu = np.round(fx * Xj[..., 0] / zc + cx).astype(np.int64); pv = np.round(fy * Xj[..., 1] / zc + cy).astype(np.int64)
        ok = (zj > 0.05) & (pu >= 0) & (pu < W) & (pv >= 0) & (pv < H); zr = Z[j[:, None], pv.clip(0, H - 1), pu.clip(0, W - 1)]
        return zj, zr, ok & np.isfinite(zr)

    def judge(self, Z, views="all"):
        """-> list per kf of fail flags (None if no depth) on the step-6 grid"""
        js = np.nonzero(np.isfinite(Z).reshape(self.n, -1).any(1))[0]; out = [None] * self.n
        for ci in js:
            Zs = Z[ci, ::STEP, ::STEP]; m = np.isfinite(Zs); z = Zs[m].astype(np.float64); viol = np.zeros(len(z), np.int32); chk = np.zeros(len(z), np.int32)
            jj = js[js != ci]
            if views == "even":
                jj = jj[(jj - ci) % 2 == 0]
            for k0 in range(0, len(jj), 64):
                zj, zr, fin = self.proj(Z, ci, jj[k0:k0 + 64], z, self.gu[m], self.gv[m])
                with np.errstate(invalid="ignore"):
                    chk += fin.sum(0); viol += (fin & (zj < zr - np.maximum(0.03 * zr, 0.03))).sum(0)
            out[ci] = (viol >= 2) & (viol >= 0.3 * chk)
        return out

    def pairs(self, Z, parity="all", dt=300):
        n, kfn = self.n, self.kfn
        js = np.nonzero(np.isfinite(Z).reshape(n, -1).any(1))[0]; I, J, Dv, Wt = [], [], [], []
        for ci in js:
            Zs = Z[ci, ::STEP, ::STEP]; m = np.isfinite(Zs); z = Zs[m].astype(np.float64)
            jj = js[(js != ci) & (np.abs(kfn[js] - kfn[ci]) <= dt)]
            if parity == "odd":
                jj = jj[(jj - ci) % 2 == 1]
            if not len(jj) or len(z) < 30:
                continue
            zj, zr, fin = self.proj(Z, ci, jj, z, self.gu[m], self.gv[m])
            with np.errstate(invalid="ignore", divide="ignore"):
                r = np.where(fin, np.log(zj / zr), np.nan); r[np.abs(r) > 0.25] = np.nan
            cnt = np.isfinite(r).sum(1)
            for q, j in enumerate(jj):
                if cnt[q] >= 30:
                    I.append(ci); J.append(j); Dv.append(np.nanmedian(r[q])); Wt.append(np.sqrt(cnt[q]))
        return np.array(I, int), np.array(J, int), np.array(Dv), np.array(Wt)


def solve_irls(I, J, Dv, Wt, n, prior=0.3, huber=0.02, iters=5):
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import lsqr
    m = len(I); w = Wt.copy(); x = np.zeros(n)
    for _ in range(iters):
        rows = np.concatenate([np.arange(m), np.arange(m), m + np.arange(n)]); cols = np.concatenate([I, J, np.arange(n)])
        A = coo_matrix((np.concatenate([w, -w, np.full(n, prior)]), (rows, cols)), shape=(m + n, n)).tocsr(); b = np.concatenate([-Dv * w, np.zeros(n)])
        x = lsqr(A, b, atol=1e-10, btol=1e-10, iter_lim=5000)[0]
        r = np.abs(x[I] - x[J] + Dv); w = Wt * np.sqrt(np.where(r <= huber, 1.0, huber / np.maximum(r, 1e-12)))
    return x


def cmd_kfit(a):
    t0 = time.time(); C = Cams(a.run); Z, _, _ = C.load(a.set)
    I, J, Dv, Wt = C.pairs(Z, a.parity)
    x = solve_irls(I, J, Dv, Wt, C.n); s_raw = np.exp(x); s = np.clip(s_raw, a.kmin, a.kmax)
    npair = np.bincount(np.concatenate([I, J]), minlength=C.n)
    Path(a.kout).parent.mkdir(parents=True, exist_ok=True)
    np.savez(a.kout, kf=C.kf, s=s, s_raw=s_raw, n_pairs=npair, made=time.strftime("%F %T"), src=str(a.set), parity=a.parity)
    hs = np.isfinite(Z).reshape(C.n, -1).any(1)
    print(f"kfit {a.set} parity {a.parity}: pairs {len(I)} |Dv| med {np.median(np.abs(Dv)):.4f}; s p5/p50/p95 {np.round(np.percentile(s[hs], [5, 50, 95]), 3)}; "
          f"clipped {int(((s_raw < a.kmin) | (s_raw > a.kmax))[hs].sum())}; |s-1|>3%: {int((np.abs(s - 1) > 0.03)[hs].sum())} kf -> {a.kout} {time.time() - t0:.0f}s", flush=True)


def cmd_judge(a):
    t0 = time.time(); C = Cams(a.run); gold = np.load(a.gold)["Z"].astype(np.float32)
    Zr, _, _ = C.load(a.ref_set); href = np.isfinite(Zr).reshape(C.n, -1).any(1); del Zr
    jp = Path(a.json); res = json.loads(jp.read_text()) if jp.exists() else dict(sets={})
    res.update(made=time.strftime("%F %T"), judge="audit5 half-res step-6 mvcheck (gsm_filtmesh rule), fail on ref-set keyframes", ref_set=str(a.ref_set), gold=str(a.gold))
    bins = [(0, 1), (1, 2), (2, 3), (3, 4)]
    for item in a.sets.split(","):
        nm, d = item.split("=", 1); Z, vc, tot = C.load(d); hs = np.isfinite(Z).reshape(C.n, -1).any(1)
        e = dict(dir=d, kf_with_ffs=int(hs.sum()), valid_med=round(float(np.median(vc[hs] / tot)), 4))
        for vw in a.views.split(","):
            fl = C.judge(Z, vw); sel = [ci for ci in range(C.n) if href[ci] and fl[ci] is not None]
            f_ = sum(int(fl[ci].sum()) for ci in sel); t_ = sum(len(fl[ci]) for ci in sel)
            fo = sum(int(fl[ci].sum()) for ci in range(C.n) if fl[ci] is not None); to = sum(len(fl[ci]) for ci in range(C.n) if fl[ci] is not None)
            e[f"fail_{vw}"] = round(100 * f_ / max(1, t_), 2); e[f"fail_{vw}_ownkf"] = round(100 * fo / max(1, to), 2)
            e[f"good_px_M_{vw}"] = round(float(vc.sum()) * (1 - fo / max(1, to)) / 1e6, 2)
        # gold, 2x grid inside the undistortion mask (measure_4m rule)
        m = np.isfinite(Z) & C.vm2[None] & (gold > 0); zz = Z[m]; gz = gold[m]; ok = np.abs(zz - gz) < 0.02 + 0.01 * gz
        e["gold"] = dict(n_2x=int(m.sum()), within_2cm_1pct=round(float(ok.mean()), 4), med_abs_rel=round(float(np.median(np.abs(zz - gz) / gz)), 4), bins={})
        for lo, hi in bins:
            sb = (zz >= lo) & ((zz < hi) if hi < 4 else (zz <= hi))
            e["gold"]["bins"][f"{lo}-{hi}m"] = dict(n=int(sb.sum()), within=round(float(ok[sb].mean()), 4) if sb.any() else None)
        res["sets"][nm] = e; print(nm, json.dumps(e), f"{time.time() - t0:.0f}s", flush=True)
        jp.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["fuse", "kfit", "judge"]); ap.add_argument("run")
    ap.add_argument("--vio", default=""); ap.add_argument("--rec", default=""); ap.add_argument("--gt", default="lensonly")
    ap.add_argument("--p2", default=""); ap.add_argument("--p3", default=""); ap.add_argument("--out", default=""); ap.add_argument("--src", default="")
    ap.add_argument("--z-split", type=float, default=1.5); ap.add_argument("--zmax", type=float, default=4.0); ap.add_argument("--cap", type=float, default=0.03)
    ap.add_argument("--k", type=float, default=2.0); ap.add_argument("--weight", default="ivw", choices=["ivw", "plain"])
    ap.add_argument("--near", default="refuse", choices=["L", "refuse"]); ap.add_argument("--name", default="fix3")
    ap.add_argument("--rs-exp", type=float, default=0.0); ap.add_argument("--kscale", default="")
    ap.add_argument("--dark-thr", type=float, default=0.0); ap.add_argument("--dark-mode", default="drop", choices=["drop", "w"]); ap.add_argument("--dark-w", type=float, default=0.25)
    ap.add_argument("--set", default=""); ap.add_argument("--kout", default=""); ap.add_argument("--parity", default="all", choices=["all", "odd"])
    ap.add_argument("--kmin", type=float, default=0.9); ap.add_argument("--kmax", type=float, default=1.1)
    ap.add_argument("--ref-set", default=""); ap.add_argument("--gold", default=""); ap.add_argument("--sets", default=""); ap.add_argument("--views", default="all,even"); ap.add_argument("--json", default="")
    a = ap.parse_args()
    {"fuse": cmd_fuse, "kfit": cmd_kfit, "judge": cmd_judge}[a.cmd](a)
