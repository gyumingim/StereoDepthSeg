"""gpuba 2026-11-09: kf_ba.ba_lm (Levenberg-Marquardt, Schur complement on the cameras, Huber) in torch float64 on the GPU.

Same steps, same order, same constants as kf_ba.ba_lm (numpy): left-perturbation Jacobians, damping of the point and camera
diagonals, reduced camera system built from all observation pairs of each point, dense solve, accept if cost drops and all
depths > 1e-3, lam /3 on accept, x4 on reject, stop at lam > 1e8 or relative cost drop < 1e-7.
Differences = floating-point summation order only (GPU reductions, LU on the GPU, Rodrigues formula in torch instead of
cv2.Rodrigues). With deterministic=True every scatter-add uses torch's deterministic (sorted) path -> repeatable run to run.
"""
import time

import numpy as np


def ba_lm_torch(R, t, X, cam, pt, uv, K, fixed=(0,), iters=30, huber=1.0, log=print, rs=None, tmr=None,
                device="cuda", chunk=3_000_000, deterministic=True):
    import torch
    torch.use_deterministic_algorithms(deterministic, warn_only=False)
    dt = torch.float64
    dev = torch.device(device)
    tmr = {} if tmr is None else tmr
    _tk = [time.time()]

    def tick(k):
        torch.cuda.synchronize()
        n_ = time.time(); tmr[k] = round(tmr.get(k, 0.0) + n_ - _tk[0], 3); _tk[0] = n_

    N, P = len(R), len(X)
    fixed = set(fixed)
    free = np.array([i not in fixed for i in range(N)])
    cidx = -np.ones(N, int); cidx[free] = np.arange(free.sum())
    Nf = int(free.sum())
    order = np.argsort(pt, kind="stable")
    cam, pt, uv = cam[order], pt[order], uv[order]
    # observation pairs within the same point (a-major, b-minor per point, as in ba_lm), free cameras only
    starts = np.r_[0, np.flatnonzero(np.diff(pt)) + 1, len(pt)]
    L = np.diff(starts)
    pa = np.repeat(np.arange(len(pt)), L[np.searchsorted(starts[1:], np.arange(len(pt)), side="right")])
    first = np.repeat(starts[:-1], L * L)
    within = np.arange(len(pa)) - np.repeat(np.cumsum(L * L) - L * L, L * L)
    pb = first + within % np.repeat(L, L * L)
    tmr["n_obs_pairs"] = int(len(pa))
    fo = free[cam]
    m = fo[pa] & fo[pb]
    Am, Bm = pa[m], pb[m]
    del pa, pb, first, within, m
    # S block index of each pair (camera-pair block, 36 entries), as int64 on the GPU
    key = (cidx[cam[Am]] * Nf + cidx[cam[Bm]]).astype(np.int64)
    so = np.argsort(key, kind="stable")              # pairs sorted by S block once (structure fixed) -> segment sums, no atomics
    Am, Bm, key = Am[so], Bm[so], key[so]
    del so
    T = lambda a, d=dt: torch.as_tensor(np.ascontiguousarray(a), dtype=d, device=dev)
    Am_t, Bm_t = T(Am, torch.int64), T(Bm, torch.int64)
    segs = []                                        # per chunk: unique blocks + run lengths
    for c0_ in range(0, len(key), chunk):
        u_, cn_ = np.unique(key[c0_:c0_ + chunk], return_counts=True)
        segs.append((c0_, T(u_, torch.int64), T(cn_, torch.int64)))
    del Am, Bm, key
    cam_t, pt_t, uv_t = T(cam, torch.int64), T(pt, torch.int64), T(uv)
    free_t = T(free, torch.bool)
    freeidx_t = T(np.flatnonzero(free), torch.int64)
    Rt, tt, Xt = T(R), T(t), T(X)
    if rs is None:
        Rr = tr_ = None
    else:
        Rr, tr_ = T(rs[0][order]), T(rs[1][order])
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    I3 = torch.eye(3, dtype=dt, device=dev)
    I6 = torch.eye(6, dtype=dt, device=dev)

    def sadd(n, idx, v):            # deterministic scatter-sum of rows v into n rows
        o = torch.zeros((n,) + tuple(v.shape[1:]), dtype=dt, device=dev)
        return o.index_add_(0, idx, v)

    def cost(R, t, X):
        Xm = torch.einsum("nij,nj->ni", R[cam_t], X[pt_t]) + t[cam_t]
        Xc = Xm if Rr is None else torch.einsum("nij,nj->ni", Rr, Xm) + tr_
        q = torch.stack([fx * Xc[:, 0] / Xc[:, 2] + cx, fy * Xc[:, 1] / Xc[:, 2] + cy], 1)
        r = q - uv_t
        e = torch.linalg.norm(r, dim=1)
        w = torch.where(e <= huber, torch.ones_like(e), huber / torch.clamp(e, min=1e-12))
        rho = torch.where(e <= huber, e ** 2, 2 * huber * e - huber ** 2)
        return r, e, w, rho.sum(), (Xm, Xc)

    def skew(v):
        z = torch.zeros_like(v[:, 0])
        return torch.stack([torch.stack([z, -v[:, 2], v[:, 1]], -1), torch.stack([v[:, 2], z, -v[:, 0]], -1),
                            torch.stack([-v[:, 1], v[:, 0], z], -1)], -2)

    def expso3(w):                  # Rodrigues, (N,3) -> (N,3,3)
        th = torch.linalg.norm(w, dim=1)
        small = th < 1e-12
        ths = torch.where(small, torch.ones_like(th), th)
        k = w / ths[:, None]
        Kx = skew(k)
        s, c = torch.sin(th)[:, None, None], torch.cos(th)[:, None, None]
        Rm = I3 + s * Kx + (1 - c) * (Kx @ Kx)
        return torch.where(small[:, None, None], I3.expand_as(Rm), Rm)

    def med(x):                     # numpy median semantics (mean of the two middle values for even n)
        return float(np.median(x.cpu().numpy()))

    r, e, w, c0, (Xm, Xc) = cost(Rt, tt, Xt)
    c0 = float(c0)
    tick("setup_s")
    lam = 1e-3
    hist = [dict(it=0, cost=c0, rmse_px=float(torch.sqrt((e ** 2).mean())), med_px=med(e))]
    rel = 0.0
    for it in range(1, iters + 1):
        Z = Xc[:, 2]
        Jq = torch.zeros((len(Z), 2, 3), dtype=dt, device=dev)
        Jq[:, 0, 0] = fx / Z; Jq[:, 0, 2] = -fx * Xc[:, 0] / Z ** 2
        Jq[:, 1, 1] = fy / Z; Jq[:, 1, 2] = -fy * Xc[:, 1] / Z ** 2
        JqR = Jq if Rr is None else Jq @ Rr
        Jc = torch.cat([-JqR @ skew(Xm), JqR], 2)
        Jp = JqR @ Rt[cam_t]
        sw = w[:, None, None]
        Hcc_o = torch.einsum("nki,nkj->nij", Jc, sw * Jc)
        Hcp_o = torch.einsum("nki,nkj->nij", Jc, sw * Jp)
        Hpp_o = torch.einsum("nki,nkj->nij", Jp, sw * Jp)
        gc_o = torch.einsum("nki,nk->ni", Jc, w[:, None] * r)
        gp_o = torch.einsum("nki,nk->ni", Jp, w[:, None] * r)
        Hpp = sadd(P, pt_t, Hpp_o)
        gp = sadd(P, pt_t, gp_o)
        Hcc = sadd(N, cam_t, Hcc_o)
        gcam = sadd(N, cam_t, gc_o)
        Hcc_f = Hcc[freeidx_t]
        tick("jac_s")
        while True:
            tmr["n_solves"] = tmr.get("n_solves", 0) + 1
            dg = torch.diagonal(Hpp, dim1=1, dim2=2)
            Hpp_d = Hpp + lam * (dg[:, :, None] * I3 + 1e-9 * I3)
            Hpp_inv = torch.linalg.inv(Hpp_d)
            # camera-camera diagonal blocks (damped)
            D = Hcc_f + lam * torch.diag_embed(torch.diagonal(Hcc_f, dim1=1, dim2=2) + 1e-9)
            V = torch.einsum("nij,njk->nik", Hcp_o, Hpp_inv[pt_t])           # (n,6,3)
            Sb = torch.zeros((Nf * Nf, 36), dtype=dt, device=dev)
            for c0_, u_, cn_ in segs:
                Va, Hb = V[Am_t[c0_:c0_ + chunk]], Hcp_o[Bm_t[c0_:c0_ + chunk]]
                blk = (Va[:, :, 0, None] * Hb[:, None, :, 0] + Va[:, :, 1, None] * Hb[:, None, :, 1]
                       + Va[:, :, 2, None] * Hb[:, None, :, 2]).reshape(-1, 36)          # V_a Hcp_b^T, 6x6 per pair
                Sb[u_] += torch.segment_reduce(blk, "sum", lengths=cn_, axis=0)
                del blk, Va, Hb
            S = (-Sb).reshape(Nf, Nf, 6, 6).permute(0, 2, 1, 3).reshape(Nf * 6, Nf * 6)
            del Sb
            S = S.contiguous()
            Sv = S.view(Nf, 6, Nf, 6)
            ar = torch.arange(Nf, device=dev)
            Sv[ar, :, ar, :] += D
            vg = torch.einsum("nij,nj->ni", V, gp[pt_t])
            bb = sadd(N, cam_t, vg)
            b = (-gcam[freeidx_t] + bb[freeidx_t]).reshape(-1)
            tick("schur_s")
            dc, info = torch.linalg.solve_ex(S, b)
            if int(info) != 0:
                lam *= 10; del S, Sv; tick("solve_s"); continue
            del S, Sv
            dcam = torch.zeros((N, 6), dtype=dt, device=dev); dcam[freeidx_t] = dc.reshape(-1, 6)
            tick("solve_s")
            wt = torch.einsum("nij,ni->nj", Hcp_o, dcam[cam_t])
            rhs = sadd(P, pt_t, wt)
            dp = torch.einsum("pij,pj->pi", Hpp_inv, -gp - rhs)
            Ed = expso3(dcam[:, :3])
            Rn = Ed @ Rt
            tn = torch.einsum("nij,nj->ni", Ed, tt) + dcam[:, 3:]
            Xn = Xt + dp
            rn, en, wn, cn, Xcn = cost(Rn, tn, Xn)
            cn = float(cn)
            ok_depth = bool((Xcn[1][:, 2] > 1e-3).all())
            tick("update_cost_s")
            if cn < c0 and ok_depth:
                Rt, tt, Xt, r, e, w, (Xm, Xc) = Rn, tn, Xn, rn, en, wn, Xcn
                rel = (c0 - cn) / c0
                c0 = cn
                lam = max(lam / 3, 1e-7)
                break
            lam *= 4
            if lam > 1e8:
                rel = 0
                break
        md, rm = med(e), float(torch.sqrt((e ** 2).mean()))
        hist.append(dict(it=it, cost=c0, lam=lam, rmse_px=rm, med_px=md,
                         step_cam_max=float(dcam.abs().max()) if lam <= 1e8 else 0.0))
        log(f"[ba] it {it} cost {c0:.1f} med {md:.3f} px rmse {rm:.3f} lam {lam:.1e}")
        if lam > 1e8 or rel < 1e-7:
            break
    e_np = e.cpu().numpy()
    out = (Rt.cpu().numpy(), tt.cpu().numpy(), Xt.cpu().numpy(),
           dict(hist=hist, final_med_px=float(np.median(e_np)), final_rmse_px=float(np.sqrt((e_np ** 2).mean())),
                frac_inlier_1px=float((e_np <= 1).mean())))
    tmr["gpu_peak_gb"] = round(torch.cuda.max_memory_allocated() / 2 ** 30, 2)
    torch.use_deterministic_algorithms(False)
    return out
