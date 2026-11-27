#!/usr/bin/env python3
"""Per-pair offset diagnostic (2026-11-27), no retraining, CPU. Same ruler as diag_plane_20261127/plane_diag.py and
diag_plane_fix_20261127/plane_fix.py (unchanged copies in this folder).

  python pair_diag.py table   -> OUT/pairs_table.json : every stage-a pair map on the 3 ROIs (cabinet_front_kf774, wall_kf538, floor_kf3205)
                                 with pair class, geometry under the poses the FFS used, rectification / FFS input, ROI image region,
                                 measured offset (signed, + = towards the camera) and tilt vs the ROI consensus plane, disparity offset
                                 (px) and depth ratio; plus the "before" (actual fused) and "b2_before" (simple re-fusion) summaries,
                                 checked against plane_diag.json (3 decimals).
  python pair_diag.py rules   -> OUT/rules.json : simple re-fusion (plane_fix.refuse rule) without / with down-weighted pair classes,
                                 a/b/c per ROI + FFS coverage kept (F == 1 pixels of all keyframes) vs the all-pairs re-fusion.
Pair classes: base (pairs2 strict, perp >= 40), base_fb (pairs2 fallback, perp < 40), low (pairs3 low tier, partial rectification),
wide_ba / wide_vio (gsm_ffs_4m wide tier re-run on ba_partners poses; partner pose = BA or VIO-anchored, ffsonly_ba/wide_poses.npz),
relax (gsm_ffs_relax, BA2 poses, partial rectification, perp >= 10).
Poses the FFS used: base/low = ba_partners/poses_all.npz (kf + partner); wide = ba_partners/run/dump_ba/cams.npz (kf) + wide_poses (partner);
relax = ba_partners2/poses_all.npz. Measurement (placing the map in the world) = ba_partners2 cams (as plane_diag).
"""
import json, sys, time, os
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import plane_diag as PD   # unchanged copy
import plane_fix as PF    # unchanged copy

V2 = PD.V2; OUT = V2 / "diag_pairs_20261127"
BAP = V2 / "ba_partners"; BA2 = V2 / "ba_partners2"; WIDE_OLD = V2 / "ffsonly_ba" / "wide"; RELAX = V2 / "ffs_relax" / "relax"


def rotvec(R):
    from scipy.spatial.transform import Rotation as Ro
    return Ro.from_matrix(R).as_rotvec()


def setup():
    import gsm_ba_partners as BP
    kf, T, Kr, (W, H) = PD.cams(); rc = PD.rays(Kr, W, H); RS_ = PD.roi_setup()
    g = np.load(Path(BP.R1) / "rec" / "lensonly" / "gt.npz"); Kfull = g["K"].copy(); Kfull[:2] *= W / 1920.0
    base_c = json.loads((PD.SRC / "plan_4m.json").read_text())["base_c"]
    fps_all = {}
    for name, R in RS_.items():
        fp = {ci: PD.footprint(R, T[ci], rc, W, H) for ci in range(len(kf))}
        fps_all[name] = {ci: f for ci, f in fp.items() if f.sum() >= PD.MIN_FP}
    return kf, T, Kr, W, H, rc, RS_, Kfull, base_c, fps_all


def load_poses():
    a = np.load(BAP / "poses_all.npz"); PBAP = {int(f): t for f, t in zip(a["frames"], a["T"])}
    b = np.load(BA2 / "poses_all.npz"); PBA2 = {int(f): t for f, t in zip(b["frames"], b["T"])}
    w = np.load(V2 / "ffsonly_ba" / "wide_poses.npz"); PW = {int(f): (t, str(s)) for f, t, s in zip(w["frames"], w["T"], w["src"])}
    c = np.load(BAP / "run" / "dump_ba" / "cams.npz"); KB = {int(f): t for f, t in zip(c["kf"], c["T"])}
    info = {}
    for nm, fn in (("low", BAP / "ffs_pairs3" / "extra" / "ffs_info.json"), ("relax", RELAX / "ffs_info.json")):
        info[nm] = json.loads(fn.read_text())["per_kf"]
    return PBAP, PBA2, PW, KB, info


def epi_px(K, p, W, H):
    """projection of direction p; returns (inside, distance of the projected point outside the image border in px (0 if inside),
    distance from the image centre in px). behind-camera direction is projected as well (both signs, as P2.epi_inside)."""
    if abs(p[2]) < 1e-9:
        return False, float("inf"), float("inf")
    u = K[0, 0] * p[0] / p[2] + K[0, 2]; v = K[1, 1] * p[1] / p[2] + K[1, 2]
    ins = bool(0 <= u < W and 0 <= v < H)
    dout = float(np.hypot(max(0, -u, u - W), max(0, -v, v - H)))
    return ins, dout, float(np.hypot(u - W / 2, v - H / 2))


def pair_meta(kf, ci, W, H, Kr, poses):
    """meta for every pair in PF.pairs order (base, low, wide/relax)."""
    import motion_stereo as ms
    PBAP, PBA2, PW, KB, info = poses
    fi = int(kf[ci]); M = []
    fb = PD.P2D / "pairs" / f"{fi:05d}.npz"
    if fb.exists():
        d = np.load(fb)
        for p in range(len(d["B"])):
            M.append(dict(tag="base", j=int(d["partner"][p]), perp_stored=float(d["perp"][p]), B=float(d["B"][p]), pose="bap", rect="full"))
    for tag, fe in (("low", PD.P3D / "extra" / f"{fi:05d}.npz"), ("wide", PD.SRC / "wide" / f"{fi:05d}.npz")):
        if fe.exists():
            d = np.load(fe); isrelax = "ffs_relax/relax" in str(fe.resolve())
            for p in range(len(d["B"])):
                if tag == "low" and str(d["tag"][p]) != "low":
                    continue
                m = dict(tag=tag, j=int(d["partner"][p]), perp_stored=float(d["perp"][p]), B=float(d["B"][p]))
                if tag == "low":
                    m.update(pose="bap", rect="partial")
                elif isrelax:
                    m.update(tag="relax", pose="ba2", rect="partial")
                else:
                    m.update(pose="wide", rect="full", scale=float(d["scale"][p]))
                M.append(m)
    for m in M:
        j = m["j"]
        if m["pose"] == "bap":
            Tk, Tj, src = PBAP[fi], PBAP[j], "ba"
        elif m["pose"] == "ba2":
            Tk, Tj, src = PBA2[fi], PBA2[j], "ba"
        else:
            Tk = KB[fi]; Tj, src = PW[j]
        R, t = ms.rel_pose(Tk, Tj); g = ms.pair_geometry(R, t)
        cls = m["tag"]
        if cls == "base":
            cls = "base" if m["perp_stored"] >= 40 else "base_fb"
        if cls == "wide":
            cls = "wide_vio" if src != "ba" else "wide_ba"
        ik, dk, ck = epi_px(Kr, g["c"], W, H); ij, dj, cj = epi_px(Kr, t, W, H)
        m.update(cls=cls, partner_pose_src=src, B_geom_cm=g["B"] * 100, perp_deg=g["perp_deg"], rot_deg=g["rot_deg"], roll_deg=g["roll_deg"],
                 dt_frames=int(j - fi), epi_in_kf=ik, epi_out_kf_px=dk, epi_ctr_kf_px=ck, epi_in_partner=ij, epi_out_partner_px=dj, R=R, t=t)
        # FFS input resolution
        if m["rect"] == "full":
            f = float(Kr[0, 0]); Wr, Hr = (H, W) if abs(g["roll_deg"]) > 45 else (W, H)
            if "scale" not in m:
                d_need = f * m["B"] / 0.5     # pairs2: pair_depth without z_near -> cfg.z_near_default 0.5, ffs_max_disp 192
                m["scale"] = round(float(np.clip(0.92 * 192 / max(d_need, 1.0), 0.35, 1.0)), 3)
            m.update(rect_f=f, rect_W=Wr, rect_H=Hr, ffs_W=int(round(Wr * m["scale"])), ffs_H=int(round(Hr * m["scale"])), sectors=None)
        else:
            ent = [e for e in info["low" if m["tag"] == "low" else "relax"].get(str(fi), []) if int(e["j"]) == j]
            m["sectors"] = ent[0]["sec"] if ent else None
        # BA2 consistency of the relative pose the FFS used
        if fi in PBA2 and j in PBA2:
            R2, t2 = ms.rel_pose(PBA2[fi], PBA2[j]); c2 = -R2.T @ t2
            dphi = rotvec(R2 @ R.T)        # p_j = R p_i + t ; rotation change in the partner frame (small-angle, ~ kf frame)
            e = g["c"][:2] / max(np.linalg.norm(g["c"][:2]), 1e-9)
            m.update(ba2_rot_change_deg=float(np.degrees(np.linalg.norm(dphi))), ba2_pred_dpx=float(Kr[0, 0] * (dphi[1] * e[0] - dphi[0] * e[1])),
                     ba2_B_ratio=float(np.linalg.norm(c2) / max(g["B"], 1e-9)),
                     ba2_dir_change_deg=float(np.degrees(np.arccos(np.clip(c2 @ g["c"] / np.linalg.norm(c2) / max(g["B"], 1e-9), -1, 1)))))
    return M


def huber_lin(x, y, c, it=6):
    A = np.c_[np.ones(len(x)), x]; w = np.ones(len(y))
    for _ in range(it):
        sw = np.sqrt(w); p = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)[0]
        e = np.abs(y - A @ p); w = np.where(e <= c, 1, c / np.maximum(e, 1e-12))
    return p


def map_extras(X, iv, iu, Zmap, q, m, R, ci, T, rc_sel, Kr, Kfull, W, H):
    n, d = R["n"], R["d"]; o = T[ci][:3, 3]; D = rc_sel @ T[ci][:3, :3].T
    Zref = -(n @ o + d) / (D @ n); Z = Zmap[iv, iu]; sn = q["sn"][iv, iu]; fBs = q["f"] * q["B"] * sn
    sgn = float(np.sign(n @ o + d))
    dinv = 1 / Z - 1 / Zref; dpx = fBs * dinv; rho = Z / Zref - 1
    sub = slice(None) if len(Z) < 30000 else np.random.default_rng(0).choice(len(Z), 30000, replace=False)
    al, be = huber_lin(1 / Zref[sub], dinv[sub], c=0.003)
    u_c, v_c = float(np.mean(iu)), float(np.mean(iv))
    rad = float(np.median(np.hypot((iu - Kr[0, 2]) / (W / 2), (iv - Kr[1, 2]) / (H / 2))))
    e = dict(cam_sign=sgn, Z_med_m=float(np.median(Z)), Zref_med_m=float(np.median(Zref)), Zref_p5_p95=[float(x) for x in np.percentile(Zref, [5, 95])],
             dpx_med=float(np.median(dpx)), rho_med=float(np.median(rho)), disp_px_med=float(np.median(fBs / Z)), sin_med=float(np.median(sn)),
             sin_p5=float(np.percentile(sn, 5)), theta_med_deg=float(np.degrees(np.arcsin(np.clip(np.median(sn), 0, 1)))),
             fit_alpha_px=float(al * np.median(fBs)), fit_beta=float(be), u_c=u_c, v_c=v_c, radius_med=rad, n_px=int(len(Z)),
             sigma_px_model_cm=float(np.median(Z ** 2 / fBs) * 100))
    if m["rect"] == "partial":
        # sector geometry exactly as gsm_ffs_pairs3.partial_depth (rays from the lens K, no distortion)
        Rr, t = m["R"], m["t"]; c = -Rr.T @ t; e1 = c / np.linalg.norm(c)
        r = np.stack([(iu - Kfull[0, 2]) / Kfull[0, 0], (iv - Kfull[1, 2]) / Kfull[1, 1], np.ones(len(iu))], -1); r /= np.linalg.norm(r, axis=1, keepdims=True)
        zm = np.array([0, 0, 1.0]) + Rr.T @ np.array([0, 0, 1.0]); e3b = zm - (zm @ e1) * e1; e3b /= np.linalg.norm(e3b); e2b = np.cross(e3b, e1)
        psi = np.degrees(np.arctan2(r @ e2b, r @ e3b)); sec = np.round(psi / 90.0).astype(int) % 4
        bd = 45 - np.abs(psi - 90 * np.round(psi / 90))
        cnt = np.bincount(sec, minlength=4); smaj = int(np.argmax(cnt))
        e.update(sector_major=smaj, sector_major_frac=float(cnt[smaj] / cnt.sum()), n_sectors_roi=int((cnt > 0.02 * cnt.sum()).sum()),
                 near_sector_boundary_frac=float(np.mean(bd < 10)), boundary_dist_med_deg=float(np.median(bd)))
        si = [s for s in (m["sectors"] or []) if s.get("sec") == smaj and "f" in s]
        if si:
            s = si[0]; e.update(rect_f=s["f"], rect_W=s["W"], rect_H=s["H"], scale=s["scale"], ffs_W=int(round(s["W"] * s["scale"])), ffs_H=int(round(s["H"] * s["scale"])))
    return e


def measure_ext(name_cond, RS_, fps_all, maps_of, kf, T, rc, W, H, Kr, Kfull, extras=True):
    """= plane_fix.measure (same plane refit, gate, map_stats, summaries) + per-map extras and per-map class labels."""
    res = {}
    for name, R0 in RS_.items():
        R = dict(R0); fps = fps_all[name]; rng = np.random.default_rng(0)

        def world(ci, Z, fp):
            mm = fp & np.isfinite(Z) & (Z > 0.05)
            iv, iu = np.nonzero(mm); sel = rc[mm.reshape(-1)]
            return sel * Z[mm][:, None] @ T[ci][:3, :3].T + T[ci][:3, 3], iv, iu, sel
        for it in range(2):
            Pb = []
            for ci, fp in fps.items():
                X = world(ci, maps_of(ci)["b"], fp)[0]; X = X[np.abs(X @ R["n"] + R["d"]) <= PD.GATE]
                if len(X) >= PD.MIN_PTS:
                    Pb.append(X[rng.choice(len(X), min(len(X), 5000), replace=False)])
            if not Pb:          # no fused map left on this ROI (rule removed every pair): keep the setup plane, report n_maps 0
                break
            n, d = PD.fit_plane(np.concatenate(Pb))
            if n @ R["n"] < 0:
                n, d = -n, -d
            R["n"], R["d"] = n, d; R["e1"], R["e2"] = PD.basis(n)
        stages = {"a": [], "b": []}; pooled = {"a": [], "b": []}
        for ci, fp in fps.items():
            mo = maps_of(ci)
            L = [("a", q["Z"], q) for q in mo["a"]] + [("b", mo["b"], None)]
            for st, Z, q in L:
                X, iv, iu, sel = world(ci, Z, fp); keep = np.abs(X @ R["n"] + R["d"]) <= PD.GATE
                X, iv, iu, sel = X[keep], iv[keep], iu[keep], sel[keep]
                s = PD.map_stats(X, R)
                if s is None:
                    continue
                s.pop("r_own"); s.update(idx=int(ci), kf=int(kf[ci]))
                if q is not None:
                    m = q["meta"]
                    s.update(partner=q["partner"], tag=q["tag"], cls=m["cls"])
                    if extras:
                        s.update({k: v for k, v in m.items() if k not in ("R", "t", "sectors")})
                        s.update(map_extras(X, iv, iu, Z, q, m, R, ci, T, sel, Kr, Kfull, W, H))
                else:
                    if extras:
                        s.update(cam_sign=float(np.sign(R["n"] @ T[ci][:3, 3] + R["d"])))
                stages[st].append(s)
                k = min(len(X), 20000); sl = rng.choice(len(X), k, replace=False)
                pooled[st].append(X[sl] @ R["n"] + R["d"])
        rr = {}
        for st in "ab":
            Ls = stages[st]
            if not Ls:
                rr[st] = dict(summary=dict(n_maps=0, within_rms_own_cm_median=None, offset_sd_cm=None, offset_robust_sd_cm=None, tilt_median_deg=None,
                                           combined_rms_global_cm=None, combined_mad_global_cm=None), maps=[])
                continue
            off = np.array([s["offset_cm"] for s in Ls]); rg = np.concatenate(pooled[st])
            sm = dict(n_maps=len(Ls), within_rms_own_cm_median=float(np.median([s["rms_own_cm"] for s in Ls])),
                      offset_sd_cm=float(off.std()), offset_robust_sd_cm=float(1.4826 * np.median(np.abs(off - np.median(off)))),
                      tilt_median_deg=float(np.median([s["tilt_deg"] for s in Ls])),
                      combined_rms_global_cm=float(np.sqrt(np.mean(rg ** 2)) * 100), combined_mad_global_cm=float(1.4826 * np.median(np.abs(rg - np.median(rg))) * 100))
            if st == "a":
                by = {}
                for s in Ls:
                    by.setdefault(s["idx"], []).append(s["offset_cm"])
                w = [np.std(v) for v in by.values() if len(v) >= 2]
                sm.update(within_kf_partner_offset_sd_cm_mean=float(np.mean(w)) if w else None, across_kf_mean_offset_sd_cm=float(np.std([np.mean(v) for v in by.values()])))
            rr[st] = dict(summary=sm, maps=Ls)
        rr["c"] = dict(summary=dict(note="pooled stage b", combined_rms_global_cm=rr["b"]["summary"]["combined_rms_global_cm"],
                                    combined_mad_global_cm=rr["b"]["summary"]["combined_mad_global_cm"]))
        rr["plane"] = dict(n=R["n"].tolist(), d=float(R["d"]))
        res[name] = rr
        print(name_cond, name, {st: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rr[st]["summary"].items() if k in
                                     ("n_maps", "within_rms_own_cm_median", "offset_sd_cm", "offset_robust_sd_cm", "combined_rms_global_cm")} for st in "ab"}, flush=True)
    return res


def refuse_w(maps, F, wts=None):
    """plane_fix.refuse with an optional per-map weight multiplier on the inverse variance (w = 1 = identical to refuse)."""
    if not maps:
        return np.full(F.shape, np.nan)
    wts = np.ones(len(maps)) if wts is None else np.asarray(wts, float)
    Zs = np.stack([m["Z"] for m in maps]); S = np.stack([m["Z"] ** 2 / (m["f"] * m["B"] * m["sn"]) for m in maps])
    with np.errstate(invalid="ignore", all="ignore"):
        med = np.nanmedian(Zs, 0)
        ok = np.isfinite(Zs) & (np.abs(Zs - med) <= np.maximum(2 * S, 0.01))
        w = np.where(ok, wts[:, None, None] / np.maximum(S, 1e-4) ** 2, 0); z = np.where(ok, Zs, 0)
        out = (w * z).sum(0) / w.sum(0)
    return np.where(F & (w.sum(0) > 0), out, np.nan)


class Maps:
    def __init__(s, kf, Kfull, base_c, W, H, Kr, poses):
        s.kf, s.Kfull, s.base_c, s.W, s.H, s.Kr, s.poses = kf, Kfull, base_c, W, H, Kr, poses; s.cache = {}

    def get(s, ci):
        if ci not in s.cache:
            P = PF.pairs(s.kf, ci, s.Kfull, s.base_c, s.W, s.H); M = pair_meta(s.kf, ci, s.W, s.H, s.Kr, s.poses)
            assert len(P) == len(M) and all(p["partner"] == m["j"] for p, m in zip(P, M)), ci
            for p, m in zip(P, M):
                p["meta"] = m
            d = np.load(PD.FUSED / f"{int(s.kf[ci]):05d}.npz"); F = d["F"] == 1
            if len(s.cache) > 3:
                s.cache.pop(next(iter(s.cache)))
            s.cache[ci] = (P, F, np.where(F, d["Z"], np.nan).astype(np.float64))
        return s.cache[ci]


def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k not in ("R", "t")}
    if isinstance(o, list):
        return [strip(v) for v in o]
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def cmd_table():
    t0 = time.time(); OUT.mkdir(parents=True, exist_ok=True)
    kf, T, Kr, W, H, rc, RS_, Kfull, base_c, fps_all = setup(); poses = load_poses()
    MP = Maps(kf, Kfull, base_c, W, H, Kr, poses)

    def before(ci):
        P, F, Zb = MP.get(ci); return dict(a=P, b=Zb)

    def b2(ci):
        P, F, Zb = MP.get(ci); return dict(a=P, b=PF.refuse(P, F))
    rb = measure_ext("before", RS_, fps_all, before, kf, T, rc, W, H, Kr, Kfull, extras=True)
    r2 = measure_ext("b2_before", RS_, fps_all, b2, kf, T, rc, W, H, Kr, Kfull, extras=False)
    pdj = json.loads((V2 / "diag_plane_20261127" / "plane_diag.json").read_text())
    chk = {}
    for name in RS_:
        for st in "ab":
            ref = pdj["rois"][name]["stages"][st]["summary"]; mine = rb[name][st]["summary"]
            chk[f"{name}/{st}"] = {k: [round(ref[k], 3), round(mine[k], 3), round(ref[k], 3) == round(mine[k], 3)] for k in
                                   ("n_maps", "within_rms_own_cm_median", "offset_sd_cm", "offset_robust_sd_cm", "combined_rms_global_cm")}
    ok = all(v[2] for x in chk.values() for v in x.values())
    print("repro plane_diag 3 decimals:", ok, flush=True)
    # join plane_fix per-pair rows (P1 anchor fit a/b, RS predictor)
    pf = json.loads((V2 / "diag_plane_fix_20261127" / "plane_fix.json").read_text())
    jr = {(r["idx"], r["partner"], r["tag"]): r for r in pf["pairs"]}
    for name in RS_:
        for s in rb[name]["a"]["maps"]:
            r = jr.get((s["idx"], s["partner"], "wide" if s["tag"] == "wide" else s["tag"]))
            if r:
                s.update(p1_a_px=r.get("a"), p1_b_px100=r.get("b"), p1_nfit=r.get("n_fit"), rs_pred_b=r.get("rs_pred_b"), dw_norm=r.get("dw_norm"))
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), note=__doc__,
               inputs=dict(cams=str(PD.DUMP / "cams.npz"), fused=str(PD.FUSED), pairs=[str(PD.P2D / "pairs"), str(PD.P3D / "extra (tag low)"), str(PD.SRC / "wide")],
                           poses_bap=str(BAP / "poses_all.npz"), poses_ba2=str(BA2 / "poses_all.npz"), wide_poses=str(V2 / "ffsonly_ba/wide_poses.npz"),
                           cams_bap=str(BAP / "run/dump_ba/cams.npz"), ffs_info=[str(PD.P3D / "extra/ffs_info.json"), str(RELAX / "ffs_info.json")],
                           rois=str(PD.ROIS), plane_diag=str(V2 / "diag_plane_20261127/plane_diag.json"), plane_fix=str(V2 / "diag_plane_fix_20261127/plane_fix.json")),
               repro_check=dict(all_equal_3dec=ok, detail=chk), before=strip(rb), b2_before={k: {st: v[st]["summary"] for st in "abc"} for k, v in r2.items()},
               runtime_s=round(time.time() - t0, 1))
    (OUT / "pairs_table.json").write_text(json.dumps(res, indent=1, default=float))
    print("done", res["runtime_s"])


# ------------------------------------------------------------------ rules
GLOBAL_RATIO = 1.0068   # replaced in cmd_rules by the median BA2/BAP baseline ratio of all BAP-posed pairs (computed, stored in rules.json)


def rule_set():
    """name -> function(meta) -> (weight, z_scale, sin_min). weight 0 = drop the pair; z_scale multiplies Z and B (sigma scales with it);
    sin_min drops pixels whose ray is within asin(sin_min) of the baseline. All rules are global (no ROI information)."""
    g = lambda: GLOBAL_RATIO
    lowang = lambda m: m["perp_deg"] < 30
    R = {
        "all": lambda m: (1.0, 1.0, 0.0),
        "no_relax": lambda m: (0.0 if m["cls"] == "relax" else 1.0, 1.0, 0.0),
        "no_low": lambda m: (0.0 if m["cls"] == "low" else 1.0, 1.0, 0.0),
        "no_partial": lambda m: (0.0 if m["rect"] == "partial" else 1.0, 1.0, 0.0),
        "no_wide_vio": lambda m: (0.0 if m["cls"] == "wide_vio" else 1.0, 1.0, 0.0),
        "perp_ge30": lambda m: (0.0 if lowang(m) else 1.0, 1.0, 0.0),
        "w_perp_lt30_0.25": lambda m: (0.25 if lowang(m) else 1.0, 1.0, 0.0),
        "sin_ge_0.6": lambda m: (1.0, 1.0, 0.6),
        "rescale_ba2_pair": lambda m: (1.0, m["ba2_B_ratio"] if m.get("ba2_B_ratio") else (1.0 if m["pose"] == "ba2" else g()), 0.0),
        "rescale_ba2_global": lambda m: (1.0, 1.0 if m["pose"] == "ba2" else g(), 0.0),
        "rescale_global+no_relax": lambda m: (0.0 if m["cls"] == "relax" else 1.0, 1.0 if m["pose"] == "ba2" else g(), 0.0),
    }
    extra = os.environ.get("PAIR_RULES")
    if extra:
        keep = extra.split(",")
        R = {k: v for k, v in R.items() if k in keep or k == "all"}
    return R


def apply_rule(P, fn):
    out, wts = [], []
    for q in P:
        w, zs, smin = fn(q["meta"])
        if w <= 0:
            continue
        Z = q["Z"] * zs
        if smin > 0:
            Z = np.where(q["sn"] >= smin, Z, np.nan)
        out.append(dict(q, Z=Z, B=q["B"] * zs)); wts.append(w)
    return out, wts


def _cov_worker(args):
    ci, kf, Kfull, base_c, W, H, Kr = args
    import warnings; warnings.filterwarnings("ignore")
    poses = _cov_worker.poses
    MP = Maps(kf, Kfull, base_c, W, H, Kr, poses); P, F, Zb = MP.get(ci)
    global GLOBAL_RATIO
    GLOBAL_RATIO = _cov_worker.ratio
    out = dict(ci=ci, F=int(F.sum()), b2=int(np.isfinite(PF.refuse(P, F)).sum()))
    for nm, fn in rule_set().items():
        Pk, wk = apply_rule(P, fn)
        out[nm] = int(np.isfinite(refuse_w(Pk, F, wk)).sum())
    return out


def cmd_rules():
    t0 = time.time()
    kf, T, Kr, W, H, rc, RS_, Kfull, base_c, fps_all = setup(); poses = load_poses()
    global GLOBAL_RATIO
    rat = [m["ba2_B_ratio"] for ci in range(len(kf)) for m in pair_meta(kf, ci, W, H, Kr, poses) if m["pose"] != "ba2" and m.get("ba2_B_ratio")]
    GLOBAL_RATIO = float(np.median(rat))
    print("global BA2/BAP baseline ratio", GLOBAL_RATIO, len(rat), np.percentile(rat, [5, 25, 75, 95]), flush=True)
    MP = Maps(kf, Kfull, base_c, W, H, Kr, poses)
    res = dict(made=time.strftime("%F %T"), script=str(Path(__file__)), note=__doc__, inputs=dict(pairs_table=str(OUT / "pairs_table.json")),
               global_ratio=dict(median=GLOBAL_RATIO, n=len(rat), p5_25_75_95=[float(x) for x in np.percentile(rat, [5, 25, 75, 95])]),
               rules_doc=rule_set.__doc__, conditions={}, coverage={})
    for nm, fn in rule_set().items():
        def mo(ci, fn=fn):
            P, F, Zb = MP.get(ci); Pk, wk = apply_rule(P, fn)
            return dict(a=Pk, b=refuse_w(Pk, F, wk))
        r = measure_ext(nm, RS_, fps_all, mo, kf, T, rc, W, H, Kr, Kfull, extras=False)
        res["conditions"][nm] = {k: {st: v[st]["summary"] for st in "abc"} for k, v in r.items()}
        res["conditions"][nm]["_maps_b"] = {k: [dict(idx=s["idx"], offset_cm=s["offset_cm"], tilt_deg=s["tilt_deg"]) for s in v["b"]["maps"]] for k, v in r.items()}
    print("roi conditions done", round(time.time() - t0), flush=True)
    # coverage over all keyframes with FFS (CPU pool)
    from multiprocessing import Pool
    _cov_worker.poses = poses; _cov_worker.ratio = GLOBAL_RATIO
    roi_kf = sorted({ci for v in fps_all.values() for ci in v})
    step = int(os.environ.get("COV_STEP", "1"))
    todo = sorted(set(range(0, len(kf), step)) | set(roi_kf))
    with Pool(int(os.environ.get("NPROC", "8"))) as pool:
        rows = pool.map(_cov_worker, [(ci, kf, Kfull, base_c, W, H, Kr) for ci in todo], chunksize=4)
    names = list(rule_set())
    cov = {}
    for sub, sel in (("all_kf", lambda r: True), ("roi_kf", lambda r: r["ci"] in set(roi_kf))):
        L = [r for r in rows if sel(r)]; b2 = sum(r["b2"] for r in L); Fs = sum(r["F"] for r in L)
        cov[sub] = dict(n_kf=len(L), F_px=Fs, b2_px=b2, b2_over_F=b2 / max(Fs, 1),
                        kept_vs_b2={nm: sum(r[nm] for r in L) / max(b2, 1) for nm in names},
                        kf_losing_all={nm: sum(1 for r in L if r["b2"] > 0 and r[nm] == 0) for nm in names},
                        kf_losing_gt50pct={nm: sum(1 for r in L if r["b2"] > 0 and r[nm] < 0.5 * r["b2"]) for nm in names})
    res["coverage"] = cov; res["coverage_step"] = step; res["runtime_s"] = round(time.time() - t0, 1)
    tag = os.environ.get("RULES_TAG", "")
    (OUT / f"rules{tag}.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps(cov, indent=1), "\ndone", res["runtime_s"], flush=True)


if __name__ == "__main__":
    {"table": cmd_table, "rules": cmd_rules}[sys.argv[1]]()
