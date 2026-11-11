#!/usr/bin/env python3
"""ghost 11-11: fuse cached keyframe depth maps (ghost_cache.py, GHOST_CACHE npz) with different filter / fusion variants.
Depth maps are computed ONCE (FFS); only the step between depth and mesh changes. README.md in this folder.

usage: ghost_fuse.py CACHE_DIR OUT_DIR [--variants base,cons,...] [--diag]
  per variant -> OUT/<v>/{mesh.ply, mesh.obj, summary.json}; OUT/diag_kf.json (per-keyframe board height in the fused world).
Variants (all: TSDF 5 mm, z_max 2, sigma_max 0.08, sigma_ok 0.02, min_obs 2 = scan_offline / kfonly A, unless noted):
  base        = A (same inputs, same TSDF) -> should reproduce A's mesh
  twide       = truncation floor 2.5 vox -> 5 vox (2.5 cm), trunc_k 2 -> 3
  cons        = cross-view depth consistency: reproject each kf depth into K=8 overlapping kfs (any time), keep a pixel only if
                >= half of the neighbours that see it agree (|dz| <= max(5 mm, 3 hypot(sigma_i, sigma_j))); pixels seen by none kept
  consx       = same, neighbours only from other passes (|dt| >= 3 s)
  icp         = frame-to-model point-to-plane ICP of each kf cloud to the cloud of already fused kfs (open3d), small corrections only
  icp_cons    = icp poses, then cons
"""
import argparse
import glob
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
import tsdf_map  # noqa: E402

TS = dict(voxel_m=0.005, z_max=2.0, sigma_max=0.08, sigma_ok=0.02, min_obs=2)


def load(cache):
    K = []
    for f in sorted(glob.glob(f"{cache}/kf_*.npz")):
        d = np.load(f)
        h, w = [int(x) for x in d["shape"]]
        V = np.unpackbits(d["V"])[:h * w].reshape(h, w).astype(bool)
        Z = np.full((h, w), np.nan, np.float32); Z[V] = d["Zv"]
        S = np.full((h, w), np.inf, np.float32); S[V] = d["Sv"]
        inb = np.unpackbits(d["inb"])[:h * w].reshape(h, w).astype(bool)
        K.append(dict(i=int(Path(f).stem[3:]), Z=Z, V=V, S=S, inb=inb, jpg=d["jpg"].tobytes(), T=d["T"].astype(np.float64),
                      Tgt=d["Tgt"].astype(np.float64), t=int(d["t_ns"]) * 1e-9, frame=int(d["frame"]), f=float(d["f"]),
                      cx=float(d["cx"]), cy=float(d["cy"]), BW=float(d["BW"]), BH=float(d["BH"])))
    return K


def rays(k):
    h, w = k["Z"].shape
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    return np.stack([(uu - k["cx"]) / k["f"], (vv - k["cy"]) / k["f"], np.ones_like(uu)], -1)


def gt_plane_depth(k, R_):
    Rw, pw = k["Tgt"][:3, :3], k["Tgt"][:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        return -pw[2] / (R_ @ Rw[2])


def diag(K, R_):
    out = []
    for k in K:
        m = k["V"] & k["inb"]
        if m.sum() < 200:
            continue
        X = k["Z"][m, None] * R_[m]
        zw = X @ k["T"][:3, :3].T[:, 2] + k["T"][2, 3]          # fused world (BA pose), board frame z
        zg = X @ k["Tgt"][:3, :3].T[:, 2] + k["Tgt"][2, 3]      # GT pose (depth error only)
        Zp = gt_plane_depth(k, R_)[m]
        out.append(dict(i=k["i"], frame=k["frame"], t=k["t"], zw_med_cm=float(np.median(zw) * 100), zgt_med_cm=float(np.median(zg) * 100),
                        dz_med_cm=float(np.median(k["Z"][m] - Zp) * 100), Z_med_m=float(np.median(k["Z"][m])),
                        cam=k["T"][:3, 3].round(4).tolist(), dpos_cm=float(np.linalg.norm(k["T"][:3, 3] - k["Tgt"][:3, 3]) * 100),
                        n=int(m.sum())))
    return out


# ---------------- cross-view consistency ----------------
def neighbours(K, i, n=8, min_dt=0.0, max_dist=0.30, max_ang=35.0):
    c0, a0 = K[i]["T"][:3, 3], K[i]["T"][:3, 2]
    cand = []
    for j, k in enumerate(K):
        if j == i or abs(k["t"] - K[i]["t"]) < max(min_dt, 1e-6):
            continue
        d = np.linalg.norm(k["T"][:3, 3] - c0)
        ang = np.degrees(np.arccos(np.clip(k["T"][:3, 2] @ a0, -1, 1)))
        if d < max_dist and ang < max_ang:
            cand.append((d / max_dist + ang / max_ang, j))
    cand.sort()
    # spread over time: greedy, skip candidates within 0.5 s of an already chosen one
    ch = []
    for _, j in cand:
        if all(abs(K[j]["t"] - K[c]["t"]) > 0.5 for c in ch):
            ch.append(j)
        if len(ch) >= n:
            break
    return ch


def cons_mask(K, i, nbs, dev, thr_abs=0.005, ks=3.0, frac=0.5):
    k = K[i]
    Z = torch.as_tensor(k["Z"], device=dev); S = torch.as_tensor(k["S"], device=dev)
    h, w = Z.shape
    vs, us = torch.nonzero(torch.isfinite(Z), as_tuple=True)
    z = Z[vs, us].double()
    pc = torch.stack([(us - k["cx"]) / k["f"] * z, (vs - k["cy"]) / k["f"] * z, z], 1)
    seen = torch.zeros(len(z), dtype=torch.int32, device=dev); agree = torch.zeros_like(seen)
    for j in nbs:
        n = K[j]
        Tr = torch.as_tensor(np.linalg.inv(n["T"]) @ k["T"], device=dev)
        pn = pc @ Tr[:3, :3].T + Tr[:3, 3]
        zn = pn[:, 2]
        zz = torch.where(zn > 0.05, zn, torch.ones_like(zn))
        un = torch.round(pn[:, 0] / zz * n["f"] + n["cx"]).long(); vn = torch.round(pn[:, 1] / zz * n["f"] + n["cy"]).long()
        ok = (zn > 0.05) & (un >= 0) & (un < w) & (vn >= 0) & (vn < h)
        Zn_t = torch.as_tensor(n["Z"], device=dev); Sn_t = torch.as_tensor(n["S"], device=dev)
        Zn = torch.where(ok, Zn_t[vn.clamp(0, h - 1), un.clamp(0, w - 1)].double(), torch.full_like(zn, float("nan")))
        Sn = Sn_t[vn.clamp(0, h - 1), un.clamp(0, w - 1)].double()
        sn = torch.isfinite(Zn)
        tol = torch.clamp(ks * torch.hypot(S[vs, us].double(), Sn), min=thr_abs)
        seen += sn.int(); agree += (sn & ((Zn - zn).abs() <= tol)).int()
    keep = (seen == 0) | (agree.double() >= frac * seen.double())
    M = torch.zeros((h, w), dtype=torch.bool, device=dev); M[vs, us] = keep
    return M.cpu().numpy(), float((seen > 0).double().mean()), float((~keep).double().mean())


# ---------------- frame-to-model ICP ----------------
def icp_poses(K, R_, ds=4, max_t=0.03, max_r=2.0):
    import open3d as o3d
    model = None
    out, info = [], []
    for k in K:
        m = np.isfinite(k["Z"])[::ds, ::ds]
        X = (k["Z"][::ds, ::ds][m, None] * R_[::ds, ::ds][m])
        Xw = X @ k["T"][:3, :3].T + k["T"][:3, 3]
        src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Xw))
        T = k["T"].copy(); acc = False; fit = rmse = 0.0
        if model is not None and len(Xw) > 500:
            r = o3d.pipelines.registration.registration_icp(src, model, 0.02, np.eye(4),
                                                            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
            D = np.asarray(r.transformation)
            dt = np.linalg.norm(D[:3, 3]); dr = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
            fit, rmse = float(r.fitness), float(r.inlier_rmse)
            if fit > 0.3 and dt < max_t and dr < max_r:
                T = D @ T; acc = True; src.transform(D)
            info.append(dict(i=k["i"], acc=acc, fit=fit, rmse_mm=rmse * 1000, dt_mm=float(dt * 1000), dr_deg=float(dr)))
        out.append(T)
        model = src if model is None else (model + src)
        model = model.voxel_down_sample(0.005)
        model.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=20))
    return out, info


def mesh_metrics(Vm, BW, BH):
    Vm = np.asarray(Vm)
    if len(Vm) == 0:
        return {}
    mb = (Vm[:, 0] > 0.02) & (Vm[:, 0] < BW - 0.02) & (Vm[:, 1] > 0.02) & (Vm[:, 1] < BH - 0.02) & (np.abs(Vm[:, 2]) < 0.10)
    z = Vm[mb, 2] * 100
    med = np.median(z); far = np.abs(z - med) > 0.5
    near = mb & (np.abs(Vm[:, 2]) < 0.03)
    cx_ = np.floor((Vm[near, 0] - 0.02) * 100).astype(int); cy_ = np.floor((Vm[near, 1] - 0.02) * 100).astype(int)
    nx_, ny_ = int(np.ceil((BW - 0.04) * 100)), int(np.ceil((BH - 0.04) * 100))
    cells = set(zip(cx_.clip(0, nx_ - 1), cy_.clip(0, ny_ - 1)))
    return dict(mesh_vertices=int(len(Vm)), mesh_board_vertices=int(mb.sum()), ghost_frac=float(far.mean()),
                near_rms_cm=float(np.sqrt(np.mean((z[~far] - med) ** 2))), mesh_board_z_med_cm=float(med),
                mesh_board_z_rms_cm=float(np.sqrt(np.mean((z - med) ** 2))),
                mesh_board_z_p5_p95_cm=[float(np.percentile(z, 5)), float(np.percentile(z, 95))],
                mesh_board_absz_p90_cm=float(np.percentile(np.abs(z), 90)),
                mesh_board_cover_frac=len(cells) / float(nx_ * ny_))


def run_variant(K, R_, name, out, dev, icpT=None):
    t0 = time.time()
    tcfg = dict(TS)
    if name.startswith("twide"):
        tcfg.update(trunc_min_vox=5.0, trunc_k=3.0)
    use_cons = "cons" in name
    min_dt = 3.0 if "consx" in name else 0.0
    Ts = icpT if name.startswith("icp") else [k["T"] for k in K]
    Kp = [dict(k, T=T) for k, T in zip(K, Ts)]
    tsdf = tsdf_map.TSDFMap(**tcfg)
    dz, cst = [], []
    for ii, k in enumerate(Kp):
        V = k["V"].copy()
        if use_cons:
            M, sf, dropf = cons_mask(Kp, ii, neighbours(Kp, ii, min_dt=min_dt), dev)
            V &= M; cst.append((sf, dropf))
        m = V & k["inb"]
        if m.sum() > 200:
            dz.append((k["Z"][m] - gt_plane_depth(k, R_)[m]).astype(np.float32))
        bgr = cv2.imdecode(np.frombuffer(k["jpg"], np.uint8), cv2.IMREAD_COLOR)
        tsdf.integrate(np.where(V, k["Z"], np.nan).astype(np.float32), V, bgr, k["f"], k["cx"], k["cy"], 0.08, k["T"],
                       sigma=np.where(V, k["S"], np.inf).astype(np.float32))
    tsdf.update_mesh(force=True)
    Vm, Cm, Fm = tsdf.mesh_arrays()
    o = Path(out) / name; o.mkdir(parents=True, exist_ok=True)
    tsdf_map.write_mesh_ply(o / "mesh.ply", Vm, Cm, Fm)
    (o / "mesh.obj").write_bytes(tsdf_map.mesh_obj_bytes(Vm, Cm, Fm))
    D = np.concatenate(dz) * 100
    s = dict(variant=name, made=time.strftime("%Y-%m-%d %H:%M:%S"), tsdf=tcfg, n_kf=len(K), board_mm=[K[0]["BW"] * 1000, K[0]["BH"] * 1000],
             pooled_absdZ_cm=dict(p90=float(np.percentile(np.abs(D), 90)), median=float(np.median(np.abs(D)))), pooled_n=int(len(D)),
             fuse_s=round(time.time() - t0, 1))
    if cst:
        s["cons_seen_frac_med"] = float(np.median([c[0] for c in cst])); s["cons_drop_frac_med"] = float(np.median([c[1] for c in cst]))
        s["cons_drop_frac_mean"] = float(np.mean([c[1] for c in cst]))
    s.update(mesh_metrics(Vm, K[0]["BW"], K[0]["BH"]))
    (o / "summary.json").write_text(json.dumps(s, indent=1))
    print(json.dumps(s), flush=True)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cache"); ap.add_argument("out")
    ap.add_argument("--variants", default="base,twide,cons,consx,icp,icp_cons")
    a = ap.parse_args()
    t0 = time.time()
    K = load(a.cache)
    print(f"loaded {len(K)} kf in {time.time() - t0:.1f} s", flush=True)
    R_ = rays(K[0])
    Path(a.out).mkdir(parents=True, exist_ok=True)
    dg = diag(K, R_)
    (Path(a.out) / "diag_kf.json").write_text(json.dumps(dg, indent=0))
    dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    vs = a.variants.split(",")
    icpT = None
    if any(v.startswith("icp") for v in vs):
        t1 = time.time()
        icpT, info = icp_poses(K, R_)
        (Path(a.out) / "icp_info.json").write_text(json.dumps(dict(s=round(time.time() - t1, 1), rows=info), indent=0))
        print(f"icp {time.time() - t1:.1f} s, accepted {sum(r['acc'] for r in info)}/{len(info)}", flush=True)
    res = {v: run_variant(K, R_, v, a.out, dev, icpT) for v in vs}
    (Path(a.out) / "variants.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
