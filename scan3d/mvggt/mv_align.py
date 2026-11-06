#!/usr/bin/env python3
"""mvggt: turn raw VGGT / MASt3R outputs into scan_offline inputs in the BA (= VIO metric) frame (README 3) + pose accuracy vs GT.
  vggt  : chunks chained (overlap: depth-ratio scale, mean rotation, centre translation) -> V1 poses + depth (one Sim3 to BA);
          V2 depth = per-chunk depth x (chunk centres -> BA centres Sim3 scale); mask = depth_conf >= per-frame median (drop low 50 %).
  mast3r: M1 poses + depth (one Sim3 to BA); M2 depth x same scale; mask = conf > 1.5.
Writes OUT/<V>_poses.jsonl.gz, OUT/<V>_depth.npz (t_ns = recording-row times, depth (N,540,960) m, mask), OUT/align_<kind>.json.
usage: mv_align.py vggt|mast3r FRAMES_DIR RAW.npz --out DIR [--vio-kf poses_vio_kf.jsonl.gz]
"""
import argparse
import gzip
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def umeyama(A, B):
    """s, R, t with B ~ s R A + t (rows = points)."""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum())
    return s, R, mb - s * R @ ma


def proj_rot(M):
    U, _, Vt = np.linalg.svd(M)
    return U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt


def apply_sim3(T, s, R, t):
    out = T.copy()
    out[:, :3, :3] = R @ T[:, :3, :3]
    out[:, :3, 3] = (s * (R @ T[:, :3, 3].T)).T + t
    return out


def rot_deg(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Ra, Rb) - 1) / 2, -1, 1)))


def ortho(T):
    """float32 model rotations are slightly non-orthonormal (MASt3R det ~1.0005): arccos(trace) would clip errors < ~2 deg
    to 0 -> project every rotation to SO(3) (11-06 fix, found on 165506 M1 rot error 0.00)."""
    T = np.array(T, np.float64)
    for k in range(len(T)):
        T[k, :3, :3] = proj_rot(T[k, :3, :3])
    return T


def pose_eval(T, Tgt, ok):
    """Sim3 to GT camera centres on GT-ok frames: ATE RMSE cm, scale k = 1/s - 1 (%; >0 = estimate larger than GT), rotation error."""
    A, B = T[ok, :3, 3], Tgt[ok, :3, 3]
    s, R, t = umeyama(A, B)
    Ta = apply_sim3(T[ok], s, R, t)
    e = np.linalg.norm(Ta[:, :3, 3] - B, axis=1) * 100
    r = rot_deg(Ta[:, :3, :3], Tgt[ok, :3, :3])
    return dict(n=int(ok.sum()), ate_rmse_cm=float(np.sqrt((e ** 2).mean())), ate_med_cm=float(np.median(e)),
                ate_p90_cm=float(np.percentile(e, 90)), k_pct=float((1 / s - 1) * 100), rot_med_deg=float(np.median(r)),
                rot_p90_deg=float(np.percentile(r, 90)))


def write_poses(path, t_ns, T):
    with gzip.open(path, "wt") as f:
        for t, M in zip(t_ns, T):
            f.write(json.dumps(dict(type="pose", status="tracking", t_ns=int(t), T_world_cam0=M.reshape(-1).tolist())) + "\n")


def load_poses(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    return np.array(t, np.int64), np.array(T)


def up(D, M, W0, H0):
    Du = np.stack([cv2.resize(d, (W0, H0), interpolation=cv2.INTER_LINEAR) for d in D]).astype(np.float32)
    Mu = np.stack([cv2.resize(m.astype(np.uint8), (W0, H0), interpolation=cv2.INTER_NEAREST) for m in M]).astype(bool)
    return Du, Mu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["vggt", "mast3r"])
    ap.add_argument("frames")
    ap.add_argument("raw")
    ap.add_argument("--out", required=True)
    ap.add_argument("--vio-kf", default="")
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    fr = np.load(Path(a.frames) / "frames.npz")
    tn, trow, Tba, Tgt, gok = fr["t_ns"], fr["t_row"], fr["T_ba"], fr["T_gt"], fr["gt_ok"].astype(bool)
    Kr = fr["K"]; W0, H0 = [int(x) for x in fr["size"]]
    raw = np.load(a.raw)
    assert np.array_equal(raw["t_ns"], tn)
    N = len(tn)
    info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), kind=a.kind, frames=a.frames, raw=a.raw, n=N, n_gt_ok=int(gok.sum()))
    if a.kind == "vggt":
        E, I, D, C, cid, fidx = raw["extrinsic"], raw["intrinsic"], raw["depth"], raw["conf"], raw["chunk"], raw["fidx"]
        h, w = D.shape[1:]
        info["model_hw"] = [int(h), int(w)]
        # focal in our 960 grid units (VGGT crop mode stretches x by w/W0 and y by h/H0)
        info["focal_pred_x_960"] = float(np.median(I[:, 0, 0]) * W0 / w)
        info["focal_pred_y_960"] = float(np.median(I[:, 1, 1]) * H0 / h)
        info["focal_ours"] = float(Kr[0, 0])
        c2w = np.zeros((len(E), 4, 4)); c2w[:, 3, 3] = 1
        Rw = np.transpose(E[:, :3, :3], (0, 2, 1))
        c2w[:, :3, :3] = Rw; c2w[:, :3, 3] = -np.einsum("nij,nj->ni", Rw, E[:, :3, 3])
        c2w = ortho(c2w)
        msk = C >= np.median(C.reshape(len(C), -1), axis=1)[:, None, None]
        nch = int(cid.max()) + 1
        chain = np.full((N, 4, 4), np.nan); dep1 = np.full((N, h, w), np.nan, np.float32); m1 = np.zeros((N, h, w), bool)
        dep2 = np.full((N, h, w), np.nan, np.float32); src_scale2 = np.full(N, np.nan)
        links = []
        for k in range(nch):
            q = np.flatnonzero(cid == k); f = fidx[q]
            Tk, Dk, Mk = c2w[q], D[q], msk[q]
            # V2 scale: this chunk's centres -> BA centres
            s2, _, _ = umeyama(Tk[:, :3, 3], Tba[f, :3, 3])
            new = np.isnan(dep2[f, 0, 0])
            dep2[f[new]] = Dk[new] * s2; src_scale2[f[new]] = s2
            if k == 0:
                chain[f] = Tk; dep1[f] = Dk; m1[f] = Mk
                links.append(dict(chunk=0, s=1.0, n_overlap=0, v2_scale=s2))
                continue
            ov = np.flatnonzero(~np.isnan(chain[f, 0, 0]))
            assert len(ov) >= 2, (k, len(ov))
            fo = f[ov]
            both = m1[fo] & Mk[ov] & np.isfinite(dep1[fo]) & (Dk[ov] > 0)
            s = float(np.median(dep1[fo][both] / Dk[ov][both]))
            R = proj_rot(sum(chain[fo[i], :3, :3] @ Tk[ov[i], :3, :3].T for i in range(len(ov))))
            t = np.mean([chain[fo[i], :3, 3] - s * R @ Tk[ov[i], :3, 3] for i in range(len(ov))], axis=0)
            Ta = apply_sim3(Tk, s, R, t)
            resid = np.linalg.norm(Ta[ov, :3, 3] - chain[fo, :3, 3], axis=1)
            rres = rot_deg(Ta[ov, :3, :3], chain[fo, :3, :3])
            links.append(dict(chunk=k, s=s, n_overlap=int(len(ov)), centre_resid_rel=float(np.median(resid) / max(1e-9, np.median(
                np.linalg.norm(np.diff(chain[fo, :3, 3], axis=0), axis=1)))), rot_resid_med_deg=float(np.median(rres)), v2_scale=s2))
            nw = np.isnan(chain[f, 0, 0])
            chain[f[nw]] = Ta[nw]; dep1[f[nw]] = Dk[nw] * s; m1[f[nw]] = Mk[nw]
        info["links"] = links
        assert not np.isnan(chain).any()
        model_T, model_D, model_M = chain, dep1, m1
        v2_D = dep2
        names = ("V1", "V2")
    else:
        c2w, K, D, C = ortho(raw["cam2w"]), raw["K"], raw["depth"], raw["conf"]
        h, w = D.shape[1:]
        info["model_hw"] = [int(h), int(w)]
        info["focal_final_960"] = float(np.median(K[:, 0, 0]) * W0 / w); info["focal_ours"] = float(Kr[0, 0])
        model_T, model_D, model_M = c2w, D, C > 1.5
        v2_D = None
        names = ("M1", "M2")
    s, R, t = umeyama(model_T[:, :3, 3], Tba[:, :3, 3])
    T1 = apply_sim3(model_T, s, R, t)
    info["sim3_to_ba"] = dict(s=s, centre_rms_cm=float(np.sqrt((np.linalg.norm(T1[:, :3, 3] - Tba[:, :3, 3], axis=1) ** 2).mean()) * 100),
                              rot_med_deg=float(np.median(rot_deg(T1[:, :3, :3], Tba[:, :3, :3]))))
    D1u, M1u = up(model_D * s, model_M, W0, H0)
    write_poses(out / f"{names[0]}_poses.jsonl.gz", tn, T1)
    np.savez(out / f"{names[0]}_depth.npz", t_ns=trow, depth=D1u, mask=M1u)
    D2 = (v2_D if v2_D is not None else model_D * s)
    D2u, M2u = up(D2, model_M, W0, H0)
    np.savez(out / f"{names[1]}_depth.npz", t_ns=trow, depth=D2u, mask=M2u)
    info["mask_frac_mean"] = float(model_M.mean())
    info["depth_med_m_V1like"] = float(np.nanmedian(D1u[M1u]))
    # pose accuracy (all on the same frames, GT ok only)
    Tba = ortho(Tba); Tgt = ortho(Tgt)
    pe = dict(BA=pose_eval(Tba, Tgt, gok), **{names[0]: pose_eval(T1, Tgt, gok)})
    if a.kind == "mast3r":
        sg, _, _ = umeyama(model_T[gok, :3, 3], Tgt[gok, :3, 3])
        info["mast3r_raw_metric_k_pct"] = float((1 / sg - 1) * 100)     # its own metric scale vs GT
    if a.vio_kf:
        vt, vT = load_poses(a.vio_kf)
        idx = [int(np.argmin(np.abs(vt - x))) for x in tn]
        if all(abs(int(vt[i]) - int(x)) < 1e6 for i, x in zip(idx, tn)):
            pe["VIO_kf"] = pose_eval(vT[idx], Tgt, gok)
    info["pose_eval"] = pe
    (out / f"align_{a.kind}.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info, indent=1)[:3000])


if __name__ == "__main__":
    main()
