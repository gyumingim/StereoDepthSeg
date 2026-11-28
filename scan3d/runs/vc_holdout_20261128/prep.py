#!/usr/bin/env python3
"""vc_holdout 2026-11-28 step 1 (CPU, gsmap_env): region cut + keyframe removal + condition cloud + clip paths. Rules: GATE.md.

  nice -n 10 ~/scanner/gsmap_env/venv/bin/python prep.py   (writes prep/R1, prep/R2)
"""
import json, time
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

HERE = Path(__file__).resolve().parent
RUN = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs/gsmap_20261125_121801_v2"
DUMP = RUN / "ba_partners2/run/dump_ba"
FUSE = RUN / "ffsonly_ba2/fused"
REGIONS = {"R1": dict(kind="structure: wall + corner + map paper", kf=538, rect=(230, 700, 70, 225)),
           "R2": dict(kind="object: grey locker front", kf=774, rect=(550, 750, 120, 500))}
VIS_FRAC, VIS_TOL, CUT_R, RAY_TOL, SEEDS, NF = 0.002, 0.10, 0.05, 0.05, (123, 7), 25
CLOUD_R, STRIDE, VOX = 3.0, 2, 0.01
AMEND_1B = True


def prep_img(im):
    return cv2.resize(im[:, 48:912], (512, 320), interpolation=cv2.INTER_AREA)


def mask512(m):
    return cv2.resize(m[:, 48:912].astype(np.float32), (512, 320), interpolation=cv2.INTER_AREA) > 0.5


def main():
    t0 = time.time()
    cams = np.load(DUMP / "cams.npz"); T, kf, Kr = cams["T"], cams["kf"], cams["Kr"]; zmax = float(cams["zmax"])
    Ki = np.linalg.inv(Kr); N = len(kf); H0, W0 = 540, 960
    K864 = Kr.copy(); K864[0, 2] -= 48; K512 = K864.copy(); K512[:2] *= 512 / 864
    Zs = {}
    def Z(i):
        if i not in Zs:
            Zs[i] = np.load(FUSE / f"{kf[i]:05d}.npz")["Z"].astype(np.float32)
        return Zs[i]
    def ok(z):
        return np.isfinite(z) & (z > 0.05)
    C = T[:, :3, 3]
    for name, r in REGIONS.items():
        out = HERE / "prep" / name; out.mkdir(parents=True, exist_ok=True)
        ti = int(np.nonzero(kf == r["kf"])[0][0]); x0, x1, y0, y1 = r["rect"]
        Zt = Z(ti).copy(); sub = Zt[y0:y1, x0:x1]; med = float(np.median(sub[ok(sub)])); valid_frac = float(ok(sub).mean())
        # amendment 1: region = full rectangle; invalid depth inside it -> depth of the nearest valid pixel of the rectangle
        bad = ~ok(sub); _, lab = cv2.distanceTransformWithLabels(bad.astype(np.uint8), cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        vv_ok, uu_ok = np.nonzero(~bad); lut = np.zeros(lab.max() + 1, np.float32); lut[lab[~bad]] = sub[vv_ok, uu_ok]
        sub = np.where(bad, lut[lab], sub); Zt[y0:y1, x0:x1] = sub
        Mt = np.zeros((H0, W0), bool); Mt[y0:y1, x0:x1] = True
        v, u = np.nonzero(Mt); z = Zt[v, u]
        S = ((np.stack([u, v, np.ones_like(u)], 1) @ Ki.T) * z[:, None]) @ T[ti, :3, :3].T + T[ti, :3, 3]
        Ss = S[np.random.default_rng(0).choice(len(S), min(20000, len(S)), replace=False)]
        # --- keyframe visibility of S
        vis = np.zeros(N)
        for j in range(N):
            w2c = np.linalg.inv(T[j]); Xc = Ss @ w2c[:3, :3].T + w2c[:3, 3]; zz = Xc[:, 2]; f = zz > 0.05
            uv = (Xc[f] @ Kr.T); uu = uv[:, 0] / uv[:, 2]; vv = uv[:, 1] / uv[:, 2]
            inn = (uu >= 0) & (uu < W0 - 0.5) & (vv >= 0) & (vv < H0 - 0.5)
            if not inn.any():
                continue
            ui, vi = np.round(uu[inn]).astype(int), np.round(vv[inn]).astype(int); zj = Z(j)[vi, ui]
            seen = ~ok(zj) | (zz[f][inn] <= zj + VIS_TOL)
            vis[j] = seen.sum() / len(Ss)
            if len(Zs) > 64:
                Zs.clear()
        removed = np.nonzero(vis >= VIS_FRAC)[0]; keep = np.setdiff1d(np.arange(N), removed)
        assert ti in removed
        # --- start keyframes
        Rt = T[ti, :3, :3]
        ang = np.array([np.arccos(np.clip((np.trace(Rt.T @ T[j, :3, :3]) - 1) / 2, -1, 1)) for j in range(N)])
        d = np.linalg.norm(C - C[ti], axis=1) + 0.5 * ang
        order = [j for j in np.argsort(d) if j in set(keep.tolist())]
        s1 = order[0]; s2 = next(j for j in order[1:] if abs(j - s1) >= 5)
        starts = [int(s1), int(s2)]
        # --- condition cloud from kept keyframes near target/starts
        # amendment 1: removed keyframes give no photo, but their points outside the region stay (= measured map minus region)
        near = [j for j in range(N) if min(np.linalg.norm(C[j] - C[k]) for k in [ti] + starts) < CLOUD_R]
        keys, cols = [], []
        vv0, uu0 = np.mgrid[0:H0:STRIDE, 0:W0:STRIDE]
        for j in near:
            zj = Z(j)[vv0, uu0]; im = cv2.cvtColor(cv2.imread(str(DUMP / "img" / f"{kf[j]:05d}.png")), cv2.COLOR_BGR2RGB)
            border = (cv2.dilate((im.max(-1) <= 8).astype(np.uint8), np.ones((7, 7), np.uint8)) > 0)[vv0, uu0]
            m = ok(zj) & (zj < zmax) & ~border
            X = (np.stack([uu0[m], vv0[m], np.ones(m.sum())], 1) @ Ki.T) * zj[m][:, None]
            keys.append((X @ T[j, :3, :3].T + T[j, :3, 3]).astype(np.float32)); cols.append(im[vv0[m], uu0[m]])
            if len(Zs) > 64:
                Zs.clear()
        P = np.concatenate(keys); Cc = np.concatenate(cols).astype(np.float32); n_raw = len(P)
        key = np.floor(P / VOX).astype(np.int64); _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel(); m = len(cnt)
        P = (np.stack([np.bincount(inv, P[:, k], m) for k in range(3)], 1) / cnt[:, None]).astype(np.float32)
        Cc = (np.stack([np.bincount(inv, Cc[:, k], m) for k in range(3)], 1) / cnt[:, None])
        n_vox = len(P)
        dS, _ = cKDTree(S).query(P, distance_upper_bound=CUT_R); cut1 = np.isfinite(dS)
        w2c = np.linalg.inv(T[ti]); Xc = P @ w2c[:3, :3].T + w2c[:3, 3]; cut2 = np.zeros(len(P), bool)
        f = Xc[:, 2] > 0.05; uv = Xc[f] @ Kr.T; uu = np.round(uv[:, 0] / uv[:, 2]).astype(int); vv = np.round(uv[:, 1] / uv[:, 2]).astype(int)
        inn = (uu >= 0) & (uu < W0) & (vv >= 0) & (vv < H0); Md = cv2.dilate(Mt.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        Zf = cv2.dilate(np.where(Mt, Zt, 0).astype(np.float32), np.ones((5, 5), np.uint8))   # region depth incl. 2 px rim
        idx_f = np.nonzero(f)[0][inn]; uu, vv = uu[inn], vv[inn]; zz = Xc[idx_f, 2]
        # amendment 1b: nothing occludes either rectangle in the target photo (checked by eye) -> cut every point on rectangle rays
        hit = Md[vv, uu] & (zz > 0.05) if AMEND_1B else Md[vv, uu] & (zz >= np.where(Zf[vv, uu] > 0, Zf[vv, uu], 1e9) - RAY_TOL)
        cut2[idx_f[hit]] = True
        P, Cc = P[~(cut1 | cut2)], Cc[~(cut1 | cut2)]
        np.savez_compressed(out / "cloud.npz", P=P, C=np.clip(Cc, 0, 255).astype(np.uint8))
        # --- paths
        clips = []
        for si in starts:
            w = np.linspace(0, 1, NF)
            Rk = Slerp([0, 1], Rotation.from_matrix(np.stack([T[si, :3, :3], Rt])))(w).as_matrix()
            c2w = np.tile(np.eye(4), (NF, 1, 1)); c2w[:, :3, :3] = Rk; c2w[:, :3, 3] = (1 - w)[:, None] * C[si] + w[:, None] * C[ti]
            np.save(out / f"path_s{kf[si]}.npy", c2w)
            cv2.imwrite(str(out / f"start_{kf[si]}.png"), prep_img(cv2.imread(str(DUMP / "img" / f"{kf[si]:05d}.png"))))
            for sd in SEEDS:
                clips.append(dict(clip=f"{name}_s{kf[si]}_seed{sd}", start_idx=si, start_kf=int(kf[si]), seed=sd,
                                  path=f"path_s{kf[si]}.npy", start_png=f"start_{kf[si]}.png",
                                  move_m=round(float(np.linalg.norm(C[si] - C[ti])), 3), turn_deg=round(float(np.degrees(ang[si])), 1),
                                  start_sees_S=round(float(vis[si]), 5)))
        cv2.imwrite(str(out / "gt.png"), prep_img(cv2.imread(str(DUMP / "img" / f"{r['kf']:05d}.png"))))
        cv2.imwrite(str(out / "region_mask.png"), mask512(Mt).astype(np.uint8) * 255)
        info = dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), region=name, **r, target_idx=ti, rect_median_depth_m=round(med, 3), rect_valid_depth_frac=round(valid_frac, 3),
                    amendment="1 (GATE.md): full rectangle, nearest-valid depth fill; removed kf points outside region kept",
                    n_S=int(len(S)), inputs=dict(cams=str(DUMP / "cams.npz"), img=str(DUMP / "img"), fused=str(FUSE)),
                    params=dict(VIS_FRAC=VIS_FRAC, VIS_TOL=VIS_TOL, CUT_R=CUT_R, RAY_TOL=RAY_TOL, CLOUD_R=CLOUD_R, STRIDE=STRIDE, VOX=VOX, zmax=zmax),
                    removed_kf=[int(kf[j]) for j in removed], n_removed=int(len(removed)), n_kf=int(N),
                    vis_of_target=round(float(vis[ti]), 4), starts=[int(kf[s]) for s in starts],
                    start_dist=[round(float(d[s]), 3) for s in starts], cloud_kf=len(near), n_raw=int(n_raw), n_vox=int(n_vox),
                    n_cut_near_S=int(cut1.sum()), n_cut_ray=int((cut2 & ~cut1).sum()), n_points=int(len(P)),
                    K512=K512.tolist(), clips=clips)
        (out / "info.json").write_text(json.dumps(info, indent=1))
        print(json.dumps({k: v for k, v in info.items() if k not in ("removed_kf", "K512")}, indent=1), flush=True)
    print("prep s", round(time.time() - t0, 1))


if __name__ == "__main__":
    main()
