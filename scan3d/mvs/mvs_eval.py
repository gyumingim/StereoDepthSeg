#!/usr/bin/env python3
"""mvs 11-12: compare meshes / point clouds of the same keyframes + poses (no ground truth: agreement and coverage only).
Per input (label=path.ply):
  * vertices; camera depth of each vertex = nearest keyframe camera (ALL keyframes of PREP/cams.npz) that has it in its 960x540 view
    (no occlusion test; same rule as scan_offline mesh_cam_depth_m but over all 494 keyframes) -> p50 / p90 / p99 / max, frac > 1.6 / 2.0 m
  * keyframe view cover = fraction of each keyframe's image (480x270 cells) hit by projected vertices (no occlusion; = fixes_tmp/cover.py)
    median over all keyframes and over the baseline's integrated keyframes (--kf-subset npz)
  * near-range agreement with the reference (first input): for vertices with camera depth < 1 m, distance to the nearest reference vertex
    (median / p90 / frac > 2 cm), and the reverse (reference vertices < 1 m -> nearest vertex of this input = completeness)
  * top-view area of 5 cm cells
Renders (headless numpy): OUT_top.png (row per input: top colour | slice 0.4-1.2 m above floor), OUT_kfviews.jpg (2 keyframes: photo |
mesh splat with z-buffer per input).
Usage: mvs_eval.py PREP OUT_PREFIX ref=REF.ply name=X.ply ... [--kf-subset BASE/kf_poses.npz] [--zoff 1000]"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "runs" / "fixes_20261112"))
from render_mesh import read_ply  # noqa: E402


def cam_depth_and_cover(X, T, f, cx, cy, W=960, H=540):
    best = np.full(len(X), np.inf, np.float32)
    cov = []
    Xf = X.astype(np.float32)
    for Tw in T:
        R = Tw[:3, :3].astype(np.float32); t = Tw[:3, 3].astype(np.float32)
        pc = (Xf - t) @ R
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = pc[:, 0] / z * f + cx; v = pc[:, 1] / z * f + cy
        inn = (z > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
        best = np.where(inn, np.minimum(best, z), best)
        m = inn & (z > 0.1)
        img = np.zeros((H // 2, W // 2), bool); img[(v[m] // 2).astype(int), (u[m] // 2).astype(int)] = True
        cov.append(img.mean())
    return best, np.array(cov)


def top(X, C, lo, hi, res, cams, zsel=None, gray=False):
    W, H = int((hi[0] - lo[0]) / res) + 1, int((hi[1] - lo[1]) / res) + 1
    img = np.full((H, W, 3), 255, np.uint8)
    m = np.ones(len(X), bool) if zsel is None else (X[:, 2] >= zsel[0]) & (X[:, 2] <= zsel[1])
    x, c = X[m], C[m]
    u = ((x[:, 0] - lo[0]) / res).astype(int); v = (H - 1 - (x[:, 1] - lo[1]) / res).astype(int)
    k = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if gray:
        cnt = np.zeros((H, W)); np.add.at(cnt, (v[k], u[k]), 1)
        g = np.clip(255 - cnt * 25, 0, 255).astype(np.uint8); img = np.dstack([g, g, g])
    else:
        o = np.argsort(x[k, 2])
        img[v[k][o], u[k][o]] = c[k][o][:, ::-1]
    for cxy in cams:
        cv2.circle(img, (int((cxy[0] - lo[0]) / res), int(H - 1 - (cxy[1] - lo[1]) / res)), 2, (0, 0, 255), -1)
    return img


def splat(X, C, Tw, f, cx, cy, W, H, zmax=12.0):
    R = Tw[:3, :3]; t = Tw[:3, 3]
    pc = (X - t) @ R
    z = pc[:, 2]
    ok = (z > 0.1) & (z < zmax)
    u = (pc[ok, 0] / z[ok] * f + cx).astype(int); v = (pc[ok, 1] / z[ok] * f + cy).astype(int)
    zz, cc = z[ok], C[ok][:, ::-1]
    img = np.full((H, W, 3), 255, np.uint8); zb = np.full((H, W), np.inf, np.float32)
    for du in (0, 1):
        for dv in (0, 1):
            uu, vv = u + du, v + dv
            m = (uu >= 0) & (uu < W) & (vv >= 0) & (vv < H)
            uu, vv, z_, c_ = uu[m], vv[m], zz[m], cc[m]
            o = np.argsort(-z_)                       # far first; near overwrite = z-buffer
            img[vv[o], uu[o]] = c_[o]
            np.minimum.at(zb, (vv, uu), z_)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prep"); ap.add_argument("out"); ap.add_argument("inputs", nargs="+")
    ap.add_argument("--kf-subset", default="")
    ap.add_argument("--zoff", type=float, default=1000.0)
    ap.add_argument("--kfs", default="175,437")
    a = ap.parse_args()
    P = Path(a.prep)
    Cm = np.load(P / "cams.npz")
    T = Cm["T"]; K = Cm["K960"]; f, cx, cy = float(K[0, 0]), float(K[0, 2]), float(K[1, 2])
    names = [str(x) for x in Cm["names"]]
    sub = None
    if a.kf_subset:
        S = np.load(a.kf_subset)
        cen = T[:, :3, 3]
        sub = np.array([int(np.argmin(np.linalg.norm(cen - Ts[:3, 3], axis=1))) for Ts in S["T"]])
    res, meshes = {}, []
    ref_tree = ref_near = None
    for s in a.inputs:
        lab, _, p = s.partition("=")
        X, C = read_ply(p)
        d, cov = cam_depth_and_cover(X, T, f, cx, cy)
        fin = np.isfinite(d)
        r = dict(path=p, vertices=int(len(X)), cam_depth_m=dict(n=int(fin.sum()), p50=float(np.median(d[fin])), p90=float(np.percentile(d[fin], 90)),
                 p99=float(np.percentile(d[fin], 99)), max=float(d[fin].max()), frac_gt_1p0=float(np.mean(d[fin] > 1.0)),
                 frac_gt_1p6=float(np.mean(d[fin] > 1.6)), frac_gt_2p0=float(np.mean(d[fin] > 2.0))),
                 frac_not_in_any_view=float(np.mean(~fin)),
                 kf_view_cover_median=float(np.median(cov)), kf_view_cover_lt_0p1=float(np.mean(cov < 0.1)),
                 top_area_m2_5cm=round(len(np.unique(np.floor(X[:, :2] / 0.05).astype(np.int64), axis=0)) * 0.0025, 2))
        if sub is not None:
            r["kf_view_cover_median_base_kfs"] = float(np.median(cov[sub]))
        near = fin & (d < 1.0)
        if ref_tree is None:
            ref_tree, refX, ref_near = cKDTree(X), X, near
            r["near_lt_1m_vertices"] = int(near.sum())
        else:
            dd, _ = ref_tree.query(X[near], k=1, workers=-1)
            tr = cKDTree(X)
            rd, _ = tr.query(refX[ref_near], k=1, workers=-1)
            r["near_lt_1m"] = dict(vertices=int(near.sum()), to_ref_median_mm=float(np.median(dd) * 1000), to_ref_p90_mm=float(np.percentile(dd, 90) * 1000),
                                   to_ref_frac_gt_2cm=float(np.mean(dd > 0.02)), ref_to_this_median_mm=float(np.median(rd) * 1000),
                                   ref_to_this_p90_mm=float(np.percentile(rd, 90) * 1000), ref_covered_frac_2cm=float(np.mean(rd <= 0.02)))
        res[lab] = r
        meshes.append((lab, X, C))
        print(lab, json.dumps(r), flush=True)
    # renders
    allX = np.concatenate([m[1] for m in meshes[:1]])
    lo = np.percentile(allX[:, :2], 0.5, 0) - 1.0; hi = np.percentile(allX[:, :2], 99.5, 0) + 1.0
    floor = np.percentile(allX[:, 2], 2); ceil = np.percentile(allX[:, 2], 97)
    camxy = T[:, :3, 3][:, :2]
    rows = []
    for lab, X, C in meshes:
        m = X[:, 2] < ceil
        t1 = top(X[m], C[m], lo, hi, 0.01, camxy)
        t2 = top(X, C, lo, hi, 0.01, camxy, zsel=(floor + 0.4, floor + 1.2), gray=True)
        cv2.putText(t1, f"{lab}: top colour ({len(X)} v)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
        cv2.putText(t2, "slice 0.4-1.2 m above floor", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
        rows.append(np.hstack([t1, np.zeros((t1.shape[0], 8, 3), np.uint8), t2]))
    img = np.vstack([np.vstack([r_, np.zeros((8, r_.shape[1], 3), np.uint8)]) for r_ in rows])
    sc = min(1.0, 2400 / img.shape[1])
    cv2.imwrite(a.out + "_top.png", cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
    kv = []
    for k in [int(x) for x in a.kfs.split(",")]:
        k = min(k, len(names) - 1)
        ph = cv2.imread(str(P / "img960" / names[k]))
        cv2.putText(ph, f"kf {k} ({names[k]}) photo", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        tiles = [ph]
        for lab, X, C in meshes:
            t_ = splat(X, C, T[k], f, cx, cy, 960, 540)
            cv2.putText(t_, lab, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            tiles.append(t_)
        kv.append(np.hstack(tiles))
    cv2.imwrite(a.out + "_kfviews.jpg", cv2.resize(np.vstack(kv), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 85])
    Path(a.out + "_eval.json").write_text(json.dumps(dict(prep=str(P), inputs=a.inputs, results=res), indent=1))
    print("wrote", a.out + "_top.png", a.out + "_kfviews.jpg", a.out + "_eval.json")


if __name__ == "__main__":
    main()
