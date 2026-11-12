#!/usr/bin/env python3
"""fixes 11-12 u11: headless render of a merged scan (no window).
render_merge.py MERGE_DIR OUT_PREFIX [--lens FILE]
 -> OUT_PREFIX_top.png  (left: top view colour, right: wall slice 0.4-1.2 m above floor; cameras A red, B blue)
 -> OUT_PREFIX_kfviews.jpg (last A keyframe + B keyframe nearest to it: photo | mesh points from that camera)"""
import argparse, gzip, json, sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from render_mesh import read_ply  # noqa: E402


def load_poses(p):
    out = {}
    with gzip.open(p, "rt") as f:
        for l in f:
            d = json.loads(l)
            if d.get("type") == "pose":
                out[int(d["frame"])] = np.array(d["T_world_cam0"], float).reshape(4, 4)
    return out


def top(X, C, cams, lo, hi, res, zsel=None, gray=False):
    W, H = int((hi[0] - lo[0]) / res) + 1, int((hi[1] - lo[1]) / res) + 1
    img = np.full((H, W, 3), 255, np.uint8)
    m = np.ones(len(X), bool) if zsel is None else (X[:, 2] >= zsel[0]) & (X[:, 2] <= zsel[1])
    x, c = X[m], C[m]
    u = ((x[:, 0] - lo[0]) / res).astype(int); v = (H - 1 - (x[:, 1] - lo[1]) / res).astype(int)
    k = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    o = np.argsort(x[k, 2])  # low first, high overwrites
    if gray:
        cnt = np.zeros((H, W)); np.add.at(cnt, (v[k], u[k]), 1)
        g = np.clip(255 - cnt * 25, 0, 255).astype(np.uint8); img = np.dstack([g, g, g])
    else:
        img[v[k][o], u[k][o]] = c[k][o][:, ::-1]
    for (cx, cy), col in cams:
        cv2.circle(img, (int((cx - lo[0]) / res), int(H - 1 - (cy - lo[1]) / res)), 2, col, -1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("merge"); ap.add_argument("out")
    ap.add_argument("--lens", default="/home/system/scanner/StereoDepthSeg-fixes/scan3d/config/lens/std_1920x1080_f0.50_calcap20261026.json")
    a = ap.parse_args()
    M = Path(a.merge)
    X, C = read_ply(next((M / "scan" / "mesh").glob("mesh.ply")))
    X[:, 2] -= 1000.0  # scan_offline mesh = VIO world + 1000 m in z (lensonly_gt.py stand-in GT)
    P = load_poses(M / "scan" / "ba" / "poses_ba.jsonl.gz")
    mj = json.loads(next(M.glob("merged/*_align/merge.json")).read_text())
    off = mj["frame_offset"][1]
    fr = np.array(sorted(P)); cen = np.array([P[f][:3, 3] for f in fr]); isB = fr >= off
    floor = np.percentile(X[:, 2], 2)
    lo, hi = np.percentile(X[:, :2], 0.5, 0) - 0.3, np.percentile(X[:, :2], 99.5, 0) + 0.3
    res = 0.01
    cams = [((c[0], c[1]), (255, 0, 0) if b else (0, 0, 255)) for c, b in zip(cen, isB)]
    ceil = np.percentile(X[:, 2], 97)
    t1 = top(X[X[:, 2] < ceil], C[X[:, 2] < ceil], cams, lo, hi, res)
    t2 = top(X, C, cams, lo, hi, res, zsel=(floor + 0.4, floor + 1.2), gray=True)
    for t, s in ((t1, "top colour (red cam = 121801, blue = 131328)"), (t2, "slice 0.4-1.2 m above floor (dark = many points)")):
        cv2.putText(t, s, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
    cv2.imwrite(a.out + "_top.png", np.hstack([t1, np.full((t1.shape[0], 10, 3), 0, np.uint8), t2]))
    # kf views: B keyframes at ~10 s and ~14 s of the 2nd recording (the overlap), each next to the A keyframe that looked most alike
    # (centre distance + 2 cm per degree of view direction)
    fbB = fr[isB]; zA = np.array([P[f][:3, 2] for f in fr[~isB]]); cA = cen[~isB]
    pairs = []
    for src in (600, 840):
        fb = int(fbB[np.argmin(np.abs(fbB - off - src))])
        ang = np.degrees(np.arccos(np.clip(zA @ P[fb][:3, 2], -1, 1)))
        sc = np.linalg.norm(cA - P[fb][:3, 3], axis=1) + 0.02 * ang
        j = int(np.argmin(sc)); pairs += [int(fr[~isB][j]), fb]
    d = np.array([0.0])
    # zoom of the wall slice around the overlap cameras (B src frames 300-900), 5 mm cells
    ov = cen[isB & (fr - off >= 300) & (fr - off <= 900)]
    zlo, zhi = ov[:, :2].min(0) - 1.5, ov[:, :2].max(0) + 1.5
    cams2 = [((c[0], c[1]), (255, 0, 0) if b else (0, 0, 255)) for c, b in zip(cen, isB)]
    z1 = top(X[X[:, 2] < ceil], C[X[:, 2] < ceil], cams2, zlo, zhi, 0.005)
    z2 = top(X, C, cams2, zlo, zhi, 0.005, zsel=(floor + 0.4, floor + 1.2), gray=True)
    cv2.putText(z1, "overlap zoom (131328 5-15 s), 5 mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 200), 2)
    cv2.imwrite(a.out + "_overlap_zoom.png", np.hstack([z1, np.full((z1.shape[0], 10, 3), 0, np.uint8), z2]))
    L = json.loads(Path(a.lens).read_text()); K = np.array(L["K"]); D = np.array(L["dist"])
    kfd = next(M.glob("merged/*_merged_kf"))
    rows = []
    for f in pairs:
        im = cv2.imread(str(kfd / "frames_kf" / f"{f:05d}.jpg"))
        T = np.linalg.inv(P[f]); pc = X @ T[:3, :3].T + T[:3, 3]
        ok = (pc[:, 2] > 0.1) & (pc[:, 2] < 3.0)
        uv, _ = cv2.projectPoints(np.ascontiguousarray(pc[ok], np.float64).reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D.astype(np.float64)); uv = uv[:, 0].astype(int)
        Hh, Ww = im.shape[:2]; r = np.full_like(im, 255); zb = np.full((Hh, Ww), np.inf)
        z = pc[ok, 2]; cc = C[ok][:, ::-1]
        o = np.argsort(-z)
        for du in (0, 1, 2):
            for dv in (0, 1, 2):
                uu, vv = uv[o, 0] + du, uv[o, 1] + dv
                m = (uu >= 0) & (uu < Ww) & (vv >= 0) & (vv < Hh)
                r[vv[m], uu[m]] = cc[o][m]
        lab = f"frame {f} ({'131328' if f >= off else '121801'} kf, src frame {f - off if f >= off else f})"
        cv2.putText(im, lab, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
        cv2.putText(r, "mesh points, same camera", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 4)
        rows.append(cv2.resize(np.hstack([im, r]), None, fx=0.5, fy=0.5))
    cv2.imwrite(a.out + "_kfviews.jpg", np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(json.dumps(dict(top=a.out + "_top.png", kf=pairs, n_vertices=len(X),
                          extent_m=[float(v) for v in hi - lo])))


if __name__ == "__main__":
    main()
