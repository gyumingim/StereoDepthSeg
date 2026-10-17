#!/usr/bin/env python3
"""복원 메시의 '벽 두 장' 지표 (읽기 전용): 큰 평면 N 개에서 두께.

복원 폴더의 mesh.ply 정점 (없으면 voxels.ply 점) →
  1. 1 cm 격자로 솎음, RANSAC (안쪽 3 cm) 으로 가장 큰 평면 → 안쪽 점으로 SVD 다시 맞춤.
  2. 판 전체 두께: 평면에서 ±15 cm 띠 안 점의 부호 있는 거리 p95 − p5 (cm). 같은 벽이 두 장이면 두 장 간격만큼 커짐.
  3. 국소 두께: 띠 안 점을 평면 위 15 cm 칸으로 나눠 (칸마다 점 ≥ 20) p95 − p5 의 평균 (cm).
  4. 갈라짐: 칸마다 거리 분포를 두 무리로 (1차원 2-means) 나눠 두 무리 중심 차 ≥ 5 cm 이고 작은 무리 ≥ 20 % 인 칸 비율.
  5. 띠 점을 지우고 다음 평면 (점 ≥ 3000 일 때까지, 최대 --planes).
같은 규칙 · 같은 seed 라 복원끼리 비교만 의미 있음 (tsdf_map.py 주석의 '국소 두께' 와 같은 생각, 입력은 최종 메시).

--revisit (정밀 복원 결과의 kf_points.npz 가 있을 때): '다시 본 곳' 어긋남 = 자세 누적 오차만 보는 지표.
  시간 차 ≥ 3 s 인 키프레임 쌍 (a, b) 에서 a 의 가까운 점 (z ≤ 1.5 m) 마다 b 의 최근접 점 (15 cm 안) 벡터 → 성분별 중앙값
  = 두 번 본 같은 표면 사이의 계통 어긋남 (깊이 잡음은 중앙값에서 상쇄, 자세 표류는 남음). 겹침 30 % 이상 쌍만.
  비교 기준으로 시간 차 0.3~1 s 쌍 (표류 거의 없음) 도 같은 식으로.

  venv_ffs/bin/python scan3d/tools/eval_planes.py <recon 폴더> [<recon 폴더> ...] [--json out.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def load_pts(d):
    d = Path(d)
    if (d / "mesh.ply").is_file():
        import tsdf_map
        V, _, F = tsdf_map.read_mesh_ply(d / "mesh.ply")
        return np.asarray(V, np.float64), "mesh", int(len(V)), int(len(F))
    import open3d as o3d
    pc = o3d.io.read_point_cloud(str(d / "voxels.ply"))
    P = np.asarray(pc.points, np.float64)
    return P, "voxels", int(len(P)), 0


def thin(P, cell=0.01):
    k = np.floor(P / cell).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[np.sort(idx)]


def fit_plane(P, rng, thr=0.03, iters=600):
    n = len(P)
    best, bc = None, -1
    for _ in range(iters):
        a, b, c = P[rng.choice(n, 3, replace=False)]
        nr = np.cross(b - a, c - a)
        ln = np.linalg.norm(nr)
        if ln < 1e-9:
            continue
        nr /= ln
        d = -nr @ a
        cnt = int((np.abs(P @ nr + d) < thr).sum())
        if cnt > bc:
            bc, best = cnt, (nr, d)
    nr, d = best
    for _ in range(3):
        Q = P[np.abs(P @ nr + d) < thr]
        cen = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - cen, full_matrices=False)
        nr = vt[-1]
        d = -nr @ cen
    return nr, d


def two_means(x):
    lo, hi = np.percentile(x, 10), np.percentile(x, 90)
    for _ in range(20):
        m = np.abs(x - lo) <= np.abs(x - hi)
        if m.all() or not m.any():
            return 0.0, 0.0
        lo, hi = x[m].mean(), x[~m].mean()
    m = np.abs(x - lo) <= np.abs(x - hi)
    return abs(hi - lo), min(m.mean(), 1 - m.mean())


def eval_recon(d, planes=4, band=0.15, cell=0.15, seed=0):
    P0, src, nv, nf = load_pts(d)
    P = thin(P0)
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(planes):
        if len(P) < 3000:
            break
        nr, dd = fit_plane(P, rng)
        s = P @ nr + dd
        inb = np.abs(s) <= band
        B, sb = P[inb], s[inb]
        if len(B) < 2000:
            break
        u = np.cross(nr, [1.0, 0, 0] if abs(nr[0]) < 0.9 else [0, 1.0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(nr, u)
        cu, cv = np.floor(B @ u / cell).astype(np.int64), np.floor(B @ v / cell).astype(np.int64)
        key = cu * 1000003 + cv
        loc, split = [], 0
        for k in np.unique(key):
            x = sb[key == k]
            if len(x) < 20:
                continue
            loc.append(np.percentile(x, 95) - np.percentile(x, 5))
            gap, frac = two_means(x)
            if gap >= 0.05 and frac >= 0.2:
                split += 1
        out.append(dict(n=int(len(B)), area_m2=round(len(B) * 1e-4, 2), normal=nr.round(3).tolist(),
                        whole_cm=round(float(np.percentile(sb, 95) - np.percentile(sb, 5)) * 100, 2),
                        rms_cm=round(float(np.sqrt(np.mean(sb ** 2))) * 100, 2),
                        local_cm=round(float(np.mean(loc)) * 100, 2) if loc else None, cells=len(loc),
                        split_frac=round(split / max(1, len(loc)), 3)))
        P = P[~inb]
    def avg(k):
        v = [p[k] for p in out if p.get(k) is not None]
        return round(float(np.mean(v)), 2) if v else None
    return dict(recon=str(d), src=src, vertices=nv, faces=nf, planes=out, whole_cm=avg("whole_cm"), local_cm=avg("local_cm"),
                rms_cm=avg("rms_cm"), split_frac=avg("split_frac"))


def revisit(d, max_pairs=400, seed=0):
    from scipy.spatial import cKDTree
    f = Path(d) / "kf_points.npz"
    if not f.is_file():
        return None
    z = np.load(f)
    t, n, P = z["t"], z["n"], z["pts"]
    off = np.r_[0, np.cumsum(n)]
    K = [P[off[i]:off[i + 1]].astype(np.float64) for i in range(len(t))]
    trees = [cKDTree(k) if len(k) else None for k in K]
    rng = np.random.default_rng(seed)

    def run(lo, hi):
        cand = [(i, j) for i in range(len(t)) for j in range(i + 1, len(t)) if lo <= t[j] - t[i] < hi]
        rng.shuffle(cand)
        res = []
        for i, j in cand:
            if len(res) >= max_pairs:
                break
            a, tb = K[i], trees[j]
            if tb is None or len(a) < 300:
                continue
            dist, idx = tb.query(a, distance_upper_bound=0.15)
            ok = np.isfinite(dist)
            if ok.mean() < 0.3 or ok.sum() < 300:
                continue
            v = K[j][idx[ok]] - a[ok]
            res.append(float(np.linalg.norm(np.median(v, axis=0))))
        r = np.array(res)
        return dict(pairs=len(r), off_med_cm=round(float(np.median(r)) * 100, 2) if len(r) else None,
                    off_p90_cm=round(float(np.percentile(r, 90)) * 100, 2) if len(r) else None)
    return dict(far=run(3.0, 1e9), near=run(0.3, 1.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recons", nargs="+")
    ap.add_argument("--planes", type=int, default=4)
    ap.add_argument("--json", default=None)
    ap.add_argument("--revisit", action="store_true", help="kf_points.npz 로 다시 본 곳 어긋남")
    a = ap.parse_args()
    res = [eval_recon(d, a.planes) for d in a.recons]
    if a.revisit:
        for r in res:
            r["revisit"] = revisit(r["recon"])
    for r in res:
        print(f"{Path(r['recon']).parent.name}/{Path(r['recon']).name}: {r['src']} V={r['vertices']} "
              f"whole={r['whole_cm']} cm local={r['local_cm']} cm rms={r['rms_cm']} cm split={r['split_frac']}"
              + (f" revisit={r['revisit']}" if r.get("revisit") else ""))
        for p in r["planes"]:
            print("   ", p)
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
