"""정답 궤적을 '사진마다 점' 대신 '사진만으로 그린 매끈한 곡선' 으로 (docs/gt_spline_easy.html, traj_compare.html).

IMU 는 쓰지 않음 → 이 정답으로 IMU 를 평가해도 공정. 50 ms 넘게 끊긴 곳에서 구간을 나눔.
축마다 평활 스플라인 (매끈함 정도는 GCV 자동). 곡선에서 크게 벗어난 점 (위치 5 mm / 회전 1° 넘게) 은 튄 점으로 보고 빼고 다시 그림.
회전은 1 s 조각마다 그 조각 가운데 자세 기준 회전벡터로 맞춤: 긴 구간을 구간 시작 기준으로 하면 180° 를 넘어
회전벡터가 뒤집혀 곡선이 깨졌음 (hs_20261019_145816: 회전 차이 6° rms, 2026-10-19).
"""
import numpy as np
from scipy.interpolate import make_smoothing_spline
from scipy.spatial.transform import Rotation as Rot

CHUNK_S = 1.0


def segments(t, gap=0.05, min_len=40):
    cut = np.where(np.diff(t) > gap)[0] + 1
    return [s for s in np.split(np.arange(len(t)), cut) if len(s) >= min_len]


def _fit(t, y):
    return np.stack([make_smoothing_spline(t, y[:, j])(t) for j in range(y.shape[1])], 1)


def smooth_rot(t, R):
    """(N,) s, (N,3,3) → smoothed (N,3,3), error per frame (deg). 1 s chunks, each relative to its middle pose."""
    n = len(t)
    Rs = np.empty_like(R)
    err = np.zeros(n)
    edges = np.searchsorted(t, np.arange(t[0], t[-1] + CHUNK_S, CHUNK_S))
    edges = np.unique(np.r_[edges, n])
    if len(edges) > 2 and edges[-1] - edges[-2] < 20:          # don't leave a tiny last chunk
        edges = np.r_[edges[:-2], n]
    for a, b in zip(edges[:-1], edges[1:]):
        if b - a < 5:
            Rs[a:b] = R[a:b]
            continue
        ref = Rot.from_matrix(R[(a + b) // 2])
        rv = (ref.inv() * Rot.from_matrix(R[a:b])).as_rotvec()
        rf = _fit(t[a:b], rv) if b - a >= 8 else rv
        Rs[a:b] = (ref * Rot.from_rotvec(rf)).as_matrix()
        err[a:b] = np.degrees(np.linalg.norm(rv - rf, axis=1))
    return Rs, err


def smooth_gt(t_ns, R, p, max_mm=5.0, max_deg=1.0):
    """t_ns (N,) 증가, R (N,3,3) 카메라→보드, p (N,3) m → (keep mask, R_s, p_s) — keep=False 는 짧은 구간·튄 점."""
    t = (np.asarray(t_ns) - t_ns[0]) / 1e9
    keep = np.zeros(len(t), bool)
    Rs, ps = R.copy(), p.copy()
    for s in segments(t):
        good = np.ones(len(s), bool)
        for _ in range(3):                                   # fit, drop outliers, refit (until none dropped)
            ss = s[good]
            if len(ss) < 40:
                break
            pf = _fit(t[ss], p[ss])
            rs, er = smooth_rot(t[ss], R[ss])
            ep = np.linalg.norm(p[ss] - pf, axis=1) * 1000
            bad = (ep > max_mm) | (er > max_deg)
            if not bad.any():
                break
            good[np.where(good)[0][bad]] = False
        else:                                                # still dropping after 3 rounds: fit the final set once more
            ss = s[good]
            if len(ss) >= 40:
                pf = _fit(t[ss], p[ss])
                rs, er = smooth_rot(t[ss], R[ss])
        if len(ss) < 40:
            continue
        keep[ss] = True
        ps[ss] = pf
        Rs[ss] = rs
    return keep, Rs, ps
