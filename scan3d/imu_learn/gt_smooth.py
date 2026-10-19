"""정답 궤적을 '사진마다 점' 대신 '사진만으로 그린 매끈한 곡선' 으로 (docs/gt_spline_easy.html, traj_compare.html).

IMU 는 쓰지 않음 → 이 정답으로 IMU 를 평가해도 공정. 50 ms 넘게 끊긴 곳에서 구간을 나눔.
축마다 평활 스플라인 (매끈함 정도는 GCV 자동). 곡선에서 크게 벗어난 점 (위치 5 mm / 회전 1° 넘게) 은 튄 점으로 보고 빼고 다시 그림.
"""
import numpy as np
from scipy.interpolate import make_smoothing_spline
from scipy.spatial.transform import Rotation as Rot


def segments(t, gap=0.05, min_len=40):
    cut = np.where(np.diff(t) > gap)[0] + 1
    return [s for s in np.split(np.arange(len(t)), cut) if len(s) >= min_len]


def _fit(t, y):
    return np.stack([make_smoothing_spline(t, y[:, j])(t) for j in range(y.shape[1])], 1)


def smooth_gt(t_ns, R, p, max_mm=5.0, max_deg=1.0):
    """t_ns (N,) 증가, R (N,3,3) 카메라→보드, p (N,3) m → (keep mask, R_s, p_s) — keep=False 는 짧은 구간·튄 점."""
    t = (np.asarray(t_ns) - t_ns[0]) / 1e9
    keep = np.zeros(len(t), bool)
    Rs, ps = R.copy(), p.copy()
    for s in segments(t):
        good = np.ones(len(s), bool)
        for _ in range(2):                                   # fit, drop outliers, refit
            ss = s[good]
            if len(ss) < 40:
                good[:] = False
                break
            ref = Rot.from_matrix(R[ss[0]])
            rv = (ref.inv() * Rot.from_matrix(R[ss])).as_rotvec()
            pf, rf = _fit(t[ss], p[ss]), _fit(t[ss], rv)
            ep = np.linalg.norm(p[ss] - pf, axis=1) * 1000
            er = np.degrees(np.linalg.norm(rv - rf, axis=1))
            bad = (ep > max_mm) | (er > max_deg)
            if not bad.any():
                break
            idx = np.where(good)[0]
            good[idx[bad]] = False
        ss = s[good]
        if len(ss) < 40:
            continue
        keep[ss] = True
        ps[ss] = pf
        Rs[ss] = (ref * Rot.from_rotvec(rf)).as_matrix()
    return keep, Rs, ps
