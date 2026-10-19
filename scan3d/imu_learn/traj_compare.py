"""실제 녹화의 정답 궤적: 사진마다 따로 찍은 점 (PnP) vs 매끈한 곡선 하나 → imu_gt/traj_compare.json (docs/traj_compare.html).

곡선 = 사진만으로 그린 평활 스플라인 (축마다, 매끈함 정도는 GCV 로 자동). 회전은 구간 첫 자세 기준 회전벡터를 같은 방법으로.
IMU 를 같이 맞추는 '진짜' 곡선 (Kalibr 방식) 은 아직 아님 — 이건 "점들이 얼마나 흔들리나" 를 보는 간이판.
50 ms 넘게 정답이 끊기면 구간을 나눔 (끊긴 곳을 곡선으로 잇지 않음).
  python imu_learn/traj_compare.py recordings/hs_...
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import make_smoothing_spline
from scipy.spatial.transform import Rotation as Rot


def segments(t, gap=0.05, min_len=40):
    cut = np.where(np.diff(t) > gap)[0] + 1
    return [s for s in np.split(np.arange(len(t)), cut) if len(s) >= min_len]


def smooth(t, y):
    out = np.empty_like(y)
    for j in range(y.shape[1]):
        out[:, j] = make_smoothing_spline(t, y[:, j])(t)
    return out


def main(rec):
    rec = Path(rec)
    g = dict(np.load(rec / "imu_gt" / "gt.npz"))
    ok = g["ok"].astype(bool) & (g["t_ns"] > 0)
    t = (g["t_ns"][ok] - g["t_ns"][ok][0]) / 1e9
    p = g["p_bc"][ok]
    R = g["R_bc"][ok]
    keep = np.r_[True, np.diff(t) > 1e-5]                    # strictly increasing for the spline
    t, p, R = t[keep], p[keep], R[keep]
    ps, rv_pt, rv_sm, seg_id = np.full_like(p, np.nan), np.zeros_like(p), np.zeros_like(p), np.zeros(len(t), int)
    for k, s in enumerate(segments(t)):
        ps[s] = smooth(t[s], p[s])
        ref = Rot.from_matrix(R[s[0]])
        rv = (ref.inv() * Rot.from_matrix(R[s])).as_rotvec()       # small rotations from the segment start
        rv_pt[s], rv_sm[s] = rv, smooth(t[s], rv)
        seg_id[s] = k + 1
    m = seg_id > 0
    dp = (p[m] - ps[m]) * 1000
    dr = np.degrees(np.linalg.norm(rv_pt[m] - rv_sm[m], axis=1))
    speed = np.r_[0, np.linalg.norm(np.diff(ps[m], axis=0), axis=1) / np.maximum(np.diff(t[m]), 1e-6)]
    stats = dict(frames=int(m.sum()), segments=int(seg_id.max()), seconds=float(t[-1]),
                 jitter_mm_rms=[round(float(v), 3) for v in np.sqrt(np.mean(dp ** 2, 0))],
                 jitter_mm_total=round(float(np.sqrt(np.mean(np.sum(dp ** 2, 1)))), 3),
                 jitter_deg_rms=round(float(np.sqrt(np.mean(dr ** 2))), 4),
                 jitter_deg_med=round(float(np.median(dr)), 4))
    q = lambda a, s=1e4: (np.round(a * s) / s).tolist()
    out = dict(rec=rec.name, stats=stats, t=q(t[m], 1e5), p=q(p[m]), ps=q(ps[m]),
               r=q(np.degrees(rv_pt[m]), 1e3), rs=q(np.degrees(rv_sm[m]), 1e3), seg=seg_id[m].tolist(),
               speed=q(speed, 1e3))
    (rec / "imu_gt" / "traj_compare.json").write_text(json.dumps(out, separators=(",", ":")))
    print(f"{rec.name}: 사진 {stats['frames']}, 구간 {stats['segments']}, 점-곡선 차이 위치 {stats['jitter_mm_total']} mm "
          f"(x,y,z {stats['jitter_mm_rms']}), 회전 {stats['jitter_deg_rms']}° rms")


if __name__ == "__main__":
    for r in sys.argv[1:]:
        main(r)
