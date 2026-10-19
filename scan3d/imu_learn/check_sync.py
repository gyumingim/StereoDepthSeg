#!/usr/bin/env python3
"""카메라–IMU 맞춤을 믿을 수 있는가 (imu_gt 결과 검사).

  venv_ffs/bin/python scan3d/imu_learn/check_sync.py REC [REC ...] [--seg 10]

녹화마다, 그리고 녹화 안의 --seg 초 구간마다 따로:
- 시간차 dt: 정답 |ω| 와 자이로 |ω| 의 평균제곱오차를 ±30 ms 에서 0.1 ms 간격으로 → 최소점 (포물선 보간)
- 회전 R_ci: 같은 순간 ω 벡터들의 Kabsch (움직임 0.3~3 rad/s 만 — 보드 정답이 60 fps 라 빠른 떨림은 못 따라감)
- 잔차: 느린 움직임(|ω| < 1 rad/s)에서 정답 ω 와 회전한 자이로 ω 차이 (rad/s)
그리고 녹화·구간 사이 흩어짐, 공장 렌즈 자세(cameras.json LENS_POSE_ROTATION, 센서 축 → 카메라 축)와의 각도.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import cam_omega, kabsch, read_imu  # noqa: E402


def quat_R(q):                                    # (x, y, z, w) → 3x3
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def factory_R(rec):
    """공장 렌즈 자세 (cameras.json); 고속 녹화 (arlat hs) 에는 없음 → None."""
    if not (rec / "cameras.json").exists():
        return None
    cams = json.load(open(rec / "cameras.json"))
    cam = cams[0] if isinstance(cams, list) else cams
    return next((quat_R(p["pose_rotation"]) for p in cam.get("physical", []) if str(p.get("id")) == "5"), None)


def ang(Ra, Rb):
    if Ra is None or Rb is None:
        return float("nan")
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1))))


def estimate(tc, wc, tg, wg):
    grid = np.arange(tc[0], tc[-1], 2_000_000)
    nc = np.interp(grid, tc, np.linalg.norm(wc, axis=1))
    gn = np.linalg.norm(wg, axis=1)
    shifts = np.arange(-30_000_000, 30_000_001, 100_000)
    e = np.array([np.mean((nc - np.interp(grid, tg + s, gn)) ** 2) for s in shifts])
    k = int(np.argmin(e))
    dt = float(shifts[k])
    if 0 < k < len(e) - 1:                                     # 포물선 꼭짓점
        den = e[k - 1] - 2 * e[k] + e[k + 1]
        if den > 0:
            dt += 0.5 * (e[k - 1] - e[k + 1]) / den * 100_000
    # 곡률 → 대략적 날카로움: 최소에서 ±2 ms 떨어졌을 때 오차가 몇 % 늘어나나
    j = min(len(e) - 1, k + 20)
    sharp = float((e[j] / max(e[k], 1e-12) - 1) * 100)
    wga = np.stack([np.interp(tc, tg + dt, wg[:, i]) for i in range(3)], 1)
    n = np.linalg.norm(wc, axis=1)
    m = (n > 0.3) & (n < 3.0)
    R = kabsch(wga[m], wc[m]) if m.sum() >= 30 else None
    slow = n < 1.0
    res = None
    if R is not None and slow.sum() > 20:
        res = float(np.sqrt(np.mean(np.sum((wc[slow] - (R @ wga[slow].T).T) ** 2, 1))))
    return dict(dt_ms=dt / 1e6, sharp_pct_2ms=sharp, R=R, n=int(m.sum()), slow_res=res)


def so3_log_v(R):                               # (N,3,3) → (N,3)
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], 1)
    s = np.sin(th)
    k = np.where(s > 1e-9, th / (2 * np.maximum(s, 1e-9)), 0.5)
    return v * k[:, None]


def gyro_orientation(tg, wg):
    """자이로 적분 자세 Q(t) (IMU 축, 시작 = 단위). 표본마다 (N,3,3)."""
    Q = np.empty((len(tg), 3, 3))
    Q[0] = np.eye(3)
    for i in range(1, len(tg)):
        w = 0.5 * (wg[i - 1] + wg[i]) * (tg[i] - tg[i - 1]) * 1e-9
        th = np.linalg.norm(w)
        if th < 1e-12:
            Q[i] = Q[i - 1]
            continue
        k = w / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        Q[i] = Q[i - 1] @ (np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K)
    return Q


def increments(gt, n=12):
    """보드 정답의 n 프레임 (60 fps 면 12, 240 fps 면 48 = 0.2 s) 자세 변화 (카메라 축 회전벡터) 와 그 두 시각."""
    t, R, ok = gt["t_ns"], gt["R_bc"], gt["ok"]
    a = np.where(ok)[0]
    a = a[a + n < len(t)]
    step = np.median(np.diff(t[ok])) if ok.sum() > 2 else 16_666_667
    a = a[ok[a + n] & ((t[a + n] - t[a]) < n * 1.5 * step)]
    dR = np.einsum("nji,njk->nik", R[a], R[a + n])            # R_a^T R_b
    return t[a], t[a + n], so3_log_v(dR)


def refine(t0, t1, wc_inc, tg, Q, dt_range=15_000_000, step=200_000):
    """dt·R_ci 를 같이: dt 마다 자이로 적분 증분 (IMU 축) → Kabsch R → 잔차, 최소 dt. 반환 dt(ms), R, 잔차(°/증분)."""
    best = None
    rv_i = None
    for dt in range(-dt_range, dt_range + 1, step):
        i0 = np.clip(np.searchsorted(tg + dt, t0), 1, len(tg) - 1)
        i1 = np.clip(np.searchsorted(tg + dt, t1), 1, len(tg) - 1)
        dRi = np.einsum("nji,njk->nik", Q[i0], Q[i1])
        vi = so3_log_v(dRi)
        m = np.linalg.norm(wc_inc, axis=1) > np.radians(2)
        R = kabsch(vi[m], wc_inc[m])
        r = np.degrees(np.linalg.norm(wc_inc[m] - (R @ vi[m].T).T, axis=1))
        e = float(np.median(r))
        if best is None or e < best[0]:
            best = (e, dt, R)
    return best[1] / 1e6, best[2], best[0]


def main_inc(a):
    print("[정밀] 0.2 s 자세 변화 비교 (보드 증분 vs 자이로 적분 증분), dt·R 같이 맞춤")
    Rs, rows = [], []
    for rec in a.recs:
        rec = Path(rec)
        gt = dict(np.load(rec / "imu_gt" / "gt.npz"))
        _, gyr = read_imu(rec / "imu.csv")
        tg, wg = gyr[:, 0].astype(np.int64), gyr[:, 1:4]
        Q = gyro_orientation(tg, wg)
        fps = float(json.load(open(rec / "meta.json")).get("save_fps") or 60)
        t0, t1, wci = increments(gt, n=int(round(0.2 * fps)))
        dt, R, e = refine(t0, t1, wci, tg, Q)
        fac = factory_R(rec)
        segs = []
        T = t0[0]
        while T + a.seg * 1e9 <= t0[-1]:
            sel = (t0 >= T) & (t0 < T + a.seg * 1e9)
            if sel.sum() > 150:
                segs.append(refine(t0[sel], t1[sel], wci[sel], tg, Q, dt_range=10_000_000, step=200_000))
            T += a.seg * 1e9
        sd = [x[0] for x in segs]
        sa = [ang(x[1], R) for x in segs]
        print(f"== {rec.name}: dt {dt:+.1f} ms, 증분 잔차 중앙 {e:.2f}°, 공장과 {ang(fac, R):.2f}° | "
              f"{a.seg:.0f} s 구간 {len(segs)}: dt {np.round(sd, 1).tolist()} (표준편차 {np.std(sd):.1f} ms), R 차이 중앙 {np.median(sa) if sa else 0:.2f}°")
        Rs.append(R)
        if a.write:                       # imu_gt 의 align.json 을 이 (더 정확한) 값으로 바꿈 — 옛 값은 *_omega 로 남김
            al_p = rec / "imu_gt" / "align.json"
            al = json.load(open(al_p))
            al.setdefault("dt_ns_omega", al["dt_ns"]); al.setdefault("R_ci_omega", al["R_ci"])
            al.update(dt_ns=int(round(dt * 1e6)), R_ci=R.tolist(), method="rotation increments 0.2 s (check_sync.py)",
                      inc_res_deg=e)
            json.dump(al, open(al_p, "w"), indent=1)
        rows.append(dict(rec=rec.name, dt_ms=dt, inc_res_deg=e, factory_deg=ang(fac, R), seg_dt=sd, seg_R_deg=sa))
    if len(Rs) > 1:
        print(f"녹화 사이 R 차이: {[round(ang(Rs[0], R), 2) for R in Rs[1:]]}°, dt {[round(r['dt_ms'], 1) for r in rows]} ms")
    json.dump(rows, open(HERE.parent / "runs" / "imu_sync_check_inc.json", "w"), indent=1, ensure_ascii=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--seg", type=float, default=10.0)
    ap.add_argument("--write", action="store_true", help="정밀 dt·R 로 각 녹화의 imu_gt/align.json 갱신")
    a = ap.parse_args()
    allR, rows = [], []
    for rec in a.recs:
        rec = Path(rec)
        gt = dict(np.load(rec / "imu_gt" / "gt.npz"))
        _, gyr = read_imu(rec / "imu.csv")
        tg, wg = gyr[:, 0].astype(np.int64), gyr[:, 1:4]
        tc, wc = cam_omega(gt)
        fac = factory_R(rec)
        full = estimate(tc, wc, tg, wg)
        segs = []
        t = tc[0]
        while t + a.seg * 1e9 <= tc[-1]:
            sel = (tc >= t) & (tc < t + a.seg * 1e9)
            if sel.sum() > 200:
                r = estimate(tc[sel], wc[sel], tg, wg)
                if r["R"] is not None:
                    segs.append(r)
            t += a.seg * 1e9
        dts = [s["dt_ms"] for s in segs]
        seg_ang = [ang(s["R"], full["R"]) for s in segs]
        print(f"== {rec.name}: 전체 dt {full['dt_ms']:+.2f} ms (±2 ms 떨어지면 오차 +{full['sharp_pct_2ms']:.0f} %), "
              f"느린 구간 잔차 {full['slow_res'] if full['slow_res'] is None else round(full['slow_res'], 3)} rad/s, 표본 {full['n']}")
        if segs:
            print(f"   {a.seg:.0f} s 구간 {len(segs)} 개: dt {np.round(dts, 2).tolist()} ms (표준편차 {np.std(dts):.2f}), "
                  f"R 차이 중앙 {np.median(seg_ang):.2f}° 최대 {max(seg_ang):.2f}°")
        if fac is not None:
            print(f"   공장 렌즈 자세와 차이 {ang(fac, full['R']):.2f}°")
        allR.append(full["R"])
        rows.append(dict(rec=rec.name, dt_ms=full["dt_ms"], seg_dt_std=float(np.std(dts)) if dts else None,
                         seg_R_med_deg=float(np.median(seg_ang)) if segs else None,
                         factory_deg=ang(fac, full["R"]) if fac is not None else None, slow_res=full["slow_res"]))
    if len(allR) > 1:
        Rm = allR[0]
        print(f"녹화 사이 R 차이: {[round(ang(Rm, R), 2) for R in allR[1:]]}° (첫 녹화 기준), "
              f"dt 범위 {min(r['dt_ms'] for r in rows):+.2f} ~ {max(r['dt_ms'] for r in rows):+.2f} ms")
    json.dump(rows, open(HERE.parent / "runs" / "imu_sync_check.json", "w"), indent=1, ensure_ascii=False)
    main_inc(a)


if __name__ == "__main__":
    main()
