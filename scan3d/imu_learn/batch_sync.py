#!/usr/bin/env python3
"""카메라↔IMU 시간차 dt · 축 방향 R_ci · 자이로 바이어스를 녹화 전체로 한 번에 맞춤 (docs/batch_easy.html).

check_sync.py (예전) 는 0.2 s 떨어진 정답 사진 두 장의 회전 차이를 자이로와 비교 → 사진 한 장의 흔들림이 비교마다 그대로 들어감.
여기서는 녹화를 2 s 창으로 나누고, 창마다 시작 자세 R0 (정답에서 출발, 조금 움직일 수 있는 보조 값) 에서 자이로를 적분한
자세가 창 안의 정답 사진 '전부' 와 맞도록 dt · R_ci · 바이어스 (20 s 마다 하나, 사이는 직선) 를 같이 푼다 (torch, 미분 가능).
  - 시작 자세도 같이 풀어서 첫 사진의 흔들림이 창 전체에 끼지 않음
  - 손실은 튀는 사진에 덜 끌리는 Cauchy (기준 0.3°), 빨리 돌 때(정답 흐림) 덜 믿음
  - 결과 비교용으로 예전과 같은 '0.2 s 증분 잔차' 도 새 값으로 다시 계산
  ../venv_ffs/bin/python imu_learn/batch_sync.py recordings/hs_A [...] [--write]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402

HZ = 500


def so3_exp(w):                                   # (...,3) → (...,3,3)
    th = w.norm(dim=-1, keepdim=True).clamp_min(1e-12)
    k = w / th
    K = torch.zeros(*w.shape[:-1], 3, 3, dtype=w.dtype, device=w.device)
    K[..., 0, 1], K[..., 0, 2], K[..., 1, 0] = -k[..., 2], k[..., 1], k[..., 2]
    K[..., 1, 2], K[..., 2, 0], K[..., 2, 1] = -k[..., 0], -k[..., 1], k[..., 0]
    s, c = torch.sin(th)[..., None], torch.cos(th)[..., None]
    return torch.eye(3, dtype=w.dtype, device=w.device) + s * K + (1 - c) * K @ K


def ang(Ra, Rb):                                  # angle between rotations (rad)
    tr = (Ra.transpose(-1, -2) @ Rb).diagonal(dim1=-2, dim2=-1).sum(-1)
    return torch.acos(((tr - 1) / 2).clamp(-1 + 1e-7, 1 - 1e-7))


def fit(rec, win=2.0, knot=20.0, iters=400, dev="cuda", log=print):
    rec = Path(rec)
    g = dict(np.load(rec / "imu_gt" / "gt.npz"))
    al = json.load(open(rec / "imu_gt" / "align.json"))
    _, gyr = read_imu(rec / "imu.csv")
    tg = gyr[:, 0].astype(np.int64)
    ok = g["ok"].astype(bool) & (g["t_ns"] > tg[0] + 50_000_000) & (g["t_ns"] < tg[-1] - 50_000_000)
    tf, Rbc = g["t_ns"][ok], g["R_bc"][ok]
    T0 = tg[0]
    # windows of `win` s on camera time, frames inside; integrate gyro on a 500 Hz grid from the window's first frame
    K = int(win * HZ)
    wins = []
    t = tf[0]
    while t + win * 1e9 <= tf[-1]:
        js = np.where((tf >= t) & (tf < t + win * 1e9))[0]
        if len(js) >= 30:
            wins.append(js)
        t += win * 1e9
    F = max(len(j) for j in wins)
    B = len(wins)
    start = np.array([tf[j[0]] for j in wins])                         # camera time of each window's first frame
    idx = np.zeros((B, F), np.float64)                                 # fractional sample index of each frame
    m = np.zeros((B, F))
    Rg = np.tile(np.eye(3), (B, F, 1, 1))
    for b, js in enumerate(wins):
        n = len(js)
        idx[b, :n] = (tf[js] - start[b]) / 1e9 * HZ
        m[b, :n] = 1
        Rg[b, :n] = Rbc[js]
    tt = lambda x, d=torch.float64: torch.tensor(x, dtype=d, device=dev)
    TG, WG = tt((tg - T0) / 1e9), tt(gyr[:, 1:4])
    START, IDX, M, RG = tt((start - T0) / 1e9), tt(idx), tt(m), tt(Rg)
    R0g = RG[:, 0]                                                     # window start from ground truth
    nk = int(np.ceil((tf[-1] - T0) / 1e9 / knot)) + 1
    # parameters
    dt = torch.tensor([al["dt_ns"] / 1e9], dtype=torch.float64, device=dev, requires_grad=True)
    Rci0 = tt(np.asarray(al["R_ci"]))
    dci = torch.zeros(3, dtype=torch.float64, device=dev, requires_grad=True)
    bias = torch.zeros(nk, 3, dtype=torch.float64, device=dev, requires_grad=True)
    eps = torch.zeros(B, 3, dtype=torch.float64, device=dev, requires_grad=True)   # window start-rotation nuisance

    def interp_gyro(ts):                                               # (B,K) seconds → (B,K,3), differentiable in ts
        x = ts.reshape(-1)
        i = torch.searchsorted(TG, x.detach()).clamp(1, len(TG) - 1)
        t0_, t1_ = TG[i - 1], TG[i]
        a = ((x - t0_) / (t1_ - t0_)).clamp(0, 1)[:, None]
        return ((1 - a) * WG[i - 1] + a * WG[i]).reshape(*ts.shape, 3)

    def bias_at(ts):
        u = (ts / knot).clamp(0, nk - 1.000001)
        k0 = u.floor().long()
        f = (u - k0)[..., None]
        return (1 - f) * bias[k0] + f * bias[k0 + 1]

    kgrid = tt(np.arange(K + 1) / HZ)

    def residuals():
        ts = START[:, None] + kgrid[None, :-1] + 0.5 / HZ - dt         # IMU time of each step midpoint
        w = interp_gyro(ts) - bias_at(ts)
        Rci = Rci0 @ so3_exp(dci)
        Q = torch.eye(3, dtype=torch.float64, device=dev).expand(B, 3, 3)
        steps = so3_exp(w / HZ)                                        # (B,K,3,3)
        Qs = [Q]
        for k in range(K):
            Q = Q @ steps[:, k]
            Qs.append(Q)
        Qs = torch.stack(Qs, 1)                                        # (B,K+1,3,3) IMU rotation since window start
        i0 = IDX.floor().long().clamp(0, K - 1)
        fr = (IDX - i0)[..., None]
        bidx = torch.arange(B, device=dev)[:, None]
        # orientation at fractional index: Q[i0] · exp(fr · log(step)) ≈ Q[i0] · exp(fr · w dt)
        Qf = Qs[bidx, i0] @ so3_exp(fr * w[bidx, i0] / HZ)
        R0 = R0g @ so3_exp(eps)
        pred = R0[:, None] @ Rci @ Qf @ Rci.T                           # predicted camera→board rotation
        return ang(pred, RG), Rci, w

    def loss_fn():
        r, _, w = residuals()
        c = np.radians(0.3)
        speed = w.norm(dim=-1).mean(1, keepdim=True)
        lw = M / (1 + (speed / 1.0) ** 2)
        return (torch.log1p((r / c) ** 2) * lw).sum() / lw.sum() + 1e-4 * (eps ** 2).mean() / np.radians(1) ** 2

    # Adam step ≈ lr per parameter: dt 0.01 ms, bias 1e-5 rad/s, rotations 0.03°
    opt = torch.optim.Adam([dict(params=[dt], lr=1e-5), dict(params=[bias], lr=1e-5),
                            dict(params=[dci, eps], lr=5e-4)])
    for it in range(iters):
        opt.zero_grad()
        L = loss_fn()
        L.backward()
        opt.step()
        if it % 100 == 0:
            log(f"    it {it}: loss {L.item():.4f}, dt {dt.item() * 1e3:+.2f} ms")
    with torch.no_grad():
        r, Rci, _ = residuals()
        rr = np.degrees(r[M > 0].cpu().numpy())
    out = dict(dt_ns=int(round(dt.item() * 1e9)), R_ci=Rci.cpu().numpy().tolist(),
               gyro_bias_knots=bias.detach().cpu().numpy().tolist(), knot_s=knot,
               frame_res_deg_med=float(np.median(rr)), frame_res_deg_p90=float(np.percentile(rr, 90)),
               windows=B, frames=int(M.sum().item()))
    log(f"    dt {out['dt_ns'] / 1e6:+.2f} ms, 사진별 자세 잔차 중앙 {out['frame_res_deg_med']:.3f}° (p90 "
        f"{out['frame_res_deg_p90']:.3f}°), 바이어스 {np.round(np.mean(out['gyro_bias_knots'], 0), 4).tolist()} rad/s")
    return out


def increment_check(rec, dt_ns, R_ci, bias_mean):
    """예전 지표: 0.2 s 증분 (보드 vs 자이로 적분) 잔차 중앙 — 같은 잣대로 비교."""
    import check_sync as cs
    rec = Path(rec)
    g = dict(np.load(rec / "imu_gt" / "gt.npz"))
    _, gyr = read_imu(rec / "imu.csv")
    tg, wg = gyr[:, 0].astype(np.int64), gyr[:, 1:4] - np.asarray(bias_mean)
    Q = cs.gyro_orientation(tg, wg)
    fps = float(json.load(open(rec / "meta.json")).get("save_fps") or 60)
    t0, t1, wci = cs.increments(g, n=int(round(0.2 * fps)))
    i0 = np.clip(np.searchsorted(tg + dt_ns, t0), 1, len(tg) - 1)
    i1 = np.clip(np.searchsorted(tg + dt_ns, t1), 1, len(tg) - 1)
    vi = cs.so3_log_v(np.einsum("nji,njk->nik", Q[i0], Q[i1]))
    msk = np.linalg.norm(wci, axis=1) > np.radians(2)
    R = np.asarray(R_ci)
    return float(np.median(np.degrees(np.linalg.norm(wci[msk] - (R @ vi[msk].T).T, axis=1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--win", type=float, default=2.0)
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--write", action="store_true", help="imu_gt/align.json 을 이 값으로 (예전 값은 *_inc 로 남김)")
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    rows = []
    for rec in a.recs:
        rec = Path(rec)
        print(f"== {rec.name}", flush=True)
        al_p = rec / "imu_gt" / "align.json"
        al = json.load(open(al_p))
        old = increment_check(rec, al["dt_ns"], al["R_ci"], [0, 0, 0])
        r = fit(rec, a.win, iters=a.iters, dev=dev, log=lambda s: print(s, flush=True))
        new = increment_check(rec, r["dt_ns"], r["R_ci"], np.mean(r["gyro_bias_knots"], 0))
        print(f"   0.2 s 증분 잔차: 예전 값 {old:.3f}° → 전체 맞춤 {new:.3f}°  (dt {al['dt_ns'] / 1e6:+.2f} → "
              f"{r['dt_ns'] / 1e6:+.2f} ms)", flush=True)
        rows.append(dict(rec=rec.name, inc_old=old, inc_new=new, **{k: r[k] for k in ("dt_ns", "frame_res_deg_med")}))
        if a.write:
            for k in ("dt_ns", "R_ci"):
                al.setdefault(f"{k}_inc", al[k])
            al.update(dt_ns=r["dt_ns"], R_ci=r["R_ci"], gyro_bias_knots=r["gyro_bias_knots"], knot_s=r["knot_s"],
                      method="whole-recording fit (batch_sync.py)", inc_res_deg=new, frame_res_deg=r["frame_res_deg_med"])
            json.dump(al, open(al_p, "w"), indent=1)
    json.dump(rows, open(HERE.parent / "runs" / "imu_batch_sync.json", "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
