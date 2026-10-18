#!/usr/bin/env python3
"""IMU 보정 CNN 학습 (혼합 방식, IMU_PLAN.md 4) — 녹화들의 imu_gt (imu_gt.py 결과) 로.

  venv_ffs/bin/python scan3d/imu_learn/train_imu.py REC [REC ...] [--epochs 300] [--val-frac 0.3] [--out runs/imu_cnn_x]

구조
- 입력: 1 초 창의 IMU (자이로 3 + 가속도 3, 200 Hz 로 다시 표본, IMU 축).
- 모델: 과거만 보는(causal) 1D CNN, 필터 7, 간격 1·2·4·8·16 (약 1 s 를 봄) → 표본마다 자이로·가속도 보정값 6.
- 물리 적분 (미분 가능): R ← R·exp(ω dt), a = R f + g, v += a dt, p += v dt + ½ a dt²  (정답 시작 상태에서 출발).
- 손실: 1 s 뒤 자세(각도) + 속도 + 위치 오차 (정답 = 보정판).
- 평가: 학습에 안 쓴 구간(녹화마다 뒤쪽 val-frac, 또는 녹화 단위)에서 보정 없음 대비 1 s 적분 오차.
정답 좌표: 보드 좌표. IMU 자세 R_bi = R_bc · R_ci (align.json), 위치 = 카메라 위치 (지렛대 길이 무시, 수 cm 이하).
보드 좌표의 중력 g_b 는 거의 정지한 구간의 가속도로 추정.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from imu_gt import read_imu  # noqa: E402

HZ = 200
WIN = HZ          # 1 s
DT = 1.0 / HZ


# ---------------------------------------------------------------- data
def slerp_R(R0, R1, a):
    w = so3_log_np(R0.T @ R1)
    return R0 @ so3_exp_np(w * a)


def so3_exp_np(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def so3_log_np(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(th)) * th


def load_rec(rec):
    rec = Path(rec)
    gt = np.load(rec / "imu_gt" / "gt.npz")
    al = json.load(open(rec / "imu_gt" / "align.json"))
    R_ci = np.asarray(al["R_ci"])
    acc, gyr = read_imu(rec / "imu.csv")
    dt = al["dt_ns"]
    # IMU 를 카메라 시각으로 (+dt) 옮겨 200 Hz 격자에
    t0 = max(acc[0, 0], gyr[0, 0]) + dt
    t1 = min(acc[-1, 0], gyr[-1, 0]) + dt
    grid = np.arange(t0, t1, int(1e9 / HZ))
    g = np.stack([np.interp(grid, gyr[:, 0] + dt, gyr[:, 1 + j]) for j in range(3)], 1)
    a = np.stack([np.interp(grid, acc[:, 0] + dt, acc[:, 1 + j]) for j in range(3)], 1)
    ok = gt["ok"].astype(bool)
    tf, Rbc, pbc = gt["t_ns"][ok], gt["R_bc"][ok], gt["p_bc"][ok]
    Rbi = Rbc @ R_ci                                        # IMU 축 → 보드
    # 위치를 살짝 매끄럽게 (보드 자세 떨림) 후 속도 = 중앙 차분
    from scipy.ndimage import uniform_filter1d
    ps = uniform_filter1d(pbc, 5, axis=0)
    vf = np.gradient(ps, tf * 1e-9, axis=0)
    # 보드 좌표의 중력: 거의 정지(자이로 작음) 프레임에서 g_b = -R_bi f
    gi = np.stack([np.interp(tf, grid, g[:, j]) for j in range(3)], 1)
    ai = np.stack([np.interp(tf, grid, a[:, j]) for j in range(3)], 1)
    still = (np.linalg.norm(gi, axis=1) < 0.05) & (np.linalg.norm(vf, axis=1) < 0.05)
    if still.sum() < 10:
        still = np.argsort(np.linalg.norm(gi, axis=1))[:30]
    gb = -np.mean(np.einsum("nij,nj->ni", Rbi[still], ai[still]), 0)
    gb = gb / np.linalg.norm(gb) * 9.80665
    return dict(name=rec.name, grid=grid, gyr=g, acc=a, tf=tf, Rbi=Rbi, p=ps, v=vf, gb=gb, n_still=int(np.sum(still)))


def state_at(r, t):
    """정답 프레임 사이 보간 (프레임 간격 > 40 ms 면 None)."""
    k = np.searchsorted(r["tf"], t)
    if k <= 0 or k >= len(r["tf"]):
        return None
    t0, t1 = r["tf"][k - 1], r["tf"][k]
    if t1 - t0 > 40_000_000:
        return None
    a = (t - t0) / (t1 - t0)
    R = slerp_R(r["Rbi"][k - 1], r["Rbi"][k], a)
    p = r["p"][k - 1] * (1 - a) + r["p"][k] * a
    v = r["v"][k - 1] * (1 - a) + r["v"][k] * a
    return R, p, v


def windows(r, stride=20):
    X, S0, S1 = [], [], []
    for i in range(0, len(r["grid"]) - WIN, stride):
        ta, tb = r["grid"][i], r["grid"][i + WIN]
        s0, s1 = state_at(r, ta), state_at(r, tb)
        if s0 is None or s1 is None:
            continue
        X.append(np.c_[r["gyr"][i:i + WIN], r["acc"][i:i + WIN]])
        S0.append(np.r_[s0[0].ravel(), s0[1], s0[2]])
        S1.append(np.r_[s1[0].ravel(), s1[1], s1[2]])
    return np.asarray(X, np.float32), np.asarray(S0, np.float32), np.asarray(S1, np.float32)


# ---------------------------------------------------------------- model + physics
class CausalCNN(nn.Module):
    def __init__(self, ch=64):
        super().__init__()
        self.layers = nn.ModuleList()
        c_in = 6
        for d in (1, 2, 4, 8, 16):
            self.layers.append(nn.Conv1d(c_in, ch, 7, dilation=d))
            c_in = ch
        self.head = nn.Conv1d(ch, 6, 1)
        nn.init.zeros_(self.head.weight)
        nn.init.zeros_(self.head.bias)          # 처음엔 보정 0 = 원래 IMU

    def forward(self, x):                       # x (B, 6, T) 정규화된 IMU
        h = x
        for conv in self.layers:
            pad = (conv.kernel_size[0] - 1) * conv.dilation[0]
            h = torch.relu(conv(nn.functional.pad(h, (pad, 0))))   # 왼쪽(과거)만 채움 → 미래를 안 봄
        return self.head(h)                     # (B, 6, T) 보정값 (자이로 rad/s, 가속도 m/s²) × 스케일


def so3_exp(w):                                  # (B,3) → (B,3,3) Rodrigues
    th = w.norm(dim=1, keepdim=True).clamp_min(1e-9)
    k = w / th
    K = torch.zeros(w.shape[0], 3, 3, device=w.device)
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -k[:, 2], k[:, 1], k[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 0], -k[:, 1], k[:, 0]
    s, c = torch.sin(th)[..., None], torch.cos(th)[..., None]
    return torch.eye(3, device=w.device)[None] + s * K + (1 - c) * K @ K


def integrate(gyr, acc, R, p, v, gb):
    """물리 적분 (미분 가능). gyr, acc (B,T,3) IMU 축; R (B,3,3) IMU→보드; gb (3,) 보드 좌표 중력."""
    for k in range(gyr.shape[1]):
        a_w = (R @ acc[:, k, :, None])[..., 0] + gb
        p = p + v * DT + 0.5 * a_w * DT * DT
        v = v + a_w * DT
        R = R @ so3_exp(gyr[:, k] * DT)
    return R, p, v


def errors(R, p, v, S1):
    R1, p1, v1 = S1[:, :9].view(-1, 3, 3), S1[:, 9:12], S1[:, 12:15]
    dR = R1.transpose(1, 2) @ R
    ang = torch.acos(((dR.diagonal(dim1=1, dim2=2).sum(1) - 1) / 2).clamp(-1 + 1e-6, 1 - 1e-6))
    return ang, (p - p1).norm(dim=1), (v - v1).norm(dim=1)


def run(model, X, S0, S1, gb, mu, sd, scale, train=True):
    x = (X - mu) / sd
    corr = model(x.transpose(1, 2)).transpose(1, 2) * scale if model is not None else 0
    imu = X + corr
    R0, p0, v0 = S0[:, :9].view(-1, 3, 3), S0[:, 9:12], S0[:, 12:15]
    R, p, v = integrate(imu[..., :3], imu[..., 3:], R0, p0, v0, gb)
    return errors(R, p, v, S1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recs", nargs="+")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--val-frac", type=float, default=0.3)
    ap.add_argument("--out", default=str(HERE.parent / "runs" / time.strftime("imu_cnn_%Y%m%d-%H%M%S")))
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    tr, va = [], []
    for rec in a.recs:
        r = load_rec(rec)
        X, S0, S1 = windows(r)
        n = len(X)
        cut = int(n * (1 - a.val_frac))
        gap = WIN // 20                                     # 학습·평가 창이 겹치지 않게 1 s 띄움
        print(f"{r['name']}: 창 {n} (학습 {cut}, 평가 {max(0, n - cut - gap)}), 정지 프레임 {r['n_still']}, "
              f"g_b {np.round(r['gb'], 2)}")
        tr.append((X[:cut], S0[:cut], S1[:cut], r["gb"]))
        va.append((X[cut + gap:], S0[cut + gap:], S1[cut + gap:], r["gb"]))
    T = lambda z: torch.tensor(z, device=dev)
    allX = np.concatenate([t[0] for t in tr])
    mu = T(allX.reshape(-1, 6).mean(0)); sd = T(allX.reshape(-1, 6).std(0) + 1e-6)
    scale = T(np.array([0.02] * 3 + [0.2] * 3, np.float32))   # 보정 크기: 자이로 0.02 rad/s, 가속도 0.2 m/s² 단위
    model = CausalCNN().to(dev)
    opt = torch.optim.Adam(model.parameters(), 1e-3)
    nparam = sum(p.numel() for p in model.parameters())

    def evaluate(m, sets):
        out = []
        with torch.no_grad():
            for X, S0, S1, gb in sets:
                if len(X) == 0:
                    continue
                ang, dp, dv = run(m, T(X), T(S0), T(S1), T(gb.astype(np.float32)), mu, sd, scale)
                out.append(torch.stack([ang, dp, dv], 1).cpu().numpy())
        z = np.concatenate(out)
        return dict(ang_deg=float(np.degrees(np.median(z[:, 0]))), pos_cm=float(np.median(z[:, 1]) * 100),
                    vel_cms=float(np.median(z[:, 2]) * 100), n=len(z))

    base_tr, base_va = evaluate(None, tr), evaluate(None, va)
    print(f"모델 무게 {nparam}개, 장치 {dev}\n보정 없음 (1 s 적분 중앙 오차): 학습 {base_tr}\n                           평가 {base_va}")
    t0 = time.time()
    for ep in range(1, a.epochs + 1):
        model.train()
        tot = 0
        for X, S0, S1, gb in tr:
            idx = torch.randperm(len(X))
            for b in range(0, len(X), 64):
                j = idx[b:b + 64]
                ang, dp, dv = run(model, T(X[j]), T(S0[j]), T(S1[j]), T(gb.astype(np.float32)), mu, sd, scale)
                loss = (ang / 0.01).pow(2).mean() + (dp / 0.05).pow(2).mean() + (dv / 0.05).pow(2).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                tot += loss.item()
        if ep % 25 == 0 or ep == a.epochs:
            model.eval()
            print(f"  ep {ep:4d} 손실 {tot:.1f}  학습 {evaluate(model, tr)}  평가 {evaluate(model, va)}  ({time.time() - t0:.0f} s)")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    torch.save(dict(model=model.state_dict(), mu=mu.cpu(), sd=sd.cpu(), scale=scale.cpu()), out / "model.pt")
    res = dict(recs=[Path(r).name for r in a.recs], base_train=base_tr, base_val=base_va,
               model_train=evaluate(model, tr), model_val=evaluate(model, va), params=nparam, epochs=a.epochs)
    json.dump(res, open(out / "result.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
