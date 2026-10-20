#!/usr/bin/env python3
"""IMU 보정 CNN 학습 (혼합 방식, IMU_PLAN.md 4) — 녹화들의 imu_gt (imu_gt.py 결과) 로.

  venv_ffs/bin/python scan3d/imu_learn/train_imu.py --train REC [REC ...] --val REC [REC ...] [--epochs 200] [--out DIR]

구조
- 입력: 1 초 창의 IMU (자이로 3 + 가속도 3, 폰 원래 비율 500 Hz 격자, IMU 축). 200 Hz 로 솎으면 드론 모터 진동
  (100~300 Hz) 이 가짜 느린 움직임으로 섞여서(에일리어싱) 500 Hz 그대로 씀.
- 모델: 과거만 보는(causal) 1D CNN, 필터 7, 간격 1·2·4·…·64 (7 층, 약 1.5 s 를 봄) → 표본마다 자이로·가속도 보정값 6.
  비교용 'bias' 모델 = 녹화 전체에 상수 보정 6 개만 (단순 바이어스 빼기).
- 물리 적분 (미분 가능): R ← R·exp(ω dt), a = R f + g, v += a dt, p += v dt + ½ a dt².
- 손실 (창 안의 정답 프레임 j 전부, 약 50 개):
  자세: 창 첫 정답 프레임의 정답 자세에서 적분한 R(τ_j) 와 정답 R_j 의 각도.
  위치: 시작 위치·속도는 정답 미분값을 쓰지 않고(흔들 때 부정확) 적분 위치 q(τ_j) + p0 + v0 τ_j 가 정답 p_j 에 가장 잘
        맞도록 p0·v0 를 최소제곱으로 풀고 남은 잔차 → 가속도·자세가 정답 궤적의 '휘어짐' 을 설명하는지만 봄.
- 정답 거르기: 연속 프레임의 정답 회전과 자이로 적분 회전이 3° 넘게 다르면 그 프레임 버림 (보드 자세 뒤집힘·롤링셔터).
정답 좌표: 보드 좌표. IMU 자세 R_bi = R_bc · R_ci (align.json), 위치 = 카메라 위치 (지렛대 길이 무시).
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

HZ = 500          # S24 IMU 원래 비율 (가속도·자이로 약 499 Hz)
WIN = HZ          # 1 s
DT = 1.0 / HZ
MAX_DIST_M = 1.2  # 정답 사진: 보드에서 이보다 멀면 버림 (hs_..151358 시작 0.4 s 가 3.3 m 에서 위치가 수십 cm 씩 튐)
MAXF = 72        # 창 안 정답 프레임 최대 (60 fps × 1 s + 여유), --win 과 녹화 fps (240 fps 고속 녹화) 에 맞춰 늘림


# ---------------------------------------------------------------- data
def so3_exp_np(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def so3_exp_batch(w):
    """(N,3) → (N,3,3) Rodrigues, vectorized (same as so3_exp_np per row)."""
    th = np.linalg.norm(w, axis=1)
    k = np.where(th[:, None] > 1e-12, w / np.maximum(th, 1e-12)[:, None], 0.0)
    K = np.zeros((len(w), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -k[:, 2], k[:, 1], k[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 0], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], np.cos(th)[:, None, None]
    return np.eye(3)[None] + s * K + (1 - c) * K @ K


def ang_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


CACHE_VERSION = 1


def load_rec(rec, max_jump_deg=3.0, smooth=False, gt_dir="imu_gt"):
    """Cached _load_rec: the prepared recording (IMU grid, filtered/smoothed GT) is saved next to the GT and reused while
    its inputs (gt.npz, align.json, trim.json, imu.csv) and the options are unchanged — every training used to redo this
    (≈1 h for 10 recordings, 2026-10-19 night)."""
    import hashlib
    base, part = (str(rec).split("@") + [None])[:2]
    rb = Path(base)
    deps = [rb / gt_dir / "gt.npz", rb / "imu_gt" / "align.json", rb / "imu_gt" / "trim.json", rb / "imu.csv"]
    sig = "|".join(f"{p.name}:{p.stat().st_mtime_ns if p.exists() else 0}" for p in deps)
    key = hashlib.md5(f"{CACHE_VERSION}|{sig}|{part}|{max_jump_deg}|{smooth}|{gt_dir}|{MAX_DIST_M}|{HZ}".encode()).hexdigest()[:12]
    cache = rb / gt_dir / f"train_cache_{key}.npz"
    import os
    if os.environ.get("IMU_CACHE_DIR"):                     # keep GT folders untouched (cache elsewhere)
        Path(os.environ["IMU_CACHE_DIR"]).mkdir(parents=True, exist_ok=True)
        cache = Path(os.environ["IMU_CACHE_DIR"]) / f"{rb.name}_{gt_dir}_{key}.npz"
    if cache.exists():
        z = np.load(cache, allow_pickle=True)
        return {k: (z[k].item() if z[k].ndim == 0 else z[k]) for k in z.files}
    r = _load_rec(rec, max_jump_deg, smooth, gt_dir)
    tmp =cache.with_name(cache.stem + f".tmp{os.getpid()}.npz")    # atomic: parallel runs may build the same cache
    np.savez(tmp, **r)
    os.replace(tmp, cache)
    return r


def _load_rec(rec, max_jump_deg=3.0, smooth=False, gt_dir="imu_gt"):
    """rec = 녹화 폴더, 또는 'folder@a:b' = 그 녹화의 시간 비율 a~b 구간만 (녹화 하나를 학습/평가로 나눌 때)."""
    rec, part = (str(rec).split("@") + [None])[:2]
    rec = Path(rec)
    gt = np.load(rec / gt_dir / "gt.npz")              # gt_dir: e.g. imu_gt_rs (rolling-shutter derotated GT)
    al = json.load(open(rec / "imu_gt" / "align.json"))
    R_ci = np.asarray(al["R_ci"])
    acc, gyr = read_imu(rec / "imu.csv")
    dt = al["dt_ns"]
    t0 = max(acc[0, 0], gyr[0, 0]) + dt
    t1 = min(acc[-1, 0], gyr[-1, 0]) + dt
    trim = rec / "imu_gt" / "trim.json"                     # e.g. board moved on screen at the start (board_shift.py)
    if trim.exists():
        t0 = max(t0, int(json.load(open(trim))["start_ns"]))
    if part:
        fa, fb = (float(v) for v in part.split(":"))
        t0, t1 = int(t0 + fa * (t1 - t0)), int(t0 + fb * (t1 - t0))
    grid = np.arange(t0, t1, int(1e9 / HZ))
    g = np.stack([np.interp(grid, gyr[:, 0] + dt, gyr[:, 1 + j]) for j in range(3)], 1)
    a = np.stack([np.interp(grid, acc[:, 0] + dt, acc[:, 1 + j]) for j in range(3)], 1)
    ok = gt["ok"].astype(bool) & (gt["t_ns"] > t0) & (gt["t_ns"] < t1)
    ok &= np.linalg.norm(gt["p_bc"], axis=1) < MAX_DIST_M     # far board = few px per square = unstable pose
    tf, Rbc, pbc = gt["t_ns"][ok], gt["R_bc"][ok], gt["p_bc"][ok]
    rms = np.nan_to_num(gt["rms_px"][ok], nan=1.5)
    n_raw = len(tf)
    if smooth:                                              # 사진마다 점 → 사진만으로 그린 곡선 (gt_smooth.py, IMU 안 씀)
        from gt_smooth import smooth_gt
        k = np.r_[True, np.diff(tf) > 10_000]
        tf, Rbc, pbc, rms = tf[k], Rbc[k], pbc[k], rms[k]
        keep, Rbc, pbc = smooth_gt(tf, Rbc, pbc)
        tf, Rbc, pbc, rms = tf[keep], Rbc[keep], pbc[keep], rms[keep]
    Rbi = Rbc @ R_ci                                        # IMU 축 → 보드
    # 정답 거르기: 이웃 프레임 사이 회전을 자이로 적분과 비교. 자이로 자세 Q (Q[i] = 표본 0..i-1 의 회전 곱) 를 한 번 만들고
    # 두 프레임 사이 = Q[i0]ᵀ Q[i1] (예전: 프레임마다 표본을 다시 곱하는 Python 이중 반복 — 녹화 읽기 병목, 2026-10-20)
    Q = np.empty((len(grid) + 1, 3, 3))
    Q[0] = np.eye(3)
    E = so3_exp_batch(g * DT)
    for i in range(len(grid)):
        Q[i + 1] = Q[i] @ E[i]
    i0 = np.searchsorted(grid, tf[:-1])
    i1 = np.searchsorted(grid, tf[1:])
    Rg = np.einsum("nji,njk->nik", Q[i0], Q[i1])
    Rc = np.einsum("nji,njk->nik", Rbi[:-1], Rbi[1:])
    D = np.einsum("nji,njk->nik", Rc, Rg)
    ang = np.degrees(np.arccos(np.clip((np.trace(D, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    bad = np.r_[False, (ang > max_jump_deg) & (np.diff(tf) <= 40_000_000)]
    keep = ~bad
    tf, Rbi, pf, rms = tf[keep], Rbi[keep], pbc[keep], rms[keep]
    # 보드 좌표 중력: 거의 정지(자이로 작음) 프레임에서 g_b = -R_bi f
    gi = np.stack([np.interp(tf, grid, g[:, j]) for j in range(3)], 1)
    ai = np.stack([np.interp(tf, grid, a[:, j]) for j in range(3)], 1)
    still = np.where(np.linalg.norm(gi, axis=1) < 0.05)[0]
    if len(still) < 10:
        still = np.argsort(np.linalg.norm(gi, axis=1))[:30]
    gb = -np.mean(np.einsum("nij,nj->ni", Rbi[still], ai[still]), 0)
    gb = gb / np.linalg.norm(gb) * 9.80665
    return dict(name=rec.name + (f"@{part}" if part else ""), grid=grid, gyr=g, acc=a, tf=tf, Rbi=Rbi, p=pf, gb=gb, rms=rms,
                n_still=len(still), n_bad=int(bad.sum()), n_gt=len(tf))


def windows(r, stride=HZ // 4, min_frames=25):  # stride: --stride s
    """겹치는 1 s 창 (0.25 s 씩). 창 안 정답 프레임: 표본 위치 idx, 자세, 위치 (MAXF 로 채움 + mask)."""
    X, IDX, RG, PG, M, Q = [], [], [], [], [], []
    grid, tf = r["grid"], r["tf"]
    for i in range(0, len(grid) - WIN, stride):
        ta, tb = grid[i], grid[i + WIN - 1]
        js = np.where((tf >= ta) & (tf <= tb))[0]
        if len(js) < min_frames:
            continue
        js = js[:MAXF]
        idx = np.round((tf[js] - ta) / (1e9 / HZ)).astype(int)
        n = len(js)
        pad = MAXF - n
        X.append(np.c_[r["gyr"][i:i + WIN], r["acc"][i:i + WIN]])
        IDX.append(np.r_[idx, np.full(pad, idx[-1])])
        RG.append(np.concatenate([r["Rbi"][js], np.repeat(r["Rbi"][js[-1:]], pad, 0)]))
        PG.append(np.concatenate([r["p"][js], np.repeat(r["p"][js[-1:]], pad, 0)]))
        M.append(np.r_[np.ones(n), np.zeros(pad)])
        Q.append(np.r_[r["rms"][js], np.full(pad, 1.0)])
    f = lambda z, t=np.float32: np.asarray(z, t)
    return dict(X=f(X), idx=f(IDX, np.int64), R=f(RG), p=f(PG), m=f(M), q=f(Q), gb=r["gb"].astype(np.float32), name=r["name"])


# ---------------------------------------------------------------- models + physics
class CausalCNN(nn.Module):
    def __init__(self, ch=64):
        super().__init__()
        self.layers = nn.ModuleList()
        c_in = 6
        for d in (1, 2, 4, 8, 16, 32, 64):
            self.layers.append(nn.Conv1d(c_in, ch, 7, dilation=d))
            c_in = ch
        self.head = nn.Conv1d(ch, 6, 1)
        nn.init.zeros_(self.head.weight)
        nn.init.zeros_(self.head.bias)          # 처음엔 보정 0 = 원래 IMU

    def forward(self, x):                       # x (B, 6, T) 정규화된 IMU → (B, 6, T)
        h = x
        for conv in self.layers:
            pad = (conv.kernel_size[0] - 1) * conv.dilation[0]
            h = torch.relu(conv(nn.functional.pad(h, (pad, 0))))   # 왼쪽(과거)만 채움 → 미래를 안 봄
        return self.head(h)


class BiasOnly(nn.Module):
    """비교 기준: 상수 보정 6 개 (자이로·가속도 바이어스 빼기)."""
    def __init__(self):
        super().__init__()
        self.b = nn.Parameter(torch.zeros(6))

    def forward(self, x):
        return self.b[None, :, None].expand(x.shape[0], 6, x.shape[2])


class Calib(nn.Module):
    """물리 보정 (폰 하나에 공통): 자이로·가속도 각각 3x3 행렬(축척·축 어긋남, 처음 = 단위) + 바이어스, 그리고
    IMU → 카메라 레버암 r (IMU 축, m). 보드 정답은 카메라 위치라 돌기만 해도 IMU 위치와 R·r 만큼 다름."""
    def __init__(self):
        super().__init__()
        self.G = nn.Parameter(torch.eye(3)); self.bg = nn.Parameter(torch.zeros(3))
        self.A = nn.Parameter(torch.eye(3)); self.ba = nn.Parameter(torch.zeros(3))
        self.r = nn.Parameter(torch.zeros(3))

    def forward(self, X):                       # (B,T,6) → (B,T,6)
        g = X[..., :3] @ self.G.T + self.bg
        a = X[..., 3:] @ self.A.T + self.ba
        return torch.cat([g, a], -1)

    def reg(self):                              # 단위 행렬에서 멀어지는 것에 작은 벌점 (정답 잡음을 외우지 않게)
        I = torch.eye(3, device=self.G.device)
        return ((self.G - I) ** 2).sum() / 0.01 ** 2 + ((self.A - I) ** 2).sum() / 0.02 ** 2


class Stack(nn.Module):
    def __init__(self, cal, net):
        super().__init__()
        self.cal, self.net = cal, net


def so3_exp(w):                                  # (B,3) → (B,3,3)
    th = w.norm(dim=1, keepdim=True).clamp_min(1e-9)
    k = w / th
    K = torch.zeros(w.shape[0], 3, 3, device=w.device)
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -k[:, 2], k[:, 1], k[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 0], -k[:, 1], k[:, 0]
    s, c = torch.sin(th)[..., None], torch.cos(th)[..., None]
    return torch.eye(3, device=w.device)[None] + s * K + (1 - c) * K @ K


def prefix_prod(E):
    """(B,T,3,3) → inclusive prefix products C_k = E_0 E_1 … E_k, in log2(T) batched steps (Hillis–Steele scan).
    Matrix product is associative, so this equals the sequential chain; it replaces 1,500 tiny GPU launches per window
    with ~11 big ones (the training bottleneck, 2026-10-19 night: ~8 h of 10 h)."""
    C = E
    d = 1
    T = E.shape[1]
    while d < T:
        C = torch.cat([C[:, :d], C[:, :-d] @ C[:, d:]], 1)
        d *= 2
    return C


def integrate_all(gyr, acc, R0, gb, start):
    """창 첫 정답 프레임(표본 start)에서 R0, p=v=0 으로 출발해 모든 표본의 R, p (B,T,·). start 이전은 R0, 0.
    Vectorized: R_k = R0 · P_startᵀ · P_k with P_k = E_0…E_{k−1} (prefix products), a_k = R_k f_k + g (zero before start),
    v_k = Σ_{j<k} a_j dt, p_k = Σ_{j<k} (v_j dt + ½ a_j dt²) — identical to _integrate_all_loop (checked)."""
    B, T, _ = gyr.shape
    E = so3_exp((gyr * DT).reshape(-1, 3)).reshape(B, T, 3, 3)
    C = prefix_prod(E)                                          # C_k = E_0 … E_k
    I = torch.eye(3, device=gyr.device).expand(B, 1, 3, 3)
    P = torch.cat([I, C[:, :-1]], 1)                            # P_k = E_0 … E_{k−1}
    b = torch.arange(B, device=gyr.device)
    Ps = P[b, start]                                            # (B,3,3)
    Rk = R0[:, None] @ Ps.transpose(-1, -2)[:, None] @ P        # valid for k ≥ start
    k = torch.arange(T, device=gyr.device)[None]
    on = (k >= start[:, None]).float()                          # (B,T)
    Rs = torch.where(on[..., None, None] > 0, Rk, R0[:, None].expand(B, T, 3, 3))
    a_w = ((Rs @ acc[..., None])[..., 0] + gb) * on[..., None]
    v = torch.cumsum(a_w, 1) * DT
    v = torch.cat([torch.zeros_like(v[:, :1]), v[:, :-1]], 1)    # v_k uses a_j for j < k
    dp = v * DT + 0.5 * a_w * DT * DT
    p = torch.cumsum(dp, 1)
    p = torch.cat([torch.zeros_like(p[:, :1]), p[:, :-1]], 1)
    return Rs, p


def _integrate_all_loop(gyr, acc, R0, gb, start):
    """Reference (old, sequential) version of integrate_all — kept for the equivalence check."""
    B, T, _ = gyr.shape
    Rs, ps = [], []
    R = R0
    p = torch.zeros(B, 3, device=gyr.device)
    v = torch.zeros(B, 3, device=gyr.device)
    for k in range(T):
        on = (k >= start).float()[:, None]
        Rs.append(R)
        ps.append(p)
        a_w = (R @ acc[:, k, :, None])[..., 0] + gb
        p = p + on * (v * DT + 0.5 * a_w * DT * DT)
        v = v + on * a_w * DT
        R = torch.where(on[..., None] > 0, R @ so3_exp(gyr[:, k] * DT), R)
    return torch.stack(Rs, 1), torch.stack(ps, 1)


def window_loss(imu, W, gb, lever=None):
    """imu (B,T,6) 보정된 IMU; W: idx (B,F), R (B,F,3,3), p (B,F,3), m (B,F). → 자세 각 (B,F), 위치 잔차 (B,F)."""
    idx, Rg, pg, m = W["idx"], W["R"], W["p"], W["m"]
    start = idx[:, 0]
    Rall, pall = integrate_all(imu[..., :3], imu[..., 3:], Rg[:, 0], gb, start)
    b = torch.arange(idx.shape[0], device=idx.device)[:, None]
    Ri, qi = Rall[b, idx], pall[b, idx]                       # (B,F,3,3), (B,F,3)
    if lever is not None:                                     # 카메라 위치 = IMU 위치 + R · r
        qi = qi + (Ri @ lever[:, None])[..., 0]
    dR = Rg.transpose(-1, -2) @ Ri
    ang = torch.acos(((dR.diagonal(dim1=-2, dim2=-1).sum(-1) - 1) / 2).clamp(-1 + 1e-6, 1 - 1e-6))
    # p_j ≈ q_j + p0 + v0 τ_j : 축마다 [1, τ] 최소제곱 (mask 가중)
    tau = (idx - start[:, None]).float() * DT
    y = pg - qi
    w = m
    sw = w.sum(1, keepdim=True)
    mt = (w * tau).sum(1, keepdim=True) / sw
    my = (w[..., None] * y).sum(1, keepdim=True) / sw[..., None]
    vt = (w * (tau - mt) ** 2).sum(1, keepdim=True).clamp_min(1e-9)
    v0 = (w[..., None] * (tau - mt)[..., None] * (y - my)).sum(1, keepdim=True) / vt[..., None]
    res = y - my - v0 * (tau - mt)[..., None]
    return ang, res.norm(dim=-1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train", nargs="+", required=True)
    ap.add_argument("--val", nargs="+", required=True)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--batch", type=int, default=192)
    ap.add_argument("--win", type=float, default=1.0, help="창 길이 s (IMU 표류가 보드 정답 잡음보다 커지게 길게)")
    ap.add_argument("--eval-only", action="store_true", help="보정 없음·바이어스만 평가 (CNN 학습 안 함)")
    ap.add_argument("--skip-bias", action="store_true", help="바이어스만 단계 건너뜀 (물리 보정값만 필요할 때)")
    ap.add_argument("--rot-sigma", type=float, default=0.005, help="자세 손실 기준 rad (보드 정답 잡음이 크면 키움)")
    ap.add_argument("--gt-dir", default="imu_gt", help="정답 폴더 (imu_gt_rs = 롤링셔터 펴기). align·trim 은 늘 imu_gt")
    ap.add_argument("--stride", type=float, default=0.25, help="창 시작 간격 s (데이터가 많으면 늘려 학습 시간 줄임)")
    ap.add_argument("--gt-smooth", action="store_true", help="정답을 사진만으로 그린 매끈한 곡선으로 (gt_smooth.py)")
    ap.add_argument("--calib-json", default=None,
                    help="학습 안 함: 이 result.json 의 물리 보정값을 --val 에서 평가만 (다른 정답 잣대로 같은 보정을 채점)")
    ap.add_argument("--out", default=str(HERE.parent / "runs" / time.strftime("imu_cnn_%Y%m%d-%H%M%S")))
    ap.add_argument("--seed", type=int, default=None, help="난수 고정 (중력 맞춤·학습 순서) — 평가끼리 비교할 때")
    a = ap.parse_args()
    if a.seed is not None:
        torch.manual_seed(a.seed)
        np.random.seed(a.seed)
    global WIN, MAXF
    WIN = int(round(a.win * HZ))
    fps = max(float(json.load(open(Path(r.split("@")[0]) / "meta.json")).get("save_fps") or 60) for r in a.train + a.val)
    MAXF = int(fps * 1.2 * a.win)
    print(f"창 {a.win} s, 정답 프레임 최대 {MAXF} (최대 {fps:.0f} fps)", flush=True)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    import os
    if dev == "cuda" and os.environ.get("IMU_VRAM_GB"):     # several runs share the GPU: cap each (total under 15.5 GB)
        tot = torch.cuda.get_device_properties(0).total_memory / 2**30
        torch.cuda.set_per_process_memory_fraction(min(1.0, float(os.environ["IMU_VRAM_GB"]) / tot))

    def prep(recs):
        sets = []
        for rec in recs:
            r = load_rec(rec, smooth=a.gt_smooth, gt_dir=a.gt_dir)
            W = windows(r, stride=max(1, int(a.stride * HZ)))
            print(f"{r['name']}: 정답 프레임 {r['n_gt']} (버림 {r['n_bad']}), 창 {len(W['X'])}, 정지 {r['n_still']}, "
                  f"g_b {np.round(r['gb'], 2)}", flush=True)
            if len(W["X"]):
                sets.append({k: (torch.tensor(v, device=dev) if isinstance(v, np.ndarray) else v) for k, v in W.items()})
        return sets

    tr, va = prep(a.train), prep(a.val)

    def fit_gravity(sets, iters=60):
        """보드 좌표의 중력 방향을 녹화마다 원래 IMU 로 맞춤 (보정 모델 없이, 모든 방식에 같은 값).
        정지 프레임 몇 개로 잡은 g_b 가 0.5 m/s² 넘게 틀린 녹화가 있어(221015: z -0.82 vs 다른 녹화 +0.3~0.5), CNN 이
        그 틀림을 가속도 보정으로 외워 평가 녹화에서 오히려 나빠짐 (2026-10-18 밤 1 차 학습) → 방향만 먼저 맞춘다."""
        for s in sets:
            g = s["gb"].clone().requires_grad_(True)
            opt = torch.optim.Adam([g], 0.02)
            for it in range(iters):
                j = torch.randperm(len(s["X"]), device=dev)[:256]
                W = {k: s[k][j] for k in ("idx", "R", "p", "m")}
                gn = g / g.norm() * 9.80665
                ang, res = window_loss(s["X"][j], W, gn)
                loss = ((res / 0.01) ** 2 * W["m"]).sum() / W["m"].sum()
                opt.zero_grad(); loss.backward(); opt.step()
            with torch.no_grad():
                old = s["gb"].cpu().numpy()
                s["gb"] = (g / g.norm() * 9.80665).detach()
            print(f"  중력 맞춤 {s['name']}: {np.round(old, 2)} → {np.round(s['gb'].cpu().numpy(), 2)}", flush=True)

    fit_gravity(tr + va)
    allX = torch.cat([s["X"] for s in tr]).reshape(-1, 6)
    mu, sd = allX.mean(0), allX.std(0) + 1e-6
    scale = torch.tensor([0.02] * 3 + [0.2] * 3, device=dev)

    def apply(model, X):
        if model is None:
            return X
        if isinstance(model, Calib):
            return model(X)
        if isinstance(model, Stack):                  # 물리 보정 뒤 CNN
            X1 = model.cal(X)
            return X1 + model.net(((X1 - mu) / sd).transpose(1, 2)).transpose(1, 2) * scale
        return X + model(((X - mu) / sd).transpose(1, 2)).transpose(1, 2) * scale

    def lever_of(model):
        if isinstance(model, Calib):
            return model.r
        if isinstance(model, Stack):
            return model.cal.r
        return None

    def evaluate(model, sets):
        A, P = [], []
        with torch.no_grad():
            for s in sets:
                for b in range(0, len(s["X"]), 64):
                    W = {k: s[k][b:b + 64] for k in ("idx", "R", "p", "m")}
                    ang, res = window_loss(apply(model, s["X"][b:b + 64]), W, s["gb"], lever_of(model))
                    m = W["m"] > 0
                    last = W["m"].sum(1).long() - 1
                    A.append(ang[torch.arange(len(last)), last].cpu().numpy())
                    P.append((torch.sqrt((res ** 2 * W["m"]).sum(1) / W["m"].sum(1))).cpu().numpy())
        A, P = np.concatenate(A), np.concatenate(P)
        return dict(rot_end_deg=round(float(np.degrees(np.median(A))), 3), pos_rms_cm=round(float(np.median(P) * 100), 2),
                    rot_end_p90=round(float(np.degrees(np.percentile(A, 90))), 3), n=len(A))

    def train(model, epochs, lr):
        opt = torch.optim.Adam(model.parameters(), lr)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
        t0 = time.time()
        for ep in range(1, epochs + 1):
            model.train()
            tot = 0.0
            for s in tr:
                perm = torch.randperm(len(s["X"]), device=dev)
                for b in range(0, len(perm), a.batch):
                    j = perm[b:b + a.batch]
                    W = {k: s[k][j] for k in ("idx", "R", "p", "m", "q")}
                    ang, res = window_loss(apply(model, s["X"][j]), W, s["gb"], lever_of(model))
                    # 빠르게 돌 때 보정판 정답 자체가 1~2° 흔들림(롤링셔터·번짐) → 그 프레임은 덜 믿음
                    wn = s["X"][j][..., :3].norm(dim=-1).gather(1, W["idx"])
                    # + 재투영 오차가 큰 사진 (보통 0.7 px) 도 덜 믿음
                    m = W["m"] / (1 + (wn / 1.0) ** 2) / (1 + (W["q"] / 1.0) ** 2)
                    loss = (((ang / a.rot_sigma) ** 2 + (res / 0.01) ** 2) * m).sum() / m.sum()
                    if isinstance(model, Calib):
                        loss = loss + 1e-3 * model.reg()
                    opt.zero_grad()
                    loss.backward()
                    nn.utils.clip_grad_norm_(model.parameters(), 10.0)
                    opt.step()
                    tot += loss.item()
            sched.step()
            if ep == 1:                                 # early time estimate so a watchdog sees slow runs at once
                dt1 = time.time() - t0
                print(f"  ep 1 took {dt1:.1f} s → this stage ≈ {dt1 * epochs / 60:.1f} min", flush=True)
            if ep % max(1, epochs // 8) == 0 or ep == epochs:
                model.eval()
                print(f"  ep {ep:4d} 손실 {tot:.1f}  학습 {evaluate(model, tr)}  평가 {evaluate(model, va)}  "
                      f"({time.time() - t0:.0f} s)", flush=True)
        return model

    res = dict(train=[s["name"] for s in tr], val=[s["name"] for s in va])
    if a.calib_json:                                # evaluate a saved physical calibration only (no training)
        cj = json.load(open(a.calib_json))["calib"]
        cal = Calib().to(dev)
        with torch.no_grad():
            for k in ("G", "bg", "A", "ba", "r"):
                getattr(cal, k).copy_(torch.tensor(cj[k], device=dev))
        res["calib_from"] = a.calib_json
        res["none"] = dict(val=evaluate(None, va))
        res["calib"] = dict(val=evaluate(cal, va))
        print("보정 없음:", res["none"], "\n물리 보정 (불러옴):", res["calib"], flush=True)
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        json.dump(res, open(out / "result.json", "w"), indent=1, ensure_ascii=False)
        print(f"→ {out}")
        return
    res["none"] = dict(train=evaluate(None, tr), val=evaluate(None, va))
    print("보정 없음:", res["none"], flush=True)
    if not a.skip_bias:
        bias = train(BiasOnly().to(dev), max(20, a.epochs // 4), 3e-3)
        res["bias"] = dict(train=evaluate(bias, tr), val=evaluate(bias, va), b=(bias.b.detach() * scale).cpu().tolist())
        print("바이어스만:", res["bias"], flush=True)
    cal = train(Calib().to(dev), max(20, a.epochs // 3), 3e-3)
    res["calib"] = dict(train=evaluate(cal, tr), val=evaluate(cal, va),
                        G=cal.G.detach().cpu().tolist(), bg=cal.bg.detach().cpu().tolist(),
                        A=cal.A.detach().cpu().tolist(), ba=cal.ba.detach().cpu().tolist(), r=cal.r.detach().cpu().tolist())
    print("물리 보정 (축척·축·바이어스·레버암):", {k: res["calib"][k] for k in ("train", "val", "r")}, flush=True)
    if a.eval_only:                                 # still save: the physical calibration is used by vio_bench_hs.py
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        json.dump(res, open(out / "result.json", "w"), indent=1, ensure_ascii=False)
        print(f"→ {out}")
        return
    for prm in cal.parameters():
        prm.requires_grad_(False)
    stk = train(Stack(cal, CausalCNN()).to(dev), a.epochs, 1e-3)
    res["calib_cnn"] = dict(train=evaluate(stk, tr), val=evaluate(stk, va))
    print("물리 보정 + CNN:", res["calib_cnn"], flush=True)
    cnn = train(CausalCNN().to(dev), a.epochs, 1e-3)
    res["cnn"] = dict(train=evaluate(cnn, tr), val=evaluate(cnn, va), params=sum(p.numel() for p in cnn.parameters()))
    print("CNN:", res["cnn"], flush=True)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    torch.save(dict(model=cnn.state_dict(), mu=mu.cpu(), sd=sd.cpu(), scale=scale.cpu(), hz=HZ), out / "model.pt")
    json.dump(res, open(out / "result.json", "w"), indent=1, ensure_ascii=False)
    print(json.dumps(res, ensure_ascii=False, indent=1), f"\n→ {out}")


if __name__ == "__main__":
    main()
