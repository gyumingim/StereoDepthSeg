"""키프레임 깊이 다듬기 (TSDF 앞단): 두 렌즈 쌍 깊이 합치기 · 여러 키프레임 일관성 거르기 · 잔차로 노이즈 실측.

왜: 위치(VIO·SfM)는 충분히 정확해졌고 남은 병목은 깊이 노이즈 (2+5 기선 15.5 mm, σ_Z = σ_d·Z²/(f·B)).

1. 두 쌍 합치기 (fuse_pairs) — 3 렌즈 녹화 (2·5·6): 같은 렌즈 2 사진으로 2+5 깊이와 2+6 깊이 (망원 화각 = 가운데).
   두 정류는 같은 렌즈 2 광학 중심에서 회전만 다르다 → 2+6 깊이를 2+5 정류 격자로 옮기고 (회전 격자, 최근접)
   겹치는 곳은 역분산 가중 (σ ∝ Z²/(f·B), f = FFS 해상도), 두 값이 k·σ 넘게 다르면 버림. 화소별 σ 를 함께 돌려줌
   → tsdf_map.integrate(sigma=...).
2. 여러 키프레임 일관성 (ConsistencyFilter): 현재 키프레임 깊이 점을 이웃 키프레임 (실시간 = 최근 것, 오프라인 = 앞뒤) 카메라로
   옮겨 그 이웃의 깊이와 비교. r = Z_이웃 − z_투영:
     |r| ≤ k·σ  → 맞음,   r > k·σ → 이웃이 그 점 뒤를 봄 (빈 공간에 뜬 점) → 어긋남,   r < −k·σ → 가려짐 (정보 없음).
   다운샘플 격자 (ds) 에서 세고 원래 해상도로 키움. mode: drop = 어긋남만 있고 맞음 0 이면 버림,
   weight = 가중치 배수 (맞음+0.5)/(본 수+0.5) (안 본 화소 1). 결과는 valid 마스크·가중치 배수 (tsdf_map.integrate(wmul=)).
3. 노이즈 실측 (sigma_px_estimate): 위 비교 잔차를 시차 단위 (FFS 해상도 px) 로 바꿔 강건 표준편차 (MAD) / √2
   = 한 깊이 지도의 실제 σ_d (자세 오차 포함) → tsdf_map sigma_px 와 비교.
"""
import numpy as np

import cv2


# ---------------------------------------------------------------- 공통
def f_eff_B(rp, ffs_scale):
    """시차 노이즈 기준 f·B (FFS 입력 해상도 화소 · m)."""
    return float(rp["f"]) * float(ffs_scale or 1.0) * float(rp["baseline_m"])


def sigma_map(Z, valid, fB, sigma_px):
    """σ_Z = σ_d · Z² / (f·B), 무효 NaN."""
    with np.errstate(invalid="ignore"):
        s = (float(sigma_px) * Z * Z / float(fB)).astype(np.float32)
    return np.where(valid & np.isfinite(Z), s, np.nan).astype(np.float32)


def warp_same_center(Z_src, V_src, rp_src, rp_dst, extra=()):
    """같은 렌즈 2 사진의 두 정류 (회전·내부값만 다름) 사이 깊이 옮기기 → (Z_dst NaN 무효, V_dst, extra).
    p_src = R1_src · p_lens2, p_dst = R1_dst · p_lens2 → p_src = R1_src R1_dstᵀ p_dst."""
    import motion_stereo
    R = np.asarray(rp_src["R1"], float) @ np.asarray(rp_dst["R1"], float).T
    Z, V, ex = motion_stereo.warp_depth(Z_src, V_src, motion_stereo.rect0_K(rp_src), R, motion_stereo.rect0_K(rp_dst),
                                        tuple(int(x) for x in rp_dst["size"]), extra=extra)
    return Z, V, ex


def fuse_pairs(Za, Va, Sa, Zb, Vb, Sb, k=3.0):
    """같은 격자의 두 깊이 (σ 지도 포함) → (Z, V, S, stats). 겹침 = 역분산 가중, |Za−Zb| > k·√(Sa²+Sb²) 이면 버림."""
    Va = Va & np.isfinite(Za) & np.isfinite(Sa)
    Vb = Vb & np.isfinite(Zb) & np.isfinite(Sb)
    both = Va & Vb
    with np.errstate(invalid="ignore", divide="ignore"):
        d = Za - Zb
        s = np.sqrt(Sa * Sa + Sb * Sb)
        agree = both & (np.abs(d) <= k * s)
        wa, wb = 1.0 / (Sa * Sa), 1.0 / (Sb * Sb)
        Zf = (Za * wa + Zb * wb) / (wa + wb)
        Sf = 1.0 / np.sqrt(wa + wb)
    conflict = both & ~agree
    Z = np.where(agree, Zf, np.where(Va & ~Vb, Za, np.where(Vb & ~Va, Zb, np.nan))).astype(np.float32)
    S = np.where(agree, Sf, np.where(Va & ~Vb, Sa, np.where(Vb & ~Va, Sb, np.nan))).astype(np.float32)
    V = (agree | (Va & ~Vb) | (Vb & ~Va))
    nb = int(both.sum())
    rel = (d / Zb)[agree] if agree.any() else np.zeros(0)
    st = dict(a=int(Va.sum()), b=int(Vb.sum()), both=nb, agree_frac=round(float(agree.sum()) / max(1, nb), 4),
              conflict=int(conflict.sum()), b_only=int((Vb & ~Va).sum()),
              rel_bias=round(float(np.median(rel)), 5) if len(rel) else None)
    return Z, V, S, st


def points_from(Z, valid, rectL, rp, stride):
    """depth_seg 과 같은 규칙 (stride 격자, YOLO 없음) 으로 정류 좌표 점·색 다시 뽑기 (깊이를 거른 뒤)."""
    import frames
    sub = np.zeros_like(valid)
    sub[::stride, ::stride] = True
    vs, us = np.nonzero(valid & sub)
    pts = frames.rect0_points(Z[vs, us], rp, us.astype(np.float32), vs.astype(np.float32))
    rgb = rectL[vs, us][:, ::-1].copy()
    return pts, rgb, np.full(len(pts), -1, np.int16)


# ---------------------------------------------------------------- 여러 키프레임 일관성
class View:
    """키프레임 깊이 요약 (다운샘플 격자): Z (h,w) NaN 무효, 내부값 (격자 화소), T_world_cam, f·B."""
    __slots__ = ("Z", "f", "cx", "cy", "T", "fB", "t", "full")

    def __init__(self, Z, valid, rp, T_world_cam, ffs_scale, ds, t=0.0):
        Zd, Vd = Z[::ds, ::ds], valid[::ds, ::ds]
        self.Z = np.ascontiguousarray(np.where(Vd & np.isfinite(Zd), Zd, np.nan).astype(np.float32))
        self.f, self.cx, self.cy = float(rp["f"]) / ds, float(rp["cx"]) / ds, float(rp["cy"]) / ds
        self.T = np.asarray(T_world_cam, np.float64).reshape(4, 4)
        self.fB = f_eff_B(rp, ffs_scale)
        self.t = t
        self.full = (Z.shape[1], Z.shape[0])


class ConsistencyFilter:
    """현재 키프레임 깊이를 이웃 키프레임 깊이와 맞춰 보기 (모듈 설명 2). 실시간은 push 한 최근 n 개, 오프라인은 neighbors 를 직접 줌."""

    def __init__(self, k=2.5, ds=4, sigma_px=0.5, mode="drop", n_recent=3, min_dt=0.0, gate_px=4.0, pose_sigma=0.0, gpu=False):
        self.k, self.ds, self.sigma_px, self.mode = float(k), int(ds), float(sigma_px), mode
        self.n_recent, self.min_dt = int(n_recent), float(min_dt)
        self.gate_px = float(gate_px)
        self.pose_sigma = float(pose_sigma)              # 자세 오차 몫 (m): 비교 허용 σ 에 제곱합으로 더함
        self.gpu = bool(gpu)                             # 실시간: torch GPU 로 세기 (잔차 표본은 안 모음)
        self.recent = []
        self.stats = dict(n=0, ms=[], drop_frac=[], agree_frac=[], seen_frac=[])
        self.rd = []                     # 시차 잔차 표본 (D, 노이즈 실측)

    def view(self, Z, valid, rp, T_world_cam, ffs_scale, t=0.0):
        return View(Z, valid, rp, T_world_cam, ffs_scale, self.ds, t)

    def reset(self):
        self.recent = []

    def push(self, v):
        self.recent = (self.recent + [v])[-self.n_recent:]

    def compare(self, cur, nb):
        """cur 격자 화소마다 (seen, agree, conflict) bool 과 시차 잔차 (seen 화소, FFS px)."""
        Z = cur.Z
        h, w = Z.shape
        vs, us = np.nonzero(np.isfinite(Z))
        z = Z[vs, us].astype(np.float64)
        pc = np.stack([(us - cur.cx) / cur.f * z, (vs - cur.cy) / cur.f * z, z], 1)
        T = np.linalg.inv(nb.T) @ cur.T                                  # cur 카메라 → nb 카메라
        pn = pc @ T[:3, :3].T + T[:3, 3]
        zn = pn[:, 2]
        okz = zn > 0.05
        zz = np.where(okz, zn, 1.0)
        un = np.round(pn[:, 0] / zz * nb.f + nb.cx).astype(np.int64)
        vn = np.round(pn[:, 1] / zz * nb.f + nb.cy).astype(np.int64)
        hn, wn = nb.Z.shape
        inb = okz & (un >= 0) & (un < wn) & (vn >= 0) & (vn < hn)
        Zn = np.full(len(z), np.nan)
        Zn[inb] = nb.Z[vn[inb], un[inb]]
        seen = inb & np.isfinite(Zn)
        r = np.where(seen, Zn - zn, 0.0)
        sc = self.sigma_px * z * z / cur.fB
        sn = self.sigma_px * np.where(seen, Zn, 0.0) ** 2 / nb.fB
        sr = np.sqrt(sc * sc + sn * sn + self.pose_sigma ** 2)
        agree = seen & (np.abs(r) <= self.k * sr)
        conflict = seen & (r > self.k * sr)
        # 시차 잔차 (FFS px): d = fB / Z → 두 깊이 차를 같은 fB (nb) 로
        with np.errstate(divide="ignore", invalid="ignore"):
            rd = nb.fB * (1.0 / zn - 1.0 / Zn)
        sel = seen & np.isfinite(rd)
        rd = np.stack([rd[sel], zn[sel], r[sel]], 1)                       # (n, 3) 시차 잔차 · 깊이 · 깊이 잔차 (m)
        out = np.zeros((3, h, w), bool)
        out[0, vs, us] = seen
        out[1, vs, us] = agree
        out[2, vs, us] = conflict
        return out, rd

    def _counts_torch(self, cur, nbs):
        """compare 와 같은 규칙을 GPU 로 (실시간 키프레임 시간 예산). 반환 (seen, agree, conflict) (h,w) int16 numpy."""
        import torch
        dev = torch.device("cuda")
        Z = torch.as_tensor(cur.Z, device=dev)
        h, w = Z.shape
        vs, us = torch.nonzero(torch.isfinite(Z), as_tuple=True)
        z = Z[vs, us]
        pc = torch.stack([(us - cur.cx) / cur.f * z, (vs - cur.cy) / cur.f * z, z], 1)
        cnt = torch.zeros((3, len(z)), dtype=torch.int16, device=dev)
        sc2 = (self.sigma_px * z * z / cur.fB) ** 2 + self.pose_sigma ** 2
        for nb in nbs:
            T = torch.as_tensor(np.linalg.inv(nb.T) @ cur.T, dtype=torch.float32, device=dev)
            pn = pc @ T[:3, :3].T + T[:3, 3]
            zn = pn[:, 2]
            okz = zn > 0.05
            zz = torch.where(okz, zn, torch.ones_like(zn))
            un = torch.round(pn[:, 0] / zz * nb.f + nb.cx).to(torch.int64)
            vn = torch.round(pn[:, 1] / zz * nb.f + nb.cy).to(torch.int64)
            hn, wn = nb.Z.shape
            inb = okz & (un >= 0) & (un < wn) & (vn >= 0) & (vn < hn)
            Zm = torch.as_tensor(nb.Z, device=dev)
            Zn = Zm[vn.clamp(0, hn - 1), un.clamp(0, wn - 1)]
            seen = inb & torch.isfinite(Zn)
            Zn = torch.where(seen, Zn, torch.zeros_like(Zn))
            r = Zn - zn
            sr = torch.sqrt(sc2 + (self.sigma_px * Zn * Zn / nb.fB) ** 2)
            cnt[0] += seen.to(torch.int16)
            cnt[1] += (seen & (r.abs() <= self.k * sr)).to(torch.int16)
            cnt[2] += (seen & (r > self.k * sr)).to(torch.int16)
        c = cnt.cpu().numpy()
        vs, us = vs.cpu().numpy(), us.cpu().numpy()
        out = np.zeros((3, h, w), np.int16)
        out[:, vs, us] = c
        return out[0], out[1], out[2]

    def check(self, cur, neighbors, sample_rd=4000):
        """→ (keep (H,W) bool 원래 해상도, wmul (H,W) f32 또는 None, info). keep 은 valid 에 & 해서 씀."""
        import time
        t0 = time.perf_counter()
        h, w = cur.Z.shape
        seen = np.zeros((h, w), np.int16)
        agree = np.zeros((h, w), np.int16)
        conf = np.zeros((h, w), np.int16)
        nbs = [n for n in neighbors if n is not cur and abs(n.t - cur.t) >= self.min_dt]
        if self.gpu and nbs:
            seen, agree, conf = self._counts_torch(cur, nbs)
            nbs_np = []
        else:
            nbs_np = nbs
        for nb in nbs_np:
            m, rd = self.compare(cur, nb)
            seen += m[0]
            agree += m[1]
            conf += m[2]
            if len(rd):
                if len(rd) > sample_rd:
                    rd = rd[np.random.default_rng(len(self.rd)).choice(len(rd), sample_rd, replace=False)]
                self.rd.append(rd.astype(np.float32))
        valid = np.isfinite(cur.Z)
        nv = max(1, int(valid.sum()))
        W, H = cur.full
        wmul = None
        if self.mode == "measure":                         # 재기만 (노이즈 실측, 깊이는 그대로)
            keep_l = np.ones((h, w), bool)
            drop = 0
        elif self.mode == "weight":
            wl = np.where(seen > 0, (agree + 0.5) / (seen + 0.5), 1.0).astype(np.float32)
            keep_l = np.ones((h, w), bool)
            wmul = _up(wl, self.ds, W, H)
            drop = int((valid & (seen > 0) & (agree == 0)).sum())
        else:
            if self.mode == "strict":                      # 본 이웃이 있는데 맞는 이웃이 없으면 버림
                bad = (seen > 0) & (agree == 0)
            else:                                          # drop: 어긋남이 있고 맞음이 없으면 버림
                bad = (conf > 0) & (agree == 0)
            keep_l = ~bad
            drop = int((valid & bad).sum())
        keep = _up(keep_l.astype(np.uint8), self.ds, W, H) > 0
        info = dict(nb=len(nbs), seen_frac=round(float((valid & (seen > 0)).sum()) / nv, 4),
                    agree_frac=round(float((valid & (agree > 0)).sum()) / nv, 4), drop_frac=round(drop / nv, 4),
                    ms=round(1000 * (time.perf_counter() - t0), 2))
        st = self.stats
        st["n"] += 1
        for kk in ("ms", "drop_frac", "agree_frac", "seen_frac"):
            st[kk].append(info[kk])
        return keep, wmul, info

    def summary(self):
        st = self.stats
        out = dict(n=st["n"], mode=self.mode, k=self.k, ds=self.ds, sigma_px=self.sigma_px)
        for kk in ("ms", "drop_frac", "agree_frac", "seen_frac"):
            v = st[kk]
            out[kk + "_med"] = round(float(np.median(v)), 4) if v else None
        out.update(sigma_px_estimate(self.rd))
        return out


def _up(a, ds, W, H):
    """다운샘플 격자 (a[::ds, ::ds] 로 만든 것) → 원래 해상도 (최근접: 원래 화소 (u, v) ↔ 격자 (round(u/ds), round(v/ds)))."""
    h, w = a.shape
    o = ds // 2                                          # round(u/ds) = floor((u + ds/2)/ds) → 반복한 격자를 o 만큼 밀기
    rep = np.repeat(np.repeat(a, ds, 0), ds, 1)
    need_h, need_w = H + o, W + o
    if rep.shape[0] < need_h or rep.shape[1] < need_w:
        rep = np.pad(rep, ((0, max(0, need_h - rep.shape[0])), (0, max(0, need_w - rep.shape[1]))), mode="edge")
    return rep[o:o + H, o:o + W]


SIGMA_BANDS = ((0.3, 0.7), (0.7, 1.0), (1.0, 1.5), (1.5, 2.5))


def _robust_sigma(x):
    med = float(np.median(x))
    return med, 1.4826 * float(np.median(np.abs(x - med))) / np.sqrt(2)


def sigma_px_estimate(rd_list, gate_px=4.0):
    """이웃 비교 시차 잔차 → 한 깊이 지도의 σ_d 추정 (FFS px): 1.4826·MAD / √2 (|rd| ≤ gate 안, 가림·큰 이상치 제외).
    잔차에는 두 깊이의 노이즈 + 자세 오차가 함께 들어감 → TSDF 가 실제로 겪는 노이즈.
    거리 구간별 σ_d 가 거의 같으면 σ_Z ∝ Z² 모형이 맞음 (구간마다 다르면 sigma_px 하나로는 모자람)."""
    if not rd_list:
        return dict(rd_n=0)
    a = np.concatenate(rd_list)
    a = a[np.isfinite(a).all(1)]
    rd, z, rz = a[:, 0], a[:, 1], a[:, 2]
    gm = np.abs(rd) <= gate_px
    if gm.sum() < 100:
        return dict(rd_n=int(len(rd)))
    med, sg = _robust_sigma(rd[gm])
    out = dict(rd_n=int(len(rd)), rd_in_gate=round(float(gm.mean()), 4), rd_med_px=round(med, 4), sigma_px_est=round(sg, 4))
    bands = {}
    for lo, hi in SIGMA_BANDS:
        m = gm & (z >= lo) & (z < hi)
        if m.sum() >= 100:
            bm, bs = _robust_sigma(rd[m])
            _, bz = _robust_sigma(rz[m])
            bands[f"{lo}-{hi}"] = dict(n=int(m.sum()), sigma_px=round(bs, 4), med_px=round(bm, 4), sigma_cm=round(100 * bz, 3))
    out["sigma_px_by_z"] = bands
    return out


# ---------------------------------------------------------------- 좌우 일관성 (LR check)
def lr_check_mask(disp, dispR, tol_px=1.0):
    """왼쪽 기준 시차 disp, 오른쪽 기준 시차 dispR (같은 정류 격자, 오른쪽 화소 기준) → 좌우가 맞는 왼쪽 화소."""
    H, W = disp.shape
    u = np.arange(W, dtype=np.float32)[None, :]
    ur = np.clip(np.round(u - disp).astype(np.int64), 0, W - 1)
    vv = np.repeat(np.arange(H)[:, None], W, 1)
    return np.abs(dispR[vv, ur] - disp) < tol_px


def infer_right(model, rectL, rectR, iters, scale):
    """오른쪽 기준 시차 (좌우 뒤집어 FFS 한 번 더): 뒤집은 오른쪽을 왼쪽으로."""
    import lib_ffs
    a = cv2.cvtColor(np.ascontiguousarray(rectR[:, ::-1]), cv2.COLOR_BGR2RGB)
    b = cv2.cvtColor(np.ascontiguousarray(rectL[:, ::-1]), cv2.COLOR_BGR2RGB)
    return lib_ffs.infer(model, a, b, valid_iters=iters, scale=scale)[:, ::-1]


# ---------------------------------------------------------------- TSDF 넣기 (화소별 σ·가중치)
def integrate_tsdf(vmap, Z, valid, bgr, rp, T_cam, ffs_scale, t, log=None, epoch=None, sigma=None, wmul=None):
    """voxel_map.VoxelMap.integrate_depth 와 같고, sigma/wmul (화소별) 이 있으면 TSDF 에 그대로 넘김."""
    if sigma is None and wmul is None:
        return vmap.integrate_depth(Z, valid, bgr, rp, T_cam, ffs_scale=ffs_scale, t=t, epoch=epoch, log=log)
    with vmap.lock:
        if getattr(vmap, "_tsdf", None) is None:                 # 처음: 빈 깊이로 TSDF 만들기 (voxel_map 규칙대로)
            vmap.integrate_depth(Z, np.zeros_like(valid), bgr, rp, T_cam, ffs_scale=ffs_scale, t=t, epoch=epoch, log=log)
        ts = vmap._tsdf
        if ts is None or (epoch is not None and epoch != vmap.epoch):
            return None
        return ts.integrate(Z, valid, bgr, rp["f"], rp["cx"], rp["cy"], rp["baseline_m"], T_cam, ffs_scale=ffs_scale, t=t,
                            sigma=sigma, wmul=wmul)


# ---------------------------------------------------------------- 실시간 (scan3d._process_keyframe)
# 실시간 일관성 거르기: 앞 키프레임 n 개 (뒤 키프레임은 아직 없음). 기본 꺼짐 — 환경 변수 SCAN3D_CONSIST=1 이면 켬
# (SCAN3D_CONSIST_POSE_SIGMA · SCAN3D_CONSIST_DS 로 값 바꿈).
# 근거 (2026-10-17, 서버와 같은 재생 경로, VIO 자세, 메시 국소 두께 / 판 전체 / RMS cm):
#   165645 2+5: 끔 7.43 / 20.27 / 5.68 → 켬 (자세 σ 2 cm, ds 4) 7.40 / 18.81 / 5.36
#   175446    : 끔 10.64 / 21.24 / 6.12 → 켬 9.17 / 21.32 / 6.06   (정점 −4~−9 %)
#   자세 σ 0 (깊이 σ 만) 이면 VIO 상대 자세 오차 때문에 맞음 비율 0.30, 버림 7 %, 두께 이득 없음 → 자세 σ 2 cm 필요.
#   seed 4 개 평균 국소 두께: 165645 7.85 → 7.32, 175446 11.0 → 9.96 (−7~−9 %).
#   이득이 작고 키프레임당 +12~15 ms (numpy; GPU 세기 _counts_torch 로 7~9 ms, ds 8 은 7 ms 지만 165645 가 나빠짐)
#   → 키프레임 예산 (~115 ms) 을 넘겨 기본 끔.
#   오프라인 정밀 복원 (SfM 자세, 앞뒤 이웃) 은 효과가 커서 sfm_recon --consist 2 가 기본.
REALTIME = dict(enabled=False, n=2, k=2.5, ds=4, mode="drop", pose_sigma=0.02)


def realtime_cfg():
    import os
    c = dict(REALTIME)
    env = os.environ.get("SCAN3D_CONSIST")
    if env is not None:
        c["enabled"] = env.strip().lower() not in ("0", "off", "false", "")
    ps = os.environ.get("SCAN3D_CONSIST_POSE_SIGMA")
    if ps:
        c["pose_sigma"] = float(ps)
    dsv = os.environ.get("SCAN3D_CONSIST_DS")
    if dsv:
        c["ds"] = int(dsv)
    return c


def realtime_check(owner, r, rp, T_cam, ffs_scale, t, epoch, vm, sigma_px):
    """depth_seg 결과 r 을 앞 키프레임들과 맞춰 보고, 어긋난 화소를 r 에서 뺌 (valid 와 점·색·클래스·물체 색인 모두).
    필터는 owner._cfilt 에 붙여 둠 (지도 객체나 epoch 가 바뀌면 새로). 반환 info dict 또는 None (꺼짐)."""
    c = realtime_cfg()
    if not c["enabled"]:
        return None
    key = (id(vm), epoch)
    cf = getattr(owner, "_cfilt", None)
    if cf is None or getattr(cf, "key", None) != key:
        try:
            import torch
            gpu = torch.cuda.is_available()
        except ImportError:
            gpu = False
        cf = ConsistencyFilter(k=c["k"], ds=c["ds"], sigma_px=sigma_px, mode=c["mode"], n_recent=c["n"],
                               pose_sigma=c["pose_sigma"], gpu=gpu)
        cf.key = key
        owner._cfilt = cf
    v = cf.view(r["Z"], r["valid"], rp, T_cam, ffs_scale, t)
    nbs = list(cf.recent)
    cf.push(v)
    if not nbs:
        return dict(nb=0)
    keep, _, info = cf.check(v, nbs)
    if not keep.all():
        r["valid"] = r["valid"] & keep
        if r.get("us") is not None:
            kp = keep[r["vs"], r["us"]]
            for kk in ("pts_rect0", "rgb", "cls", "det_idx", "us", "vs"):
                if r.get(kk) is not None:
                    r[kk] = r[kk][kp]
            r["n"] = len(r["pts_rect0"])
    st = cf.stats
    info["drop_med"] = round(float(np.median(st["drop_frac"])), 4) if st["drop_frac"] else None
    info["ms_med"] = round(float(np.median(st["ms"])), 2) if st["ms"] else None
    return info
