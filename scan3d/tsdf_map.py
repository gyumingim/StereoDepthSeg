"""TSDF 융합 지도 + 메시 (GPU torch, 희소 8³ 블록 해시, 월드 z-up, 단위 m).

왜: 두 렌즈 기선이 짧아 (2+5 = 15.5 mm) 깊이 노이즈가 거리 제곱에 비례 (σ_Z = σ_d · Z² / (f·B), 640 정류 f≈470,
σ_d≈0.5 px 이면 1 m ±7 cm, 2 m ±28 cm). 점을 복셀에 '관측 횟수' 로 쌓으면 벽이 두껍다.
→ 키프레임 깊이 지도마다 투영형 TSDF 갱신, 관측마다 가중치 ∝ 1/σ² (가까운 깊이를 훨씬 믿음),
   truncation μ(Z) = clip(k·σ(Z), μ_min, μ_max) (거리별 노이즈에 맞게 키움), 부호 있는 거리는 미터 그대로 저장.
→ marching cubes (자체 GPU 구현, 삼각형 표 = skimage lorensen 으로 256 경우를 한 번 만들어 씀) 로 메시.

저장 (블록 b 마다 8³ 복셀, 색인 = x + 8y + 64z):
  sdf (cap,512) f32  부호 있는 거리 (m, [-μ, +μ]),  w (cap,512) f32  누적 가중치 (σ_ref=1 cm 기준: w = (σ_ref/σ)²),
  rgb (cap,512,3) u8 가중 평균 색,  nobs (cap,512) u8 관측 수,  bver (cap,) i64 마지막으로 바뀐 판 번호.
메시 표시 기준: w ≥ w_min (= (σ_ref/sigma_ok)², 융합 노이즈가 sigma_ok 이하) 이고 관측 ≥ min_obs 인 복셀만.
증분 메시: 청크 (4³ 블록) 단위. 바뀐 블록이 닿는 청크만 다시 뽑아 판 번호를 올린다 → 웹은 바뀐 청크만 받는다.

공개 함수 (재위치 쪽 '지도 조각 잇기' 용):
  merge(other, T=None)  다른 TSDFMap 의 복셀을 T (4x4, other 월드 → self 월드) 로 옮겨 가중 평균으로 합침
                        (복셀 중심을 옮겨 둘레 8 복셀에 3선형 가중치로 나눠 넣음 — 회전해도 구멍 없음. 강체 변환이라 거리 값은 그대로).
  transform(T)          지도 전체를 T 로 옮김 (새 지도에 merge 하고 바꿔 끼움).
"""
import itertools
import json
import os
import struct
import threading
import time
from pathlib import Path

import numpy as np

BLK = 8                       # 블록 한 변 복셀 수
CHUNK = 4                     # 메시 청크 한 변 블록 수 (2 cm 복셀이면 64 cm)
SIGMA_REF = 0.01              # 가중치 기준 노이즈 (m): w = (SIGMA_REF / σ)²
_OFFB = 1 << 20
_EPOCH = itertools.count(int(time.time()) % 100000 * 1000 + 1)   # 프로세스 안에서 지도마다 다른 epoch (웹이 전체 다시 받게)
MESH_MAGIC = b"S3M1"
MERGE_CHUNK = 400000         # merge 한 번에 옮기는 원본 복셀 수 (8 배로 늘어 GPU 메모리 약 0.2 GB)

# 기본값 근거 (녹화 3개 오프라인 비교, 2026-10-17, lib_ffs 좌우 같은 크기 고친 뒤 — 같은 키프레임 깊이 입력,
#   큰 평면 4개에서 15 cm 칸마다 p95−p5 두께의 평균. 143702 640 2+5 / 165645 1440 2+5 / 162204 1440 2+6):
#   기존 복셀 3 cm·2.5 m: 국소 두께 24.3 / 23.0 / 14.0 cm, 평면 RMS 8.0 / 7.9 / 6.1 cm
#   TSDF 2 cm (이 기본값): 11.2 / 10.9 / 6.7 cm (약 절반), RMS 5.9 / 6.4 / 4.7 cm, 적분 중앙값 7 / 13 / 13 ms (< 30 ms 목표).
#   복셀 3 cm 도 두께는 거의 같음 (노이즈·VIO 이중 벽이 지배) — 2 cm 는 모서리·작은 물체가 살아 정점이 약 2.5배, 메모리 수십 MB.
#   깊이 한계: 노이즈 σ(Z) 가 sigma_max (15 cm) 되는 거리와 z_max (2.5 m) 중 가까운 것.
#     2+5 (B 15.5 mm, FFS 해상도 f·B ≈ 7.3) → 약 1.45 m: 1.5 m 와 2.5 m 의 두께가 같고 (11.0 vs 11.2 cm) 메모리 1/4.
#     2+6 (B 32 mm, f·B ≈ 42) → 2.5 m 상한: 1.5 m 로 자르면 2.1 m 바닥이 사라지고 두께도 6.7 → 7.8 cm 로 나빠짐.
#   sigma_ok 3 cm: 2 cm 는 두께 8.2 / 8.8 cm 로 더 얇지만 구멍이 늘어 정점 약 35 % 감소, 5 cm 는 12.6 / 12.8 cm.
#   trunc_max 10 cm (6 cm 는 11.9 / 12.0 cm 로 나빠짐), min_obs 3.
DEFAULTS = dict(voxel_m=0.02, z_min=0.3, z_max=2.5, sigma_max=0.15, sigma_px=0.5, trunc_k=2.0, trunc_min_vox=2.5,
                trunc_max=0.10, sigma_ok=0.03, min_obs=3, max_blocks=60000, sigma_floor=0.0)

_TABLES = {}


def _tables(device):
    """(OFF (8,3), EDGE (12,2), TRI (256,16) int64 -1 = 끝, NTRI (256,)) — skimage lorensen 으로 256 경우를 한 번 만든다."""
    import torch
    key = str(device)
    if key in _TABLES:
        return _TABLES[key]
    if "np" not in _TABLES:
        import warnings
        from skimage.measure import marching_cubes
        off = np.array([[(i >> 0) & 1, (i >> 1) & 1, (i >> 2) & 1] for i in range(8)])
        edges = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count("1") == 1]
        mid = {tuple((off[a] + off[b]) / 2): e for e, (a, b) in enumerate(edges)}
        tri = np.full((256, 16), -1, np.int64)
        for m in range(1, 255):
            v = np.ones((2, 2, 2), np.float32)
            for i in range(8):
                if m >> i & 1:
                    v[tuple(off[i])] = -1.0
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                verts, faces, _, _ = marching_cubes(v, 0.0, method="lorensen", allow_degenerate=False)
            t = [mid[tuple(np.round(verts[j], 3))] for f in faces for j in f]
            tri[m, :len(t)] = t
        _TABLES["np"] = (off, np.array(edges), tri)
    off, edges, tri = _TABLES["np"]
    T = (torch.as_tensor(off, device=device), torch.as_tensor(edges, device=device), torch.as_tensor(tri, device=device),
         torch.as_tensor((tri >= 0).sum(1) // 3, device=device))
    _TABLES[key] = T
    return T


def _pack(k):
    """정수 좌표 (N,3) int64 → 한 정수 (±2^20)."""
    k = k + _OFFB
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def _unpack(p):
    m = (1 << 21) - 1
    import torch
    return torch.stack([(p >> 42) & m, (p >> 21) & m, p & m], 1) - _OFFB


class TSDFMap:
    def __init__(self, voxel_m=None, z_min=None, z_max=None, sigma_max=None, sigma_px=None, trunc_k=None, trunc_min_vox=None,
                 trunc_max=None, sigma_ok=None, min_obs=None, max_blocks=None, sigma_floor=None, device=None, log=None):
        import torch
        c = dict(DEFAULTS)
        for k, v in dict(voxel_m=voxel_m, z_min=z_min, z_max=z_max, sigma_max=sigma_max, sigma_px=sigma_px, trunc_k=trunc_k,
                         trunc_min_vox=trunc_min_vox, trunc_max=trunc_max, sigma_ok=sigma_ok, min_obs=min_obs,
                         max_blocks=max_blocks, sigma_floor=sigma_floor).items():
            if v is not None:
                c[k] = v
        self.cfg = c
        self.vm = float(c["voxel_m"])
        self.dev = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.log = log or (lambda m: None)
        self.lock = threading.RLock()
        self.w_min = (SIGMA_REF / float(c["sigma_ok"])) ** 2
        self._alloc(1024)
        self.epoch = next(_EPOCH)
        self.version = 0                     # 적분 판 (블록 bver 와 비교)
        self.frames = 0
        self.full = False                    # max_blocks 에 닿음
        self.t_int = []                      # 최근 적분 시간 (ms)
        self.z_max_eff = None                # 마지막 키프레임의 실제 깊이 한계 (σ 한계와 z_max 중 가까운 것)
        self._mesh = {}                      # 청크 키 → dict(v, c, f, ver)
        self._mesh_ver = 0                   # 청크 판 번호 (웹 since)
        self._mesh_at = 0                    # 마지막 추출 때의 self.version
        self._mesh_t = 0.0
        self._mesh_ms = None
        self._dump = os.environ.get("SCAN3D_TSDF_DUMP")   # 실험용: 키프레임 깊이 입력을 npz 로 저장 (오프라인 비교)

    # ---------------- 저장소 ----------------
    def _alloc(self, cap):
        import torch
        d = self.dev
        self.cap, self.nb = cap, 0
        self.keys = torch.zeros((cap, 3), dtype=torch.int64, device=d)
        self.sdf = torch.zeros((cap, BLK ** 3), dtype=torch.float32, device=d)
        self.w = torch.zeros((cap, BLK ** 3), dtype=torch.float32, device=d)
        self.rgb = torch.zeros((cap, BLK ** 3, 3), dtype=torch.uint8, device=d)
        self.nobs = torch.zeros((cap, BLK ** 3), dtype=torch.uint8, device=d)
        self.bver = torch.zeros(cap, dtype=torch.int64, device=d)
        self._sk = torch.zeros(0, dtype=torch.int64, device=d)     # 정렬한 블록 키
        self._si = torch.zeros(0, dtype=torch.int64, device=d)     # 그 블록 행

    def _grow(self, need):
        import torch
        if need <= self.cap:
            return
        new = max(need, self.cap * 2)
        for name in ("keys", "sdf", "w", "rgb", "nobs", "bver"):
            a = getattr(self, name)
            b = torch.zeros((new,) + tuple(a.shape[1:]), dtype=a.dtype, device=a.device)
            b[:self.cap] = a
            setattr(self, name, b)
        self.cap = new

    def _find(self, pk):
        """블록 키 (N,) → 행 (없으면 -1)."""
        import torch
        if len(self._sk) == 0:
            return torch.full_like(pk, -1)
        pos = torch.searchsorted(self._sk, pk).clamp(max=len(self._sk) - 1)
        return torch.where(self._sk[pos] == pk, self._si[pos], torch.full_like(pk, -1))

    def _activate(self, bkeys):
        """블록 좌표 (N,3) (중복 가능) → 행 (N,). 새 블록은 할당 (max_blocks 넘으면 -1)."""
        import torch
        pk = _pack(bkeys)
        u, inv = torch.unique(pk, return_inverse=True)
        rows = self._find(u)
        miss = rows < 0
        nm = int(miss.sum())
        if nm:
            room = int(self.cfg["max_blocks"]) - self.nb
            if nm > room:
                if not self.full:
                    self.log(f"[tsdf] 블록 한도 {self.cfg['max_blocks']} — 새 영역은 더 안 넣음")
                self.full = True
                idx = torch.nonzero(miss).squeeze(1)[:max(room, 0)]
                miss = torch.zeros_like(miss); miss[idx] = True
                nm = len(idx)
            if nm:
                self._grow(self.nb + nm)
                new_rows = torch.arange(self.nb, self.nb + nm, device=self.dev)
                rows[miss] = new_rows
                self.keys[new_rows] = _unpack(u[miss])
                self.nb += nm
                sk, order = torch.sort(_pack(self.keys[:self.nb]))
                self._sk, self._si = sk, order
        return rows[inv]

    # ---------------- 적분 ----------------
    def z_limit(self, f_eff, B):
        """깊이 한계 = min(z_max, σ(Z) = sigma_max 인 Z)."""
        c = self.cfg
        zs = (float(c["sigma_max"]) * f_eff * B / float(c["sigma_px"])) ** 0.5 if c.get("sigma_max") else float("inf")
        return float(min(float(c["z_max"]), zs))

    def sigma(self, Z, f_eff, B):
        s = self.cfg["sigma_px"] * Z * Z / (f_eff * B)
        fl = float(self.cfg.get("sigma_floor") or 0.0)
        return (s * s + fl * fl) ** 0.5 if fl > 0 else s

    def integrate(self, Z, valid, bgr, f, cx, cy, baseline_m, T_world_cam, ffs_scale=1.0, t=0.0, sigma=None, wmul=None):
        """Z (H,W) f32 정류 왼쪽 깊이 (m, NaN 가능), valid (H,W) bool, bgr (H,W,3) u8 (정류 왼쪽, 같은 화소),
        f·cx·cy = 그 정류 카메라 내부값 (화소, Z 해상도), baseline_m, T_world_cam (4x4, 정류 카메라 → 월드),
        ffs_scale = FFS 입력 축소 (시차 노이즈는 FFS 해상도 화소 단위라 σ 에 f·ffs_scale 을 씀). 반환 dict(ms, blocks, new).
        sigma (H,W) f32 (선택) = 화소별 깊이 노이즈 (m) — 두 렌즈 쌍 합친 깊이처럼 화소마다 f·B 가 다를 때 (depth_fuse.py).
          주면 σ(Z) 식 대신 이 값으로 가중치·잘림·깊이 한계 (σ ≤ sigma_max, Z ≤ z_max).
        wmul (H,W) f32 (선택) = 화소별 가중치 배수 (여러 키프레임 일관성, depth_fuse.ConsistencyFilter)."""
        import torch
        t0 = time.perf_counter()
        if self._dump:
            self._dump_kf(Z, valid, bgr, f, cx, cy, baseline_m, T_world_cam, ffs_scale, t)
        c = self.cfg
        dev = self.dev
        f_eff = float(f) * float(ffs_scale or 1.0)
        B = float(baseline_m)
        zmax = self.z_limit(f_eff, B) if sigma is None else float(c["z_max"])
        with self.lock:
            self.z_max_eff = zmax
            Zt = torch.as_tensor(np.ascontiguousarray(Z, np.float32), device=dev)
            Mt = torch.as_tensor(np.ascontiguousarray(valid), device=dev)
            Mt = Mt & torch.isfinite(Zt) & (Zt >= c["z_min"]) & (Zt <= zmax)
            St = Wm = None
            if sigma is not None:
                St = torch.as_tensor(np.ascontiguousarray(sigma, np.float32), device=dev)
                fl = float(c.get("sigma_floor") or 0.0)
                if fl > 0:
                    St = torch.sqrt(St * St + fl * fl)
                Mt &= torch.isfinite(St) & (St > 0)
                if c.get("sigma_max"):
                    Mt &= St <= float(c["sigma_max"])
            if wmul is not None:
                Wm = torch.as_tensor(np.ascontiguousarray(wmul, np.float32), device=dev)
                Mt &= Wm > 0
            H, W = Zt.shape
            if not bool(Mt.any()):
                return dict(ms=0.0, blocks=0, new=0)
            Tt = torch.as_tensor(np.asarray(T_world_cam, np.float64)[:4, :4], dtype=torch.float32, device=dev)
            R, tr = Tt[:3, :3], Tt[:3, 3]
            # 1) 블록 할당: 듬성듬성한 화소에서 표면 앞뒤 ±μ 띠 (3 점) 가 닿는 블록
            s = max(2, W // 160)
            vs, us = torch.nonzero(Mt[::s, ::s], as_tuple=True)
            vs, us = vs * s, us * s
            z = Zt[vs, us]
            mu = self._trunc(z, f_eff, B) if St is None else self._trunc_sig(St[vs, us])
            pts = []
            for k in (-1.0, 0.0, 1.0):
                zz = z + k * mu
                pc = torch.stack([(us - cx) / f * zz, (vs - cy) / f * zz, zz], 1)
                pts.append(pc @ R.T + tr)
            pw = torch.cat(pts)
            bsz = self.vm * BLK
            bk = torch.floor(pw / bsz).to(torch.int64)
            nb0 = self.nb
            rows = torch.unique(self._activate(bk))
            rows = rows[rows >= 0]
            if len(rows) == 0:
                return dict(ms=0.0, blocks=0, new=0)
            # 2) 블록 안 모든 복셀 중심 → 카메라 → 화소 (가장 가까운 화소), sdf = Z(화소) − z(복셀)
            off = _local_offsets(dev)                                       # (512,3)
            g = self.keys[rows][:, None, :] * BLK + off[None]              # (n,512,3) 전역 복셀 좌표
            pw = (g.to(torch.float32) + 0.5) * self.vm
            pc = (pw - tr) @ R                                             # 월드 → 카메라 (Rᵀ (p − t))
            zc = pc[..., 2]
            zs = zc.clamp(min=1e-3)
            u = torch.round(pc[..., 0] / zs * f + cx).to(torch.int64)
            v = torch.round(pc[..., 1] / zs * f + cy).to(torch.int64)
            inb = (zc > 1e-3) & (u >= 0) & (u < W) & (v >= 0) & (v < H)
            ui, vi = u.clamp(0, W - 1), v.clamp(0, H - 1)
            d = Zt[vi, ui]
            ok = inb & Mt[vi, ui]
            d = torch.where(ok, d, torch.ones_like(d))
            sdf = d - zc
            sg = self.sigma(d, f_eff, B) if St is None else torch.where(ok, St[vi, ui], torch.ones_like(d))
            mu = self._trunc_sig(sg)
            ok &= sdf >= -mu
            val = torch.minimum(sdf, mu)
            wobs = (SIGMA_REF / sg) ** 2
            if Wm is not None:
                wobs = wobs * Wm[vi, ui]
            wobs = torch.where(ok, wobs, torch.zeros_like(wobs))
            S, Wt = self.sdf[rows], self.w[rows]
            Wn = Wt + wobs
            S = torch.where(ok, (S * Wt + val * wobs) / Wn.clamp(min=1e-12), S)
            self.sdf[rows] = S
            self.w[rows] = Wn.clamp(max=1e6)
            self.nobs[rows] = (self.nobs[rows].to(torch.int16) + ok.to(torch.int16)).clamp(max=255).to(torch.uint8)
            # 색: 표면 가까이 (|sdf| < μ) 만
            near = ok & (sdf.abs() < mu)
            if bool(near.any()):
                img = torch.as_tensor(np.ascontiguousarray(bgr[..., ::-1]), device=dev)      # RGB
                col = img[vi, ui].to(torch.float32)
                C = self.rgb[rows].to(torch.float32)
                wc = torch.where(near, wobs, torch.zeros_like(wobs))[..., None]
                C = torch.where(near[..., None], (C * Wt[..., None] + col * wc) / (Wt[..., None] + wc).clamp(min=1e-12), C)
                self.rgb[rows] = C.round().clamp(0, 255).to(torch.uint8)
            self.version += 1
            self.frames += 1
            self.bver[rows] = self.version
            if dev.type == "cuda":
                torch.cuda.synchronize(dev)
            ms = 1000 * (time.perf_counter() - t0)
            self.t_int = (self.t_int + [ms])[-50:]
            return dict(ms=ms, blocks=int(len(rows)), new=int(self.nb - nb0))

    def _trunc(self, Z, f_eff, B):
        return self._trunc_sig(self.sigma(Z, f_eff, B))

    def _trunc_sig(self, sig):
        c = self.cfg
        return (c["trunc_k"] * sig).clamp(min=c["trunc_min_vox"] * self.vm, max=max(c["trunc_max"], c["trunc_min_vox"] * self.vm))

    def _dump_kf(self, Z, valid, bgr, f, cx, cy, B, T, s, t):
        try:
            import cv2
            d = Path(self._dump); d.mkdir(parents=True, exist_ok=True)
            i = len(list(d.glob("kf_*.npz")))
            ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
            np.savez_compressed(d / f"kf_{i:05d}.npz", Z=np.where(valid, Z, 0).astype(np.float16), jpg=jpg.tobytes(),
                                f=f, cx=cx, cy=cy, B=B, T=np.asarray(T, np.float64), s=float(s or 1.0), t=t)
        except Exception as e:
            self.log(f"[tsdf] dump 실패 {e!r}")

    # ---------------- 메시 ----------------
    def _mc(self, rows):
        """블록 행들 안의 정육면체 (최소 꼭짓점이 그 블록 안) → 청크별 (v, c, f)."""
        import torch
        dev = self.dev
        OFF, EDGE, TRI, NTRI = _tables(dev)
        out = {}
        if len(rows) == 0:
            return out
        loc = _local_offsets(dev)
        for part in torch.split(rows, 512):
            g = self.keys[part][:, None, :] * BLK + loc[None]                       # (n,512,3)
            gc = g[:, :, None, :] + OFF[None, None]                                 # (n,512,8,3)
            bk = torch.div(gc, BLK, rounding_mode="floor")
            li = gc - bk * BLK
            r = self._find(_pack(bk.reshape(-1, 3))).reshape(gc.shape[:3])
            li = li[..., 0] + BLK * li[..., 1] + BLK * BLK * li[..., 2]
            found = r >= 0
            rr = r.clamp(min=0)
            s = self.sdf[rr, li]
            w = self.w[rr, li]
            no = self.nobs[rr, li]
            good = found & (w >= self.w_min) & (no >= int(self.cfg["min_obs"]))
            allg = good.all(-1)
            neg = (s < 0) & good
            ci = (neg.to(torch.int64) << torch.arange(8, device=dev)).sum(-1)
            sel = allg & (ci > 0) & (ci < 255)
            if not bool(sel.any()):
                continue
            bi, vi = torch.nonzero(sel, as_tuple=True)
            ci = ci[bi, vi]
            nt = NTRI[ci]
            gcube = g[bi, vi]                                                         # (M,3) 최소 꼭짓점 전역 좌표
            sc, rc, cc = s[bi, vi], rr[bi, vi], li[bi, vi]
            # 정육면체 m 의 삼각형 j (j < nt[m])
            m_idx = torch.repeat_interleave(torch.arange(len(ci), device=dev), nt)
            first = torch.cumsum(nt, 0) - nt
            j = torch.arange(len(m_idx), device=dev) - first[m_idx]
            e = TRI[ci[m_idx], :15].reshape(-1, 5, 3)[torch.arange(len(m_idx), device=dev), j]   # (F,3) 모서리 번호
            mi = m_idx[:, None].expand(-1, 3).reshape(-1)
            e = e.reshape(-1)
            a, b = EDGE[e, 0], EDGE[e, 1]                                           # a < b, b = a + 축 비트
            axis = torch.log2((b - a).to(torch.float32)).round().to(torch.int64)
            ga = gcube[mi] + OFF[a]
            ekey = _pack(ga) * 3 + axis
            chunk = _pack(torch.div(torch.div(gcube[mi], BLK, rounding_mode="floor"), CHUNK, rounding_mode="floor"))
            # 청크 안에서만 정점 공유 (청크끼리 따로 그림)
            pair = torch.stack([chunk, ekey], 1)
            up, inv = torch.unique(pair, dim=0, return_inverse=True)
            rep = torch.zeros(len(up), dtype=torch.int64, device=dev)
            rep.scatter_(0, inv, torch.arange(len(inv), device=dev))
            ra, rb = a[rep], b[rep]
            mr = mi[rep]
            sa, sb = sc[mr, ra], sc[mr, rb]
            tt = (sa / (sa - sb)).clamp(0, 1)
            pa = (gcube[mr] + OFF[ra]).to(torch.float32)
            pb = (gcube[mr] + OFF[rb]).to(torch.float32)
            vpos = ((pa + tt[:, None] * (pb - pa)) + 0.5) * self.vm
            ca = self.rgb[rc[mr, ra], cc[mr, ra]].to(torch.float32)
            cb = self.rgb[rc[mr, rb], cc[mr, rb]].to(torch.float32)
            vcol = (ca + tt[:, None] * (cb - ca)).round().clamp(0, 255).to(torch.uint8)
            faces = inv.reshape(-1, 3)
            fchunk = up[faces[:, 0], 0]
            vchunk = up[:, 0]
            # 청크마다 나눔 (정점은 청크 순으로 정렬돼 있음: unique 가 (chunk, ekey) 순)
            uc, vcount = torch.unique_consecutive(vchunk, return_counts=True)
            vstart = torch.cumsum(vcount, 0) - vcount
            forder = torch.argsort(fchunk)
            faces, fchunk = faces[forder], fchunk[forder]
            fuc, fcount = torch.unique_consecutive(fchunk, return_counts=True)
            vpos, vcol = vpos.cpu().numpy(), vcol.cpu().numpy()
            faces = faces.cpu().numpy()
            uc, vstart, vcount = uc.cpu().numpy(), vstart.cpu().numpy(), vcount.cpu().numpy()
            fuc, fcount = fuc.cpu().numpy(), fcount.cpu().numpy()
            vmap = {int(k): (int(s0), int(n)) for k, s0, n in zip(uc, vstart, vcount)}
            f0 = 0
            for k, n in zip(fuc.tolist(), fcount.tolist()):
                s0, nv = vmap[k]
                ff = faces[f0:f0 + n] - s0
                f0 += n
                prev = out.get(k)
                vv, cc_, fff = vpos[s0:s0 + nv], vcol[s0:s0 + nv], ff.astype(np.int32)
                if prev is not None:                      # 같은 청크가 split 경계에 걸침
                    fff = fff + len(prev[0])
                    vv, cc_, fff = np.concatenate([prev[0], vv]), np.concatenate([prev[1], cc_]), np.concatenate([prev[2], fff])
                out[k] = (vv, cc_, fff)
        return out

    def update_mesh(self, force=False, min_interval=0.0):
        """바뀐 블록이 닿는 청크만 다시 뽑기. 반환 = 다시 뽑은 청크 수 (안 했으면 0)."""
        import torch
        with self.lock:
            if not force and self.version == self._mesh_at:
                return 0
            if not force and time.monotonic() - self._mesh_t < min_interval:
                return 0
            t0 = time.perf_counter()
            nb = self.nb
            if nb == 0:
                self._mesh_at = self.version
                return 0
            if force or not self._mesh:
                dirty_rows = torch.arange(nb, device=self.dev)
            else:
                dirty_rows = torch.nonzero(self.bver[:nb] > self._mesh_at).squeeze(1)
            # 바뀐 블록 B 는 B 와 B − {0,1}³ 의 정육면체에 쓰인다 → 그 청크들
            k = self.keys[dirty_rows]
            nbk = (k[:, None, :] - _unit_offsets(self.dev)[None]).reshape(-1, 3)
            dch = torch.unique(_pack(torch.div(nbk, CHUNK, rounding_mode="floor")))
            allk = self.keys[:nb]
            bch = _pack(torch.div(allk, CHUNK, rounding_mode="floor"))
            sel = torch.isin(bch, dch)
            rows = torch.nonzero(sel).squeeze(1)
            res = self._mc(rows)
            self._mesh_ver += 1
            n = 0
            for ck in dch.cpu().numpy().tolist():
                v = res.get(ck)
                if v is None:
                    if ck in self._mesh:
                        self._mesh[ck] = dict(v=None, ver=self._mesh_ver)      # 비었음 (웹이 지움)
                        n += 1
                    continue
                self._mesh[ck] = dict(v=v[0], c=v[1], f=v[2], ver=self._mesh_ver)
                n += 1
            self._mesh_at = self.version
            self._mesh_t = time.monotonic()
            self._mesh_ms = 1000 * (time.perf_counter() - t0)
            return n

    def mesh_arrays(self, refresh=True):
        """전체 메시 (V (N,3) f32, C (N,3) u8, F (M,3) i32)."""
        if refresh:
            self.update_mesh()
        with self.lock:
            vs, cs, fs, base = [], [], [], 0
            for ch in self._mesh.values():
                if ch.get("v") is None:
                    continue
                vs.append(ch["v"]); cs.append(ch["c"]); fs.append(ch["f"] + base)
                base += len(ch["v"])
        if not vs:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8), np.zeros((0, 3), np.int32)
        return np.concatenate(vs).astype(np.float32), np.concatenate(cs), np.concatenate(fs).astype(np.int32)

    def mesh_bytes(self, since=0, epoch=None, min_interval=0.5):
        """웹 증분 메시. 형식 S3M1: magic(4) u32 json 길이, JSON {epoch, version, voxel_m, chunks:[{k, nv, nf}]}, 공백 패딩
        (4 B 정렬), 그다음 청크마다 positions f32 nv·3, indices u32 nf·3, colors u8 nv·3 (+0~3 B 패딩).
        epoch 이 다르면 since=0 (전체). nv=0 인 청크 = 지움."""
        self.update_mesh(min_interval=min_interval)
        with self.lock:
            if epoch is not None and int(epoch) != self.epoch:
                since = 0
            items = [(k, ch) for k, ch in self._mesh.items() if ch["ver"] > since and (since > 0 or ch.get("v") is not None)]
            head, bufs = [], []
            for k, ch in items:
                if ch.get("v") is None:
                    head.append(dict(k=str(k), nv=0, nf=0))
                    continue
                v, c, f = ch["v"], ch["c"], ch["f"]
                head.append(dict(k=str(k), nv=len(v), nf=len(f)))
                cb = c.astype(np.uint8).tobytes()
                bufs += [v.astype("<f4").tobytes(), f.astype("<u4").tobytes(), cb, b"\0" * (-len(cb) % 4)]
            js = json.dumps(dict(epoch=self.epoch, version=self._mesh_ver, voxel_m=self.vm, chunks=head),
                            separators=(",", ":")).encode()
            js += b" " * (-len(js) % 4)
            return MESH_MAGIC + struct.pack("<I", len(js)) + js + b"".join(bufs)

    def stats(self):
        with self.lock:
            nv = sum(len(ch["v"]) for ch in self._mesh.values() if ch.get("v") is not None)
            nf = sum(len(ch["f"]) for ch in self._mesh.values() if ch.get("v") is not None)
            per = 4 + 4 + 3 + 1
            return dict(epoch=self.epoch, version=self._mesh_ver, blocks=self.nb, voxel_m=self.vm,
                        z_range=[self.cfg["z_min"], self.cfg["z_max"]],
                        z_max_eff=None if self.z_max_eff is None else round(self.z_max_eff, 2), sigma_max=self.cfg["sigma_max"], frames=self.frames,
                        mem_mb=round(self.cap * BLK ** 3 * per / 2 ** 20, 1), full=self.full,
                        integrate_ms=round(float(np.median(self.t_int)), 1) if self.t_int else None,
                        integrate_ms_max=round(float(np.max(self.t_int)), 1) if self.t_int else None,
                        mesh_ms=None if self._mesh_ms is None else round(self._mesh_ms, 1),
                        mesh_vertices=nv, mesh_faces=nf, stale=self.version != self._mesh_at)

    # ---------------- 내보내기 ----------------
    def save_ply(self, path):
        V, C, F = self.mesh_arrays()
        write_mesh_ply(path, V, C, F)
        return len(V), len(F)

    def save_obj(self, path):
        Path(path).write_bytes(mesh_obj_bytes(*self.mesh_arrays()))

    # ---------------- 지도 저장·불러오기·질의 (map_store.py, esdf_map.py) ----------------
    def save_npz(self, path):
        """블록 저장 (지도 저장): 값이 있는 블록만. sdf 는 f16 (±10 cm 범위라 0.1 mm 정밀), 나머지 그대로. 반환 블록 수."""
        with self.lock:
            nb = self.nb
            keep = (self.w[:nb] > 0).any(1)
            rows = keep.nonzero().squeeze(1)
            np.savez_compressed(path, format=np.array("scan3d-tsdf/1"), cfg=np.array(json.dumps(self.cfg)),
                                voxel_m=np.float32(self.vm), keys=self.keys[rows].cpu().numpy().astype(np.int32),
                                sdf=self.sdf[rows].cpu().numpy().astype(np.float16), w=self.w[rows].cpu().numpy(),
                                rgb=self.rgb[rows].cpu().numpy(), nobs=self.nobs[rows].cpu().numpy(),
                                frames=np.int64(self.frames))
            return int(len(rows))

    @classmethod
    def load_npz(cls, path, device=None, log=None, cfg=None):
        """save_npz 로 저장한 지도 → 새 TSDFMap (cfg = 덮어쓸 설정, 복셀 크기는 저장 값)."""
        import torch
        z = np.load(path, allow_pickle=False)
        c = dict(json.loads(str(z["cfg"])))
        c.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS and k != "voxel_m"})
        c["voxel_m"] = float(z["voxel_m"])
        c = {k: v for k, v in c.items() if k in DEFAULTS}
        keys = z["keys"].astype(np.int64)
        n = len(keys)
        c["max_blocks"] = max(int(c.get("max_blocks") or 0), n + 20000)
        m = cls(device=device, log=log, **c)
        with m.lock:
            m._alloc(max(1024, n))
            d = m.dev
            if n:
                m.keys[:n] = torch.as_tensor(keys, device=d)
                m.sdf[:n] = torch.as_tensor(z["sdf"].astype(np.float32), device=d)
                m.w[:n] = torch.as_tensor(z["w"], device=d)
                m.rgb[:n] = torch.as_tensor(z["rgb"], device=d)
                m.nobs[:n] = torch.as_tensor(z["nobs"], device=d)
                m.nb = n
                sk, order = torch.sort(_pack(m.keys[:n]))
                m._sk, m._si = sk, order
            m.version = 1
            m.bver[:n] = 1
            m.frames = int(z["frames"]) if "frames" in z else 0
        return m

    def voxel_layers(self, since_version=None):
        """ESDF 재료 (esdf_map.py): 블록 키·sdf·w·nobs 를 CPU numpy 로. since_version 을 주면 그 판 뒤에 바뀐 블록만.
        반환 dict(keys (B,3) i64, sdf (B,512) f32, w, nobs, version, voxel_m, w_min, min_obs, trunc_max)."""
        import torch
        with self.lock:
            nb = self.nb
            if since_version is None:
                rows = torch.arange(nb, device=self.dev)
            else:
                rows = torch.nonzero(self.bver[:nb] > int(since_version)).squeeze(1)
            out = dict(keys=self.keys[rows].cpu().numpy(), sdf=self.sdf[rows].cpu().numpy(), w=self.w[rows].cpu().numpy(),
                       nobs=self.nobs[rows].cpu().numpy(), version=int(self.version), voxel_m=self.vm, w_min=self.w_min,
                       min_obs=int(self.cfg["min_obs"]), trunc_max=float(self.cfg["trunc_max"]))
        return out

    def cell_layers(self, res, since_version=None):
        """ESDF 재료 (esdf_map.py, GPU 에서 줄여 CPU 로): 복셀을 res 격자 칸 (정수 좌표 floor(중심/res)) 으로 모아
        touched = 다시 셀 칸 (since_version 뒤 바뀐 블록이 덮는 칸, None 이면 전체), occ = 그중 표면 칸
        (|sdf| < 복셀 크기, 가중치 ≥ w_min, 관측 ≥ min_obs — 메시 기준과 같음), free = 표면 앞 관측 칸 (sdf > 1.5 복셀, 관측 ≥ 1).
        한 칸이 바뀐 블록과 안 바뀐 이웃 블록에 걸칠 수 있어 바뀐 블록의 26-이웃 블록까지 읽고 touched 칸만 돌려준다.
        반환 dict(touched, occ, free (N,3) i64 numpy, version)."""
        import torch
        with self.lock:
            nb = self.nb
            ver = int(self.version)
            e = np.zeros((0, 3), np.int64)
            if nb == 0:
                return dict(touched=e, occ=e, free=e, version=ver)
            if since_version is None:
                rows = torch.arange(nb, device=self.dev)
                ch = rows
            else:
                ch = torch.nonzero(self.bver[:nb] > int(since_version)).squeeze(1)
                if len(ch) == 0:
                    return dict(touched=e, occ=e, free=e, version=ver)
                nk = (self.keys[ch][:, None, :] + _nbr_offsets(self.dev)[None]).reshape(-1, 3)
                rows = self._find(torch.unique(_pack(nk)))
                rows = rows[rows >= 0]
            off = _local_offsets(self.dev)
            vm, wmin, mobs = self.vm, float(self.w_min), int(self.cfg["min_obs"])
            touched, occ, free = [], [], []
            for a in range(0, len(rows), 4000):
                r = rows[a:a + 4000]
                g = self.keys[r][:, None, :] * BLK + off[None]
                cell = torch.floor((g.to(torch.float32) + 0.5) * (vm / res)).to(torch.int64).reshape(-1, 3)
                sd, w, no = self.sdf[r].reshape(-1), self.w[r].reshape(-1), self.nobs[r].reshape(-1)
                o = (sd.abs() < vm) & (w >= wmin) & (no >= mobs)
                f = (sd > 1.5 * vm) & (no >= 1)
                occ.append(torch.unique(_pack(cell[o])))
                free.append(torch.unique(_pack(cell[f])))
            for a in range(0, len(ch), 4000):
                g = self.keys[ch[a:a + 4000]][:, None, :] * BLK + off[None]
                cell = torch.floor((g.to(torch.float32) + 0.5) * (vm / res)).to(torch.int64).reshape(-1, 3)
                touched.append(torch.unique(_pack(cell)))
            T = torch.unique(torch.cat(touched))

            def pick(lst):
                u = torch.unique(torch.cat(lst)) if lst else T[:0]
                return _unpack(u[torch.isin(u, T)]).cpu().numpy()
            return dict(touched=_unpack(T).cpu().numpy(), occ=pick(occ), free=pick(free), version=ver)

    # ---------------- 조각 합치기 (재위치) ----------------
    def merge(self, other, T=None):
        """other 의 복셀을 T (other 월드 → self 월드, 4x4, None = 단위) 로 옮겨 self 에 가중 평균으로 더한다.
        복셀 크기가 달라도 됨 (옮긴 중심을 self 격자 둘레 8 복셀에 3선형 가중치로). 반환 = 값이 바뀐 self 복셀 수."""
        import torch
        if other is self:
            raise ValueError("자기 자신과 merge 할 수 없음 (transform 을 쓰세요)")
        with other.lock:
            nb = other.nb
            if nb == 0:
                return 0
            sel = other.w[:nb] > 0
            bi, li = torch.nonzero(sel, as_tuple=True)
            g = other.keys[bi] * BLK + _local_offsets(other.dev)[li]
            p = ((g.to(torch.float32) + 0.5) * other.vm).to(self.dev)
            s, w = other.sdf[bi, li].to(self.dev), other.w[bi, li].to(self.dev)
            c, no = other.rgb[bi, li].to(self.dev), other.nobs[bi, li].to(self.dev)
        if T is not None:
            Tt = torch.as_tensor(np.asarray(T, np.float64)[:4, :4], dtype=torch.float32, device=self.dev)
            p = p @ Tt[:3, :3].T + Tt[:3, 3]
        # 옮긴 복셀 중심을 둘레 8 복셀에 3선형 가중치로 나눠 넣음 (splat). 가장 가까운 한 복셀에만 넣으면 회전(예: yaw 30°)
        # 때 빈 복셀이 생겨 메시에 구멍 (평면 시험: 30° 면 54 %, 45° 면 66 % 면이 사라짐). 격자에 딱 맞는 이동·90° 회전은 그대로.
        # 원본 복셀을 MERGE_CHUNK 개씩 (GPU 메모리: 8 배로 늘어남)
        offs = _unit_offsets(self.dev)
        touched = []
        with self.lock:
            for a in range(0, len(p), MERGE_CHUNK):
                q = p[a:a + MERGE_CHUNK] / self.vm - 0.5             # 정수 = self 복셀 중심
                g0 = torch.floor(q)
                fr = (q - g0).clamp(0.0, 1.0)
                o = offs.to(torch.float32)                             # (8,3)
                coef = torch.where(o[None].bool(), fr[:, None, :], 1.0 - fr[:, None, :]).prod(2)   # (n,8)
                gv = (g0.to(torch.int64)[:, None, :] + offs[None]).reshape(-1, 3)
                coef = coef.reshape(-1)
                src = torch.arange(len(q), device=self.dev).repeat_interleave(8) + a
                m = coef > 1e-4
                gv, coef, src = gv[m], coef[m], src[m]
                bk = torch.div(gv, BLK, rounding_mode="floor")
                lc = gv - bk * BLK
                li2 = lc[:, 0] + BLK * lc[:, 1] + BLK * BLK * lc[:, 2]
                rows = self._activate(bk)
                ok = rows >= 0
                rows, li2, coef, src = rows[ok], li2[ok], coef[ok], src[ok]
                wk = w[src] * coef
                flat = rows * BLK ** 3 + li2
                u, inv = torch.unique(flat, return_inverse=True)
                nu = len(u)
                sw = torch.zeros(nu, device=self.dev).index_add_(0, inv, s[src] * wk)
                ww = torch.zeros(nu, device=self.dev).index_add_(0, inv, wk)
                cw = torch.zeros((nu, 3), device=self.dev).index_add_(0, inv, c[src].to(torch.float32) * wk[:, None])
                nn = torch.zeros(nu, device=self.dev).index_add_(0, inv, no[src].to(torch.float32) * coef)
                # 가장자리 (둘레 원본이 적어 3선형 계수 합 < 1) 의 가중치·관측 수가 깎여 메시가 줄지 않게 계수 합으로 나눔
                # (계수 합 0.5 미만은 그대로 줄임 — 원본에서 반 복셀 넘게 떨어진 곳까지 넓히지 않게)
                csum = torch.zeros(nu, device=self.dev).index_add_(0, inv, coef)
                k = 1.0 / csum.clamp(min=0.5)
                sw, ww, cw, nn = sw * k, ww * k, cw * k[:, None], nn * k
                S, W = self.sdf.view(-1), self.w.view(-1)
                Cv, No = self.rgb.view(-1, 3), self.nobs.view(-1)
                Wn = W[u] + ww
                S[u] = (S[u] * W[u] + sw) / Wn.clamp(min=1e-12)
                Cv[u] = ((Cv[u].to(torch.float32) * W[u, None] + cw) / Wn[:, None].clamp(min=1e-12)).round().clamp(0, 255).to(torch.uint8)
                No[u] = (No[u].to(torch.float32) + nn).round().clamp(max=255).to(torch.uint8)
                W[u] = Wn.clamp(max=1e6)
                self.version += 1
                self.bver[torch.unique(rows)] = self.version
                touched.append(u)
            return int(torch.unique(torch.cat(touched)).numel()) if touched else 0

    def transform(self, T):
        """지도 전체를 T (4x4, 옛 월드 → 새 월드) 로 옮김 (제자리)."""
        new = TSDFMap(device=self.dev, log=self.log, **self.cfg)
        new.merge(self, T)
        with self.lock:
            for name in ("cap", "nb", "keys", "sdf", "w", "rgb", "nobs", "bver", "_sk", "_si", "full"):
                setattr(self, name, getattr(new, name))
            self.version += 1
            self.bver[:self.nb] = self.version
            self.epoch = next(_EPOCH)                   # 웹이 전체 다시 받게
            self._mesh, self._mesh_at = {}, 0


_LOC = {}


def _local_offsets(dev):
    import torch
    k = ("loc", str(dev))
    if k not in _LOC:
        i = torch.arange(BLK ** 3, device=dev)
        _LOC[k] = torch.stack([i % BLK, (i // BLK) % BLK, i // (BLK * BLK)], 1).to(torch.int64)
    return _LOC[k]


def _nbr_offsets(dev):
    """(27,3) int64 블록 이웃 (자기 포함)."""
    key = ("nbr", str(dev))
    if key not in _LOC:
        import torch
        _LOC[key] = torch.tensor([[x, y, z] for z in (-1, 0, 1) for y in (-1, 0, 1) for x in (-1, 0, 1)],
                                 dtype=torch.int64, device=dev)
    return _LOC[key]


def _unit_offsets(dev):
    import torch
    k = ("unit", str(dev))
    if k not in _LOC:
        _LOC[k] = torch.tensor([[(i >> 0) & 1, (i >> 1) & 1, (i >> 2) & 1] for i in range(8)], dtype=torch.int64, device=dev)
    return _LOC[k]


def write_mesh_ply(path, V, C, F):
    """이진 PLY (정점 x y z float + r g b uchar, 면 uchar n + int 꼭짓점)."""
    head = ("ply\nformat binary_little_endian 1.0\ncomment scan3d tsdf mesh (world z-up, m)\n"
            f"element vertex {len(V)}\nproperty float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            f"element face {len(F)}\nproperty list uchar int vertex_indices\nend_header\n").encode()
    vd = np.zeros(len(V), np.dtype([("p", "<f4", 3), ("c", "u1", 3)]))
    vd["p"], vd["c"] = V, C
    fd = np.zeros(len(F), np.dtype([("n", "u1"), ("i", "<i4", 3)]))
    fd["n"], fd["i"] = 3, F
    if hasattr(path, "write"):                       # 파일 객체 (웹 내려받기)
        path.write(head); path.write(vd.tobytes()); path.write(fd.tobytes())
        return
    with open(path, "wb") as fh:
        fh.write(head); fh.write(vd.tobytes()); fh.write(fd.tobytes())


def read_mesh_ply(path):
    """write_mesh_ply 형식만 읽음 → (V, C, F)."""
    b = Path(path).read_bytes()
    e = b.index(b"end_header\n") + len(b"end_header\n")
    head = b[:e].decode()
    nv = int(head.split("element vertex ")[1].split()[0])
    nf = int(head.split("element face ")[1].split()[0])
    vd = np.frombuffer(b, np.dtype([("p", "<f4", 3), ("c", "u1", 3)]), nv, e)
    fd = np.frombuffer(b, np.dtype([("n", "u1"), ("i", "<i4", 3)]), nf, e + vd.nbytes)
    return vd["p"].copy(), vd["c"].copy(), fd["i"].copy()


def mesh_obj_bytes(V, C, F):
    """OBJ 텍스트 (정점 색 'v x y z r g b' 0~1 — MeshLab·Blender 가 읽음), 면 1부터."""
    lines = ["# scan3d tsdf mesh (world z-up, m)"]
    cc = C.astype(np.float32) / 255.0
    lines += [f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {q[0]:.3f} {q[1]:.3f} {q[2]:.3f}" for p, q in zip(V, cc)]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in F]
    return ("\n".join(lines) + "\n").encode()


def mesh_bin_static(V, C, F, voxel_m=0.0):
    """저장한 메시 하나를 웹 S3M1 형식 한 청크로 (복원 결과 보기용). 비었으면 청크 없음 (epoch 0 = 지도 없음)."""
    js = json.dumps(dict(epoch=0, version=1, voxel_m=voxel_m, chunks=[dict(k="0", nv=len(V), nf=len(F))] if len(V) else []),
                    separators=(",", ":")).encode()
    js += b" " * (-len(js) % 4)
    cb = np.asarray(C, np.uint8).tobytes()
    return (MESH_MAGIC + struct.pack("<I", len(js)) + js + np.asarray(V, "<f4").tobytes() + np.asarray(F, "<u4").tobytes()
            + cb + b"\0" * (-len(cb) % 4))
