"""ESDF (가까운 표면까지 거리) + 관측된 빈 공간 + 2D 장애물 지도 (드론 경로 계획 재료, 2026-10-17).

왜: TSDF (tsdf_map.py) 는 표면 앞뒤 ±μ 띠 블록만 만든다 → 띠 밖 빈 공간은 '모름' 과 구별이 안 된다.
   그래서 키프레임 깊이마다 광선을 따라 '본 빈 칸' 을 따로 센다 (FreeSpace, 성긴 격자 기본 5 cm, CPU numpy).
칸 상태 (EsdfMap.state): 0 = 모름, 1 = 빈 곳 (광선이 지나감 또는 TSDF 표면 앞 띠), 2 = 장애물 (TSDF 표면 칸, 메시 기준과 같음).
   장애물이 빈 곳보다 우선 (같은 칸이면 장애물).
거리 (EsdfMap.dist, m): 칸 중심 → 가장 가까운 장애물 칸 중심 (유클리드, scipy distance_transform_edt), d_max 에서 자름.
   증분: 바뀐 칸의 상자 ± d_max 만 다시 (그 안 거리는 d_max 바깥 장애물과 상관없음 → 정확).
   모름 칸에도 거리는 있다 (경로 계획은 state 로 모름을 막을지 정한다).
2D 지도 (slice2d): 바닥 높이 (자동 = 장애물 칸 높이 분포에서 가장 낮은 큰 층) + [z0, z1] (기본 0.3~1.5 m) 의 기둥마다
   장애물 = 띠 안에 장애물 칸이 하나라도, 빈 곳 = 장애물 없고 띠 칸의 free_frac (기본 30 %) 이상을 빈 곳으로 봄, 나머지 모름.
   2D 거리 = 2D 장애물에서의 거리 (edt). PNG: 장애물 빨강, 빈 곳 흰색, 모름 회색, 거리 등고선 0.25 / 0.5 / 1.0 m.
질의: query(points (N,3)) → dict(dist (N,), grad (N,3) 거리의 기울기 (3선형 보간 + 중앙 차분), state (N,)).
저장: save(dir) → esdf.npz (dist f16·state u8·원점·해상도), esdf2d.npz · esdf2d.png · esdf.json (메타).

공개:
  FreeSpace(res).observe(Z, valid, f, cx, cy, T_world_cam)   키프레임 하나 (VoxelMap.observe_free 가 부름)
  FreeSpace.merge(other, T)                                   재위치 조각 합치기 (칸 중심을 옮겨 표시)
  EsdfMap(res, d_max).update(tsdf, free)                      바뀐 것만 반영 (tsdf = tsdf_map.TSDFMap 또는 None)
  EsdfMap.query(points) · slice2d(...) · render_png(...) · save(dir)
  for_vmap(vmap, res) → 그 지도의 EsdfMap (VoxelMap 에 붙여 둠, TSDF 가 바뀌면 새로)
"""
import json
import threading
import time
from pathlib import Path

import numpy as np

RES = 0.05                    # 기본 격자 (m)
D_MAX = 2.0                   # 거리 자름 (m)
MAX_CELLS = 120_000_000       # 촘촘한 격자 한도 (5 cm 면 약 60 m × 60 m × 4 m 까지 넉넉)
EXTENT_M = 60.0               # 원점에서 이 거리 밖 칸은 버림 (VIO 가 튄 점)
CONTOURS = (0.25, 0.5, 1.0)


class DenseGrid:
    """정수 칸 좌표 (floor(p/res)) 의 촘촘한 3D 배열 여러 개, 필요하면 커짐 (여유 pad 칸)."""

    def __init__(self, res, fields, pad=20):
        self.res = float(res)
        self.fields = dict(fields)            # 이름 → (dtype, 채움 값)
        self.pad = int(pad)
        self.lo = np.zeros(3, np.int64)
        self.shape = (0, 0, 0)
        self.a = {k: np.zeros((0, 0, 0), dt) for k, (dt, _) in self.fields.items()}

    @property
    def empty(self):
        return self.shape[0] == 0

    def hi(self):
        return self.lo + np.array(self.shape, np.int64)

    def ensure(self, lo, hi):
        """칸 범위 [lo, hi) 가 들어가게 (밖이면 pad 를 더해 다시 할당)."""
        lo, hi = np.asarray(lo, np.int64), np.asarray(hi, np.int64)
        if not self.empty and (lo >= self.lo).all() and (hi <= self.hi()).all():
            return False
        if self.empty:
            nlo, nhi = lo - self.pad, hi + self.pad
        else:
            nlo = np.minimum(self.lo, lo - self.pad)
            nhi = np.maximum(self.hi(), hi + self.pad)
        shp = tuple(int(x) for x in (nhi - nlo))
        if int(np.prod(shp)) > MAX_CELLS:
            raise MemoryError(f"ESDF 격자 {shp} 가 너무 큼 (한도 {MAX_CELLS} 칸)")
        o = self.lo - nlo
        for k, (dt, fill) in self.fields.items():
            b = np.full(shp, fill, dt)
            if not self.empty:
                s = self.shape
                b[o[0]:o[0] + s[0], o[1]:o[1] + s[1], o[2]:o[2] + s[2]] = self.a[k]
            self.a[k] = b
        self.lo, self.shape = nlo, shp
        return True

    def local(self, cells):
        return np.asarray(cells, np.int64) - self.lo


_OFF = 1 << 20


def _pack(c):
    c = np.asarray(c, np.int64) + _OFF
    return (c[:, 0] << 42) | (c[:, 1] << 21) | c[:, 2]


def _unpack(p):
    m = (1 << 21) - 1
    return np.stack([(p >> 42) & m, (p >> 21) & m, p & m], 1).astype(np.int64) - _OFF


def _cells_ok(c):
    lim = int(EXTENT_M / RES) * 4
    return np.all(np.abs(c) < lim, axis=1)


class FreeSpace:
    """관측된 빈 칸 (깊이 광선이 지나간 칸) 수. 촘촘한 u8 격자 (255 에서 멈춤)."""

    def __init__(self, res=RES, stride=None, margin=None, z_cap=3.0, rays=1500):
        self.res = float(res)
        self.stride = stride                  # None = 깊이 지도마다 광선 약 rays 개가 되게
        self.rays = int(rays)
        # 표면 바로 앞은 깊이 노이즈 → 안 깎음: max(margin, 2 칸, 깊이의 8 %) (2+5 깊이 노이즈 1.5 m 에서 약 ±7 cm)
        self.margin = float(margin if margin is not None else max(2 * self.res, 0.06))
        self.z_cap = float(z_cap)
        self.g = DenseGrid(self.res, {"n": (np.uint8, 0)})
        self.lock = threading.RLock()
        self.version = 0
        self.dirty_lo = self.dirty_hi = None   # 마지막 EsdfMap.update 뒤 바뀐 칸 상자
        self.frames = 0
        self.ms = []

    def _mark(self, cells):
        if len(cells) == 0:
            return
        cells = cells[_cells_ok(cells)]
        if len(cells) == 0:
            return
        u = _unpack(np.unique(_pack(cells)))           # 1차원 키로 (unique axis=0 보다 약 10 배 빠름)
        lo, hi = u.min(0), u.max(0) + 1
        with self.lock:
            self.g.ensure(lo, hi)
            L = self.g.local(u)
            n = self.g.a["n"]
            v = n[L[:, 0], L[:, 1], L[:, 2]]
            n[L[:, 0], L[:, 1], L[:, 2]] = np.minimum(v.astype(np.int16) + 1, 255).astype(np.uint8)
            self.version += 1
            self.dirty_lo = lo if self.dirty_lo is None else np.minimum(self.dirty_lo, lo)
            self.dirty_hi = hi if self.dirty_hi is None else np.maximum(self.dirty_hi, hi)

    def observe(self, Z, valid, f, cx, cy, T_world_cam):
        """깊이 지도 하나 (Z (H,W) m, 정류 카메라 f·cx·cy 는 Z 해상도) → 카메라에서 (깊이 − margin) 까지 광선이 지나는 칸 +1.
        깊이가 없는 화소는 아무것도 안 함 (모름 그대로)."""
        t0 = time.perf_counter()
        Hh, Ww = np.shape(Z)[:2]
        s = int(self.stride or max(4, round((Hh * Ww / self.rays) ** 0.5)))
        Zs = np.asarray(Z, np.float32)[::s, ::s]
        M = np.asarray(valid, bool)[::s, ::s] & np.isfinite(Zs) & (Zs > 0.15)
        vs, us = np.nonzero(M)
        if len(vs) == 0:
            return 0
        zv = np.minimum(Zs[vs, us], self.z_cap)
        z_end = zv - np.maximum(self.margin, 0.08 * zv)
        keep = z_end > 0.12
        vs, us, z_end = vs[keep] * s, us[keep] * s, z_end[keep]
        if len(vs) == 0:
            return 0
        d = np.stack([(us - cx) / f, (vs - cy) / f, np.ones(len(us))], 1)          # z = 1 방향
        step = 0.5 * self.res
        nst = np.floor(z_end / step).astype(np.int64) + 1
        ray = np.repeat(np.arange(len(nst)), nst)
        k = np.arange(len(ray)) - np.repeat(np.cumsum(nst) - nst, nst)
        zz = k * step                                   # 카메라 자리부터 (폰이 있던 곳 = 빈 곳)
        P = d[ray] * zz[:, None]
        T = np.asarray(T_world_cam, np.float64)
        Pw = P @ T[:3, :3].T + T[:3, 3]
        self._mark(np.floor(Pw / self.res).astype(np.int64))
        self.frames += 1
        ms = 1000 * (time.perf_counter() - t0)
        self.ms = (self.ms + [ms])[-50:]
        return len(ray)

    def cells(self):
        """빈 칸 좌표 (N,3) 와 수 (N,)."""
        with self.lock:
            if self.g.empty:
                return np.zeros((0, 3), np.int64), np.zeros(0, np.uint8)
            idx = np.argwhere(self.g.a["n"] > 0)
            return idx + self.g.lo, self.g.a["n"][tuple(idx.T)]

    def merge(self, other, T=None):
        """다른 FreeSpace 의 빈 칸을 T (other 월드 → self 월드) 로 옮겨 표시 (칸 중심의 둘레 2³ 칸 중 중심이 가장 가까운 칸)."""
        c, n = other.cells()
        if len(c) == 0:
            return 0
        p = (c + 0.5) * other.res
        if T is not None:
            T = np.asarray(T, np.float64)
            p = p @ T[:3, :3].T + T[:3, 3]
        self._mark(np.floor(p / self.res).astype(np.int64))
        return len(c)

    def save_npz(self, path):
        c, n = self.cells()
        np.savez_compressed(path, format=np.array("scan3d-free/1"), res=np.float32(self.res), cells=c.astype(np.int32), n=n)

    @classmethod
    def load_npz(cls, path):
        z = np.load(path, allow_pickle=False)
        fs = cls(res=float(z["res"]))
        c = z["cells"].astype(np.int64)
        if len(c):
            with fs.lock:
                fs.g.ensure(c.min(0), c.max(0) + 1)
                L = fs.g.local(c)
                fs.g.a["n"][L[:, 0], L[:, 1], L[:, 2]] = z["n"]
                fs.version += 1
                fs.dirty_lo, fs.dirty_hi = c.min(0), c.max(0) + 1
        return fs

    def stats(self):
        with self.lock:
            nfree = int((self.g.a["n"] > 0).sum()) if not self.g.empty else 0
        return dict(res=self.res, frames=self.frames, free_cells=nfree,
                    observe_ms=round(float(np.median(self.ms)), 1) if self.ms else None)


class EsdfMap:
    def __init__(self, res=RES, d_max=D_MAX):
        import importlib
        importlib.import_module("scipy.ndimage")          # 없으면 여기서 바로 알림
        self.res, self.d_max = float(res), float(d_max)
        self.g = DenseGrid(self.res, {"occ": (np.bool_, False), "tfree": (np.bool_, False),
                                      "dist": (np.float32, self.d_max)}, pad=int(np.ceil(0.5 / self.res)))
        self.lock = threading.RLock()
        self.tsdf_key = None                  # (id, epoch) 지금 반영한 TSDF
        self.tsdf_ver = None
        self.free_ver = -1
        self.free_ref = None
        self.version = 0
        self.last = {}                        # 마지막 update 숫자 (ms, 칸 수)

    # ---------------- 갱신 ----------------
    def update(self, tsdf=None, free=None, full=False):
        """TSDF 에서 바뀐 블록 칸 + 빈 칸 격자 변화만 반영하고 바뀐 상자 ± d_max 의 거리를 다시. 반환 self.last."""
        from scipy import ndimage
        t0 = time.perf_counter()
        with self.lock:
            g0 = (tuple(self.g.lo), self.g.shape)
            key = None if tsdf is None else (id(tsdf), tsdf.epoch)
            if key != self.tsdf_key:
                full = True
            if free is not self.free_ref:
                full = True
            if full:
                self.g = DenseGrid(self.res, self.g.fields, self.g.pad)
                self.tsdf_ver, self.free_ver = None, -1
            dlo = dhi = None
            n_occ = n_touch = 0
            if tsdf is not None:
                cl = tsdf.cell_layers(self.res, self.tsdf_ver)
                T, O, F = cl["touched"], cl["occ"], cl["free"]
                T = T[_cells_ok(T)]
                if len(T):
                    O, F = O[_cells_ok(O)], F[_cells_ok(F)]
                    lo, hi = T.min(0), T.max(0) + 1
                    self.g.ensure(lo, hi)
                    a = self.g.a
                    L = self.g.local(T)
                    a["occ"][tuple(L.T)] = False
                    a["tfree"][tuple(L.T)] = False
                    if len(O):
                        a["occ"][tuple(self.g.local(O).T)] = True
                    if len(F):
                        a["tfree"][tuple(self.g.local(F).T)] = True
                    dlo, dhi = lo, hi
                    n_occ, n_touch = len(O), len(T)
                self.tsdf_ver = cl["version"]
                self.tsdf_key = key
            else:
                self.tsdf_key = None
            if free is not None:
                # 빈 칸은 거리에 영향이 없음 (거리 = 장애물까지) → 격자가 빈 칸 범위를 덮게만. 새로 늘어난 칸은 옛 장애물에서
                # pad (≥ d_max) 넘게 떨어져 있어 d_max 로 채워도 맞다
                # 격자는 TSDF (표면) 범위 + 0.5 m 만: 그 밖의 빈 칸은 '모름' 으로 둠 (안전 쪽). 빈 칸 격자 전체로 늘리면
                # 광선 3 m 때문에 2 cm 격자가 수천만 칸이 됨 (143702: 5900 만 칸, 340 MB, 6.6 s)
                with free.lock:
                    self.free_ver = free.version
                    free.dirty_lo = free.dirty_hi = None
                self.free_ref = free
            if not self.g.empty and (tuple(self.g.lo), self.g.shape) != g0:
                # 격자가 커짐: 늘어난 칸 가까이에 옛 장애물이 있을 수 있음 (pad 0.5 m < d_max) → 전체 다시
                dlo, dhi = self.g.lo, self.g.hi()
            if dlo is not None and not self.g.empty:
                self._edt_region(dlo, dhi, ndimage)
                self.version += 1
            self.last = dict(ms=round(1000 * (time.perf_counter() - t0), 1), touched=int(n_touch), occ_new=int(n_occ),
                             full=bool(full), cells=int(np.prod(self.g.shape)), shape=list(self.g.shape),
                             mem_mb=round(self.mem_bytes() / 2 ** 20, 1))
            return self.last

    def _edt_region(self, lo, hi, ndimage):
        m = int(np.ceil(self.d_max / self.res)) + 1
        glo, ghi = self.g.lo, self.g.hi()
        rlo = np.maximum(np.asarray(lo) - m, glo)             # 다시 쓸 상자
        rhi = np.minimum(np.asarray(hi) + m, ghi)
        elo = np.maximum(rlo - m, glo)                        # 계산 상자 (장애물을 d_max 만큼 더 봄)
        ehi = np.minimum(rhi + m, ghi)
        a, b = elo - glo, ehi - glo
        occ = self.g.a["occ"][a[0]:b[0], a[1]:b[1], a[2]:b[2]]
        if occ.any():
            d = ndimage.distance_transform_edt(~occ, sampling=self.res).astype(np.float32)
            np.minimum(d, self.d_max, out=d)
        else:
            d = np.full(occ.shape, self.d_max, np.float32)
        r0, r1 = rlo - elo, rhi - elo
        w0, w1 = rlo - glo, rhi - glo
        self.g.a["dist"][w0[0]:w1[0], w0[1]:w1[1], w0[2]:w1[2]] = d[r0[0]:r1[0], r0[1]:r1[1], r0[2]:r1[2]]

    def mem_bytes(self):
        return int(sum(x.nbytes for x in self.g.a.values()))

    # ---------------- 상태 ----------------
    def free_grid(self, k0=0, k1=None):
        """ESDF 격자의 z 층 [k0, k1) 의 '본 빈 칸' bool (FreeSpace 격자를 이 해상도로 옮김) — TSDF 표면 앞 띠 포함."""
        k1 = self.g.shape[2] if k1 is None else k1
        fr = self.g.a["tfree"][:, :, k0:k1].copy()
        fs = self.free_ref
        if fs is not None and not fs.g.empty:
            with fs.lock:
                n = fs.g.a["n"]
                idx = []
                for ax in range(3):
                    c = self.g.lo[ax] + (np.arange(k0, k1) if ax == 2 else np.arange(self.g.shape[ax]))
                    j = np.floor((c + 0.5) * self.res / fs.res).astype(np.int64) - fs.g.lo[ax]
                    idx.append(j)
                ok = [(j >= 0) & (j < n.shape[ax]) for ax, j in enumerate(idx)]
                sub = n[np.ix_(np.clip(idx[0], 0, n.shape[0] - 1), np.clip(idx[1], 0, n.shape[1] - 1),
                               np.clip(idx[2], 0, n.shape[2] - 1))] > 0
                sub &= ok[0][:, None, None] & ok[1][None, :, None] & ok[2][None, None, :]
            fr |= sub
        return fr

    def state_at(self, ci):
        """칸 좌표 (격자 기준, 격자 안) (N,3) → 상태 0/1/2."""
        a = self.g.a
        st = np.where(a["occ"][ci[:, 0], ci[:, 1], ci[:, 2]], 2,
                      np.where(a["tfree"][ci[:, 0], ci[:, 1], ci[:, 2]], 1, 0)).astype(np.uint8)
        fs = self.free_ref
        if fs is not None and not fs.g.empty:
            with fs.lock:
                j = np.floor((ci + self.g.lo + 0.5) * self.res / fs.res).astype(np.int64) - fs.g.lo
                ok = np.all((j >= 0) & (j < np.array(fs.g.shape)), axis=1)
                f = np.zeros(len(ci), bool)
                f[ok] = fs.g.a["n"][j[ok, 0], j[ok, 1], j[ok, 2]] > 0
            st[(st == 0) & f] = 1
        return st

    def state_grid(self):
        """0 모름, 1 빈 곳, 2 장애물."""
        st = self.free_grid().astype(np.uint8)
        st[self.g.a["occ"]] = 2
        return st

    # ---------------- 질의 ----------------
    def query(self, points, h=None):
        """points (N,3) m → dict(dist (N,) m (3선형), grad (N,3) (중앙 차분, 단위 없음 ≈ 장애물 반대 방향 단위벡터),
        state (N,) 0 모름 / 1 빈 곳 / 2 장애물 (가장 가까운 칸), inside (N,) 격자 안인지). 격자 밖 = d_max · 모름."""
        from scipy import ndimage
        P = np.asarray(points, np.float64).reshape(-1, 3)
        h = float(h or self.res)
        with self.lock:
            if self.g.empty:
                n = len(P)
                return dict(dist=np.full(n, self.d_max), grad=np.zeros((n, 3)), state=np.zeros(n, np.uint8),
                            inside=np.zeros(n, bool))
            D = self.g.a["dist"]
            lo = self.g.lo

            def samp(Q):
                c = (Q / self.res - 0.5 - lo).T
                return ndimage.map_coordinates(D, c, order=1, mode="nearest")
            dist = samp(P)
            grad = np.stack([(samp(P + h * e) - samp(P - h * e)) / (2 * h) for e in np.eye(3)], 1)
            ci = np.floor(P / self.res).astype(np.int64) - lo
            inside = np.all((ci >= 0) & (ci < np.array(self.g.shape)), axis=1)
            st = np.zeros(len(P), np.uint8)
            if inside.any():
                st[inside] = self.state_at(ci[inside])
            dist[~inside] = self.d_max
        return dict(dist=dist, grad=grad, state=st, inside=inside)

    # ---------------- 2D ----------------
    def floor_z(self, below=None):
        """바닥 높이 추정 (m): below (보통 카메라 최저 높이 − 0.3 m) 보다 낮은 층 중, 장애물 칸 수가 그 층들 최대의 25 % 이상인
        가장 낮은 층 (못 찾으면 None — 바닥을 못 봄). 책상만 찍은 녹화는 책상 면을 바닥으로 볼 수 있다 (추정)."""
        occ = self.g.a["occ"]
        if self.g.empty or not occ.any():
            return None
        cnt = occ.sum((0, 1)).astype(np.float64)
        zs = (self.g.lo[2] + np.arange(len(cnt)) + 0.5) * self.res
        if below is not None:
            cnt[zs >= below] = 0
        if cnt.max() < 20:
            return None
        k = np.nonzero(cnt >= 0.25 * cnt.max())[0]
        return float(zs[k[0]])

    def slice2d(self, z0=0.3, z1=1.5, floor=None, free_frac=0.3, cam_z=None):
        """floor = 바닥 높이 m (None = 자동: 카메라 최저 높이 cam_z − 0.3 m 아래에서 floor_z, 못 찾으면 cam_z − 1.0 m 로 가정,
        cam_z 도 없으면 0). 반환 dict(state2d (X,Y) u8 0/1/2, dist2d (X,Y) f32 m, origin [x, y] (칸 [0,0] 의 모서리, m), res,
        z_band [아래, 위] 월드 m, floor_z, floor_auto, floor_src = given | auto | assumed)."""
        from scipy import ndimage
        with self.lock:
            auto = floor is None
            src = "given"
            if auto:
                fz = self.floor_z(None if cam_z is None else float(cam_z) - 0.3)
                src = "auto"
                if fz is None:
                    fz, src = (float(cam_z) - 1.0 if cam_z is not None else 0.0), "assumed"
            else:
                fz = float(floor)
            if self.g.empty:
                return None
            a = int(np.floor((fz + z0) / self.res)) - self.g.lo[2]
            b = int(np.ceil((fz + z1) / self.res)) - self.g.lo[2]
            a, b = max(a, 0), min(b, self.g.shape[2])
            X, Y = self.g.shape[:2]
            if b <= a:
                st2 = np.zeros((X, Y), np.uint8)
            else:
                occ = self.g.a["occ"][:, :, a:b].any(2)
                fr = self.free_grid(a, b).mean(2) >= free_frac
                st2 = np.where(occ, 2, np.where(fr, 1, 0)).astype(np.uint8)
            o2 = st2 == 2
            d2 = (ndimage.distance_transform_edt(~o2, sampling=self.res).astype(np.float32) if o2.any()
                  else np.full(st2.shape, self.d_max, np.float32))
            np.minimum(d2, self.d_max, out=d2)
            return dict(state2d=st2, dist2d=d2, origin=[float(self.g.lo[0] * self.res), float(self.g.lo[1] * self.res)],
                        res=self.res, z_band=[fz + z0, fz + z1], floor_z=round(fz, 3), floor_auto=auto, floor_src=src,
                        cells=dict(obstacle=int((st2 == 2).sum()), free=int((st2 == 1).sum()), unknown=int((st2 == 0).sum())))

    def render_png(self, s2, traj_xy=None, px=None, crop=True):
        """slice2d 결과 → PNG 바이트 (위에서 본 지도: x 오른쪽, y 위쪽). 장애물 빨강, 빈 곳 흰색, 모름 회색, 거리 등고선,
        궤적 (선택, 파랑), 1 m 눈금 막대."""
        import cv2
        st, d2 = s2["state2d"], s2["dist2d"]
        if crop:
            nz = np.argwhere(st > 0)
            if len(nz):
                lo, hi = np.maximum(nz.min(0) - 6, 0), np.minimum(nz.max(0) + 7, st.shape)
            else:
                lo, hi = np.zeros(2, int), np.array(st.shape)
        else:
            lo, hi = np.zeros(2, int), np.array(st.shape)
        st, d2 = st[lo[0]:hi[0], lo[1]:hi[1]], d2[lo[0]:hi[0], lo[1]:hi[1]]
        res = s2["res"]
        ox, oy = s2["origin"][0] + lo[0] * res, s2["origin"][1] + lo[1] * res
        k = int(px or max(2, min(12, round(0.2 / res * 4))))
        img = np.zeros(st.shape + (3,), np.uint8)
        img[st == 0] = (150, 150, 150)
        img[st == 1] = (245, 245, 245)
        img[st == 2] = (40, 40, 210)                                   # BGR 빨강
        img = np.ascontiguousarray(np.flip(img.transpose(1, 0, 2), 0))  # 행 = y (위가 +y), 열 = x
        dd = np.ascontiguousarray(np.flip(d2.T, 0))
        img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
        ddk = cv2.resize(dd, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)
        cols = {0.25: (0, 140, 255), 0.5: (0, 200, 0), 1.0: (200, 120, 0)}
        for lv in CONTOURS:
            m = (ddk <= lv).astype(np.uint8)
            cs, _ = cv2.findContours(m, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(img, cs, -1, cols[lv], 1)
        H = img.shape[0]

        def to_px(x, y):
            return int(round((x - ox) / res * k)), int(round(H - (y - oy) / res * k))
        if traj_xy is not None and len(traj_xy) > 1:
            pts = np.array([to_px(p[0], p[1]) for p in traj_xy], np.int32)
            cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (180, 60, 0), 2)
            cv2.circle(img, tuple(int(v) for v in pts[-1]), 4, (180, 60, 0), -1)
        ox0, oy0 = to_px(0.0, 0.0)
        if 0 <= ox0 < img.shape[1] and 0 <= oy0 < H:
            cv2.drawMarker(img, (ox0, oy0), (0, 0, 0), cv2.MARKER_CROSS, 10, 1)
        L = int(round(1.0 / res * k))
        pad = 8
        img = cv2.copyMakeBorder(img, 0, 44, 0, max(0, 420 - img.shape[1]), cv2.BORDER_CONSTANT, value=(255, 255, 255))
        y = img.shape[0] - 28
        cv2.line(img, (pad, y), (pad + L, y), (0, 0, 0), 2)
        cv2.putText(img, "1 m", (pad + L + 4, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        cv2.putText(img, f"z {s2['z_band'][0]:.2f}..{s2['z_band'][1]:.2f} m  res {res * 100:.0f} cm  "
                         f"contours 0.25/0.5/1.0 m", (pad, y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        ok, buf = cv2.imencode(".png", img)
        return buf.tobytes() if ok else b""

    # ---------------- 저장 ----------------
    def save(self, d, traj_xy=None, z0=0.3, z1=1.5, floor=None):
        """traj_xy = 궤적 [(x, y[, z])] (z 가 있으면 카메라 최저 높이로 바닥 추정을 돕는다)."""
        cam_z = cam_min_z(traj_xy)
        """esdf.npz (3D: dist f16, state u8, lo, res, d_max), esdf2d.npz · esdf2d.png, esdf.json. 반환 메타 dict."""
        d = Path(d)
        d.mkdir(parents=True, exist_ok=True)
        with self.lock:
            if self.g.empty:
                return None
            st = self.state_grid()
            np.savez_compressed(d / "esdf.npz", format=np.array("scan3d-esdf/1"), dist=self.g.a["dist"].astype(np.float16),
                                state=st, lo=self.g.lo, res=np.float32(self.res), d_max=np.float32(self.d_max),
                                note=np.array("cell (i,j,k) center = (lo + [i,j,k] + 0.5) * res (world m, z-up); "
                                              "state 0 unknown 1 free 2 occupied"))
            s2 = self.slice2d(z0, z1, floor, cam_z=cam_z)
            np.savez_compressed(d / "esdf2d.npz", state2d=s2["state2d"], dist2d=s2["dist2d"].astype(np.float16),
                                origin=np.array(s2["origin"]), res=np.float32(self.res), z_band=np.array(s2["z_band"]),
                                floor_z=np.float32(s2["floor_z"]))
            (d / "esdf2d.png").write_bytes(self.render_png(s2, traj_xy))
            meta = dict(format="scan3d-esdf/1", created=time.strftime("%Y-%m-%d %H:%M:%S"), res=self.res, d_max=self.d_max,
                        lo=[int(x) for x in self.g.lo], shape=list(self.g.shape),
                        origin_m=[float(x * self.res) for x in self.g.lo],
                        counts=dict(occupied=int((st == 2).sum()), free=int((st == 1).sum()), unknown=int((st == 0).sum())),
                        slice2d=dict(z_band=s2["z_band"], floor_z=s2["floor_z"], floor_auto=s2["floor_auto"],
                                     floor_src=s2["floor_src"], cam_min_z=cam_z,
                                     origin=s2["origin"], cells=s2["cells"]),
                        last_update=self.last, mem_mb=round(self.mem_bytes() / 2 ** 20, 1),
                        free=None if self.free_ref is None else self.free_ref.stats())
            (d / "esdf.json").write_text(json.dumps(meta, indent=1))
            return meta


def cam_min_z(traj):
    """궤적 [(x, y, z)] 의 낮은 쪽 5 % 높이 (없거나 z 가 없으면 None)."""
    z = [p[2] for p in (traj or []) if len(p) > 2]
    return float(np.percentile(z, 5)) if z else None


def for_vmap(vm, res=RES, d_max=D_MAX, update=True):
    """VoxelMap 의 ESDF (vm._esdf 에 붙여 둠, 해상도가 다르면 새로). update=True 면 바뀐 것 반영."""
    with vm.lock:
        e = getattr(vm, "_esdf", None)
        if e is None or abs(e.res - res) > 1e-9 or abs(e.d_max - d_max) > 1e-9:
            e = EsdfMap(res, d_max)
            vm._esdf = e
        ts, fs = vm.tsdf, getattr(vm, "free", None)
    if update:
        e.update(ts, fs)
    return e
