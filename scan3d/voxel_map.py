"""희소 복셀 맵 + 물체 인스턴스 (월드 z-up, 단위 m).

복셀마다: 관측 프레임 수(hits), 평균 색, 클래스 투표(0 = 배경, c+1 = COCO 클래스 c), 인스턴스 투표(다수결 근사),
최근 관측 시각, 변경 버전(증분 전송용).
시간 정보 (복원 리플레이, scan3d-voxels/2): t_first = 처음 관측된 시각, t_shown·kf_shown = 표시 기준(hits ≥ min_hits)을
처음 넘은 시각·키프레임 번호(frames, 1부터). 웹 17 B 레코드는 그대로 두고 save() 가 voxels.npz 에 따로 넣는다.
물체 = 표시 복셀(hits ≥ min_hits) 중 같은 클래스의 26-이웃 연결 성분 → xy PCA yaw 박스, 부피, 라벨.
"""
import json
import threading
import time
from pathlib import Path

import numpy as np

N_CLS = 81                    # 배경 + COCO 80
_OFF = 1 << 20                # 키 패킹 오프셋 (±2^20 복셀)

# ultralytics Colors() 팔레트 순서 (utils/plotting.py)
_HEX = ("042AFF", "0BDBEB", "F3F3F3", "00DFB7", "111F68", "FF6FDD", "FF444F", "CCED00", "00F344", "BD00FF",
        "00B4FF", "DD00BA", "00FFFF", "26C000", "01FFB3", "7D24FF", "7B0068", "FF1B6C", "FC6D2F", "A2FF0B")
PALETTE = [tuple(int(h[i:i + 2], 16) for i in (0, 2, 4)) for h in _HEX]


def class_color(c):
    return PALETTE[int(c) % len(PALETTE)] if c >= 0 else (150, 150, 150)


def pack_keys(k):
    k = k.astype(np.int64) + _OFF
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def unpack_keys(p):
    m = (1 << 21) - 1
    return np.stack([(p >> 42) & m, (p >> 21) & m, p & m], axis=1).astype(np.int64) - _OFF


VOXEL_DTYPE = np.dtype([("ix", "<i2"), ("iy", "<i2"), ("iz", "<i2"), ("r", "u1"), ("g", "u1"), ("b", "u1"),
                        ("cls", "<i2"), ("inst", "<i4"), ("hits", "<u2")])          # 17 B (웹 바이너리 형식)
VOXEL_FORMAT = "scan3d-voxels/2"          # /1 = 17 B 레코드만, /2 = + 시간 배열 (t_first, t_shown, kf_shown)

# TSDF 융합 지도 (tsdf_map.py) 설정 — VoxelMap 마다 하나 (리셋·조각 교체 때 같이 바뀜). 첫 깊이 적분 때 만든다 (대기 중 GPU 안 씀).
# configure_tsdf(voxel_m=, z_min=, z_max=, enabled=, ...) 로 바꾸면 다음에 만드는 TSDF 부터 적용 (지금 것은 reset_tsdf()).
TSDF_CFG = dict(enabled=True)


def configure_tsdf(**kw):
    TSDF_CFG.update({k: v for k, v in kw.items() if v is not None})
    return dict(TSDF_CFG)


def tsdf_cfg():
    import tsdf_map
    c = dict(tsdf_map.DEFAULTS)
    c.update({k: v for k, v in TSDF_CFG.items() if k in c})
    return dict(c, enabled=bool(TSDF_CFG.get("enabled", True)))


class InstanceTracker:
    """프레임 마스크 → 전역 인스턴스 id. 2D 트랙 id 우선, 아니면 같은 클래스 중심 거리 < dist 또는 3D IoU > iou."""

    def __init__(self, dist_m=0.3, iou=0.2):
        self.dist, self.iou = dist_m, iou
        self.items = {}            # id → dict(cls, center, lo, hi, n, last)
        self.track_map = {}        # (cls, track_id) → id
        self.next_id = 1

    def reset(self):
        self.__init__(self.dist, self.iou)

    @staticmethod
    def _iou(lo1, hi1, lo2, hi2):
        inter = np.clip(np.minimum(hi1, hi2) - np.maximum(lo1, lo2), 0, None).prod()
        v = (hi1 - lo1).prod() + (hi2 - lo2).prod() - inter
        return float(inter / v) if v > 0 else 0.0

    def associate(self, cls, pts, track_id=None, t=0.0):
        if len(pts) < 10:
            return -1
        lo, hi = np.percentile(pts, 5, axis=0), np.percentile(pts, 95, axis=0)
        c = np.median(pts, axis=0)
        iid = self.track_map.get((cls, track_id)) if track_id is not None else None
        if iid is None or iid not in self.items:
            best, best_s = None, 0.0
            for k, it in self.items.items():
                if it["cls"] != cls:
                    continue
                d = float(np.linalg.norm(it["center"] - c))
                iou = self._iou(lo, hi, it["lo"], it["hi"])
                if d < self.dist or iou > self.iou:
                    s = iou + (self.dist - d)
                    if best is None or s > best_s:
                        best, best_s = k, s
            iid = best
        if iid is None:
            iid = self.next_id; self.next_id += 1
            self.items[iid] = dict(cls=cls, center=c, lo=lo, hi=hi, n=0, last=t)
        it = self.items[iid]
        a = 0.3 if it["n"] else 1.0                          # 지수 평균으로 위치 갱신
        it["center"] = (1 - a) * it["center"] + a * c
        it["lo"] = (1 - a) * it["lo"] + a * lo
        it["hi"] = (1 - a) * it["hi"] + a * hi
        it["n"] += 1; it["last"] = t
        if track_id is not None:
            self.track_map[(cls, track_id)] = iid
        return iid


class VoxelMap:
    def __init__(self, voxel_m=0.03, min_hits=2, names=None):
        self.lock = threading.RLock()
        self.names = names or {}
        self.min_hits = min_hits
        self.tracker = InstanceTracker()
        self._alloc(voxel_m)

    def _alloc(self, voxel_m):
        self.voxel_m = float(voxel_m)
        self.index = {}                                   # 패킹 키 → 행
        cap = 1 << 16
        self.n = 0
        self.pk = np.zeros(cap, np.int64)
        self.rgb_sum = np.zeros((cap, 3), np.float32)
        self.npts = np.zeros(cap, np.float32)
        self.hits = np.zeros(cap, np.int32)
        self.cls_hist = np.zeros((cap, N_CLS), np.uint8)
        self.inst = np.full(cap, -1, np.int32)
        self.inst_score = np.zeros(cap, np.int16)
        self.last = np.zeros(cap, np.float32)
        self.ver = np.zeros(cap, np.uint32)
        self.t_first = np.zeros(cap, np.float32)
        self.t_shown = np.full(cap, np.inf, np.float32)
        self.kf_shown = np.full(cap, -1, np.int32)
        self.version = 0
        self.epoch = getattr(self, "epoch", 0) + 1
        self.frames = 0
        self._obj_cache, self._obj_ver = [], -1
        self._tsdf = None                                 # TSDF 지도 (첫 integrate_depth 때 만듦)

    def _grow(self, need):
        cap = len(self.pk)
        if need <= cap:
            return
        new = max(need, cap * 2)
        for name in ("pk", "rgb_sum", "npts", "hits", "cls_hist", "inst", "inst_score", "last", "ver",
                     "t_first", "t_shown", "kf_shown"):
            a = getattr(self, name)
            b = np.zeros((new,) + a.shape[1:], a.dtype)
            if name in ("inst", "kf_shown"):
                b[:] = -1
            elif name == "t_shown":
                b[:] = np.inf
            b[:cap] = a
            setattr(self, name, b)

    def reset(self, voxel_m=None):
        with self.lock:
            self._alloc(voxel_m or self.voxel_m)
            self.tracker.reset()

    # ---- TSDF (메시) ----
    @property
    def tsdf(self):
        """지금 TSDF 지도 (없으면 None — 아직 깊이를 안 넣었거나 꺼짐)."""
        return self._tsdf

    def reset_tsdf(self):
        with self.lock:
            self._tsdf = None

    def integrate_depth(self, Z, valid, bgr, rp, T_world_cam, ffs_scale=1.0, t=0.0, epoch=None, log=None):
        """키프레임 깊이 지도 → TSDF. rp = 그 깊이의 정류 (f, cx, cy, baseline_m), T_world_cam = 그 정류 카메라 → 월드
        (움직임 보정 정류면 T_world_rect0 · R_fix). epoch 가 다르면 버림. 반환 tsdf_map.integrate 결과 또는 None."""
        with self.lock:
            if epoch is not None and epoch != self.epoch:
                return None
            if self._tsdf is None:
                c = tsdf_cfg()
                if not c.pop("enabled"):
                    return None
                import tsdf_map
                self._tsdf = tsdf_map.TSDFMap(log=log, **c)
            ts = self._tsdf
            return ts.integrate(Z, valid, bgr, rp["f"], rp["cx"], rp["cy"], rp["baseline_m"], T_world_cam,
                                ffs_scale=ffs_scale, t=t)

    def merge(self, other, T=None, keep_time=False):
        """재위치 '조각 잇기': 다른 VoxelMap 을 T (4x4, other 월드 → self 월드, None = 단위) 로 옮겨 합침.
        복셀: 표시 기준과 상관없이 모든 복셀의 중심을 옮겨 그 관측 수(hits)·색 합·클래스 투표를 더함 (새 키프레임 하나로 셈).
        keep_time=True: other 의 키프레임 번호를 그대로 이어 붙임 (self.frames 를 other.frames 만큼 늘리고, 새로 표시 기준을
        넘은 복셀의 kf_shown·t_shown 은 other 의 값 + 옛 self.frames) → 리플레이 시간 정보에서 조각 키프레임 i 는 옛 frames + i.
        TSDF: tsdf_map.TSDFMap.merge (가중 평균). 반환 dict(voxels=합친 복셀 행 수, tsdf=합친 TSDF 복셀 수, kf_base=옛 frames)."""
        with other.lock:
            n = other.n
            centers = (unpack_keys(other.pk[:n]) + 0.5) * other.voxel_m
            rgb_sum, npts, hits = other.rgb_sum[:n].copy(), other.npts[:n].copy(), other.hits[:n].copy()
            ch = other.cls_hist[:n].copy()
            inst, t_first, last = other.inst[:n].copy(), other.t_first[:n].copy(), other.last[:n].copy()
            o_kf, o_ts, o_frames = other.kf_shown[:n].copy(), other.t_shown[:n].copy(), int(other.frames)
            ots = other._tsdf
        if T is not None:
            T = np.asarray(T, np.float64).reshape(4, 4)
            centers = centers @ T[:3, :3].T + T[:3, 3]
        out = dict(voxels=0, tsdf=0, kf_base=None)
        with self.lock:
            out["kf_base"] = base = int(self.frames)
            if n:
                keys = np.floor(centers / self.voxel_m).astype(np.int64)
                ok = np.all(np.abs(keys) < 32000, axis=1)
                keys = keys[ok]
                pk = pack_keys(keys)
                rows = np.empty(len(pk), np.int64)
                new = 0
                for j, key in enumerate(pk.tolist()):
                    r = self.index.get(key)
                    if r is None:
                        r = self.n + new
                        self.index[key] = r
                        new += 1
                    rows[j] = r
                self._grow(self.n + new)
                if new:
                    nr = rows[rows >= self.n]
                    self.pk[nr] = pk[rows >= self.n]
                    self.t_first[nr] = t_first[ok][rows >= self.n]
                    self.n += new
                self.version += 1
                self.frames += max(1, o_frames) if keep_time else 1
                np.add.at(self.rgb_sum, rows, rgb_sum[ok])
                np.add.at(self.npts, rows, npts[ok])
                np.add.at(self.hits, rows, hits[ok])
                h = self.cls_hist[rows].astype(np.int32)
                np.add.at(h, np.arange(len(rows)), ch[ok].astype(np.int32))
                self.cls_hist[rows] = np.minimum(h, 255).astype(np.uint8)
                keep = self.inst[rows] < 0
                self.inst[rows[keep]] = inst[ok][keep]
                self.last[rows] = np.maximum(self.last[rows], last[ok])
                self.ver[rows] = self.version
                fm = (self.kf_shown[rows] < 0) & (self.hits[rows] >= self.min_hits)
                fresh = rows[fm]
                if keep_time:
                    ok_kf, ok_ts = o_kf[ok][fm], o_ts[ok][fm]
                    has = ok_kf >= 0
                    self.kf_shown[fresh] = np.where(has, base + ok_kf, self.frames)
                    self.t_shown[fresh] = np.where(has, ok_ts, self.last[fresh])
                else:
                    self.t_shown[fresh] = self.last[fresh]
                    self.kf_shown[fresh] = self.frames
                out["voxels"] = int(len(rows))
            if ots is not None:
                if self._tsdf is None:
                    import tsdf_map
                    self._tsdf = tsdf_map.TSDFMap(device=ots.dev, log=ots.log, **ots.cfg)
                out["tsdf"] = self._tsdf.merge(ots, T)
        return out

    def set_min_hits(self, m):
        with self.lock:
            self.min_hits = int(m)
            self.epoch += 1                                   # 뷰어가 전체 다시 받게

    def integrate(self, pts, rgb, cls, inst, t=0.0, epoch=None):
        """pts (N,3) f32 월드, rgb (N,3) u8, cls (N,) i16 (-1 배경), inst (N,) i32 (-1).

        epoch: 키프레임을 만들 때의 맵 epoch. 그 사이 리셋·복셀 크기 변경이 있었으면 버린다 (REVIEW 8).
        키 계산부터 끝까지 lock 안에서 한다 (옛 voxel_m 으로 계산한 키가 새 맵에 들어가지 않게).
        """
        if len(pts) == 0:
            return 0
        with self.lock:
            if epoch is not None and epoch != self.epoch:
                return 0
            return self._integrate_locked(pts, rgb, cls, inst, t)

    def _integrate_locked(self, pts, rgb, cls, inst, t):
        keys = np.floor(pts / self.voxel_m).astype(np.int64)
        ok = np.all(np.abs(keys) < 32000, axis=1)              # int16 전송 범위
        keys, rgb, cls, inst = keys[ok], rgb[ok], cls[ok], inst[ok]
        pk = pack_keys(keys)
        u, inv = np.unique(pk, return_inverse=True)
        m = len(u)
        csum = np.zeros((m, 3), np.float32)
        np.add.at(csum, inv, rgb.astype(np.float32))
        cnt = np.bincount(inv, minlength=m).astype(np.float32)
        ch = np.zeros((m, N_CLS), np.int32)
        np.add.at(ch, (inv, cls.astype(np.int64) + 1), 1)
        frame_cls = ch.argmax(1)                               # 이 프레임에서 복셀의 대표 클래스
        # 프레임 내 대표 인스턴스: inst ≥ 0 인 점 중 최빈 (점 수가 적으니 정렬로 처리)
        frame_inst = np.full(m, -1, np.int32)
        sel = inst >= 0
        if sel.any():
            pairs = np.unique(np.stack([inv[sel], inst[sel]], 1), axis=0, return_counts=True)
            (vi, ii), c = pairs[0].T, pairs[1]
            order = np.lexsort((-c, vi))
            vi, ii = vi[order], ii[order]
            first = np.r_[True, vi[1:] != vi[:-1]]
            frame_inst[vi[first]] = ii[first]
        rows = np.empty(m, np.int64)
        idx = self.index
        new = 0
        for j, key in enumerate(u.tolist()):
            r = idx.get(key)
            if r is None:
                r = self.n + new
                idx[key] = r
                new += 1
            rows[j] = r
        self._grow(self.n + new)
        if new:
            nr = rows[rows >= self.n]
            self.pk[nr] = u[rows >= self.n]
            self.t_first[nr] = t
            self.n += new
        self.version += 1
        self.frames += 1
        self.rgb_sum[rows] += csum
        self.npts[rows] += cnt
        self.hits[rows] += 1
        h = self.cls_hist[rows].astype(np.int32)
        h[np.arange(m), frame_cls] += 1
        sat = h.max(1) >= 255
        h[sat] //= 2                                       # 포화 시 반으로 (최근 관측 가중)
        self.cls_hist[rows] = h.astype(np.uint8)
        # 인스턴스 투표 (Boyer-Moore 근사)
        fi = frame_inst
        has = fi >= 0
        cur, sc = self.inst[rows], self.inst_score[rows].astype(np.int32)
        same = has & (cur == fi)
        diff = has & ~same
        sc[same] += 1
        sc[diff] -= 1
        repl = diff & (sc <= 0)
        cur[repl] = fi[repl]; sc[repl] = 1
        self.inst[rows] = cur
        self.inst_score[rows] = np.clip(sc, 0, 1000).astype(np.int16)
        self.last[rows] = t
        self.ver[rows] = self.version
        fresh = rows[(self.kf_shown[rows] < 0) & (self.hits[rows] >= self.min_hits)]
        if len(fresh):
            self.t_shown[fresh] = t
            self.kf_shown[fresh] = self.frames
        return m

    # ---- 조회 ----
    def _cls_of(self, rows):
        return self.cls_hist[rows].argmax(1).astype(np.int16) - 1

    def _rgb_of(self, rows):
        return np.clip(self.rgb_sum[rows] / np.maximum(self.npts[rows, None], 1), 0, 255).astype(np.uint8)

    def delta(self, since=0, epoch=None, with_time=False):
        """(epoch, version, records) — since 이후 바뀐 표시 복셀. epoch 가 다르면 전체.
        with_time=True 면 (epoch, version, records, times) — times = dict(t_first, t_shown, kf_shown) 같은 순서
        (min_hits 를 낮춰 표시 기준을 나중에 넘은 복셀은 t_shown 대신 t_first, kf_shown 은 0)."""
        with self.lock:
            n = self.n
            if epoch is not None and epoch != self.epoch:
                since = 0
            sel = (self.hits[:n] >= self.min_hits) & (self.ver[:n] > since)
            rows = np.nonzero(sel)[0]
            times = None
            if with_time:
                ks = self.kf_shown[rows].copy()
                ts = self.t_shown[rows].copy()
                miss = ks < 0
                ts[miss] = self.t_first[rows][miss]
                ks[miss] = 0
                times = dict(t_first=self.t_first[rows].copy(), t_shown=ts, kf_shown=ks)
            rec = np.zeros(len(rows), VOXEL_DTYPE)
            k = unpack_keys(self.pk[rows])
            rec["ix"], rec["iy"], rec["iz"] = k[:, 0], k[:, 1], k[:, 2]
            c = self._rgb_of(rows)
            rec["r"], rec["g"], rec["b"] = c[:, 0], c[:, 1], c[:, 2]
            rec["cls"] = self._cls_of(rows)
            rec["inst"] = self.inst[rows]
            rec["hits"] = np.minimum(self.hits[rows], 65535)
            if with_time:
                return self.epoch, self.version, rec, times
            return self.epoch, self.version, rec

    def count_shown(self):
        with self.lock:
            return int((self.hits[:self.n] >= self.min_hits).sum())

    def points(self):
        """표시 복셀 중심점 (M,3), 색, 클래스, 인스턴스."""
        with self.lock:
            rows = np.nonzero(self.hits[:self.n] >= self.min_hits)[0]
            centers = (unpack_keys(self.pk[rows]) + 0.5) * self.voxel_m
            return centers.astype(np.float32), self._rgb_of(rows), self._cls_of(rows), self.inst[rows].copy()

    def objects(self, min_voxels=8, max_age_s=None, force=False):
        """같은 클래스 표시 복셀의 26-이웃 연결 성분 → 물체 목록 (버전이 같으면 캐시)."""
        with self.lock:
            if not force and self._obj_ver == self.version:
                return self._obj_cache
            ver = self.version
            rows = np.nonzero(self.hits[:self.n] >= self.min_hits)[0]
            cls = self._cls_of(rows)
            fg = cls >= 0
            rows, cls = rows[fg], cls[fg]
            keys = unpack_keys(self.pk[rows])
            inst = self.inst[rows].copy()
            last = self.last[rows].copy()
            hits = self.hits[rows].copy()
            vm = self.voxel_m
        out = []
        if len(rows):
            from scipy.sparse import coo_matrix
            from scipy.sparse.csgraph import connected_components
            # 지역 키(최소값 기준) + 클래스를 한 정수로 → 다른 클래스끼리는 이웃이 안 된다
            kk = keys - keys.min(0) + 1
            kk = np.minimum(kk, (1 << 17) - 2)
            c1 = cls.astype(np.int64) + 1

            def lpack(k):
                return ((k[:, 0] << 34) | (k[:, 1] << 17) | k[:, 2]) * 128 + c1

            pk = lpack(kk)
            order = np.argsort(pk)
            spk = pk[order]
            src, dst = [], []
            offs = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                    if (dx, dy, dz) > (0, 0, 0)]                # 13 방향 (나머지는 대칭)
            for o in offs:
                nk = lpack(kk + np.array(o))
                pos = np.searchsorted(spk, nk)
                pos = np.minimum(pos, len(spk) - 1)
                hit = spk[pos] == nk
                src.append(np.nonzero(hit)[0]); dst.append(order[pos[hit]])
            src, dst = np.concatenate(src), np.concatenate(dst)
            n = len(rows)
            g = coo_matrix((np.ones(len(src), np.int8), (src, dst)), shape=(n, n))
            ncomp, lab = connected_components(g, directed=False)
            counts = np.bincount(lab, minlength=ncomp)
            for comp in np.nonzero(counts >= min_voxels)[0]:
                sel = lab == comp
                c = int(cls[sel][0])
                centers = (keys[sel] + 0.5) * vm
                ii = inst[sel]; ii = ii[ii >= 0]
                oid = int(np.bincount(ii).argmax()) if len(ii) else -1
                box = _pca_box(centers, vm)
                out.append(dict(id=oid, comp=int(comp), class_id=c, label=self.names.get(c, str(c)),
                                color=list(class_color(c)), n_voxels=int(sel.sum()),
                                volume_m3=float(sel.sum() * vm ** 3), last_seen=float(last[sel].max()),
                                conf=float(np.clip(hits[sel].mean() / 10.0, 0, 1)), **box))
            # 같은 인스턴스 id 가 여러 성분이면 가장 큰 것만 id 유지
            out.sort(key=lambda o: -o["n_voxels"])
            seen = set()
            for o in out:
                if o["id"] in seen or o["id"] < 0:
                    o["id"] = 100000 + o["comp"]
                seen.add(o["id"])
        with self.lock:
            self._obj_cache, self._obj_ver = out, ver
        return out

    def save(self, out_dir, trajectory=None, extra=None):
        """voxels.ply (중심점), voxels.json, voxels.npz, objects.json, viewer.html (write_viewer),
        TSDF 가 있으면 mesh.ply · mesh_web.bin · mesh.json."""
        from lib_pointcloud_export import write_ply
        from lib_pointcloud_view import write_viewer
        d = Path(out_dir); d.mkdir(parents=True, exist_ok=True)
        pts, rgb, cls, inst = self.points()
        write_ply(d / "voxels.ply", pts, rgb)
        epoch, ver, rec, tm = self.delta(0, with_time=True)
        # records·centers 는 같은 순서 (hits ≥ min_hits 행 순). t_* 도 records 와 같은 순서 (맵 시각 = 첫 IMU 기준 초)
        np.savez_compressed(d / "voxels.npz", centers=pts, rgb=rgb, cls=cls, inst=inst, records=rec,
                            voxel_m=np.float32(self.voxel_m), min_hits=np.int32(self.min_hits),
                            format=np.array(VOXEL_FORMAT), **tm)
        (d / "voxels.json").write_text(json.dumps(dict(
            format=VOXEL_FORMAT, time_fields="voxels.npz: t_first, t_shown, kf_shown (records 순서); 복원이면 voxels_t.bin",
            voxel_m=self.voxel_m, version=ver, min_hits=self.min_hits, created=time.strftime("%Y-%m-%d %H:%M:%S"),
            fields=list(VOXEL_DTYPE.names), voxels=[list(map(int, r)) for r in rec.tolist()],
            classes={int(k): v for k, v in self.names.items()}), separators=(",", ":")))
        objs = self.objects(force=True)
        (d / "objects.json").write_text(json.dumps(objs, indent=1))
        # write_viewer: 물체 순서대로 점을 이어 붙이고 나머지는 배경 하나로
        order_pts, order_rgb, meta_objs = [], [], []
        used = np.zeros(len(pts), bool)
        for o in objs:
            sel = (cls == o["class_id"]) & (np.linalg.norm(pts - np.array(o["box_center"]), axis=1)
                                            <= np.linalg.norm(o["box_size"]) / 2 + self.voxel_m) & ~used
            used |= sel
            order_pts.append(pts[sel]); order_rgb.append(rgb[sel])
            meta_objs.append(dict(id=o["id"], label=o["label"], points=int(sel.sum()), center_m=o["box_center"]))
        order_pts.append(pts[~used]); order_rgb.append(rgb[~used])
        meta_objs.append(dict(id=-1, label="background", points=int((~used).sum()), center_m=[0, 0, 0]))
        traj = [dict(T_world_camera=np.asarray(T).reshape(4, 4).tolist()) for T in (trajectory or [])]
        write_viewer(d / "viewer.html", np.concatenate(order_pts) if len(pts) else np.zeros((0, 3)),
                     np.concatenate(order_rgb) if len(pts) else np.zeros((0, 3)),
                     dict(objects=meta_objs, trajectory=traj))
        ts = self._tsdf
        if ts is not None:
            # 최종 메시 (TSDF marching cubes): mesh.ply (색 포함), mesh_web.bin (웹 리플레이 '최종 맵' 메시), mesh.json (숫자)
            import tsdf_map
            V, C, F = ts.mesh_arrays()
            tsdf_map.write_mesh_ply(d / "mesh.ply", V, C, F)
            (d / "mesh_web.bin").write_bytes(tsdf_map.mesh_bin_static(V, C, F, ts.vm))
            (d / "mesh.json").write_text(json.dumps(dict(ts.stats(), cfg=ts.cfg, w_min=ts.w_min,
                                                         created=time.strftime("%Y-%m-%d %H:%M:%S")), indent=1))
        if extra:
            (d / "summary.json").write_text(json.dumps(extra, indent=1, default=str))
        return d


def _pca_box(centers, vm):
    """xy PCA 로 yaw, 2~98 백분위 크기, z 는 축 정렬."""
    c = centers.mean(0)
    xy = centers[:, :2] - c[:2]
    yaw = 0.0
    if len(centers) >= 3:
        cov = xy.T @ xy / len(xy)
        w, v = np.linalg.eigh(cov)
        a = v[:, 1]
        yaw = float(np.arctan2(a[1], a[0]))
    cy, sy = np.cos(-yaw), np.sin(-yaw)
    loc = np.stack([cy * xy[:, 0] - sy * xy[:, 1], sy * xy[:, 0] + cy * xy[:, 1], centers[:, 2]], 1)
    lo, hi = np.percentile(loc, 2, axis=0), np.percentile(loc, 98, axis=0)
    size = np.maximum(hi - lo + vm, vm)
    mid = (lo + hi) / 2
    cyaw, syaw = np.cos(yaw), np.sin(yaw)
    center = [float(c[0] + cyaw * mid[0] - syaw * mid[1]), float(c[1] + syaw * mid[0] + cyaw * mid[1]), float(mid[2])]
    return dict(box_center=center, box_size=[float(s) for s in size], yaw=yaw)
