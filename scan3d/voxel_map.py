"""희소 복셀 맵 + 물체 인스턴스 (월드 z-up, 단위 m).

복셀마다: 관측 프레임 수(hits), 평균 색, 클래스 투표(0 = 배경, c+1 = COCO 클래스 c), 인스턴스 투표(다수결 근사),
최근 관측 시각, 변경 버전(증분 전송용).
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
        self.version = 0
        self.epoch = getattr(self, "epoch", 0) + 1
        self.frames = 0
        self._obj_cache, self._obj_ver = [], -1

    def _grow(self, need):
        cap = len(self.pk)
        if need <= cap:
            return
        new = max(need, cap * 2)
        for name in ("pk", "rgb_sum", "npts", "hits", "cls_hist", "inst", "inst_score", "last", "ver"):
            a = getattr(self, name)
            b = np.zeros((new,) + a.shape[1:], a.dtype)
            if name == "inst":
                b[:] = -1
            b[:cap] = a
            setattr(self, name, b)

    def reset(self, voxel_m=None):
        with self.lock:
            self._alloc(voxel_m or self.voxel_m)
            self.tracker.reset()

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
        return m

    # ---- 조회 ----
    def _cls_of(self, rows):
        return self.cls_hist[rows].argmax(1).astype(np.int16) - 1

    def _rgb_of(self, rows):
        return np.clip(self.rgb_sum[rows] / np.maximum(self.npts[rows, None], 1), 0, 255).astype(np.uint8)

    def delta(self, since=0, epoch=None):
        """(epoch, version, records) — since 이후 바뀐 표시 복셀. epoch 가 다르면 전체."""
        with self.lock:
            n = self.n
            if epoch is not None and epoch != self.epoch:
                since = 0
            sel = (self.hits[:n] >= self.min_hits) & (self.ver[:n] > since)
            rows = np.nonzero(sel)[0]
            rec = np.zeros(len(rows), VOXEL_DTYPE)
            k = unpack_keys(self.pk[rows])
            rec["ix"], rec["iy"], rec["iz"] = k[:, 0], k[:, 1], k[:, 2]
            c = self._rgb_of(rows)
            rec["r"], rec["g"], rec["b"] = c[:, 0], c[:, 1], c[:, 2]
            rec["cls"] = self._cls_of(rows)
            rec["inst"] = self.inst[rows]
            rec["hits"] = np.minimum(self.hits[rows], 65535)
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
        """voxels.ply (중심점), voxels.json, voxels.npz, objects.json, viewer.html (write_viewer)."""
        from lib_pointcloud_export import write_ply
        from lib_pointcloud_view import write_viewer
        d = Path(out_dir); d.mkdir(parents=True, exist_ok=True)
        pts, rgb, cls, inst = self.points()
        write_ply(d / "voxels.ply", pts, rgb)
        epoch, ver, rec = self.delta(0)
        np.savez_compressed(d / "voxels.npz", centers=pts, rgb=rgb, cls=cls, inst=inst, records=rec,
                            voxel_m=np.float32(self.voxel_m), min_hits=np.int32(self.min_hits))
        (d / "voxels.json").write_text(json.dumps(dict(
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
