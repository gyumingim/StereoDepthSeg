"""재위치(relocalization): VIO 가 리셋돼 새 월드 좌표로 다시 시작하면, 새 키프레임 영상을 옛 지도 키프레임과
특징 매칭 → PnP + RANSAC 으로 새 좌표 → 옛(전역) 좌표 변환 T_old_new 를 구한다 (2026-10-17).

- 영상: 렌즈 2 를 왜곡만 편 lens0u (mono VIO 입력과 같은 넓은 화각), 회색, 절반 크기 (SIFT).
  넓은 화각이라 2+6 쌍(망원 정류)이라도 같은 장소를 다시 볼 확률이 높다.
- 옛 키프레임(DB): 깊이(FFS)가 계산된 전역 좌표 키프레임만. 깊이 점을 lens0u 로 투영한 희소 깊이에서 특징점마다 깊이 →
  전역 3D 점. 같은 자리 키프레임은 하나만 (3 cm·3° 안이면 건너뜀), 최대 MAX_DB 개.
- 매칭: 모든 DB 기술자를 한 FLANN 색인에. 비율 검사는 '3D 로 다른 점' 과만 비교 (같은 점이 여러 키프레임에 있어도
  애매하다고 버리지 않게).
- 판정: PnP 내점 수 ≥ min_inliers, 내점 비율, 재투영 오차 중앙값 ≤ max_reproj_px, 두 월드 모두 중력 정렬(z 위)이므로
  T_old_new 의 기울기(롤·피치) ≤ max_tilt_deg. 서로 다른 새 키프레임 min_agree 개가 같은 변환(회전·이동 차이 기준 안)을
  주면 받아들인다 (한 장 우연 매칭으로 잘못 붙는 것 방지). 최종 변환은 기울기를 뺀 yaw + 이동: 가중 평균에서 시작해
  합의한 키프레임들의 모든 내점으로 4 자유도 재투영 최소제곱 (_refine4).
- 축척: 모노 VIO 도 IMU 로 미터 축척이라 SE(3) 로 붙인다. 합의한 키프레임 쌍의 카메라 거리 비(옛/새)는 로그만.
- 확인: 붙인 뒤 새 키프레임 깊이 점이 옛 지도 점(표본)과 10 cm 안에 드는 비율·최근접 거리 중앙값 (_check),
  같은 값을 리셋 없는 평소 키프레임에서도 재서(seg "base") 비교.

스레드: 무거운 일(SIFT·FLANN·PnP)은 이 모듈의 작업 스레드에서. 결과는 on_aligned(seg, T, info) 콜백.
"""
import queue
import threading

import cv2
import numpy as np

MAX_DB = 400


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, float).reshape(3)
    return T


def inv_T(T):
    R, p = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ p)


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def tilt_deg(R):
    return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


def T_yaw(yaw, p):
    c, s = np.cos(yaw), np.sin(yaw)
    return make_T(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]), p)


class Relocalizer:
    # 문턱 (2026-10-17 시험 분할 3개 녹화): 한 장 판정은 느슨하게(내점 30, 비율 0.15, 재투영 중앙 2.5 px, 기울기 4°),
    # 잘못 붙는 것은 '서로 다른 키프레임 2장이 8 cm·3° 안에서 같은 변환' 합의로 막는다. 2.0 px·0.25 이면 2+6 녹화
    # (162204) 질의 58개 중 45개가 재투영·비율에서 떨어져 끝까지 못 붙음.
    def __init__(self, log, K_lens0u, maps, min_inliers=30, min_inlier_frac=0.15, max_reproj_px=2.5, max_tilt_deg=4.0,
                 min_agree=2, agree_rot_deg=3.0, agree_trans_m=0.08, on_aligned=None):
        self.log = log
        # 특징 영상 폭 약 720 px: 1440 녹화는 절반, 640 녹화는 그대로 (640 을 절반으로 줄이면 특징·내점이 너무 적음 —
        # rec_20261016_143702 시험 분할에서 내점 30~44, 정답과 11 cm 차이)
        W = int(maps[0].shape[1])
        self.scale = min(1.0, 720.0 / max(1, W))
        self.K = np.asarray(K_lens0u, float).copy()
        self.K[:2] *= self.scale
        self.Kinv = np.linalg.inv(self.K)
        self.maps = maps
        self.min_inliers, self.min_inlier_frac, self.max_reproj = min_inliers, min_inlier_frac, max_reproj_px
        self.max_tilt, self.min_agree = max_tilt_deg, min_agree
        self.agree_rot, self.agree_trans = agree_rot_deg, agree_trans_m
        self.on_aligned = on_aligned
        self.sift = cv2.SIFT_create(nfeatures=3000)
        self.db = []                 # dict(t, C (cam 중심), R, desc, X)
        self.cloud = []              # 전역 깊이 점 표본 (확인용)
        self._index = None
        self._dirty = True
        self.seg = None              # 지금 맞추는 조각 번호 (None = 붙일 것 없음)
        self.cands = []
        self.stats = dict(db=0, db_skipped=0, queries=0, pnp_ok=0, rejected={}, alignments=[], checks=[])
        self.q = queue.Queue(maxsize=64)
        self.busy = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True, name="reloc")
        self.thread.start()

    # ---------------- 공개 ----------------
    def clear(self):
        with self.lock:
            self.db, self.cloud, self._index, self._dirty = [], [], None, True
            self.seg, self.cands = None, []
            self.stats = dict(db=0, db_skipped=0, queries=0, pnp_ok=0, rejected={}, alignments=[], checks=[])
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass

    def begin_segment(self, seg):
        with self.lock:
            self.seg, self.cands = seg, []

    def end_segment(self):
        with self.lock:
            self.seg, self.cands = None, []

    def n_db(self):
        return len(self.db)

    def submit(self, kind, **kw):
        try:
            self.q.put_nowait((kind, kw))
            return True
        except queue.Full:
            self._rej("queue_full")
            return False

    def gray(self, bgr):
        g = cv2.cvtColor(cv2.remap(bgr, *self.maps, interpolation=cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY)
        if self.scale >= 1.0:
            return g
        return cv2.resize(g, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)

    def summary(self):
        s = dict(self.stats)
        s["rejected"] = dict(s["rejected"])
        s["db"] = len(self.db)
        return s

    # ---------------- 작업 스레드 ----------------
    def _rej(self, why):
        r = self.stats["rejected"]
        r[why] = r.get(why, 0) + 1

    def _run(self):
        while True:
            kind, kw = self.q.get()
            self.busy = True
            try:
                if kind == "db":
                    self._add_db(**kw)
                elif kind == "query":
                    self._query(**kw)
                elif kind == "check":
                    self._check(**kw)
            except Exception as e:
                self.log(f"[reloc] {kind} 오류 {e!r}")
            finally:
                self.busy = False

    def _add_db(self, bgr, T_w_c1, Z, valid, f, cx, cy, R_c1_d, t):
        T_w_c1 = np.asarray(T_w_c1, float)
        C, R = T_w_c1[:3, 3], T_w_c1[:3, :3]
        for e in self.db[-60:]:
            if np.linalg.norm(e["C"] - C) < 0.03 and rot_deg(e["R"].T @ R) < 3.0:
                self.stats["db_skipped"] += 1
                return
        g = self.gray(bgr)
        H, W = g.shape
        # 깊이 점 (깊이 카메라 = 쌍별 정류 rect0') → lens0u(cam1) → 절반 크기 영상으로 투영 → 희소 깊이 (가장 가까운 값)
        vs, us = np.nonzero(valid[::2, ::2])
        vs, us = vs * 2, us * 2
        z = Z[vs, us].astype(np.float64)
        P = np.stack([(us - cx) / f * z, (vs - cy) / f * z, z], 1) @ np.asarray(R_c1_d, float).T
        P = P[P[:, 2] > 0.1]
        uv = P @ self.K.T
        u = np.round(uv[:, 0] / uv[:, 2]).astype(np.int64)
        v = np.round(uv[:, 1] / uv[:, 2]).astype(np.int64)
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u, v, zc = u[ok], v[ok], P[ok, 2]
        D = np.full(H * W, np.inf)
        np.minimum.at(D, v * W + u, zc)
        D = D.reshape(H, W)
        Dm = cv2.erode(np.where(np.isfinite(D), D, 1e9).astype(np.float32), np.ones((3, 3), np.uint8))
        Dx = cv2.dilate(np.where(np.isfinite(D), D, 0).astype(np.float32), np.ones((3, 3), np.uint8))
        kps, desc = self.sift.detectAndCompute(g, None)
        if desc is None or not kps:
            return
        pts = np.float32([k.pt for k in kps])
        iu = np.clip(np.round(pts[:, 0]).astype(int), 0, W - 1)
        iv = np.clip(np.round(pts[:, 1]).astype(int), 0, H - 1)
        zmin, zmax = Dm[iv, iu], Dx[iv, iu]
        good = (zmin < 1e8) & (zmax > 0) & (zmax - zmin < 0.08 * zmin)     # 3x3 안 깊이가 고르게 있을 때만 (경계 제외)
        if good.sum() < 20:
            self._rej("db_few_depth")
            return
        zz = zmin[good].astype(np.float64)
        X = (np.c_[pts[good], np.ones(good.sum())] @ self.Kinv.T) * zz[:, None]
        Xw = X @ R.T + C
        with self.lock:
            self.db.append(dict(t=t, C=C, R=R, desc=desc[good].astype(np.float32), X=Xw))
            if len(self.db) > MAX_DB:
                self.db.pop(0)
            sel = np.random.default_rng(len(self.db)).choice(len(P), min(len(P), 1500), replace=False)
            self.cloud.append(P[sel] @ R.T + C)
            if len(self.cloud) > MAX_DB:
                self.cloud.pop(0)
            self._dirty = True
            self.stats["db"] = len(self.db)

    def _ensure_index(self):
        if not self._dirty and self._index is not None:
            return self._index
        with self.lock:
            if not self.db:
                return None
            D = np.concatenate([e["desc"] for e in self.db])
            X = np.concatenate([e["X"] for e in self.db])
            E = np.concatenate([np.full(len(e["desc"]), i) for i, e in enumerate(self.db)])
            self._dirty = False
        fl = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=64))
        fl.add([D])
        fl.train()
        self._index = (fl, X, E)
        return self._index

    def _query(self, bgr, T_seg_c1, t, seg, truth=None):
        if seg != self.seg:
            return
        self.stats["queries"] += 1
        idx = self._ensure_index()
        if idx is None:
            self._rej("empty_db")
            return
        fl, X, _E = idx
        g = self.gray(bgr)
        kps, desc = self.sift.detectAndCompute(g, None)
        if desc is None or len(kps) < 20:
            self._rej("few_features")
            return
        mm = fl.knnMatch(desc.astype(np.float32), k=4)
        obj, img = [], []
        for qi, ms in enumerate(mm):
            if not ms:
                continue
            b = ms[0]
            xb = X[b.trainIdx]
            other = next((m for m in ms[1:] if np.linalg.norm(X[m.trainIdx] - xb) > 0.05), None)
            if other is not None and b.distance > 0.8 * other.distance:
                continue
            obj.append(xb)
            img.append(kps[qi].pt)
        if len(obj) < self.min_inliers:
            self._rej("few_matches")
            return
        obj = np.asarray(obj, np.float64)
        img = np.asarray(img, np.float64)
        ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, self.K, None, iterationsCount=500, reprojectionError=3.0,
                                                 confidence=0.999, flags=cv2.SOLVEPNP_AP3P)
        if not ok or inl is None or len(inl) < self.min_inliers:
            self._rej("pnp_few_inliers")
            return
        inl = inl.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(obj[inl], img[inl], self.K, None, rvec, tvec)
        pr, _ = cv2.projectPoints(obj[inl], rvec, tvec, self.K, None)
        err = np.linalg.norm(pr.reshape(-1, 2) - img[inl], axis=1)
        med = float(np.median(err))
        frac = len(inl) / len(obj)
        if med > self.max_reproj or frac < self.min_inlier_frac:
            self._rej("pnp_quality")
            return
        R_cw = cv2.Rodrigues(rvec)[0]
        T_w_c = inv_T(make_T(R_cw, tvec))
        T_on = T_w_c @ inv_T(np.asarray(T_seg_c1, float))
        tilt = tilt_deg(T_on[:3, :3])
        if tilt > self.max_tilt:
            self._rej("tilt")
            return
        self.stats["pnp_ok"] += 1
        c = dict(t=t, T=T_on, n=len(inl), frac=round(frac, 3), reproj=round(med, 3), tilt=round(tilt, 2),
                 C_old=T_w_c[:3, 3].copy(), C_new=np.asarray(T_seg_c1, float)[:3, 3].copy(),
                 obj=obj[inl], img=img[inl], T_sc=np.asarray(T_seg_c1, float))
        with self.lock:
            if seg != self.seg:
                return
            self.cands.append(c)
            agree = [d for d in self.cands if rot_deg(d["T"][:3, :3].T @ T_on[:3, :3]) < self.agree_rot
                     and np.linalg.norm(d["T"][:3, 3] - T_on[:3, 3]) < self.agree_trans]
            if len(agree) < self.min_agree:
                return
            self.seg, self.cands = None, []
        w = np.array([d["n"] for d in agree], float)
        yaws = np.array([yaw_of(d["T"][:3, :3]) for d in agree])
        yaw = float(np.arctan2((w * np.sin(yaws)).sum(), (w * np.cos(yaws)).sum()))
        p = (w[:, None] * np.array([d["T"][:3, 3] for d in agree])).sum(0) / w.sum()
        yaw, p, ref_px = self._refine4(agree, yaw, p)
        T = T_yaw(yaw, p)
        ratios = []
        for i in range(len(agree)):
            for j in range(i + 1, len(agree)):
                dn = np.linalg.norm(agree[i]["C_new"] - agree[j]["C_new"])
                if dn > 0.05:
                    ratios.append(np.linalg.norm(agree[i]["C_old"] - agree[j]["C_old"]) / dn)
        info = dict(seg=seg, t=round(float(t), 3), n_agree=len(agree), inliers=[d["n"] for d in agree],
                    reproj_px=[d["reproj"] for d in agree], tilt_deg=[d["tilt"] for d in agree],
                    spread_rot_deg=round(max(rot_deg(d["T"][:3, :3].T @ T[:3, :3]) for d in agree), 3),
                    spread_trans_m=round(max(float(np.linalg.norm(d["T"][:3, 3] - p)) for d in agree), 4),
                    yaw_deg=round(np.degrees(yaw), 3), trans_m=[round(float(x), 4) for x in p],
                    scale_ratio=round(float(np.median(ratios)), 4) if ratios else None, refined_reproj_px=ref_px)
        if truth is not None:
            Tt = np.asarray(truth, float)
            info["truth_err_rot_deg"] = round(rot_deg(Tt[:3, :3].T @ T[:3, :3]), 3)
            info["truth_err_trans_m"] = round(float(np.linalg.norm(Tt[:3, 3] - T[:3, 3])), 4)
        self.stats["alignments"].append(info)
        self.log(f"[reloc] 조각 {seg} 붙임: 키프레임 {len(agree)}장 합의, 내점 {info['inliers']}, 재투영 {info['reproj_px']} px, "
                 f"yaw {info['yaw_deg']}° 이동 {info['trans_m']} m, 축척비 {info['scale_ratio']}"
                 + (f", 정답과 차이 {info['truth_err_rot_deg']}° {info['truth_err_trans_m'] * 100:.1f} cm" if truth is not None else ""))
        if self.on_aligned:
            self.on_aligned(seg, T, info)

    def _refine4(self, agree, yaw, p):
        """합의한 키프레임들의 모든 PnP 내점으로 yaw + 이동 (4 자유도, 두 월드 모두 중력 정렬) 재투영 최소제곱 (Huber 2 px).
        반환 (yaw, p, 재투영 오차 중앙값 px). scipy 가 없거나 실패하면 가중 평균 그대로."""
        try:
            from scipy.optimize import least_squares
        except ImportError:
            return yaw, p, None
        K = self.K

        def res(x):
            T = T_yaw(x[0], x[1:4])
            out = []
            for d in agree:
                Tcw = inv_T(T @ d["T_sc"])
                Xc = d["obj"] @ Tcw[:3, :3].T + Tcw[:3, 3]
                uv = Xc @ K.T
                out.append((uv[:, :2] / np.maximum(uv[:, 2:3], 1e-6) - d["img"]).ravel())
            return np.concatenate(out)

        x0 = np.r_[yaw, p]
        try:
            r = least_squares(res, x0, loss="huber", f_scale=2.0, max_nfev=200)
        except Exception:
            return yaw, p, None
        e = np.linalg.norm(r.fun.reshape(-1, 2), axis=1)
        return float(r.x[0]), r.x[1:4].copy(), round(float(np.median(e)), 3)

    def _check(self, pts_w, seg, t):
        """붙인 뒤 키프레임 깊이 점(전역) ↔ 옛 지도 점: 최근접 거리, 짧은 ICP 보정량 (정렬 정확도)."""
        try:
            from scipy.spatial import cKDTree
        except ImportError:
            return
        with self.lock:
            if not self.cloud:
                return
            ref = np.concatenate(self.cloud)
        pts = np.asarray(pts_w, float)
        if len(pts) > 4000:
            pts = pts[np.random.default_rng(0).choice(len(pts), 4000, replace=False)]
        tree = cKDTree(ref)
        d0, _ = tree.query(pts, distance_upper_bound=0.5)
        near = np.isfinite(d0) & (d0 < 0.10)
        # (짧은 ICP 보정량도 재 봤지만 부분 겹침·깊이 잡음에서 평소 키프레임끼리도 10~40° 로 튀어 지표로 못 씀 → 뺌)
        res = dict(seg=seg, t=round(float(t), 3), n=len(pts), overlap_10cm=round(float(near.mean()), 3),
                   med_nn_cm=round(float(np.median(d0[near])) * 100, 2) if near.any() else None)
        self.stats["checks"].append(res)
        self.log(f"[reloc] 확인 조각 {seg} t={res['t']}: 옛 지도와 10 cm 안 {res['overlap_10cm'] * 100:.0f}% · 최근접 중앙 "
                 f"{res['med_nn_cm']} cm")
