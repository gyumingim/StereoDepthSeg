"""키프레임 깊이 + 분할 (GPU, 워커 스레드 1개에서만 호출).

정류(BGR) → FFS 시차 → 유효 화소(범위·roi·불연속 제외) → YOLO-seg track (정류 왼쪽 BGR)
→ stride 간격 화소의 rect0 점 + 색 + 클래스 id + 2D 트랙 id/마스크.
"""
import time

import cv2
import numpy as np

import lib_ffs
import lib_rectify as lr

import frames


def valid_mask(disp, rp, z_range, min_disp=0.5):
    """z_object_depth.valid_mask 와 같은 규칙 (그 모듈은 무거운 import 가 많아 복사):
    시차 양수·유한, 오른쪽 대응점이 roi2 안, 깊이 범위 안, 왼쪽 roi1 안."""
    H, W = disp.shape
    u = np.arange(W, dtype=np.float32)[None, :]
    v = np.arange(H, dtype=np.float32)[:, None]
    ur = u - disp
    x2, y2, w2, h2 = rp["roi2"]
    Z = lr.disparity_to_depth(disp, rp)
    ok = np.isfinite(disp) & (disp > min_disp) & (ur >= x2) & (ur < x2 + w2) & (v >= y2) & (v < y2 + h2)
    ok &= np.isfinite(Z) & (Z >= z_range[0]) & (Z <= z_range[1])
    x1, y1, w1, h1 = rp["roi1"]
    box = np.zeros_like(ok); box[y1:y1 + h1, x1:x1 + w1] = True
    return ok & box, Z


def uncertain_mask(disp, rel=0.15, abs_px=1.0, k=5):
    """깊이 불연속(날아다니는 점) 영역: k×k 안 시차 범위가 max(abs_px, rel·d) 를 넘는 화소."""
    d = np.nan_to_num(disp, nan=0.0).astype(np.float32)
    ker = np.ones((k, k), np.uint8)
    rng = cv2.dilate(d, ker) - cv2.erode(d, ker)
    return rng > np.maximum(abs_px, rel * d)


class YoloTracker:
    """ultralytics YOLO.track 래퍼: 클래스 id·트랙 id·마스크(0/255, 입력 해상도). rotate=True 면 90° 돌려 검출 후 되돌림."""

    def __init__(self, model="yolo11m-seg.pt", conf=0.25, device="0", rotate=False, tracker="bytetrack.yaml"):
        from ultralytics import YOLO
        self.net = YOLO(model)
        self.conf, self.device, self.rotate, self.tracker = conf, device, rotate, tracker
        self.names = dict(self.net.names)

    def reset(self):
        try:
            for t in getattr(self.net.predictor, "trackers", []) or []:
                t.reset()
        except Exception:
            pass

    def __call__(self, bgr):
        H, W = bgr.shape[:2]
        img = cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE) if self.rotate else bgr
        res = self.net.track(img, persist=True, retina_masks=True, conf=self.conf, device=self.device,
                             verbose=False, tracker=self.tracker)[0]
        out = []
        if res.boxes is None or len(res.boxes) == 0:
            return out
        md = res.masks.data.cpu().numpy() if res.masks is not None else None
        if md is not None and md.shape[1:] != img.shape[:2]:
            md = None
        cls = res.boxes.cls.cpu().numpy().astype(int)
        conf = res.boxes.conf.cpu().numpy()
        ids = res.boxes.id.cpu().numpy().astype(int) if res.boxes.id is not None else [None] * len(cls)
        xyxy = res.boxes.xyxy.cpu().numpy()
        for i in range(len(cls)):
            x1, y1, x2, y2 = xyxy[i]
            mask = (md[i] > 0).astype(np.uint8) * 255 if md is not None else None
            if self.rotate:
                # 시계 90°: 원래 (u,v) → (H-1-v, u). 되돌림: u = y', v = H-1-x'
                if mask is not None:
                    mask = cv2.rotate(mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
                x1, y1, x2, y2 = y1, H - x2, y2, H - x1
            out.append(dict(box=(int(x1), int(y1), int(x2 - x1), int(y2 - y1)), class_id=int(cls[i]),
                            label=self.names.get(int(cls[i]), str(cls[i])), conf=float(conf[i]),
                            track_id=None if ids[i] is None else int(ids[i]), mask=mask))
        out.sort(key=lambda d: d["conf"])            # 낮은 신뢰도 먼저 → 높은 것이 덮어씀
        return out


class DepthSeg:
    def __init__(self, rp, iters=4, scale=1.0, z_range=(0.3, 2.5), stride=3, yolo_model="yolo11m-seg.pt",
                 yolo_conf=0.25, yolo_rotate=False, device="0", no_yolo=False, log=print):
        self.rp, self.iters, self.scale, self.z_range, self.stride = rp, iters, scale, z_range, stride
        self.log = log
        self.model = lib_ffs.load(valid_iters=iters, max_disp=192)
        self.log(f"[depth] FFS 로드 (triton={lib_ffs.triton_usable()}, iters={iters}, scale={scale})")
        self.yolo = None if no_yolo else YoloTracker(yolo_model, yolo_conf, device, yolo_rotate)
        self.names = self.yolo.names if self.yolo else {}

    def set_iters(self, iters):
        """FFS GRU 반복 수 변경 (load 가 model.args.valid_iters 를 고정하므로 다시 load; lib_ffs 가 키별로 캐시). 워커 스레드에서만."""
        iters = int(iters)
        if iters == self.iters:
            return
        self.model = lib_ffs.load(valid_iters=iters, max_disp=192)
        self.iters = iters
        self.log(f"[depth] FFS iters={iters}")

    def warmup(self):
        W, H = self.rp["size"]
        z = np.zeros((H, W, 3), np.uint8)
        t = time.perf_counter()
        lib_ffs.infer(self.model, z, z, valid_iters=self.iters, scale=self.scale)
        if self.yolo:
            self.yolo(z)
            self.yolo.reset()
        self.log(f"[depth] 워밍업 {1000 * (time.perf_counter() - t):.0f} ms")

    def set_geometry(self, rp, scale=None):
        """세션 해상도 변경: 정류맵·정류 K (rp) 와 FFS 입력 축소 비율. 진행 중인 호출은 시작 때 잡은 rp 를 그대로 쓴다."""
        self.rp = rp
        if scale is not None:
            self.scale = float(scale)
        W, H = rp["size"]
        self.log(f"[depth] 해상도 {W}x{H} · FFS 입력 {int(W * self.scale)}x{int(H * self.scale)} (scale {self.scale:g})")

    def __call__(self, bgrL, bgrR, rp=None, scale=None):
        """원본 BGR 쌍 (렌즈 2, 렌즈 5) → dict(rectL, disp, Z, valid, dets, us, vs, Zs, rgb, cls, track, det_idx, ms...).
        rp/scale 을 주면 그 값 (세션 해상도가 바뀌는 순간에도 한 호출 안에서는 같은 정류맵), 아니면 self 값.
        영상 크기가 rp 크기와 다르면 ValueError (해상도 바뀌기 전 프레임)."""
        rp = rp or self.rp
        scale = float(scale or self.scale)
        if (bgrL.shape[1], bgrL.shape[0]) != tuple(rp["size"]) or bgrL.shape != bgrR.shape:
            raise ValueError(f"프레임 {bgrL.shape[1]}x{bgrL.shape[0]} ≠ 정류 크기 {tuple(rp['size'])}")
        t0 = time.perf_counter()
        rectL, rectR = lr.rectify_pair(bgrL, bgrR, rp)
        disp = lib_ffs.infer(self.model, cv2.cvtColor(rectL, cv2.COLOR_BGR2RGB), cv2.cvtColor(rectR, cv2.COLOR_BGR2RGB),
                             valid_iters=self.iters, scale=scale)
        t1 = time.perf_counter()
        valid, Z = valid_mask(disp, rp, self.z_range)
        valid &= ~uncertain_mask(disp)
        dets = self.yolo(rectL) if self.yolo else []
        t2 = time.perf_counter()
        H, W = Z.shape
        det_map = np.full((H, W), -1, np.int32)            # 화소 → dets 색인 (높은 신뢰도가 덮어씀)
        for k, d in enumerate(dets):
            if d["mask"] is not None:
                det_map[d["mask"] > 0] = k
        s = self.stride
        sub = np.zeros_like(valid); sub[::s, ::s] = True
        # 마스크 안은 두 배 촘촘히 (작은 물체 보존)
        s2 = max(1, s // 2)
        sub2 = np.zeros_like(valid); sub2[::s2, ::s2] = True
        take = valid & (sub | (sub2 & (det_map >= 0)))
        vs, us = np.nonzero(take)
        di = det_map[vs, us]
        # 물체 마스크 이상치 제거: 마스크별 깊이 중앙값 ±20% 밖이면 배경으로
        Zs = Z[vs, us]
        for k in np.unique(di[di >= 0]):
            sel = di == k
            med = np.median(Zs[sel])
            bad = sel & (np.abs(Zs - med) > 0.2 * med)
            di[bad] = -1
        rgb = rectL[vs, us][:, ::-1].copy()                 # BGR → RGB
        cls = np.where(di >= 0, np.array([d["class_id"] for d in dets] + [-1], np.int16)[di], -1).astype(np.int16)
        pts = frames.rect0_points(Zs, rp, us.astype(np.float32), vs.astype(np.float32))
        return dict(rectL=rectL, disp=disp, Z=Z, valid=valid, dets=dets, pts_rect0=pts, rgb=rgb, cls=cls, det_idx=di,
                    ffs_ms=1000 * (t1 - t0), yolo_ms=1000 * (t2 - t1), n=len(pts), size=tuple(rp["size"]), ffs_scale=scale)
