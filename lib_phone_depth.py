"""Calibrated live phone pairs -> existing FFS/seg/bbox depth implementation.

Camera-specific calibration is mandatory. Laptop/phone calibration must never be
silently reused for two phone lenses. This module does not fabricate calibration.
"""
import json
from pathlib import Path
import time

import cv2
import numpy as np


def load_phone_calibration(path, logical, physical, size):
    """(st, swap). swap=True 면 캘리의 (왼,오른) 이 --physical 의 첫 둘과 반대 — 수신 쌍을 뒤집어 쓴다."""
    st = json.loads(Path(path).read_text())
    want = [str(p) for p in physical[:2]]
    got = (st.get("phone") or {}).get("physical")
    if (st.get("phone") or {}).get("logical") != str(logical) or got not in (want, want[::-1]):
        raise ValueError(f"Calibration phone={st.get('phone')} does not match --logical {logical} --physical {want} "
                         f"(z_phone_calib.py 로 만든 json 의 phone.physical 과 같은 두 렌즈여야 함)")
    swap = got == want[::-1]
    if st.get("image_size") != list(size):
        raise ValueError("Calibration image_size does not match the phone output")
    for key in ("K1", "K2", "dist1", "dist2", "R", "T"):
        if key not in st or not np.isfinite(np.asarray(st[key], dtype=float)).all():
            raise ValueError(f"Missing/nonfinite calibration field: {key}")
    return st, swap


def gravity_frame(st, imu):
    """중력 센서(기기 프레임, 위쪽을 가리키는 반작용 벡터) -> cam1(왼쪽 렌즈, 정류 전) 기준 Z-up 세계 프레임.

    반환 dict(R_cam1_to_world(3x3), up_cam1, pitch_deg, roll_deg, ...) 또는 None (센서/캘리 회전 없음).
      z_world = 위(−중력), x_world = 카메라 x축의 수평 투영(오른쪽), y_world = z × x (수평 전방).
      pitch = 광축이 수평면에서 들린 각(+위), roll = 영상 가로축이 수평에서 기운 각.
    Android TYPE_GRAVITY 는 정지 시 가속도계와 같아 화면을 위로 두면 z=+9.81, 즉 벡터가 '위' 를 가리킨다.
    """
    g = (imu or {}).get("gravity")
    Rs = st.get("R_sensor_to_left")
    if not g or Rs is None:
        return None
    up = np.asarray(Rs, float) @ np.asarray(g["values"], float)
    n = np.linalg.norm(up)
    if not 8.0 < n < 11.5:                      # 자유낙하/흔들림 중이면 신뢰 불가
        return None
    up /= n
    x_cam, z_cam = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    xw = x_cam - (x_cam @ up) * up
    if np.linalg.norm(xw) < 0.2:                # 카메라가 거의 수직으로 위/아래를 볼 때는 y축으로 기준
        xw = np.array([0, 1.0, 0]) - (np.array([0, 1.0, 0]) @ up) * up
    xw /= np.linalg.norm(xw); yw = np.cross(up, xw)
    R = np.vstack([xw, yw, up])
    return dict(R_cam1_to_world=R, up_cam1=up.round(5).tolist(), gravity_sensor=g["values"], dt_ms=g.get("dt_ms"),
                pitch_deg=float(np.degrees(np.arcsin(np.clip(z_cam @ up, -1, 1)))),
                roll_deg=float(np.degrees(np.arcsin(np.clip(x_cam @ up, -1, 1)))),
                frame="x=camera right (horizontal), y=forward (horizontal), z=up; origin=left lens")


def depth_colormap(depth, valid, zmin=0.3, zmax=3.0):
    """깊이맵 → 색: 가까움=빨강, 멂=파랑 (JET 역방향), 로그 스케일(스테레오 정밀도가 Z² 로 나빠지므로). 무효=검정. 오른쪽에 눈금 막대."""
    z = np.where(valid, depth, np.nan).astype(np.float32)
    t = (np.log(np.clip(z, zmin, zmax)) - np.log(zmin)) / (np.log(zmax) - np.log(zmin))     # 0=가까움 .. 1=멂
    u8 = np.clip((1.0 - np.nan_to_num(t, nan=1.0)) * 255, 0, 255).astype(np.uint8)         # JET: 0 파랑 .. 255 빨강
    img = cv2.applyColorMap(u8, cv2.COLORMAP_JET)
    img[~valid] = 0
    H, W = img.shape[:2]
    bar = cv2.applyColorMap(np.linspace(255, 0, H - 40, dtype=np.uint8).reshape(-1, 1).repeat(22, 1), cv2.COLORMAP_JET)  # 위 빨강(가까움)
    panel = np.zeros((H, 70, 3), np.uint8); panel[20:H - 20, 6:28] = bar
    for frac, label in ((0.0, f"{zmin:g}m"), (0.5, f"{np.sqrt(zmin * zmax):.2g}m"), (1.0, f"{zmax:g}m")):
        y = int(20 + frac * (H - 41))
        cv2.putText(panel, label, (31, y + 5), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    out = np.hstack([img, panel])
    cv2.putText(out, "depth  red=near  blue=far", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
    return out


def skew_rotation(st, imu):
    """두 렌즈 노출 시각 사이 폰 회전(자이로 적분) → 이 쌍에 쓸 R' = R · Rg.

    왼쪽(초광각) 노출 t_L, 오른쪽(광각) 노출 t_R. 세계점의 왼쪽 카메라 좌표는 X_L(t_R) = Rg X_L(t_L) (평행이동 무시),
    스테레오 식 X_R = R X_L(t_R) + T 에 넣으면 X_R = (R Rg) X_L(t_L) + T. Rg 는 기기 프레임 자이로 적분을 Rs 로 카메라 프레임에 옮긴 것.
    반환 (Rg, deg) 또는 (I, 0).
    """
    win = (imu or {}).get("gyro_window") or []
    ts = (imu or {}).get("exposure_ts_ns")
    Rs = st.get("R_sensor_to_left")
    if len(win) < 2 or not ts or Rs is None or ts[0] == ts[1]:
        return np.eye(3), 0.0
    import lib_phone_motion as pm
    tL, tR = int(ts[0]), int(ts[1])
    Rg_dev = pm.gyro_integrated_rotation(win, min(tL, tR), max(tL, tR))          # X_dev(later) = Rg_dev X_dev(earlier)
    if Rg_dev is None:
        return np.eye(3), 0.0
    if tL > tR:                                                                  # 왼쪽이 나중이면 방향 반대
        Rg_dev = Rg_dev.T
    Rs = np.asarray(Rs, float)
    Rg = Rs @ Rg_dev @ Rs.T
    return Rg, float(np.degrees(np.arccos(np.clip((np.trace(Rg) - 1) / 2, -1, 1))))


class PhoneDepth:
    def __init__(self, a):
        import lib_rectify as lr
        st, self.swap = load_phone_calibration(a.calib_stereo, a.logical, a.physical, a.size)
        cal = [dict(K=np.array(st[f"K{i}"]), dist=np.array(st[f"dist{i}"]), image_size=st["image_size"])
               for i in (1, 2)]
        self.rp = lr.rectify_maps(*cal, np.array(st["R"]), np.array(st["T"]))
        self.cal, self.rp0 = cal, self.rp
        self.a, self.st = a, st
        # 캘리를 만든 번들의 초점 고정값과 다르면 광각 fx 가 달라져 dy·시차가 어긋난다
        self.focus_warning = None
        if st.get("focus_m") is not None and abs(float(st["focus_m"]) - float(getattr(a, "focus_m", st["focus_m"]))) > 1e-6:
            self.focus_warning = f"focus_mismatch:calib {st['focus_m']}m vs run {a.focus_m}m"
        self.model = None
        self.ep = None

    def __call__(self, pair, imu=None):
        import lib_ffs
        import lib_detect as ld
        import lib_rectify as lr
        import z_object_depth as od
        a, rp = self.a, self.rp
        t0 = time.perf_counter()
        fL, fR = (pair[1], pair[0]) if self.swap else (pair[0], pair[1])      # 캘리의 (왼,오른) 순서로
        if self.swap and imu and imu.get("exposure_ts_ns"):
            imu = dict(imu, exposure_ts_ns=list(reversed(imu["exposure_ts_ns"])))
        # 동기 어긋남(두 노출 시각 차) 동안의 폰 회전을 자이로로 보상: 0.02° 넘으면 이 쌍만 정류 테이블을 다시 만든다 (~10ms)
        Rg, skew_deg = skew_rotation(self.st, imu)
        if skew_deg > 0.02:
            rp = lr.rectify_maps(*self.cal, np.asarray(self.st["R"]) @ Rg, np.asarray(self.st["T"]))
        else:
            rp = self.rp0
        self.rp = rp
        left, right = lr.rectify_pair(fL.bgr, fR.bgr, rp)
        if self.ep is None:
            self.ep = lr.epipolar_check(left, right)
            if self.ep["status"] == "ok" and (self.ep["dy_med"] >= od.EPI_GATE_PX or self.ep["inlier_frac"] < od.EPI_INLIER_FRAC):
                raise ValueError(f"Phone rectification failed: {self.ep}")
        if self.model is None:
            self.model = lib_ffs.load(valid_iters=a.iters)
        disp = lib_ffs.infer(self.model, cv2.cvtColor(left, cv2.COLOR_BGR2RGB),
                             cv2.cvtColor(right, cv2.COLOR_BGR2RGB), valid_iters=a.iters, scale=a.scale)
        valid, depth = od.valid_mask(disp, rp, (0.1, 20.0))
        points = lr.depth_to_points_cam1(np.where(valid, depth, np.nan).astype(np.float32), rp)
        mode = a.yolo if a.yolo != "none" else "both"
        dets = ld.detect_yolo(left, model=ld.DEFAULT_MODEL if mode == "bbox" else ld.DEFAULT_SEG_MODEL, device="0")
        gf = gravity_frame(self.st, imu)
        overlays, results = [], {}
        for method in (["seg", "bbox"] if mode == "both" else [mode]):
            rows = []
            for i, d in enumerate(dets):
                rec = od._empty(d["label"], i+1, d, method)
                if self.st.get("scale_status") != "metric":
                    rec["warnings"].append("scale_not_validated:" + str(self.st.get("scale_status")))
                if self.ep["status"] != "ok":
                    rec["warnings"].append("rectification_unverified")
                if self.focus_warning:
                    rec["warnings"].append(self.focus_warning)
                if method == "seg":
                    region = d["mask"]
                    if region is None:
                        rec["status"] = "no_mask"
                        rows.append(rec)
                        continue
                else:
                    region = np.zeros(depth.shape, np.uint8)
                    x, y, w, h = d["box"]
                    region[max(0,y):max(0,y+h), max(0,x):max(0,x+w)] = 255
                rec = od.aggregate(rec, region, valid, depth, points, rp=rp, outline=method=="seg")
                if gf and rec.get("center_cam1_m") is not None:
                    w = gf["R_cam1_to_world"] @ np.asarray(rec["center_cam1_m"], float)
                    rec["center_world_m"] = w.round(4).tolist()          # 중력 정렬: z = 카메라 기준 높이(+위)
                    rec["height_rel_camera_m"] = round(float(w[2]), 4)
                else:
                    rec["center_world_m"] = None; rec["height_rel_camera_m"] = None
                rows.append(rec)
            results[method] = rows
            vis = od.draw_overlay(left, dets, rows, method)
            cv2.putText(vis, f"{method} depth - {self.st.get('scale_status', 'unverified')}", (12,24),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0,220,255), 2)
            overlays.append(vis)
        zr = getattr(a, "depth_range", None) or (0.3, 3.0)
        overlays.append(depth_colormap(depth, valid, float(zr[0]), float(zr[1])))
        report = dict(timestamp_ns=[fL.timestamp_ns, fR.timestamp_ns], physical_lr=self.st["phone"]["physical"],
                      exposure_skew_ms=(fR.timestamp_ns - fL.timestamp_ns) / 1e6, gyro_skew_compensation_deg=round(skew_deg, 4),
                      mode=mode, device="cuda:0",
                      imu={k: v for k, v in (gf or {}).items() if k != "R_cam1_to_world"} if gf else dict(available=False),
                      imu_raw={k: v for k, v in (imu or {}).items() if k != "gravity"},
                      ms=(time.perf_counter()-t0)*1000, epipolar=self.ep, objects=results,
                      calibration=str(a.calib_stereo), scale_status=self.st.get("scale_status", "unverified"))
        return np.hstack(overlays), od._clean(report)
