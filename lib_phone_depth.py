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


class PhoneDepth:
    def __init__(self, a):
        import lib_rectify as lr
        st, self.swap = load_phone_calibration(a.calib_stereo, a.logical, a.physical, a.size)
        cal = [dict(K=np.array(st[f"K{i}"]), dist=np.array(st[f"dist{i}"]), image_size=st["image_size"])
               for i in (1, 2)]
        self.rp = lr.rectify_maps(*cal, np.array(st["R"]), np.array(st["T"]))
        self.a, self.st = a, st
        # 캘리를 만든 번들의 초점 고정값과 다르면 광각 fx 가 달라져 dy·시차가 어긋난다
        self.focus_warning = None
        if st.get("focus_m") is not None and abs(float(st["focus_m"]) - float(getattr(a, "focus_m", st["focus_m"]))) > 1e-6:
            self.focus_warning = f"focus_mismatch:calib {st['focus_m']}m vs run {a.focus_m}m"
        self.model = None
        self.ep = None

    def __call__(self, pair):
        import lib_ffs
        import lib_detect as ld
        import lib_rectify as lr
        import z_object_depth as od
        a, rp = self.a, self.rp
        t0 = time.perf_counter()
        fL, fR = (pair[1], pair[0]) if self.swap else (pair[0], pair[1])      # 캘리의 (왼,오른) 순서로
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
                rows.append(od.aggregate(rec, region, valid, depth, points, rp=rp, outline=method=="seg"))
            results[method] = rows
            vis = od.draw_overlay(left, dets, rows, method)
            cv2.putText(vis, f"{method} depth - {self.st.get('scale_status', 'unverified')}", (12,24),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0,220,255), 2)
            overlays.append(vis)
        report = dict(timestamp_ns=[fL.timestamp_ns, fR.timestamp_ns], physical_lr=self.st["phone"]["physical"],
                      mode=mode, device="cuda:0",
                      ms=(time.perf_counter()-t0)*1000, epipolar=self.ep, objects=results,
                      calibration=str(a.calib_stereo), scale_status=self.st.get("scale_status", "unverified"))
        return np.hstack(overlays), od._clean(report)
