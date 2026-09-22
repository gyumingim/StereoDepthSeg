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
    st = json.loads(Path(path).read_text())
    if st.get("phone") != {"logical": logical, "physical": list(physical[:2])}:
        raise ValueError("Calibration phone.logical / phone.physical must match the selected first two lenses, in order")
    if st.get("image_size") != list(size):
        raise ValueError("Calibration image_size does not match the phone output")
    for key in ("K1", "K2", "dist1", "dist2", "R", "T"):
        if key not in st or not np.isfinite(np.asarray(st[key], dtype=float)).all():
            raise ValueError(f"Missing/nonfinite calibration field: {key}")
    return st


class PhoneDepth:
    def __init__(self, a):
        import lib_rectify as lr
        st = load_phone_calibration(a.calib_stereo, a.logical, a.physical, a.size)
        cal = [dict(K=np.array(st[f"K{i}"]), dist=np.array(st[f"dist{i}"]), image_size=st["image_size"])
               for i in (1, 2)]
        self.rp = lr.rectify_maps(*cal, np.array(st["R"]), np.array(st["T"]))
        self.a, self.st = a, st
        self.model = None
        self.ep = None

    def __call__(self, pair):
        import lib_ffs
        import lib_detect as ld
        import lib_rectify as lr
        import z_object_depth as od
        a, rp = self.a, self.rp
        t0 = time.perf_counter()
        left, right = lr.rectify_pair(pair[0].bgr, pair[1].bgr, rp)
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
                    rec["warnings"].append("scale_not_validated")
                if self.ep["status"] != "ok":
                    rec["warnings"].append("rectification_unverified")
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
        report = dict(timestamp_ns=[f.timestamp_ns for f in pair[:2]], mode=mode, device="cuda:0",
                      ms=(time.perf_counter()-t0)*1000, epipolar=self.ep, objects=results,
                      calibration=str(a.calib_stereo), scale_status=self.st.get("scale_status", "unverified"))
        return np.hstack(overlays), od._clean(report)
