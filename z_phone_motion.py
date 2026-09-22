#!/usr/bin/env python3
"""폰을 옆으로 옮긴 두 번들(A, B) → 자이로 회전 + 영상 이동방향 + 15.76mm 스테레오 스케일 → 넓은 baseline 스테레오.

  # 1) z_phone_stereo.py GUI 에서 S 를 두 번 (A 에서 정지 → 10~20cm 옆으로 옮겨 정지 → S). 번들에 IMU 가 함께 저장된다.
  ./venv/bin/python z_phone_stereo.py --physical 2 5
  # 2) 분석 (venv): 회전·이동·baseline·가속도 교차검증 → motion.json
  ./venv/bin/python z_phone_motion.py --a phone_stereo/runs/<t>/captures/<tsA> --b phone_stereo/runs/<t>/captures/<tsB>
  # 3) 조밀 depth (venv_ffs): 이동 쌍을 정류해 FFS + YOLO seg → motion_depth.png / motion.json 에 objects
  ./venv_ffs/bin/python z_phone_motion.py --a ... --b ... --dense

한계: |T| 는 15.76mm 스테레오의 yaw/baseline 편향을 물려받는다 (--known/--board 뒤에 절대 미터). 세로로 옮기면 정류 거부.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

import lib_phone_calib as pc
import lib_phone_motion as pm
import lib_rectify as lr


def load_bundle(d, L, R):
    d = Path(d)
    meta = json.loads((d / "pair.json").read_text())
    imL, imR = cv2.imread(str(d / f"camera_{L}.png")), cv2.imread(str(d / f"camera_{R}.png"))
    if imL is None or imR is None or not meta.get("imu", {}).get(pm.ROTVEC_KEY):
        raise FileNotFoundError(f"{d}: camera_{L}/{R}.png 와 pair.json 의 imu.{pm.ROTVEC_KEY} 필요 (구버전 번들은 IMU 없음)")
    return imL, imR, meta


def main(a):
    st = pc.load(a.calib_stereo)
    L, R = st["phone"]["physical"]
    AL, AR, mA = load_bundle(a.a, L, R); BL, BR, mB = load_bundle(a.b, L, R)
    res = pm.analyze(st, AL, AR, BL, mA["imu"], mB["imu"], tA_ns=mA["timestamp_ns"][0], tB_ns=mB["timestamp_ns"][0])
    d = res["diag"]
    print(f"회전(회전벡터) {d['rotation_deg']:.3f}° xyz {d['rotvec_deg_xyz']}  A↔B 대응점 {d['n_AB']}  회전 보정 후 흐름 {d['flow_after_rotation_med_px']:.1f}px")
    out = dict(a=str(a.a), b=str(a.b), calibration=str(a.calib_stereo), R=res["R"].tolist(), T=None if res["T"] is None else res["T"].tolist(), diag=d)
    for w in d.get("warnings", []):
        print("  !", w)
    if d["status"] != "ok":
        print(f"판정: {d['status']}" + (f"  (baseline 추정치 {d['baseline_m']*1000:.1f}mm 는 신뢰 불가)" if d.get("baseline_m") else ""))
    if d["status"] == "ok" or (d["status"] == "baseline_unreliable" and a.dense):
        print(f"이동방향 t̂ {d['t_hat']}  에피폴라 잔차 {d['epipolar_residual_med_px']:.2f}px 인라이어 {d['epipolar_inlier_frac']:.2f}  "
              f"baseline **{d['baseline_m']*1000:.1f}mm** (3-뷰 점 {d['n_scale']}, MAD {d['scale_mad_rel']*100:.1f}%)")
        if "accel" in d:
            acc = d["accel"]
            print(f"가속도 적분 교차검증: |d| {acc['norm_m']*1000:.0f}mm ({acc['duration_s']:.1f}s), 시각 baseline 대비 비 {acc['norm_ratio_accel_over_visual']:.2f}, 방향차 {acc['angle_to_visual_deg']:.1f}°")
        if a.dense and res["T"] is not None:
            import lib_ffs, lib_detect as ld, z_object_depth as od
            order, Rm, Tm = pm.motion_rectify_order(st, res["R"], res["T"])
            cal = dict(K=st["K1"], dist=st["dist1"], image_size=tuple(st["image_size"]))
            rp = lr.rectify_maps(cal, cal, Rm, Tm)
            imgL, imgR = (AL, BL) if order == "AB" else (BL, AL)
            rL, rR = lr.rectify_pair(imgL, imgR, rp)
            ep = lr.epipolar_check(rL, rR)
            model = lib_ffs.load(valid_iters=a.iters)
            disp = lib_ffs.infer(model, cv2.cvtColor(rL, cv2.COLOR_BGR2RGB), cv2.cvtColor(rR, cv2.COLOR_BGR2RGB), valid_iters=a.iters, scale=a.scale)
            valid, depth = od.valid_mask(disp, rp, (0.1, 20.0))
            pts = lr.depth_to_points_cam1(np.where(valid, depth, np.nan).astype(np.float32), rp)
            dets = ld.detect_yolo(rL, model=ld.DEFAULT_SEG_MODEL, device="0")
            rows = []
            for i, det in enumerate(dets):
                rec = od._empty(det["label"], i + 1, det, "seg")
                if det["mask"] is None:
                    rec["status"] = "no_mask"; rows.append(rec); continue
                rec = od.aggregate(rec, det["mask"], valid, depth, pts, rp=rp, outline=True)
                rec["warnings"].append(f"scale_inherits:{st.get('scale_status')}")
                rows.append(rec)
                print(f"   seg #{i+1} {det['label']:10s} {rec['status']:12s} dist {rec['dist_m']}m  dims_mm {rec['dims_mm']}")
            vis = od.draw_overlay(rL, dets, rows, "seg")
            cv2.putText(vis, f"motion stereo b={rp['baseline_m']*1000:.0f}mm order {order} dy {ep['dy_med']}", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 220, 255), 2)
            cv2.imwrite(str(Path(a.a) / "motion_depth.png"), vis)
            out.update(dense=dict(order=order, baseline_m=rp["baseline_m"], f_px=rp["f"], epipolar=ep, objects=od._clean(rows),
                                  depth_err_per_half_px_at_1m=float(0.5 / (rp["f"] * rp["baseline_m"]))))
            print(f"조밀: 정류 dy {ep['dy_med']} 인라이어 {ep['inlier_frac']}  baseline {rp['baseline_m']*1000:.1f}mm → 1m 에서 시차 0.5px = 깊이 {0.5/(rp['f']*rp['baseline_m'])*100:.1f}%")
    (Path(a.a) / "motion.json").write_text(json.dumps(out, indent=2, ensure_ascii=False, default=float))
    print("저장:", Path(a.a) / "motion.json")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", required=True, help="A 시점 번들 (IMU 포함 pair.json)")
    ap.add_argument("--b", required=True)
    ap.add_argument("--calib-stereo", default="calib_phone_pair.json")
    ap.add_argument("--dense", action="store_true", help="이동 쌍 FFS 조밀 depth + YOLO seg (venv_ffs)")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--iters", type=int, default=8)
    main(ap.parse_args())
