#!/usr/bin/env python3
"""
z_reconstruct.py — 두 장의 사진에서 객체의 3D 좌표·크기를 구하고 3D 맵에 배치

  python z_reconstruct.py --yolo             # YOLO 로 자동 검출 (권장)
  python z_reconstruct.py --yolo --seg       # 세그멘테이션 마스크로 영역 선택
  python z_reconstruct.py --yolo --classes bottle cup --conf 0.4
  python z_reconstruct.py                    # 마우스로 대상을 드래그해 지정
  python z_reconstruct.py --roi 400,200,180,140 --roi 700,250,120,120
  python z_reconstruct.py --known-mm 210x297  # 자로 잰 실치수와 대조(검증용)
  python z_reconstruct.py --no-view           # 3D 창 없이 수치만

촬영 방식이 뭐든 이 스크립트는 같다. 다른 건 (R,T)를 어디서 얻느냐뿐이고,
그건 shots/meta.json 의 mode 를 보고 자동으로 갈린다.

  board  : 두 사진의 체커보드에서 상대 포즈를 직접 측정 (권장)
  mono   : baseline 으로 R=I, T=(-b,0,0) 을 가정
  stereo : calib_stereo.json (stereoCalibrate 또는 z_stereo_pose.py 결과)

출력 좌표계는 맵 좌표계(Z-up): X 오른쪽, Y 전방, Z 위. 원점은 왼쪽(1번) 카메라.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
results.json 계약 (doc_DEPTH_CODE_REVIEW R06/R11 반영)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  - 매 실행마다 **항상** 새로 쓴다 (임시 파일에 쓰고 교체). 전부 실패해도 이전 결과가
    남지 않는다. 예전엔 성공 객체만 저장하고 전부 실패하면 파일을 안 건드려서, 이전
    장면의 값을 현재 값으로 착각할 수 있었다.
  - meta 에 포즈 출처·baseline·scale_status·촬영 파일 시각을 남긴다. z_stereo_pose.py
    --rescale 은 이 baseline 이 현재 calib_stereo.json 과 같을 때만 보정을 허용한다.
  - 모든 객체를 저장한다. 게이트(재투영·점 수)에 걸린 객체는 status 가 "fail:..." 이고
    값은 진단용으로만 남는다. status=="ok" 인 것만 측정값으로 쓸 것.
  - distance_m 은 카메라 중심까지의 **유클리드 거리**(최종 점군 중심), z_med_m 은 광축
    방향 깊이 중앙값이다. 둘은 다르다. 줄자로 렌즈에서 물체까지 잰 값은 distance_m 에 대응.
"""
import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

import lib_calib as lc
import lib_detect as ld
import lib_stereo as ls
import lib_viz

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

HERE = Path(__file__).parent
SHOT_DIR = HERE / "shots"
CALIB_JSON = HERE / "calib.json"
STEREO_JSON = HERE / "calib_stereo.json"
RESULTS_JSON = HERE / "results.json"

REPROJ_GATE_PX = 1.0
MIN_POINTS = 20


def load_pose(meta, imgL, imgR):
    """mode 에 따라 (R, T, calibL, calibR, info) 를 준다. 코어는 이 뒤로 완전히 동일하다."""
    calibL = lc.load_calib(CALIB_JSON)
    if meta["mode"] == "board":
        R, T, d = ls.pose_from_board(imgL, imgR, calibL, square_m=calibL["square_m"])
        if R is None:
            sys.exit(f"체커보드 포즈 측정 실패 (검출 L={d['found_L']} R={d['found_R']})"
                     " — 보드가 두 사진에 모두 온전히 보여야 함")
        print(f"체커보드 포즈: baseline {d['baseline_m']*1000:.2f}mm, 회전 {d['rot_deg']:.3f}deg, "
              f"solvePnP 재투영 {d['pnp_reproj_px'][0]:.3f}/{d['pnp_reproj_px'][1]:.3f}px")
        for line in ls.board_check_lines(ls.check_board_consistency(imgL, imgR, calibL, R, T)):
            print(line)
        info = dict(pose_source="board", baseline_m=d["baseline_m"], scale_status="measured(board)")
        return R, T, calibL, calibL, info
    if meta["mode"] == "mono":
        R, T = ls.pose_from_baseline(meta["baseline_m"])
        return R, T, calibL, calibL, dict(pose_source="mono", baseline_m=meta["baseline_m"],
                                          scale_status="measured(ruler)")
    if not STEREO_JSON.exists():
        sys.exit(f"{STEREO_JSON} 없음 — z_stereo_pose.py 또는 z_calibrate.py --run-stereo 먼저")
    d = json.loads(STEREO_JSON.read_text())
    calibR = dict(K=np.array(d["K2"]), dist=np.array(d["dist2"]), image_size=tuple(d["image_size"]))
    calibL = dict(K=np.array(d["K1"]), dist=np.array(d["dist1"]), image_size=tuple(d["image_size"]))
    T = np.array(d["T"]).reshape(3, 1)
    info = dict(pose_source="stereo", baseline_m=float(d.get("baseline_m", np.linalg.norm(T))),
                scale_status=d.get("scale_status", "unknown"), method=d.get("method"),
                stereo_json_mtime=os.path.getmtime(STEREO_JSON))
    if info["scale_status"] == "provisional":
        print("  [WARN] calib_stereo.json 의 스케일이 임시값(provisional)이다. 아래 거리·크기는")
        print("         비율만 맞고 절대값은 틀리다. z_stereo_pose.py --rescale 로 확정할 것.")
    return np.array(d["R"]), T, calibL, calibR, info


def parse_roi(text):
    v = [int(x) for x in text.replace(" ", "").split(",")]
    if len(v) != 4:
        raise argparse.ArgumentTypeError("ROI 형식은 x,y,w,h")
    return tuple(v)


def _record(i, det, roi, out):
    """객체 하나의 결과 레코드. 실패해도 같은 키를 가진다 (status 로 구분)."""
    d, box = out["diag"], out["box"]
    rec = dict(instance=i + 1, label=det["label"], conf=round(float(det["conf"]), 3), roi=list(roi),
               status="fail:no_points", gates={}, n_points=int(d.get("n_final", 0)),
               n_match=int(d.get("n_ratio", 0)), epi_px=d.get("epi_px"), reproj_px=d.get("reproj_px"),
               tri_deg_med=(d["tri_deg"][1] if "tri_deg" in d else None),
               center_map=None, center_cam=None, distance_m=None, z_med_m=None,
               dims_mm=None, method=None)
    if box is None or d.get("n_final", 0) == 0:
        return rec
    c_cam = np.asarray(d["center_cam_m"])
    rec.update(center_map=(ls.CAM_TO_MAP @ c_cam).round(4).tolist(), center_cam=c_cam.round(4).tolist(),
               distance_m=float(np.linalg.norm(c_cam)), z_med_m=float(d["z_m"][1]),
               dims_mm=(box["dims_sorted"] * 1000).round(1).tolist(), method=box["method"])
    rep_ok = d["reproj_px"] < REPROJ_GATE_PX
    pts_ok = d["n_final"] >= MIN_POINTS
    rec["gates"] = dict(reproj="pass" if rep_ok else "fail", points="pass" if pts_ok else "fail")
    rec["status"] = "ok" if (rep_ok and pts_ok) else ("fail:reproj" if not rep_ok else "fail:points")
    return rec


def main(a):
    meta_p = SHOT_DIR / "meta.json"
    if not meta_p.exists():
        sys.exit(f"{meta_p} 없음 — z_capture.py 를 먼저 돌릴 것")
    meta = json.loads(meta_p.read_text())
    imgL = cv2.imread(str(SHOT_DIR / "L.png"))
    imgR = cv2.imread(str(SHOT_DIR / "R.png"))
    if imgL is None or imgR is None:
        sys.exit("shots/L.png 또는 R.png 를 읽을 수 없음")

    R, T, calibL, calibR, info = load_pose(meta, imgL, imgR)
    fx = calibL["K"][0, 0]
    print(f"모드 {meta['mode']}, 이미지 {imgL.shape[1]}x{imgL.shape[0]}, fx {fx:.1f}px, "
          f"baseline {info['baseline_m']*1000:.1f}mm, 스케일 {info['scale_status']}")
    if meta["mode"] == "mono":
        b = meta["baseline_m"]
        print("예상 깊이 오차(시차오차 0.5px): " +
              ", ".join(f"{z}m:{ls.depth_error_m(z, b, fx)*1000:.0f}mm" for z in (0.5, 1, 2, 3)))

    # 검출기 선택 — 코어는 ROI/마스크만 받으므로 여기만 갈아끼우면 된다.
    detector = "roi"
    if a.roi:
        dets = [{"box": r, "label": f"roi{i+1}", "conf": 1.0, "mask": None} for i, r in enumerate(a.roi)]
    elif a.yolo:
        model = a.model
        if a.seg and model == ld.DEFAULT_MODEL:
            model = ld.DEFAULT_SEG_MODEL
        dets = ld.detect_yolo(imgL, model=model, conf=a.conf, classes=a.classes, device=a.device)
        detector = f"yolo:{model}:{getattr(ld.detect_yolo, 'last_device', '?')}"
        print(f"\nYOLO({model}, {getattr(ld.detect_yolo, 'last_device', '?')}) 검출 {len(dets)}개: "
              + (", ".join(f"{d['label']}({d['conf']:.2f})" for d in dets) or "없음"))
        prev = HERE / "detections.png"
        cv2.imwrite(str(prev), ld.draw(imgL, dets))
        print(f"검출 미리보기 저장: {prev}")
        dets = [dict(d, box=ld.expand_box(d["box"], imgL.shape)) for d in dets]
    else:
        detector = "manual"
        dets = [dict(d, box=ld.expand_box(d["box"], imgL.shape)) for d in ld.detect_manual(imgL)]

    known = None
    if a.known_mm:
        known = sorted((float(x) for x in a.known_mm.lower().split("x")), reverse=True)

    objects, records = [], []
    for i, det in enumerate(dets):
        roi = det["box"]
        mask = det.get("mask") if a.seg else None
        out = ls.reconstruct(imgL, imgR, calibL, R, T, roi=roi, calibR=calibR,
                             z_range=tuple(a.z_range), mask=mask)
        d, box = out["diag"], out["box"]
        rec = _record(i, det, roi, out)
        records.append(rec)
        print(f"\n{'='*62}\n객체 {i+1}  {det['label']} (conf {det['conf']:.2f})"
              f"  {'MASK' if mask is not None else 'ROI'} {roi}\n{'='*62}")
        print(f"  매칭  knn {d['n_knn']} -> ratio {d['n_ratio']} -> epipolar {d['n_epipolar']}"
              f" -> depth {d['n_depth']} -> 최종 {d['n_final']}")
        if rec["status"] == "fail:no_points":
            print("  복원 실패 — 점이 너무 적다. 대상에 질감이 있어야 하고,")
            print("  두 사진 모두에 같은 면이 보여야 한다.")
            continue
        print(f"  에피폴라 잔차 {d['epi_px']:.3f} px   광선각 {d['tri_deg'][0]:.2f}/{d['tri_deg'][1]:.2f}/{d['tri_deg'][2]:.2f} deg")
        print(f"  깊이 Z(최종집합) {d['z_m'][0]:.3f} / {d['z_m'][1]:.3f} / {d['z_m'][2]:.3f} m (min/중앙/max)"
              f"   필터 전 중앙 {d['z_raw_m'][1]:.3f}")
        c = rec["center_map"]; dims = rec["dims_mm"]
        print(f"\n  중심 (맵계) X {c[0]:+.4f}  Y {c[1]:+.4f}  Z {c[2]:+.4f}  m   거리 {rec['distance_m']:.3f} m")
        print(f"  크기        {dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm  [{rec['method']}]")
        print(f"              가장 얇은 축({dims[2]:.1f}mm)은 물체의 두께가 아니라 복원된 표면의 두께다.")
        g = rec["gates"]
        print(f"\n  [{'PASS' if g['reproj']=='pass' else 'FAIL'}] 재투영 오차 {d['reproj_px']:.3f} px (< {REPROJ_GATE_PX})")
        print(f"  [{'PASS' if g['points']=='pass' else 'FAIL'}] 점 개수 {d['n_final']} (>= {MIN_POINTS})")
        if known:
            e = [abs(dims[j] - known[j]) / known[j] * 100 for j in range(min(2, len(known)))]
            print(f"  [{'PASS' if max(e) < 10 else 'FAIL'}] 실측 대조 {known[0]:.1f}x{known[1]:.1f}mm "
                  f"-> 오차 {e[0]:.2f}% / {e[1]:.2f}% (< 10%)")
        if rec["status"] != "ok":
            print(f"  => status {rec['status']} — 진단용으로만 저장, 측정값으로 쓰지 말 것")
        objects.append(out)

    # 항상 새로 쓴다 (임시 파일 -> 교체). 전부 실패해도 이전 결과가 남지 않는다.
    payload = dict(
        meta=dict(time=datetime.now().isoformat(timespec="seconds"), mode=meta["mode"],
                  pose_source=info["pose_source"], scale_status=info["scale_status"],
                  stereo_baseline_m=info["baseline_m"], detector=detector,
                  shots=dict(L=os.path.getmtime(SHOT_DIR / "L.png"), R=os.path.getmtime(SHOT_DIR / "R.png")),
                  gates=dict(reproj_px_max=REPROJ_GATE_PX, min_points=MIN_POINTS),
                  n_ok=sum(r["status"] == "ok" for r in records), n_total=len(records)),
        objects=records)
    tmp = RESULTS_JSON.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(payload, indent=2, allow_nan=False,
                              default=lambda o: None if isinstance(o, float) else str(o)))
    os.replace(tmp, RESULTS_JSON)
    print(f"\n저장: {RESULTS_JSON}  (ok {payload['meta']['n_ok']}/{len(records)})")

    if objects and not a.no_view:
        print("3D 맵 창을 띄운다 (창을 닫으면 종료). 파랑=왼쪽 촬영, 주황=오른쪽 촬영, 격자 25cm")
        lib_viz.show(objects, calibL["K"], calibL["image_size"], R, T)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--yolo", action="store_true", help="YOLO 자동 검출")
    ap.add_argument("--seg", action="store_true",
                    help="YOLO 세그멘테이션 마스크로 객체 영역 선택 (기본은 bbox). --model 미지정 시 yolo11m-seg.pt")
    ap.add_argument("--model", default=ld.DEFAULT_MODEL, help=f"YOLO 가중치 (기본 {ld.DEFAULT_MODEL})")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO 신뢰도 하한")
    ap.add_argument("--device", default=ld.DEFAULT_DEVICE, help="YOLO 실행 장치: 0/cuda:0 은 GPU, cpu 는 CPU. 생략하면 자동")
    ap.add_argument("--classes", nargs="+", help="COCO 클래스 이름으로 필터 (예: --classes bottle cup)")
    ap.add_argument("--roi", type=parse_roi, action="append", help="x,y,w,h — 여러 번 주면 여러 객체. 좌표를 그대로 쓴다")
    ap.add_argument("--known-mm", help="자로 잰 실치수 검증용, 예: 210x297")
    ap.add_argument("--z-range", type=float, nargs=2, default=(0.1, 20.0), metavar=("MIN", "MAX"),
                    help="유효 거리 범위 m (기본 0.1 20)")
    ap.add_argument("--no-view", action="store_true", help="3D 창 없이 수치만")
    main(ap.parse_args())
