#!/usr/bin/env python3
"""
z_stereo_pose.py — 체커보드 없이 두 카메라의 상대 포즈를 구한다

  python z_stereo_pose.py --provisional                 # 회전·방향만, 스케일 미확정
  python z_reconstruct.py --yolo                        # 깊이가 (비율만 맞게) 나온다
  python z_stereo_pose.py --rescale 1=0.62              # 1번 객체 실제 0.62m -> 스케일 확정
  python z_stereo_pose.py --baseline-mm 180             # 자로 잰 렌즈 간 거리로 확정
  python z_stereo_pose.py --roi 400,200,180,140 --known-mm 210x297   # 기준 물체 치수로 확정

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
왜 stereoCalibrate 를 못 쓰는가
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  stereoCalibrate 는 두 카메라가 "동시에 보는" 체커보드를 요구한다. 그런데
    - 노트북 웹캠은 화면 위 베젤에 있어 자기 화면을 못 본다
    - 폰은 이제 카메라라서 체커보드를 띄울 수 없다
    - 프린터도 다른 화면도 없다
  다행히 어려운 쪽(각 카메라의 내부 파라미터)은 이미 따로 끝났다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
방법
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1) 회전 R 과 이동 "방향" t_hat: 특징점 -> Essential -> recoverPose.
     두 카메라의 K 가 다르므로 각자의 K/dist 로 정규화한 뒤 쓴다.
  2) 이동 "크기"(baseline)는 Essential 로는 안 나온다. 셋 중 하나로 정한다:
       --rescale     물체 하나의 실제 거리 (깊이는 baseline 에 정비례 -> 비례식 한 번)
       --baseline-mm 자로 잰 렌즈 간 거리
       --known-mm    실치수를 아는 물체 (검산 가능)
  결과 (R, T) 는 X2 = R*X1 + T 규약(stereoCalibrate 와 동일)으로 calib_stereo.json 에 저장.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
스케일 출처를 파일에 남긴다 (doc_DEPTH_CODE_REVIEW R04)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  scale_status : provisional | measured | reference_scaled
  예전엔 임시 baseline 도 "measured-baseline" 으로 저장돼 콘솔의 "미확정"이 파일에
  남지 않았고, --rescale 을 결과 재생성 없이 반복하면 0.1 -> 0.05 -> 0.025 로 두 번
  보정됐다. 이제 results.json 의 meta.stereo_baseline_m 이 현재 파일의 baseline 과
  같을 때만 보정을 허용하고, 보정 이력(history)을 파일에 남긴다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
한계
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  - 장면이 평면 한 장에 가까우면 Essential 이 퇴화한다 (planar_ratio 로 확인).
  - 인라이어가 적으면(기본 30 미만) 포즈를 저장하지 않는다.
  - 두 카메라를 고정한 뒤에 찍어야 한다. 이후 움직이면 이 포즈는 무효다.
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
import lib_stereo as ls

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

HERE = Path(__file__).parent
SHOT_DIR = HERE / "shots"
OUT_JSON = HERE / "calib_stereo.json"
RESULTS_JSON = HERE / "results.json"
PLANAR_GATE = 0.90
MIN_INLIERS = 30


def parse_roi(text):
    v = [int(x) for x in text.replace(" ", "").split(",")]
    if len(v) != 4:
        raise argparse.ArgumentTypeError("ROI 형식은 x,y,w,h")
    return tuple(v)


def _write(path, d):
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(d, indent=2, allow_nan=False))
    os.replace(tmp, path)


def _save(calL, calR, R, T, ang, planar, n_match, n_inlier, method, scale_status, ref=None):
    """calib_stereo.json 을 stereoCalibrate 와 같은 키로 쓴다 (+ 스케일 출처)."""
    T = np.asarray(T, float).ravel()
    d = dict(K1=calL["K"].tolist(), dist1=calL["dist"].ravel().tolist(),
             K2=calR["K"].tolist(), dist2=calR["dist"].ravel().tolist(),
             R=np.asarray(R).tolist(), T=T.tolist(), image_size=list(calL["image_size"]),
             method=method, scale_status=scale_status, baseline_m=float(np.linalg.norm(T)),
             t_unit=(T / np.linalg.norm(T)).round(6).tolist(), rot_deg=float(ang),
             planar_ratio=float(planar), n_match=int(n_match), n_inlier=int(n_inlier),
             scale_ref=ref, history=[], time=datetime.now().isoformat(timespec="seconds"))
    _write(OUT_JSON, d)
    print(f"\n저장: {OUT_JSON}   scale_status={scale_status}")
    if scale_status == "provisional":
        print("  스케일이 임시값이다. z_reconstruct.py 결과는 비율만 맞다. --rescale 로 확정할 것.")
    else:
        print("  이제 z_reconstruct.py 가 mode=stereo 로 이 파일을 그대로 읽는다.")


def cmd_rescale(spec):
    """저장된 calib_stereo.json 의 baseline 만 보정한다.

    깊이는 baseline 에 정비례하므로 물체 하나의 실제 거리로
        새 baseline = 기존 baseline * (실제거리 / 복원거리)
    복원거리는 results.json 의 distance_m(카메라 중심까지 유클리드) 이다 — 줄자로 렌즈에서
    물체 앞면까지 잰 값과 대응한다. 광축 깊이(z_med_m)가 아니다.
    """
    if not OUT_JSON.exists():
        sys.exit(f"{OUT_JSON} 없음 — 먼저 --provisional 로 포즈를 구할 것")
    if not RESULTS_JSON.exists():
        sys.exit(f"{RESULTS_JSON} 없음 — 먼저 z_reconstruct.py 를 돌릴 것")
    try:
        idx_s, true_s = spec.split("=")
        idx, true_m = int(idx_s) - 1, float(true_s)
        assert true_m > 0
    except Exception:
        sys.exit("형식: --rescale <객체번호>=<실제거리m>   예: --rescale 1=0.62")

    cal = json.loads(OUT_JSON.read_text())
    res = json.loads(RESULTS_JSON.read_text())
    if "meta" not in res or "objects" not in res:
        sys.exit("results.json 형식이 예전 것 — z_reconstruct.py 를 다시 돌릴 것")
    m = res["meta"]
    if m.get("pose_source") != "stereo":
        sys.exit(f"results.json 은 {m.get('pose_source')} 모드 결과 — stereo 결과로만 보정 가능")
    cur_b = float(cal["baseline_m"])
    if abs(float(m["stereo_baseline_m"]) - cur_b) > 1e-9 * max(1.0, cur_b):
        sys.exit(f"results.json 은 baseline {m['stereo_baseline_m']*1000:.3f}mm 로 만든 결과인데 현재 "
                 f"calib_stereo.json 은 {cur_b*1000:.3f}mm — 이미 보정됐거나 갱신됐다. "
                 f"z_reconstruct.py 를 다시 돌린 뒤 --rescale 할 것 (중복 보정 방지)")
    objs = res["objects"]
    if not (0 <= idx < len(objs)):
        sys.exit(f"객체 번호가 1~{len(objs)} 범위를 벗어남")
    obj = objs[idx]
    if obj.get("status") != "ok":
        sys.exit(f"객체 {idx+1} ({obj.get('label')}) 의 status 가 {obj.get('status')} — 게이트를 통과한 객체로만 보정")
    est = float(obj["distance_m"])
    factor = true_m / est
    T = np.array(cal["T"]) * factor
    cal["history"] = cal.get("history", []) + [dict(time=datetime.now().isoformat(timespec="seconds"),
                                                    from_baseline_m=cur_b, factor=factor,
                                                    prev_status=cal.get("scale_status"))]
    cal.update(T=T.tolist(), baseline_m=float(np.linalg.norm(T)), scale_status="reference_scaled",
               method="essential+known-depth",
               scale_ref=dict(kind="distance", instance=idx + 1, label=obj.get("label"),
                              est_m=est, true_m=true_m, results_time=m.get("time")))
    _write(OUT_JSON, cal)
    print(f"객체 {idx+1} ({obj.get('label')}): 복원 거리 {est:.4f} m -> 실제 {true_m:.4f} m  (계수 {factor:.6f})")
    print(f"  baseline {cur_b*1000:.2f} -> {cal['baseline_m']*1000:.2f} mm   scale_status=reference_scaled")
    print(f"\n저장: {OUT_JSON}")
    print("  z_reconstruct.py --yolo 를 다시 돌리면 전부 미터 단위로 나온다.")
    print("  검증은 다른 객체의 거리를 실측과 비교하는 것으로 한다 (그 객체로 다시 rescale 하지 말 것).")


def main(a):
    calL = lc.load_calib(a.calib_l)
    calR = lc.load_calib(a.calib_r)
    imgL = cv2.imread(str(SHOT_DIR / "L.png"))
    imgR = cv2.imread(str(SHOT_DIR / "R.png"))
    if imgL is None or imgR is None:
        sys.exit("shots/L.png 또는 R.png 없음 — z_capture.py --stereo 를 먼저")
    for img, cal, nm in ((imgL, calL, "L"), (imgR, calR, "R")):
        hw = (img.shape[1], img.shape[0])
        if tuple(cal["image_size"]) != hw:
            sys.exit(f"{nm} 이미지 {hw} 가 캘리 해상도 {tuple(cal['image_size'])} 와 다름")
    print(f"L: {a.calib_l}  fx {calL['K'][0,0]:.1f}  화각 {calL['fov_deg'][0]:.1f}deg")
    print(f"R: {a.calib_r}  fx {calR['K'][0,0]:.1f}  화각 {calR['fov_deg'][0]:.1f}deg")

    # --- 1) 회전 + 이동 방향 -------------------------------------------------
    gL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    gR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    pL, pR, n_knn = ls._match_sift(gL, gR, None, a.ratio)
    print(f"\n특징점 매칭 {n_knn} -> ratio test {len(pL)}")
    if len(pL) < 20:
        sys.exit("매칭이 너무 적다 — 두 카메라가 같은 장면을 보는지, 장면에 질감이 있는지 확인할 것")
    nL = ls._to_normalized(pL, calL["K"], calL["dist"])
    nR = ls._to_normalized(pR, calR["K"], calR["dist"])
    thr = 1.0 / calL["K"][0, 0]
    _, hmask = cv2.findHomography(nL, nR, cv2.RANSAC, thr)
    planar = (int(hmask.sum()) if hmask is not None else 0) / len(pL)
    E, mask = cv2.findEssentialMat(nL, nR, np.eye(3), method=cv2.RANSAC, prob=0.999, threshold=thr)
    if E is None or E.shape != (3, 3):
        sys.exit("Essential 행렬 추정 실패")
    n_in, R, t, mask = cv2.recoverPose(E, nL, nR, np.eye(3), mask=mask)
    n_in = int(np.asarray(mask).ravel().astype(bool).sum())
    t = t.ravel()
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    print(f"  인라이어 {n_in}/{len(pL)}   [{'PASS' if n_in >= MIN_INLIERS else 'FAIL'}] (>= {MIN_INLIERS})")
    print(f"  [{'PASS' if planar < PLANAR_GATE else 'FAIL'}] 평면비 {planar:.3f} (< {PLANAR_GATE}) — 평면 한 장이면 Essential 이 퇴화한다")
    print(f"  상대 회전 {ang:.3f} deg,  이동 방향 t_hat = {np.round(t, 4).tolist()}  "
          f"({'2번이 오른쪽' if t[0] < 0 else '2번이 왼쪽'})")
    if planar >= PLANAR_GATE:
        sys.exit("평면 퇴화 — 깊이가 다른 대상이 여러 개 보이게 다시 찍을 것")
    if n_in < MIN_INLIERS:
        sys.exit("인라이어 부족 — 포즈를 저장하지 않는다. 질감·겹침을 늘려 다시 찍을 것")
    # 두 카메라에서 모두 앞에 있는 점의 비율 (양안 전방성)
    P1 = np.hstack([np.eye(3), np.zeros((3, 1))]); P2 = np.hstack([R, t.reshape(3, 1)])
    inl = np.asarray(mask).ravel().astype(bool)
    h4 = cv2.triangulatePoints(P1, P2, nL[inl].T, nR[inl].T); X1 = (h4[:3] / h4[3]).T
    X2 = X1 @ R.T + t
    front = float(np.mean((X1[:, 2] > 0) & (X2[:, 2] > 0)))
    print(f"  양안 전방성 {front*100:.1f}% (인라이어 중 두 카메라 모두 Z>0)")

    # --- 2) baseline 확정 ----------------------------------------------------
    if a.provisional:
        b = a.provisional / 1000.0
        print(f"\n임시 baseline {a.provisional:.1f} mm 로 저장 (스케일 미확정)")
        print("  다음: z_reconstruct.py --yolo -> 객체 하나 실제 거리 재기 -> z_stereo_pose.py --rescale N=거리m")
        _save(calL, calR, R, b * t, ang, planar, len(pL), n_in, "essential+provisional", "provisional")
        return
    if a.baseline_mm:
        b = a.baseline_mm / 1000.0
        print(f"\nbaseline = 자로 잰 렌즈 간 거리 {a.baseline_mm:.1f} mm  (검산 없음 — 나중에 --known-mm 으로 확인할 것)")
        _save(calL, calR, R, b * t, ang, planar, len(pL), n_in, "essential+measured-baseline", "measured",
              ref=dict(kind="lens-distance", baseline_mm=a.baseline_mm))
        return

    known = sorted((float(v) for v in a.known_mm.lower().split("x")), reverse=True)
    out1 = ls.reconstruct(imgL, imgR, calL, R, t.reshape(3, 1), roi=a.roi, calibR=calR, z_range=(0.01, 1000.0))
    box1, d1 = out1["box"], out1["diag"]
    if box1 is None:
        sys.exit(f"기준 물체 복원 실패 (매칭 {d1.get('n_ratio',0)} -> 최종 {d1.get('n_final',0)}) — ROI 와 질감을 확인할 것")
    s1 = box1["dims_sorted"][0]
    baseline = known[0] / 1000.0 / s1
    T = baseline * t
    print(f"\n기준 물체 (ROI {a.roi}, 실측 {known[0]:.1f}mm): 점 {d1['n_final']}개, 재투영 {d1['reproj_px']:.3f}px")
    print(f"  baseline=1 일 때 최대 치수 {s1:.6f} -> baseline = {baseline*1000:.2f} mm")
    out2 = ls.reconstruct(imgL, imgR, calL, R, T.reshape(3, 1), roi=a.roi, calibR=calR)
    dims = out2["box"]["dims_sorted"] * 1000
    print(f"검산: 복원 {dims[0]:.1f} x {dims[1]:.1f} mm  (실측 {known[0]:.1f} x {known[1] if len(known)>1 else float('nan'):.1f}), "
          f"거리 {out2['diag']['z_m'][1]:.3f} m")
    if len(known) > 1:
        e1 = abs(dims[1] - known[1]) / known[1] * 100
        print(f"  [{'PASS' if e1 < 15 else 'WARN'}] 2축 오차 {e1:.2f}% (< 15%) — 스케일 맞춤에 안 쓰인 축이라 진짜 검증값")
    _save(calL, calR, R, T, ang, planar, len(pL), n_in, "essential+known-size", "reference_scaled",
          ref=dict(kind="size", known_mm=known, roi=list(a.roi)))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--provisional", type=float, nargs="?", const=200.0, metavar="MM",
                   help="스케일 미확정 상태로 포즈만 저장 (기본 임시값 200mm). 나중에 --rescale 로 확정")
    g.add_argument("--rescale", metavar="N=METERS", help="results.json 의 N번 객체 실제 거리(m)로 baseline 보정. 예: 1=0.62")
    g.add_argument("--baseline-mm", type=float, help="자로 잰 두 렌즈 사이 거리 mm")
    g.add_argument("--known-mm", help="기준 물체 실측 치수, 예: 210x297 (--roi 와 함께)")
    ap.add_argument("--roi", type=parse_roi, help="기준 물체 영역 x,y,w,h (--known-mm 과 함께, 왼쪽 이미지 기준)")
    ap.add_argument("--calib-l", default=str(HERE / "calib.json"))
    ap.add_argument("--calib-r", default=str(HERE / "calib_phone.json"))
    ap.add_argument("--ratio", type=float, default=0.75)
    args = ap.parse_args()
    if args.rescale:
        cmd_rescale(args.rescale)
        raise SystemExit
    if args.known_mm and args.roi is None:
        ap.error("--known-mm 에는 --roi 가 필요함")
    main(args)
