#!/usr/bin/env python3
"""
z_capture.py — 본 촬영 (1대 좌우 이동 / 2대 동시)

1대(권장): python z_capture.py --board
        폰에 체커보드(target_s24.png)를 띄워 물체 옆에 세워두고 두 장 찍는다.
        카메라를 아무렇게나 옮겨도 된다 — 상대 포즈를 가정하지 않고 "측정"한다.
        자도, 곧은 모서리도, 회전 주의도 필요 없다.
1대(수동): python z_capture.py --mono --baseline-mm 200
        체커보드를 장면에 둘 수 없을 때. 왼쪽 위치에서 SPACE -> 노트북을
        오른쪽으로 정확히 그만큼 밀고 -> SPACE. 회전 검사를 즉시 해 준다.
2대:       python z_capture.py --stereo --index-l 0 --index-r 2

왜 --board 가 나은가 (합성 검증, yaw 3도가 섞인 상황):
        체커보드 포즈   치수 오차  1.7~2.9%  (보드 거리 35~80cm 에서 평탄)
        평행이동 가정   치수 오차  39.6%     거리도 0.80m -> 1.11m 로 틀어짐
    평행이동 방식은 R=I 를 "가정"하므로 yaw 가 섞이면 그대로 무너진다.
    --board 는 두 사진의 보드에서 포즈를 직접 재므로 회전이 섞여도 반영된다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
1대 촬영 요령 (여기서 정확도가 갈린다)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  - 노트북을 평평한 책상 위에 두고, 곧은 모서리(자·책등)에 밑면을 붙인 채
    미끄러뜨린다. 들었다 놓으면 회전이 섞인다.
  - 화면 각도(힌지)를 절대 건드리지 말 것. pitch 회전이 그대로 들어간다.
  - 이동량은 자로 잰 값을 --baseline-mm 에 그대로 넣는다. 이 값이 미터 단위
    스케일의 유일한 출처라, 여기 오차가 곧 결과 오차다.
  - 두 장 사이에 장면이 움직이면 안 된다(사람·커튼·화면 등).
  - 깊이가 다른 대상이 여러 개 보이게 찍을 것. 평평한 벽 하나만 찍으면
    회전 검사가 원리적으로 불가능해진다(lib_stereo.check_pure_translation 참조).
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

import lib_calib as lc
import lib_cam
import lib_stereo as ls

HERE = Path(__file__).parent
SHOT_DIR = HERE / "shots"
CALIB_JSON = HERE / "calib.json"

ROT_GATE_DEG = 1.0      # 회전 합격선
PLANAR_GATE = 0.90      # 이 이상이면 장면이 평면이라 회전 검사 자체가 무의미


def _save(imgL, imgR, meta):
    SHOT_DIR.mkdir(exist_ok=True)
    for name, img in (("L.png", imgL), ("R.png", imgR)):
        if not cv2.imwrite(str(SHOT_DIR / name), img):
            sys.exit(f"저장 실패: {SHOT_DIR/name}")
    (SHOT_DIR / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"저장: {SHOT_DIR}/L.png, R.png, meta.json")


def _report_translation(imgL, imgR):
    """촬영 직후 평행이동 가정을 검사한다. 여기서 떨어지면 다시 찍는 게 맞다."""
    if not CALIB_JSON.exists():
        print("\ncalib.json 이 없어 회전 검사를 건너뜀 — 먼저 z_calibrate.py 를 돌릴 것")
        return
    r = ls.check_pure_translation(imgL, imgR, lc.load_calib(CALIB_JSON))
    if r is None:
        print("\n회전 검사 불가 — 매칭점이 8개 미만. 장면에 질감이 너무 없다.")
        return

    print(f"\n{'='*58}\n평행이동 가정 검사\n{'='*58}")
    print(f"  매칭 {r['n_match']}개, E 인라이어 {r['n_inlier']}개")
    print(f"  시차  중앙값 {r['disp_med']:.1f} px, 5백분위 {r['disp_p5']:.1f} px")
    print(f"  dy    중앙값 {r['dy_med']:.3f} px, RMS {r['dy_rms']:.3f} px")

    planar_ok = r["planar_ratio"] < PLANAR_GATE
    print(f"  [{'PASS' if planar_ok else 'FAIL'}] 평면비 {r['planar_ratio']:.3f} "
          f"(< {PLANAR_GATE}) — 장면에 깊이 차이가 있는가")
    if not planar_ok or not r["reliable"]:
        print("       장면이 평면 한 장에 가까워 회전 검사가 성립하지 않는다.")
        print("       깊이가 다른 대상이 여러 개 보이도록 다시 찍을 것.")
        return

    rot_ok = r["rot_deg"] < ROT_GATE_DEG
    print(f"  [{'PASS' if rot_ok else 'FAIL'}] 회전 {r['rot_deg']:.3f} deg (< {ROT_GATE_DEG})")
    print(f"       이동방향 편차 {r['trans_deg']:.3f} deg, "
          f"t={np.round(r['t_unit'], 3).tolist()} (기대 [-1,0,0])")
    if not rot_ok:
        print("       회전이 섞였다. 특히 yaw 는 시차에 offset 을 더해 거리를 통째로")
        print("       틀리게 만든다. 곧은 모서리에 붙여 다시 밀 것.")


def _report_board(imgL, imgR):
    """체커보드로 잰 상대 포즈를 보고한다. 여기 수치가 그대로 결과 스케일이다."""
    if not CALIB_JSON.exists():
        print("\ncalib.json 이 없어 포즈 측정 불가 — 먼저 z_calibrate.py 를 돌릴 것")
        return
    cal = lc.load_calib(CALIB_JSON)
    R, T, d = ls.pose_from_board(imgL, imgR, cal, square_m=cal["square_m"])
    print(f"\n{'='*58}\n체커보드 포즈 측정\n{'='*58}")
    print(f"  [{'PASS' if d['found_L'] else 'FAIL'}] 왼쪽 사진 보드 검출")
    print(f"  [{'PASS' if d['found_R'] else 'FAIL'}] 오른쪽 사진 보드 검출")
    if R is None:
        print("       보드가 두 사진에 모두, 가려지지 않고 온전히 보여야 한다.")
        return
    print(f"  baseline  {d['baseline_m']*1000:.2f} mm   "
          f"(보드까지 거리 {d['board_z_m'][0]:.3f} / {d['board_z_m'][1]:.3f} m)")
    print(f"  회전      {d['rot_deg']:.3f} deg  — 가정이 아니라 측정값이라 커도 괜찮다")
    print(f"  solvePnP 재투영 {d['pnp_reproj_px'][0]:.3f} / {d['pnp_reproj_px'][1]:.3f} px")
    print(f"  [{'PASS' if d['second_is_right'] else 'WARN'}] 두 번째가 오른쪽 "
          f"(T_x = {T[0,0]*1000:+.1f} mm)")
    if not d["second_is_right"]:
        print("       왼쪽으로 옮겨 찍었다. 동작은 하지만 L/R 이름과 방향이 반대다.")

    # 보드가 고정돼 있었는지 배경으로 교차 확인 — 이게 이 방식의 유일한 취약점이다
    for line in ls.board_check_lines(
            ls.check_board_consistency(imgL, imgR, cal, R, T)):
        print(line)


def cmd_board(index):
    """폰 체커보드를 장면에 두고 두 장 찍는다. 이동량 측정도, 회전 주의도 불필요."""
    shots = []

    def on_frame(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = lc.find_corners(gray, fast=True)          # HUD 전용
        vis = frame.copy()
        if c is not None:
            cv2.drawChessboardCorners(vis, lc.PATTERN, c, True)
        msg = ("shot 1/2: place phone board in scene" if len(shots) == 0 else
               "shot 2/2: move camera sideways (any way)" if len(shots) == 1 else
               "done - press Q")
        return lib_cam.put_lines(
            vis, [msg, f"board: {'FOUND' if c is not None else '--'}",
                  f"taken: {len(shots)}/2"],
            color=(0, 255, 0) if c is not None else (0, 200, 255))

    def on_grab(frame):
        if len(shots) >= 2:
            return "이미 2장 — Q로 종료"
        if lc.find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)) is None:
            return "거부: 보드 미검출 — 폰이 온전히 보여야 함"
        shots.append(frame.copy())
        return ("1번째 촬영. 카메라를 옆으로 옮기고 SPACE (아무렇게나 옮겨도 됨, "
                "단 폰과 물체는 건드리지 말 것)" if len(shots) == 1
                else "2번째 촬영 완료 — Q로 종료")

    lib_cam.live_capture(lib_cam.open_camera(index), on_frame, on_grab, "capture board")
    if len(shots) != 2:
        sys.exit(f"2장이 필요한데 {len(shots)}장만 찍힘")
    h, w = shots[0].shape[:2]
    _save(shots[0], shots[1], dict(mode="board", image_size=[w, h], index=index))
    _report_board(shots[0], shots[1])


def cmd_mono(index, baseline_mm):
    b = baseline_mm / 1000.0
    if CALIB_JSON.exists():
        fx = lc.load_calib(CALIB_JSON)["K"][0, 0]
        print(f"baseline {baseline_mm}mm, fx {fx:.1f}px 기준 예상 깊이 오차(시차오차 0.5px):")
        print("  " + "  ".join(f"{z}m:{ls.depth_error_m(z, b, fx)*1000:.0f}mm"
                               for z in (0.5, 1, 2, 3, 5)))

    shots = []
    def on_frame(frame):
        n = len(shots)
        msg = (f"shot 1/2: LEFT position" if n == 0 else
               f"shot 2/2: slide RIGHT by {baseline_mm:.0f} mm" if n == 1 else
               "done - press Q")
        return lib_cam.put_lines(frame.copy(), [msg, f"taken: {n}/2"])

    def on_grab(frame):
        if len(shots) >= 2:
            return "이미 2장 — Q로 종료"
        shots.append(frame.copy())
        return (f"{len(shots)}번째 촬영. 이제 오른쪽으로 {baseline_mm:.0f}mm 밀고 SPACE"
                if len(shots) == 1 else "2번째 촬영 완료 — Q로 종료")

    lib_cam.live_capture(lib_cam.open_camera(index), on_frame, on_grab, "capture mono")
    if len(shots) != 2:
        sys.exit(f"2장이 필요한데 {len(shots)}장만 찍힘")

    h, w = shots[0].shape[:2]
    _save(shots[0], shots[1], dict(mode="mono", baseline_m=b, image_size=[w, h], index=index))
    _report_translation(shots[0], shots[1])


def cmd_stereo(il, ir, loopback_r=False):
    """두 카메라로 한 쌍을 찍는다.

    lib_cam.read_pair 로 grab 둘 -> retrieve 둘 순서로 읽어 같은 쌍을 프리뷰와 저장에 쓴다.
    예전엔 왼쪽 read 뒤 오른쪽을 따로 read 하고 저장 시 오른쪽을 또 read 해서
    프리뷰·저장 프레임이 서로 달랐다 (doc_DEPTH_CODE_REVIEW R09). grab 간격(ms)을 meta 에 남긴다.
    하드웨어 동기는 아니므로 움직이는 대상은 grab_skew_ms 만큼 시차가 섞일 수 있다
    (예: 1m/s 물체, 20ms 차이 -> 2cm 이동 -> f=800,B=0.1,Z=2m 에서 시차 40px 중 8px 오염).
    """
    capL = lib_cam.open_camera(il)
    capR = lib_cam.open_camera(ir, set_format=not loopback_r)
    shot = None
    print("  [SPACE] 촬영   [Q/ESC] 종료")
    try:
        while True:
            ok, fL, fR, skew = lib_cam.read_pair(capL, capR)
            if not ok:
                print("  프레임 읽기 실패")
                break
            pair = np.hstack([cv2.resize(fL, (640, 360)), cv2.resize(fR, (640, 360))])
            lib_cam.put_lines(pair, ["LEFT | RIGHT", f"grab skew {skew:.1f} ms",
                                     f"taken: {1 if shot else 0}", "[SPACE] save  [Q] quit"])
            cv2.imshow("capture stereo", pair)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                shot = (fL.copy(), fR.copy(), skew)
                print(f"  한 쌍 촬영 (grab 간격 {skew:.1f} ms) — 다시 찍으면 덮어씀, Q로 종료")
    finally:
        capL.release(); capR.release(); cv2.destroyAllWindows()
    if shot is None:
        sys.exit("촬영된 쌍이 없음")
    fL, fR, skew = shot
    h, w = fL.shape[:2]
    _save(fL, fR, dict(mode="stereo", image_size=[w, h], index_l=il, index_r=ir,
                       loopback_r=loopback_r, grab_skew_ms=round(skew, 2)))
    print("\n2대 모드는 calib_stereo.json 의 (R,T) 를 쓰므로 평행이동 검사가 필요 없다.")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--board", action="store_true",
                   help="(권장) 폰 체커보드를 장면에 두고 2장 — 자·회전주의 불필요")
    g.add_argument("--mono", action="store_true", help="1대를 좌우로 옮겨 2장 (자 필요)")
    g.add_argument("--stereo", action="store_true", help="2대로 동시에 2장")
    ap.add_argument("--baseline-mm", type=float, help="--mono 필수: 자로 잰 이동거리")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--index-l", type=int, default=0)
    ap.add_argument("--index-r", type=int, default=2)
    ap.add_argument("--loopback-r", action="store_true",
                    help="오른쪽이 v4l2loopback 가상장치일 때 (폰 카메라 = /dev/video9)")
    a = ap.parse_args()
    if a.board:
        cmd_board(a.index)
    elif a.mono:
        if a.baseline_mm is None:
            ap.error("--mono 에는 --baseline-mm 이 필요함 (자로 잰 이동거리)")
        cmd_mono(a.index, a.baseline_mm)
    else:
        cmd_stereo(a.index_l, a.index_r, a.loopback_r)
