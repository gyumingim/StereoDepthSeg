#!/usr/bin/env python3
"""
z_calibrate.py — 웹캠 내부 파라미터 캘리브레이션 실행 스크립트

사용 순서
  1) python z_calibrate.py --make-target
       target_s24.png 생성 → 폰으로 옮겨 전체화면으로 띄운다.
       폰을 가로로 눕혀 전체화면으로 띄우고, 위쪽 기준막대 2개의
       '왼쪽 모서리' 간격을 자로 재둔다.
  2) python z_calibrate.py --capture
       라이브 창을 보며 폰을 움직인다. HUD의 sharp/FOUND/size%를 보고
       "선명하면서 보드가 크게 잡히는" 거리를 먼저 찾은 뒤 SPACE로 15~20장.
  3) python z_calibrate.py --run --ruler-mm 97.5
       calib.json 생성 + 게이트 수치 출력.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
촬영 시 지켜야 할 것 (품질이 여기서 갈린다)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  - S24 화면은 A4의 약 1/9 면적이라 멀면 보드가 너무 작게 잡힌다.
    HUD의 size%(보드 폭 / 이미지 폭)가 최소 30% 이상 나오는 거리까지 붙일 것.
  - 대신 너무 붙으면 고정초점 웹캠이라 흐려진다. sharp 값이 급락하면 그 거리는 버린다.
  - 폰을 기울인 각도를 매번 바꿀 것. 정면만 찍으면 초점거리와 거리가 분리되지
    않아(구조적 모호성) K가 엉뚱하게 나온다. 좌우/상하로 20~40도씩 기울여 찍는다.
  - 화면 가장자리·모서리에도 보드가 오게 찍을 것. 왜곡계수는 가장자리 데이터로만
    추정된다. HUD에 지난 촬영 위치가 옅게 그려지니 빈 곳을 채운다.
  - 폰을 가로로 눕혀 화면을 꽉 채운다. 밝기는 중간 이상
    (너무 낮으면 PWM 조광 때문에 줄무늬가 생긴다).
  - 방 조명을 줄이고 각도를 틀어 화면 반사를 피한다.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

import lib_calib as lc
import lib_cam

HERE = Path(__file__).parent
SCREEN = "s24"           # 타겟을 어느 화면에 띄우는가 (lib_calib.SCREENS)
TARGET_PNG = HERE / "target_s24.png"
SHOT_DIR = HERE / "calib_shots"
CALIB_JSON = HERE / "calib.json"
STEREO_DIR = HERE / "calib_shots_stereo"
STEREO_JSON = HERE / "calib_stereo.json"


def set_paths(screen, cam):
    """--screen/--cam 에 따라 파일 경로를 정한다.

    카메라가 여러 대면(노트북 웹캠 / 폰 초광각) 캘리 결과를 섞으면 안 된다.
    K 는 카메라마다 다르므로 파일을 분리한다. cam 을 안 주면 기존 경로 그대로.
    """
    global SCREEN, TARGET_PNG, SHOT_DIR, CALIB_JSON, STEREO_DIR, STEREO_JSON
    SCREEN = screen
    TARGET_PNG = HERE / f"target_{screen}.png"
    sfx = f"_{cam}" if cam else ""
    SHOT_DIR = HERE / f"calib_shots{sfx}"
    CALIB_JSON = HERE / f"calib{sfx}.json"
    STEREO_DIR = HERE / f"calib_shots_stereo{sfx}"
    STEREO_JSON = HERE / f"calib_stereo{sfx}.json"

RMS_GATE_PX = 1.0     # 재투영 오차 합격선 (설계 시 합의한 게이트)


def _collect_corners(paths):
    """이미지 목록에서 코너를 검출한다. 해상도가 섞이면 즉시 실패시킨다.

    K는 해상도 종속이라 해상도가 다른 이미지를 섞으면 결과가 통째로 틀어진다.
    """
    corner_list, used, size = [], [], None
    for p in paths:
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if img is None:
            sys.exit(f"읽기 실패: {p}")
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif (img.shape[1], img.shape[0]) != size:
            sys.exit(f"{p.name} 해상도가 다름 {img.shape[1]}x{img.shape[0]} != {size}")
        c = lc.find_corners(img)
        if c is not None:
            corner_list.append(c)
            used.append(p.name)
    return corner_list, used, size


def cmd_make_target():
    info = lc.make_target(TARGET_PNG, screen=SCREEN)
    print(f"생성: {TARGET_PNG}")
    print(f"  화면 {info['size_px'][0]}x{info['size_px'][1]}, "
          f"보드 {info['board_px'][0]}x{info['board_px'][1]}px, "
          f"내부코너 {lc.PATTERN[0]}x{lc.PATTERN[1]}")
    print(f"  기준막대 x={info['ref_x']} → 간격 {info['ref_gap_px']}px")
    how = ("폰을 가로로 눕혀 전체화면으로 띄우고" if SCREEN == "s24"
           else "이미지 뷰어에서 '실제 크기(100%)'로 전체화면 표시하고")
    print(f"  {how}, 위쪽 기준막대 두 개의 '왼쪽 모서리' 간격을 자로 잴 것")
    lo, hi = lc.SCREENS[SCREEN]["expect_mm"]
    print(f"  (예상 범위 {lo}~{hi}mm — 여기서 크게 벗어나면 뷰어가 이미지를 1:1로")
    print(f"   안 띄운 것이니 실측값을 그대로 쓰면 됨)")


def cmd_capture(index, loopback=False, auto=0, min_move_px=35.0):
    """캘리 이미지 촬영.

    auto > 0 이면 자동 촬영 모드로 그 장수만큼 모으고 끝낸다.
    폰을 카메라로 쓸 때는 노트북 화면 전체가 체커보드라 HUD 를 볼 화면이 없다.
    그래서 사람이 SPACE 를 누르는 대신, 보드가 검출되고 직전 저장들과
    코너 위치가 min_move_px 이상 다를 때만 자동으로 저장한다.
    "충분히 다른 자세"만 모으므로 같은 각도가 중복 저장되지 않는다.
    """
    SHOT_DIR.mkdir(exist_ok=True)
    n = [len(list(SHOT_DIR.glob("*.png")))]
    past = []   # 지난 촬영들의 보드 외곽선 — 커버리지 확인용

    def on_frame(frame):
        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = lc.find_corners(gray, fast=True)   # HUD 전용 — 표시용 정밀도면 충분

        for hull in past:                      # 지난 촬영 위치를 옅게
            cv2.polylines(vis, [hull], True, (120, 120, 120), 1)

        if corners is None:
            lines = ["board: --", f"sharp: {lc.sharpness(gray):7.1f}", f"shots: {n[0]}"]
            color = (0, 200, 255)
        else:
            cv2.drawChessboardCorners(vis, lc.PATTERN, corners, True)
            pct = 100.0 * np.ptp(corners[:, 0, 0]) / frame.shape[1]
            # 보드 영역만의 선명도를 본다 (배경이 선명해도 의미 없음)
            x, y, w, h = cv2.boundingRect(corners.astype(np.int32))
            lines = ["board: FOUND", f"sharp: {lc.sharpness(gray[y:y+h, x:x+w]):7.1f}",
                     f"size : {pct:5.1f}% of width", f"shots: {n[0]}"]
            color = (0, 255, 0)
        return lib_cam.put_lines(vis, lines, color=color)

    saved_corners = []

    def _save(frame, corners):
        p = SHOT_DIR / f"{n[0]:03d}.png"
        if not cv2.imwrite(str(p), frame):
            return f"저장 실패: {p}"
        past.append(cv2.convexHull(corners.astype(np.int32)))
        saved_corners.append(corners.reshape(-1, 2).copy())
        n[0] += 1
        return f"저장 {p.name}  (총 {n[0]}장)"

    def on_grab(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = lc.find_corners(gray)
        if corners is None:
            return "거부: 코너 미검출 — 저장 안 함"
        return _save(frame, corners)

    def auto_step(frame):
        """자동 모드 한 스텝. 저장했으면 메시지, 아니면 None."""
        corners = lc.find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if corners is None:
            return None
        c = corners.reshape(-1, 2)
        for prev in saved_corners:
            if np.mean(np.linalg.norm(c - prev, axis=1)) < min_move_px:
                return None          # 이미 비슷한 자세를 찍었다
        return _save(frame, corners)

    cap = lib_cam.open_camera(index, set_format=not loopback)
    if auto:
        print(f"  자동 촬영 — 보드를 비추며 각도/거리를 천천히 바꿀 것. "
              f"{auto}장 모으면 자동 종료 (중단: Ctrl+C)")
        try:
            while n[0] < auto:
                ok, frame = cap.read()
                if not ok:
                    print("  프레임 읽기 실패")
                    break
                msg = auto_step(frame)
                if msg:
                    print(f"  {msg}")
        except KeyboardInterrupt:
            print("\n  사용자 중단")
        finally:
            cap.release()
    else:
        lib_cam.live_capture(cap, on_frame, on_grab, "calibrate")
    print(f"촬영 종료 — {SHOT_DIR} 에 {n[0]}장")


def cmd_run(ruler_mm):
    shots = sorted(SHOT_DIR.glob("*.png"))
    if not shots:
        sys.exit(f"촬영 이미지 없음: {SHOT_DIR} — 먼저 --capture 할 것")

    square_m = lc.square_m_from_ruler(ruler_mm, screen=SCREEN)
    gap = lc.SCREENS[SCREEN]["ref_gap_px"]      # 화면마다 다르다. 상수를 쓰면 틀린다.
    print(f"[{SCREEN}] 기준막대 실측 {ruler_mm}mm / {gap}px "
          f"→ 화소 피치 {ruler_mm/gap*1000:.4f}um "
          f"→ 사각형 {square_m*1000:.4f}mm")

    corner_list, used, size = _collect_corners(shots)
    print(f"코너 검출 {len(corner_list)}/{len(shots)}장")

    c = lc.calibrate(corner_list, size, square_m)
    K, per = c["K"], np.array(c["per_view_err"])

    # 커버리지: 이미지를 4x4로 나눠 코너가 들어간 칸 비율.
    # 왜곡계수는 가장자리 데이터로만 추정되므로 이 값이 낮으면 dist를 믿으면 안 된다.
    grid = np.zeros((4, 4), bool)
    for cl in corner_list:
        for x, y in cl[:, 0]:
            grid[min(3, int(y / size[1] * 4)), min(3, int(x / size[0] * 4))] = True

    print(f"\n{'='*58}\n결과\n{'='*58}")
    print(f"  fx, fy      : {K[0,0]:8.2f}, {K[1,1]:8.2f} px")
    print(f"  cx, cy      : {K[0,2]:8.2f}, {K[1,2]:8.2f} px  (이미지 중심 "
          f"{size[0]/2:.1f}, {size[1]/2:.1f})")
    print(f"  왜곡 dist   : {np.round(c['dist'].ravel(), 5).tolist()}")
    print(f"  화각 H,V    : {c['fov_deg'][0]:.1f}, {c['fov_deg'][1]:.1f} deg")
    print(f"  커버리지    : {grid.sum()}/16 칸")
    print(f"\n{'='*58}\n게이트\n{'='*58}")
    ok = c["rms"] < RMS_GATE_PX
    print(f"  [{'PASS' if ok else 'FAIL'}] 재투영 RMS {c['rms']:.4f} px  (< {RMS_GATE_PX})")
    print(f"         뷰별 최악 {per.max():.4f} px ({used[int(per.argmax())]}), "
          f"중앙값 {np.median(per):.4f}")
    print(f"  [{'PASS' if len(corner_list) >= 15 else 'WARN'}] 뷰 수 {len(corner_list)} (권장 >=15)")
    print(f"  [{'PASS' if grid.sum() >= 12 else 'WARN'}] 커버리지 {grid.sum()}/16 (권장 >=12)")

    lc.save_calib(CALIB_JSON, c)
    print(f"\n저장: {CALIB_JSON}")
    if not ok:
        print("게이트 미달 — 촬영을 다시 하거나 나쁜 뷰를 지우고 재실행할 것")


def cmd_capture_stereo(il, ir, loopback_r=False):
    """두 카메라로 동시에 체커보드를 찍는다. 양쪽 다 검출된 쌍만 저장한다.

    lib_cam.read_pair 로 같은 쌍을 프리뷰·판정·저장에 쓴다 (R09).
    """
    STEREO_DIR.mkdir(exist_ok=True)
    capL = lib_cam.open_camera(il)
    capR = lib_cam.open_camera(ir, set_format=not loopback_r)
    n = [len(list(STEREO_DIR.glob("*_L.png")))]
    print("  [SPACE] 촬영   [Q/ESC] 종료")
    try:
        while True:
            ok, fL, fR, skew = lib_cam.read_pair(capL, capR)
            if not ok:
                print("  프레임 읽기 실패")
                break
            cL = lc.find_corners(cv2.cvtColor(fL, cv2.COLOR_BGR2GRAY), fast=True)
            cR = lc.find_corners(cv2.cvtColor(fR, cv2.COLOR_BGR2GRAY), fast=True)
            vL, vR = fL.copy(), fR.copy()
            if cL is not None:
                cv2.drawChessboardCorners(vL, lc.PATTERN, cL, True)
            if cR is not None:
                cv2.drawChessboardCorners(vR, lc.PATTERN, cR, True)
            pair = np.hstack([cv2.resize(vL, (640, 360)), cv2.resize(vR, (640, 360))])
            both = cL is not None and cR is not None
            lib_cam.put_lines(pair, [f"L:{'OK' if cL is not None else '--'}  R:{'OK' if cR is not None else '--'}",
                                     f"pairs: {n[0]}   grab skew {skew:.1f} ms"],
                              color=(0, 255, 0) if both else (0, 200, 255))
            cv2.imshow("calibrate stereo", pair)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                # 저장 판정은 엄격 경로(풀 해상도)로, 같은 프레임에 대해
                if (lc.find_corners(cv2.cvtColor(fL, cv2.COLOR_BGR2GRAY)) is None
                        or lc.find_corners(cv2.cvtColor(fR, cv2.COLOR_BGR2GRAY)) is None):
                    print("  거부: 양쪽 모두 검출돼야 저장함")
                    continue
                cv2.imwrite(str(STEREO_DIR / f"{n[0]:03d}_L.png"), fL)
                cv2.imwrite(str(STEREO_DIR / f"{n[0]:03d}_R.png"), fR)
                n[0] += 1
                print(f"  쌍 저장 (총 {n[0]}, grab 간격 {skew:.1f} ms)")
    finally:
        capL.release(); capR.release(); cv2.destroyAllWindows()
    print(f"촬영 종료 — {STEREO_DIR} 에 {n[0]}쌍")


def cmd_run_stereo(ruler_mm):
    """각 카메라를 따로 캘리한 뒤 stereoCalibrate 로 상대 포즈를 구한다.

    CALIB_FIX_INTRINSIC 을 쓴다. 내부 파라미터를 각자 충분한 뷰로 먼저 확정해 두고
    상대 포즈만 푸는 쪽이, 전부 동시에 푸는 것보다 수렴이 안정적이다.
    여기서 나오는 (R,T)는 X2 = R*X1 + T 규약이라(calib3d.hpp:1933-1934)
    lib_stereo 가 1대 모드에서 쓰는 규약과 같다 — 그래서 코어를 공유할 수 있다.
    """
    pl = sorted(STEREO_DIR.glob("*_L.png"))
    pr = sorted(STEREO_DIR.glob("*_R.png"))
    if not pl or len(pl) != len(pr):
        sys.exit(f"쌍이 맞지 않음: L {len(pl)}장, R {len(pr)}장")

    square_m = lc.square_m_from_ruler(ruler_mm, screen=SCREEN)
    print(f"사각형 {square_m*1000:.4f}mm, 쌍 {len(pl)}개")

    cl, _, sizeL = _collect_corners(pl)
    cr, _, sizeR = _collect_corners(pr)
    if len(cl) != len(pl) or len(cr) != len(pr):
        sys.exit(f"검출 실패한 이미지가 있음 (L {len(cl)}/{len(pl)}, R {len(cr)}/{len(pr)})"
                 " — 쌍이 어긋나므로 해당 파일을 지우고 다시 실행할 것")
    if sizeL != sizeR:
        sys.exit(f"두 카메라 해상도가 다름 {sizeL} != {sizeR}")

    c1 = lc.calibrate(cl, sizeL, square_m)
    c2 = lc.calibrate(cr, sizeR, square_m)
    print(f"  카메라1 RMS {c1['rms']:.4f} px, 카메라2 RMS {c2['rms']:.4f} px")

    objp = lc.object_points(lc.PATTERN, square_m)
    rms, K1, d1, K2, d2, R, T, E, F = cv2.stereoCalibrate(
        [objp] * len(cl), cl, cr, c1["K"], c1["dist"], c2["K"], c2["dist"], sizeL,
        flags=cv2.CALIB_FIX_INTRINSIC)

    base = float(np.linalg.norm(T))
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    print(f"\n{'='*58}\n결과\n{'='*58}")
    print(f"  baseline    {base*1000:.2f} mm   (T = {np.round(T.ravel()*1000, 2).tolist()} mm)")
    print(f"  두 카메라 간 상대 회전 {ang:.3f} deg")
    print(f"  [{'PASS' if rms < RMS_GATE_PX else 'FAIL'}] 스테레오 RMS {rms:.4f} px (< {RMS_GATE_PX})")
    print(f"  2번 카메라가 오른쪽에 있으려면 T의 x성분이 음수여야 한다: "
          f"{'OK' if T.ravel()[0] < 0 else 'FAIL — 2번이 왼쪽이다. index를 바꿔 다시 찍을 것'}")

    STEREO_JSON.write_text(json.dumps(dict(
        K1=K1.tolist(), dist1=d1.ravel().tolist(),
        K2=K2.tolist(), dist2=d2.ravel().tolist(),
        R=R.tolist(), T=T.ravel().tolist(), image_size=list(sizeL),
        rms=float(rms), baseline_m=base, square_m=square_m, n_pairs=len(cl)), indent=2))
    print(f"\n저장: {STEREO_JSON}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--make-target", action="store_true", help="체커보드 PNG 생성")
    g.add_argument("--capture", action="store_true", help="캘리 이미지 촬영")
    g.add_argument("--run", action="store_true", help="캘리브레이션 실행")
    g.add_argument("--capture-stereo", action="store_true", help="2대 동시 캘리 촬영")
    g.add_argument("--run-stereo", action="store_true", help="2대 스테레오 캘리브레이션")
    ap.add_argument("--ruler-mm", type=float, help="자로 잰 기준막대 간격 mm (--run/--run-stereo 필수)")
    ap.add_argument("--screen", default="s24", choices=sorted(lc.SCREENS),
                    help="타겟을 띄울 화면 (기본 s24). 폰을 카메라로 쓸 땐 laptop")
    ap.add_argument("--cam", default="",
                    help="카메라 이름. 주면 calib_<이름>.json 으로 분리 저장")
    ap.add_argument("--index", type=int, default=0, help="/dev/videoN (기본 0)")
    ap.add_argument("--auto", type=int, default=0, metavar="N",
                    help="자동 촬영: 충분히 다른 자세 N장을 모으고 종료 "
                         "(화면 전체가 체커보드라 HUD 를 볼 수 없을 때)")
    ap.add_argument("--loopback", action="store_true",
                    help="대상이 v4l2loopback 가상장치일 때 (폰 카메라). "
                         "포맷을 이쪽에서 정하지 않고 공급자(scrcpy) 것을 따른다")
    ap.add_argument("--index-l", type=int, default=0)
    ap.add_argument("--index-r", type=int, default=2)
    a = ap.parse_args()
    set_paths(a.screen, a.cam)

    if a.make_target:
        cmd_make_target()
    elif a.capture:
        cmd_capture(a.index, a.loopback, a.auto)
    elif a.capture_stereo:
        cmd_capture_stereo(a.index_l, a.index_r, a.loopback)
    else:
        if a.ruler_mm is None:
            ap.error("--run/--run-stereo 에는 --ruler-mm 이 필요함 (자로 잰 기준막대 간격 mm)")
        (cmd_run_stereo if a.run_stereo else cmd_run)(a.ruler_mm)
