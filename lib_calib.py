"""
lib_calib.py — 웹캠 내부 파라미터(K, 왜곡계수) 캘리브레이션

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
타겟을 종이가 아니라 폰 화면으로 쓰는 이유
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  프린터가 없다. 그리고 노트북 내장캠은 화면 위 베젤에 있어 화면과 같은 방향을
  보므로 자기 노트북 화면을 촬영할 수 없다(외부 모니터도 미연결).
  → 쓸 수 있는 평면 타겟은 Galaxy S24 화면(2340x1080)뿐이고, 폰을 가로로
    눕혀 보드를 최대한 크게 띄운다(아래 S24_SCREEN_PX 주석 참조).
  LCD/OLED 타겟은 평면도가 종이보다 좋고 프린터 스케일 오차가 없다는 장점이 있다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
사각형 실치수를 ppi 스펙이 아니라 자로 재는 이유 (중요)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  불확실성이 두 개 있다.
    1) S24 ppi 스펙이 출처마다 다르다 — 삼성 418 / GSMArena 416 /
       2340x1080을 6.2인치로 직접 나누면 415.7.
       → 화소 피치 0.0608~0.0611 mm, 약 0.5% 편차.
    2) 갤러리 앱이 이미지를 1:1로 표시하지 않고 축소/확대하면 오차가 10%를
       넘을 수도 있는데, 화면만 봐서는 그걸 알아낼 방법이 없다.

  → PNG에 정확히 REF_GAP_PX 간격의 기준선 2개를 같이 그려 넣고,
    그 간격을 자로 재서 입력받는다.
        화소 피치 = 실측 mm / REF_GAP_PX
        사각형 실치수 = square_px * 화소 피치
    이 한 번의 측정으로 위 두 불확실성이 동시에 제거된다.
    (기준선은 보드 위/아래 여백에 대칭으로 놓아 코너 검출을 방해하지 않는다.)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
단위 규약
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  내부 계산과 calib.json 저장은 전부 **미터**.
  사용자 입출력(자로 잰 값, 사각형 크기 표기)만 mm를 쓴다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
근거
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  objectPoints 생성 규약: opencv 4.13 samples/python/calibrate.py:72-76
      pattern_size = (cols, rows)   # 내부 코너 수
      pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2) * square
  findChessboardCornersSB: opencv 4.13 calib3d.hpp:1452
      고전 findChessboardCorners보다 검출률이 높고 서브픽셀 정밀도를 자체 제공하므로
      cornerSubPix 후처리가 필요 없다.
  선명도 지표 variance of Laplacian: Pech-Pacheco et al., ICPR 2000
      "Diatom autofocusing in brightfield microscopy: a comparative study"
"""
import json
from pathlib import Path

import cv2
import numpy as np

# ── Galaxy S24 화면 규격 (삼성 공식: 6.2", FHD+ 2340x1080) ────────────────────
# 실제 화소 피치는 위 docstring대로 자로 재서 확정하므로 여기 ppi는 쓰지 않는다.
#
# 폰을 "가로"로 눕혀 쓴다. 화면의 긴 변(2340px)을 보드의 긴 변에 쓸 수 있어
# 보드 실물이 55.6x79.4mm(세로) -> 132.0x57.7mm(가로)로 면적이 1.7배 커진다.
# 보드가 이미지 폭의 30%를 채우는 거리가 13cm -> 31cm로 멀어지므로,
# 고정초점 웹캠이 근접에서 흐려지는 문제를 피할 수 있다.
S24_SCREEN_PX = (2340, 1080)   # (w, h) 가로 방향

# 타겟을 띄울 화면 프리셋. 폰이 카메라가 되면(scrcpy) 체커보드를 폰에 못 띄우므로
# 노트북 화면을 타겟으로 쓴다. 노트북 화면이 훨씬 커서 보드도 커진다:
#   s24    보드 131.2 x  57.4 mm
#   laptop 보드 315.3 x 137.6 mm
# ref_gap_px 는 자로 잴 구간이다. 길수록 상대 측정오차가 작지만 긴 자가 필요하다.
#   s24    2000px -> 약 121.5mm (15cm 자)
#   laptop 1000px -> 약 179.2mm (20cm 자)
SCREENS = {
    # expect_mm: 기준막대 간격의 예상 실측 범위. 이 범위를 크게 벗어나면 뷰어가
    # 이미지를 1:1 로 안 띄운 것인데, 그래도 실측값을 그대로 쓰면 된다.
    #   s24    ppi 스펙 415.7~418 에서 산출
    #   laptop EDID 상 344mm(+-1mm) / 1920px 에서 산출
    "s24":    dict(px=(2340, 1080), square_px=135, ref_gap_px=2000,
                   expect_mm=(121.5, 122.2)),
    "laptop": dict(px=(1920, 1080), square_px=110, ref_gap_px=1000,
                   expect_mm=(178.6, 179.7)),
}

# ── 체커보드 기본 설정 ────────────────────────────────────────────────────────
# 내부 코너 (cols, rows) = 15x6 -> 사각형 16x7칸, 코너 90개.
# 가로세로를 다르게 둬야 보드 방향이 유일하게 결정된다(정사각 격자는 90도 모호).
PATTERN = (15, 6)
SQUARE_PX = 135                # 16*135=2160px, 7*135=945px -> 2340x1080 안에 여백 67px
REF_GAP_PX = 2000              # 위쪽 여백의 기준막대 2개 간격(자로 재는 구간)


# ══════════════════════════════════════════════════════════════════════════════
# 타겟 생성
# ══════════════════════════════════════════════════════════════════════════════
def make_target(path, screen="s24", pattern=PATTERN, square_px=None,
                screen_px=None, ref_gap_px=None):
    """폰 화면에 띄울 체커보드 PNG를 만든다. 위쪽 여백에 기준막대 2개를 함께 그린다.

    기준막대는 보드 위쪽 여백 안에만 세로로 세운다. 가로 배치에서는 보드가
    화면을 거의 채우므로 좌우에 막대를 놓을 자리가 없기 때문이다.
    막대 두께가 있어도 "왼쪽 모서리끼리" 재면 간격이 정확히 ref_gap_px가 되도록
    각 막대를 x에서 오른쪽으로 THICK px 그린다.

    반환: dict(보드 배치 정보)
    """
    cfg = SCREENS[screen]
    square_px = square_px or cfg["square_px"]
    screen_px = screen_px or cfg["px"]
    ref_gap_px = ref_gap_px or cfg["ref_gap_px"]

    w, h = screen_px
    cols, rows = pattern
    bw, bh = (cols + 1) * square_px, (rows + 1) * square_px
    if bw > w or bh > h:
        raise ValueError(f"보드({bw}x{bh})가 화면({w}x{h})보다 큼 — square_px를 줄일 것")

    img = np.full((h, w), 255, np.uint8)
    x0, y0 = (w - bw) // 2, (h - bh) // 2

    # (r+c)가 짝수인 칸을 검게 -> 좌상단 칸이 검정
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y, x = y0 + r * square_px, x0 + c * square_px
                img[y:y + square_px, x:x + square_px] = 0

    THICK, PAD = 4, 8
    cx = x0 + bw // 2
    rx1, rx2 = cx - ref_gap_px // 2, cx + ref_gap_px // 2
    bar_bot = y0 - PAD                      # 막대는 보드 위쪽 여백 안에서만
    # 무작동 방지: 자리가 안 나오면 조용히 넘어가지 말고 즉시 실패시킨다.
    assert 0 <= rx1 and rx2 + THICK <= w, f"기준막대가 화면 밖 (rx1={rx1}, rx2={rx2})"
    assert bar_bot > 20, f"위쪽 여백이 {y0}px뿐이라 기준막대를 못 그림"
    img[0:bar_bot, rx1:rx1 + THICK] = 0
    img[0:bar_bot, rx2:rx2 + THICK] = 0

    # 안내 문구 — OpenCV putText는 한글을 못 그리므로 영문/숫자만 쓴다.
    # 아래쪽 여백에 한 줄로 넣어 보드와 기준막대를 건드리지 않는다.
    txt = (f"MEASURE TOP BARS: left-edge to left-edge = {ref_gap_px} px"
           f"   |   {cols}x{rows} inner, {square_px} px square")
    cv2.putText(img, txt, (x0, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), img):
        raise IOError(f"PNG 저장 실패: {path}")
    return dict(size_px=(w, h), board_px=(bw, bh), board_origin=(x0, y0),
                ref_x=(rx1, rx2), ref_gap_px=ref_gap_px)


def square_m_from_ruler(measured_mm, screen="s24", square_px=None, ref_gap_px=None):
    """자로 잰 기준선 간격(mm) → 사각형 한 변의 실치수(m).

    화소 피치 = measured_mm / ref_gap_px 이므로
    사각형 실치수 = square_px * 화소 피치.
    """
    if measured_mm <= 0:
        raise ValueError("실측값(mm)은 양수여야 함")
    cfg = SCREENS[screen]
    square_px = square_px or cfg["square_px"]
    ref_gap_px = ref_gap_px or cfg["ref_gap_px"]
    pitch_mm = measured_mm / ref_gap_px
    return square_px * pitch_mm / 1000.0


# ══════════════════════════════════════════════════════════════════════════════
# 검출 / 선명도
# ══════════════════════════════════════════════════════════════════════════════
def sharpness(gray):
    """라플라시안 분산 — 값이 클수록 선명. 초점 확인용 상대 지표."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def find_corners(gray, pattern=PATTERN, fast=False):
    """체커보드 내부 코너 검출. SB는 서브픽셀 정밀도를 자체 제공한다.

    fast=True 는 라이브 HUD 전용이다. 절반 해상도 + EXHAUSTIVE 없이 돌린다.
    실측(1280x720, 코너 90개):
        1280x720 + EXHAUSTIVE   198.5 ms  (~5 fps)   <- 원래 이걸 매 프레임 돌렸다
        1280x720 EXHAUSTIVE 없음 168.0 ms  (~6 fps)
        640x360  EXHAUSTIVE 없음  35.0 ms  (~28 fps)
    느린 주범은 EXHAUSTIVE 가 아니라 해상도였다. 640x360 에서도 코너 90개가
    그대로 검출되는 것을 확인했다.

    좌표는 원본 해상도로 되돌려 주지만 정밀도는 절반 격자 수준이므로
    **화면 표시용으로만** 쓸 것. 저장 판정과 실제 캘리브레이션은 fast=False 다.
    최종 캘리브레이션은 저장된 PNG에서 풀 해상도로 다시 검출하므로
    HUD 의 정밀도는 결과 정확도에 전혀 영향이 없다.

    반환: (N,1,2) float32 또는 None
    """
    flags = cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast and gray.shape[1] > 800:
        small = cv2.resize(gray, (gray.shape[1] // 2, gray.shape[0] // 2))
        ok, corners = cv2.findChessboardCornersSB(small, pattern, flags=flags)
        # 절반 해상도 좌표 -> 원본 좌표: x_full = 2*x_half + 0.5
        return corners * 2.0 + 0.5 if ok else None

    flags |= cv2.CALIB_CB_EXHAUSTIVE
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    return corners if ok else None


def object_points(pattern=PATTERN, square_m=None):
    """체커보드 좌표계 기준 코너 3D 좌표 (Z=0 평면, 단위 m).

    생성 규약은 opencv samples/python/calibrate.py:74-76 과 동일.
    """
    if square_m is None:
        raise ValueError("square_m 필수 — 자로 잰 값에서 square_m_from_ruler()로 구할 것")
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.indices(pattern).T.reshape(-1, 2) * square_m
    return objp


# ══════════════════════════════════════════════════════════════════════════════
# 캘리브레이션
# ══════════════════════════════════════════════════════════════════════════════
def calibrate(corner_list, image_size, square_m, pattern=PATTERN):
    """검출된 코너 목록 → K, dist.

    corner_list : find_corners()가 반환한 (N,1,2) 배열들의 리스트
    image_size  : (w, h)
    반환: dict — K, dist, rms, per_view_err, fov_deg 등
    """
    if len(corner_list) < 5:
        raise ValueError(f"뷰가 {len(corner_list)}장뿐 — 최소 5장, 권장 15~20장")

    objp = object_points(pattern, square_m)
    objpoints = [objp] * len(corner_list)

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, corner_list, image_size, None, None)

    # 뷰별 재투영 오차 — 어느 장이 나쁜지 보려고 따로 낸다.
    per_view = []
    for i in range(len(corner_list)):
        proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, dist)
        per_view.append(float(cv2.norm(corner_list[i], proj, cv2.NORM_L2)
                              / np.sqrt(len(proj))))

    w, h = image_size
    fx, fy = K[0, 0], K[1, 1]
    return dict(
        K=K, dist=dist, rms=float(rms), per_view_err=per_view,
        image_size=(int(w), int(h)), square_m=float(square_m), n_views=len(corner_list),
        fov_deg=(float(np.degrees(2 * np.arctan(w / (2 * fx)))),
                 float(np.degrees(2 * np.arctan(h / (2 * fy))))),
    )


def save_calib(path, c):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    d = dict(c)
    d["K"] = np.asarray(c["K"]).tolist()
    d["dist"] = np.asarray(c["dist"]).ravel().tolist()
    Path(path).write_text(json.dumps(d, indent=2))


def load_calib(path):
    d = json.loads(Path(path).read_text())
    d["K"] = np.array(d["K"], np.float64)
    d["dist"] = np.array(d["dist"], np.float64)
    d["image_size"] = tuple(d["image_size"])
    return d
