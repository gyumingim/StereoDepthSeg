"""보정값 비례 변환: calib_phone_pair.json (보정한 크기, 예 640x480) → 같은 4:3 의 다른 크기 W×H.

왜 다시 캘리하지 않아도 되나
- 폰이 같은 센서 영역(active array 전체)을 4:3 그대로 축소/확대해서 내보내면, 화소 좌표만 s = W/W0 배가 된다.
  내부 파라미터: fx' = s·fx, fy' = s·fy, cx' = s·(cx+0.5) − 0.5, cy' 도 같은 식 (화소 중심 규약).
  왜곡 계수 (정규화 좌표 기준), 두 렌즈 사이 R·T (미터), R_sensor_to_left 는 그대로.
- 16:9 등 다른 비율은 센서를 위아래로 잘라 내므로 이 변환이 틀린다 → 지원하지 않는다 (ValueError).
- 전제(같은 센서 영역)는 폰이 4:3 YUV 크기를 모두 전체 화각으로 낸다는 가정이다 (확인 안 됨, 추정).
  그래서 첫 쌍의 정류 검증(epi dy, 기준도 s 배)을 크기마다 다시 돌려 보여 준다.
"""
import copy
import json
from pathlib import Path

import numpy as np

import lib_rectify as lr
from lib_phone_depth import load_phone_calibration


REPO = Path(__file__).resolve().parent.parent
# 기본 보정 파일 (2026-10-18): v2 = 원래 체커보드 보정 + 렌즈 5 회전 고침 (yaw 1.5 · pitch 2.0 mrad, 근거는 파일 안
# rotation_fix 와 scan3d/tools/calib_rot_fix.py). 옛 파일은 지우지 않고 SCAN3D_CALIB=v1 (또는 scan3d --calib-v1) 로 쓴다.
CALIB_FILES = {"v1": {"2+5": "calib_phone_pair.json", "2+6": "calib_phone_pair_2_6.json"},
               "v2": {"2+5": "calib_phone_pair_v2.json", "2+6": "calib_phone_pair_2_6_v2.json"}}


def default_calib(pair="2+5", version=None):
    """렌즈 쌍 기본 보정 파일 경로 (str). version None = 환경 변수 SCAN3D_CALIB (v1|v2), 없으면 v2.
    v2 파일이 없으면 v1 로 돌아간다."""
    import os
    pair = str(pair or "2+5").replace(",", "+")
    ver = (version or os.environ.get("SCAN3D_CALIB") or "v2").lower()
    p = REPO / CALIB_FILES.get(ver, CALIB_FILES["v2"])[pair]
    if not p.is_file():
        p = REPO / CALIB_FILES["v1"][pair]
    return str(p)


def parse_size(v):
    """'640x480' | (640, 480) | [640, 480] → (W, H) int."""
    if isinstance(v, str):
        w, _, h = v.lower().replace("×", "x").partition("x")
        v = (w, h)
    W, H = (int(x) for x in v)
    return W, H


def check_43(W, H):
    """4:3·짝수(NV21)·너무 작거나 크지 않은지. 아니면 ValueError."""
    if W * 3 != H * 4:
        raise ValueError(f"{W}x{H} 는 4:3 이 아님 (4:3 크기만 지원: 640x480, 800x600, 960x720, 1280x960 …)")
    if W % 2 or H % 2 or not (160 <= W <= 4096):
        raise ValueError(f"{W}x{H}: 가로 160~4096, 짝수여야 함")
    return W, H


def scale_calib(st, W, H):
    """보정 dict(st) → W×H 용 사본. K1/K2 만 바뀌고 dist·R·T 는 그대로. 원래 크기면 사본만."""
    W0, H0 = (int(x) for x in st["image_size"])
    check_43(W, H)
    if W * H0 != H * W0:
        raise ValueError(f"보정 크기 {W0}x{H0} 와 비율이 다름: {W}x{H}")
    s = W / W0
    out = copy.deepcopy(st)
    for k in ("K1", "K2"):
        K = np.array(st[k], float)
        K2 = K.copy()
        K2[0, 0] *= s
        K2[1, 1] *= s
        K2[0, 1] *= s
        K2[0, 2] = s * (K[0, 2] + 0.5) - 0.5
        K2[1, 2] = s * (K[1, 2] + 0.5) - 0.5
        out[k] = K2.tolist()
    out["image_size"] = [int(W), int(H)]
    out["calib_base_size"] = [W0, H0]
    out["calib_scale"] = s
    return out


def calib_base_size(path):
    return tuple(int(x) for x in json.loads(Path(path).read_text())["image_size"])


def rectify(st, alpha=0.0):
    cal = [dict(K=np.array(st[f"K{i}"], float), dist=np.array(st[f"dist{i}"], float), image_size=st["image_size"])
           for i in (1, 2)]
    return lr.rectify_maps(cal[0], cal[1], np.array(st["R"], float), np.array(st["T"], float), alpha=alpha)


def load_geometry(path, physical, size, logical="0"):
    """보정 파일 + 원하는 크기 → dict(st, swap, rp, size, base_size, scale, scaled, note).

    st 는 W×H 로 비례 변환한 보정 (image_size = W×H), rp 는 그 크기의 정류맵·정류 K."""
    W, H = check_43(*parse_size(size))
    base = calib_base_size(path)
    st0, swap = load_phone_calibration(path, logical, physical, base)
    st = scale_calib(st0, W, H)
    rp = rectify(st)
    scaled = (W, H) != base
    note = (f"보정은 {base[0]}x{base[1]} 값을 비례 변환 (×{W / base[0]:.3g})" if scaled else
            f"보정 크기 그대로 ({W}x{H})")
    return dict(st=st, swap=swap, rp=rp, size=(W, H), base_size=base, scale=W / base[0], scaled=scaled, note=note,
                vio=vio_lens0_maps(st))


def vio_lens0_K(st):
    """mono VIO 입력 카메라 'lens0u' = 왼쪽 렌즈(K1, 렌즈 2 초광각)를 왜곡만 편 핀홀 (정류 R1 없음, 원래 렌즈 축).
    f = K1 의 (fx+fy)/2, 주점 = K1 그대로 → 원래 초광각 화각(640 폭에서 수평 약 100°)을 거의 다 쓴다.
    정류 rect0 (f ≈ 463 @640, 수평 약 69°) 는 두 렌즈 공통 영역으로 잘려 화각이 좁고 같은 회전에 화소 이동이 1.5~1.8배라
    빠른 손 회전·민무늬 천장에서 특징점이 끊긴다 (rec_20261016_143702: rect0 mono = 13 s 만에 lost, lens0u = 26 s 끝까지,
    특징 0 프레임 59 → 12, 2026-10-16 오프라인 재생). make_ov_config.py mono 설정과 scan3d mono VIO 입력이 이 함수를 같이 쓴다."""
    K1 = np.asarray(st["K1"], float)
    f = 0.5 * (K1[0, 0] + K1[1, 1])
    return np.array([[f, 0.0, K1[0, 2]], [0.0, f, K1[1, 2]], [0.0, 0.0, 1.0]])


def vio_lens0_maps(st):
    """lens0u 왜곡 펴기 맵 dict(K, map=(mapx, mapy), size). 원래 렌즈 2 영상 → lens0u (R = I)."""
    import cv2
    W, H = (int(x) for x in st["image_size"])
    K = vio_lens0_K(st)
    mx, my = cv2.initUndistortRectifyMap(np.asarray(st["K1"], float), np.asarray(st["dist1"], float), np.eye(3), K,
                                         (W, H), cv2.CV_32FC1)
    return dict(K=K, map=(mx, my), size=(W, H))


def auto_ffs_scale(W, H, max_px=640 * 480):
    """FFS 입력 축소 비율: 화소 수가 max_px 를 넘으면 √(max_px/화소) (FFS 시간 ∝ 화소 수라 PC 부하를 그대로 둔다).
    0.25 아래로는 안 줄인다. 3자리 반올림."""
    s = min(1.0, float(np.sqrt(max_px / float(W * H))))
    return round(max(0.25, s), 3)
