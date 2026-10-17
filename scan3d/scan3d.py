#!/usr/bin/env python3
"""scan3d — 실시간 3D 복셀 맵 스캐너 (WSL/우분투, venv_ffs).

폰(렌즈 2+5 / 2+6 + IMU) ──릴레이/adb forward──▶ 수신 ─┬─ IMU 동기화 ───────────────┐
                                                   └─ 프레임 쌍 → 정류 gray ────┴▶ OpenVINS 브리지 → 포즈
                                            키프레임 쌍 + 포즈 → FFS 깊이 + YOLO-seg → 복셀 맵 (클래스·물체)
녹화 재생: 폰 녹화 폴더(영상 + 센서 CSV) → 같은 파이프라인 (replay.py, --replay DIR).
웹 뷰어: http://localhost:8795 (WSL 0.0.0.0:8795, Windows localhost 포워딩) — 모드 선택·시작·정지·녹화 목록·가져와서 복원.
대기(idle) 중에는 GPU 를 안 쓴다: FFS·YOLO 로드와 깊이 스레드는 첫 실시간·녹화+실시간·복원 세션 때 한 번만 시작 (_ensure_depth).

모드 (웹 또는 --mode):
  live    실시간: 앱 --es mode live, PC 에서 VIO·깊이·맵 계산. PC 에 자동 저장 없음 (저장 버튼·--save-on-exit 만).
  record  녹화: 앱 --es mode record, PC 계산 없음. status.txt 폴링으로 녹화 시간·프레임 수만 표시.
  both    녹화+실시간: 폰은 녹화, PC 는 실시간 계산 (PC 에 저장 없음).
  idle    대기.
  replay  녹화 재생 (웹 "복원" 또는 --replay DIR).
연결 (--link): relay = WSL, Windows win_relay.py 가 바이트 중계 (기본 phone listen:8766);
              native = 우분투 직접, adb forward tcp:8765 후 127.0.0.1:8765 (앱 시작도 직접); auto = WSL 이면 relay.
adb 는 phone_link.find_adb (리눅스 adb 또는 WSL interop 으로 Windows adb.exe). 없으면 앱 제어 버튼만 꺼진다.

실행 예:
  venv_ffs/bin/python scan3d/scan3d.py --phone listen:8766          # 브리지는 자동 (--ov auto, 죽으면 재시작)
  venv_ffs/bin/python scan3d/scan3d.py --vio-mode stereo            # 두 렌즈 쌍 VIO (기본 mono = 왼쪽 렌즈 + IMU)
  venv_ffs/bin/python scan3d/scan3d.py --ov none        # VIO 없이 깊이·분할만 (고정 포즈)
  venv_ffs/bin/python scan3d/scan3d.py --mode idle --replay recordings/rec_20261016_120000 --every-frame
"""
import argparse
from collections import OrderedDict, deque
import json
import queue
import re
import shlex
import shutil
import signal
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(ROOT))

import cv2                      # noqa: E402
import numpy as np              # noqa: E402

import lib_rectify as lr        # noqa: E402
from lib_phone_depth import depth_colormap   # noqa: E402

import calib_scale              # noqa: E402
import frames                   # noqa: E402
import make_ov_config           # noqa: E402
import motion_rect              # noqa: E402
import ov_client                # noqa: E402
import phone_link               # noqa: E402
import phone_rx                 # noqa: E402
import rec_sync                 # noqa: E402
import replay_view              # noqa: E402
import voxel_map                # noqa: E402
from voxel_map import VoxelMap, class_color   # noqa: E402
import web_server               # noqa: E402

PHONE_MODES = ("live", "record", "both")

# 깊이 스테레오 렌즈 쌍: 왼쪽(스트림 0) = 렌즈 2 초광각 고정, 오른쪽만 바꾼다 (2026-10-16 사용자 결정 1단계).
#   2+5 = 초광각 + 광각 (기본, 체커보드 보정 calib_phone_pair.json, 기선 15.5 mm, 두 렌즈 시각차 약 -26~-33 ms)
#   2+6 = 초광각 + 망원 (공장 값 보정 calib_phone_pair_2_6.json — scan3d/tools/make_factory_pair_calib.py,
#         기선 32.4 mm, 시각차 약 +1~+3 ms = 거의 동시. 정류 화각 = 망원 화각 (약 29°x22°, 초광각 가운데를 약 4.8배 확대))
# 모노 VIO 는 렌즈 2 (lens0u) 만 쓰므로 쌍과 상관없이 같은 설정 (make_ov_config 는 늘 --calib = 2+5 보정으로 만든다).
LENS_PAIRS = ("2+5", "2+6")
# 비교용: '폰에 녹화만' 에서 렌즈 3개(2,5,6)를 동시에 녹화 (앱 --es physical 2,5,6, 실측 1440x1080 15 fps 됨).
# 복원은 두 쌍(2+6, 2+5)을 따로 → recon_<시각>_p26 / _p25 (replay_view.rec_pairs). 실시간 깊이는 한 쌍만.
ALL_LENSES = ("all", "2+5+6", "2,5,6")
ALL_REC_PHYSICAL = ("2", "5", "6")


def reloc_T_yaw(yaw):
    c, s_ = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])


def norm_pair(v):
    """'2+6' | '2,6' | ['2', '6'] → '2+6'. None·'' → None. 모르는 쌍이면 ValueError."""
    if v is None or v == "":
        return None
    if isinstance(v, (list, tuple)):
        s = "+".join(str(x).strip() for x in v[:2])
    else:
        s = str(v).strip().replace(",", "+").replace(" ", "")
    if s not in LENS_PAIRS:
        raise ValueError(f"렌즈 쌍 {v!r}: {' / '.join(LENS_PAIRS)} 중 하나여야 함")
    return s


def pair_physical(pair):
    """'2+6' → ['2', '6'] (앱 --es physical 순서 = 스트림 0, 1)."""
    return norm_pair(pair).split("+")


def sync_from_status(st):
    """앱 status 줄 꼬리 'sync=aligning phase=+3.2ms method=C try=4 rec_wait=1' / 'sync=locked phase=+0.4ms method=C'
    → dict (SPEC 15.4). status 를 읽는 녹화 모드용 (실시간은 web_server 가 metrics.json sync 를 씀). 없으면 None."""
    if not isinstance(st, dict) or not st.get("sync"):
        return None
    ph = str(st.get("phase") or "").replace("ms", "")
    try:
        phase = float(ph) if ph else None
    except ValueError:
        phase = None
    try:
        tries = int(st.get("try")) if st.get("try") else None
    except ValueError:
        tries = None
    return dict(state=st.get("sync"), phase_ms=phase, method=st.get("method"), c_try=tries,
                rec_wait=st.get("rec_wait") == "1", src="status")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="실시간 3D 복셀 맵 스캐너 (OpenVINS + FFS + YOLO-seg)")
    p.add_argument("--calib", default=None,
                   help="렌즈 2+5 보정 (기본 calib_phone_pair_v2.json = 체커보드 + 렌즈 5 회전 고침 2026-10-18, "
                        "calib_scale.default_calib)")
    p.add_argument("--aruco-board", default="auto",
                   help="ChArUco 보드 json (aruco_scale.py, ARUCO_SCALE.md): 키프레임마다 보드를 찾아 깊이 축척·렌즈 자세 잔차를 "
                        "웹 상태에 보여 줌. auto = scan3d/config/charuco/board_user_A4_6x6.json (있으면), none = 끔")
    p.add_argument("--aruco-apply", action="store_true",
                   help="보드로 잰 시차 보정 (s, Δd) 을 깊이에 실제로 적용 (기본은 관측만)")
    p.add_argument("--calib-v1", action="store_true",
                   help="옛 보정 (calib_phone_pair.json, calib_phone_pair_2_6.json, 회전 고침 전) 을 기본으로")
    p.add_argument("--physical", nargs=2, default=["2", "5"], help="앱 --physical 순서 (스트림 0 = 왼쪽 렌즈 2). --pair 를 주면 그 쌍")
    p.add_argument("--pair", choices=LENS_PAIRS, default=None,
                   help="깊이 렌즈 쌍 (시작 기본값, 웹에서 세션마다 바꿈): 2+5 = 초광각+광각 (기본, 체커보드 보정), "
                        "2+6 = 초광각+망원 (공장 값 보정 --calib-26, 기선 32 mm, 거의 동시 촬영)")
    p.add_argument("--calib-26", default=None,
                   help="렌즈 2+6 보정 (scan3d/tools/make_factory_pair_calib.py 가 공장 값으로 만듦). --calib 는 2+5 보정이고 "
                        "모노 VIO 설정(렌즈 2)도 늘 --calib 로 만든다")
    p.add_argument("--app-size", default="1440x1080",
                   help="폰 스트림·녹화 크기 WxH (4:3 만: 640x480, 800x600, 960x720, 1280x960 …). 보정 크기와 다르면 "
                        "보정값을 비례 변환 (calib_scale.py, 다시 캘리 안 함). 웹에서 세션마다 바꿀 수 있음")
    p.add_argument("--size", nargs=2, type=int, default=None, help="(예전 인자) W H — --app-size 와 같음")
    p.add_argument("--link", choices=["auto", "relay", "native"], default="auto",
                   help="relay = Windows win_relay.py 중계 (WSL), native = 우분투에서 adb forward 로 직접, auto = WSL 이면 relay")
    p.add_argument("--phone", default=None,
                   help="listen:PORT (relay 기본 listen:8766) | HOST:PORT 로 접속 (native 기본 127.0.0.1:8765; "
                        "HOST 생략 = WSL 기본 게이트웨이, win_relay --mode listen 과 짝)")
    p.add_argument("--adb", default=None, help="adb 경로 (기본: SCAN3D_ADB > PATH > tools/platform-tools/adb > WSL 이면 Windows adb.exe)")
    p.add_argument("--mode", choices=["live", "record", "both", "idle"], default="live", help="시작 모드")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--start-app", dest="start_app", action="store_true", default=None,
                   help="시작할 때 adb 로 앱 시작 (native 기본 켬, relay 기본 끔 = win_relay 가 시작)")
    g.add_argument("--no-start-app", dest="start_app", action="store_false")
    p.add_argument("--app-fps", type=int, default=15)
    p.add_argument("--app-imu-us", type=int, default=2500)
    p.add_argument("--app-focus", type=float, default=1.0)
    p.add_argument("--app-stream", choices=["h264", "jpeg", "nv21"], default="h264",
                   help="앱 스트림 형식 (SPEC 1.2): h264 = 기본 (I+P, 키 1 s, PC 에서 PyAV 로 풀기), "
                        "jpeg = USB 대역 약 1/10 (추정), nv21 = 원본 (USB 끊김·발열 큼)")
    p.add_argument("--app-jpeg-q", type=int, default=90, help="jpeg 품질 1~100 (--app-stream jpeg 일 때만)")
    p.add_argument("--app-sync", choices=["auto", "a", "b", "off"], default="off",
                   help="렌즈 위상 맞추기 (앱 --es sync, SPEC 15.3). 기본 off = 안 맞춤 (S24 는 다시 열어도 위상이 안 맞음 — 실측 "
                        "19번 실패; 대신 깊이를 쌍마다 움직임 보정 정류). auto = 카메라 다시 열기 (방법 C, 옵션), a·b = 시험용")
    p.add_argument("--app-rec-wait-sync-s", type=int, default=40,
                   help="녹화(record·both)가 위상 맞춤을 기다리는 최대 초 (앱 --ei rec_wait_sync_s, 0 = 안 기다림)")
    p.add_argument("--restart-app", action="store_true", default=True,
                   help="폰 쪽이 연달아 바로 끊기면 앱 다시 시작 (adb 있을 때, REVIEW 11)")
    p.add_argument("--pair-skew-ms", type=float, default=None,
                   help="쌍 묶기 허용 두 렌즈 시각차 ms. 기본 = 프레임 주기의 0.45배 (15 fps 면 30 ms) — 반 주기 미만이라 "
                        "가장 가까운 프레임끼리만 짝이 된다. 쌍마다 dt = t(lens1) − t(lens0) 를 같이 넘겨 깊이에서 움직임 보정")
    p.add_argument("--ov", default="auto",
                   help="auto = scan3d 가 브리지를 직접 띄움 (죽으면 자동 재시작, 로그 runs/ov_bridge_live_*.log) | "
                        "HOST:PORT (따로 띄운 TCP 브리지) | pipe:<명령> | none")
    p.add_argument("--ov-bridge", default=str(HERE / "ov_bridge" / "build" / "ov_bridge"), help="브리지 실행 파일")
    p.add_argument("--ov-config", default=None,
                   help="스테레오 estimator_config.yaml 고정 (--vio-mode stereo). 기본 = 세션 크기·fps 로 자동 "
                        "(make_ov_config.ensure_config → config/<W>x<H>_<fps>/, 없거나 낡으면 만듦)")
    p.add_argument("--ov-config-mono", default=None,
                   help="mono estimator_config.yaml 고정 (--vio-mode mono). 기본 = 자동 config/<W>x<H>_<fps>/mono/")
    # --reinit-state (2026-10-17): 추적이 끊기면 정지 초기화를 기다리지 않고 옛 상태로 바로 다시 시작 (ov_bridge/PROTOCOL.md)
    # --init-gate MEAN,MAX,DEG (2026-10-18, 기본 꺼짐): 정지 초기화 시차 기준을 DEG 로 넓히고 자이로로 확인 (ov_bridge/PROTOCOL.md).
    # rec_20261018_002349 는 초기화 못 함 → 추적 90 % 가 되지만 (그 뒤 벽만 보여 VIO 가 퍼짐), 참조 175446 재생 키프레임이
    # 142 → 111 로 줄어 기본으로 안 켬. 켜려면 "--auto-reset --reinit-state --init-gate 0.006,0.15,0.8".
    p.add_argument("--live-bridge-args", default="--auto-reset --reinit-state", help="실시간 브리지 추가 인자 (--ov auto)")
    p.add_argument("--replay-bridge-args", default="--auto-reset --reinit-state", help="재생용 브리지 추가 인자")
    p.add_argument("--reloc", choices=["on", "off"], default="on",
                   help="재위치 (reloc.py): VIO 리셋 뒤 새 조각을 옛 지도 키프레임과 특징 매칭·PnP 로 맞춰 같은 지도에 이어 쌓음. "
                        "붙기 전에는 새 조각을 따로 쌓고, 붙으면 그 키프레임들을 전역 좌표로 다시 넣는다")
    p.add_argument("--reloc-merge", choices=["on", "off"], default="on",
                   help="재위치로 조각이 붙을 때: on = 조각이 따로 쌓던 지도(복셀+TSDF)를 변환해 그대로 합침 (VoxelMap.merge), "
                        "off = 조각 키프레임 깊이를 전역 좌표로 다시 계산 (옛 방식, 비교용 · merge 실패 때도 이것)")
    p.add_argument("--reloc-test-split", default=None, metavar="T,YAW_DEG,X,Y,Z",
                   help="시험용: VIO 시각 T s 에 가짜 리셋을 내고 이후 포즈에 알려진 변환(yaw, 이동)을 곱함 → 재위치가 그 변환을 "
                        "얼마나 정확히 되찾는지 (정답 비교) 확인")
    p.add_argument("--map-load", default=None, metavar="NAME",
                   help="저장 지도 (map_store.py, scan3d/maps/NAME) 기준으로 세션 시작: 첫 키프레임들을 지도 기준 키프레임과 맞춰 "
                        "(재위치) 지도 좌표로 바꾼 뒤 이어 쌓거나 (--map-mode append) 위치만 추정 (localize)")
    p.add_argument("--map-mode", choices=["append", "localize"], default="append")
    p.add_argument("--map-save", default=None, metavar="NAME", help="복원(--replay)이 끝나면 결과를 이 이름의 지도로 저장")
    p.add_argument("--map-test", default=None, metavar="T,YAW_DEG,X,Y,Z",
                   help="시험용 (--map-load 와 함께): VIO 시각 T s 전 포즈는 버리고 (키프레임 없음), 뒤 포즈에 알려진 변환을 곱함 "
                        "→ 같은 녹화 앞부분으로 만든 지도에 뒷부분이 얼마나 정확히 붙는지 (정답 = 그 변환의 역)")
    p.add_argument("--replay-pair", default=None, choices=LENS_PAIRS, help="재생: 3렌즈 녹화에서 쓸 깊이 쌍 (기본 2+6)")
    p.add_argument("--replay-until", type=float, default=None, metavar="SEC", help="재생: 녹화 이 초까지만 (지도 나누기 시험)")
    p.add_argument("--bridge-restart-s", type=float, default=2.0, help="브리지가 죽으면 이만큼 뒤 다시 띄움 (s)")
    p.add_argument("--ov-images", choices=["rect"], default="rect",
                   help="VIO 입력 영상: rect = 정류 gray (명세 9.4, make_ov_config.py 설정과 짝). raw 는 설정과 안 맞아 제거 (REVIEW 5). "
                        "단 mono 는 설정 파일의 '# scan3d_vio_cam0: lens0u' 표시를 따라 렌즈 2 왜곡만 편 영상 (2026-10-16)")
    p.add_argument("--on-vio-reset", choices=["clear", "keep"], default="clear",
                   help="브리지 auto-reset (월드 원점이 바뀜) 때 맵을 지울지 (재생 중이면 지우기 전 조각 저장)")
    p.add_argument("--vio-mode", choices=["stereo", "mono"], default="mono",
                   help="mono (기본) = 왼쪽 렌즈(lens0) + IMU 만 VIO 에 (mono 설정으로 브리지; 두 렌즈가 동시에 안 찍혀서). "
                        "stereo = 두 렌즈 쌍 (시각차 큰 쌍은 버림, 옵션). 섞어 보내지 않는다 (섞으면 OpenVINS TrackKLT 가 죽음)")
    p.add_argument("--vio-stereo-skew-ms", type=float, default=5.0,
                   help="stereo 모드만: 두 렌즈 시각차가 이보다 크면 그 쌍을 VIO 에 안 보냄 (REVIEW4 B6)")
    p.add_argument("--motion-rect", choices=["on", "off"], default="on",
                   help="깊이 정류를 쌍마다 움직임 보정 (on, 기본): 두 촬영 순간 사이 폰 움직임(자이로 적분 회전 + VIO 속도×dt) "
                        "으로 실제 두 카메라 상대자세를 구해 그 쌍만 stereoRectify. off = 보정 (R,T) 고정 (예전 방식)")
    p.add_argument("--gyro-type", type=int, default=phone_rx.SENSOR_GYRO, help="4 = 자이로, 16 = uncalibrated")
    p.add_argument("--accel-type", type=int, default=phone_rx.SENSOR_ACCEL, help="1 = 가속도계, 35 = uncalibrated")
    p.add_argument("--voxel", type=float, default=0.03, help="복셀 크기 m")
    p.add_argument("--min-hits", type=int, default=2, help="표시할 최소 관측 수")
    p.add_argument("--depth-range", nargs=2, type=float, default=[0.3, 2.5], help="적분 깊이 범위 m (최대 3 m)")
    p.add_argument("--tsdf", choices=["on", "off"], default="on", help="TSDF 융합 + 메시 (tsdf_map.py, GPU)")
    p.add_argument("--tsdf-voxel", type=float, default=None, help="TSDF 복셀 m (기본 tsdf_map.DEFAULTS = 0.02, 근거 주석 거기)")
    p.add_argument("--tsdf-depth", nargs=2, type=float, default=None, metavar=("ZMIN", "ZMAX"),
                   help="TSDF 에 넣는 깊이 범위 m (기본 0.3 2.5 + 노이즈 σ 15 cm 한계 = 2+5 는 약 1.5 m; --depth-range 안쪽만 의미 있음)")
    p.add_argument("--iters", type=int, default=4, help="실시간 FFS GRU 반복")
    p.add_argument("--scale", type=float, default=None,
                   help="FFS 입력 축소 (0,1]. 기본 자동 = 화소 수가 --ffs-max-px 를 넘으면 √(max/화소) 로 줄임 (FFS 시간 ∝ 화소 수)")
    p.add_argument("--ffs-max-px", type=int, default=640 * 480,
                   help="--scale 자동일 때 FFS 입력 최대 화소 수 (기본 640x480 = 지금 PC 부하 그대로)")
    p.add_argument("--stride", type=int, default=3, help="3D 점 화소 간격 (2~4)")
    p.add_argument("--yolo-model", default="yolo11m-seg.pt")
    p.add_argument("--yolo-conf", type=float, default=0.25)
    p.add_argument("--yolo-rotate", action="store_true", help="YOLO 입력만 시계 90° 회전 (영상이 누워 있을 때)")
    p.add_argument("--no-yolo", dest="no_yolo", action="store_true", default=True,
                   help="물체 구분(YOLO-seg) 끔 — 기본 (2026-10-16 사용자: 인식률이 낮아 뺌)")
    p.add_argument("--yolo", dest="no_yolo", action="store_false", help="물체 구분(YOLO-seg) 켜기")
    p.add_argument("--device", default="0")
    p.add_argument("--kf-trans", type=float, default=0.015, help="키프레임 이동 m (2026-10-16: 0.03 → 0.015, 더 촘촘히)")
    p.add_argument("--kf-rot", type=float, default=2.5, help="키프레임 회전 deg (2026-10-16: 5 → 2.5)")
    p.add_argument("--kf-dt", type=float, default=0.0, help="이 시간(s) 지나면 움직임 없어도 키프레임 (0 = 끔)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8795)
    p.add_argument("--out", default=str(HERE / "runs"), help="저장 버튼 결과 폴더")
    p.add_argument("--rec-root", default=str(HERE / "recordings"), help="가져온 녹화 폴더 위치 (기본 scan3d/recordings)")
    p.add_argument("--no-rec-auto", action="store_true",
                   help="폰 녹화 자동 가져오기·자동 복원 대기열 끄기 (rec_sync.py, 웹에서도 켜고 끔)")
    p.add_argument("--replay", default=None, metavar="DIR", help="시작하자마자 이 녹화 폴더를 재생 (이름이면 --rec-root 아래)")
    p.add_argument("--replay-out", default=None, help="재생 결과 폴더 (기본 <녹화>/recon_<시각>)")
    p.add_argument("--every-frame", action="store_true", help="재생: 키프레임 기준 없이 포즈가 있는 모든 쌍에 FFS")
    p.add_argument("--replay-iters", type=int, default=8, help="재생 FFS GRU 반복 (실시간보다 높게)")
    p.add_argument("--exit-after-replay", action="store_true", help="--replay 가 끝나면 종료")
    p.add_argument("--skip-epi", action="store_true", help="첫 쌍 정류 검증 생략")
    p.add_argument("--save-on-exit", action="store_true")
    a = p.parse_args(argv)
    a.depth_range[1] = min(a.depth_range[1], 3.0)      # 3 m 넘는 깊이는 쓰지 않는다 (baseline 15.5 mm)
    if a.pair is None:
        a.pair = "2+6" if sorted(str(x) for x in a.physical) == ["2", "6"] else "2+5"
    if sorted(str(x) for x in a.physical) != sorted(pair_physical(a.pair)):
        a.physical = pair_physical(a.pair)
    if a.size:
        a.app_size = f"{a.size[0]}x{a.size[1]}"
    try:
        a.size = list(calib_scale.check_43(*calib_scale.parse_size(a.app_size)))   # 지금 세션 크기 (apply_geometry 가 갱신)
    except ValueError as e:
        p.error(str(e))
    if a.scale is not None and not 0.0 < a.scale <= 1.0:
        p.error("--scale 은 (0,1]")
    a.stride = int(np.clip(a.stride, 1, 8))
    if a.link == "auto":
        a.link = "relay" if phone_link.is_wsl() else "native"
    if a.phone is None:
        a.phone = "listen:8766" if a.link == "relay" else "127.0.0.1:8765"
    if a.start_app is None:
        a.start_app = a.link == "native"
    ver = "v1" if a.calib_v1 else None
    a.calib = a.calib or calib_scale.default_calib("2+5", ver)
    a.calib_26 = a.calib_26 or calib_scale.default_calib("2+6", ver)
    if a.aruco_board == "auto":
        b = HERE / "config" / "charuco" / "board_user_A4_6x6.json"
        a.aruco_board = str(b) if b.is_file() else None
    elif str(a.aruco_board).lower() in ("none", "off", ""):
        a.aruco_board = None
    return a


def pair_skew_ms_for(a, fps):
    """쌍 묶기 기준 ms: --pair-skew-ms 를 주면 그 값, 아니면 0.45 × 프레임 주기 (반 주기 미만 → 가장 가까운 프레임끼리만)."""
    if a.pair_skew_ms is not None:
        return float(a.pair_skew_ms)
    return round(0.45 * 1000.0 / max(1, int(fps)), 2)


class IdleVIO:
    """세션 없을 때 자리 채움."""
    max_inflight = max_pending = 0

    def push_imu(self, *a):
        pass

    def push_cam(self, *a):
        pass

    def reset(self):
        pass

    def pending(self):
        return 0

    def close(self):
        pass

    def stats(self):
        return dict(addr="-", init=False, status="idle")


class Scanner:
    def __init__(self, a):
        self.a = a
        self.log_lines = deque(maxlen=300)
        # 보정·정류맵은 세션 해상도마다 apply_geometry 가 만든다 (__init__ 끝에서 --app-size/--app-fps 로 처음 한 번)
        self.geom = None                                 # dict(st, swap, rp, size, fps, ffs_scale, base_size, scale, scaled, note)
        self.st = self.swap = self.rp = None
        self.want = dict(size=tuple(a.size), fps=int(a.app_fps), ffs_scale=a.scale,   # 실시간·녹화 세션에 쓸 값 (웹이 바꿈)
                         pair=a.pair)
        self.boot_size = tuple(a.size)                   # 외부 브리지(--ov HOST:PORT)가 쓰는 크기로 본다 (REVIEW6 R1)
        self.vio_cfg_used = None                         # 지금 브리지가 쓰는 estimator_config.yaml (저장 때 복사)
        voxel_map.configure_tsdf(enabled=a.tsdf == "on", voxel_m=a.tsdf_voxel,
                                 z_min=a.tsdf_depth[0] if a.tsdf_depth else None,
                                 z_max=a.tsdf_depth[1] if a.tsdf_depth else None)
        self.vmap = VoxelMap(a.voxel, a.min_hits)
        self.paused = False
        self.t0_ns = None
        self.last_cam_ns = -1
        self.recent = OrderedDict()                      # 카메라 t_ns (lens0 노출 가운데) → (pair, dt_ns = t(lens1) − t(lens0))
        self.recent_lock = threading.Lock()
        self.vio_in = deque(maxlen=2)
        self.vio_evt = threading.Event()
        self.traj = []                                   # [(t, T_world_rect0)]
        self.traj_lock = threading.Lock()
        self.cur_T = None
        self.last_kf = None                              # (t, T)
        self.kf_queue = queue.Queue(maxsize=8)           # (pair, T, t, epoch)
        self.worker_busy = False
        self.want_iters = a.iters
        self.ds = None
        self.aruco = None                                # 보드 관측기 (aruco_scale, --aruco-board). 깊이 워커에서 observe
        if a.aruco_board:
            try:
                import aruco_scale
                self.aruco = aruco_scale.make(a.aruco_board, log=self.log)
            except Exception as e:                       # 보드가 없어도 스캐너는 돈다
                print(f"[aruco] 보드 관측 끔: {e!r}", flush=True)
        self.dstats = dict(ffs_ms=None, yolo_ms=None, total_ms=None, keyframes=0, points=0, fps=0.0, ready=False,
                           error=None, iters=None, loading=False, loaded=False,   # loaded = 로드 시작함 (지연 로드)
                           load_t0=None, load_s=None, load_stage=None)   # 로드 진행 (웹 진행 표시, REVIEW5 N2)
        self.depth_thread = None                         # 첫 live/both/replay 때 _ensure_depth 가 시작 (대기 중 GPU 안 씀)
        self.depth_lock = threading.Lock()
        self.last_pair = None                            # VIO 에 들어간 가장 최근 (왼쪽, 오른쪽) — 카메라 창 (재생 포함)
        self.kf_times = deque(maxlen=20)
        self.panel = {}
        self.panel_lock = threading.Lock()
        self.objects, self.objects_t = [], 0.0
        self.epi = None
        self.vio_mono = self.vio_stereo = 0
        self.vio_cam0 = frames.VIO_CAM0_RECT0              # 브리지 설정의 cam0 종류 (_bridge_cmd 가 설정 표시로 정함)
        self.vio_skew_dropped = 0                        # stereo 모드에서 시각차가 커서 VIO 에 안 보낸 쌍
        self.skew_ms = deque(maxlen=600)                  # 최근 쌍의 두 렌즈 시각차 (ms) → 중앙값·p95 를 웹에
        self.dt_ms = deque(maxlen=600)                    # 최근 쌍의 부호 있는 dt = t(lens1) − t(lens0) (ms)
        self.gyro_buf = deque(maxlen=12000)               # (t_ns, gyro) 약 30 s — ImuSync 출력 (실시간·재생 같음), 움직임 보정 정류용
        self.mrect = dict(motion=0, fixed=0, skipped=0, last=None, recent=deque(maxlen=12))   # 키프레임 움직임 보정 통계
        self.pair_hist = deque(maxlen=64)                 # (monotonic, 프레임 합, 짝 없어 버린 합) → 최근 버린 비율 (REVIEW5 N1)
        # 재위치 (reloc.py, _reloc_* 함수): 조각 좌표 → 전역 변환, 붙기 전 보관한 옛(전역) 지도, 다시 넣을 키프레임
        self.reloc = None                                 # Relocalizer (첫 사용 때 만듦)
        self._reloc_lock = threading.RLock()
        self._seg_T = np.eye(4)                           # 전역 ← 지금 VIO 월드 (None = 아직 못 붙임)
        self._seg_n = 0                                   # VIO 조각 번호
        self._pend = None                                 # 붙기 전: dict(main=옛 지도 묶음, jobs=[키프레임], seg)
        self._epoch_hi = 0
        self._requeue = deque()                           # 붙은 뒤 전역 좌표로 다시 넣을 키프레임 job
        self._test_split = None                           # --reloc-test-split (T, T_fake), 쓴 뒤 'done'
        self._post_checks = 0
        # 저장 지도 (map_store.py): map_req = 다음 세션이 쓸 지도 {name, mode}, _map_sess = 지금 세션 상태 (_map_begin)
        self.map_req = dict(name=a.map_load, mode=a.map_mode) if a.map_load else None
        self.map_save_next = a.map_save                  # 복원이 끝나면 이 이름으로 지도 저장 (replay.py)
        self._map_sess = None
        self._map_test = None
        self.vio_events = []                              # [{t, kind, reason, reinit, ...}] → 복원 summary
        self.vio_auto_resets = 0
        self.bridge_restarts = 0
        self.bridge_fails = 0                             # 연속 실패(시작 실패·곧 죽음) → 다시 시도 간격 (B11)
        self.bridge_t = 0.0                               # 지금 브리지를 띄운 시각 (monotonic)
        self.bridge_next = 0.0                            # 브리지 시작 실패 뒤 다음 시도 시각 (B9)
        self.receiver = None
        self.timing_src = None                            # 재생 중이면 녹화의 FrameTimingIndex
        self.stopped = False
        self.cam0_is_rect = True
        self.pose_cv = threading.Condition()
        self.pose_seen_ns = -1
        self._replay_kf = None
        self.replay_every = False
        # 세션
        self.mode = "idle"
        self.session_lock = threading.RLock()
        self.session_stop = None
        self.app_stop_thread = None                       # '정지'가 띄운 앱 정지 스레드 (새 세션 시작 전에 join)
        self.app_status = None
        self.rec_status = None
        self.pull_progress = None
        self.replayer = None
        self.replay_finished = False
        self.last_result = None
        self._phone_recs, self._phone_recs_t = None, 0.0
        self.replay_tl = []                               # 복원 키프레임 [{i, t, fr: [(스트림, t_ns)], T}] → timeline.json
        self.replay_depth = {}                            # 키프레임 번호 → 작은 깊이 JPEG (복원만)
        self._best_seg = None                             # 복원 중 VIO 리셋 전 가장 큰 조각 (restore_best_segment)
        self.pose_log = None                              # 복원: 브리지 포즈·이벤트 기록 (3렌즈 녹화 → 다른 쌍 복원이 다시 씀, replay.py)
        self.rec3 = False                                 # 지금(또는 마지막) 녹화 세션이 렌즈 3개 모두 녹화인가
        self.link = phone_link.PhoneLink(a.adb, self.log, physical=",".join(a.physical), fps=a.app_fps,
                                         focus=a.app_focus, imu_us=a.app_imu_us,
                                         stream_fmt=a.app_stream, jpeg_q=a.app_jpeg_q, width=a.size[0], height=a.size[1],
                                         sync=a.app_sync, rec_wait_sync_s=a.app_rec_wait_sync_s)
        self.log(f"[link] {a.link} · phone {a.phone} · adb {self.link.adb or '없음'} ({self.link.kind or '-'})")
        self.live_vio = None
        self.vio = IdleVIO()
        self.vio_error = None
        self.imu = phone_rx.ImuSync(self.on_imu, a.gyro_type, a.accel_type)
        self.apply_geometry(tuple(a.size), a.app_fps, a.scale, a.pair)
        self.recsync = rec_sync.RecSync(self, enabled=not a.no_rec_auto)   # 폰 녹화 자동 가져오기 + 자동 복원 대기열

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        self.log_lines.append(line)

    def pair_skew_ms(self):
        """지금 세션 fps 의 쌍 묶기 기준 ms (기본 0.45 × 프레임 주기)."""
        return pair_skew_ms_for(self.a, self.geom["fps"] if self.geom else self.a.app_fps)

    # ---------------- 해상도·fps (세션마다) ----------------
    def _norm_geometry(self, size, fps, ffs_scale):
        """검증·정규화: (W,H) 4:3, fps 1~60, ffs_scale (0,1] 또는 None(자동). 잘못되면 ValueError."""
        W, H = calib_scale.check_43(*calib_scale.parse_size(size))
        fps = int(round(float(fps if fps is not None else self.a.app_fps)))
        if not 1 <= fps <= 60:
            raise ValueError(f"fps {fps}: 1~60 이어야 함")
        if ffs_scale is not None:
            ffs_scale = float(ffs_scale)
            if not 0.0 < ffs_scale <= 1.0:
                raise ValueError(f"ffs_scale {ffs_scale}: (0,1] 이어야 함")
        return (W, H), fps, ffs_scale

    def calib_for(self, pair):
        """렌즈 쌍의 보정 파일: 2+5 = --calib (체커보드), 2+6 = --calib-26 (공장 값). 없으면 ValueError (쉬운 말)."""
        pair = norm_pair(pair)
        path = Path(self.a.calib if pair == "2+5" else self.a.calib_26)
        if not path.is_file():
            raise ValueError(f"렌즈 {pair} 보정 파일이 없습니다: {path} — "
                             f"'venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py' 로 만드세요")
        return path

    def apply_geometry(self, size, fps=None, ffs_scale=None, pair=None):
        """세션 해상도·fps·렌즈 쌍 적용 (세션이 멈춘 상태에서, session_lock 안 또는 __init__ 에서 부른다).

        - 렌즈 쌍 (pair, None = 지금 쌍): 2+5 | 2+6 → 보정 파일 (calib_for)·앱 --es physical·카메라 이름.
        - 보정: calib_phone_pair.json (보정 크기) → W×H 비례 변환 (K 만, dist·R·T 그대로) → 정류맵·정류 K (rp).
        - FFS 입력 축소: ffs_scale 또는 자동 (--scale, 없으면 --ffs-max-px 기준).
        - 앱 인자 (PhoneLink width/height/fps), VIO 설정 키 (_vio_key; 다르면 다음 _ensure_live_vio 가 브리지 재시작).
        - 크기가 바뀌면 첫 쌍 정류 검증(epi)을 다시 한다. 진행 중 키프레임은 크기가 안 맞으면 버린다."""
        (W, H), fps, ffs_scale = self._norm_geometry(size, fps, ffs_scale)
        if ffs_scale is None:
            ffs_scale = calib_scale.auto_ffs_scale(W, H, self.a.ffs_max_px)
        g = self.geom
        pair = norm_pair(pair) or (g["pair"] if g is not None else self.a.pair)
        same_pair = g is not None and g.get("pair") == pair
        if same_pair and g["size"] == (W, H) and g["fps"] == fps and abs(g["ffs_scale"] - ffs_scale) < 1e-9:
            return g
        if same_pair and g["size"] == (W, H):
            ng = dict(g, fps=fps, ffs_scale=ffs_scale)
        else:
            phys = pair_physical(pair)
            calib = self.calib_for(pair)
            ng = calib_scale.load_geometry(str(calib), phys, (W, H))
            ng.update(fps=fps, ffs_scale=ffs_scale, pair=pair, physical=phys, calib=str(calib),
                      calib_src=f"{ng['st'].get('pose_source', '-')} ({ng['st'].get('scale_status', '-')})")
            self.epi = None                                   # 새 크기·쌍으로 정류 검증 다시 (기준 dy 도 크기 배율)
            if self.aruco is not None:
                self.aruco.reset()                            # f·기선이 바뀜 (맵 리셋 때는 안 지움 — 열 상태는 이어짐)
            self.a.physical = list(phys)
            self.link.physical = ",".join(phys)              # 앱 --es physical (다음 앱 시작부터)
        self.st, self.swap, self.rp = ng["st"], ng["swap"], ng["rp"]
        self.geom = ng
        self.a.size = [W, H]
        self.link.width, self.link.height, self.link.fps = W, H, fps
        rp = ng["rp"]
        self.log(f"[calib] 렌즈 {pair} · {W}x{H} {fps} fps · {ng['note']} · f={rp['f']:.1f} cx={rp['cx']:.1f} cy={rp['cy']:.1f} "
                 f"baseline={rp['baseline_m'] * 1000:.2f} mm swap={ng['swap']} · {Path(ng['calib']).name} {ng['calib_src']} · FFS 입력 "
                 f"{int(round(W * ffs_scale))}x{int(round(H * ffs_scale))} (scale {ffs_scale:g})")
        if self.ds is not None:
            self.ds.set_geometry(rp, ffs_scale)
        return ng

    @staticmethod
    def _cfg_resolution(cfg):
        """OpenVINS 설정 폴더(estimator_config.yaml 과 같은 폴더의 yaml)에서 cam0 resolution [W, H]. 못 찾으면 None."""
        cfg = Path(cfg)
        for p in [cfg] + sorted(cfg.parent.glob("*.yaml")):
            try:
                m = re.search(r"resolution:\s*\[\s*(\d+)\s*,\s*(\d+)\s*\]", p.read_text(encoding="utf-8", errors="replace"))
            except OSError:
                continue
            if m:
                return int(m.group(1)), int(m.group(2))
        return None

    def vio_fixed_size(self, replay=False):
        """VIO 가 크기를 따라가지 못하는 경우의 고정 크기 (REVIEW6 R1). --ov-config(-mono) = 그 설정의 resolution,
        실시간 외부 브리지(--ov HOST:PORT·pipe:) = 시작 때 크기 (--app-size). 크기를 따라가면 None."""
        a = self.a
        if a.ov == "none":
            return None
        fixed = a.ov_config_mono if a.vio_mode == "mono" else a.ov_config
        if fixed:
            return self._cfg_resolution(fixed)
        if not replay and a.ov != "auto":
            return tuple(int(x) for x in self.boot_size)
        return None

    def _check_vio_size(self, size, replay=False):
        fs = self.vio_fixed_size(replay)
        W, H = calib_scale.parse_size(size)
        if fs is not None and tuple(fs) != (W, H):
            why = "--ov-config 설정" if (self.a.ov_config or self.a.ov_config_mono) else f"외부 브리지 (--ov {self.a.ov})"
            raise ValueError(f"위치 추적 설정이 {fs[0]}x{fs[1]} 로 고정돼 있어 {W}x{H} 를 쓸 수 없습니다 ({why}). "
                             f"{fs[0]}x{fs[1]} 를 고르거나, 고정 설정 없이 스캐너를 켜세요.")

    def _vio_key(self):
        """브리지 설정을 가르는 값: 크기·fps·VIO 모드. --ov auto 일 때만 (따로 띄운 브리지는 그쪽 설정)."""
        if self.a.ov != "auto":
            return None
        g = self.geom
        return (tuple(g["size"]), int(g["fps"]), self.a.vio_mode)

    def stream_info(self):
        """웹·저장용: 지금 세션 해상도·fps·보정 변환 정보, 다음 실시간·녹화 세션에 쓸 값(want)."""
        g = self.geom
        W, H = g["size"]
        w = self.want
        return dict(size=[W, H], fps=g["fps"], aspect="4:3", ffs_scale=g["ffs_scale"],
                    ffs_size=[int(round(W * g["ffs_scale"])), int(round(H * g["ffs_scale"]))],
                    ffs_auto=self.a.scale is None, ffs_max_px=self.a.ffs_max_px,
                    calib_size=list(g["base_size"]), calib_scaled=bool(g["scaled"]), calib_scale=g["scale"],
                    calib_note=g["note"], rect=dict(f=g["rp"]["f"], cx=g["rp"]["cx"], cy=g["rp"]["cy"]),
                    epi_gate_px=round(1.0 * g["scale"], 3),
                    want=dict(size=list(w["size"]), fps=w["fps"], ffs_scale=w["ffs_scale"], pair=w.get("pair")),
                    rec3=self.rec3, app_physical=str(self.link.physical),
                    pair=g.get("pair"), physical=list(g.get("physical") or self.a.physical),
                    baseline_mm=round(g["rp"]["baseline_m"] * 1000, 2), calib_file=Path(g.get("calib") or "").name,
                    calib_src=g.get("calib_src"), pairs=list(LENS_PAIRS),
                    stream_fmt=self.link.stream_fmt, live_kbps=self.link.live_kbps,
                    vio_config=str(self.vio_cfg_used) if self.vio_cfg_used else None)

    @property
    def stream(self):
        """web_server.stream() 용: w,h,fps = 폰 세션 중이면 지금 값, 아니면 다음 세션 값(want) + stream_info()."""
        info = self.stream_info()
        if self.mode in PHONE_MODES:
            W, H, fps = info["size"][0], info["size"][1], info["fps"]
        else:
            (W, H), fps = self.want["size"], self.want["fps"]
        return dict(info, w=int(W), h=int(H), fps_now=info["fps"], fps=int(fps))

    def _default_live_kbps(self, size, fps):
        """새 크기·fps 의 실시간 H.264 기본 비트레이트 (렌즈 하나, kbps; load_model.live_kbps = 웹 부하 모델과 같은 식).
        부하 모델이 없으면 None (앱이 크기·fps 비례로 정함)."""
        lm = getattr(getattr(getattr(self, "http", None), "helper", None), "lm", None)
        if lm is None:
            return None
        try:
            import load_model
            return int(load_model.live_kbps(lm.coeffs, lm.caps, int(size[0]), int(size[1]), int(fps)))
        except Exception:
            return None

    def set_stream(self, size=None, h=None, fps=None, rec_kbps=None, ffs_scale=None, keep_ffs=False,
                   stream_fmt=None, live_kbps=None, live_kbps_auto=False, pair=None):
        """다음 실시간·녹화 세션의 크기·fps 를 정한다 (검증만, 적용은 start_session). 반환 want.
        size = "WxH" | (W,H) | W (그때 h 도 줌, web_server 의 set_stream(w, h, fps, rec_kbps=…) 호출).
        rec_kbps = 녹화 H.264 비트레이트 (앱 --ei rec_kbps, None = 앱 기본).
        stream_fmt = 실시간 형식 h264|jpeg|nv21 (앱 --es stream_fmt), live_kbps = 렌즈당 실시간 H.264 kbps
        (앱 --ei live_kbps, None/0 = 앱이 크기·fps 비례로 정함). 둘 다 다음 세션(앱 시작)부터.
        live_kbps_auto = True 면 live_kbps 는 부하 모델이 계산한 기본값 (웹 [스캔 시작]), False 면 사용자가 직접 준 값.
        크기·fps 만 바뀌고 live_kbps 가 없으면: 사용자가 직접 준 값만 유지하고, 아니면 새 크기 기준 기본값으로 다시 계산
        (REVIEW7 H4: 1280x960 의 16 Mbps 가 640x480 에 남지 않게).
        pair = 깊이 렌즈 쌍 '2+5' | '2+6' (None = 지난 값). 보정 파일이 없으면 ValueError."""
        pair = norm_pair(pair) or self.want.get("pair") or self.a.pair
        self.calib_for(pair)                              # 보정 파일 확인 (없으면 세션을 멈추기 전에 ValueError)
        if stream_fmt is not None and stream_fmt not in ("h264", "jpeg", "nv21"):
            raise ValueError(f"stream_fmt {stream_fmt}")
        if h is not None and size is not None and not isinstance(size, (str, tuple, list)):
            size = (int(size), int(h))
        w = self.want
        old_geo = (tuple(w["size"]), int(w["fps"]))
        sz, f, fs = self._norm_geometry(size if size is not None else w["size"], fps if fps is not None else w["fps"],
                                        ffs_scale if ffs_scale is not None else (w["ffs_scale"] if keep_ffs else None))
        self.want = dict(size=sz, fps=f, ffs_scale=fs, pair=pair)
        if rec_kbps is not None:
            self.link.rec_kbps = max(500, int(rec_kbps))
        if stream_fmt is not None:
            self.link.stream_fmt = self.a.app_stream = stream_fmt
        h264 = (stream_fmt or self.link.stream_fmt) == "h264"
        user = getattr(self, "live_kbps_user", False)
        if live_kbps and h264:
            self.link.live_kbps = int(live_kbps)
            self.live_kbps_user = not live_kbps_auto
        elif not h264:
            self.link.live_kbps, self.live_kbps_user = None, False
        elif stream_fmt is not None or live_kbps is not None or (tuple(sz), int(f)) != old_geo:
            # h264 인데 값이 안 옴: 사용자가 직접 준 값은 유지, 아니면 새 크기·fps 기준 기본값
            if not (user and self.link.live_kbps):
                self.link.live_kbps, self.live_kbps_user = self._default_live_kbps(sz, f), False
        return self.want

    # ---------------- 시작 ----------------
    def start(self):
        self.http = web_server.serve(self, self.a.host, self.a.port)
        self.log(f"[web] http://localhost:{self.a.port}  (WSL {self.a.host}:{self.a.port})")
        # 깊이 스레드(FFS·YOLO, GPU)는 여기서 안 띄운다 — 첫 live/both/replay 세션 때 _ensure_depth (대기 중 GPU 안 씀)
        threading.Thread(target=self._vio_feeder, daemon=True, name="vio-feed").start()
        threading.Thread(target=self._vio_watchdog, daemon=True, name="vio-watchdog").start()
        if self.a.replay:
            self.start_replay(self.a.replay, self.a.every_frame, self.a.replay_iters, self.a.replay_out, pair=self.a.replay_pair)
        elif self.a.mode != "idle":
            self.start_session(self.a.mode, start_app=self.a.start_app)
        if not self.a.replay:                             # 명령줄 한 번 재생(replay.py)에서는 자동 대기열을 안 띄움
            self.recsync.start()

    def _ensure_depth(self):
        """FFS·YOLO 로드 + 깊이 스레드: 처음 필요할 때 한 번만 (그 뒤로는 계속 둔다). 웹은 dstats.loading 으로
        "깊이 모델을 불러오는 중…" 을 보여 준다."""
        with self.depth_lock:
            if self.depth_thread is not None and self.depth_thread.is_alive():
                return
            if self.dstats.get("ready"):
                return
            self.dstats.update(loading=True, loaded=True, error=None, load_t0=time.time(), load_s=None,
                               load_stage="모델 파일 읽는 중")
            self.log("[depth] 깊이 모델 불러오기 시작 (첫 스캔·복원 때 한 번)")
            self.depth_thread = threading.Thread(target=self._depth_worker, daemon=True, name="depth")
            self.depth_thread.start()

    # ---------------- 세션 (모드) ----------------
    def start_session(self, mode, start_app=True, size=None, fps=None, ffs_scale=None, pair=None, rec3=False):
        """size(WxH 4:3)·fps·ffs_scale 를 주면 그 값으로 (self.want 갱신), 안 주면 self.want. 크기·fps 가 바뀌면
        정류맵·FFS 입력·앱 인자를 다시 정하고, 실시간 브리지는 그 크기·fps 설정(config/<W>x<H>_<fps>/)으로 다시 띄운다.
        rec3 = 렌즈 3개(2,5,6) 모두 녹화 (비교용, '폰에 녹화만' 에서만). pair 는 그때 쓰지 않는 깊이 쌍 (기본 2+6)."""
        if mode not in PHONE_MODES:
            raise ValueError(f"mode {mode}")
        if rec3 and mode != "record":
            raise ValueError("렌즈 3개 모두 녹화는 '폰에 녹화만' 에서만 됩니다 — 실시간 깊이는 초광각+광각 또는 초광각+망원 중 하나를 고르세요")
        if size is not None or fps is not None or ffs_scale is not None or pair is not None:
            self.set_stream(size=size, fps=fps, ffs_scale=ffs_scale, keep_ffs=size is None, pair=pair)   # 잘못된 값이면 세션을 멈추기 전에 ValueError
        if mode in ("live", "both") and self.a.vio_mode == "stereo" and self.want.get("pair") != "2+5":
            raise ValueError("두 렌즈 위치 추적(--vio-mode stereo)은 렌즈 2+5 설정만 있습니다 — 렌즈 쌍을 2+5 로 고르세요")
        if mode in ("live", "both"):
            self._check_vio_size(self.want["size"])      # 고정 VIO 설정·외부 브리지와 크기가 다르면 거절 (REVIEW6 R1)
            self._ensure_depth()
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
            w = self.want
            self.apply_geometry(w["size"], w["fps"], w["ffs_scale"], w.get("pair"))
            # 앱 --es physical: 3개 모두 녹화면 2,5,6, 아니면 깊이 쌍 (apply_geometry 가 쌍이 같으면 안 건드리므로 늘 다시 정함)
            self.rec3 = bool(rec3)
            self.link.physical = ",".join(ALL_REC_PHYSICAL if self.rec3 else self.geom["physical"])
            if self.rec3:
                self.log(f"[link] 렌즈 3개 모두 녹화 (비교용): 앱 --es physical {self.link.physical}")
            self.mode = mode
            self.replay_finished = False
            stop = threading.Event()
            self.session_stop = stop
            self.rec_status = None
            threading.Thread(target=self._session_main, args=(mode, start_app, stop), daemon=True,
                             name=f"session-{mode}").start()
        return dict(ok=True, mode=mode)

    def stop_session(self, stop_app=True):
        with self.session_lock:
            was = self.mode
            rec = (self.rec_status or {}).get("rec") or (self.app_status or {}).get("rec")
            if was == "replay":
                self.recsync.on_user_stop()               # 사용자가 멈춘 자동 복원은 [다시 시도] 전까지 다시 안 함
            self._stop_session_locked(stop_app=stop_app)
            stopper = self.app_stop_thread
        pull = None
        if was in ("record", "both"):
            # [정지] → (앱 정지 = mp4 마무리 끝나면) 폰 녹화를 PC 로 가져오고 → 자동 복원 대기열 (rec_sync)
            pull = self.recsync.after_record_stop(rec, stopper if stop_app else None)
        return dict(ok=True, was=was, rec=rec, pull=pull)

    def _stop_session_locked(self, stop_app):
        was = self.mode
        if self.session_stop is not None:
            self.session_stop.set()
            self.session_stop = None
        rx, self.receiver = self.receiver, None
        if rx:
            try:
                rx.close()
            except Exception:
                pass
        if self.replayer is not None:
            rp = self.replayer
            rp.stop()
            rp.join(20.0)                                 # end_replay 는 lock 을 안 잡으므로 여기서 기다려도 됨
            self.replayer = None
        self.mode = "idle"
        if stop_app and was in PHONE_MODES and self.link.available and self.link.app_mode:
            self.app_status = dict(state="정지 중")

            def _stop():
                self.link.stop_app()
                self.app_status = dict(state="정지됨")
            t = threading.Thread(target=_stop, daemon=True, name="app-stop")
            self.app_stop_thread = t
            t.start()

    def _join_app_stop(self, timeout=20.0):
        """REVIEW2 1: '정지' 직후 '시작' → 정지 스레드(broadcast·mp4 마무리 대기)가 끝난 뒤에 앱을 다시 띄운다."""
        t = self.app_stop_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            self.log("[link] 앱 정지(녹화 마무리)가 끝나기를 기다림")
            t.join(timeout)
            if t.is_alive():
                self.log(f"[link] 앱 정지 스레드가 {timeout:.0f} s 안에 안 끝남 (start_app 이 lock 으로 다시 기다림)")

    def _session_main(self, mode, start_app, stop):
        try:
            if start_app:
                if self.link.available:
                    self.app_status = dict(state="시작 중", mode=mode)
                    self._join_app_stop()
                    if stop.is_set():
                        return
                    ok, st = self.link.start_app(mode)     # 녹화 중이면 start_app 이 먼저 정상 정지 (mp4 마무리)
                    self.app_status = dict(phone_link.parse_status(st), ok=ok, mode=mode)
                else:
                    self.log("[link] adb 없음 — 앱은 Windows win_relay.py (--app-mode) 로 시작할 것")
            if stop.is_set():
                return
            if mode in ("record", "both"):
                threading.Thread(target=self._status_loop, args=(stop,), daemon=True, name="rec-status").start()
            if mode in ("live", "both"):
                self.want_iters = self.a.iters
                if not self._ensure_live_vio():
                    # B9: 브리지 시작 실패여도 스트림은 받는다 (카메라 창·폰 연결 확인). 브리지는 watchdog 이 간격을 늘리며 다시 시도
                    self.log("[ov] 브리지 없이 폰 스트림만 받습니다 — 브리지는 자동으로 다시 시도")
                self._reset_pipeline(vio_reset=True)
                self.timing_src = None
                self._conn_loop(stop)
        except Exception as e:
            self.log(f"[session] 오류: {e!r}")

    # ---------------- OpenVINS 브리지 (자식 프로세스) ----------------
    def _vio_config(self):
        """--vio-mode·세션 크기·fps 에 맞는 estimator_config.yaml. --ov-config(-mono) 를 주면 그 파일 고정,
        아니면 make_ov_config.ensure_config → config/<W>x<H>_<fps>/ (mono 는 그 아래 mono/), 없거나 낡으면 만든다."""
        mono = self.a.vio_mode == "mono"
        fixed = self.a.ov_config_mono if mono else self.a.ov_config
        if fixed:
            return Path(fixed)
        g = self.geom
        return Path(make_ov_config.ensure_config(g["size"], g["fps"], mono=mono, calib=self.a.calib, log=self.log))

    def _bridge_cmd(self, extra_args, tag):
        """pipe: 브리지 명령과 stderr(OpenVINS 로그) 파일 경로."""
        bridge, cfg = Path(self.a.ov_bridge), self._vio_config()
        if not bridge.is_file():
            raise FileNotFoundError(f"브리지 실행 파일 없음: {bridge} (bash scan3d/setup.sh bridge, 또는 --ov none)")
        if not cfg.is_file():
            raise FileNotFoundError(f"OpenVINS 설정 없음: {cfg} (venv_ffs/bin/python scan3d/make_ov_config.py)")
        self.vio_cfg_used = cfg
        self.vio_cam0 = make_ov_config.vio_cam0_kind(cfg)   # 'lens0u' (mono 기본) | 'rect0' → VIO 입력 영상·포즈 변환
        cmd = f"pipe:{shlex.quote(str(bridge))} {shlex.quote(str(cfg))} --stdio {extra_args}".strip()
        Path(self.a.out).mkdir(parents=True, exist_ok=True)
        errlog = Path(self.a.out) / f"ov_bridge_{tag}_{time.strftime('%Y%m%d-%H%M%S')}.log"
        return cmd, str(errlog)

    def _make_live_vio(self):
        if self.a.ov == "none":
            return ov_client.NullVIO(self.on_pose, self.log)
        if self.a.ov == "auto":
            key = self._vio_key()
            cmd, errlog = self._bridge_cmd(self.a.live_bridge_args, "live")
            v = ov_client.OVClient(cmd, self.on_pose, self.log, on_event=self.on_vio_event, stderr_path=errlog)
            v.geom_key = key                               # 크기·fps·모드가 바뀌면 _ensure_live_vio 가 다시 띄운다
            self.log(f"[ov] 브리지 로그: {errlog} (모드 {self.a.vio_mode}, {key[0][0]}x{key[0][1]} {key[1]} fps)")
            return v
        # 따로 띄운 브리지: 그쪽 설정을 모르므로 이 모드의 기본(또는 --ov-config) 설정 표시를 따른다
        self.vio_cam0 = make_ov_config.vio_cam0_kind(self._vio_config())
        self.log(f"[ov] 외부 브리지 {self.a.ov}: VIO cam0 = {self.vio_cam0} 로 가정 (브리지도 같은 설정이어야 함)")
        return ov_client.OVClient(self.a.ov, self.on_pose, self.log, on_event=self.on_vio_event, connect_timeout_s=10.0)

    @staticmethod
    def _vio_bad(v):
        return v is None or bool(getattr(v, "error", None)) or bool(getattr(v, "dead", lambda: False)())

    def _ensure_live_vio(self):
        """실시간 브리지 확보. 새로 띄우는 일은 lock 밖, 교체는 session_lock 안에서 `is` 확인 뒤 (watchdog 과 겹쳐도
        브리지가 고아로 남지 않게, REVIEW4 B10)."""
        if self.a.ov == "none":
            with self.session_lock:
                if not isinstance(self.live_vio, ov_client.NullVIO):
                    self.live_vio = ov_client.NullVIO(self.on_pose, self.log)
                self.vio = self.live_vio
            return True
        old = self.live_vio
        stale = old is not None and self.a.ov == "auto" and getattr(old, "geom_key", None) != self._vio_key()
        if not self._vio_bad(old) and not stale:
            self.vio = old
            return True
        if stale and not self._vio_bad(old):
            self.log(f"[ov] 해상도·fps·모드 바뀜 {getattr(old, 'geom_key', None)} → {self._vio_key()}: 브리지 다시 띄움")
        try:
            nv = self._make_live_vio()
        except Exception as e:
            self.vio_error = f"브리지 시작 실패: {e!r}"
            self.bridge_fails += 1
            wait = self._bridge_backoff()
            self.bridge_next = time.monotonic() + wait
            self.log(f"[ov] 브리지 시작 실패: {e!r} — {wait:.0f} 초 뒤 다시 시도 (--ov none 이면 VIO 없이 깊이만)")
            with self.session_lock:
                if self.live_vio is old:
                    self.live_vio = None
                if self.mode in ("live", "both"):         # 재생 중이면 재생 브리지를 건드리지 않음
                    self.vio = IdleVIO()
            if old is not None:
                try:
                    old.close()
                except Exception:
                    pass
            return False
        drop = None
        with self.session_lock:
            if self.live_vio is old:
                self.live_vio, drop = nv, old
                self.bridge_t = time.monotonic()
            else:
                drop = nv                                  # 그 사이 watchdog 이 이미 바꿈 → 내 것은 닫는다
            if self.mode in ("live", "both"):
                self.vio = self.live_vio
            self.vio_error = None
        if drop is not None:
            try:
                drop.close()
            except Exception:
                pass
        return True

    def _bridge_backoff(self):
        """연속 실패 수 → 다시 띄우기까지 (s): restart_s, ×2, ×4 … 최대 30 s (REVIEW4 B11)."""
        return min(30.0, self.a.bridge_restart_s * (2 ** max(0, min(self.bridge_fails - 1, 4))))

    def _vio_watchdog(self):
        """실시간 브리지가 죽으면 (예: OpenVINS 예외로 종료) --bridge-restart-s 뒤 다시 띄우고 VIO 리셋 규칙을 탄다."""
        dead_since, wait = None, self.a.bridge_restart_s
        while not self.stopped:
            time.sleep(0.5)
            v = self.live_vio
            live = self.mode in ("live", "both") and self.session_stop is not None
            if v is None and live and self.a.ov != "none":
                # B9: 브리지 "시작" 자체가 실패한 세션 → 간격을 늘리며 다시 시도 (_ensure_live_vio 가 lock 아래에서 교체).
                # bridge_fails > 0 = 실패가 있었을 때만 (세션 시작 중의 첫 시도와 겹쳐 브리지를 두 번 띄우지 않게)
                if self.bridge_fails > 0 and time.monotonic() >= self.bridge_next and self._ensure_live_vio():
                    self.log("[ov] 브리지 시작됨 (다시 시도 성공)")
                    self.on_vio_event({"type": "reset", "auto": True, "reason": "bridge_restart"})
                continue
            if not isinstance(v, ov_client.OVClient) or not live or not v.dead():
                dead_since = None
                if isinstance(v, ov_client.OVClient) and not v.dead() and self.bridge_fails \
                        and time.monotonic() - self.bridge_t > 30.0:
                    self.bridge_fails = 0                  # 30 s 넘게 살아 있음 → 연속 실패 수 초기화
                continue
            now = time.monotonic()
            if dead_since is None:
                dead_since = now
                quick = now - self.bridge_t < 30.0         # 띄운 지 30 s 안에 죽음 → 연속 실패 (B11)
                self.bridge_fails = self.bridge_fails + 1 if quick else 1
                wait = self._bridge_backoff()
                code = v.proc.poll() if v.proc is not None else None
                self.log(f"[ov] 브리지 끊김 ({v.error or '프로세스 종료'}, 종료 코드 {code}) "
                         f"→ {wait:.0f} 초 뒤 다시 띄움" + (f" (연속 {self.bridge_fails}번째)" if self.bridge_fails > 1 else ""))
                continue
            if now - dead_since < wait:
                continue
            try:
                v.close()
            except Exception:
                pass
            try:
                nv = self._make_live_vio()                 # 연결/프로세스 시작은 lock 밖에서
            except Exception as e:
                self.vio_error = f"브리지 다시 띄우기 실패: {e!r}"
                self.bridge_fails += 1
                wait = self._bridge_backoff()
                self.log(f"[ov] 브리지 다시 띄우기 실패: {e!r} — {wait:.0f} 초 뒤 다시 시도")
                dead_since = time.monotonic()
                continue
            with self.session_lock:
                if self.live_vio is not v or self.stopped:
                    nv.close()
                    dead_since = None
                    continue
                self.live_vio = nv
                if self.vio is v:
                    self.vio = nv
                self.bridge_restarts += 1
                self.bridge_t = time.monotonic()
                self.vio_error = None
            dead_since = None
            self.log(f"[ov] 브리지 다시 띄움 #{self.bridge_restarts}")
            # 새 브리지 = 새 월드 원점 → auto-reset 과 같은 규칙 (궤적·키프레임 기준 초기화, 맵은 --on-vio-reset)
            self.on_vio_event({"type": "reset", "auto": True, "reason": "bridge_restart"})

    def _status_loop(self, stop):
        """녹화 모드: status.txt 폴링 (PC 계산 없음)."""
        while not stop.is_set() and not self.stopped:
            if self.link.available:
                st = self.link.status()
                st["polled"] = time.strftime("%H:%M:%S")
                if self.link.app_started:
                    st["elapsed_s"] = round(time.time() - self.link.app_started, 1)
                self.rec_status = st
            stop.wait(1.0)

    def _conn_loop(self, stop):
        """폰 스트림 접속 유지: 끊기면 다시 접속, Ctrl+C·모드 전환 때 바로 끝남 (REVIEW 3, 11)."""
        quick_fails = 0
        native = self.a.link == "native"
        while not stop.is_set() and not self.stopped:
            try:
                if native and self.link.available:
                    self.link.forward(int(self.a.phone.rpartition(":")[2]), 8765)
                sock = phone_rx.open_phone_socket(self.a.phone, self.log, timeout_s=0, stop=stop)
            except Exception as e:
                self.log(f"[phone] 접속 오류: {e!r} — 2 s 뒤 다시")
                stop.wait(2.0)
                continue
            if sock is None:
                break
            rx = phone_rx.StreamReceiver(sock, self.on_sensor, self.on_pair, 2, tuple(self.geom["size"]),
                                         self.pair_skew_ms(), log=self.log)
            if (self.app_status or {}).get("ts_ref") == "mid":     # 앱 status 꼬리 ts_ref=mid (1.7-mid): stream_info 전에도
                rx.ts_ref = phone_rx.lps.TS_REF_MID
                rx.timing.set_mid()
            with self.session_lock:
                if stop.is_set():
                    rx.stopped = True
                    sock.close()
                    break
                self.receiver = rx
            rx.start()
            while not stop.is_set() and rx.error is None:
                stop.wait(0.5)
            if stop.is_set():
                break
            got = sum(rx.counts)
            self.log(f"[phone] 수신 끊김: {rx.error} (프레임 {got}) — 다시 접속")
            try:
                rx.close()
            except Exception:
                pass
            quick_fails = quick_fails + 1 if (got == 0 and time.monotonic() - rx.t_connect < 3.0) else 0
            # 앱을 이 프로세스가 시작했을 때만 (relay 에서 win_relay 가 시작했으면 그쪽 AppKeeper 가 맡음)
            if quick_fails >= 3 and self.a.restart_app and self.link.available and self.link.app_mode \
                    and self.mode in PHONE_MODES:
                raw = self.link.status_raw()
                if phone_link.is_recording(raw):
                    # REVIEW2 5: both 모드(RECORDING+STREAMING)에서 재시작하면 녹화가 끊김 → 로그만
                    self.log(f"[phone] 폰 쪽이 바로 끊김 반복이지만 녹화 중 ({phone_link.parse_status(raw)['raw']}) "
                             "→ 앱 재시작 안 함, 접속만 다시 시도")
                elif not stop.is_set():
                    self.log("[phone] 폰 쪽이 바로 끊김 반복 → 앱 다시 시작")
                    ok, st = self.link.start_app(self.mode)
                    self.app_status = dict(phone_link.parse_status(st), ok=ok, mode=self.mode)
                quick_fails = 0
            stop.wait(1.0 + min(quick_fails, 5))

    def _reset_pipeline(self, vio_reset=True):
        """새 세션·재생 시작: 시각 기준, 궤적, 맵, IMU 동기, 키프레임 대기 비움."""
        self.t0_ns = None
        self.last_cam_ns = -1
        with self.recent_lock:
            self.recent.clear()
        self.vio_in.clear()
        while True:
            try:
                self.kf_queue.get_nowait()
            except queue.Empty:
                break
        self.last_kf = None
        with self.traj_lock:
            self.traj, self.cur_T = [], None
        self.vmap.reset()
        self.objects = []
        self.imu.reset()
        self.vio_mono = self.vio_stereo = 0
        self.vio_skew_dropped = 0
        self.skew_ms.clear()
        self.dt_ms.clear()
        self.gyro_buf.clear()
        self.mrect.update(motion=0, fixed=0, skipped=0, last=None)
        self.mrect["recent"].clear()
        self.pair_hist.clear()
        self.vio_auto_resets = 0
        self._reloc_reset()
        # 지난 세션(또는 복원)의 카메라 창·깊이 그림·물체 상자를 새 세션에 남기지 않는다 (REVIEW5 N3)
        self.last_pair = None
        with self.panel_lock:
            self.panel = {}
        with self.pose_cv:
            self.pose_seen_ns = -1
        self._replay_kf = None
        self.replay_tl, self.replay_depth = [], {}
        self._best_seg = None
        if self.ds is not None and self.ds.yolo:
            self.ds.yolo.reset()
        if vio_reset:
            self.vio.reset()
        self._map_begin()                                 # 저장 지도 기준 세션이면 지도·기준 키프레임 불러오기

    # ---------------- 녹화 재생 ----------------
    def resolve_rec(self, name):
        p = Path(name)
        if not p.is_absolute() and not p.is_dir():
            p = Path(self.a.rec_root) / name
        return p

    def start_replay(self, name, every_frame=False, iters=None, out=None, auto=False, pair=None):
        """auto=True: 자동 복원 대기열 (rec_sync) — 중간에 멈추면(실시간 시작 등) 저장하지 않고 나중에 다시.
        pair = 깊이 렌즈 쌍 '2+5'|'2+6' (3렌즈 녹화에서 고름, 없으면 2+6). 2렌즈 녹화는 녹화한 쌍만 (다르면 ValueError).
        3렌즈 녹화 결과 폴더 = recon_<시각>_p26 / _p25."""
        import replay
        path = self.resolve_rec(name)
        if not path.is_dir():
            raise FileNotFoundError(f"녹화 폴더 없음: {path}")
        meta = {}
        try:
            meta = json.loads((path / "meta.json").read_text())
        except (OSError, ValueError):
            pass
        # 녹화 크기·fps (meta.json width/height/fps, 없으면 recording.py 와 같은 기본 640x480) → 정류맵·VIO 설정
        rsize = (int(meta.get("width", 640)), int(meta.get("height", 480)))
        rfps = meta.get("fps") or self.a.app_fps
        self._norm_geometry(rsize, rfps, None)            # 4:3 아니면 세션을 멈추기 전에 ValueError
        # 녹화 렌즈 쌍 (meta.json physical "2,6" 또는 ["2","6"], 없으면 옛 녹화 = 2+5) → 그 쌍의 보정으로 복원
        mphys = meta.get("physical") or ["2", "5"]
        if isinstance(mphys, str):
            mphys = [x.strip() for x in mphys.split(",") if x.strip()]
        mphys = [str(x) for x in mphys]
        multi = len(mphys) > 2
        want = norm_pair(pair)
        if multi:
            rpair = want or ("2+6" if "6" in mphys else None)
            if rpair is None or not all(x in mphys for x in pair_physical(rpair)):
                raise ValueError(f"녹화 렌즈 {'+'.join(mphys)} 에 렌즈 쌍 {rpair} 가 없습니다")
            if out is None:
                out = path / (time.strftime("recon_%Y%m%d-%H%M%S") + replay_view.PAIR_SUFFIX[rpair])
        else:
            try:
                rpair = norm_pair(mphys[:2])
            except ValueError:
                raise ValueError(f"녹화 렌즈 {mphys} 는 복원할 수 없습니다 (렌즈 쌍 {' / '.join(LENS_PAIRS)} 만)")
            if want and want != rpair:
                raise ValueError(f"이 녹화는 렌즈 {rpair} 만 있습니다 (렌즈 {want} 로 복원할 수 없음)")
        self.calib_for(rpair)                             # 보정 파일 없으면 세션을 멈추기 전에 ValueError
        self._check_vio_size(rsize, replay=True)          # 고정 VIO 설정이 다른 크기용이면 거절 (REVIEW6 R1)
        if self.dstats.get("ready") and not str(self.dstats.get("error") or "").startswith("init:"):
            self.dstats["error"] = None                   # 지난 실시간 키프레임 오류가 복원을 바로 끝내지 않게 (REVIEW5 N6)
        self._ensure_depth()                              # 복원은 깊이 모델이 준비되기를 기다린다 (replay.py)
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
            # 재생은 self.want(실시간 세션 값)를 바꾸지 않는다. FFS 축소는 --scale 또는 자동
            self.apply_geometry(rsize, rfps, self.a.scale, rpair)
            self.replay_finished = False
            self.replayer = replay.Replayer(self, path, every_frame=bool(every_frame),
                                            iters=int(iters or self.a.replay_iters), out_dir=out,
                                            save_partial=not auto, auto=auto,
                                            pair=pair_physical(rpair) if multi else None)
            self.mode = "replay"
            self.replayer.start()
        return dict(ok=True, dir=str(path), pair=rpair, out=str(self.replayer.out_dir))

    def make_replay_vio(self):
        """재생 전용 브리지 (pipe, 새 프로세스 = 깨끗한 상태·시각 기준)."""
        if self.a.ov == "none":
            return ov_client.NullVIO(self.on_pose, self.log)
        cmd, errlog = self._bridge_cmd(self.a.replay_bridge_args, "replay")
        return ov_client.OVClient(cmd, self.on_pose, self.log, max_inflight=1 << 30, max_pending_cams=1 << 30,
                                  on_event=self.on_vio_event, max_cam_wait_s=1e9, stderr_path=errlog)

    def begin_replay(self, vio, every_frame, iters):
        self.vio = vio
        self.replay_every = every_frame
        self.want_iters = iters
        self.timing_src = None
        self._reset_pipeline(vio_reset=False)

    def end_replay(self, rp, vio):
        """재생 스레드 끝에서 호출 (session_lock 을 잡지 않는다 — 정지 쪽이 lock 을 쥔 채 join 함)."""
        try:
            if vio is not None:
                vio.close()
        except Exception:
            pass
        if self.vio is vio:
            self.vio = IdleVIO()
        if self.replayer is rp and self.mode == "replay":
            self.mode = "idle"
        self.replay_finished = True

    def wait_pose(self, t_ns, timeout):
        t_end = time.monotonic() + timeout
        with self.pose_cv:
            while self.pose_seen_ns < t_ns:
                left = t_end - time.monotonic()
                if left <= 0:
                    return False
                self.pose_cv.wait(left)
        return True

    def mark_pose_seen(self, t_ns):
        """포즈 없이 그 시각까지 본 것으로 (replay.PoseReplayVIO: 저장한 포즈에 없는 프레임)."""
        with self.pose_cv:
            if t_ns > self.pose_seen_ns:
                self.pose_seen_ns = int(t_ns)
            self.pose_cv.notify_all()

    def take_replay_kf(self):
        kf, self._replay_kf = self._replay_kf, None
        return kf

    def queue_idle(self):
        return self.kf_queue.qsize() == 0 and not self.worker_busy

    # ---------------- 수신 ----------------
    def on_sensor(self, s):
        self.imu.add(s)

    def on_imu(self, t_ns, w, acc):
        if self.t0_ns is None:
            self.t0_ns = t_ns
            self.log("[imu] 첫 IMU 샘플")
        self.gyro_buf.append((t_ns, w))                  # 움직임 보정 정류 (두 렌즈 촬영 순간 사이 자이로 적분)
        self.vio.push_imu(t_ns, w, acc)                  # 브리지가 t0 를 빼므로 원래 ns 그대로

    def on_pair(self, pair):
        if self.swap:
            pair = pair[::-1]
        self.vio_in.append(pair)
        self.vio_evt.set()

    def _vio_feeder(self):
        """프레임 쌍 → 정류 gray → VIO. 쌍은 키프레임용으로 잠시 보관."""
        while not self.stopped:
            self.vio_evt.wait(0.5)
            self.vio_evt.clear()
            while self.vio_in:
                pair = self.vio_in.popleft()
                try:
                    self._feed_pair(pair)
                except Exception as e:
                    self.log(f"[vio-feed] 오류: {e!r}")

    def _feed_pair(self, pair, timing=None):
        """반환: VIO 에 보낸 카메라 t_ns 또는 None."""
        fL, fR = pair
        g = self.geom
        rp = g["rp"]
        if (fL.bgr.shape[1], fL.bgr.shape[0]) != tuple(rp["size"]) or fL.bgr.shape != fR.bgr.shape:
            return None                                   # 해상도 바뀌기 전 프레임 (정류맵과 크기가 다름)
        self.last_pair = (fL, fR)                        # 카메라 창 (/api/cam.jpg) — 재생 중에도
        if self.t0_ns is None:
            if isinstance(self.vio, ov_client.NullVIO):
                self.t0_ns = fL.timestamp_ns
            else:
                return None                                       # IMU 가 먼저 와야 시각 기준이 생김
        # 두 렌즈 시각차: 통계는 항상, stereo 모드에서 기준을 넘는 쌍은 VIO 에 안 보낸다 (mono 로 섞어 보내지 않음:
        # 스테레오 설정에 mono 프레임이 섞이면 OpenVINS TrackKLT 가 "levels2 >= 0" 단언으로 죽는다)
        if timing is None:
            timing = self.timing_src or (self.receiver.timing if self.receiver else None)
        # 앱 1.7-mid·새 녹화·바꾼 옛 녹화는 프레임 시각이 이미 노출 가운데 → off = 0 (FrameTimingIndex.mid_streams, SPEC 1.2).
        # 옛 앱 스트림만 그 프레임 노출 값으로 0.5·(노출 + skew) 를 더한다 (REVIEW 6)
        # 노출 중앙 보정은 프레임 간격 안으로 제한: 엉뚱한 frame_timing 값(최신값 대체 등)이 카메라 시각을
        # 실제보다 한참 뒤로 밀면 IMU 를 기다리는 시간이 늘고 dt_cam_imu 추정도 흔들린다
        cap = 1e9 / max(1, g["fps"])
        off = int(np.clip(timing.offset_ns(fL.index, fL.timestamp_ns), 0, cap)) if timing is not None else 0
        offR = int(np.clip(timing.offset_ns(fR.index, fR.timestamp_ns), 0, cap)) if timing is not None else 0
        t_ns = int(fL.timestamp_ns + off)                # 왼쪽 렌즈 노출 가운데 시각
        dt_ns = int(fR.timestamp_ns + offR) - t_ns       # 쌍의 dt = t(lens1) − t(lens0) (노출 가운데끼리) → 움직임 보정 정류
        skew_ns = abs(dt_ns)
        self.skew_ms.append(skew_ns * 1e-6)
        self.dt_ms.append(dt_ns * 1e-6)
        stereo = self.a.vio_mode == "stereo"
        if stereo and skew_ns > self.a.vio_stereo_skew_ms * 1e6:
            self.vio_skew_dropped += 1
            return None
        if t_ns <= self.last_cam_ns:
            return None
        self.last_cam_ns = t_ns
        if self.epi is None and not self.a.skip_epi:
            rl, rr = lr.rectify_pair(fL.bgr, fR.bgr, rp)
            epi = lr.epipolar_check(rl, rr)
            gate = 1.0 * g["scale"]                       # 640x480 기준 1 px × 크기 배율
            ok = epi.get("status") != "ok" or (epi["dy_med"] < gate and epi["inlier_frac"] >= 0.7)
            epi.update(size=list(g["size"]), gate_px=round(gate, 3), ok=bool(ok), calib_note=g["note"],
                       dy_med_640=None if epi.get("dy_med") is None else round(epi["dy_med"] / g["scale"], 4))
            self.epi = epi
            self.log(f"[epi] {epi.get('status')} {g['size'][0]}x{g['size'][1]} dy_med={epi.get('dy_med')} "
                     f"(기준 {gate:.2f} px) inlier={epi.get('inlier_frac')} · {g['note']}"
                     + ("" if ok else "  ← 정류 검증 실패: 보정/초점(focus_diopters 1.0) 확인"
                        + (" — 비례 변환한 보정이 이 크기에 안 맞을 수 있음 (폰이 화각을 잘랐는지)" if g["scaled"] else "")))
        if not stereo and self.vio_cam0 == frames.VIO_CAM0_LENS0U:
            # mono 설정 cam0 = lens0u (렌즈 2 왜곡만 편 넓은 화각, calib_scale.vio_lens0_K — make_ov_config 와 같은 K)
            g0 = np.ascontiguousarray(cv2.cvtColor(cv2.remap(fL.bgr, *g["vio"]["map"], interpolation=cv2.INTER_LINEAR),
                                                   cv2.COLOR_BGR2GRAY))
            g1 = None
        else:
            rl, rr = lr.rectify_pair(fL.bgr, fR.bgr, rp)
            g0 = np.ascontiguousarray(cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY))
            g1 = np.ascontiguousarray(cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)) if stereo else None
        if stereo:
            self.vio_stereo += 1
        else:
            self.vio_mono += 1
        with self.recent_lock:
            self.recent[t_ns] = (pair, dt_ns)
            while len(self.recent) > 90:
                self.recent.popitem(last=False)
        self.vio.push_cam(t_ns, [g0, g1] if stereo else [g0])
        return t_ns

    def _pair_for(self, t_ns):
        """(pair, dt_ns) 또는 None."""
        with self.recent_lock:
            return self.recent.get(int(t_ns))           # 브리지가 t_ns 를 그대로 돌려줌

    # ---------------- 포즈 → 키프레임 ----------------
    def on_pose(self, msg):
        pl = self.pose_log
        if pl is not None:
            pl.append(("p", msg))                        # 브리지가 준 그대로 (rect0 로 바꾸기 전)
        try:
            t_ns = int(msg.get("t_ns", -1))
        except (TypeError, ValueError):
            t_ns = -1
        try:
            # cam0 = lens0u (mono 기본) 이면 포즈·외부 파라미터를 rect0 기준으로 바꿔 둔다 (아래는 모두 rect0 전제)
            msg = frames.pose_cam0_to_rect0(msg, self.rp, self.vio_cam0)
            self._on_pose(msg, t_ns)
        finally:
            with self.pose_cv:
                if t_ns > self.pose_seen_ns:
                    self.pose_seen_ns = t_ns
                self.pose_cv.notify_all()

    def _want_kf(self, T, t):
        if self.last_kf is None:
            return True
        t_last, T_last = self.last_kf
        dp = float(np.linalg.norm(T[:3, 3] - T_last[:3, 3]))
        dr = frames.rot_deg(T[:3, :3], T_last[:3, :3])
        timed = self.a.kf_dt > 0 and t - t_last >= self.a.kf_dt
        return dp >= self.a.kf_trans or dr >= self.a.kf_rot or timed

    def _on_pose(self, msg, t_ns):
        if msg.get("static"):
            T = frames.T_WORLD_RECT0_STATIC
        else:
            T = frames.T_world_rect0(msg, self.st, self.rp)
        if T is None or t_ns < 0:
            return
        t = (t_ns - (self.t0_ns or t_ns)) * 1e-9          # 표시용 초 (첫 IMU 기준)
        self._last_pose_t = t
        T = self._reloc_test_apply(T, t)                  # --reloc-test-split (시험용 가짜 리셋)
        T = self._map_test_apply(T, t)                    # --map-test (지도 나누기 시험): 앞부분 포즈는 버림
        if T is None:
            return
        # 재위치: VIO 조각 좌표 → 전역. 아직 못 붙인 조각(_seg_T None)은 조각 좌표 그대로 따로 쌓는다
        with self._reloc_lock:
            seg_T, pend = self._seg_T, self._pend
        T_vio = T
        if seg_T is not None and not np.array_equal(seg_T, np.eye(4)):
            T = seg_T @ T_vio
        with self.traj_lock:
            self.traj.append((t, T))
            if len(self.traj) > 50000:
                self.traj = self.traj[::2]
            self.cur_T = T
        if msg.get("stale"):                              # 상태 시각이 영상 시각과 다름 (REVIEW 9)
            return
        if pend is None and self._map_fixed():
            return                                        # 지도 위치만 추정 (localize): 붙은 뒤에는 깊이·지도 갱신 안 함
        if self.mode == "replay":
            if not (self.replay_every or self._want_kf(T, t)):
                return
            got = self._pair_for(t_ns)
            if got is not None:
                self.last_kf = (t, T)
                mot = self._motion_of(msg, t_ns, got[1])
                self._replay_kf = (got[0], T, t, self.vmap.epoch, mot)
                if pend is not None:
                    self._reloc_pending_kf(got[0], T_vio, t, mot, pend)
            return
        if self.paused or self.worker_busy or self.kf_queue.qsize():
            return
        if not self._want_kf(T, t):
            return
        got = self._pair_for(t_ns)
        if got is None:
            return
        try:
            mot = self._motion_of(msg, t_ns, got[1])
            self.kf_queue.put_nowait((got[0], T, t, self.vmap.epoch, mot))
            self.last_kf = (t, T)
            if pend is not None:
                self._reloc_pending_kf(got[0], T_vio, t, mot, pend)
        except queue.Full:
            pass

    @staticmethod
    def _motion_of(msg, t_ns, dt_ns):
        """키프레임 움직임 보정 재료 (lens0 시각 t_ns 의 VIO 상태): 속도 v_G, R_GtoI, 자이로 바이어스, IMU→cam0 외부 파라미터.
        NullVIO(--ov none) 는 static=True (이동 0 으로 보고 회전만 자이로)."""
        mot = dict(t0_ns=int(t_ns), dt_ns=int(dt_ns), static=bool(msg.get("static")))
        try:
            if msg.get("v") is not None and msg.get("q_GtoI_jpl") is not None:
                v = np.asarray(msg["v"], float).reshape(3)
                R = frames.jpl_quat_to_R(msg["q_GtoI_jpl"])
                if np.isfinite(v).all() and np.isfinite(R).all():
                    mot.update(v_G=v, R_GtoI=R)
            for k, n in (("bg", 3), ("R_ItoC0", 9), ("p_IinC0", 3)):
                if msg.get(k) is not None:
                    x = np.asarray(msg[k], float).reshape(-1)
                    if x.size == n and np.isfinite(x).all():
                        mot[k] = x
        except (TypeError, ValueError):
            pass
        return mot

    def on_vio_event(self, msg):
        """브리지 auto-reset: 월드 원점이 바뀜 → 궤적·키프레임 기준 초기화, 맵은 --on-vio-reset (REVIEW 10)."""
        pl = self.pose_log
        if pl is not None and msg.get("type") == "reset":
            pl.append(("e", msg))
        if msg.get("type") != "reset" or not msg.get("auto"):
            return
        self.vio_auto_resets += 1
        reinit = msg.get("reinit")
        why = {"exception": f"VIO 내부 오류 ({str(msg.get('msg', ''))[:120]}) → 자동 재시작",
               "bridge_restart": "브리지 다시 띄움", "test_split": "시험용 가짜 리셋"}.get(msg.get("reason"), "추적 실패 → VIO 재시작")
        hint = "옛 상태로 바로 다시 시작, 멈추지 않아도 됨" if reinit == "state" else "폰을 잠시 멈추면 다시 초기화"
        self.log(f"[ov] auto-reset #{self.vio_auto_resets} ({why}, {hint})")
        self.vio_events.append(dict(t=round(float(getattr(self, "_last_pose_t", 0.0)), 3), kind="reset",
                                    reason=msg.get("reason"), reinit=reinit, gap=msg.get("gap"), wall=time.time()))
        if reinit == "state" and self._pend is not None and self.a.reloc == "on":
            # 상태 주입 재시작은 월드 좌표가 그대로 → 아직 못 붙인 조각을 새로 시작하지 않고 그대로 이어 쌓는다
            # (rec_20261016_175446 을 --lost-frames 6 으로 일부러 끊었을 때 1 s 마다 새 조각이 생겨 키프레임이 모이지 않았음)
            self.log(f"[reloc] 조각 {self._seg_n} 에 그대로 이어 쌓음 (좌표 그대로인 재시작)")
            return
        with self.traj_lock:
            traj_before = self.traj
            self.traj, self.cur_T = [], None
        self.last_kf = None
        if self.a.on_vio_reset == "clear" and (self.vmap.n or self._pend is not None):
            rp = self.replayer
            if self.mode == "replay" and rp is not None and rp.out_dir is not None and self.vmap.n:
                try:
                    self.save(Path(rp.out_dir) / f"segment_{self.vio_auto_resets:02d}", include_cfg=False)
                except Exception as e:
                    self.log(f"[save] 조각 저장 실패 {e!r}")
            if self.a.reloc == "on" and self._reloc_get() is not None:
                self._reloc_start_segment(traj_before)       # 옛 지도는 보관, 새 조각은 따로 → 붙으면 합침
                return
            if self.mode == "replay":
                # 복원: 리셋 전 조각이 지금까지 가장 크면 (맵·시간 정보·궤적·물체) 를 통째로 보관하고 새 맵으로 이어 감.
                # 끝날 때 restore_best_segment() 가 마지막 조각보다 크면 이것을 결과로 씀.
                best = self._best_seg
                if best is None or self.vmap.n > best["vmap"].n:
                    self._best_seg = dict(vmap=self.vmap, tl=self.replay_tl, depth=self.replay_depth,
                                          traj=traj_before, objects=self.objects, seg=self.vio_auto_resets)
                old_vm = self.vmap
                self.vmap = VoxelMap(self.a.voxel, self.a.min_hits, names=old_vm.names)
                self.vmap.epoch = old_vm.epoch + 1            # 리셋 전에 만든 키프레임은 새 맵에 안 들어가게 (REVIEW 8)
            else:
                self.vmap.reset()
            self.objects = []
            self.replay_tl, self.replay_depth = [], {}     # 리플레이 시간 정보는 새 맵(지금 조각)만

    def restore_best_segment(self, log=None):
        """복원 끝: 리셋 전 보관한 조각이 마지막 조각보다 크면 그것을 결과로 되돌림. 되돌렸으면 조각 번호, 아니면 None.
        재위치(--reloc on): 끝까지 못 붙인 조각이 있으면 보관해 둔 전역 지도(번호 0)·버린 조각·마지막 조각 중 가장 큰 것."""
        with self._reloc_lock:
            pend, self._pend = self._pend, None
            if pend is not None:
                self._seg_T = np.eye(4)
        best = self._best_seg
        self._best_seg = None
        if pend is not None and pend.get("base_map"):
            # 저장 지도 기준 세션이 끝까지 지도에 못 붙음: 결과는 이 세션 지도 (불러온 지도는 결과로 쓰지 않음),
            # 재위치 DB 의 지도 키프레임도 비움 (이 세션 결과를 지도로 저장할 때 다른 좌표계 키프레임이 섞이지 않게)
            ms = self._map_sess
            if ms is not None:
                ms.update(status="failed")
            if log:
                log(f"[map] 지도 '{pend['base_map']}' 에 끝까지 못 붙음 → 이 세션 지도만 결과로 (질의 {pend.get('n_query', 0)}번)")
            if self.reloc is not None:
                self.reloc.clear()
            pend = None
        if pend is not None:
            main = dict(pend["main"], seg=0)
            if log:
                log(f"[reloc] 조각 {pend['seg']} 은 끝까지 옛 지도에 못 붙음 (다시 넣으려던 키프레임 {len(pend['jobs'])}개)")
            if best is None or main["vmap"].n >= best["vmap"].n:
                best = main
        if best is None or best["vmap"].n <= self.vmap.n:
            return None
        if log:
            log(f"VIO 리셋으로 맵이 나뉨 → 가장 큰 조각 (리셋 #{best['seg']} 전, 복셀 {best['vmap'].n}) 을 결과로 씀 "
                f"(마지막 조각 복셀 {self.vmap.n})")
        best["vmap"].epoch = self.vmap.epoch + 1
        self.vmap = best["vmap"]
        self.replay_tl, self.replay_depth, self.objects = best["tl"], best["depth"], best["objects"]
        with self.traj_lock:
            self.traj = best["traj"]
        return best["seg"]

    # ---------------- 저장 지도 (map_store.py) ----------------
    def _map_begin(self):
        """세션 시작 (_reset_pipeline 끝): map_req 가 있으면 지도를 불러와 '보관한 전역 지도' 로 두고, 이 세션은 처음부터
        아직 못 붙은 조각 (재위치 _pend, base_map) 으로 따로 쌓는다. 재위치 DB 에 지도 기준 키프레임을 넣어 두면 첫 키프레임들의
        질의가 PnP + 두 키프레임 합의로 VIO → 지도 변환을 찾고, _reloc_on_aligned 가 이어 쌓기·위치만 추정을 한다."""
        self._map_sess = None
        self._map_test = None
        req, self.map_req = self.map_req, None             # 이번 세션만 (자동 복원 등 다음 세션에는 안 씀)
        if not req or not req.get("name"):
            return
        sess = dict(name=req["name"], mode=req.get("mode") or "append", status="loading", started_wall=time.time())
        self._map_sess = sess
        try:
            import map_store
            if self.a.reloc != "on":
                raise ValueError("재위치가 꺼져 있음 (--reloc on 필요)")
            rl = self._reloc_get()
            if rl is None:
                raise ValueError("재위치를 만들 수 없음 (VIO 없음)")
            m = map_store.load_map(req["name"], log=self.log)
            if not m["db"] or not int(m["db"]["n"]):
                raise ValueError("지도에 기준 키프레임이 없음 (위치를 찾을 수 없음)")
            rl.import_db(m["db"])
        except Exception as e:
            sess.update(status="error", error=str(e))
            self.log(f"[map] 지도 '{req['name']}' 를 쓸 수 없음: {e} → 지도 없이 새로 시작")
            return
        gvm = m["vmap"]
        gvm.names = self.vmap.names
        with self._reloc_lock:
            self._seg_n = 1
            self._epoch_hi = max(self._epoch_hi, self.vmap.epoch, gvm.epoch) + 1
            gvm.epoch = self._epoch_hi
            main = dict(vmap=gvm, tl=[], depth={}, traj=[], objects=[])
            self._seg_T = None
            self._pend = dict(main=main, jobs=[], seg=1, n_query=0, base_map=req["name"])
            nv = VoxelMap(self.a.voxel, self.a.min_hits, names=self.vmap.names)
            self._epoch_hi += 1
            nv.epoch = self._epoch_hi
            nv._done_t = set()
            self.vmap = nv
        rl.begin_segment(1)
        sess.update(status="waiting", loaded=dict(voxels=int(gvm.n), tsdf_blocks=0 if gvm.tsdf is None else int(gvm.tsdf.nb),
                                                  db=int(m["db"]["n"]), load_ms=m["load_ms"]),
                    source=m["meta"].get("source"))
        mt = self.a.map_test
        if mt:
            v = [float(x) for x in str(mt).split(",")]
            Tf = frames.make_T(reloc_T_yaw(np.radians(v[1])), v[2:5])
            self._map_test = dict(t=v[0], T=Tf, truth=frames.inv_T(Tf))
            self.log(f"[map] 시험: t<{v[0]} s 포즈는 버리고, 뒤 포즈에 yaw {v[1]}° 이동 {v[2:5]} 을 곱함 (정답 = 그 역)")
        how = "이어 쌓기" if sess["mode"] == "append" else "위치만 추정"
        self.log(f"[map] 지도 '{req['name']}' 기준 세션 ({how}): 기준 키프레임 {sess['loaded']['db']}개 — 찍은 곳을 비추면 붙습니다")

    MAP_VERIFY = dict(min_pts=300, near_m=0.30, inlier_m=0.08, min_frac=0.5, recent_s=3.0)

    def _map_verify(self, gvm, piece, T, t=None):
        """기하 확인: 조각 지도 복셀 중심 (관측 1 이상, 붙인 시각 t 전 recent_s 초 안에 본 것 — 조각이 길면 VIO 표류로 한 변환에
        다 안 맞으므로) 을 T 로 옮겨, 지도 복셀 중심이 near_m 안에 있는 점 (겹치는 곳) 중 inlier_m 안 비율.
        점이 min_pts 보다 적으면 판단 보류 (통과). 반환 (통과?, 숫자 dict)."""
        c = self.MAP_VERIFY
        try:
            from scipy.spatial import cKDTree
            with piece.lock:
                sel = np.arange(piece.n)
                if t is not None:
                    sel = sel[piece.last[:piece.n] >= float(t) - c["recent_s"]]
                n = len(sel)
                P = (voxel_map.unpack_keys(piece.pk[sel]) + 0.5) * piece.voxel_m
            if n < c["min_pts"]:
                return True, dict(n=int(n), skipped="few_points")
            P = P @ T[:3, :3].T + T[:3, 3]
            ref = gvm.points()[0]
            if len(ref) < c["min_pts"]:
                return True, dict(n=int(n), skipped="empty_map")
            d, _ = cKDTree(ref).query(P, distance_upper_bound=c["near_m"])
            near = np.isfinite(d)
            frac = float((d[near] < c["inlier_m"]).mean()) if near.any() else 0.0
            vf = dict(n=int(n), near=round(float(near.mean()), 3), inlier=round(frac, 3),
                      med_cm=round(float(np.median(d[near])) * 100, 2) if near.any() else None)
            return (near.mean() >= 0.2 and frac >= c["min_frac"]), vf
        except Exception as e:
            return True, dict(error=repr(e))

    def _map_test_apply(self, T, t):
        mt = self._map_test
        if mt is None:
            return T
        if t < mt["t"]:
            return None
        return mt["T"] @ T

    def _map_fixed(self):
        """위치만 추정 모드에서 지도에 붙은 뒤인가 (깊이·지도 갱신 안 함)."""
        ms = self._map_sess
        return ms is not None and ms.get("mode") == "localize" and ms.get("status") == "attached"

    def map_state(self):
        ms = self._map_sess
        return dict(req=self.map_req, save_next=self.map_save_next,
                    session=None if ms is None else {k: v for k, v in ms.items() if k != "source"})

    def save_map(self, name, source=None):
        """지금 지도 (전역; 아직 못 붙은 조각은 빼고) + 재위치 기준 키프레임 → scan3d/maps/<name>/."""
        import map_store
        with self._reloc_lock:
            pend = self._pend
            vm = pend["main"]["vmap"] if pend is not None else self.vmap
        if pend is not None:
            self.log("[map] 아직 못 붙은 조각이 있음 → 그 조각은 빼고 전역 지도만 저장")
        with self.traj_lock:
            tr = list(self.traj) if pend is None else list(pend["main"]["traj"])
        rp = self.replayer
        ms = self._map_sess
        src = dict(mode="replay" if rp is not None and self.mode in ("replay", "idle") else self.mode,
                   recording=rp.rec_dir.name if rp is not None else None, recon=self.last_result,
                   size=list(self.geom["size"]) if self.geom else None, pair=(self.geom or {}).get("pair"),
                   saved=time.strftime("%Y-%m-%d %H:%M:%S"))
        src.update(source or {})
        frame = dict(base_map=ms["name"] if ms is not None and ms.get("status") == "attached" else None)
        if ms is not None and ms.get("status") != "attached":
            frame["note"] = f"지도 '{ms['name']}' 에 못 붙은 세션 — 이 세션 자체 좌표"
        return map_store.save_map(name, vm, self.reloc, tr, source=src, frame=frame, log=self.log)

    def esdf_get(self, res=None):
        """지금 지도의 ESDF (esdf_map.for_vmap, 바뀐 것만 갱신)."""
        import esdf_map
        return esdf_map.for_vmap(self.vmap, float(res or esdf_map.RES))

    def _esdf_kf_done(self, vm, epoch, r, rp, T, R_fix):
        """키프레임 깊이 → 관측된 빈 칸 (ESDF 의 빈 곳/모름 구분, esdf_map.FreeSpace)."""
        try:
            Z = r["Z"]
            sx = Z.shape[1] / float(rp["size"][0])
            T_cam = np.array(T, np.float64)
            if R_fix is not None:
                T_cam[:3, :3] = T_cam[:3, :3] @ np.asarray(R_fix, np.float64)
            args = (Z, r["valid"], rp["f"] * sx, rp["cx"] * sx, rp["cy"] * sx, T_cam)
            if self.mode == "replay":
                vm.observe_free(*args, epoch=epoch)        # 복원: 바로 (저장 때 빠짐 없게)
                return
            # 실시간: 깊이 스레드를 붙잡지 않게 작업 스레드 하나에서 (밀리면 버림 — 빈 칸은 다음 키프레임이 또 채움)
            if getattr(self, "_esdf_pending", 0) >= 2:
                return
            if getattr(self, "_esdf_pool", None) is None:
                from concurrent.futures import ThreadPoolExecutor
                self._esdf_pool = ThreadPoolExecutor(1, thread_name_prefix="esdf")
                self._esdf_pending = 0
            self._esdf_pending += 1

            def job():
                try:
                    vm.observe_free(*args, epoch=epoch)
                except Exception as e:
                    self.log(f"[esdf] 빈 칸 갱신 실패 {e!r}")
                finally:
                    self._esdf_pending -= 1
            self._esdf_pool.submit(job)
        except Exception as e:
            self.log(f"[esdf] 빈 칸 갱신 실패 {e!r}")

    # ---------------- 재위치 (reloc.py) ----------------
    def _reloc_get(self):
        """Relocalizer (지금 세션 lens0u 보정으로). 만들 수 없으면 None."""
        if self.a.reloc != "on" or self.a.ov == "none":
            return None
        g = self.geom or {}
        vio = g.get("vio")
        if not vio:
            return None
        rl = self.reloc
        if rl is not None and getattr(rl, "geom_size", None) == tuple(g["size"]) and np.allclose(rl.K_full, vio["K"]):
            return rl
        import reloc
        rl = reloc.Relocalizer(self.log, vio["K"], vio["map"], on_aligned=self._reloc_on_aligned)
        rl.geom_size, rl.K_full = tuple(g["size"]), np.asarray(vio["K"], float)
        self.reloc = rl
        return rl

    def _reloc_reset(self):
        """새 세션·재생 시작: 조각 상태·재위치 DB 비움, --reloc-test-split 다시 장전."""
        with self._reloc_lock:
            self._seg_T, self._seg_n, self._pend = np.eye(4), 0, None
            self._requeue.clear()
            self._post_checks = 0
            self._base_checks = 0
            self.vio_events = []
            self._test_split = None
            ts = self.a.reloc_test_split
            if ts:
                v = [float(x) for x in str(ts).split(",")]
                yaw = np.radians(v[1])
                R = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]])
                self._test_split = dict(t=v[0], T=frames.make_T(R, v[2:5]), state="armed", truth=None, seg=None)
        if self.reloc is not None:
            self.reloc.clear()

    def _reloc_test_apply(self, T, t):
        ts = self._test_split
        if ts is None:
            return T
        if ts["state"] == "armed" and t >= ts["t"]:
            ts["state"] = "on"
            base = self._seg_T if self._seg_T is not None else np.eye(4)
            ts["truth"] = base @ frames.inv_T(ts["T"])      # 정답 T_old_new (가짜 변환의 역)
            self.log(f"[reloc] 시험: t={t:.2f} s 에 가짜 리셋, 이후 포즈에 yaw·이동 변환을 곱함")
            self.on_vio_event({"type": "reset", "auto": True, "reason": "test_split"})
            ts["seg"] = self._seg_n
        return ts["T"] @ T if ts["state"] == "on" else T

    def _reloc_start_segment(self, traj_before):
        """VIO 리셋: 지금 지도가 전역이면 통째로 보관 (붙을 때 되돌림), 새 조각은 새 지도에 따로."""
        with self._reloc_lock:
            self._seg_n += 1
            seg = self._seg_n
            old = self.vmap
            if self._pend is None:
                main = dict(vmap=old, tl=self.replay_tl, depth=self.replay_depth, traj=traj_before, objects=self.objects)
            else:
                main = self._pend["main"]
                # 붙지 못한 조각: 복원 끝에 가장 큰 조각 후보로만 남김
                if self.mode == "replay" and old.n and (self._best_seg is None or old.n > self._best_seg["vmap"].n):
                    self._best_seg = dict(vmap=old, tl=self.replay_tl, depth=self.replay_depth, traj=traj_before,
                                          objects=self.objects, seg=self._pend["seg"])
            self._seg_T = None
            # 저장 지도 기준 세션이 지도에 붙기 전에 VIO 가 리셋되면 새 조각도 '지도 기준' (끝까지 못 붙으면 불러온 지도를 결과로 안 씀)
            bm = self._pend.get("base_map") if self._pend is not None else None
            self._pend = dict(main=main, jobs=[], seg=seg, n_query=0, base_map=bm)
            nv = VoxelMap(self.a.voxel, self.a.min_hits, names=old.names)
            self._epoch_hi = max(self._epoch_hi, old.epoch, main["vmap"].epoch) + 1
            nv.epoch = self._epoch_hi                     # 보관한 전역 지도·버린 조각과 겹치지 않는 epoch (REVIEW 8)
            nv._done_t = set()                            # 이 조각 지도에 깊이를 넣은(시도한) 키프레임 시각 → 붙을 때 다시 넣을 것 가림
            self.vmap = nv
            self.objects = []
            self.replay_tl, self.replay_depth = [], {}
            self._post_checks = 0
        rl = self._reloc_get()
        rl.begin_segment(seg)
        self.log(f"[reloc] 조각 {seg} 시작: 옛 지도(복셀 {main['vmap'].n}, 기준 키프레임 {rl.n_db()}) 는 보관, "
                 f"이미 찍은 곳을 다시 비추면 이어 붙임")

    def _reloc_pending_kf(self, pair, T_vio, t, mot, pend):
        """붙기 전 키프레임: 나중에 전역 좌표로 다시 넣을 수 있게 영상(JPEG)·조각 좌표 자세를 보관, 재위치 질의."""
        if len(pend["jobs"]) < 300:
            fr = []
            for f in pair:
                ok, b = cv2.imencode(".jpg", f.bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    return
                fr.append((b, f.index, f.timestamp_ns))
            pend["jobs"].append(dict(fr=fr, T=np.array(T_vio, float), t=t, mot=mot))
        rl = self.reloc
        if rl is None or rl.n_db() == 0 or rl.q.qsize() > 6:
            return
        pend["n_query"] += 1
        ts = self._test_split
        truth = ts["truth"] if ts is not None and ts.get("seg") == pend["seg"] else None
        mt = self._map_test
        if truth is None and mt is not None and pend.get("base_map"):
            truth = mt["truth"]
        R1 = np.asarray(self.rp["R1"], float)
        rl.submit("query", bgr=pair[0].bgr, T_seg_c1=np.asarray(T_vio, float) @ frames.make_T(R1, np.zeros(3)), t=t,
                  seg=pend["seg"], truth=truth)

    @staticmethod
    def _kf_tried(vm, epoch, t):
        """조각 지도(_done_t 가 있는 VoxelMap)에 깊이를 넣었거나 (움직임 보정이 버려) 시도한 키프레임 시각 기록."""
        s = getattr(vm, "_done_t", None)
        if s is not None and epoch == vm.epoch:
            s.add(t)

    def _reloc_on_aligned(self, seg, T, info):
        """재위치 스레드: 조각이 붙음 → 전역 지도 되돌리고, 조각이 따로 쌓던 지도(복셀 + TSDF)를 T 로 옮겨 그대로 합침
        (VoxelMap.merge, 깊이 다시 계산 안 함). 조각 궤적·리플레이 시간 정보(키프레임 목록·깊이 그림)도 전역 좌표·번호로.
        합치기 전에 조각 지도에 못 들어간 키프레임(처리 중·대기열)만 전역 좌표로 깊이를 다시 계산. merge 가 실패하면 전부 다시 계산."""
        T = np.asarray(T, float)
        ms = self._map_sess
        loc = ms is not None and ms.get("mode") == "localize" and ms.get("status") != "error"
        pend0 = self._pend
        if pend0 is not None and pend0["seg"] == seg and pend0.get("base_map"):
            ok, vf = self._map_verify(pend0["main"]["vmap"], self.vmap, T, t=info.get("t"))
            info["verify"] = vf
            if not ok:
                # 저장 지도 기준 붙임은 기하 확인까지: 이 세션 조각 지도 점을 T 로 옮겨 지도 점과 겹치는 정도가 낮으면 거절하고
                # 계속 찾음 (사진 특징 두 장 합의만으로는 15 cm 어긋나게 붙은 적 있음 — 175446 → 143702 지도)
                with self._reloc_lock:
                    if ms is not None:
                        ms["rejected"] = ms.get("rejected", 0) + 1
                        ms["last_reject"] = vf
                self.log(f"[map] 붙임 후보 거절 (기하 확인): {vf} → 계속 찾음")
                rl = self.reloc
                if rl is not None and self._pend is pend0:
                    rl.begin_segment(seg)
                return
        with self._reloc_lock:
            pend = self._pend
            if pend is None or pend["seg"] != seg:
                return
            if ms is not None and pend.get("base_map"):
                ms.update(status="attached", attached_wall=time.time(), attached_t=info.get("t"), info=dict(info),
                          T_map_session=T.round(5).tolist())
                self.log(f"[map] 지도 '{pend['base_map']}' 에 붙음 (t={info.get('t')} s, 키프레임 {info.get('n_agree')}장 합의) → "
                         + ("위치만 추정 (지도 고정)" if loc else "지도 좌표로 이어 쌓음"))
            main = pend["main"]
            gvm, piece = main["vmap"], self.vmap
            self._seg_T = T
            self._pend = None
            with self.traj_lock:
                self.traj = list(main["traj"]) + [(tt, T @ TT) for tt, TT in self.traj]
                self.cur_T = self.traj[-1][1] if self.traj else None
            lk = self.last_kf
            if lk is not None:
                self.last_kf = (lk[0], T @ lk[1])
            with piece.lock:                               # 조각에 넣는 중인 키프레임이 끝난 뒤 바꿈 (그 뒤 조각 epoch 은 버려짐)
                self.vmap = gvm                            # epoch 그대로: 리셋 전에 대기열에 있던 전역 키프레임도 들어감
                p_tl, p_depth = self.replay_tl, self.replay_depth
                self.replay_tl, self.replay_depth, self.objects = main["tl"], main["depth"], main["objects"]
                done = set(getattr(piece, "_done_t", None) or ())
                piece._done_t = None
            self._post_checks = 4
        t0 = time.perf_counter()
        mres = None
        if piece is not gvm and self.a.reloc_merge == "on" and not loc:
            try:
                with gvm.lock:                             # 합친 복셀 번호와 시간 정보 번호가 같게 (그 사이 새 키프레임 안 끼게)
                    mres = gvm.merge(piece, T, keep_time=True)
                    kb = mres["kf_base"]
                    for e in p_tl:
                        self.replay_tl.append(dict(e, i=kb + int(e["i"]), T=T @ np.asarray(e["T"], float)))
                    for k, v in p_depth.items():
                        self.replay_depth[kb + int(k)] = v
            except Exception as e:
                mres = None
                self.log(f"[reloc] 조각 지도 합치기(merge) 실패 {e!r} → 키프레임 깊이를 전역 좌표로 다시 계산")
        ms = round(1000 * (time.perf_counter() - t0), 1)
        jobs = [] if loc else pend["jobs"] if mres is None else [j for j in pend["jobs"] if j["t"] not in done]
        with self._reloc_lock:
            ep = gvm.epoch
            for j in jobs:
                self._requeue.append(dict(j, T=T @ j["T"], epoch=ep))
            mg = None if mres is None else dict(voxels=mres["voxels"], tsdf=mres["tsdf"], kf=len(p_tl) or int(piece.frames), ms=ms)
            self.vio_events.append(dict(info, t=round(float(getattr(self, "_last_pose_t", 0.0)), 3), kind="aligned",
                                        requeued=len(jobs), merge=mg, wall=time.time()))
        if mg is not None:
            self.log(f"[reloc] 조각 {seg} → 전역 지도에 합침 (merge {ms:.0f} ms): 조각 지도 키프레임 {mg['kf']}개 · 복셀 {mg['voxels']} · "
                     f"TSDF {mg['tsdf']} 를 그대로 옮김, 아직 깊이 전이던 키프레임 {len(jobs)}개만 전역 좌표로 다시 넣음")
        else:
            self.log(f"[reloc] 조각 {seg} → 전역 지도에 합침: 키프레임 {len(jobs)}개 전역 좌표로 다시 넣음")
        if self.mode != "replay" and jobs:
            threading.Thread(target=self._reloc_feed_live, daemon=True, name="reloc-feed").start()

    def take_reloc_jobs(self):
        """붙은 조각의 키프레임 job 들 (pair, T, t, epoch, mot) — 재생은 replay.py 가 대기열에 넣는다."""
        out = []
        while self._requeue:
            try:
                j = self._requeue.popleft()
            except IndexError:
                break
            pair = tuple(SimpleNamespace(bgr=cv2.imdecode(b, cv2.IMREAD_COLOR), index=i, timestamp_ns=ts)
                         for b, i, ts in j["fr"])
            out.append((pair, j["T"], j["t"], j["epoch"], j["mot"]))
        return out

    def _reloc_feed_live(self):
        """실시간: 다시 넣을 키프레임을 깊이 대기열이 빌 때마다 하나씩."""
        while self._requeue and not self.stopped and self.mode in PHONE_MODES:
            if self.kf_queue.qsize() >= 2 or self.worker_busy:
                time.sleep(0.05)
                continue
            for job in self.take_reloc_jobs()[:1]:
                try:
                    self.kf_queue.put(job, timeout=1.0)
                except queue.Full:
                    pass

    def reloc_flush(self, timeout=20.0):
        """재생 끝: 재위치 작업 대기열이 빌 때까지 (최대 timeout s)."""
        rl = self.reloc
        t_end = time.monotonic() + timeout
        while rl is not None and (rl.q.qsize() or rl.busy) and time.monotonic() < t_end:
            time.sleep(0.05)

    def _reloc_kf_done(self, fL, T, t, epoch, r, rp, R_fix):
        """깊이를 만든 전역 키프레임 → 재위치 기준(DB)에 추가. 붙은 직후 몇 장은 옛 지도와 겹침 확인 (정렬 정확도)."""
        try:
            if self.a.reloc != "on" or self._pend is not None or self._seg_T is None or epoch != self.vmap.epoch:
                return
            rl = self._reloc_get()
            if rl is None:
                return
            R1 = np.asarray(self.rp["R1"], float)
            Rf = np.asarray(R_fix, float) if R_fix is not None else np.eye(3)
            check = None
            if self._post_checks > 0:
                self._post_checks -= 1
                check = self._seg_n
            elif self._seg_n == 0 and rl.n_db() >= 10 and self._base_checks < 80:
                self._base_checks += 1
                if self._base_checks % 10 == 0:
                    check = "base"                        # 비교 기준: 리셋 없는 평소 키프레임의 겹침 (최대 8번)
            if check is not None:
                pts = np.asarray(r["pts_rect0"], float) @ Rf.T
                rl.submit("check", pts_w=frames.transform(np.asarray(T, float), pts.astype(np.float32)), seg=check, t=t)
            Z = r["Z"]
            sx = Z.shape[1] / float(rp["size"][0])
            rl.submit("db", bgr=fL.bgr, T_w_c1=np.asarray(T, float) @ frames.make_T(R1, np.zeros(3)), Z=Z,
                      valid=r["valid"], f=rp["f"] * sx, cx=rp["cx"] * sx, cy=rp["cy"] * sx, R_c1_d=R1.T @ Rf, t=t)
        except Exception as e:
            self.log(f"[reloc] 키프레임 추가 실패 {e!r}")

    def reloc_state(self):
        """웹·summary 용 재위치 상태."""
        rl = self.reloc
        pend = self._pend
        d = dict(on=self.a.reloc == "on", pending=pend is not None, seg=self._seg_n,
                 pending_kf=len(pend["jobs"]) if pend else 0, events=list(self.vio_events[-20:]))
        if rl is not None:
            d.update(rl.summary())
        return d

    def _depth_worker(self):
        import depth_seg
        try:
            self.ds = depth_seg.DepthSeg(self.rp, self.want_iters, self.geom["ffs_scale"], tuple(self.a.depth_range), self.a.stride,
                                         self.a.yolo_model, self.a.yolo_conf, self.a.yolo_rotate, self.a.device,
                                         self.a.no_yolo, self.log)
            self.vmap.names = self.ds.names
            if self.aruco is not None and self.a.aruco_apply:
                self.ds.disp_fix = self.aruco.apply_disp
            self.dstats["load_stage"] = "GPU 준비 (첫 계산)"
            self.ds.warmup()
            self.dstats["ready"] = True
            self.dstats["loading"] = False
            self.dstats["load_stage"] = None
            if self.dstats.get("load_t0"):
                self.dstats["load_s"] = round(time.time() - self.dstats["load_t0"], 1)
            self.dstats["iters"] = self.ds.iters
            self.log("[depth] 깊이 모델 준비됨")
        except Exception as e:
            self.dstats["error"] = f"init: {e!r}"
            self.dstats["loading"] = False
            self.dstats["load_stage"] = None
            self.log(f"[depth] 초기화 실패: {e!r}")
            return
        while not self.stopped:
            if self.ds.iters != self.want_iters:
                try:
                    self.ds.set_iters(self.want_iters)
                except Exception as e:
                    self.log(f"[depth] iters 변경 실패: {e!r}")
                    self.want_iters = self.ds.iters
                self.dstats["iters"] = self.ds.iters
            try:
                job = self.kf_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            self.worker_busy = True
            try:
                self._process_keyframe(*job)
            except Exception as e:
                self.dstats["error"] = repr(e)
                self.log(f"[depth] 키프레임 오류: {e!r}")
            finally:
                self.worker_busy = False

    def _pair_rect(self, g, mot):
        """키프레임 쌍의 정류맵 정하기 (움직임 보정, motion_rect.py). 반환 (rp, R_rect0_from_pair 또는 None, info) —
        rp None = 이 키프레임은 깊이를 만들지 않음 (ΔT 를 모르고 |dt| 도 커서 고정 보정이면 깊이가 틀어짐)."""
        rp0 = g["rp"]
        dt_ms = (mot or {}).get("dt_ns", 0) * 1e-6
        if self.a.motion_rect == "off" or mot is None:
            return rp0, None, dict(mode="fixed", dt_ms=round(dt_ms, 3), why="--motion-rect off" if mot else "dt 없음")
        # 실시간: lens1 이 lens0 보다 늦게 찍혔으면 (dt > 0) 그 시각까지의 자이로가 아직 안 왔을 수 있다 → 잠깐 (최대 0.2 s) 기다림
        t_need = mot["t0_ns"] + max(0, int(mot.get("dt_ns") or 0))
        t_wait = time.monotonic() + 0.2
        while (not self.gyro_buf or self.gyro_buf[-1][0] < t_need) and time.monotonic() < t_wait and not self.stopped:
            time.sleep(0.005)
        try:
            gyro = list(self.gyro_buf)
        except RuntimeError:                              # 다른 스레드가 append 중 → 한 번 더
            gyro = list(self.gyro_buf)
        dT, info = motion_rect.motion_delta(g["st"], rp0, mot, gyro)
        if dT is None:
            # ΔT 모름 (자이로 창 없음·VIO 속도 없음): |dt| 가 아주 작을 때만 고정 보정 (FIXED_MAX_DT_MS 주석), 아니면 버림
            if abs(dt_ms) <= motion_rect.FIXED_MAX_DT_MS:
                return rp0, None, dict(info, mode="fixed")
            return None, None, dict(info, mode="skipped")
        if motion_rect.tiny(info):
            return rp0, None, dict(info, mode="fixed", why="움직임 작음")
        ok, binfo = motion_rect.baseline_check(g["st"], dT)
        info.update(binfo)
        if not ok:
            return None, None, dict(info, mode="skipped", why="너무 빨리 움직임 (기선이 크게 돎)")
        try:
            rpp = motion_rect.rectify_with_delta(g["st"], dT)
        except ValueError as e:                           # 상하 배치 등: 그 키프레임만 버림
            return None, None, dict(info, mode="skipped", why=str(e)[:80])
        info.update(mode="motion", rect_ms=round(rpp["rect_ms"], 2))
        return rpp, motion_rect.R_rect0_from(rp0, rpp), info

    def _note_mrect(self, info, t):
        m = self.mrect
        m[info["mode"]] = m.get(info["mode"], 0) + 1
        info = dict(info, t=round(t, 3))
        m["last"] = info
        m["recent"].append(info)

    def _process_keyframe(self, pair, T, t, epoch, mot=None):
        t_start = time.perf_counter()
        fL, fR = pair
        if epoch != self.vmap.epoch:
            return                                            # 그 사이 맵 리셋 (REVIEW 8)
        g = self.geom                                         # 한 키프레임 안에서는 같은 정류맵·FFS 축소
        if (fL.bgr.shape[1], fL.bgr.shape[0]) != tuple(g["rp"]["size"]):
            return                                            # 해상도 바뀌기 전 키프레임
        # 움직임 보정 정류: 이 쌍의 두 촬영 순간 사이 폰 움직임 ΔT 로 그 쌍만의 정류맵 (깊이는 lens0 시각 기준)
        rp, R_fix, info = self._pair_rect(g, mot)
        self._note_mrect(info, t)
        if rp is None:
            self._kf_tried(self.vmap, epoch, t)
            return
        r = self.ds(fL.bgr, fR.bgr, rp=rp, scale=g["ffs_scale"])
        if self.aruco is not None:
            # 보드 관측: 보정 전 시차 (disp_raw) + 이 쌍의 rp, 자세는 쌍별 정류 rect0' 의 월드 자세 (ARUCO_SCALE.md 5 절)
            try:
                T_cam = np.array(T, np.float64)
                if R_fix is not None:
                    T_cam[:3, :3] = T_cam[:3, :3] @ np.asarray(R_fix, np.float64)
                self.aruco.observe(r["rectL"], r.get("disp_raw", r["disp"]), rp, t, valid=r["valid"], T_world_cam=T_cam,
                                   motion=dict(rot_deg=(info or {}).get("rot_deg", 0), trans_mm=(info or {}).get("trans_mm", 0)),
                                   rect1=r.get("rectR"))
            except Exception as e:
                self.log(f"[aruco] 관측 실패 {e!r}")
        if self.paused and self.mode != "replay":
            return
        # 여러 키프레임 일관성 (depth_fuse.py, 기본 꺼짐 — SCAN3D_CONSIST=1): 앞 키프레임 2 장과 어긋난 화소 (빈 공간에 뜬 점) 를 뺌
        try:
            import depth_fuse
            T_c = np.array(T, np.float64)
            if R_fix is not None:
                T_c[:3, :3] = T_c[:3, :3] @ np.asarray(R_fix, np.float64)
            ci = depth_fuse.realtime_check(self, r, rp, T_c, g["ffs_scale"], t, epoch, self.vmap,
                                           float(voxel_map.tsdf_cfg()["sigma_px"]))
            self.dstats["consist"] = ci
        except Exception as e:
            self.log(f"[depth] 일관성 거르기 실패 {e!r}")
        pts = r["pts_rect0"]
        if R_fix is not None:
            # 쌍별 정류 좌표 rect0' → 기본 rect0 (T_world_rect0 는 기본 R1 기준, lens0 시각 포즈)
            pts = (pts @ R_fix.T.astype(np.float32)).astype(np.float32)
        pts_w = frames.transform(T, pts)
        # 물체 인스턴스 (2D 트랙 id 우선) → 점마다 전역 id. 리셋과 겹치지 않게 맵 lock 안에서 (REVIEW 8)
        inst = np.full(len(pts_w), -1, np.int32)
        di = r["det_idx"]
        ent = jpg = None
        if self.mode == "replay":
            # 리플레이 시간 정보 재료 (맵 lock 밖에서 미리): 깊이 렌즈 쌍 비교 숫자 (replay_view.kf_quality), 깊이 그림
            ent = dict(t=t, fr=[(fL.index, fL.timestamp_ns), (fR.index, fR.timestamp_ns)], T=np.array(T, float))
            try:
                ent["q"] = replay_view.kf_quality(r["pts_rect0"], r["valid"], seed=int(fL.timestamp_ns) & 0xFFFFFFFF)
            except Exception as e:
                self.log(f"[replay] 비교 숫자 실패 {e!r}")
            try:
                jpg = replay_view.depth_jpeg(r["Z"], r["valid"], self.a.depth_range[0], self.a.depth_range[1])
            except Exception as e:
                self.log(f"[replay] 깊이 그림 실패 {e!r}")
        vm = self.vmap                                        # 재위치가 조각 → 전역 지도로 바꿔도 이 키프레임은 한 지도에만
        with vm.lock:
            if epoch != vm.epoch:
                return
            for k in np.unique(di[di >= 0]):
                d = r["dets"][k]
                sel = di == k
                iid = vm.tracker.associate(d["class_id"], pts_w[sel], d["track_id"], t)
                inst[sel] = iid
                d["inst"] = iid
            nf0 = vm.frames
            vm.integrate(pts_w, r["rgb"], r["cls"], inst, t, epoch=epoch)
            kf_i = vm.frames if vm.frames != nf0 else None
            # TSDF (메시): 깊이 지도 그대로 투영형 갱신. 깊이 카메라 = 쌍별 정류 rect0' → 월드 = T_world_rect0 · R_fix
            T_cam = np.array(T, np.float64)
            if R_fix is not None:
                T_cam[:3, :3] = T_cam[:3, :3] @ np.asarray(R_fix, np.float64)
            try:
                tsr = vm.integrate_depth(r["Z"], r["valid"], r["rectL"], rp, T_cam, ffs_scale=g["ffs_scale"],
                                         t=t, epoch=epoch, log=self.log)
            except Exception as e:
                tsr = None
                self.log(f"[tsdf] 적분 실패 {e!r}")
            self.dstats["tsdf_ms"] = round(tsr["ms"], 1) if tsr else None
            if kf_i is not None and ent is not None:
                # 리플레이 시간 정보: 키프레임 번호 = 복셀의 kf_shown 과 같은 번호 (voxel_map.frames).
                # 맵 lock 안에서 넣음 → 재위치가 지도를 바꿀 때 (_reloc_on_aligned) 시간 정보도 같은 지도 것으로 함께 바뀜
                ent["i"] = kf_i
                self.replay_tl.append(ent)
                if jpg:
                    self.replay_depth[kf_i] = jpg
            self._kf_tried(vm, epoch, t)
        now = time.monotonic()
        if now - self.objects_t > 1.0:
            self.objects = self.vmap.objects()
            self.objects_t = now
        self.kf_times.append(now)
        ds = self.dstats
        ds.update(ffs_ms=round(r["ffs_ms"], 1), yolo_ms=round(r["yolo_ms"], 1), points=r["n"],
                  size=list(r["size"]), ffs_scale=r["ffs_scale"],
                  total_ms=round(1000 * (time.perf_counter() - t_start), 1), keyframes=ds["keyframes"] + 1,
                  n_dets=len(r["dets"]), error=None)
        if len(self.kf_times) > 1:
            ds["fps"] = round((len(self.kf_times) - 1) / max(1e-6, self.kf_times[-1] - self.kf_times[0]), 2)
        with self.panel_lock:
            self.panel = dict(rect=r["rectL"], Z=r["Z"], valid=r["valid"], dets=r["dets"], t=t)
        self._esdf_kf_done(vm, epoch, r, rp, T, R_fix)      # ESDF 관측된 빈 칸 (esdf_map.py)
        self._reloc_kf_done(fL, T, t, epoch, r, rp, R_fix)   # 재위치 기준 키프레임 (reloc.py, 전역 키프레임만)

    # ---------------- 웹 API ----------------
    def state(self):
        rx = self.receiver
        ph = rx.stats() if rx else {}
        phone = dict(frames=ph.get("frames"), fps=ph.get("sensor_fps"), pairs=ph.get("pairs"),
                     dropped=ph.get("unmatched_dropped"), reordered=ph.get("reordered_dropped"),
                     skew_ms=ph.get("skew_median_ms"), skew_p95_ms=ph.get("skew_p95_ms"),
                     clock_gap_ms=ph.get("clock_gap_ms"), error=ph.get("error"),
                     imu_rates={k: v["rate_hz"] for k, v in (ph.get("imu") or {}).items()},
                     imu_sync=dict(out=self.imu.n_out, dropped=self.imu.n_drop),
                     pair_skew_ms=self.pair_skew_ms(), connected=rx is not None)
        phone.update(self._pair_drop(ph))
        h = ph.get("h264")
        if isinstance(h, dict):
            # 렌즈별 H.264 디코더 상태 (REVIEW7 H6, 웹 고급 → H.264 줄)
            phone["h264"] = {k: h.get(k) for k in ("waiting_key", "corrupt", "decode_errors", "seq_gaps", "seq_gap_resync",
                                                    "waiting_key_dropped", "keyreq_sent", "b_frames_seen", "kbps",
                                                    "keyframes", "frames")}
            phone["h264"].update(decode_ms=h.get("decode_ms_avg"), decode_ms_max=h.get("decode_ms_max"))
        vio = self.vio.stats()
        try:
            sk = np.array(self.skew_ms.copy(), float) if self.skew_ms else None   # 다른 스레드가 append 중이면 다음에
        except RuntimeError:
            sk = None
        vio.update(stereo_frames=self.vio_stereo, mono_frames=self.vio_mono,
                   images="rect" if self.a.vio_mode == "stereo" else self.vio_cam0,   # mono 기본 lens0u (왜곡만 편 렌즈 2)
                   auto_resets=self.vio_auto_resets, vio_mode=self.a.vio_mode, bridge_restarts=self.bridge_restarts,
                   skew_dropped=self.vio_skew_dropped, skew_limit_ms=self.a.vio_stereo_skew_ms,
                   pair_skew_ms=self.pair_skew_ms(), bridge_fails=self.bridge_fails,
                   skew_med_ms=None if sk is None else round(float(np.median(sk)), 2),
                   skew_p95_ms=None if sk is None else round(float(np.percentile(sk, 95)), 2),
                   skew_n=0 if sk is None else int(sk.size))
        # 재위치·다시 시작 안내 (웹 상태 줄에 크게): 정지 초기화를 기다리는 중이면 '잠깐 멈추세요'
        rs = self.reloc_state()
        vio["reloc"] = dict(on=rs["on"], pending=rs["pending"], seg=rs["seg"], pending_kf=rs["pending_kf"], db=rs.get("db", 0),
                            aligned=len(rs.get("alignments") or []), queries=rs.get("queries", 0))
        hint = None
        if self.vio_auto_resets and vio.get("status") == "init" and self.mode in PHONE_MODES:
            hint = "추적이 끊겼습니다 — 폰을 1~2초 가만히 들고 있으면 다시 시작합니다"
        elif rs["pending"]:
            hint = "새 조각을 따로 쌓는 중 — 이미 찍은 곳을 천천히 다시 비추면 옛 지도에 이어 붙입니다"
        vio["hint"] = hint
        try:
            dts = np.array(self.dt_ms.copy(), float) if self.dt_ms else None
        except RuntimeError:
            dts = None
        m = self.mrect
        motion = dict(on=self.a.motion_rect == "on", motion=m["motion"], fixed=m["fixed"], skipped=m["skipped"],
                      last=m["last"], recent=list(m["recent"]), fixed_max_dt_ms=motion_rect.FIXED_MAX_DT_MS,
                      dt_med_ms=None if dts is None else round(float(np.median(dts)), 2),
                      dt_min_ms=None if dts is None else round(float(dts.min()), 2),
                      dt_max_ms=None if dts is None else round(float(dts.max()), 2),
                      gyro_buf=len(self.gyro_buf))
        if self.vio_error and self.mode in ("live", "both"):
            # 다시 띄우기·시작 실패의 진짜 원인을 닫힌 옛 브리지 오류보다 먼저 (REVIEW5 N5)
            old = vio.get("error")
            vio["error"] = self.vio_error + (f" (이전 오류: {old})" if old and old != self.vio_error else "")
        elif self.vio_error and not vio.get("error"):
            vio["error"] = self.vio_error
        with self.traj_lock:
            T = self.cur_T
        vio["T_world_rect0"] = None if T is None else T.reshape(-1).round(5).tolist()
        names = self.vmap.names or {}
        rp = self.replayer
        session = dict(mode=self.mode, link=self.a.link, phone_spec=self.a.phone, adb=self.link.info(),
                       app=self.app_status, rec=self.rec_status, pull=self.pull_progress,
                       # 녹화 모드만: 1 s 마다 읽는 status 꼬리. 앱 시작 때 한 번 찍은 app_status 는 낡아서 안 씀 (REVIEW8 L3)
                       sync=sync_from_status(self.rec_status) if self.mode == "record" else None,
                       replay=dict(rp.progress) if rp is not None else None, last_result=self.last_result,
                       rec_root=self.a.rec_root, cams=self.cams_info(), recsync=self.recsync.snapshot())
        return dict(t=time.time(), paused=self.paused, vio=vio, phone=phone, depth=dict(self.dstats, motion=motion),
                    session=session,
                    map=dict(epoch=self.vmap.epoch, version=self.vmap.version, voxels=self.vmap.n,
                             shown=self.vmap.count_shown(), voxel_m=self.vmap.voxel_m, min_hits=self.vmap.min_hits,
                             frames=self.vmap.frames),
                    map_store=self.map_state(),
                    objects=self.objects, epi=self.epi, aruco=self._aruco_state(),
                    classes={int(k): dict(name=v, color=list(class_color(int(k)))) for k, v in names.items()},
                    log=list(self.log_lines)[-14:], stream=self.stream_info(),
                    settings=dict(depth_range=self.a.depth_range, stride=self.a.stride, iters=self.a.iters,
                                  replay_iters=self.a.replay_iters, kf_trans=self.a.kf_trans, kf_rot=self.a.kf_rot))

    def _aruco_state(self):
        """웹 상태용 보드 관측 요약 (없으면 None). 'last' 의 큰 숫자 목록은 빼고 사람이 읽을 것만."""
        if self.aruco is None:
            return None
        try:
            s = self.aruco.status()
        except Exception as e:
            return dict(error=repr(e))
        last = s.pop("last", None) or {}
        s["last"] = {k: last.get(k) for k in ("t", "used", "why", "z_m", "angle_deg", "offset_px", "depth_err_pct") if k in last}
        s["apply"] = bool(self.a.aruco_apply)
        return s

    def _pair_drop(self, ph):
        """최근 약 10 s 동안 짝 없어 버린 프레임 비율 (REVIEW5 N1). 기준(--pair-skew-ms)은 자동으로 넓히지 않고 웹이 안내만.
        반환 pair_drop_pct (None = 아직 모름), pair_drop_window_s, pair_drop_frames."""
        try:
            tot = int(sum(ph.get("frames") or []))
            drop = int(sum(ph.get("unmatched_dropped") or []))
        except (TypeError, ValueError):
            tot = drop = 0
        now = time.monotonic()
        h = self.pair_hist
        if not ph or (h and (tot < h[-1][1] or drop < h[-1][2])):
            h.clear()                                     # 수신기 바뀜 (카운터 처음부터)
        if ph:
            if not h or now - h[-1][0] >= 0.5:
                h.append((now, tot, drop))
            while len(h) > 2 and now - h[1][0] >= 10.0:
                h.popleft()
        if len(h) < 2:
            return dict(pair_drop_pct=None, pair_drop_window_s=None, pair_drop_frames=0)
        t0, tot0, drop0 = h[0]
        dn, dd = tot - tot0, drop - drop0
        if dn < 30:                                       # 프레임이 너무 적으면 비율을 안 냄
            return dict(pair_drop_pct=None, pair_drop_window_s=round(now - t0, 1), pair_drop_frames=dn)
        return dict(pair_drop_pct=round(100.0 * dd / dn, 1), pair_drop_window_s=round(now - t0, 1), pair_drop_frames=dn)

    def recordings_json(self, refresh=False):
        phone, err = None, None
        if self.link.available:
            if refresh or self._phone_recs is None:
                try:
                    self._phone_recs = self.link.list_recordings()
                    self._phone_recs_t = time.time()
                except Exception as e:
                    err = repr(e)
            phone = self._phone_recs
        else:
            err = "adb 없음"
        local = phone_link.list_local(self.a.rec_root)
        names = {r["name"] for r in local}
        for r in phone or []:
            r["local"] = r["name"] in names
        for r in local:
            r["ready"] = replay_view.best_recon(r["path"])
        return dict(phone=phone, phone_error=err, phone_t=self._phone_recs_t, local=local, rec_root=self.a.rec_root,
                    items=self.recsync.items(), auto=self.recsync.snapshot())

    def trajectory_json(self):
        with self.traj_lock:
            tr = list(self.traj)
            T = self.cur_T
        step = max(1, len(tr) // 3000)
        pts = [[round(t, 3), *np.round(M[:3, 3], 4).tolist()] for t, M in tr[::step]]
        W, H = self.rp["size"]
        return dict(points=pts, T_world_rect0=None if T is None else T.reshape(-1).round(5).tolist(),
                    cam=dict(f=self.rp["f"], cx=self.rp["cx"], cy=self.rp["cy"], w=W, h=H))

    def frame_jpeg(self, kind):
        with self.panel_lock:
            p = dict(self.panel)
        if not p:
            return None
        if kind == "depth":
            img = depth_colormap(p["Z"], p["valid"], self.a.depth_range[0], self.a.depth_range[1])
        elif kind == "overlay":
            img = p["rect"].copy()
            for d in p["dets"]:
                col = class_color(d["class_id"])[::-1]
                if d["mask"] is not None:
                    cnts, _ = cv2.findContours((d["mask"] > 0).astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, cnts, -1, col, 2)
                x, y, w, h = d["box"]
                txt = f"{d['label']} #{d.get('inst', '-')} {d['conf']:.2f}"
                cv2.putText(img, txt, (x, max(14, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                cv2.putText(img, txt, (x, max(14, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        else:
            img = p["rect"]
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

    LENS_NAMES = {"2": "초광각", "5": "광각", "6": "망원"}

    def cam_jpeg(self, lens, overlay=False):
        """/api/cam.jpg — 가장 최근 수신 프레임 (센서 방향 그대로 640x480; 웹이 90° 돌려 보여 줄 수 있음).
        lens 0 = 정류 왼쪽 (렌즈 2 초광각), 1 = 오른쪽 (렌즈 5 광각 또는 6 망원 — 렌즈 쌍). JPEG 스트림이면 받은 바이트 그대로,
        아니면 cv2.imencode q80. overlay=True 면 렌즈 0 을 정류해 최근 키프레임의 YOLO 마스크·박스를 겹친다."""
        lens = 1 if int(lens) == 1 else 0
        f = None
        rx = self.receiver
        if rx is not None and self.mode in ("live", "both"):
            # 수신 스트림 번호 → 렌즈: swap 이면 스트림 1 이 왼쪽 (on_pair 와 같은 규칙)
            si = (1 - lens) if self.swap else lens
            try:
                f = rx.latest[si]
            except (AttributeError, IndexError, TypeError):
                f = None
        if f is None and self.mode == "replay":
            # 재생만 last_pair 로 (실시간에서 수신기가 없거나 아직 프레임이 없으면 옛 그림 대신 "영상 없음" — REVIEW5 N3)
            lp = self.last_pair
            if lp is not None:
                f = lp[lens]
        if f is None or getattr(f, "bgr", None) is None:
            return None
        if overlay and lens == 0:
            with self.panel_lock:
                dets = list(self.panel.get("dets") or [])
            img = cv2.remap(f.bgr, *self.rp["mapL"], interpolation=cv2.INTER_LINEAR)
            for d in dets:
                col = class_color(d["class_id"])[::-1]
                m = d.get("mask")
                if m is not None and m.shape[:2] == img.shape[:2]:
                    sel = m > 0
                    img[sel] = (0.6 * img[sel] + 0.4 * np.array(col, np.float32)).astype(np.uint8)
                    cnts, _ = cv2.findContours(sel.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, cnts, -1, col, 2)
                x, y, w, h = d["box"]
                cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), col, 2)
                txt = f"{d['label']} #{d.get('inst', '-')}"
                cv2.putText(img, txt, (int(x), max(14, int(y) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                cv2.putText(img, txt, (int(x), max(14, int(y) - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        else:
            raw = getattr(f, "jpeg", None)                # lib_phone_stereo 가 JPEG 스트림이면 받은 바이트를 붙여 둠
            if raw is not None:
                return bytes(raw)
            img = f.bgr
        if img.shape[1] > 640:
            img = cv2.resize(img, (640, int(round(img.shape[0] * 640 / img.shape[1]))), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

    def cams_info(self):
        phys = list(self.a.physical)
        if self.swap:
            phys = phys[::-1]
        return [dict(lens=i, physical=p, name=self.LENS_NAMES.get(str(p), f"렌즈 {p}")) for i, p in enumerate(phys)]

    def command(self, body):
        cmd = body.get("cmd")
        if cmd in ("mode", "replay") and "map" in body:
            # 저장 지도 기준으로 시작 (map = 이름 또는 null, map_mode = append | localize) — 이번 세션부터
            nm = body.get("map")
            self.map_req = dict(name=str(nm), mode=body.get("map_mode") or "append") if nm else None
        if cmd == "replay" and "map_save" in body:
            self.map_save_next = body.get("map_save") or None
        if cmd and str(cmd).startswith("map_"):
            return self._map_command(cmd, body)
        if cmd == "pause":
            self.paused = True
        elif cmd == "resume":
            self.paused = False
        elif cmd == "reset_map":
            with self.vmap.lock:
                self.vmap.reset()
            self.objects = []
            self.last_kf = None
            if self.ds is not None and self.ds.yolo:
                self.ds.yolo.reset()
        elif cmd == "reset_vio":
            self.vio.reset()
            self.vmap.reset()
            self.objects = []
            self.last_kf = None
            with self.traj_lock:
                self.traj, self.cur_T = [], None
        elif cmd == "save":
            return dict(ok=True, dir=str(self.save()))
        elif cmd == "mode":
            # body: mode, start_app, size ("1280x960" 또는 [W,H], 4:3 만), fps, ffs_scale, pair ("2+5"|"2+6") (없으면 지난 값)
            #       pair "all" = 렌즈 3개 모두 녹화 (비교용, mode record 만) — 그때 live_pair (기본 2+6) 는 다음 실시간용으로만 둠
            pair = body.get("pair")
            rec3 = str(pair or "") in ALL_LENSES
            if rec3:
                pair = body.get("live_pair") or "2+6"
            try:
                return self.start_session(body.get("mode", "live"), start_app=bool(body.get("start_app", True)),
                                          size=body.get("size"), fps=body.get("fps"), ffs_scale=body.get("ffs_scale"),
                                          pair=pair, rec3=rec3)
            except ValueError as e:
                return dict(ok=False, error=str(e))
        elif cmd == "stream":
            # 다음 실시간·녹화 세션의 크기·fps 만 정함 (검증; 적용은 다음 '시작')
            try:
                w = self.set_stream(size=body.get("size"), fps=body.get("fps"), ffs_scale=body.get("ffs_scale"),
                                    keep_ffs=body.get("size") is None, pair=body.get("pair"))
            except ValueError as e:
                return dict(ok=False, error=str(e))
            return dict(ok=True, want=dict(size=list(w["size"]), fps=w["fps"], ffs_scale=w["ffs_scale"], pair=w.get("pair")),
                        stream=self.stream_info())
        elif cmd == "stop":
            return self.stop_session(stop_app=bool(body.get("stop_app", True)))
        elif cmd == "recordings":
            return dict(ok=True, **self.recordings_json(refresh=bool(body.get("refresh", True))))
        elif cmd == "pull":
            name = str(body.get("name") or "")
            if not self.link.available:
                return dict(ok=False, error="adb 없음 (pull_rec.py 를 Windows 에서 실행하거나 SCAN3D_ADB 지정)")
            if self.pull_progress and self.pull_progress.get("state") == "가져오는 중":
                return dict(ok=False, error="이미 가져오는 중")
            rs = self.rec_status or {}
            if rs.get("rec") == name and str(rs.get("state", "")).startswith("RECORDING"):
                return dict(ok=False, error="녹화 중인 폴더 — 먼저 '정지' (mp4 마무리) 후 가져오기")
            # 가져오기는 rec_sync 한 줄(직렬)로: 자동 가져오기와 겹치지 않고, PC 에 같은 사본이 있으면 다시 안 가져온다
            return self.recsync.request_pull(name, replay=bool(body.get("replay", True)),
                                             every_frame=bool(body.get("every_frame", False)),
                                             iters=int(body.get("iters") or self.a.replay_iters))
        elif cmd == "rec_auto":
            return self.recsync.set_cfg(pull=body.get("pull"), recon=body.get("recon"))
        elif cmd == "rec_retry":
            return self.recsync.retry(str(body.get("name") or ""))
        elif cmd == "rec_delete_phone":
            return self.recsync.delete_phone(str(body.get("name") or ""))
        elif cmd == "replay":
            # pair: 3렌즈 녹화에서 '2+5' | '2+6' (없으면 2+6). 2렌즈 녹화는 생략 (녹화한 쌍)
            try:
                return self.start_replay(str(body.get("name") or body.get("dir") or ""), bool(body.get("every_frame", False)),
                                         int(body.get("iters") or self.a.replay_iters), pair=body.get("pair") or None)
            except ValueError as e:
                return dict(ok=False, error=str(e))
        elif cmd == "set":
            if "voxel_m" in body:
                v = float(body["voxel_m"])
                if 0.005 <= v <= 0.2 and abs(v - self.vmap.voxel_m) > 1e-9:
                    self.vmap.reset(v)
                    self.objects = []
                    self.last_kf = None
            if "min_hits" in body:
                self.vmap.set_min_hits(max(1, int(body["min_hits"])))
            if "depth_max" in body:
                self.a.depth_range[1] = float(np.clip(float(body["depth_max"]), self.a.depth_range[0] + 0.1, 3.0))
                if self.ds is not None:
                    self.ds.z_range = tuple(self.a.depth_range)
        else:
            return dict(ok=False, error=f"unknown cmd {cmd}")
        return dict(ok=True, paused=self.paused)

    def _map_command(self, cmd, body):
        """map_list | map_save {name} | map_delete {name} | map_use {name|null, mode} (다음 세션) | map_esdf {res}."""
        import map_store
        try:
            if cmd == "map_list":
                return dict(ok=True, maps=map_store.list_maps(), state=self.map_state())
            if cmd == "map_save":
                if self.mode == "replay":
                    return dict(ok=False, error="복원 중 — 끝난 뒤 저장 (또는 복원 명령에 map_save)")
                if not self.vmap.n and self._pend is None:
                    return dict(ok=False, error="지도가 비어 있음")
                name = map_store.safe_name(body.get("name") or time.strftime("map_%Y%m%d-%H%M%S"))
                return dict(ok=True, map=self.save_map(name))
            if cmd == "map_delete":
                map_store.delete_map(body.get("name"))
                if self.map_req and self.map_req.get("name") == body.get("name"):
                    self.map_req = None
                return dict(ok=True, maps=map_store.list_maps())
            if cmd == "map_use":
                nm = body.get("name")
                if nm:
                    map_store.map_dir(nm)
                    if not (map_store.map_dir(nm) / "map.json").is_file():
                        return dict(ok=False, error=f"지도 없음: {nm}")
                self.map_req = dict(name=str(nm), mode=body.get("mode") or "append") if nm else None
                return dict(ok=True, state=self.map_state())
            if cmd == "map_esdf":
                e = self.esdf_get(body.get("res"))
                return dict(ok=True, update=e.last)
        except (ValueError, FileNotFoundError, FileExistsError) as e:
            return dict(ok=False, error=str(e))
        return dict(ok=False, error=f"unknown cmd {cmd}")

    def save(self, d=None, extra=None, include_cfg=True):
        d = Path(d) if d else Path(self.a.out) / time.strftime("%Y%m%d-%H%M%S")
        with self.traj_lock:
            tr = list(self.traj)
        summary = dict(created=time.strftime("%Y-%m-%d %H:%M:%S"), args=vars(self.a), state=self.state())
        if extra:
            summary.update(extra)
        self.vmap.save(d, trajectory=[T for _, T in tr[::max(1, len(tr) // 2000)]], extra=summary)
        (d / "trajectory.json").write_text(json.dumps(
            [dict(t=t, T_world_rect0=T.reshape(-1).round(6).tolist()) for t, T in tr]))
        cfg = Path(self.vio_cfg_used).parent if self.vio_cfg_used else None   # 이 세션 크기·fps 의 설정 폴더
        if include_cfg and cfg is not None and cfg.is_dir():
            shutil.copytree(cfg, d / "vio_config", dirs_exist_ok=True, ignore=shutil.ignore_patterns("mono"))
        self.log(f"[save] {d}")
        return d

    def stop(self):
        self.stopped = True
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
        if self.a.save_on_exit and self.vmap.n:
            try:
                self.save()
            except Exception as e:
                self.log(f"[save] 실패 {e!r}")
        for v in (self.live_vio, self.vio):
            try:
                if v is not None:
                    v.close()
            except Exception:
                pass
        try:
            self.http.shutdown()
        except Exception:
            pass


def main(argv=None):
    a = parse_args(argv)
    sc = Scanner(a)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        sc.start()
        while not stop.wait(1.0):
            if a.exit_after_replay and a.replay and sc.replay_finished:
                break
    finally:
        sc.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
