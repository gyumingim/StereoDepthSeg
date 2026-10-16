#!/usr/bin/env python3
"""scan3d — 실시간 3D 복셀 맵 스캐너 (WSL/우분투, venv_ffs).

폰(렌즈 2+5 NV21 + IMU) ──릴레이/adb forward──▶ 수신 ─┬─ IMU 동기화 ───────────────┐
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
  venv_ffs/bin/python scan3d/scan3d.py --vio-mode mono              # 왼쪽 렌즈만 VIO (config/mono 설정)
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
import ov_client                # noqa: E402
import phone_link               # noqa: E402
import phone_rx                 # noqa: E402
from voxel_map import VoxelMap, class_color   # noqa: E402
import web_server               # noqa: E402

PHONE_MODES = ("live", "record", "both")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="실시간 3D 복셀 맵 스캐너 (OpenVINS + FFS + YOLO-seg)")
    p.add_argument("--calib", default=str(ROOT / "calib_phone_pair.json"))
    p.add_argument("--physical", nargs=2, default=["2", "5"], help="앱 --physical 순서 (스트림 0 = 왼쪽 렌즈 2)")
    p.add_argument("--app-size", default="640x480",
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
    p.add_argument("--restart-app", action="store_true", default=True,
                   help="폰 쪽이 연달아 바로 끊기면 앱 다시 시작 (adb 있을 때, REVIEW 11)")
    p.add_argument("--pair-skew-ms", type=float, default=5.0,
                   help="쌍 묶기 허용 두 렌즈 시각차 ms (이보다 먼 프레임은 짝을 안 지음). --vio-stereo-skew-ms 기본값도 이 값")
    p.add_argument("--ov", default="auto",
                   help="auto = scan3d 가 브리지를 직접 띄움 (죽으면 자동 재시작, 로그 runs/ov_bridge_live_*.log) | "
                        "HOST:PORT (따로 띄운 TCP 브리지) | pipe:<명령> | none")
    p.add_argument("--ov-bridge", default=str(HERE / "ov_bridge" / "build" / "ov_bridge"), help="브리지 실행 파일")
    p.add_argument("--ov-config", default=None,
                   help="스테레오 estimator_config.yaml 고정 (--vio-mode stereo). 기본 = 세션 크기·fps 로 자동 "
                        "(make_ov_config.ensure_config → config/<W>x<H>_<fps>/, 없거나 낡으면 만듦)")
    p.add_argument("--ov-config-mono", default=None,
                   help="mono estimator_config.yaml 고정 (--vio-mode mono). 기본 = 자동 config/<W>x<H>_<fps>/mono/")
    p.add_argument("--live-bridge-args", default="--auto-reset", help="실시간 브리지 추가 인자 (--ov auto)")
    p.add_argument("--replay-bridge-args", default="--auto-reset", help="재생용 브리지 추가 인자")
    p.add_argument("--bridge-restart-s", type=float, default=2.0, help="브리지가 죽으면 이만큼 뒤 다시 띄움 (s)")
    p.add_argument("--ov-images", choices=["rect"], default="rect",
                   help="VIO 입력 영상: rect = 정류 gray (명세 9.4, make_ov_config.py 설정과 짝). raw 는 설정과 안 맞아 제거 (REVIEW 5)")
    p.add_argument("--on-vio-reset", choices=["clear", "keep"], default="clear",
                   help="브리지 auto-reset (월드 원점이 바뀜) 때 맵을 지울지 (재생 중이면 지우기 전 조각 저장)")
    p.add_argument("--vio-mode", choices=["stereo", "mono"], default="stereo",
                   help="stereo = 두 렌즈 쌍만 VIO 에 보냄 (시각차 큰 쌍은 버림), mono = 왼쪽 렌즈만 (mono 설정으로 브리지). "
                        "섞어 보내지 않는다 (섞으면 OpenVINS TrackKLT 가 죽음)")
    p.add_argument("--vio-stereo-skew-ms", type=float, default=None,
                   help="stereo 모드: 두 렌즈 시각차가 이보다 크면 그 쌍을 VIO 에 안 보냄 "
                        "(기본 = --pair-skew-ms, 쌍 묶기보다 크면 거르는 효과가 없음 — REVIEW4 B6)")
    p.add_argument("--gyro-type", type=int, default=phone_rx.SENSOR_GYRO, help="4 = 자이로, 16 = uncalibrated")
    p.add_argument("--accel-type", type=int, default=phone_rx.SENSOR_ACCEL, help="1 = 가속도계, 35 = uncalibrated")
    p.add_argument("--voxel", type=float, default=0.03, help="복셀 크기 m")
    p.add_argument("--min-hits", type=int, default=2, help="표시할 최소 관측 수")
    p.add_argument("--depth-range", nargs=2, type=float, default=[0.3, 2.5], help="적분 깊이 범위 m (최대 3 m)")
    p.add_argument("--iters", type=int, default=4, help="실시간 FFS GRU 반복")
    p.add_argument("--scale", type=float, default=None,
                   help="FFS 입력 축소 (0,1]. 기본 자동 = 화소 수가 --ffs-max-px 를 넘으면 √(max/화소) 로 줄임 (FFS 시간 ∝ 화소 수)")
    p.add_argument("--ffs-max-px", type=int, default=640 * 480,
                   help="--scale 자동일 때 FFS 입력 최대 화소 수 (기본 640x480 = 지금 PC 부하 그대로)")
    p.add_argument("--stride", type=int, default=3, help="3D 점 화소 간격 (2~4)")
    p.add_argument("--yolo-model", default="yolo11m-seg.pt")
    p.add_argument("--yolo-conf", type=float, default=0.25)
    p.add_argument("--yolo-rotate", action="store_true", help="YOLO 입력만 시계 90° 회전 (영상이 누워 있을 때)")
    p.add_argument("--no-yolo", action="store_true")
    p.add_argument("--device", default="0")
    p.add_argument("--kf-trans", type=float, default=0.03, help="키프레임 이동 m")
    p.add_argument("--kf-rot", type=float, default=5.0, help="키프레임 회전 deg")
    p.add_argument("--kf-dt", type=float, default=0.0, help="이 시간(s) 지나면 움직임 없어도 키프레임 (0 = 끔)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8795)
    p.add_argument("--out", default=str(HERE / "runs"), help="저장 버튼 결과 폴더")
    p.add_argument("--rec-root", default=str(ROOT / "recordings"), help="가져온 녹화 폴더 위치")
    p.add_argument("--replay", default=None, metavar="DIR", help="시작하자마자 이 녹화 폴더를 재생 (이름이면 --rec-root 아래)")
    p.add_argument("--replay-out", default=None, help="재생 결과 폴더 (기본 <녹화>/recon_<시각>)")
    p.add_argument("--every-frame", action="store_true", help="재생: 키프레임 기준 없이 포즈가 있는 모든 쌍에 FFS")
    p.add_argument("--replay-iters", type=int, default=8, help="재생 FFS GRU 반복 (실시간보다 높게)")
    p.add_argument("--exit-after-replay", action="store_true", help="--replay 가 끝나면 종료")
    p.add_argument("--skip-epi", action="store_true", help="첫 쌍 정류 검증 생략")
    p.add_argument("--save-on-exit", action="store_true")
    a = p.parse_args(argv)
    a.depth_range[1] = min(a.depth_range[1], 3.0)      # 3 m 넘는 깊이는 쓰지 않는다 (baseline 15.5 mm)
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
    if a.vio_stereo_skew_ms is None:
        a.vio_stereo_skew_ms = a.pair_skew_ms                 # 두 기준을 같은 값으로 (기본 5 ms, REVIEW4 B6)
    return a


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
        self.want = dict(size=tuple(a.size), fps=int(a.app_fps), ffs_scale=a.scale)   # 실시간·녹화 세션에 쓸 값 (웹이 바꿈)
        self.boot_size = tuple(a.size)                   # 외부 브리지(--ov HOST:PORT)가 쓰는 크기로 본다 (REVIEW6 R1)
        self.vio_cfg_used = None                         # 지금 브리지가 쓰는 estimator_config.yaml (저장 때 복사)
        self.vmap = VoxelMap(a.voxel, a.min_hits)
        self.paused = False
        self.t0_ns = None
        self.last_cam_ns = -1
        self.recent = OrderedDict()                      # 카메라 t_ns → pair
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
        self.vio_skew_dropped = 0                         # stereo 모드에서 시각차가 커서 VIO 에 안 보낸 쌍
        self.skew_ms = deque(maxlen=600)                  # 최근 쌍의 두 렌즈 시각차 (ms) → 중앙값·p95 를 웹에
        self.pair_hist = deque(maxlen=64)                 # (monotonic, 프레임 합, 짝 없어 버린 합) → 최근 버린 비율 (REVIEW5 N1)
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
        self.link = phone_link.PhoneLink(a.adb, self.log, physical=",".join(a.physical), fps=a.app_fps,
                                         focus=a.app_focus, imu_us=a.app_imu_us,
                                         stream_fmt=a.app_stream, jpeg_q=a.app_jpeg_q, width=a.size[0], height=a.size[1])
        self.log(f"[link] {a.link} · phone {a.phone} · adb {self.link.adb or '없음'} ({self.link.kind or '-'})")
        self.live_vio = None
        self.vio = IdleVIO()
        self.vio_error = None
        self.imu = phone_rx.ImuSync(self.on_imu, a.gyro_type, a.accel_type)
        self.apply_geometry(tuple(a.size), a.app_fps, a.scale)

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        self.log_lines.append(line)

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

    def apply_geometry(self, size, fps=None, ffs_scale=None):
        """세션 해상도·fps 적용 (세션이 멈춘 상태에서, session_lock 안 또는 __init__ 에서 부른다).

        - 보정: calib_phone_pair.json (보정 크기) → W×H 비례 변환 (K 만, dist·R·T 그대로) → 정류맵·정류 K (rp).
        - FFS 입력 축소: ffs_scale 또는 자동 (--scale, 없으면 --ffs-max-px 기준).
        - 앱 인자 (PhoneLink width/height/fps), VIO 설정 키 (_vio_key; 다르면 다음 _ensure_live_vio 가 브리지 재시작).
        - 크기가 바뀌면 첫 쌍 정류 검증(epi)을 다시 한다. 진행 중 키프레임은 크기가 안 맞으면 버린다."""
        (W, H), fps, ffs_scale = self._norm_geometry(size, fps, ffs_scale)
        if ffs_scale is None:
            ffs_scale = calib_scale.auto_ffs_scale(W, H, self.a.ffs_max_px)
        g = self.geom
        if g is not None and g["size"] == (W, H) and g["fps"] == fps and abs(g["ffs_scale"] - ffs_scale) < 1e-9:
            return g
        if g is not None and g["size"] == (W, H):
            ng = dict(g, fps=fps, ffs_scale=ffs_scale)
        else:
            ng = calib_scale.load_geometry(self.a.calib, self.a.physical, (W, H))
            ng.update(fps=fps, ffs_scale=ffs_scale)
            self.epi = None                                   # 새 크기로 정류 검증 다시 (기준 dy 도 크기 배율)
        self.st, self.swap, self.rp = ng["st"], ng["swap"], ng["rp"]
        self.geom = ng
        self.a.size = [W, H]
        self.link.width, self.link.height, self.link.fps = W, H, fps
        rp = ng["rp"]
        self.log(f"[calib] {W}x{H} {fps} fps · {ng['note']} · f={rp['f']:.1f} cx={rp['cx']:.1f} cy={rp['cy']:.1f} "
                 f"baseline={rp['baseline_m'] * 1000:.2f} mm swap={ng['swap']} · FFS 입력 "
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
                    want=dict(size=list(w["size"]), fps=w["fps"], ffs_scale=w["ffs_scale"]),
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
                   stream_fmt=None, live_kbps=None, live_kbps_auto=False):
        """다음 실시간·녹화 세션의 크기·fps 를 정한다 (검증만, 적용은 start_session). 반환 want.
        size = "WxH" | (W,H) | W (그때 h 도 줌, web_server 의 set_stream(w, h, fps, rec_kbps=…) 호출).
        rec_kbps = 녹화 H.264 비트레이트 (앱 --ei rec_kbps, None = 앱 기본).
        stream_fmt = 실시간 형식 h264|jpeg|nv21 (앱 --es stream_fmt), live_kbps = 렌즈당 실시간 H.264 kbps
        (앱 --ei live_kbps, None/0 = 앱이 크기·fps 비례로 정함). 둘 다 다음 세션(앱 시작)부터.
        live_kbps_auto = True 면 live_kbps 는 부하 모델이 계산한 기본값 (웹 [스캔 시작]), False 면 사용자가 직접 준 값.
        크기·fps 만 바뀌고 live_kbps 가 없으면: 사용자가 직접 준 값만 유지하고, 아니면 새 크기 기준 기본값으로 다시 계산
        (REVIEW7 H4: 1280x960 의 16 Mbps 가 640x480 에 남지 않게)."""
        if stream_fmt is not None and stream_fmt not in ("h264", "jpeg", "nv21"):
            raise ValueError(f"stream_fmt {stream_fmt}")
        if h is not None and size is not None and not isinstance(size, (str, tuple, list)):
            size = (int(size), int(h))
        w = self.want
        old_geo = (tuple(w["size"]), int(w["fps"]))
        sz, f, fs = self._norm_geometry(size if size is not None else w["size"], fps if fps is not None else w["fps"],
                                        ffs_scale if ffs_scale is not None else (w["ffs_scale"] if keep_ffs else None))
        self.want = dict(size=sz, fps=f, ffs_scale=fs)
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
            self.start_replay(self.a.replay, self.a.every_frame, self.a.replay_iters, self.a.replay_out)
        elif self.a.mode != "idle":
            self.start_session(self.a.mode, start_app=self.a.start_app)

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
    def start_session(self, mode, start_app=True, size=None, fps=None, ffs_scale=None):
        """size(WxH 4:3)·fps·ffs_scale 를 주면 그 값으로 (self.want 갱신), 안 주면 self.want. 크기·fps 가 바뀌면
        정류맵·FFS 입력·앱 인자를 다시 정하고, 실시간 브리지는 그 크기·fps 설정(config/<W>x<H>_<fps>/)으로 다시 띄운다."""
        if mode not in PHONE_MODES:
            raise ValueError(f"mode {mode}")
        if size is not None or fps is not None or ffs_scale is not None:
            self.set_stream(size=size, fps=fps, ffs_scale=ffs_scale, keep_ffs=size is None)    # 잘못된 값이면 세션을 멈추기 전에 ValueError
        if mode in ("live", "both"):
            self._check_vio_size(self.want["size"])      # 고정 VIO 설정·외부 브리지와 크기가 다르면 거절 (REVIEW6 R1)
            self._ensure_depth()
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
            w = self.want
            self.apply_geometry(w["size"], w["fps"], w["ffs_scale"])
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
            self._stop_session_locked(stop_app=stop_app)
        return dict(ok=True, was=was)

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
                                         self.a.pair_skew_ms, log=self.log)
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
        self.pair_hist.clear()
        self.vio_auto_resets = 0
        # 지난 세션(또는 복원)의 카메라 창·깊이 그림·물체 상자를 새 세션에 남기지 않는다 (REVIEW5 N3)
        self.last_pair = None
        with self.panel_lock:
            self.panel = {}
        with self.pose_cv:
            self.pose_seen_ns = -1
        self._replay_kf = None
        if self.ds is not None and self.ds.yolo:
            self.ds.yolo.reset()
        if vio_reset:
            self.vio.reset()

    # ---------------- 녹화 재생 ----------------
    def resolve_rec(self, name):
        p = Path(name)
        if not p.is_absolute() and not p.is_dir():
            p = Path(self.a.rec_root) / name
        return p

    def start_replay(self, name, every_frame=False, iters=None, out=None):
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
        self._check_vio_size(rsize, replay=True)          # 고정 VIO 설정이 다른 크기용이면 거절 (REVIEW6 R1)
        if self.dstats.get("ready") and not str(self.dstats.get("error") or "").startswith("init:"):
            self.dstats["error"] = None                   # 지난 실시간 키프레임 오류가 복원을 바로 끝내지 않게 (REVIEW5 N6)
        self._ensure_depth()                              # 복원은 깊이 모델이 준비되기를 기다린다 (replay.py)
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
            # 재생은 self.want(실시간 세션 값)를 바꾸지 않는다. FFS 축소는 --scale 또는 자동
            self.apply_geometry(rsize, rfps, self.a.scale)
            self.replay_finished = False
            self.replayer = replay.Replayer(self, path, every_frame=bool(every_frame),
                                            iters=int(iters or self.a.replay_iters), out_dir=out)
            self.mode = "replay"
            self.replayer.start()
        return dict(ok=True, dir=str(path))

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
        skew_ns = abs(fR.timestamp_ns - fL.timestamp_ns)
        self.skew_ms.append(skew_ns * 1e-6)
        stereo = self.a.vio_mode == "stereo"
        if stereo and skew_ns > self.a.vio_stereo_skew_ms * 1e6:
            self.vio_skew_dropped += 1
            return None
        if timing is None:
            timing = self.timing_src or (self.receiver.timing if self.receiver else None)
        off = timing.offset_ns(fL.index, fL.timestamp_ns) if timing is not None else 0   # 그 프레임 노출 (REVIEW 6)
        # 노출 중앙 보정은 프레임 간격 안으로 제한: 엉뚱한 frame_timing 값(최신값 대체 등)이 카메라 시각을
        # 실제보다 한참 뒤로 밀면 IMU 를 기다리는 시간이 늘고 dt_cam_imu 추정도 흔들린다
        off = int(np.clip(off, 0, 1e9 / max(1, g["fps"])))
        t_ns = int(fL.timestamp_ns + off)                # 왼쪽 렌즈 물리 SENSOR_TIMESTAMP + 노출 중앙 보정
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
        rl, rr = lr.rectify_pair(fL.bgr, fR.bgr, rp)
        g0 = np.ascontiguousarray(cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY))
        g1 = np.ascontiguousarray(cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)) if stereo else None
        if stereo:
            self.vio_stereo += 1
        else:
            self.vio_mono += 1
        with self.recent_lock:
            self.recent[t_ns] = pair
            while len(self.recent) > 90:
                self.recent.popitem(last=False)
        self.vio.push_cam(t_ns, [g0, g1] if stereo else [g0])
        return t_ns

    def _pair_for(self, t_ns):
        with self.recent_lock:
            return self.recent.get(int(t_ns))           # 브리지가 t_ns 를 그대로 돌려줌

    # ---------------- 포즈 → 키프레임 ----------------
    def on_pose(self, msg):
        try:
            t_ns = int(msg.get("t_ns", -1))
        except (TypeError, ValueError):
            t_ns = -1
        try:
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
        with self.traj_lock:
            self.traj.append((t, T))
            if len(self.traj) > 50000:
                self.traj = self.traj[::2]
            self.cur_T = T
        if msg.get("stale"):                              # 상태 시각이 영상 시각과 다름 (REVIEW 9)
            return
        if self.mode == "replay":
            if not (self.replay_every or self._want_kf(T, t)):
                return
            pair = self._pair_for(t_ns)
            if pair is not None:
                self.last_kf = (t, T)
                self._replay_kf = (pair, T, t, self.vmap.epoch)
            return
        if self.paused or self.worker_busy or self.kf_queue.qsize():
            return
        if not self._want_kf(T, t):
            return
        pair = self._pair_for(t_ns)
        if pair is None:
            return
        try:
            self.kf_queue.put_nowait((pair, T, t, self.vmap.epoch))
            self.last_kf = (t, T)
        except queue.Full:
            pass

    def on_vio_event(self, msg):
        """브리지 auto-reset: 월드 원점이 바뀜 → 궤적·키프레임 기준 초기화, 맵은 --on-vio-reset (REVIEW 10)."""
        if msg.get("type") != "reset" or not msg.get("auto"):
            return
        self.vio_auto_resets += 1
        why = {"exception": f"VIO 내부 오류 ({str(msg.get('msg', ''))[:120]}) → 자동 재시작",
               "bridge_restart": "브리지 다시 띄움"}.get(msg.get("reason"), "추적 실패 → VIO 재시작")
        self.log(f"[ov] auto-reset #{self.vio_auto_resets} ({why}, 폰을 잠시 멈추면 다시 초기화)")
        with self.traj_lock:
            self.traj, self.cur_T = [], None
        self.last_kf = None
        if self.a.on_vio_reset == "clear" and self.vmap.n:
            rp = self.replayer
            if self.mode == "replay" and rp is not None and rp.out_dir is not None:
                try:
                    self.save(Path(rp.out_dir) / f"segment_{self.vio_auto_resets:02d}", include_cfg=False)
                except Exception as e:
                    self.log(f"[save] 조각 저장 실패 {e!r}")
            self.vmap.reset()
            self.objects = []

    def _depth_worker(self):
        import depth_seg
        try:
            self.ds = depth_seg.DepthSeg(self.rp, self.want_iters, self.geom["ffs_scale"], tuple(self.a.depth_range), self.a.stride,
                                         self.a.yolo_model, self.a.yolo_conf, self.a.yolo_rotate, self.a.device,
                                         self.a.no_yolo, self.log)
            self.vmap.names = self.ds.names
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

    def _process_keyframe(self, pair, T, t, epoch):
        t_start = time.perf_counter()
        fL, fR = pair
        if epoch != self.vmap.epoch:
            return                                            # 그 사이 맵 리셋 (REVIEW 8)
        g = self.geom                                         # 한 키프레임 안에서는 같은 정류맵·FFS 축소
        if (fL.bgr.shape[1], fL.bgr.shape[0]) != tuple(g["rp"]["size"]):
            return                                            # 해상도 바뀌기 전 키프레임
        r = self.ds(fL.bgr, fR.bgr, rp=g["rp"], scale=g["ffs_scale"])
        if self.paused and self.mode != "replay":
            return
        pts_w = frames.transform(T, r["pts_rect0"])
        # 물체 인스턴스 (2D 트랙 id 우선) → 점마다 전역 id. 리셋과 겹치지 않게 맵 lock 안에서 (REVIEW 8)
        inst = np.full(len(pts_w), -1, np.int32)
        di = r["det_idx"]
        with self.vmap.lock:
            if epoch != self.vmap.epoch:
                return
            for k in np.unique(di[di >= 0]):
                d = r["dets"][k]
                sel = di == k
                iid = self.vmap.tracker.associate(d["class_id"], pts_w[sel], d["track_id"], t)
                inst[sel] = iid
                d["inst"] = iid
            self.vmap.integrate(pts_w, r["rgb"], r["cls"], inst, t, epoch=epoch)
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
                     pair_skew_ms=self.a.pair_skew_ms, connected=rx is not None)
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
        vio.update(stereo_frames=self.vio_stereo, mono_frames=self.vio_mono, images="rect",
                   auto_resets=self.vio_auto_resets, vio_mode=self.a.vio_mode, bridge_restarts=self.bridge_restarts,
                   skew_dropped=self.vio_skew_dropped, skew_limit_ms=self.a.vio_stereo_skew_ms,
                   pair_skew_ms=self.a.pair_skew_ms, bridge_fails=self.bridge_fails,
                   skew_med_ms=None if sk is None else round(float(np.median(sk)), 2),
                   skew_p95_ms=None if sk is None else round(float(np.percentile(sk, 95)), 2),
                   skew_n=0 if sk is None else int(sk.size))
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
                       replay=dict(rp.progress) if rp is not None else None, last_result=self.last_result,
                       rec_root=self.a.rec_root, cams=self.cams_info())
        return dict(t=time.time(), paused=self.paused, vio=vio, phone=phone, depth=dict(self.dstats), session=session,
                    map=dict(epoch=self.vmap.epoch, version=self.vmap.version, voxels=self.vmap.n,
                             shown=self.vmap.count_shown(), voxel_m=self.vmap.voxel_m, min_hits=self.vmap.min_hits,
                             frames=self.vmap.frames),
                    objects=self.objects, epi=self.epi,
                    classes={int(k): dict(name=v, color=list(class_color(int(k)))) for k, v in names.items()},
                    log=list(self.log_lines)[-14:], stream=self.stream_info(),
                    settings=dict(depth_range=self.a.depth_range, stride=self.a.stride, iters=self.a.iters,
                                  replay_iters=self.a.replay_iters, kf_trans=self.a.kf_trans, kf_rot=self.a.kf_rot))

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
        return dict(phone=phone, phone_error=err, phone_t=self._phone_recs_t, local=local, rec_root=self.a.rec_root)

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

    LENS_NAMES = {"2": "초광각", "5": "광각"}

    def cam_jpeg(self, lens, overlay=False):
        """/api/cam.jpg — 가장 최근 수신 프레임 (센서 방향 그대로 640x480; 웹이 90° 돌려 보여 줄 수 있음).
        lens 0 = 정류 왼쪽 (렌즈 2 초광각), 1 = 오른쪽 (렌즈 5 광각). JPEG 스트림이면 받은 바이트 그대로,
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

    def _pull_and_replay(self, name, do_replay, every_frame, iters):
        prog = dict(name=name, state="가져오는 중", done=0, total=0, t0=time.time())
        self.pull_progress = prog

        def cb(done, total):
            prog.update(done=done, total=total)
        try:
            path = self.link.pull(name, self.a.rec_root, progress=cb)
            prog.update(state="완료", path=str(path))
            self._phone_recs = None
            if do_replay:
                self.start_replay(str(path), every_frame, iters)
        except Exception as e:
            prog.update(state="오류", error=repr(e))
            self.log(f"[pull] 실패: {e!r}")

    def command(self, body):
        cmd = body.get("cmd")
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
            # body: mode, start_app, size ("1280x960" 또는 [W,H], 4:3 만), fps, ffs_scale (없으면 지난 값)
            try:
                return self.start_session(body.get("mode", "live"), start_app=bool(body.get("start_app", True)),
                                          size=body.get("size"), fps=body.get("fps"), ffs_scale=body.get("ffs_scale"))
            except ValueError as e:
                return dict(ok=False, error=str(e))
        elif cmd == "stream":
            # 다음 실시간·녹화 세션의 크기·fps 만 정함 (검증; 적용은 다음 '시작')
            try:
                w = self.set_stream(size=body.get("size"), fps=body.get("fps"), ffs_scale=body.get("ffs_scale"),
                                    keep_ffs=body.get("size") is None)
            except ValueError as e:
                return dict(ok=False, error=str(e))
            return dict(ok=True, want=dict(size=list(w["size"]), fps=w["fps"], ffs_scale=w["ffs_scale"]),
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
            threading.Thread(target=self._pull_and_replay, daemon=True, name="pull",
                             args=(name, bool(body.get("replay", True)), bool(body.get("every_frame", False)),
                                   int(body.get("iters") or self.a.replay_iters))).start()
            return dict(ok=True, name=name)
        elif cmd == "replay":
            return self.start_replay(str(body.get("name") or body.get("dir") or ""), bool(body.get("every_frame", False)),
                                     int(body.get("iters") or self.a.replay_iters))
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
