#!/usr/bin/env python3
"""scan3d — 실시간 3D 복셀 맵 스캐너 (WSL/우분투, venv_ffs).

폰(렌즈 2+5 NV21 + IMU) ──릴레이/adb forward──▶ 수신 ─┬─ IMU 동기화 ───────────────┐
                                                   └─ 프레임 쌍 → 정류 gray ────┴▶ OpenVINS 브리지 → 포즈
                                            키프레임 쌍 + 포즈 → FFS 깊이 + YOLO-seg → 복셀 맵 (클래스·물체)
녹화 재생: 폰 녹화 폴더(영상 + 센서 CSV) → 같은 파이프라인 (replay.py, --replay DIR).
웹 뷰어: http://localhost:8795 (WSL 0.0.0.0:8795, Windows localhost 포워딩) — 모드 선택·시작·정지·녹화 목록·가져와서 복원.

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
  venv_ffs/bin/python scan3d/scan3d.py --phone listen:8766 --ov 127.0.0.1:7600
  venv_ffs/bin/python scan3d/scan3d.py --ov none        # VIO 없이 깊이·분할만 (고정 포즈)
  venv_ffs/bin/python scan3d/scan3d.py --mode idle --replay recordings/rec_20261016_120000 --every-frame
"""
import argparse
from collections import OrderedDict, deque
import json
import queue
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
from lib_phone_depth import load_phone_calibration, depth_colormap   # noqa: E402

import frames                   # noqa: E402
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
    p.add_argument("--size", nargs=2, type=int, default=[640, 480])
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
    p.add_argument("--app-fps", type=int, default=30)
    p.add_argument("--app-imu-us", type=int, default=2500)
    p.add_argument("--app-focus", type=float, default=1.0)
    p.add_argument("--restart-app", action="store_true", default=True,
                   help="폰 쪽이 연달아 바로 끊기면 앱 다시 시작 (adb 있을 때, REVIEW 11)")
    p.add_argument("--pair-skew-ms", type=float, default=10.0, help="쌍 묶기 허용 시각차")
    p.add_argument("--ov", default="127.0.0.1:7600", help="실시간 OpenVINS 브리지 HOST:PORT | pipe:<명령> | none")
    p.add_argument("--ov-bridge", default=str(HERE / "ov_bridge" / "build" / "ov_bridge"), help="재생용 브리지 실행 파일")
    p.add_argument("--ov-config", default=str(HERE / "config" / "estimator_config.yaml"), help="재생용 estimator_config.yaml")
    p.add_argument("--replay-bridge-args", default="--auto-reset", help="재생용 브리지 추가 인자")
    p.add_argument("--ov-images", choices=["rect"], default="rect",
                   help="VIO 입력 영상: rect = 정류 gray (명세 9.4, make_ov_config.py 설정과 짝). raw 는 설정과 안 맞아 제거 (REVIEW 5)")
    p.add_argument("--on-vio-reset", choices=["clear", "keep"], default="clear",
                   help="브리지 auto-reset (월드 원점이 바뀜) 때 맵을 지울지 (재생 중이면 지우기 전 조각 저장)")
    p.add_argument("--vio-stereo-skew-ms", type=float, default=1.0, help="두 렌즈 시각차가 이하면 스테레오, 넘으면 mono")
    p.add_argument("--gyro-type", type=int, default=phone_rx.SENSOR_GYRO, help="4 = 자이로, 16 = uncalibrated")
    p.add_argument("--accel-type", type=int, default=phone_rx.SENSOR_ACCEL, help="1 = 가속도계, 35 = uncalibrated")
    p.add_argument("--voxel", type=float, default=0.03, help="복셀 크기 m")
    p.add_argument("--min-hits", type=int, default=2, help="표시할 최소 관측 수")
    p.add_argument("--depth-range", nargs=2, type=float, default=[0.3, 2.5], help="적분 깊이 범위 m (최대 3 m)")
    p.add_argument("--iters", type=int, default=4, help="실시간 FFS GRU 반복")
    p.add_argument("--scale", type=float, default=1.0, help="FFS 입력 축소 (0,1]")
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
    a.stride = int(np.clip(a.stride, 1, 8))
    if a.link == "auto":
        a.link = "relay" if phone_link.is_wsl() else "native"
    if a.phone is None:
        a.phone = "listen:8766" if a.link == "relay" else "127.0.0.1:8765"
    if a.start_app is None:
        a.start_app = a.link == "native"
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
        st, swap = load_phone_calibration(a.calib, "0", a.physical, tuple(a.size))
        self.st, self.swap = st, swap
        cal = [dict(K=np.array(st[f"K{i}"]), dist=np.array(st[f"dist{i}"]), image_size=st["image_size"]) for i in (1, 2)]
        self.rp = lr.rectify_maps(cal[0], cal[1], np.array(st["R"]), np.array(st["T"]), alpha=0.0)
        self.log(f"[calib] f={self.rp['f']:.1f} cx={self.rp['cx']:.1f} cy={self.rp['cy']:.1f} "
                 f"baseline={self.rp['baseline_m'] * 1000:.2f} mm swap={swap}")
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
                           error=None, iters=None)
        self.kf_times = deque(maxlen=20)
        self.panel = {}
        self.panel_lock = threading.Lock()
        self.objects, self.objects_t = [], 0.0
        self.epi = None
        self.vio_mono = self.vio_stereo = 0
        self.vio_auto_resets = 0
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
                                         focus=a.app_focus, imu_us=a.app_imu_us)
        self.log(f"[link] {a.link} · phone {a.phone} · adb {self.link.adb or '없음'} ({self.link.kind or '-'})")
        self.live_vio = None
        self.vio = IdleVIO()
        self.vio_error = None
        self.imu = phone_rx.ImuSync(self.on_imu, a.gyro_type, a.accel_type)

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        self.log_lines.append(line)

    # ---------------- 시작 ----------------
    def start(self):
        self.http = web_server.serve(self, self.a.host, self.a.port)
        self.log(f"[web] http://localhost:{self.a.port}  (WSL {self.a.host}:{self.a.port})")
        threading.Thread(target=self._depth_worker, daemon=True, name="depth").start()
        threading.Thread(target=self._vio_feeder, daemon=True, name="vio-feed").start()
        if self.a.replay:
            self.start_replay(self.a.replay, self.a.every_frame, self.a.replay_iters, self.a.replay_out)
        elif self.a.mode != "idle":
            self.start_session(self.a.mode, start_app=self.a.start_app)

    # ---------------- 세션 (모드) ----------------
    def start_session(self, mode, start_app=True):
        if mode not in PHONE_MODES:
            raise ValueError(f"mode {mode}")
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
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
                    return
                self._reset_pipeline(vio_reset=True)
                self.timing_src = None
                self._conn_loop(stop)
        except Exception as e:
            self.log(f"[session] 오류: {e!r}")

    def _ensure_live_vio(self):
        if self.a.ov == "none":
            if not isinstance(self.live_vio, ov_client.NullVIO):
                self.live_vio = ov_client.NullVIO(self.on_pose, self.log)
        elif self.live_vio is None or getattr(self.live_vio, "error", None):
            if self.live_vio is not None:
                try:
                    self.live_vio.close()
                except Exception:
                    pass
            try:
                self.live_vio = ov_client.OVClient(self.a.ov, self.on_pose, self.log, on_event=self.on_vio_event)
                self.vio_error = None
            except Exception as e:
                self.vio_error = repr(e)
                self.live_vio = None
                self.log(f"[ov] 브리지 연결 실패: {e!r} (run_scan.sh 로 브리지를 띄우거나 --ov none)")
                self.vio = IdleVIO()
                return False
        self.vio = self.live_vio
        return True

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
            rx = phone_rx.StreamReceiver(sock, self.on_sensor, self.on_pair, 2, tuple(self.a.size),
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
        self.vio_auto_resets = 0
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
        with self.session_lock:
            self._stop_session_locked(stop_app=False)
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
        bridge, cfg = Path(self.a.ov_bridge), Path(self.a.ov_config)
        if not bridge.is_file():
            raise FileNotFoundError(f"브리지 실행 파일 없음: {bridge} (--ov-bridge, 또는 --ov none)")
        if not cfg.is_file():
            raise FileNotFoundError(f"OpenVINS 설정 없음: {cfg} (make_ov_config.py)")
        cmd = f"pipe:{shlex.quote(str(bridge))} {shlex.quote(str(cfg))} --stdio {self.a.replay_bridge_args}"
        Path(self.a.out).mkdir(parents=True, exist_ok=True)
        errlog = Path(self.a.out) / f"ov_bridge_replay_{time.strftime('%Y%m%d-%H%M%S')}.log"
        return ov_client.OVClient(cmd.strip(), self.on_pose, self.log, max_inflight=1 << 30, max_pending_cams=1 << 30,
                                  on_event=self.on_vio_event, max_cam_wait_s=1e9, stderr_path=str(errlog))

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
        if self.t0_ns is None:
            if isinstance(self.vio, ov_client.NullVIO):
                self.t0_ns = fL.timestamp_ns
            else:
                return None                                       # IMU 가 먼저 와야 시각 기준이 생김
        if timing is None:
            timing = self.timing_src or (self.receiver.timing if self.receiver else None)
        off = timing.offset_ns(fL.index, fL.timestamp_ns) if timing is not None else 0   # 그 프레임 노출 (REVIEW 6)
        t_ns = int(fL.timestamp_ns + off)                # 왼쪽 렌즈 물리 SENSOR_TIMESTAMP + 노출 중앙 보정
        if t_ns <= self.last_cam_ns:
            return None
        self.last_cam_ns = t_ns
        if self.epi is None and not self.a.skip_epi:
            rl, rr = lr.rectify_pair(fL.bgr, fR.bgr, self.rp)
            self.epi = lr.epipolar_check(rl, rr)
            ok = self.epi.get("status") != "ok" or (self.epi["dy_med"] < 1.0 and self.epi["inlier_frac"] >= 0.7)
            self.log(f"[epi] {self.epi.get('status')} dy_med={self.epi.get('dy_med')} inlier={self.epi.get('inlier_frac')}"
                     + ("" if ok else "  ← 정류 검증 실패: 보정/초점(focus_diopters 1.0) 확인"))
        stereo = abs(fR.timestamp_ns - fL.timestamp_ns) <= self.a.vio_stereo_skew_ms * 1e6
        rl, rr = lr.rectify_pair(fL.bgr, fR.bgr, self.rp)
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
        self.log(f"[ov] auto-reset #{self.vio_auto_resets} (추적 실패 → VIO 재시작, 폰을 잠시 멈추면 다시 초기화)")
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
            self.ds = depth_seg.DepthSeg(self.rp, self.want_iters, self.a.scale, tuple(self.a.depth_range), self.a.stride,
                                         self.a.yolo_model, self.a.yolo_conf, self.a.yolo_rotate, self.a.device,
                                         self.a.no_yolo, self.log)
            self.vmap.names = self.ds.names
            self.ds.warmup()
            self.dstats["ready"] = True
            self.dstats["iters"] = self.ds.iters
        except Exception as e:
            self.dstats["error"] = f"init: {e!r}"
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
        r = self.ds(fL.bgr, fR.bgr)
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
                     connected=rx is not None)
        vio = self.vio.stats()
        vio.update(stereo_frames=self.vio_stereo, mono_frames=self.vio_mono, images="rect",
                   auto_resets=self.vio_auto_resets)
        if self.vio_error and not vio.get("error"):
            vio["error"] = self.vio_error
        with self.traj_lock:
            T = self.cur_T
        vio["T_world_rect0"] = None if T is None else T.reshape(-1).round(5).tolist()
        names = self.vmap.names or {}
        rp = self.replayer
        session = dict(mode=self.mode, link=self.a.link, phone_spec=self.a.phone, adb=self.link.info(),
                       app=self.app_status, rec=self.rec_status, pull=self.pull_progress,
                       replay=dict(rp.progress) if rp is not None else None, last_result=self.last_result,
                       rec_root=self.a.rec_root)
        return dict(t=time.time(), paused=self.paused, vio=vio, phone=phone, depth=dict(self.dstats), session=session,
                    map=dict(epoch=self.vmap.epoch, version=self.vmap.version, voxels=self.vmap.n,
                             shown=self.vmap.count_shown(), voxel_m=self.vmap.voxel_m, min_hits=self.vmap.min_hits,
                             frames=self.vmap.frames),
                    objects=self.objects, epi=self.epi,
                    classes={int(k): dict(name=v, color=list(class_color(int(k)))) for k, v in names.items()},
                    log=list(self.log_lines)[-14:],
                    settings=dict(depth_range=self.a.depth_range, stride=self.a.stride, iters=self.a.iters,
                                  replay_iters=self.a.replay_iters, kf_trans=self.a.kf_trans, kf_rot=self.a.kf_rot))

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
            return self.start_session(body.get("mode", "live"), start_app=bool(body.get("start_app", True)))
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
        cfg = HERE / "config"
        if include_cfg and cfg.is_dir():
            shutil.copytree(cfg, d / "vio_config", dirs_exist_ok=True)
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
