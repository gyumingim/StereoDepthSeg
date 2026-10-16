"""OpenVINS 브리지 클라이언트 — scan3d/ov_bridge/PROTOCOL.md 형식.

전송: TCP ("127.0.0.1:7600", 브리지 `--tcp`) 또는 파이프 ("pipe:<명령줄>", 브리지 `--stdio`).
요청 = 헤더 <III (magic 0x4F564252 "OVBR", type, payload_len) + payload, little-endian:
  1 IMU    <q3d3d  t_ns(int64, elapsedRealtimeNanos), gx,gy,gz rad/s, ax,ay,az m/s² (중력 포함)   56 B
  2 STEREO <qIIB3x t_ns, w, h, n_cams(1|2) + n_cams × w·h gray (cam0 = 정류 왼쪽 rect0 먼저)
  3 RESET, 4 QUIT  (payload 없음)
응답 = JSON 한 줄씩, "type": ready | pose | reset | error.
  pose: t_ns(입력 그대로 = 키프레임 키), t, status(init|tracking|lost), init, T_world_cam0(16)|null,
        q(Hamilton, 월드←IMU), p, v, q_GtoI_jpl, R_ItoC0, p_IinC0, dt_cam_imu, n_slam, n_msckf, ms, cam_dropped, imu_dropped
규칙: IMU 는 버리지 않는다. 카메라는 마지막 IMU 시각 이후 것만 보내고(브리지도 같은 규칙으로 큐잉),
응답 대기 중인 카메라(보낸 t_ns 집합, lock 아래 관리)가 max_inflight 이상이면 새 카메라를 버린다.
카메라 대기 큐(IMU 를 기다리는 카메라)는 개수 max_pending(기본 30) 과 나이(최신 카메라보다 max_cam_wait_s 이상 오래됨)로만 자른다
(REVIEW 4: IMU 가 2 프레임 넘게 늦어도 VIO 입력이 끊기지 않게).
콜백(on_pose, on_event) 예외는 로그만 남기고 읽기를 계속한다 (REVIEW 1).
"""
from collections import deque, OrderedDict
import json
import shlex
import socket
import struct
import subprocess
import threading
import time

MAGIC = 0x4F564252
T_IMU, T_CAM, T_RESET, T_QUIT = 1, 2, 3, 4
HDR = struct.Struct("<III")
IMU = struct.Struct("<q3d3d")
CAM = struct.Struct("<qIIB3x")
INFLIGHT_STALE_S = 2.0          # 이만큼 pose 가 안 오면 브리지가 버린 것으로 본다 (RESET/auto-reset 큐 비움 등)


class OVClient:
    def __init__(self, addr, on_pose=None, log=print, max_inflight=3, max_pending_cams=30, on_event=None,
                 max_cam_wait_s=1.0, stderr_path=None, connect_timeout_s=30.0):
        self.addr, self.on_pose, self.on_event, self.log = addr, on_pose, on_event, log
        self.stderr_path = stderr_path       # pipe: 브리지 stderr(OpenVINS 로그) 파일, None 이면 그대로 상속
        self.connect_timeout_s = connect_timeout_s
        self.max_inflight, self.max_pending, self.max_cam_wait_ns = max_inflight, max_pending_cams, int(max_cam_wait_s * 1e9)
        self.lock = threading.Condition()
        self.imu_q = deque()
        self.cam_q = deque()
        self.ctrl_q = deque()
        self.last_imu_ns = -(1 << 62)
        self.inflight = OrderedDict()        # 보낸 카메라 t_ns → 보낸 시각 (monotonic), self.lock 아래에서만 바꾼다 (REVIEW 7)
        self.cams_sent = self.poses_got = 0
        self._bdrop_prev = None              # 브리지 cam_dropped 는 프로세스 누적값 (RESET 해도 안 줄어듦)
        self.cams_dropped = 0
        self.imu_sent = 0
        self.poses = OrderedDict()           # t_ns → pose (최근 600개)
        self.latest = None
        self.ready = None
        self.events = deque(maxlen=20)
        self.error = None                    # 연결/프로세스가 죽었을 때만 (scan3d 가 이것으로 브리지를 다시 띄움)
        self.bridge_msg = None               # 브리지 "error" 줄 (잘못된 레코드 등, 연결은 살아 있음)
        self.exceptions = 0                  # 브리지가 OpenVINS 예외를 잡고 내부 리셋한 횟수
        self.cb_errors = 0
        self.stopped = False
        self.proc = None
        self.sock = None
        self._connect()
        if not self.proc:
            self.ctrl_q.append(T_RESET)      # TCP: 브리지 상태가 이전 세션 것일 수 있음 (PROTOCOL.md)
        threading.Thread(target=self._writer, daemon=True, name="ov-writer").start()
        threading.Thread(target=self._reader, daemon=True, name="ov-reader").start()

    # ---- 연결 ----
    def _connect(self):
        if self.addr.startswith("pipe:"):
            cmd = shlex.split(self.addr[5:])
            err = open(self.stderr_path, "ab") if self.stderr_path else None
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, bufsize=0)
            if err is not None:
                err.close()                  # 자식이 복사본을 가짐
            self._w = self.proc.stdin.write
            self._flushw = self.proc.stdin.flush
            self._rfile = self.proc.stdout
            self.log(f"[ov] 브리지 프로세스 시작: {cmd}")
            return
        host, _, port = self.addr.rpartition(":")
        t_end = time.monotonic() + self.connect_timeout_s
        while True:
            try:
                s = socket.create_connection((host or "127.0.0.1", int(port)), timeout=5)
                break
            except OSError as e:
                if time.monotonic() > t_end:
                    raise RuntimeError(f"OpenVINS 브리지 {self.addr} 접속 실패: {e}")
                time.sleep(0.5)
        s.settimeout(None)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        self._w = s.sendall
        self._flushw = lambda: None
        self._rfile = s.makefile("rb")
        self.log(f"[ov] 브리지 접속 {self.addr}")

    # ---- 입력 ----
    def push_imu(self, t_ns, w, a):
        with self.lock:
            self.imu_q.append((t_ns, IMU.pack(int(t_ns), *w, *a)))
            self.lock.notify_all()

    def push_cam(self, t_ns, grays):
        """grays: [cam0] 또는 [cam0, cam1], 같은 크기 uint8 HxW (C 연속)."""
        h, w = grays[0].shape[:2]
        payload = CAM.pack(int(t_ns), w, h, len(grays)) + b"".join(g.tobytes() for g in grays)
        with self.lock:
            self.cam_q.append((int(t_ns), payload))
            newest = self.cam_q[-1][0]
            while self.cam_q and (len(self.cam_q) > self.max_pending
                                  or newest - self.cam_q[0][0] > self.max_cam_wait_ns):
                self.cam_q.popleft(); self.cams_dropped += 1      # IMU 가 너무 오래 안 옴 → 오래된 것부터
            self.lock.notify_all()

    def reset(self):
        with self.lock:
            self.ctrl_q.append(T_RESET)
            self.cam_q.clear()
            self.poses.clear()
            self.latest = None
            self.lock.notify_all()

    def pending(self):
        """아직 pose 를 못 받은 카메라 수 (큐 대기 + 응답 대기)."""
        with self.lock:
            return len(self.cam_q) + len(self.inflight)

    def dead(self):
        """브리지 연결이 끊겼거나 (pipe 면) 프로세스가 끝났으면 True."""
        if self.stopped:
            return False
        if self.error is not None:
            return True
        return self.proc is not None and self.proc.poll() is not None

    def close(self):
        if self.proc is not None and self.proc.poll() is not None:     # 이미 죽은 브리지: 기다릴 것 없음
            self.stopped = True
            return
        with self.lock:
            self.ctrl_q.append(T_QUIT)
            self.lock.notify_all()
        time.sleep(0.3)
        self.stopped = True
        try:
            if self.proc:
                self.proc.wait(timeout=3)
            else:
                self.sock.close()
        except Exception:
            if self.proc:
                self.proc.kill()

    # ---- 스레드 ----
    def _send(self, typ, payload=b""):
        self._w(HDR.pack(MAGIC, typ, len(payload)) + payload)

    def _prune_inflight(self):
        """lock 안에서 호출. 오래된 응답 대기 항목 제거 후 개수."""
        now = time.monotonic()
        while self.inflight:
            t, ts = next(iter(self.inflight.items()))
            if now - ts <= INFLIGHT_STALE_S:
                break
            self.inflight.popitem(last=False)
        return len(self.inflight)

    def _cam_ready(self):
        return bool(self.cam_q) and self.cam_q[0][0] <= self.last_imu_ns

    def _writer(self):
        try:
            while not self.stopped:
                with self.lock:
                    while not (self.imu_q or self.ctrl_q or self._cam_ready()) and not self.stopped:
                        self.lock.wait(0.2)
                    ctrl = list(self.ctrl_q); self.ctrl_q.clear()
                    imus = list(self.imu_q); self.imu_q.clear()
                for c in ctrl:
                    self._send(c)
                    if c == T_RESET:
                        with self.lock:
                            self.inflight.clear()
                    if c == T_QUIT:
                        self._flushw()
                        return
                if imus:
                    hdr = HDR.pack(MAGIC, T_IMU, IMU.size)
                    self._w(b"".join(hdr + p for _, p in imus))
                    self.imu_sent += len(imus)
                    with self.lock:
                        self.last_imu_ns = imus[-1][0]
                while True:
                    with self.lock:
                        if not self._cam_ready():
                            break
                        t, payload = self.cam_q.popleft()
                        if self._prune_inflight() >= self.max_inflight:
                            self.cams_dropped += 1             # 브리지가 밀림 → 카메라만 버림
                            continue
                        self.inflight[t] = time.monotonic()    # 보내기 전에 넣는다 (pose 가 먼저 와도 맞게)
                    self._send(T_CAM, payload)
                    self.cams_sent += 1
                self._flushw()
        except Exception as e:
            if not self.stopped:
                self.error = f"writer: {e}"
                self.log(f"[ov] 쓰기 오류: {e}")

    def _callback(self, fn, msg):
        try:
            fn(msg)
        except Exception as e:                             # 콜백 오류로 읽기 스레드가 죽지 않게 (REVIEW 1)
            self.cb_errors += 1
            if self.cb_errors <= 5 or self.cb_errors % 100 == 0:
                self.log(f"[ov] 콜백 오류 #{self.cb_errors}: {e!r}")

    def _reader(self):
        try:
            for line in self._rfile:
                if self.stopped:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    self.events.append(line[:200].decode("utf-8", "replace"))
                    continue
                if not isinstance(msg, dict):
                    continue
                typ = msg.get("type", "pose" if "t_ns" in msg else "?")
                if typ != "pose":
                    if typ == "ready":
                        self.ready = msg
                    elif typ == "error":
                        # 레코드 하나가 잘못된 것뿐 (연결은 살아 있음) → self.error(= 연결 죽음, 재시작 기준)에 넣지 않는다
                        self.bridge_msg = f"bridge: {msg.get('msg')}"
                    elif typ == "reset":
                        if msg.get("reason") == "exception":
                            self.exceptions += 1
                        with self.lock:
                            self.poses.clear(); self.latest = None
                            if msg.get("auto"):          # auto-reset: 브리지 큐의 카메라가 pose 없이 사라짐
                                self.inflight.clear()
                    self.events.append(msg)
                    self.log(f"[ov] {json.dumps(msg)[:200]}")
                    if self.on_event:
                        self._callback(self.on_event, msg)
                    continue
                try:
                    t_ns = int(msg["t_ns"])
                except (KeyError, TypeError, ValueError):
                    self.events.append(f"pose 줄 t_ns 없음: {str(msg)[:120]}")
                    continue
                # REVIEW 9: 브리지가 상태 시각(t_state, 브리지 초)을 주면 그 영상 시각과 맞는지 본다
                ts, tc = msg.get("t_state"), msg.get("t")
                if isinstance(ts, (int, float)) and isinstance(tc, (int, float)):
                    dtc = float(msg.get("dt_cam_imu") or 0.0)          # 상태 시각이 카메라/IMU 시계 어느 쪽이어도
                    if abs(ts - tc) > 1e-3 and abs(ts - (tc + dtc)) > 1e-3:
                        msg["stale"] = True
                self.poses_got += 1
                cum = msg.get("cam_dropped")
                with self.lock:
                    self.inflight.pop(t_ns, None)
                    try:
                        cum = int(cum or 0)
                    except (TypeError, ValueError):
                        cum = self._bdrop_prev or 0
                    if self._bdrop_prev is not None and cum > self._bdrop_prev:
                        for _ in range(min(cum - self._bdrop_prev, len(self.inflight))):
                            self.inflight.popitem(last=False)     # 브리지는 가장 오래된 카메라부터 버림
                    self._bdrop_prev = cum
                    self.poses[t_ns] = msg
                    while len(self.poses) > 600:
                        self.poses.popitem(last=False)
                    self.latest = msg
                    self.lock.notify_all()
                if self.on_pose:
                    self._callback(self.on_pose, msg)
            if not self.stopped:
                self.error = "bridge closed"
        except Exception as e:
            if not self.stopped:
                self.error = f"reader: {e}"
                self.log(f"[ov] 읽기 오류: {e}")

    def stats(self):
        L = self.latest or {}
        with self.lock:
            n_inflight = len(self.inflight)
            n_wait = len(self.cam_q)
        return dict(addr=self.addr, init=bool(L.get("init")), status=L.get("status"), mono=L.get("mono"),
                    t=L.get("t"), n_slam=L.get("n_slam"), n_msckf=L.get("n_msckf"), ms=L.get("ms"),
                    dt_cam_imu=L.get("dt_cam_imu"), imu_sent=self.imu_sent, cams_sent=self.cams_sent,
                    poses=self.poses_got, cams_dropped=self.cams_dropped, bridge_cam_dropped=L.get("cam_dropped"),
                    bridge_imu_dropped=L.get("imu_dropped"), inflight=n_inflight, cam_wait=n_wait,
                    bridge_cam_rejected=L.get("cam_rejected"), bridge_exceptions=self.exceptions,
                    imu_held=L.get("imu_held"), bridge_pid=self.proc.pid if self.proc else None,
                    bridge_log=self.stderr_path, bridge_msg=self.bridge_msg,
                    ready=self.ready, error=self.error,
                    events=[e if isinstance(e, str) else json.dumps(e)[:200] for e in list(self.events)[-5:]])


class NullVIO:
    """--ov none: VIO 없이 깊이·분할만 확인 (고정 포즈)."""

    def __init__(self, on_pose=None, log=print):
        self.on_pose, self.log = on_pose, log
        self.latest = None
        self.max_inflight = self.max_pending = 0

    def push_imu(self, t_ns, w, a):
        pass

    def push_cam(self, t_ns, grays):
        self.latest = {"type": "pose", "t_ns": int(t_ns), "t": t_ns * 1e-9, "init": True, "status": "static",
                       "static": True}
        if self.on_pose:
            try:
                self.on_pose(self.latest)
            except Exception as e:
                self.log(f"[ov] 콜백 오류: {e!r}")

    def reset(self):
        pass

    def pending(self):
        return 0

    def close(self):
        pass

    def stats(self):
        return dict(addr="none", init=True, status="static")
