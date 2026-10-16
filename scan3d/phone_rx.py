"""폰 스트림 수신 (WSL/리눅스 쪽). 앱 시작(adb)은 phone_link.py(우분투 직접·WSL interop) 또는 Windows 릴레이가 한다.

- 프레임은 H.264("STEH", 앱 기본) · JPEG("STEJ") · NV21("STER") — 모두 lps.read_frame 이 같은 BGR 로 풀어 준다 (SPEC 1.2).
  H.264 는 접속마다 새 lps.H264Decoders(렌즈별 PyAV 디코더, self.h264) → 재접속하면 키프레임부터 다시 ("keyframe <렌즈>" 요청).
- lib_phone_stereo.PhoneReceiver 를 상속해 그대로 재사용 (역행 timestamp 폐기, 쌍 skew, stats()).
  _run 만 덮어써서 센서·쌍을 **도착 순서대로** 콜백으로 넘긴다 (원본은 최신 쌍 폴링만 가능).
  콜백 예외는 로그만 남기고 수신을 계속한다 (REVIEW 2). 그 밖의 예외는 self.error 에 남겨 재접속하게 한다.
- FrameTimingIndex: frame_timing 패킷(논리 ts + 물리 오프셋)으로 **그 프레임의** 노출·롤링셔터 값을 찾는다 (REVIEW 6).
- ImuSync: 자이로 시각 기준으로 가속도를 선형 보간해 (t, gyro, accel) 쌍을 만든다 (명세 2.3).
"""
from collections import deque
import bisect
import socket
import subprocess
import time

import lib_phone_stereo as lps

SENSOR_ACCEL, SENSOR_GYRO = 1, 4
SENSOR_GYRO_UNCAL, SENSOR_ACCEL_UNCAL = 16, 35
SENSOR_FRAME_TIMING = lps.SENSOR_FRAME_TIMING
lps.SENSOR_NAMES.setdefault(SENSOR_ACCEL, "accelerometer")
lps.SENSOR_NAMES.setdefault(SENSOR_GYRO_UNCAL, "gyroscope_uncalibrated")
lps.SENSOR_NAMES.setdefault(SENSOR_ACCEL_UNCAL, "accelerometer_uncalibrated")


def default_gateway():
    """WSL(NAT) 에서 Windows 호스트 IP = 기본 게이트웨이 (resolv.conf 의 10.255.255.254 는 DNS 터널이라 안 됨)."""
    try:
        out = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True, timeout=3).stdout
        parts = out.split()
        return parts[parts.index("via") + 1]
    except Exception:
        return "127.0.0.1"


def open_phone_socket(spec, log=print, timeout_s=60.0, stop=None):
    """spec: "HOST:PORT" (connect, HOST 생략 시 기본 게이트웨이) 또는 "listen:PORT" (역방향 릴레이가 접속해 옴 → b"G" 보냄).

    stop: threading.Event — 세워지면 대기를 그만두고 None 을 돌려준다 (Ctrl+C·모드 전환이 막히지 않게, REVIEW 3).
    listen 은 1 s 간격으로 stop 을 확인하며 무한 대기, connect 는 timeout_s 뒤 RuntimeError (0 이하 = 무한).
    """
    stopped = (lambda: stop is not None and stop.is_set())
    if spec.startswith("listen:"):
        port = int(spec.split(":", 1)[1])
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("0.0.0.0", port))
            srv.listen(1)
            srv.settimeout(1.0)
            log(f"[phone] 0.0.0.0:{port} 에서 릴레이 접속 대기")
            while True:
                if stopped():
                    return None
                try:
                    sock, addr = srv.accept()
                    break
                except socket.timeout:
                    continue
        finally:
            srv.close()
        sock.settimeout(5.0)
        sock.sendall(b"G")                 # 릴레이에게 폰 쪽 연결 시작 신호 (명세 8.3)
        log(f"[phone] 릴레이 접속 {addr}")
    else:
        host, _, port = spec.rpartition(":")
        host = host or default_gateway()
        t_end = time.monotonic() + timeout_s if timeout_s and timeout_s > 0 else float("inf")
        while True:
            if stopped():
                return None
            try:
                sock = socket.create_connection((host, int(port)), timeout=5)
                break
            except OSError as e:
                if time.monotonic() > t_end:
                    raise RuntimeError(f"폰 {host}:{port} 접속 실패: {e}")
                log(f"[phone] {host}:{port} 접속 재시도 ({e})")
                if stop is not None:
                    stop.wait(1.0)
                else:
                    time.sleep(1.0)
        log(f"[phone] 접속 {host}:{port}")
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    return sock


class FrameTimingIndex:
    """frame_timing (센서 타입 100) → 물리 렌즈별 노출 시각 사전.

    패킷: timestamp = 논리 SENSOR_TIMESTAMP, values = [물리0 오프셋 ms, 물리1 오프셋 ms, (물리2), 노출 ms, 주기 ms, rs skew ms].
    프레임 timestamp 는 그 렌즈의 물리 SENSOR_TIMESTAMP (= 논리 ts + 오프셋) 이므로 같은 값으로 찾는다.
    """

    def __init__(self, n_streams=2, keep=300, tol_ns=2_000_000):
        self.n, self.keep, self.tol = n_streams, keep, tol_ns
        self.ts = [[] for _ in range(n_streams)]          # 정렬된 물리 ts
        self.val = [dict() for _ in range(n_streams)]     # 물리 ts → (exp_ms, rs_ms)
        self.latest = None

    def add(self, logical_ts, values):
        v = list(values)
        if len(v) < 3 + 1:
            return
        exp_ms, rs_ms = float(v[-3]), float(v[-1])
        if not (0 <= exp_ms < 200 and 0 <= rs_ms < 100):
            return
        self.latest = (exp_ms, rs_ms)
        offs = v[:-3]
        for i in range(min(self.n, len(offs))):
            o = offs[i]
            if o != o:                                    # NaN = 그 렌즈 물리 결과 없음
                o = 0.0
            self.put(i, int(logical_ts + round(float(o) * 1e6)), exp_ms, rs_ms)

    def put(self, stream, t_phys, exp_ms, rs_ms):
        """물리 ts 하나의 노출·skew (녹화 frames CSV 에 노출 열이 있을 때 직접 넣음)."""
        i, t = stream, int(t_phys)
        if not (0 <= i < self.n):
            return
        self.latest = (exp_ms, rs_ms)
        lst = self.ts[i]
        if lst and t <= lst[-1]:
            k = bisect.bisect_left(lst, t)
            if not (k < len(lst) and lst[k] == t):
                lst.insert(k, t)
        else:
            lst.append(t)
        self.val[i][t] = (exp_ms, rs_ms)
        while len(lst) > self.keep:
            self.val[i].pop(lst.pop(0), None)

    def lookup(self, stream, t_ns):
        """(exp_ms, rs_ms) — 그 프레임 값 (±tol), 없으면 최신값, 그것도 없으면 None."""
        if 0 <= stream < self.n:
            lst = self.ts[stream]
            k = bisect.bisect_left(lst, t_ns)
            best = None
            for j in (k - 1, k):
                if 0 <= j < len(lst) and abs(lst[j] - t_ns) <= self.tol:
                    if best is None or abs(lst[j] - t_ns) < abs(best - t_ns):
                        best = lst[j]
            if best is not None:
                return self.val[stream][best]
        return self.latest

    def offset_ns(self, stream, t_ns):
        """노출 중앙 보정: 0.5·(노출 + 롤링셔터 skew)."""
        r = self.lookup(stream, t_ns)
        return 0 if r is None else int(0.5 * (r[0] + r[1]) * 1e6)


class StreamReceiver(lps.PhoneReceiver):
    """PhoneReceiver + 콜백. on_sensor(SensorSample), on_pair(tuple[PhoneFrame, PhoneFrame]) 는 수신 스레드에서 불린다 → 빨리 끝낼 것."""

    def __init__(self, sock, on_sensor, on_pair, count=2, size=(640, 480), max_skew_ms=10, log=print):
        super().__init__(sock, count, size, max_skew_ms)
        self.on_sensor, self.on_pair, self.log = on_sensor, on_pair, log
        self.frame_timing = None            # 최신 frame_timing 값 (명세 1.4)
        self.timing = FrameTimingIndex(count)
        self.cb_errors = 0
        self.t_connect = time.monotonic()

    def _cb(self, fn, arg):
        try:
            fn(arg)
        except Exception as e:                          # 콜백 오류로 수신 스레드가 죽지 않게 (REVIEW 2)
            self.cb_errors += 1
            if self.cb_errors <= 5 or self.cb_errors % 100 == 0:
                self.log(f"[phone] 콜백 오류 #{self.cb_errors}: {e!r}")

    def _run(self):
        try:
            while not self.stopped:
                f = lps.read_frame(self.sock, self.count, self.size, self.h264)
                if isinstance(f, lps.SensorSample):
                    with self.lock:
                        self.sensors.setdefault(f.type, deque(maxlen=6000)).append((f.timestamp_ns, f.values))
                        self.sensor_counts[f.type] = self.sensor_counts.get(f.type, 0) + 1
                    if f.type == SENSOR_FRAME_TIMING:
                        self.frame_timing = f.values
                        self.timing.add(f.timestamp_ns, f.values)
                    self._cb(self.on_sensor, f)
                    continue
                pair = None
                with self.lock:                                   # PhoneReceiver._run 과 같은 처리
                    i = f.index
                    if self.last_ts[i] is not None and f.timestamp_ns <= self.last_ts[i]:
                        self.reordered[i] += 1
                        continue
                    if self.first_ts[i] is None:
                        self.first_ts[i] = f.timestamp_ns
                    self.last_ts[i] = f.timestamp_ns
                    self.last_receive[i] = time.monotonic()
                    self.counts[i] += 1
                    self.latest[i] = f
                    pair = self.pairer.add(f)
                    if pair:
                        self.pair = pair
                        self.pair_count += 1
                        ts = [x.timestamp_ns for x in pair]
                        self.skews.append((max(ts) - min(ts)) / 1e6)
                if pair:
                    self._cb(self.on_pair, pair)
        except (OSError, EOFError, ValueError) as e:
            if not self.stopped:
                self.error = str(e) or type(e).__name__
        except Exception as e:                              # 예상 못 한 오류도 재접속 대상으로
            if not self.stopped:
                self.error = repr(e)
        finally:
            if not self.stopped and self.error is None:
                self.error = "수신 스레드 종료"

    def camera_time_offset_ns(self, frame=None):
        """노출 중앙 보정 ns. frame(PhoneFrame) 을 주면 그 프레임의 frame_timing 값 (REVIEW 6), 없으면 최신값."""
        if frame is not None:
            return self.timing.offset_ns(frame.index, frame.timestamp_ns)
        r = self.timing.latest
        return 0 if r is None else int(0.5 * (r[0] + r[1]) * 1e6)


class ImuSync:
    """자이로(기준 시계) + 가속도(선형 보간) → emit(t_ns, gyro(3), accel(3)). 출력 시각은 단조 증가."""

    def __init__(self, emit, gyro_type=SENSOR_GYRO, accel_type=SENSOR_ACCEL, max_gap_ns=50_000_000):
        self.emit = emit
        self.gyro_type, self.accel_type = gyro_type, accel_type
        self.max_gap = max_gap_ns
        self.reset()

    def reset(self):
        self.gyro = deque()
        self.accel = deque(maxlen=64)
        self.last_out = -1
        self.n_out = self.n_drop = 0

    def add(self, s):
        if s.type == self.gyro_type:
            self.gyro.append((s.timestamp_ns, s.values[:3]))
        elif s.type == self.accel_type:
            if self.accel and s.timestamp_ns <= self.accel[-1][0]:
                return
            self.accel.append((s.timestamp_ns, s.values[:3]))
        else:
            return
        self._flush()

    def _flush(self):
        acc = self.accel
        while self.gyro:
            tg, w = self.gyro[0]
            if not acc or acc[-1][0] < tg:
                # 뒤 가속도가 아직 없음 → 대기, 단 너무 오래(50 ms) 기다린 샘플은 버림
                if self.gyro[-1][0] - tg > self.max_gap:
                    self.gyro.popleft(); self.n_drop += 1
                    continue
                return
            self.gyro.popleft()
            if acc[0][0] > tg or tg <= self.last_out:
                self.n_drop += 1
                continue
            while len(acc) >= 2 and acc[1][0] <= tg:          # acc[0] = tg 이전 마지막 샘플
                acc.popleft()
            t0, a0 = acc[0]
            t1, a1 = acc[1] if len(acc) > 1 else acc[0]
            if t1 - t0 > self.max_gap:
                self.n_drop += 1
                continue
            r = 0.0 if t1 == t0 else (tg - t0) / (t1 - t0)
            a = tuple(a0[k] + r * (a1[k] - a0[k]) for k in range(3))
            self.last_out = tg
            self.n_out += 1
            self.emit(tg, tuple(w), a)
