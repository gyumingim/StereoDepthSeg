"""USB/ADB Camera2 NV21 receiver. Frame pairing uses phone sensor timestamps.

Wire format: big endian magic/index/width/height/timestamp_ns/length + NV21.
No JPEG, frame resizing, orientation rotation, or inferred exposure timestamps.

센서 패킷(magic "SENS"): index = Android Sensor 타입, width=height=0, payload = float32 big-endian 값들.
카메라 timestamp (SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) 와 SensorEvent.timestamp 는 같은
elapsedRealtimeNanos 시계라 프레임 시각의 IMU 상태를 바로 찾을 수 있다 (stats 의 clock_gap_ms 로 확인).
"""
from collections import deque
from dataclasses import dataclass
import socket
import struct
import threading
import time

import cv2
import numpy as np

HEADER = struct.Struct(">IIIIQI")
MAGIC = 0x53544552
MAGIC_SENSOR = 0x53454E53
# android.hardware.Sensor 타입 상수 (앱이 등록하는 것만)
SENSOR_GYRO, SENSOR_PRESSURE, SENSOR_GRAVITY, SENSOR_LINEAR_ACC, SENSOR_ROTVEC, SENSOR_GAME_ROTVEC = 4, 6, 9, 10, 11, 15
SENSOR_NAMES = {SENSOR_GYRO: "gyroscope", SENSOR_PRESSURE: "pressure", SENSOR_GRAVITY: "gravity",
                SENSOR_LINEAR_ACC: "linear_acceleration", SENSOR_ROTVEC: "rotation_vector", SENSOR_GAME_ROTVEC: "game_rotation_vector"}


@dataclass
class PhoneFrame:
    index: int
    timestamp_ns: int
    bgr: np.ndarray


@dataclass
class SensorSample:
    type: int
    timestamp_ns: int
    values: tuple


def recv_exact(sock, size):
    data = bytearray(size)
    view = memoryview(data)
    offset = 0
    while offset < size:
        n = sock.recv_into(view[offset:])
        if not n:
            raise EOFError("Phone stream closed")
        offset += n
    return data


def read_frame(sock, stream_count, size):
    """다음 패킷: PhoneFrame 또는 SensorSample."""
    magic, index, w, h, ts, length = HEADER.unpack(recv_exact(sock, HEADER.size))
    if magic == MAGIC_SENSOR:
        if (w, h) != (0, 0) or length % 4 or not 4 <= length <= 64:
            raise ValueError(f"Invalid sensor packet: type={index} {w}x{h} length={length}")
        vals = struct.unpack(f">{length // 4}f", recv_exact(sock, length))
        return SensorSample(index, ts, vals)
    if (magic != MAGIC or index >= stream_count or (w, h) != tuple(size)
            or w % 2 or h % 2 or length != w * h * 3 // 2):
        raise ValueError(f"Invalid phone packet: magic={magic:x} stream={index} {w}x{h} length={length}")
    nv21 = np.frombuffer(recv_exact(sock, length), np.uint8).reshape(h * 3 // 2, w)
    return PhoneFrame(index, ts, cv2.cvtColor(nv21, cv2.COLOR_YUV2BGR_NV21))


class FramePairer:
    """Consume each frame at most once; bounded queues discard unmatched old frames."""
    def __init__(self, count=2, max_skew_ms=10):
        self.queues = [deque(maxlen=8) for _ in range(count)]
        self.max_skew_ns = int(max_skew_ms * 1e6)
        self.dropped = [0] * count

    def add(self, frame):
        q = self.queues[frame.index]
        if len(q) == q.maxlen:
            self.dropped[frame.index] += 1
        q.append(frame)
        while all(self.queues):
            timestamps = [q[0].timestamp_ns for q in self.queues]
            if max(timestamps) - min(timestamps) <= self.max_skew_ns:
                return tuple(q.popleft() for q in self.queues)
            oldest = timestamps.index(min(timestamps))
            self.queues[oldest].popleft()
            self.dropped[oldest] += 1
        return None


class PhoneReceiver:
    def __init__(self, sock, count, size, max_skew_ms=10):
        self.sock, self.count, self.size = sock, count, tuple(size)
        self.pairer = FramePairer(count, max_skew_ms)
        self.lock = threading.Lock()
        self.latest = [None] * count
        self.counts = [0] * count
        self.first_ts = [None] * count
        self.last_ts = [None] * count
        self.last_receive = [None] * count
        self.pair = None
        self.pair_count = 0
        self.skews = deque(maxlen=10000)
        self.sensors = {}                       # type -> deque[(timestamp_ns, values)]
        self.sensor_counts = {}
        self.error = None
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped:
                f = read_frame(self.sock, self.count, self.size)
                if isinstance(f, SensorSample):
                    with self.lock:
                        self.sensors.setdefault(f.type, deque(maxlen=6000)).append((f.timestamp_ns, f.values))
                        self.sensor_counts[f.type] = self.sensor_counts.get(f.type, 0) + 1
                    continue
                with self.lock:
                    i = f.index
                    if self.last_ts[i] is not None and f.timestamp_ns <= self.last_ts[i]:
                        raise ValueError(f"Non-increasing sensor timestamp on stream {i}")
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
        except (OSError, EOFError, ValueError) as e:
            if not self.stopped:
                self.error = str(e)

    def snapshot(self):
        with self.lock:
            return list(self.latest), self.pair, self.pair_count, list(self.last_receive)

    def sensor_at(self, stype, timestamp_ns, max_dt_ms=50.0):
        """timestamp_ns 에 가장 가까운 센서 샘플 (values, dt_ms). 없거나 너무 멀면 None."""
        with self.lock:
            q = self.sensors.get(stype)
            if not q:
                return None
            ts = np.fromiter((t for t, _ in q), dtype=np.int64, count=len(q))
            k = int(np.argmin(np.abs(ts - timestamp_ns)))
            dt_ms = (ts[k] - timestamp_ns) / 1e6
            return (tuple(q[k][1]), float(dt_ms)) if abs(dt_ms) <= max_dt_ms else None

    def imu_at(self, timestamp_ns):
        """프레임 시각의 센서 묶음 {이름: {values, dt_ms}} — pair.json 과 depth 보고서에 그대로 들어간다."""
        out = {}
        for stype, name in SENSOR_NAMES.items():
            s = self.sensor_at(stype, timestamp_ns)
            if s:
                out[name] = dict(values=[float(v) for v in s[0]], dt_ms=round(s[1], 2))
        return out

    def stats(self):
        with self.lock:
            fps = [(n - 1) * 1e9 / (b - a) if n > 1 and b > a else 0.0
                   for n, a, b in zip(self.counts, self.first_ts, self.last_ts)]
            imu = {}
            for stype, q in self.sensors.items():
                rate = (len(q) - 1) * 1e9 / (q[-1][0] - q[0][0]) if len(q) > 1 and q[-1][0] > q[0][0] else 0.0
                imu[SENSOR_NAMES.get(stype, str(stype))] = dict(samples=self.sensor_counts[stype], rate_hz=round(rate, 1), last=[round(v, 4) for v in q[-1][1]])
            cam_last = max((t for t in self.last_ts if t is not None), default=None)
            imu_last = max((q[-1][0] for q in self.sensors.values() if q), default=None)
            # 같은 시계면 마지막 프레임과 마지막 센서 샘플의 차이가 수십 ms 안이어야 한다
            gap = (cam_last - imu_last) / 1e6 if cam_last is not None and imu_last is not None else None
            return dict(frames=list(self.counts), sensor_fps=fps, pairs=self.pair_count,
                        unmatched_dropped=list(self.pairer.dropped), error=self.error,
                        imu=imu, clock_gap_ms=None if gap is None else round(gap, 1),
                        skew_median_ms=float(np.median(self.skews)) if self.skews else None,
                        skew_p95_ms=float(np.percentile(self.skews, 95)) if self.skews else None,
                        first_timestamp_ns=list(self.first_ts), last_timestamp_ns=list(self.last_ts))

    def close(self):
        self.stopped = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self.thread.join(timeout=3)
