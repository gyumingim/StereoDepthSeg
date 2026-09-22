"""USB/ADB Camera2 NV21 receiver. Frame pairing uses phone sensor timestamps.

Wire format: big endian magic/index/width/height/timestamp_ns/length + NV21.
No JPEG, frame resizing, orientation rotation, or inferred exposure timestamps.
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


@dataclass
class PhoneFrame:
    index: int
    timestamp_ns: int
    bgr: np.ndarray


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
    magic, index, w, h, ts, length = HEADER.unpack(recv_exact(sock, HEADER.size))
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
        self.error = None
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped:
                f = read_frame(self.sock, self.count, self.size)
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

    def stats(self):
        with self.lock:
            fps = [(n - 1) * 1e9 / (b - a) if n > 1 and b > a else 0.0
                   for n, a, b in zip(self.counts, self.first_ts, self.last_ts)]
            return dict(frames=list(self.counts), sensor_fps=fps, pairs=self.pair_count,
                        unmatched_dropped=list(self.pairer.dropped), error=self.error,
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
