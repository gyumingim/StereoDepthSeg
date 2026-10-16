"""USB/ADB Camera2 NV21 receiver. Frame pairing uses phone sensor timestamps.

Wire format: big endian magic/index/width/height/timestamp_ns/length + payload.
프레임 payload: magic "STEH" = H.264 Annex-B 접근 단위 (앱 기본, --es stream_fmt h264, I+P 만),
"STEJ" = 같은 프레임의 JPEG (--es stream_fmt jpeg), "STER" = NV21 원본 (--es stream_fmt nv21).
셋 다 같은 BGR PhoneFrame 으로 바뀌어 나간다. 크기 변경·회전·노출 시각 추정은 없다.
헤더 timestamp (scan3d/SPEC.md 1.2): 앱 1.7-mid 부터 그 렌즈 사진 전체 노출의 가운데 시각
= 물리 SENSOR_TIMESTAMP + 노출/2 + 롤링셔터 skew/2 (시계는 그대로 elapsedRealtimeNanos). 앱이 센서 타입 101
(stream_info, [1.0]) 을 보내면 이 기준이고 ts_ref = "mid_exposure", 옛 앱(없음)은 첫 줄 노출 시작 (ts_ref = None).
H.264 는 렌즈별 PyAV 디코더(H264Decoders)가 풀고, 키프레임이 필요하면 같은 소켓으로 "keyframe <렌즈>\n" 한 줄을 폰에 보낸다.

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
MAGIC_JPEG = 0x5354454A            # "STEJ": payload = JPEG 한 장 (scan3d/SPEC.md 1.2)
JPEG_MAX_BYTES = 8 << 20
jpeg_decode_errors = 0             # 깨진 JPEG 은 건너뛰고 센다 (스트림 경계는 길이로 맞으므로 재접속 불필요)
MAGIC_H264 = 0x53544548            # "STEH" (scan3d/SPEC.md 1.2.2, 앱 1.4-h264 Main/LiveEncoder 기준)
# STEH: 28 B 공통 헤더 (index = 렌즈 번호, width/height = 프레임 크기, timestamp = 렌즈 노출 가운데 ns (1.7-mid, 옛 앱은 물리 센서 ns),
# length = 8 + NAL 바이트 수) 뒤에 [u32 flags][u32 seq][Annex-B NAL 바이트 = 프레임 1장].
H264_PREFIX = struct.Struct(">II")
H264_MAX_BYTES = 8 << 20
H264_FLAG_KEY = 1                  # bit0: 키프레임 (IDR)
H264_FLAG_SPS = 2                  # bit1: 이 패킷 안에 SPS/PPS 있음 (앱은 모든 키프레임 앞에 붙인다)
# PC→폰 키프레임 요청: 같은 소켓 반대 방향으로 텍스트 한 줄 "keyframe\n" (전부) 또는 "keyframe <렌즈>\n"
KEYREQ_MIN_INTERVAL_S = 0.5       # 렌즈별 키프레임 요청 최소 간격 (요청 폭주 방지; 키 간격 1 s 이니 기다려도 1 s 안)

# android.hardware.Sensor 타입 상수 (앱이 등록하는 것만)
SENSOR_ACCEL = 1                   # 원시 가속도계 (중력 포함 비력, VIO 용)
SENSOR_GYRO, SENSOR_PRESSURE, SENSOR_GRAVITY, SENSOR_LINEAR_ACC, SENSOR_ROTVEC, SENSOR_GAME_ROTVEC = 4, 6, 9, 10, 11, 15
SENSOR_FRAME_TIMING = 100          # 앱 자체 타입: [물리별 SENSOR_TIMESTAMP − 논리 timestamp (ms)…, 노출 ms, 프레임 주기 ms, 롤링셔터 skew ms]
SENSOR_STREAM_INFO = 101          # 앱 자체 타입 (1.7-mid): [ts_ref 부호] 1.0 = 프레임 헤더 timestamp 가 노출 가운데 시각
TS_REF_MID = "mid_exposure"
SENSOR_NAMES = {SENSOR_ACCEL: "accelerometer", SENSOR_GYRO: "gyroscope", SENSOR_PRESSURE: "pressure", SENSOR_GRAVITY: "gravity",
                SENSOR_LINEAR_ACC: "linear_acceleration", SENSOR_ROTVEC: "rotation_vector", SENSOR_GAME_ROTVEC: "game_rotation_vector",
                SENSOR_FRAME_TIMING: "frame_timing"}


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


def _ue(bits, pos):
    """exp-Golomb ue(v) 하나 읽기 → (값, 다음 위치). bits = '0101…' 문자열."""
    z = 0
    while pos + z < len(bits) and bits[pos + z] == "0":
        z += 1
    if pos + 2 * z + 1 > len(bits):
        raise IndexError
    return int(bits[pos + z:pos + 2 * z + 1], 2) - 1, pos + 2 * z + 1


def _slice_types(buf):
    """Annex-B 의 슬라이스 NAL(1·5)마다 slice_type % 5 (0=P, 1=B, 2=I, 3=SP, 4=SI). 앞쪽 8개만."""
    out, i, n = [], 0, len(buf)
    while len(out) < 8:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0 or j + 3 >= n:
            break
        i = j + 3
        if buf[j + 3] & 0x1F not in (1, 5):
            continue
        head = bytes(buf[j + 4:j + 16]).replace(b"\x00\x00\x03", b"\x00\x00")   # 에뮬레이션 방지 바이트 제거
        bits = "".join(f"{b:08b}" for b in head)
        try:
            _, p = _ue(bits, 0)                        # first_mb_in_slice
            st, _ = _ue(bits, p)                       # slice_type
            out.append(st % 5)
        except (IndexError, ValueError):
            pass
    return out


def _nal_types(buf):
    """Annex-B 바이트의 NAL 타입 목록 (00 00 01 / 00 00 00 01 시작 코드). 앞쪽 몇 개만 보면 충분."""
    out, i, n = [], 0, len(buf)
    while len(out) < 8:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0 or j + 3 >= n:
            break
        out.append(buf[j + 3] & 0x1F)
        i = j + 3
    return out


class H264Decoders:
    """렌즈별 H.264 디코더 (PyAV = FFmpeg libavcodec, 소프트웨어). 폰 앱은 I+P 만 (B-frame 0) 보내므로
    입력 순서 = 출력 순서다. 그래도 packet.pts = 센서 timestamp_ns 를 넣고 나온 frame.pts 로 다시 찾아 확인한다.

    - 키프레임(IDR) 전의 P 는 버리고 센다 (waiting_key_dropped). 접속 직후·오류 뒤 키프레임 요청("keyframe <렌즈>")을 보낸다.
    - 앱은 모든 키프레임 앞에 SPS/PPS 를 붙인다 (flags bit1). 혹시 빠지면 렌즈별로 저장해 둔 마지막 SPS/PPS 를 붙여 넣는다.
    - seq 가 건너뛰면(폰 큐·인코더 출력에서 버려짐) seq_gaps 로 센다. 그 패킷이 키프레임이 아니면 참조 프레임을 잃은 P 이므로
      그 렌즈를 키프레임 대기로 돌리고 바로 키프레임을 요청한다 (REVIEW7 H1; 0.5 s 요청 제한은 그대로).
    - B 슬라이스(slice_type 1)가 보이면 b_frames_seen 으로 센다 (앱은 B 0 을 요청하지만 일부 인코더는 힌트로만 본다; REVIEW7 H5).
    - 디코딩 예외·is_corrupt 프레임·출력 없음 → 그 렌즈를 키프레임 대기로 돌리고 키프레임 요청.
    - 결과는 BGR ndarray (기존 PhoneFrame.bgr 과 같음) → 쌍 묶기·OpenVINS·FFS 경로는 그대로.
    """

    def __init__(self, count, sock=None):
        self.av = None                                  # 첫 STEH 패킷 때 import (JPEG·NV21 만 쓰면 PyAV 없어도 됨)
        self.count, self.sock = count, sock
        self.ctx = [None] * count
        self.config = [None] * count                    # 마지막 SPS/PPS 바이트
        self.need_key = [True] * count
        self.pending = [deque(maxlen=16) for _ in range(count)]   # 넣었지만 아직 안 나온 timestamp (pts 없을 때 대비)
        self.last_keyreq = [0.0] * count
        self.decode_ms = [deque(maxlen=120) for _ in range(count)]
        self.bytes = [deque(maxlen=120) for _ in range(count)]
        self.n = dict(frames=[0] * count, keyframes=[0] * count, waiting_key_dropped=[0] * count,
                      decode_errors=[0] * count, corrupt=[0] * count, no_output=[0] * count,
                      pts_mismatch=[0] * count, keyreq_sent=[0] * count, config=[0] * count, size_mismatch=[0] * count,
                      seq_gaps=[0] * count, seq_gap_resync=[0] * count, b_frames_seen=[0] * count)
        self.rate = [deque(maxlen=120) for _ in range(count)]   # (센서 ns, 바이트) — 렌즈별 kbps
        self.last_seq = [None] * count
        self.keyreq_errors = 0

    @property
    def used(self):
        return self.av is not None

    def _ensure_av(self):
        if self.av is None:
            try:
                import av                               # venv_ffs: uv pip install av (PyAV 19, FFmpeg 내장 휠)
            except ImportError as e:
                raise ValueError("H.264 스트림을 풀려면 PyAV 가 필요합니다: venv_ffs 에서 uv pip install av "
                                 "(또는 --app-stream jpeg)") from e
            self.av = av

    def _new_ctx(self):
        c = self.av.CodecContext.create("h264", "r")
        c.thread_type = "SLICE"                         # FRAME 스레딩은 스레드 수만큼 프레임이 늦게 나온다 → 실시간엔 SLICE
        c.thread_count = 2
        try:
            c.options = {"flags": "low_delay"}          # 재정렬 버퍼 없이 바로 출력 (B-frame 없음 전제)
        except Exception:
            pass
        return c

    def reset(self, i):
        self.ctx[i] = None
        self.pending[i].clear()
        self.need_key[i] = True

    def request_key(self, i, force=False):
        """폰에 키프레임 요청 ("keyframe <렌즈>\n" 한 줄, 같은 소켓의 PC→폰 방향; win_relay 는 그 방향도 그대로 중계)."""
        now = time.monotonic()
        if self.sock is None or (not force and now - self.last_keyreq[i] < KEYREQ_MIN_INTERVAL_S):
            return
        self.last_keyreq[i] = now
        try:
            self.sock.sendall(f"keyframe {i}\n".encode())
            self.n["keyreq_sent"][i] += 1
        except OSError:
            self.keyreq_errors += 1                     # 보내기 실패는 무시 (어차피 1 s 마다 키프레임이 온다)

    def _fail(self, i, key):
        self.n[key][i] += 1
        self.reset(i)
        self.request_key(i)

    def decode(self, i, ts, flags, seq, data):
        """Annex-B 접근 단위 하나 → (timestamp_ns, bgr) 또는 None (설정·키 대기·오류).
        접속 직후 첫 패킷이 P 면 그 자리에서 키프레임을 요청한다 (다음 정기 키프레임 1 s 를 기다리지 않게)."""
        self._ensure_av()
        gap = self.last_seq[i] is not None and seq != (self.last_seq[i] + 1) & 0xFFFFFFFF
        if gap:
            self.n["seq_gaps"][i] += 1
        self.last_seq[i] = seq
        if not data:
            return None
        self.rate[i].append((int(ts), len(data)))
        types = _nal_types(data)
        if types and all(t in (7, 8) for t in types):
            self.config[i] = bytes(data)                # SPS(7)/PPS(8) 만 → 저장, 다음 키프레임 앞에 붙인다
            self.n["config"][i] += 1
            if gap and not self.need_key[i]:           # 설정 패킷 앞이 빠졌어도 다음은 키프레임부터 (H1)
                self.n["seq_gap_resync"][i] += 1
                self.reset(i)
                self.request_key(i)
            return None
        if flags & H264_FLAG_SPS or 7 in types:
            self.config[i] = _sps_pps(data) or self.config[i]   # 키프레임 앞 SPS/PPS 를 저장 (빠진 키프레임 대비)
            self.n["config"][i] += 1
        key = bool(flags & H264_FLAG_KEY) or 5 in types
        if gap and not key and not self.need_key[i]:
            # 앞 패킷이 빠졌는데 키프레임이 아님 → 참조를 잃은 P. 디코딩하지 않고 키프레임부터 다시 (H1)
            self.n["seq_gap_resync"][i] += 1
            self.reset(i)
            self.request_key(i)
        if not key and 1 in _slice_types(data):
            self.n["b_frames_seen"][i] += 1
        if self.need_key[i]:
            if not key:
                self.n["waiting_key_dropped"][i] += 1
                self.request_key(i)
                return None
            self.ctx[i] = self._new_ctx()
            self.need_key[i] = False
        if key:
            self.n["keyframes"][i] += 1
            if 7 not in types and self.config[i]:
                data = self.config[i] + bytes(data)     # KEY_PREPEND_HEADER_TO_SYNC_FRAMES 를 못 쓰는 기기 대비
        t0 = time.perf_counter()
        pkt = self.av.Packet(bytes(data))
        pkt.pts = int(ts)
        self.pending[i].append(int(ts))
        try:
            frames = self.ctx[i].decode(pkt)
        except (self.av.error.FFmpegError, ValueError):
            self._fail(i, "decode_errors")
            return None
        if not frames:
            # low_delay + I/P 면 패킷마다 한 장이 나와야 한다. 한두 장 늦게 내는 디코더는 봐주고(pts 로 다시 맞춤),
            # 4 장 넘게 밀리면 처음부터 (키프레임 요청)
            self.n["no_output"][i] += 1
            if len(self.pending[i]) > 4:
                self._fail(i, "decode_errors")
            return None
        fr = frames[-1]
        if getattr(fr, "is_corrupt", False):
            self._fail(i, "corrupt")                    # 참조 프레임을 잃고 숨김 처리된 영상 → 버리고 키프레임부터
            return None
        out_ts = fr.pts if fr.pts is not None else None
        if out_ts is None or out_ts not in self.pending[i]:
            if out_ts is not None:
                self.n["pts_mismatch"][i] += 1
            out_ts = self.pending[i][0]                 # pts 를 잃었으면 넣은 순서(= 나온 순서, B 0) 로
        while self.pending[i] and self.pending[i][0] <= out_ts:
            self.pending[i].popleft()
        bgr = fr.to_ndarray(format="bgr24")
        self.decode_ms[i].append((time.perf_counter() - t0) * 1000.0)
        self.bytes[i].append(len(data))
        self.n["frames"][i] += 1
        return out_ts, bgr

    def stats(self):
        avg = lambda q: round(sum(q) / len(q), 2) if q else None

        def kbps(q):
            q = list(q)
            if len(q) < 2 or q[-1][0] <= q[0][0]:
                return None
            return round(sum(b for _, b in q[1:]) * 8 / ((q[-1][0] - q[0][0]) / 1e9) / 1000.0, 1)
        return dict(self.n, decode_ms_avg=[avg(q) for q in self.decode_ms], kbps=[kbps(q) for q in self.rate],
                    decode_ms_max=[round(max(q), 2) if q else None for q in self.decode_ms],
                    bytes_avg=[avg(q) for q in self.bytes], waiting_key=list(self.need_key),
                    keyreq_errors=self.keyreq_errors)


def _sps_pps(buf):
    """Annex-B 버퍼 앞쪽의 SPS/PPS 부분(첫 슬라이스 NAL 직전까지). 없으면 None."""
    i = 0
    while True:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0 or j + 3 >= len(buf):
            return None
        if buf[j + 3] & 0x1F in (1, 5):
            k = j - 1 if j > 0 and buf[j - 1] == 0 else j
            return bytes(buf[:k]) if k > 0 else None
        i = j + 3


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


def read_frame(sock, stream_count, size, h264=None):
    """다음 패킷: PhoneFrame 또는 SensorSample. JPEG 디코드 실패 패킷, 프레임이 안 나온 H.264 패킷(SPS/PPS·키 대기·오류)은
    건너뛰고 다음 패킷을 읽는다. h264 = H264Decoders (접속마다 하나; STEH 가 오는데 None 이면 ValueError)."""
    global jpeg_decode_errors
    while True:
        magic, index, w, h, ts, length = HEADER.unpack(recv_exact(sock, HEADER.size))
        if magic == MAGIC_H264:
            lens = index
            if lens >= stream_count or (w, h) != tuple(size) or not H264_PREFIX.size <= length <= H264_MAX_BYTES:
                raise ValueError(f"Invalid H.264 packet: stream={lens} {w}x{h} length={length}")
            payload = recv_exact(sock, length)
            flags, seq = H264_PREFIX.unpack_from(payload)
            data = bytes(payload[H264_PREFIX.size:])
            if h264 is None:
                raise ValueError("H.264 스트림인데 디코더가 없음 (PhoneReceiver 를 쓰거나 read_frame(..., h264=H264Decoders(n, sock)))")
            out = h264.decode(lens, ts, flags, seq, data)
            if out is None:
                continue
            out_ts, bgr = out
            if bgr.shape[:2] != (h, w):
                h264.n["size_mismatch"][lens] += 1
                continue
            return PhoneFrame(lens, out_ts, bgr)
        if magic != MAGIC_JPEG:
            break
        if index >= stream_count or (w, h) != tuple(size) or not 0 < length <= JPEG_MAX_BYTES:
            raise ValueError(f"Invalid JPEG packet: stream={index} {w}x{h} length={length}")
        buf = np.frombuffer(recv_exact(sock, length), np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is not None and bgr.shape[:2] == (h, w):
            fr = PhoneFrame(index, ts, bgr)
            fr.jpeg = buf          # 받은 JPEG 바이트 (scan3d 카메라 창이 다시 인코드하지 않고 그대로 보냄)
            return fr
        jpeg_decode_errors += 1
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
            # Stream 0 is the reference. Wait until every other stream brackets its
            # timestamp, so callback arrival order cannot select an earlier, worse match.
            target = self.queues[0][0].timestamp_ns
            if any(q[-1].timestamp_ns < target for q in self.queues[1:]):
                return None
            choices = [0] + [min(range(len(q)), key=lambda i: abs(q[i].timestamp_ns-target))
                             for q in self.queues[1:]]
            timestamps = [q[i].timestamp_ns for q, i in zip(self.queues, choices)]
            if max(timestamps) - min(timestamps) <= self.max_skew_ns:
                for stream, (q, i) in enumerate(zip(self.queues, choices)):
                    for _ in range(i):
                        q.popleft()
                        self.dropped[stream] += 1
                return tuple(q.popleft() for q in self.queues)
            self.queues[0].popleft()
            self.dropped[0] += 1
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
        self.reordered = [0] * count            # 보정 timestamp 가 되돌아간 프레임(HAL 재짝짓기 순간) — 버리고 센다
        self.ts_ref = None                      # "mid_exposure" = stream_info(101) 를 받음 (앱 1.7-mid), None = 옛 앱 (노출 시작)
        self.h264 = H264Decoders(count, sock)   # 렌즈별 H.264 디코더 (첫 STEH 패킷 때 PyAV import)
        self.error = None
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped:
                f = read_frame(self.sock, self.count, self.size, self.h264)
                if isinstance(f, SensorSample):
                    with self.lock:
                        self.sensors.setdefault(f.type, deque(maxlen=6000)).append((f.timestamp_ns, f.values))
                        self.sensor_counts[f.type] = self.sensor_counts.get(f.type, 0) + 1
                        if f.type == SENSOR_STREAM_INFO and f.values and round(f.values[0]) == 1:
                            self.ts_ref = TS_REF_MID
                    continue
                with self.lock:
                    i = f.index
                    if self.last_ts[i] is not None and f.timestamp_ns <= self.last_ts[i]:
                        self.reordered[i] += 1            # 물리 timestamp 보정 후 드물게 뒤로 감 → 치명 오류 대신 폐기
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
        except (OSError, EOFError, ValueError) as e:
            if not self.stopped:
                self.error = str(e)

    def snapshot(self):
        with self.lock:
            return list(self.latest), self.pair, self.pair_count, list(self.last_receive)

    def sensor_window(self, stype, t0_ns, t1_ns, pad_ns=8_000_000):
        """[t0, t1] (±pad) 사이 센서 샘플 [(ts, values)…]. 두 노출 시각 사이 자이로 적분(회전 보상)용."""
        lo, hi = min(t0_ns, t1_ns) - pad_ns, max(t0_ns, t1_ns) + pad_ns
        with self.lock:
            q = self.sensors.get(stype)
            return [(t, list(v)) for t, v in q if lo <= t <= hi] if q else []

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

    def imu_for_pair(self, pair):
        """쌍의 첫 프레임 시각 센서값 + 두 노출 시각 사이 자이로 창(gyro_window) — 동기 어긋남 회전 보상용.
        노출 시각 = 프레임 timestamp (앱 1.7-mid 부터 노출 가운데, 옛 앱은 노출 시작)."""
        out = self.imu_at(pair[0].timestamp_ns)
        ts = [f.timestamp_ns for f in pair[:2]]
        out["per_frame"] = [self.imu_at(t) for t in ts]
        out["exposure_ts_ns"] = ts
        out["exposure_skew_ms"] = (ts[1] - ts[0]) / 1e6
        out["gyro_window"] = self.sensor_window(SENSOR_GYRO, ts[0], ts[1]) if ts[0] != ts[1] else []
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
            h = self.h264.stats() if self.h264.used else None
            # decode_ms / decode_ms_max = H.264 한 장 풀기 (렌즈 하나 기준, 렌즈 평균·최대; 웹 부하 표시용)
            dm = [v for v in (h["decode_ms_avg"] if h else []) if v is not None]
            dx = [v for v in (h["decode_ms_max"] if h else []) if v is not None]
            return dict(decode_ms=round(sum(dm) / len(dm), 2) if dm else None, decode_ms_max=max(dx) if dx else None,
                        frames=list(self.counts), sensor_fps=fps, pairs=self.pair_count,
                        unmatched_dropped=list(self.pairer.dropped), reordered_dropped=list(self.reordered), error=self.error,
                        jpeg_decode_errors=jpeg_decode_errors, h264=h, imu=imu, clock_gap_ms=None if gap is None else round(gap, 1),
                        skew_median_ms=float(np.median(self.skews)) if self.skews else None,
                        skew_p95_ms=float(np.percentile(self.skews, 95)) if self.skews else None,
                        first_timestamp_ns=list(self.first_ts), last_timestamp_ns=list(self.last_ts), ts_ref=self.ts_ref)

    def close(self):
        self.stopped = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self.thread.join(timeout=3)
