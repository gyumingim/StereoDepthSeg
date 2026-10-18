#!/usr/bin/env python3
"""폰이 녹화하면서 올린 camN.h264 (Annex-B) + camN.h264.csv (출력 버퍼 목록) → camN.mp4 (표준 라이브러리만, 다시 인코딩 안 함).

앱 1.10-liveupload 의 Recorder 는 인코더 출력 버퍼를 그대로 camN.h264 에 이어 쓰고, 버퍼마다 한 줄
`offset,size,pts_us,flags` 를 camN.h264.csv 에 쓴다 (flags = MediaCodec: 1 키프레임, 2 코덱 설정 SPS/PPS).
MP4 는 moov 가 끝에 써져서 녹화 중에 올릴 수 없으므로, PC 가 받은 뒤 같은 비트스트림·같은 PTS 로 MP4 를 다시 묶는다
(폰 MediaMuxer 의 camN.mp4 와 같은 프레임·같은 pts_us → recording.py 의 PTS 맞추기가 그대로 됨).

    python3 scan3d/h264_mp4.py REC_DIR/cam2.h264 [out.mp4]      # 기본 출력 = 같은 이름 .mp4
"""
import csv
import struct
import sys
from pathlib import Path

TIMESCALE = 1_000_000          # pts_us 그대로
FLAG_KEY, FLAG_CONFIG = 1, 2


def _box(kind, *parts):
    body = b"".join(parts)
    return struct.pack(">I4s", 8 + len(body), kind) + body


def _full(kind, version, flags, *parts):
    return _box(kind, struct.pack(">I", (version << 24) | flags), *parts)


def split_nals(buf):
    """Annex-B 버퍼 → NAL 목록 (시작 코드·뒤쪽 0 바이트 뺌)."""
    out, n, i = [], len(buf), 0
    starts = []
    while True:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0:
            break
        starts.append(j + 3)
        i = j + 3
    for k, s in enumerate(starts):
        e = (starts[k + 1] - 3) if k + 1 < len(starts) else n
        nal = buf[s:e].rstrip(b"\x00")
        if nal:
            out.append(nal)
    return out


def read_index(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) < 4:
                break                                    # 마지막 줄이 잘림 (강제 종료)
            try:
                rows.append(tuple(int(x) for x in row[:4]))
            except ValueError:
                break
    return rows


def remux(h264, out=None, index=None):
    """반환 dict(frames, keys, bytes, width, height, out). 실패하면 예외."""
    h264 = Path(h264)
    index = Path(index) if index else h264.with_name(h264.name + ".csv")
    out = Path(out) if out else h264.with_suffix(".mp4")
    rows = read_index(index)
    raw_size = h264.stat().st_size
    sps = pps = None
    samples = []                                         # (size, pts, key)
    tmp = out.with_name(out.name + ".part")
    with open(h264, "rb") as src, open(tmp, "wb") as dst:
        ftyp = _box(b"ftyp", b"isom", struct.pack(">I", 512), b"isomiso2avc1mp41")
        dst.write(ftyp)
        mdat_pos = dst.tell()
        dst.write(struct.pack(">I4sQ", 1, b"mdat", 0))   # 64 비트 크기, 끝에 채움
        data_off = dst.tell()
        for off, size, pts, flags in rows:
            if size <= 0 or off + size > raw_size:
                break
            src.seek(off)
            nals = split_nals(src.read(size))
            if flags & FLAG_CONFIG:
                for nal in nals:
                    t = nal[0] & 0x1F
                    if t == 7 and sps is None:
                        sps = nal
                    elif t == 8 and pps is None:
                        pps = nal
                continue
            if not nals:
                continue
            key = bool(flags & FLAG_KEY) or any((x[0] & 0x1F) == 5 for x in nals)
            for nal in nals:
                t = nal[0] & 0x1F
                if t == 7 and sps is None:
                    sps = nal
                elif t == 8 and pps is None:
                    pps = nal
            b = b"".join(struct.pack(">I", len(x)) + x for x in nals)
            dst.write(b)
            samples.append((len(b), pts, key))
        mdat_end = dst.tell()
        if not samples or sps is None or pps is None:
            raise ValueError(f"{h264.name}: 프레임 {len(samples)} 개, SPS {'있음' if sps else '없음'}, "
                             f"PPS {'있음' if pps else '없음'} → MP4 못 만듦")
        w, h = sps_size(sps)
        dst.write(_moov(samples, sps, pps, w, h, data_off))
        dst.seek(mdat_pos)
        dst.write(struct.pack(">I4sQ", 1, b"mdat", mdat_end - mdat_pos))
    tmp.replace(out)
    return dict(frames=len(samples), keys=sum(1 for s in samples if s[2]), bytes=out.stat().st_size,
                width=w, height=h, out=str(out), index_rows=len(rows))


def _moov(samples, sps, pps, w, h, data_off):
    n = len(samples)
    pts = [s[1] for s in samples]
    mono = all(b > a for a, b in zip(pts, pts[1:]))
    dts = pts if mono else sorted(pts)                   # B 프레임 (앱은 B=0 을 요청 — 거의 안 생김)
    deltas = [dts[i + 1] - dts[i] for i in range(n - 1)]
    last = deltas[-1] if deltas else 66666
    deltas.append(last if last > 0 else 1)
    dur = sum(deltas)
    if dur >= 1 << 32:
        raise ValueError("녹화가 너무 김 (mdhd 32 비트)")
    # stts (run length)
    runs = []
    for d in deltas:
        if runs and runs[-1][1] == d:
            runs[-1][0] += 1
        else:
            runs.append([1, d])
    stts = _full(b"stts", 0, 0, struct.pack(">I", len(runs)), b"".join(struct.pack(">II", c, d) for c, d in runs))
    ctts = b""
    if not mono:
        offs = [p - d for p, d in zip(pts, dts)]
        ctts = _full(b"ctts", 1, 0, struct.pack(">I", n), b"".join(struct.pack(">Ii", 1, o) for o in offs))
    keys = [i + 1 for i, s in enumerate(samples) if s[2]]
    stss = _full(b"stss", 0, 0, struct.pack(">I", len(keys)), b"".join(struct.pack(">I", k) for k in keys)) \
        if len(keys) < n else b""
    stsc = _full(b"stsc", 0, 0, struct.pack(">IIII", 1, 1, n, 1))
    stsz = _full(b"stsz", 0, 0, struct.pack(">II", 0, n), b"".join(struct.pack(">I", s[0]) for s in samples))
    co64 = _full(b"co64", 0, 0, struct.pack(">IQ", 1, data_off))
    avcc = _box(b"avcC", bytes([1, sps[1], sps[2], sps[3], 0xFF, 0xE1]), struct.pack(">H", len(sps)), sps,
                b"\x01", struct.pack(">H", len(pps)), pps)
    avc1 = _box(b"avc1", b"\x00" * 6, struct.pack(">H", 1), b"\x00" * 16, struct.pack(">HH", w, h),
                struct.pack(">II", 0x00480000, 0x00480000), b"\x00" * 4, struct.pack(">H", 1), b"\x00" * 32,
                struct.pack(">Hh", 0x18, -1), avcc)
    stsd = _full(b"stsd", 0, 0, struct.pack(">I", 1), avc1)
    stbl = _box(b"stbl", stsd, stts, ctts, stss, stsc, stsz, co64)
    dref = _full(b"dref", 0, 0, struct.pack(">I", 1), _full(b"url ", 0, 1))
    minf = _box(b"minf", _full(b"vmhd", 0, 1, b"\x00" * 8), _box(b"dinf", dref), stbl)
    hdlr = _full(b"hdlr", 0, 0, b"\x00" * 4, b"vide", b"\x00" * 12, b"VideoHandle\x00")
    mdhd = _full(b"mdhd", 0, 0, struct.pack(">IIIIHH", 0, 0, TIMESCALE, dur, 0x55C4, 0))
    mdia = _box(b"mdia", mdhd, hdlr, minf)
    matrix = struct.pack(">9I", 0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000)
    dur_ms = dur // 1000
    tkhd = _full(b"tkhd", 0, 3, struct.pack(">IIIII", 0, 0, 1, 0, dur_ms), b"\x00" * 8,
                 struct.pack(">hhhH", 0, 0, 0, 0), matrix, struct.pack(">II", w << 16, h << 16))
    trak = _box(b"trak", tkhd, mdia)
    mvhd = _full(b"mvhd", 0, 0, struct.pack(">IIII", 0, 0, 1000, dur_ms), struct.pack(">IH", 0x10000, 0x100),
                 b"\x00" * 10, matrix, b"\x00" * 24, struct.pack(">I", 2))
    return _box(b"moov", mvhd, trak)


# ---------------- SPS → 가로·세로 ----------------
class _Bits:
    def __init__(self, nal):
        rbsp, z = bytearray(), 0                         # 에뮬레이션 방지 바이트 (00 00 03) 빼기
        for b in nal[1:]:
            if z >= 2 and b == 3:
                z = 0
                continue
            rbsp.append(b)
            z = z + 1 if b == 0 else 0
        self.d, self.p = bytes(rbsp), 0

    def u(self, n):
        v = 0
        for _ in range(n):
            v = (v << 1) | ((self.d[self.p >> 3] >> (7 - (self.p & 7))) & 1)
            self.p += 1
        return v

    def ue(self):
        z = 0
        while self.u(1) == 0:
            z += 1
        return (1 << z) - 1 + self.u(z)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k & 1 else -(k // 2)


def sps_size(sps):
    b = _Bits(sps)
    prof = b.u(8)
    b.u(16)
    b.ue()
    chroma = 1
    if prof in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
        chroma = b.ue()
        if chroma == 3:
            b.u(1)
        b.ue()
        b.ue()
        b.u(1)
        if b.u(1):
            for i in range(8 if chroma != 3 else 12):
                if b.u(1):
                    last, nxt = 8, 8
                    for _ in range(16 if i < 6 else 64):
                        if nxt:
                            nxt = (last + b.se() + 256) % 256
                        last = nxt or last
    b.ue()
    poc = b.ue()
    if poc == 0:
        b.ue()
    elif poc == 1:
        b.u(1)
        b.se()
        b.se()
        for _ in range(b.ue()):
            b.se()
    b.ue()
    b.u(1)
    mbw, mbh = b.ue() + 1, b.ue() + 1
    frame_mbs_only = b.u(1)
    if not frame_mbs_only:
        b.u(1)
    b.u(1)
    w, h = mbw * 16, mbh * 16 * (2 - frame_mbs_only)
    if b.u(1):
        l, r, t, bo = b.ue(), b.ue(), b.ue(), b.ue()
        cx, cy = (2, 2) if chroma == 1 else ((2, 1) if chroma == 2 else (1, 1))
        cy *= 2 - frame_mbs_only
        w -= (l + r) * cx
        h -= (t + bo) * cy
    return w, h


def remux_dir(d, keep=False, log=print):
    """폴더 안 camN.h264 + camN.h264.csv 중 camN.mp4 가 없는 것을 MP4 로. keep=False 면 성공 뒤 .h264·색인 지움.
    반환 {렌즈 파일: 결과 dict}."""
    d = Path(d)
    res = {}
    for h in sorted(d.glob("cam*.h264")):
        idx = h.with_name(h.name + ".csv")
        mp4 = h.with_suffix(".mp4")
        if not idx.is_file() or mp4.exists():
            continue
        r = remux(h, mp4, idx)
        res[mp4.name] = r
        log(f"{d.name}/{mp4.name}: {h.name} 에서 MP4 묶음 ({r['frames']} 프레임, 키 {r['keys']}, {r['width']}x{r['height']})")
        if not keep:
            h.unlink()
            idx.unlink()
    return res


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)
    print(remux(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None))
