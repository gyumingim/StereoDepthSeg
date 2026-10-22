#!/usr/bin/env python3
"""Recover a truncated phone H.264 mp4 (no moov, e.g. storage full mid-recording).

Uses a complete reference recording made with the same app/encoder settings for the
avcC (SPS/PPS) and NAL length size, walks the length-prefixed NAL units in the broken
file's mdat, groups them into access units (new AU at a slice with first_mb_in_slice==0)
and stream-copies them (no re-encode) into a new mp4 with PyAV. Frame pts come from the
recording's frames_cam<N>.csv pts_us column (1:1 with encoded samples, verified on the
reference: max deviation 5.6 us at the 90 kHz track timescale).

Usage:
  ../venv_ffs/bin/python tools/mp4_recover.py BROKEN.mp4 REFERENCE.mp4 OUT.mp4 \
      [--csv frames_cam5.csv] [--scan-only] [--keep-untimed]
The broken file is only read. Prints a JSON summary at the end.
"""
import argparse, json, os, struct, sys

import numpy as np

CONTAINERS = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"edts", b"dinf"}


def iter_boxes(f, off, end):
    while off + 8 <= end:
        f.seek(off)
        h = f.read(16)
        if len(h) < 8:
            return
        size, typ = struct.unpack(">I4s", h[:8])
        hl = 8
        if size == 1:
            if len(h) < 16:
                return
            size = struct.unpack(">Q", h[8:16])[0]
            hl = 16
        elif size == 0:
            size = end - off
        yield off, size, typ, hl
        if size < hl:
            return
        off += size


def find_box(f, off, end, path):
    for o, s, t, hl in iter_boxes(f, off, end):
        if t == path[0]:
            if len(path) == 1:
                return o, s, hl
            return find_box(f, o + hl, o + s, path[1:])
    return None


def read_reference(path):
    """Return dict with avcC bytes, length size, timescale, width/height, track count."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        moov = find_box(f, 0, size, [b"moov"])
        if moov is None:
            raise SystemExit("reference has no moov")
        mo, ms, mhl = moov
        traks = []
        for o, s, t, hl in iter_boxes(f, mo + mhl, mo + ms):
            if t == b"trak":
                traks.append((o, s, hl))
        info = {"tracks": [], "avcC": None}
        for o, s, hl in traks:
            hd = find_box(f, o + hl, o + s, [b"mdia", b"hdlr"])
            f.seek(hd[0] + hd[2] + 8)
            handler = f.read(4).decode("latin1")
            md = find_box(f, o + hl, o + s, [b"mdia", b"mdhd"])
            f.seek(md[0] + md[2])
            v = f.read(1)[0]
            f.seek(md[0] + md[2] + (20 if v == 1 else 12))
            ts = struct.unpack(">I", f.read(4))[0]
            sd = find_box(f, o + hl, o + s, [b"mdia", b"minf", b"stbl", b"stsd"])
            f.seek(sd[0] + sd[2])
            body = f.read(sd[1] - sd[2])
            codec = body[12:16].decode("latin1")
            info["tracks"].append({"handler": handler, "timescale": ts, "codec": codec})
            if handler == "vide":
                # visual sample entry: 8 (size,type) + 6 reserved + 2 dref + 16 + w,h
                ent = body[8:]
                w, h = struct.unpack(">HH", ent[32:36])
                info.update(width=w, height=h, timescale=ts, codec=codec)
                for tag in (b"avcC", b"hvcC"):
                    i = ent.find(tag)
                    if i >= 4:
                        bl = struct.unpack(">I", ent[i - 4:i])[0]
                        info[tag.decode()] = ent[i + 4:i - 4 + bl]
        if info.get("avcC") is None:
            raise SystemExit("reference video track has no avcC (HEVC not handled by this script)")
        a = info["avcC"]
        info["length_size"] = (a[4] & 3) + 1
        n_sps = a[5] & 0x1F
        p = 6
        sps = []
        for _ in range(n_sps):
            l = struct.unpack(">H", a[p:p + 2])[0]; sps.append(a[p + 2:p + 2 + l]); p += 2 + l
        n_pps = a[p]; p += 1
        pps = []
        for _ in range(n_pps):
            l = struct.unpack(">H", a[p:p + 2])[0]; pps.append(a[p + 2:p + 2 + l]); p += 2 + l
        info["sps"], info["pps"] = sps, pps
        return info


def first_mb_in_slice(payload):
    """ue(v) from the first bytes after the NAL header (emulation bytes ignorable this early)."""
    bits = int.from_bytes(payload[:8].ljust(8, b"\0"), "big")
    n = 64
    lz = 0
    while lz < 32 and not (bits >> (n - 1 - lz)) & 1:
        lz += 1
    val = (bits >> (n - 1 - 2 * lz)) & ((1 << (lz + 1)) - 1)
    return val - 1


def scan_nals(path, length_size, max_nal=20_000_000):
    """Walk length-prefixed NALs in mdat. Returns (aus, stats). aus = list of
    (offset_of_first_len_prefix, total_bytes, is_idr, nal_types)."""
    size = os.path.getsize(path)
    stats = {"file_size": size, "nal_types": {}, "stop": None}
    with open(path, "rb") as f:
        mdat = None
        for o, s, t, hl in iter_boxes(f, 0, size):
            if t == b"mdat":
                mdat = (o, hl)
                break
            if t not in (b"ftyp", b"free", b"skip", b"wide", b"uuid"):
                break
        if mdat is None:
            # fall back: search the first 4 KB for 'mdat'
            f.seek(0); head = f.read(4096); i = head.find(b"mdat")
            if i < 4:
                raise SystemExit("no mdat found")
            sz = struct.unpack(">I", head[i - 4:i])[0]
            mdat = (i - 4, 16 if sz == 1 else 8)
        off = mdat[0] + mdat[1]
        stats["mdat_payload_start"] = off
        aus = []
        cur = None  # [start, length, is_idr, types]
        f.seek(off)
        buf_off = off
        CH = 64 << 20
        buf = f.read(CH)
        while True:
            rel = off - buf_off
            if rel + length_size + 8 > len(buf):
                f.seek(off); buf_off = off; buf = f.read(CH); rel = 0
                if len(buf) < length_size + 1:
                    stats["stop"] = "eof"
                    break
            n = int.from_bytes(buf[rel:rel + length_size], "big")
            hdr = buf[rel + length_size]
            ftype = hdr & 0x1F
            if n == 0 or n > max_nal or hdr & 0x80 or ftype in (0,) or ftype > 23:
                stats["stop"] = f"invalid nal at {off} len={n} hdr=0x{hdr:02x}"
                break
            if off + length_size + n > size:
                stats["stop"] = f"truncated nal at {off} len={n} (needs {off + length_size + n - size} more bytes)"
                break
            stats["nal_types"][ftype] = stats["nal_types"].get(ftype, 0) + 1
            new_au = False
            if ftype in (1, 5):
                fm = first_mb_in_slice(buf[rel + length_size + 1:rel + length_size + 9])
                new_au = fm == 0
            elif ftype in (6, 7, 8, 9):
                # SEI/SPS/PPS/AUD start a new AU if the current AU already has a slice
                new_au = cur is not None and any(t in (1, 5) for t in cur[3])
            if cur is None or new_au:
                if cur is not None:
                    aus.append(tuple(cur))
                cur = [off, 0, False, []]
            cur[1] += length_size + n
            cur[2] = cur[2] or ftype == 5
            cur[3].append(ftype)
            off += length_size + n
        if cur is not None and any(t in (1, 5) for t in cur[3]):
            aus.append(tuple(cur))
        stats["end_offset"] = off
        stats["tail_bytes_dropped"] = size - off
    return aus, stats


def read_pts_us(csv_path):
    """Complete rows only (last line may be cut)."""
    vals = []
    with open(csv_path, "r", newline="") as fh:
        header = fh.readline().strip().split(",")
        k = header.index("pts_us")
        for line in fh:
            if not line.endswith("\n"):
                break
            parts = line.rstrip("\r\n").split(",")
            if len(parts) != len(header):
                break
            vals.append(int(parts[k]))
    return np.array(vals, dtype=np.int64)


def mux(broken, aus, info, pts_us, out_path, ref_path):
    import av
    ts = info["timescale"]
    n = len(aus)
    if pts_us is not None and len(pts_us) >= n:
        pts = np.round((pts_us[:n] - pts_us[0]) * ts / 1e6).astype(np.int64)
    else:
        base = [] if pts_us is None else list(np.round((pts_us - pts_us[0]) * ts / 1e6).astype(np.int64))
        step = ts // 240
        while len(base) < n:
            base.append((base[-1] + step) if base else 0)
        pts = np.array(base, dtype=np.int64)
    if np.any(np.diff(pts) <= 0):
        raise SystemExit("non-increasing pts")
    out = av.open(out_path, "w", format="mp4")
    # stream copy: take codec parameters (incl. avcC extradata) from the reference track.
    # (add_stream("h264") + extradata writes an empty mdat in PyAV 13+, so do not use it.)
    ref = av.open(ref_path)
    st = out.add_stream_from_template(ref.streams.video[0])
    st.time_base = __import__("fractions").Fraction(1, ts)
    if bytes(st.codec_context.extradata or b"") != bytes(info["avcC"]):
        raise SystemExit("template extradata differs from parsed avcC")
    with open(broken, "rb") as f:
        for i, (o, ln, idr, _t) in enumerate(aus):
            f.seek(o)
            pkt = av.Packet(f.read(ln))
            pkt.stream = st
            pkt.time_base = st.time_base
            pkt.pts = pkt.dts = int(pts[i])
            if i + 1 < n:
                pkt.duration = int(pts[i + 1] - pts[i])
            else:
                pkt.duration = ts // 240
            pkt.is_keyframe = bool(idr)
            out.mux(pkt)
    out.close()
    ref.close()
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("broken"); ap.add_argument("reference"); ap.add_argument("out")
    ap.add_argument("--csv", help="frames_camN.csv with pts_us (default: next to broken file)")
    ap.add_argument("--scan-only", action="store_true")
    ap.add_argument("--keep-untimed", action="store_true",
                    help="also keep trailing frames that have no CSV row (pts extrapolated at 240 fps); "
                         "default drops them so video frames == CSV rows")
    a = ap.parse_args()
    if os.path.abspath(a.out) == os.path.abspath(a.broken):
        raise SystemExit("refusing to overwrite the broken input")
    info = read_reference(a.reference)
    print("reference:", {k: v for k, v in info.items() if k not in ("avcC", "sps", "pps")},
          "sps", [s.hex() for s in info["sps"]], "pps", [p.hex() for p in info["pps"]], flush=True)
    aus, stats = scan_nals(a.broken, info["length_size"])
    n_idr = sum(1 for x in aus if x[2])
    gop = [i for i, x in enumerate(aus) if x[2]]
    stats.update(access_units=len(aus), idr_frames=n_idr, first_au_idr=bool(aus and aus[0][2]),
                 multi_nal_aus=sum(1 for x in aus if len(x[3]) > 1),
                 idr_spacing=sorted(set(np.diff(gop).tolist()))[:10] if len(gop) > 1 else [])
    csv = a.csv or os.path.join(os.path.dirname(a.broken), "frames_cam5.csv")
    pts_us = read_pts_us(csv) if os.path.exists(csv) else None
    stats["csv_complete_rows"] = None if pts_us is None else int(len(pts_us))
    print(json.dumps(stats, indent=1), flush=True)
    if a.scan_only or not aus:
        return
    if not aus[0][2]:
        raise SystemExit("first AU is not IDR; refusing (would need to skip to first IDR)")
    if pts_us is not None and len(aus) > len(pts_us) and not a.keep_untimed:
        print(f"dropping {len(aus) - len(pts_us)} trailing frames without CSV timestamps", flush=True)
        aus = aus[:len(pts_us)]
    pts = mux(a.broken, aus, info, pts_us, a.out, a.reference)
    print(json.dumps({"out": a.out, "frames": len(aus), "duration_s": float(pts[-1]) / info["timescale"],
                      "out_bytes": os.path.getsize(a.out)}), flush=True)


if __name__ == "__main__":
    main()
