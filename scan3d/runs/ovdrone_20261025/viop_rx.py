#!/usr/bin/env python3
"""PC receiver / decoder / checker for VIOP v2 frames (scan3d/ov_bridge/viop_proto.h, 2026-10-25, ovdrone).
No dependencies (pyserial only for `serial`). Mirrors viop_proto.h: sync A5 5A, version 2, type, len, src, seq,
payload, CRC-16/CCITT-FALSE over [2, 8+len).

  python viop_rx.py listen --port 14556 --secs 120 --out DIR   # UDP from the phone (bench); 2 Hz TIME_REQ pings
  python viop_rx.py serial --port COM5 --baud 921600 --secs 120 --out DIR
        # byte stream from the PC-side ESP32 (ESP-NOW relay) over USB serial; no pings (clock via the relay is a later
        # design step: the drone ESP32 would forward TIME_REQ/RESP); latency then = arrival - t_pred with the offset
        # taken as min(arrival - t_pred) (relative jitter only, labelled as such)
  python viop_rx.py check DIR                                  # rate / jitter / latency / frame sanity -> DIR/check.json
Capture: DIR/raw.bin (records: int64 pc_ns, uint16 len, bytes = one datagram or one serial read), DIR/pose.csv,
DIR/ping.csv, DIR/meta.json. PC clock = time.perf_counter_ns.
"""
import argparse
import json
import math
import os
import socket
import struct
import sys
import time

SYNC = b"\xA5\x5A"
VER = 2
T_POSE, T_TREQ, T_TRESP = 0x01, 0x10, 0x11
POSE_FMT = "<QIHH3f3f4h3f3H3H3HBBH"
assert struct.calcsize(POSE_FMT) == 82
POSE_F = ["t_pred_us", "send_delay_us", "horizon_100us", "flags", "px", "py", "pz", "vx", "vy", "vz", "qw", "qx", "qy", "qz",
          "wx", "wy", "wz", "sp_x", "sp_y", "sp_z", "sv_x", "sv_y", "sv_z", "sa_x", "sa_y", "sa_z", "reset_counter", "quality",
          "n_feat"]


def crc16(d):
    crc = 0xFFFF
    for b in d:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build(typ, src, seq, payload):
    h = bytes([VER, typ, len(payload), src]) + struct.pack("<H", seq & 0xFFFF)
    return SYNC + h + payload + struct.pack("<H", crc16(h + payload))


def check_frame(f):
    """(type, src, seq, payload) or None"""
    if len(f) < 10 or f[:2] != SYNC or f[2] != VER or len(f) != 10 + f[4]:
        return None
    if crc16(f[2:8 + f[4]]) != struct.unpack("<H", f[8 + f[4]:10 + f[4]])[0]:
        return None
    return f[3], f[5], struct.unpack("<H", f[6:8])[0], f[8:8 + f[4]]


class Stream:
    """byte-stream parser, same rules as viop_stream_next(): bad sync / CRC drops one byte and rescans"""
    def __init__(self):
        self.buf = bytearray()
        self.ok = self.bad = self.dropped = 0

    def push(self, d):
        self.buf += d
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xA5" else 0
                self.dropped += len(self.buf) - keep
                self.buf = self.buf[len(self.buf) - keep:]
                return out
            if i:
                self.dropped += i
                del self.buf[:i]
            if len(self.buf) < 10:
                return out
            if self.buf[2] != VER or self.buf[4] > 86:
                self.bad += 1; self.dropped += 1; del self.buf[:1]; continue
            fs = 10 + self.buf[4]
            if len(self.buf) < fs:
                return out
            r = check_frame(bytes(self.buf[:fs]))
            if r is None:
                self.bad += 1; self.dropped += 1; del self.buf[:1]; continue
            self.ok += 1
            out.append(r)
            del self.buf[:fs]


def pose(payload):
    v = dict(zip(POSE_F, struct.unpack(POSE_FMT, payload)))
    for k in ("qw", "qx", "qy", "qz"):
        v[k] /= 32767.0
    return v


class Cap:
    def __init__(self, out):
        os.makedirs(out, exist_ok=True)
        self.out = out
        self.raw = open(os.path.join(out, "raw.bin"), "wb")
        self.fp = open(os.path.join(out, "pose.csv"), "w")
        self.fp.write("pc_ns,src,seq," + ",".join(POSE_F) + "\n")
        self.fq = open(os.path.join(out, "ping.csv"), "w")
        self.fq.write("pc_send_ns,pc_recv_ns,phone_rx_ns,rtt_ms\n")
        self.n = 0
        self.types = {}

    def frame(self, pc, r, pend):
        typ, src, seq, pl = r
        self.types[typ] = self.types.get(typ, 0) + 1
        if typ == T_POSE and len(pl) == 82:
            v = pose(pl)
            self.n += 1
            self.fp.write(f"{pc},{src},{seq}," + ",".join(repr(v[k]) for k in POSE_F) + "\n")
        elif typ == T_TRESP and len(pl) == 16:
            treq, tph = struct.unpack("<QQ", pl)
            if treq in pend:
                pend.discard(treq)
                self.fq.write(f"{treq},{pc},{tph},{(pc - treq) / 1e6:.3f}\n")

    def close(self, meta):
        for f in (self.raw, self.fp, self.fq):
            f.close()
        meta.update(made=time.strftime("%F %T"), pose_frames=self.n, types={hex(k): c for k, c in self.types.items()},
                    clock="time.perf_counter_ns", argv=sys.argv)
        json.dump(meta, open(os.path.join(self.out, "meta.json"), "w"), indent=1)
        print("saved", meta)


def listen(a):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((a.bind, a.port))
    s.settimeout(0.05)
    cap = Cap(a.out)
    bad, peer, next_ping, pseq, pend, last = 0, None, 0.0, 0, set(), time.perf_counter()
    t_end = time.perf_counter() + a.secs
    while time.perf_counter() < t_end:
        if peer and time.perf_counter() >= next_ping:
            t = time.perf_counter_ns()
            pend.add(t)
            s.sendto(build(T_TREQ, 3, pseq, struct.pack("<QQ", t, 0)), peer)
            pseq += 1
            next_ping = time.perf_counter() + a.ping_period
        try:
            d, addr = s.recvfrom(2048)
        except socket.timeout:
            continue
        pc = time.perf_counter_ns()
        cap.raw.write(struct.pack("<qH", pc, len(d)) + d)
        r = check_frame(d)
        if r is None:
            bad += 1
            continue
        peer = addr
        cap.frame(pc, r, pend)
        if time.perf_counter() - last > 5:
            last = time.perf_counter()
            print(f"{time.strftime('%T')} pose {cap.n} bad {bad} peer {peer}", flush=True)
    cap.close(dict(mode="udp", port=a.port, secs=a.secs, bad_datagrams=bad, peer=str(peer)))


def serial_mode(a):
    import serial  # pyserial
    sp = serial.Serial(a.port, a.baud, timeout=0.02)
    cap = Cap(a.out)
    st = Stream()
    t_end = time.perf_counter() + a.secs
    while time.perf_counter() < t_end:
        d = sp.read(4096)
        if not d:
            continue
        pc = time.perf_counter_ns()
        cap.raw.write(struct.pack("<qH", pc, len(d)) + d)
        for r in st.push(d):
            cap.frame(pc, r, set())
    cap.close(dict(mode="serial", port=a.port, baud=a.baud, secs=a.secs, stream_ok=st.ok, stream_bad=st.bad,
                   stream_dropped_bytes=st.dropped))


def read_csv(p):
    with open(p) as f:
        h = f.readline().strip().split(",")
        def num(x):
            try:
                return float(x)
            except ValueError:
                return x
        return [dict(zip(h, map(num, l.strip().split(",")))) for l in f if l.count(",") == len(h) - 1]


def pct(x, q):
    x = sorted(x)
    return x[min(len(x) - 1, int(q * len(x)))] if x else float("nan")


def rot(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def rms(x):
    return round(math.sqrt(sum(e * e for e in x) / len(x)), 4) if x else None


def check(a):
    D = a.dir
    v = read_csv(os.path.join(D, "pose.csv"))
    pg = read_csv(os.path.join(D, "ping.csv")) if os.path.exists(os.path.join(D, "ping.csv")) else []
    meta = json.load(open(os.path.join(D, "meta.json"))) if os.path.exists(os.path.join(D, "meta.json")) else {}
    rep = dict(dir=D, made=time.strftime("%F %T"), pose_frames=len(v), meta=meta)
    if len(v) < 3:
        print(rep)
        return
    pc = [r["pc_ns"] for r in v]
    tp = [r["t_pred_us"] * 1000 for r in v]
    ts = [tp[i] + v[i]["send_delay_us"] * 1000 for i in range(len(v))]
    seq = [int(r["seq"]) for r in v]
    ga = [(pc[i + 1] - pc[i]) / 1e6 for i in range(len(v) - 1)]
    gs = [(tp[i + 1] - tp[i]) / 1e6 for i in range(len(v) - 1)]
    dur = (pc[-1] - pc[0]) / 1e9
    m = sum(ga) / len(ga)
    lost = sum((seq[i + 1] - seq[i] - 1) % 65536 for i in range(len(seq) - 1))
    rep["rate"] = dict(duration_s=round(dur, 2), frames_per_s=round((len(v) - 1) / dur, 2), lost_by_seq=lost,
                       arrival_gap_ms_p50=round(pct(ga, .5), 2), arrival_gap_ms_p95=round(pct(ga, .95), 2),
                       arrival_gap_ms_max=round(max(ga), 2),
                       arrival_gap_std_ms=round(math.sqrt(sum((g - m) ** 2 for g in ga) / len(ga)), 2),
                       t_pred_gap_ms_p50=round(pct(gs, .5), 2), t_pred_gap_ms_p95=round(pct(gs, .95), 2),
                       t_pred_gap_ms_max=round(max(gs), 2), non_increasing_t_pred=sum(1 for g in gs if g <= 0))
    age = [r["send_delay_us"] / 1000 for r in v]
    rep["phone_internal"] = dict(send_minus_t_pred_ms_p50=round(pct(age, .5), 2), p95=round(pct(age, .95), 2),
                                 max=round(max(age), 2), horizon_ms_p50=round(pct([r["horizon_100us"] / 10 for r in v], .5), 1),
                                 horizon_ms_p95=round(pct([r["horizon_100us"] / 10 for r in v], .95), 1))
    if pg:
        best = sorted(pg, key=lambda r: r["rtt_ms"])[:max(3, len(pg) // 10)]
        offs = sorted(r["phone_rx_ns"] - (r["pc_send_ns"] + r["pc_recv_ns"]) / 2 for r in best)
        off = offs[len(offs) // 2]
        lp = [(pc[i] - (tp[i] - off)) / 1e6 for i in range(len(v))]
        ln = [(pc[i] - (ts[i] - off)) / 1e6 for i in range(len(v))]
        rep["latency"] = dict(pings=len(pg), rtt_ms_min=round(min(r["rtt_ms"] for r in pg), 2),
                              rtt_ms_p50=round(pct([r["rtt_ms"] for r in pg], .5), 2),
                              offset_uncertainty_ms=round(min(r["rtt_ms"] for r in pg) / 2, 2),
                              offset_spread_best_ms=round((offs[-1] - offs[0]) / 1e6, 2),
                              arrival_minus_t_pred_ms_p50=round(pct(lp, .5), 2), p95=round(pct(lp, .95), 2), max=round(max(lp), 2),
                              network_ms_p50=round(pct(ln, .5), 2), network_ms_p95=round(pct(ln, .95), 2),
                              network_ms_max=round(max(ln), 2), network_ms_min=round(min(ln), 2))
    else:
        rel = [(pc[i] - tp[i]) / 1e6 for i in range(len(v))]
        mn = min(rel)
        rel = [x - mn for x in rel]
        rep["latency_relative_only"] = dict(note="no pings: arrival - t_pred minus its minimum (jitter, not absolute)",
                                            p50=round(pct(rel, .5), 2), p95=round(pct(rel, .95), 2), max=round(max(rel), 2))
    qn = max(abs(math.sqrt(r["qw"] ** 2 + r["qx"] ** 2 + r["qy"] ** 2 + r["qz"] ** 2) - 1) for r in v)
    ve, vn, we, wn = [], [], [], []
    for i in range(1, len(v) - 1):
        dt = (tp[i + 1] - tp[i - 1]) / 1e9
        if 0 < dt < 0.1 and v[i + 1]["reset_counter"] == v[i - 1]["reset_counter"]:
            fd = [(v[i + 1][k] - v[i - 1][k]) / dt for k in ("px", "py", "pz")]
            ve.append(math.sqrt(sum((fd[j] - v[i][k]) ** 2 for j, k in enumerate(("vx", "vy", "vz")))))
            vn.append(math.sqrt(sum(v[i][k] ** 2 for k in ("vx", "vy", "vz"))))
    for i in range(len(v) - 1):
        dt = (tp[i + 1] - tp[i]) / 1e9
        if not 0 < dt < 0.05:
            continue
        A, B = rot(v[i]["qw"], v[i]["qx"], v[i]["qy"], v[i]["qz"]), rot(v[i + 1]["qw"], v[i + 1]["qx"], v[i + 1]["qy"], v[i + 1]["qz"])
        Rr = [[sum(A[k][r] * B[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
        wf = [(Rr[2][1] - Rr[1][2]) / (2 * dt), (Rr[0][2] - Rr[2][0]) / (2 * dt), (Rr[1][0] - Rr[0][1]) / (2 * dt)]
        wr = [(v[i][k] + v[i + 1][k]) / 2 for k in ("wx", "wy", "wz")]
        we.append(math.sqrt(sum((wf[j] - wr[j]) ** 2 for j in range(3))))
        wn.append(math.sqrt(sum(x * x for x in wr)))
    flags = {}
    for r in v:
        flags[hex(int(r["flags"]))] = flags.get(hex(int(r["flags"])), 0) + 1
    rep["frames"] = dict(
        quat_norm_err_max=float(f"{qn:.2e}"), vel_consistency_rms_m_s=rms(ve), vel_norm_rms_m_s=rms(vn),
        rate_consistency_rms_rad_s=rms(we), rate_norm_rms_rad_s=rms(wn),
        pos_range_m=[round(max(r[k] for r in v) - min(r[k] for r in v), 4) for k in ("px", "py", "pz")],
        first_pos=[round(v[0][k], 4) for k in ("px", "py", "pz")], last_pos=[round(v[-1][k], 4) for k in ("px", "py", "pz")],
        pos_std_mm_p50=[pct([r[k] for r in v], .5) for k in ("sp_x", "sp_y", "sp_z")],
        att_std_deg_p50=[round(math.degrees(pct([r[k] for r in v], .5) * 1e-4), 3) for k in ("sa_x", "sa_y", "sa_z")],
        quality_min_p50=[int(min(r["quality"] for r in v)), int(pct([r["quality"] for r in v], .5))],
        n_feat_p50=int(pct([r["n_feat"] for r in v], .5)), flags=flags, reset_counters=sorted(set(int(r["reset_counter"]) for r in v)))
    # gravity check needs the specific force: not in the packet; use the bridge's --viop-log (f_I in IMU frame)
    if a.bridge_log and os.path.exists(a.bridge_log):
        bl = read_csv(a.bridge_log)
        fw = []
        for r in bl:
            M = rot(r["qw"], r["qx"], r["qy"], r["qz"])
            fw.append([sum(M[i][j] * r[k] for j, k in enumerate(("f_bx", "f_by", "f_bz"))) for i in range(3)])
        rep["gravity_check"] = dict(specific_force_world_mean=[round(sum(f[i] for f in fw) / len(fw), 3) for i in range(3)],
                                    expect="(0, 0, +9.81): world z up, q = IMU -> world", rows=len(fw))
        byt = {int(r["time_usec"]): r for r in bl}
        dmax, nm = 0.0, 0
        for r in v:
            b = byt.get(int(r["t_pred_us"]))
            if b is None:
                continue
            nm += 1
            dmax = max(dmax, max(abs(r[k] - b[kb]) for k, kb in (("px", "x"), ("py", "y"), ("pz", "z"), ("vx", "vx"),
                                                                    ("vy", "vy"), ("vz", "vz"))),
                       max(abs(abs(r["qw"] * b["qw"] + r["qx"] * b["qx"] + r["qy"] * b["qy"] + r["qz"] * b["qz"]) - 1) for _ in [0]))
        rep["transport_vs_bridge_log"] = dict(matched=nm, max_abs_diff=float(f"{dmax:.2e}"))
    json.dump(rep, open(os.path.join(D, "check.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    l = sp.add_parser("listen")
    l.add_argument("--port", type=int, default=14556)
    l.add_argument("--bind", default="0.0.0.0")
    l.add_argument("--secs", type=float, default=120)
    l.add_argument("--ping-period", type=float, default=0.5)
    l.add_argument("--out", required=True)
    s = sp.add_parser("serial")
    s.add_argument("--port", required=True)
    s.add_argument("--baud", type=int, default=921600)
    s.add_argument("--secs", type=float, default=120)
    s.add_argument("--out", required=True)
    c = sp.add_parser("check")
    c.add_argument("dir")
    c.add_argument("--bridge-log")
    a = ap.parse_args()
    {"listen": listen, "serial": serial_mode, "check": check}[a.cmd](a)


if __name__ == "__main__":
    main()
