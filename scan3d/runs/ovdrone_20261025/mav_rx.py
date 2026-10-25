#!/usr/bin/env python3
"""PC-side receiver / checker for the ov_bridge --mav-udp drone output (2026-10-25, branch ovdrone-20261025).

No dependencies (hand MAVLink 2 decoder with CRC check; `--pymavlink-check RAW` re-decodes a saved capture with pymavlink
when it is installed, e.g. in WSL). It never talks to a flight controller; it only listens on a UDP port and answers
nothing except its own TIMESYNC requests to the sender.

  python mav_rx.py listen --port 14555 --secs 60 --out DIR      # capture + live summary every 5 s, files in DIR
  python mav_rx.py check DIR [--bridge-log mav_log.csv]         # rate / latency / frame-convention report (+ json)
  python3 mav_rx.py pymavlink-check DIR                         # same capture decoded by pymavlink (field equality)

Capture files: DIR/raw.bin (per datagram: int64 pc_ns, uint16 len, bytes), DIR/odom.csv, DIR/timesync.csv, DIR/hb.csv.
Latency = PC arrival - (time_usec on the phone boot clock mapped to the PC clock with the TIMESYNC offset). The offset
is taken from the TIMESYNC round trips with the smallest RTT (uncertainty <= RTT_min / 2, printed). PC clock =
time.perf_counter_ns (monotonic).
"""
import argparse
import json
import math
import os
import socket
import struct
import sys
import time

CRC_EXTRA = {0: 50, 331: 91, 102: 158, 111: 34}


def crc_acc(b, crc):
    tmp = (b ^ (crc & 0xFF)) & 0xFF
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc16(data, extra):
    crc = 0xFFFF
    for b in data:
        crc = crc_acc(b, crc)
    return crc_acc(extra, crc)


def frames(dgram):
    """yield (msgid, sysid, compid, seq, payload) for every valid MAVLink 2 frame in a datagram"""
    i = 0
    while i + 12 <= len(dgram):
        if dgram[i] != 0xFD:
            i += 1
            continue
        ln = dgram[i + 1]
        end = i + 12 + ln
        if end > len(dgram):
            break
        if dgram[i + 2] & 1:  # signed
            end += 13
        msgid = dgram[i + 7] | (dgram[i + 8] << 8) | (dgram[i + 9] << 16)
        if msgid in CRC_EXTRA:
            c = crc16(dgram[i + 1:i + 10 + ln], CRC_EXTRA[msgid])
            if c == (dgram[i + 10 + ln] | (dgram[i + 11 + ln] << 8)):
                yield msgid, dgram[i + 5], dgram[i + 6], dgram[i + 4], bytes(dgram[i + 10:i + 10 + ln])
        i = end


def pad(p, n):
    return p + b"\0" * (n - len(p)) if len(p) < n else p


def dec_odom(p):
    p = pad(p, 233)
    v = struct.unpack("<Q3f4f3f3f21f21fBBBBb", p[:233])
    return dict(time_usec=v[0], x=v[1], y=v[2], z=v[3], qw=v[4], qx=v[5], qy=v[6], qz=v[7], vx=v[8], vy=v[9], vz=v[10],
                wx=v[11], wy=v[12], wz=v[13], pcov=list(v[14:35]), vcov=list(v[35:56]), frame_id=v[56], child_frame_id=v[57],
                reset_counter=v[58], estimator_type=v[59], quality=v[60])


def dec_vpe(p):
    v = struct.unpack("<Q6f21fB", pad(p, 117)[:117])
    return dict(usec=v[0], x=v[1], y=v[2], z=v[3], roll=v[4], pitch=v[5], yaw=v[6], cov=list(v[7:28]), reset_counter=v[28])


def mk_timesync(tc1, ts1, seq):
    pl = struct.pack("<qq", tc1, ts1)
    while len(pl) > 1 and pl[-1] == 0:
        pl = pl[:-1]
    hdr = bytes([len(pl), 0, 0, seq & 0xFF, 255, 190, 111, 0, 0])
    c = crc16(hdr + pl, 34)
    return b"\xFD" + hdr + pl + struct.pack("<H", c)


def listen(a):
    os.makedirs(a.out, exist_ok=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((a.bind, a.port))
    s.settimeout(0.05)
    raw = open(os.path.join(a.out, "raw.bin"), "wb")
    fo = open(os.path.join(a.out, "odom.csv"), "w")
    fo.write("pc_ns,seq,time_usec,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id,reset_counter,estimator_type,"
             "quality,pcov_xx,pcov_yy,pcov_zz,pcov_rr,pcov_pp,pcov_yawyaw,vcov_xx\n")
    ft = open(os.path.join(a.out, "timesync.csv"), "w")
    ft.write("pc_send_ns,pc_recv_ns,phone_boot_ns,rtt_ms\n")
    fh = open(os.path.join(a.out, "hb.csv"), "w")
    fh.write("pc_ns,sysid,compid,type,autopilot,system_status\n")
    meta = dict(made=time.strftime("%F %T"), port=a.port, secs=a.secs, clock="time.perf_counter_ns", argv=sys.argv)
    t_end = time.perf_counter() + a.secs
    peer, next_ts, ts_seq, n_odom, n_vpe, n_bad, last_print = None, 0.0, 0, 0, 0, 0, time.perf_counter()
    pending = {}
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        if peer and now >= next_ts:
            t_send = time.perf_counter_ns()
            pending[t_send] = True
            s.sendto(mk_timesync(0, t_send, ts_seq), peer)
            ts_seq += 1
            next_ts = now + a.ts_period
        try:
            d, addr = s.recvfrom(2048)
        except (socket.timeout, ConnectionResetError):  # Windows: ICMP port unreachable after the sender exits
            continue
        pc = time.perf_counter_ns()
        raw.write(struct.pack("<qH", pc, len(d)) + d)
        peer = addr
        got = False
        for msgid, sysid, compid, seq, pl in frames(d):
            got = True
            if msgid == 331:
                o = dec_odom(pl)
                n_odom += 1
                fo.write(",".join(str(x) for x in [pc, seq, o["time_usec"], o["x"], o["y"], o["z"], o["qw"], o["qx"], o["qy"],
                                                   o["qz"], o["vx"], o["vy"], o["vz"], o["wx"], o["wy"], o["wz"], o["frame_id"],
                                                   o["child_frame_id"], o["reset_counter"], o["estimator_type"], o["quality"],
                                                   o["pcov"][0], o["pcov"][6], o["pcov"][11], o["pcov"][15], o["pcov"][18],
                                                   o["pcov"][20], o["vcov"][0]]) + "\n")
            elif msgid == 102:
                n_vpe += 1
            elif msgid == 111:
                tc1, ts1 = struct.unpack("<qq", pad(pl, 16)[:16])
                if tc1 != 0 and ts1 in pending:
                    del pending[ts1]
                    ft.write(f"{ts1},{pc},{tc1},{(pc - ts1) / 1e6:.3f}\n")
            elif msgid == 0:
                cm, typ, ap, bm, st, ver = struct.unpack("<IBBBBB", pad(pl, 9)[:9])
                fh.write(f"{pc},{sysid},{compid},{typ},{ap},{st}\n")
        if not got:
            n_bad += 1
        if time.perf_counter() - last_print > 5.0:
            last_print = time.perf_counter()
            print(f"{time.strftime('%T')} odom {n_odom} vpe {n_vpe} bad {n_bad} peer {peer}", flush=True)
    for f in (raw, fo, ft, fh):
        f.close()
    meta.update(n_odom=n_odom, n_vpe=n_vpe, n_bad_datagrams=n_bad, peer=str(peer))
    json.dump(meta, open(os.path.join(a.out, "meta.json"), "w"), indent=1)
    print("saved", a.out, meta)


def read_csv(path):
    with open(path) as f:
        hdr = f.readline().strip().split(",")
        rows = []
        for line in f:
            p = line.strip().split(",")
            if len(p) != len(hdr):
                continue
            r = {}
            for k, v in zip(hdr, p):
                try:
                    r[k] = float(v)
                except ValueError:
                    r[k] = v
            rows.append(r)
    return rows


def pct(xs, q):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def qrot(qw, qx, qy, qz):
    """rotation matrix of a (w, x, y, z) quaternion (body -> local)"""
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    w, x, y, z = qw / n, qx / n, qy / n, qz / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def mv(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def check(a):
    D = a.dir
    od = read_csv(os.path.join(D, "odom.csv"))
    ts = read_csv(os.path.join(D, "timesync.csv")) if os.path.exists(os.path.join(D, "timesync.csv")) else []
    hb = read_csv(os.path.join(D, "hb.csv")) if os.path.exists(os.path.join(D, "hb.csv")) else []
    rep = dict(dir=D, made=time.strftime("%F %T"), n_odom=len(od), n_heartbeat=len(hb))
    if len(od) < 3:
        print("too few ODOMETRY", len(od))
        return
    pc = [r["pc_ns"] for r in od]
    tu = [r["time_usec"] for r in od]
    dur = (pc[-1] - pc[0]) / 1e9
    gaps = [(pc[i + 1] - pc[i]) / 1e6 for i in range(len(pc) - 1)]
    sgaps = [(tu[i + 1] - tu[i]) / 1e3 for i in range(len(tu) - 1)]
    seqs = [int(r["seq"]) for r in od]
    rep["rate"] = dict(duration_s=round(dur, 2), msgs_per_s=round((len(od) - 1) / dur, 2),
                       arrival_gap_ms_p50=round(pct(gaps, .5), 2), p95=round(pct(gaps, .95), 2), max=round(max(gaps), 2),
                       stamp_gap_ms_p50=round(pct(sgaps, .5), 2), stamp_gap_p95=round(pct(sgaps, .95), 2),
                       stamp_gap_max=round(max(sgaps), 2), non_increasing_stamps=sum(1 for g in sgaps if g <= 0))
    # latency via TIMESYNC
    if ts:
        rtt = sorted(ts, key=lambda r: r["rtt_ms"])
        best = rtt[:max(3, len(rtt) // 10)]
        offs = [r["phone_boot_ns"] - (r["pc_send_ns"] + r["pc_recv_ns"]) / 2 for r in best]  # phone - pc
        off = sorted(offs)[len(offs) // 2]
        lat = [(r["pc_ns"] - (r["time_usec"] * 1000 - off)) / 1e6 for r in od]
        rep["latency"] = dict(timesync_n=len(ts), rtt_ms_min=round(rtt[0]["rtt_ms"], 2),
                              rtt_ms_p50=round(pct([r["rtt_ms"] for r in ts], .5), 2),
                              offset_uncertainty_ms=round(rtt[0]["rtt_ms"] / 2, 2),
                              offset_spread_ms_best=round((max(offs) - min(offs)) / 1e6, 2),
                              arrival_minus_stamp_ms_p50=round(pct(lat, .5), 2), p95=round(pct(lat, .95), 2),
                              max=round(max(lat), 2), min=round(min(lat), 2))
    # frame-convention sanity
    qn = [abs(math.sqrt(r["qw"] ** 2 + r["qx"] ** 2 + r["qy"] ** 2 + r["qz"] ** 2) - 1) for r in od]
    rpy = []
    for r in od:
        R = qrot(r["qw"], r["qx"], r["qy"], r["qz"])
        rpy.append((math.degrees(math.atan2(R[2][1], R[2][2])), math.degrees(-math.asin(max(-1, min(1, R[2][0])))),
                    math.degrees(math.atan2(R[1][0], R[0][0]))))
    spd = [math.sqrt(r["vx"] ** 2 + r["vy"] ** 2 + r["vz"] ** 2) for r in od]
    # velocity consistency: finite-difference local position vs R_LB v_B (on the sensor time axis)
    errs, nrm = [], []
    for i in range(1, len(od) - 1):
        dt = (tu[i + 1] - tu[i - 1]) / 1e6
        if dt <= 0 or dt > 0.1 or od[i + 1]["reset_counter"] != od[i - 1]["reset_counter"]:
            continue
        fd = [(od[i + 1][k] - od[i - 1][k]) / dt for k in ("x", "y", "z")]
        R = qrot(od[i]["qw"], od[i]["qx"], od[i]["qy"], od[i]["qz"])
        vl = mv(R, [od[i]["vx"], od[i]["vy"], od[i]["vz"]])
        errs.append(math.sqrt(sum((fd[k] - vl[k]) ** 2 for k in range(3))))
        nrm.append(math.sqrt(sum(vl[k] ** 2 for k in range(3))))
    # rate consistency: body rate from consecutive attitudes (R_i^T R_{i+1} = Exp(w dt)) vs reported body rate
    werr, wn = [], []
    for i in range(len(od) - 1):
        dt = (tu[i + 1] - tu[i]) / 1e6
        if dt <= 0 or dt > 0.05:
            continue
        A = qrot(od[i]["qw"], od[i]["qx"], od[i]["qy"], od[i]["qz"])
        B = qrot(od[i + 1]["qw"], od[i + 1]["qx"], od[i + 1]["qy"], od[i + 1]["qz"])
        Rr = [[sum(A[k][r] * B[k][c] for k in range(3)) for c in range(3)] for r in range(3)]  # A^T B
        wfd = [(Rr[2][1] - Rr[1][2]) / (2 * dt), (Rr[0][2] - Rr[2][0]) / (2 * dt), (Rr[1][0] - Rr[0][1]) / (2 * dt)]
        wr = [(od[i][k] + od[i + 1][k]) / 2 for k in ("wx", "wy", "wz")]
        werr.append(math.sqrt(sum((wfd[k] - wr[k]) ** 2 for k in range(3))))
        wn.append(math.sqrt(sum(wr[k] ** 2 for k in range(3))))
    rep["frames"] = dict(
        frame_id=sorted(set(int(r["frame_id"]) for r in od)), child_frame_id=sorted(set(int(r["child_frame_id"]) for r in od)),
        estimator_type=sorted(set(int(r["estimator_type"]) for r in od)), reset_counters=sorted(set(int(r["reset_counter"]) for r in od)),
        quat_norm_err_max=float(f"{max(qn):.2e}"),
        roll_deg=[round(min(x[0] for x in rpy), 1), round(pct([x[0] for x in rpy], .5), 1), round(max(x[0] for x in rpy), 1)],
        pitch_deg=[round(min(x[1] for x in rpy), 1), round(pct([x[1] for x in rpy], .5), 1), round(max(x[1] for x in rpy), 1)],
        yaw_deg=[round(min(x[2] for x in rpy), 1), round(pct([x[2] for x in rpy], .5), 1), round(max(x[2] for x in rpy), 1)],
        first_pos=[round(od[0][k], 4) for k in ("x", "y", "z")], last_pos=[round(od[-1][k], 4) for k in ("x", "y", "z")],
        pos_range_m=[round(max(r[k] for r in od) - min(r[k] for r in od), 4) for k in ("x", "y", "z")],
        speed_body_m_s_p50=round(pct(spd, .5), 4), speed_p95=round(pct(spd, .95), 4),
        vel_consistency_rms_m_s=round(math.sqrt(sum(e * e for e in errs) / len(errs)), 4) if errs else None,
        vel_norm_rms_m_s=round(math.sqrt(sum(e * e for e in nrm) / len(nrm)), 4) if nrm else None,
        rate_consistency_rms_rad_s=round(math.sqrt(sum(e * e for e in werr) / len(werr)), 4) if werr else None,
        rate_norm_rms_rad_s=round(math.sqrt(sum(e * e for e in wn) / len(wn)), 4) if wn else None,
        quality=[int(min(r["quality"] for r in od)), int(pct([r["quality"] for r in od], .5)), int(max(r["quality"] for r in od))],
        sigma_pos_m_p50=round(math.sqrt(pct([(r["pcov_xx"] + r["pcov_yy"] + r["pcov_zz"]) / 3 for r in od], .5)), 4))
    if hb:
        hp = [r["pc_ns"] for r in hb]
        rep["heartbeat"] = dict(n=len(hb), period_s=round((hp[-1] - hp[0]) / 1e9 / max(1, len(hb) - 1), 3),
                                status=sorted(set(int(r["system_status"]) for r in hb)),
                                sys_comp=sorted(set(f"{int(r['sysid'])}/{int(r['compid'])}" for r in hb)))
    # gravity check with the bridge's own log (specific force in body frame): R_LB f_B should be (0, 0, -g) when still
    if a.bridge_log and os.path.exists(a.bridge_log):
        bl = read_csv(a.bridge_log)
        fl = []
        for r in bl:
            R = qrot(r["qw"], r["qx"], r["qy"], r["qz"])
            fl.append(mv(R, [r["f_bx"], r["f_by"], r["f_bz"]]))
        n = len(fl)
        mean = [sum(f[k] for f in fl) / n for k in range(3)]
        still = [f for f, r in zip(fl, bl) if math.sqrt(r["vx"] ** 2 + r["vy"] ** 2 + r["vz"] ** 2) < 0.05]
        ms = [sum(f[k] for f in still) / len(still) for k in range(3)] if still else None
        # transport check: decoded UDP vs the bridge's own log (same time_usec)
        byt = {int(r["time_usec"]): r for r in bl}
        dmax, nm = 0.0, 0
        for r in od:
            b = byt.get(int(r["time_usec"]))
            if b is None:
                continue
            nm += 1
            dmax = max(dmax, max(abs(r[k] - b[k]) for k in ("x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz")))
        ag = [r["age_ms"] for r in bl]
        im = [r["int_ms"] for r in bl if r["int_ms"] >= 0]
        rep["bridge_log"] = dict(rows=n, specific_force_local_mean=[round(x, 3) for x in mean],
                                 specific_force_local_mean_still=[round(x, 3) for x in ms] if ms else None, n_still=len(still),
                                 expect="(0, 0, -9.8) in LOCAL_FRD when the attitude convention is right",
                                 matched_udp_rows=nm, udp_vs_log_max_abs_diff=float(f"{dmax:.2e}"),
                                 age_ms_p50=round(pct(ag, .5), 2), age_p95=round(pct(ag, .95), 2), age_max=round(max(ag), 2),
                                 send_minus_imu_rx_ms_p50=round(pct(im, .5), 3) if im else None,
                                 p95=round(pct(im, .95), 3) if im else None, max=round(max(im), 3) if im else None)
    json.dump(rep, open(os.path.join(D, "check.json"), "w"), indent=1)
    print(json.dumps(rep, indent=1))


def pymavlink_check(a):
    from pymavlink.dialects.v20 import common as m
    mv_ = m.MAVLink(None)
    mv_.robust_parsing = True
    f = open(os.path.join(a.dir, "raw.bin"), "rb")
    n, bad, diff = 0, 0, 0.0
    counts = {}
    while True:
        h = f.read(10)
        if len(h) < 10:
            break
        pc, ln = struct.unpack("<qH", h)
        d = f.read(ln)
        mine = {mid: pl for mid, _, _, _, pl in frames(d)}
        try:
            msgs = mv_.parse_buffer(d) or []
        except Exception:
            bad += 1
            continue
        for msg in msgs:
            counts[msg.get_type()] = counts.get(msg.get_type(), 0) + 1
            if msg.get_type() == "ODOMETRY" and 331 in mine:
                o = dec_odom(mine[331])
                n += 1
                diff = max(diff, abs(o["x"] - msg.x), abs(o["qw"] - msg.q[0]), abs(o["vz"] - msg.vz),
                           abs(o["time_usec"] - msg.time_usec), abs(o["pcov"][20] - msg.pose_covariance[20]),
                           abs(o["quality"] - getattr(msg, "quality", o["quality"])), abs(o["frame_id"] - msg.frame_id))
    r = dict(pymavlink_types=counts, odometry_compared=n, max_abs_diff=diff, parse_errors=bad)
    json.dump(r, open(os.path.join(a.dir, "pymavlink_check.json"), "w"), indent=1)
    print(r)


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest="cmd", required=True)
    l = sp.add_parser("listen")
    l.add_argument("--port", type=int, default=14555)
    l.add_argument("--bind", default="0.0.0.0")
    l.add_argument("--secs", type=float, default=60)
    l.add_argument("--ts-period", type=float, default=0.5)
    l.add_argument("--out", required=True)
    c = sp.add_parser("check")
    c.add_argument("dir")
    c.add_argument("--bridge-log")
    p = sp.add_parser("pymavlink-check")
    p.add_argument("dir")
    a = ap.parse_args()
    {"listen": listen, "check": check, "pymavlink-check": pymavlink_check}[a.cmd](a)


if __name__ == "__main__":
    main()
