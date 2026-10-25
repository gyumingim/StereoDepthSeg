# VIOP UDP listener: per second counts of HEARTBEAT state and POSE TRACKING flag / quality (wire-level check)
import socket, struct, sys, time, json
port, dur, out = int(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", port)); s.settimeout(0.5)
t0 = None; sec = {}
end = time.time() + dur
while time.time() < end:
    try: d, _ = s.recvfrom(2048)
    except socket.timeout: continue
    if len(d) < 10 or d[0] != 0xA5 or d[1] != 0x5A: continue
    typ, ln = d[3], d[4]; p = d[8:8 + ln]
    if t0 is None: t0 = time.time()
    k = int(time.time() - t0); e = sec.setdefault(k, {"hb": {}, "pose_trk": 0, "pose_lost": 0, "qmin": 255, "hmax_ms": 0})
    if typ == 2 and ln >= 18:
        st = p[8]; e["hb"][st] = e["hb"].get(st, 0) + 1
    elif typ == 1 and ln >= 82:
        hz = struct.unpack_from("<H", p, 12)[0] / 10.0; fl = struct.unpack_from("<H", p, 14)[0]; q = p[79]
        if fl & 2: e["pose_trk"] += 1
        else: e["pose_lost"] += 1
        e["qmin"] = min(e["qmin"], q); e["hmax_ms"] = max(e["hmax_ms"], hz)
json.dump(sec, open(out, "w"), indent=0)
for k in sorted(sec): print(k, sec[k])
