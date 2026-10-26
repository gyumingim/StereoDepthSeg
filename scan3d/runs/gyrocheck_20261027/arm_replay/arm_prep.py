#!/usr/bin/env python3
"""gyrocheck arm replay: build the event stream for arm_replay (see arm_replay.c) from a bench poses.jsonl.gz + the
recording imu.csv, run it, summarise. Mapping = ov_bridge mav_out.h emit_viop / hb_build_send (PLAN.md 할 일 2).
usage: arm_prep.py <poses.jsonl.gz> <recording dir> <out prefix> [std_div]   (std_div 1 = h.pos_std, 1.732 = sqrt(trace/3))"""
import sys, gzip, json, csv, subprocess, math
import numpy as np

pf, recd, outp = sys.argv[1], sys.argv[2], sys.argv[3]
std_div = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
BIN = sys.argv[5] if len(sys.argv) > 5 else "/home/system/scanner/tmp_gyrocheck/arm/arm_replay"
rows = list(csv.DictReader(open(f"{recd}/imu.csv")))
acc = np.array([[int(r["t_ns"]), float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == "1"])
gyr = np.array([[int(r["t_ns"]), float(r["v0"]), float(r["v1"]), float(r["v2"])] for r in rows if r["type"] == "16"])
t0 = min(acc[0, 0], gyr[0, 0])
t_end = min(acc[-1, 0], gyr[-1, 0])
# reference "level" = mean gravity direction of the last 2 s (end still; assumption: phone resting = vehicle level)
ref = acc[acc[:, 0] > t_end - 2e9, 1:].mean(0)
ref /= np.linalg.norm(ref)
VIOP = pf.endswith(".viop.bin.gz")  # POST HOC (supervisor C5): real VIOP POSE frames from --viop-udp capture
if VIOP:
    import struct
    raw = gzip.open(pf, "rb").read()
    P, i = [], 0
    while i + 10 <= len(raw):
        n = int.from_bytes(raw[i:i + 2], "little"); fr = raw[i + 10:i + 10 + n]; i += 10 + n
        if len(fr) < 10 or fr[0] != 0xA5 or fr[1] != 0x5A or fr[3] != 0x01:
            continue
        pl = fr[8:8 + fr[4]]
        t_us, = struct.unpack_from("<Q", pl, 0)
        fl, = struct.unpack_from("<H", pl, 14)
        sd = struct.unpack_from("<3H", pl, 60)
        vel = struct.unpack_from("<3f", pl, 28)
        P.append(dict(t_ns=t_us * 1000, init=bool(fl & 1), status="tracking" if fl & 2 else "lost", viop_flags=fl,
                      quality=pl[79], sd_max=max(sd) / 1000.0, sd_axes=[x / 1000.0 for x in sd], v=list(vel)))
    P.sort(key=lambda m: m["t_ns"])
else:
    P = [json.loads(x) for x in gzip.open(pf, "rt") if x.strip()]
    P = [m for m in P if m.get("type", "pose") == "pose"]
ev = []
grid = np.arange(t0, t_end, 10e6)
ia = np.searchsorted(acc[:, 0], grid).clip(0, len(acc) - 1)
ig = np.searchsorted(gyr[:, 0], grid).clip(0, len(gyr) - 1)
# 50 ms low-pass for the tilt (accelerometer direction), raw |a| for the safety's own EMA
k = 25
acs = np.cumsum(np.vstack([np.zeros(3), acc[:, 1:]]), 0)
for j, t in enumerate(grid):
    a = acc[ia[j], 1:]
    g = gyr[ig[j], 1:]
    lo, hi = max(0, ia[j] - k), min(len(acc), ia[j] + 1)
    am = (acs[hi] - acs[lo]) / max(1, hi - lo)
    c = float(np.dot(am, ref) / max(1e-9, np.linalg.norm(am)))
    tilt = math.acos(max(-1.0, min(1.0, c)))
    ev.append((int((t - t0) / 1e6), 2, f"I {int((t - t0) / 1e6)} {np.linalg.norm(g):.5f} {np.linalg.norm(a):.5f} {tilt:.5f}"))
state_at = []  # (t_ms, initialized, lost)
for m in P:
    tm = int((m["t_ns"] - t0) / 1e6)
    if tm < 0:
        continue
    init = bool(m.get("init"))
    lost = m.get("status") != "tracking"
    state_at.append((tm, init, lost))
    if init and VIOP:
        ev.append((tm, 0, f"P {tm} {m['viop_flags'] & 3} {m['quality']} {m['sd_max']:.5f}"))
    elif init:
        h = m.get("h") or {}
        sd = h.get("pos_std")
        sd = float("nan") if sd is None else sd / std_div
        q = 0 if lost or not (sd == sd) else min(100, max(1, round(100 * (1 - sd / 0.20))))
        flags = 1 | (0 if lost else 2)
        ev.append((tm, 0, f"P {tm} {flags} {q} {sd:.5f}"))
# heartbeat 20 Hz from the newest known state (STARTING 0 / TRACKING 1 / LOST 2)
si = 0
last = (False, False)
for tm in range(0, int((t_end - t0) / 1e6), 50):
    while si < len(state_at) and state_at[si][0] <= tm:
        last = state_at[si][1:]
        si += 1
    st = 0 if not last[0] else (2 if last[1] else 1)
    ev.append((tm, 1, f"H {tm} {st}"))
ev.sort(key=lambda x: (x[0], x[1]))
txt = "\n".join(e[2] for e in ev) + "\n"
r = subprocess.run([BIN], input=txt, capture_output=True, text=True, check=True)
open(outp + ".csv", "w").write(r.stdout)
# summary
B = {1 << 0: "STATE", 1 << 1: "NO_INPUT", 1 << 2: "SWITCH", 1 << 3: "IMU", 1 << 4: "TILT", 1 << 5: "THR", 1 << 6: "MOVING",
     1 << 7: "POSE", 1 << 8: "HB", 1 << 9: "LINK", 1 << 10: "BATT", 1 << 11: "OVERRUN", 1 << 12: "PHONE_DISC", 1 << 13: "TOO_SOON"}
out = np.array([[float(x) for x in l.split(",")] for l in r.stdout.splitlines()[1:]])
tms, bl, ok = out[:, 0], out[:, 1].astype(int), out[:, 2].astype(int)
gw = np.array([float(e[2].split()[2]) for e in ev if e[1] == 2])
moving = np.convolve(np.abs(gw - np.median(gw[:200])) > 0.08, np.ones(20), "same") > 0
m_first = tms[np.argmax(moving)] if moving.any() else None
m_last = tms[len(moving) - 1 - np.argmax(moving[::-1])] if moving.any() else None
t_init = next((s[0] for s in state_at if s[1]), None)
segs = dict(still0=(tms < m_first), motion=(tms >= m_first) & (tms <= m_last), still_end=(tms > m_last))


def names(b):
    return "|".join(v for kk, v in B.items() if b & kk) or "0"


summ = dict(poses=pf, rec=recd, std_div=std_div, t_motion_start_s=m_first / 1e3, t_motion_end_s=m_last / 1e3,
            t_vio_init_s=None if t_init is None else t_init / 1e3, t_rec_end_s=float(tms[-1]) / 1e3,
            first_armable_s=float(tms[np.argmax(ok)]) / 1e3 if ok.any() else None)
for sname, msk in segs.items():
    if not msk.any():
        continue
    u, c = np.unique(bl[msk], return_counts=True)
    summ[sname] = dict(dur_s=round(float(msk.sum()) / 100, 2), armable_frac=round(float(ok[msk].mean()), 3),
                       blocks={names(int(x)): int(n) for x, n in sorted(zip(u, c), key=lambda z: -z[1])})
# first armable after motion end + continuous armable run at the end
if ok.any() and m_last is not None:
    after = (tms > m_last) & (ok == 1)
    summ["first_armable_after_motion_s"] = round(float(tms[np.argmax(after)] - m_last) / 1e3, 2) if after.any() else None
    # time of the first armable relative to put-down (motion end)
if VIOP and m_last is not None:
    tp = np.array([(m["t_ns"] - t0) / 1e6 for m in P if m["init"]])
    sdm = np.array([m["sd_max"] for m in P if m["init"]])
    vv = np.array([np.linalg.norm(m["v"]) for m in P if m["init"]])
    e = tp > m_last
    if e.any():
        summ["viop_end_still"] = dict(n=int(e.sum()), sd_max_med=round(float(np.median(sdm[e])), 4), sd_max_min=round(float(sdm[e].min()), 4),
                                      sd_max_max=round(float(sdm[e].max()), 4), speed_med=round(float(np.median(vv[e])), 3),
                                      speed_max=round(float(vv[e].max()), 3), frac_sd_ok=round(float(np.mean(sdm[e] <= 0.15)), 3))
json.dump(summ, open(outp + ".json", "w"), indent=1)
print(json.dumps(summ))
