#!/usr/bin/env python3
"""Paced (real-time) replay of an ov_bridge input stream + receive-time stamping of every output line (PC check of the
0.6.19 prediction latency). Pacing as idle_spin_check_20261025/paced_feed.py (record t_ns of IMU / STEREO records; chunk
records 5/6 go out at the time of the last timed record). A reader thread stamps each bridge output line with the stream
clock (t_first + wall elapsed) -> latency of a pred line = t_rx - t_pred (t_ns), age at a consumer sampled at 1 kHz =
t - t_pred of the newest pred line received by t.
  pfeed.py STREAM OUT.jsonl RX.json -- bridge args...
"""
import json
import struct
import subprocess
import sys
import threading
import time

import numpy as np

sys.setswitchinterval(1e-4)   # the pacing loop spins: let the line reader get the GIL quickly
stream, outp, rxp = sys.argv[1], sys.argv[2], sys.argv[3]
cmd = sys.argv[sys.argv.index("--") + 1:]
data = open(stream, "rb").read()
recs, i = [], 0
while i + 12 <= len(data):
    m, t, n = struct.unpack_from("<III", data, i)
    tn = struct.unpack_from("<q", data, i + 12)[0] if (n >= 8 and t in (1, 2)) else None
    recs.append((i, 12 + n, t, tn))
    i += 12 + n
t_first = min(r[3] for r in recs if r[3] is not None)
lines = []          # (wall perf_counter, line bytes)
w0 = None


def reader(p):
    for l in p.stdout:
        lines.append((time.perf_counter(), l))


p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
th = threading.Thread(target=reader, args=(p,), daemon=True)
th.start()
time.sleep(1.0)
w0 = time.perf_counter()
last = t_first
late = []
for off, ln, t, tn in recs:
    if tn is not None:
        last = max(last, tn)
    due = w0 + (last - t_first) / 1e9
    d = due - time.perf_counter()
    if d > 0.0002:
        time.sleep(d - 0.0002)
    while time.perf_counter() < due:
        pass
    p.stdin.write(data[off:off + ln])
    if t == 1:
        late.append(time.perf_counter() - due)
p.stdin.close()
p.wait()
th.join(5)
with open(outp, "wb") as fo:
    for _, l in lines:
        fo.write(l)
# stats
pred = []
for w, l in lines:
    if b'"pred"' not in l:
        continue
    j = json.loads(l)
    if j.get("status") != "tracking":
        continue
    rx = t_first + (w - w0) * 1e9
    pred.append((rx, j["t_ns"], j["src"], j["h"]))
res = dict(stream=stream, cmd=cmd, made=time.strftime("%Y-%m-%d %H:%M:%S"), n_lines=len(lines), n_pred=len(pred),
           imu_write_late_ms=dict(p50=float(np.percentile(late, 50) * 1e3), p95=float(np.percentile(late, 95) * 1e3),
                                  max=float(np.max(late) * 1e3)))


def st(x):
    x = np.asarray(x, float)
    return dict(n=int(len(x)), p50=round(float(np.percentile(x, 50)), 3), p95=round(float(np.percentile(x, 95)), 3),
                p99=round(float(np.percentile(x, 99)), 3), max=round(float(x.max()), 3), mean=round(float(x.mean()), 3)) if len(x) else dict(n=0)


if pred:
    rx = np.array([x[0] for x in pred])
    tp = np.array([x[1] for x in pred], float)
    res["latency_ms_all"] = st((rx - tp) / 1e6)
    for s in ("frame", "rate"):
        m = np.array([x[2] == s for x in pred])
        if m.any():
            res[f"latency_ms_{s}"] = st(((rx - tp) / 1e6)[m])
            res[f"horizon_ms_{s}"] = st(np.array([x[3] for x in pred])[m] * 1e3)
    o = np.argsort(rx)
    rx, tp = rx[o], tp[o]
    tpm = np.maximum.accumulate(tp)      # newest predicted instant known at each receive
    g = np.arange(rx[0], rx[-1], 1e6)    # 1 kHz consumer clock
    k = np.searchsorted(rx, g, side="right") - 1
    res["age_at_consumer_ms_1khz"] = st((g - tpm[k]) / 1e6)
    res["gap_between_lines_ms"] = st(np.diff(rx) / 1e6)
    res["rate_hz"] = round(len(rx) / ((rx[-1] - rx[0]) / 1e9), 1)
json.dump(res, open(rxp, "w"), indent=1)
print(json.dumps({k: v for k, v in res.items() if k not in ("cmd",)}, indent=None))
