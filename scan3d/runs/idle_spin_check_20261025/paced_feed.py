# replay a captured bridge input stream (MAGIC,type,len + payload records) in real time (record t_ns pacing) into a
# bridge command; bridge stdout -> file. usage: paced_feed.py STREAM OUT.jsonl -- bridge args...
import struct, subprocess, sys, time

stream, outp = sys.argv[1], sys.argv[2]
cmd = sys.argv[sys.argv.index("--") + 1:]
data = open(stream, "rb").read()
recs, i = [], 0
while i + 12 <= len(data):
    m, t, n = struct.unpack_from("<III", data, i)
    tn = struct.unpack_from("<q", data, i + 12)[0] if n >= 8 else None
    recs.append((i, 12 + n, t, tn))
    i += 12 + n
ts = [r[3] for r in recs if r[3] is not None and r[2] in (1, 2)]
t_first = min(ts)
with open(outp, "wb") as fo:
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=fo, bufsize=0)
    time.sleep(1.0)
    w0 = time.perf_counter()
    last = t_first
    for off, ln, t, tn in recs:
        if tn is not None and t in (1, 2):
            last = max(last, tn)
        due = w0 + (last - t_first) / 1e9
        d = due - time.perf_counter()
        if d > 0.0002:
            time.sleep(d - 0.0002)
        while time.perf_counter() < due:
            pass
        p.stdin.write(data[off:off + ln])
    p.stdin.close()
    p.wait()
print("records", len(recs), "span_s", (max(ts) - t_first) / 1e9, "wall_s", time.perf_counter() - w0)
