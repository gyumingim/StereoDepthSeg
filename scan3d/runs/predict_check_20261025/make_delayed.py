# Reorder a captured bridge input stream (records MAGIC,type,len + payload; t_ns = first 8 payload bytes) so every camera
# record (type 2) comes after the IMU records up to t_cam + D ms = what the bridge sees on the phone, where the frame
# reaches the bridge ~40 ms after its exposure while the IMU arrives within a few ms.
# usage: make_delayed.py IN.bin OUT.bin DELAY_MS
import struct, sys

src, dst, dms = sys.argv[1], sys.argv[2], float(sys.argv[3])
data = open(src, "rb").read()
recs, i, key = [], 0, 0
while i + 12 <= len(data):
    m, t, n = struct.unpack_from("<III", data, i)
    tn = struct.unpack_from("<q", data, i + 12)[0] if n >= 8 and t in (1, 2) else None
    if tn is not None:
        key = tn + (int(dms * 1e6) if t == 2 else 0)
    recs.append((key, len(recs), i, 12 + n))
    i += 12 + n
recs.sort()
with open(dst, "wb") as f:
    for _, _, off, ln in recs:
        f.write(data[off:off + ln])
print("records", len(recs), "delay_ms", dms)
