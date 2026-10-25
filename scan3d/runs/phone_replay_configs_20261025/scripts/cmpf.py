import struct, numpy as np, av, sys
S = sys.argv[1]
ph = np.fromfile(S + "/f100_phone.y", np.uint8).reshape(1080, 1920)
f = open(S + "/raw80.bin", "rb"); k = 0
while True:
    m, t, n = struct.unpack("<III", f.read(12)); p = f.read(n)
    if t == 2:
        if k == 100: orig = np.frombuffer(p, np.uint8, 1920*1080, 20).reshape(1080, 1920); break
        k += 1
ic = av.open("/tmp/mp4t20/frames.mp4")
for i, fr in enumerate(ic.decode(video=0)):
    if i == 100: pc = fr.to_ndarray(format="yuv420p")[:1080]; break
d = lambda a, b: (round(float(np.abs(a.astype(int) - b).mean()), 3), int(np.abs(a.astype(int) - b).max()), round(float(a.astype(int).mean() - b.mean()), 3))
print("phone vs orig", d(ph, orig), "phone vs pcY", d(ph, pc), "pcY vs orig", d(pc, orig))
A = np.polyfit(ph.ravel().astype(float), pc.ravel().astype(float), 1); print("fit pcY = a*ph + b", A)
lut = np.clip(np.round(np.arange(256) * 219 / 255 + 16), 0, 255).astype(np.uint8)
print("lut(ph) vs pcY", d(lut[ph], pc), "lut(ph) vs orig", d(lut[ph], orig))
L = np.arange(256).astype(float)
for v in range(256):
    m = ph == v
    if m.sum() > 20: L[v] = np.median(pc[m])
v = np.nonzero(np.bincount(ph.ravel(), minlength=256) > 20)[0]
print("ph range", v.min(), v.max(), "L samples", [(int(x), L[x]) for x in v[::15]])
L2 = np.round(np.interp(np.arange(256), v, L[v])).astype(np.uint8)
print("emp lut vs pcY", d(L2[ph], pc), "vs orig", d(L2[ph], orig))
print("frac exact", float((L2[ph] == pc).mean()))
open(S + "/lut.h", "w").write("static const unsigned char LUT[256] = {" + ",".join(str(int(x)) for x in L2) + "};\n")
