# Check new remaps: header size == yaml resolution, source 1920x1080, map = projection of K_rect^-1 q through the lens json.
import json, sys, hashlib, re
from pathlib import Path
import numpy as np, cv2
O = Path(sys.argv[1]); lens = json.load(open(sys.argv[2])); OLD = Path(sys.argv[3]) if len(sys.argv) > 3 else None
K = np.array(lens["K"]); d = np.array(lens["dist"])
def rd(p):
    b = p.read_bytes(); w, h = np.frombuffer(b[:8], np.int32); n = w * h
    mx = np.frombuffer(b[8:8 + 4 * n], np.float32).reshape(h, w); my = np.frombuffer(b[8 + 4 * n:8 + 8 * n], np.float32).reshape(h, w)
    tail = np.frombuffer(b[8 + 8 * n:], np.int32); return int(w), int(h), mx, my, tail.tolist(), len(b)
def yres(dr):
    t = (dr / "kalibr_imucam_chain.yaml").read_text()
    r = [int(x) for x in re.search(r"resolution:\s*\[(\d+),\s*(\d+)\]", t).group(1, 2)]
    f = [float(x) for x in re.search(r"intrinsics:\s*\[([^\]]+)\]", t).group(1).split(",")]
    return r, f
rows = []
for dr in sorted([p for p in O.iterdir() if p.is_dir()]):
    res, intr = yres(dr)
    rp = dr / "remap_1920x1080.bin"
    src = "own" if rp.exists() else "top-level"
    if not rp.exists(): rp = O / "remap_1920x1080.bin"
    w, h, mx, my, tail, n = rd(rp)
    ok = [w, h] == res and tail == [1920, 1080]
    # projection check on a grid
    Kr = np.array([[intr[0], 0, intr[2]], [0, intr[1], intr[3]], [0, 0, 1]])
    ys, xs = np.mgrid[0:h:7, 0:w:7]
    q = np.stack([xs.ravel(), ys.ravel(), np.ones(xs.size)], 0).astype(float)
    ray = np.linalg.inv(Kr) @ q
    pts, _ = cv2.projectPoints(ray.T.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, d)
    pts = pts.reshape(-1, 2); m = np.stack([mx[ys.ravel(), xs.ravel()], my[ys.ravel(), xs.ravel()]], 1)
    err = np.abs(pts - m).max()
    inside = ((m[:, 0] >= 0) & (m[:, 0] <= 1919) & (m[:, 1] >= 0) & (m[:, 1] <= 1079)).mean()
    od = ""
    if OLD:
        orp = OLD / dr.name / "remap_1920x1080.bin"
        if not orp.exists(): orp = OLD / "remap_1920x1080.bin"
        ow, oh, omx, omy, _, _ = rd(orp)
        if (ow, oh) == (w, h):
            dd = np.hypot(mx - omx, my - omy)
            od = f"raw-px shift vs old map: centre {dd[h//2, w//2]:.2f} median {np.median(dd):.2f} max {dd.max():.2f}"
    rows.append(f"{dr.name:15s} yaml {res} remap {w}x{h} ({src}) src {tail} {'OK' if ok else 'MISMATCH'} proj_err_max {err:.3f} px inside {inside:.4f} {od}")
print("\n".join(rows))
for p in sorted(O.rglob("*.bin")):
    print(hashlib.sha256(p.read_bytes()).hexdigest()[:16], p.relative_to(O), p.stat().st_size)
