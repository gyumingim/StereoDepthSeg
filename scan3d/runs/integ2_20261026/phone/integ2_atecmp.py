# integ2: per-cell pose comparison of the ATE runs: integ2 O30 (old profile, integ2 build) vs ov60eval P30 (integ build 14de3b8),
# OL30 vs L30, and N30 vs O30 / NL30 vs OL30 (max position difference). Poses compared on t, status, T_world_cam0 (all values).
import gzip, json, glob, os, math, sys
def load(f):
    out = []
    for l in gzip.open(f, "rt"):
        if not l.strip(): continue
        m = json.loads(l)
        out.append(m)
    return out
def pos(m):
    T = m.get("T_world_cam0") or m.get("T")
    if T is None: return m.get("p")
    T = T if len(T) == 16 else sum(T, [])
    return (T[3], T[7], T[11])
def cmp(fa, fb):
    A, B = load(fa), load(fb)
    if len(A) != len(B): return dict(n=(len(A), len(B)), same=False, dmax=None)
    same = True; dmax = 0.0; st = 0
    for x, y in zip(A, B):
        kx = {k: v for k, v in x.items() if k not in ("ms", "remap_ms", "wait_ms")}
        ky = {k: v for k, v in y.items() if k not in ("ms", "remap_ms", "wait_ms")}
        if kx != ky: same = False
        if x.get("status") != y.get("status"): st += 1
        px, py = pos(x), pos(y)
        if px is not None and py is not None: dmax = max(dmax, math.dist(px, py))
    return dict(n=len(A), same=same, dmax_cm=dmax * 100, status_diff=st)
R = {}
for a, b in (("/tmp/ov60eval/ate/P30", "/tmp/integ2/ate/O30"), ("/tmp/ov60eval/ate/L30", "/tmp/integ2/ate/OL30"),
             ("/tmp/integ2/ate/O30", "/tmp/integ2/ate/N30"), ("/tmp/integ2/ate/OL30", "/tmp/integ2/ate/NL30")):
    ca, cb = os.path.basename(a), os.path.basename(b)
    rows = []
    for fb in sorted(glob.glob(f"{b}/*_{cb}.poses.jsonl.gz")):
        key = os.path.basename(fb).split(f"_{cb}.")[0]
        fa = f"{a}/{key}_{ca}.poses.jsonl.gz"
        if not os.path.exists(fa): continue
        rows.append((key, cmp(fa, fb)))
    same = sum(r["same"] for _, r in rows)
    dm = max((r["dmax_cm"] for _, r in rows if r["dmax_cm"] is not None), default=None)
    sd = sum(r.get("status_diff", 0) for _, r in rows)
    print(f"{ca} -> {cb}: cells {len(rows)}, identical pose lines {same}/{len(rows)}, max |dp| {dm} cm, status diffs {sd}")
    for k, r in rows:
        if not r["same"] and ca in ("P30", "L30"): print("   differs:", k, r)
    R[f"{ca}->{cb}"] = dict(cells=len(rows), identical=same, max_dp_cm=dm, status_diffs=sd, per_cell={k: r for k, r in rows})
json.dump(R, open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/integ2/ate/atecmp.json", "w"), indent=1)
