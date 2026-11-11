#!/usr/bin/env python3
"""Compare a laptop-worker result against the desktop reference for the same step (docs/laptop_worker.md).

  python laptop_compare.py vio  LAPTOP.poses.jsonl.gz DESKTOP.poses.jsonl.gz
  python laptop_compare.py ba   LAPTOP_BA_DIR DESKTOP_BA_DIR          (ba.json counts/scale + poses_ba)
  python laptop_compare.py json LAPTOP.json DESKTOP.json              (every numeric field, e.g. mesh summary / board_scale)
Prints max |diff| per field group and the fields that differ. Timing fields (ms, wall, runtime, made, timing) are skipped.
"""
import gzip, json, sys
import numpy as np

SKIP = ("ms", "wall", "runtime", "made", "timing", "remap_ms", "gpu_max", "_s")


def skip(k):
    return any(k == s or k.endswith(s) or k.startswith(s) for s in SKIP)


def flat(d, p=""):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            if skip(k):
                continue
            out.update(flat(v, f"{p}.{k}" if p else k))
    elif isinstance(d, list):
        for i, v in enumerate(d):
            out.update(flat(v, f"{p}[{i}]"))
    else:
        out[p] = d
    return out


def cmp_flat(a, b, label):
    fa, fb = flat(a), flat(b)
    keys = sorted(set(fa) | set(fb))
    diff, maxd, n = [], 0.0, 0
    for k in keys:
        x, y = fa.get(k, "<missing>"), fb.get(k, "<missing>")
        if isinstance(x, (int, float)) and isinstance(y, (int, float)) and not isinstance(x, bool):
            n += 1
            d = abs(float(x) - float(y))
            if d > 0:
                rel = d / max(abs(float(y)), 1e-12)
                diff.append((k, x, y, d, rel)); maxd = max(maxd, d)
        elif x != y:
            diff.append((k, x, y, None, None))
    print(f"[{label}] numeric fields {n}, differing {len(diff)}, max |diff| {maxd:.3g}")
    for k, x, y, d, rel in sorted(diff, key=lambda t: -(t[4] or 0))[:25]:
        print(f"   {k}: laptop {x!r:.60} desktop {y!r:.60}" + (f"  |d| {d:.3g} rel {rel:.3g}" if d is not None else ""))
    return diff


def read_poses(p):
    with gzip.open(p, "rt") as f:
        return [json.loads(l) for l in f if l.strip()]


def cmp_poses(pa, pb, label):
    A, B = read_poses(pa), read_poses(pb)
    print(f"[{label}] rows laptop {len(A)} desktop {len(B)}")
    same_rows = sum(1 for a, b in zip(A, B) if flat(a) == flat(b))
    dT = []
    for a, b in zip(A, B):
        ta, tb = a.get("T_world_cam0"), b.get("T_world_cam0")
        if ta is not None and tb is not None:
            dT.append(np.max(np.abs(np.array(ta, float) - np.array(tb, float))))
        elif (ta is None) != (tb is None):
            dT.append(np.inf)
    dT = np.array(dT) if dT else np.zeros(1)
    print(f"[{label}] rows identical (all non-timing fields) {same_rows}/{min(len(A), len(B))}; "
          f"T_world_cam0 max |d| {dT.max():.3g}, median {np.median(dT):.3g}, rows with d>0: {(dT > 0).sum()}")
    if same_rows < min(len(A), len(B)):
        for i, (a, b) in enumerate(zip(A, B)):
            if flat(a) != flat(b):
                cmp_flat(a, b, f"{label} first differing row {i}")
                break


def main():
    mode, a, b = sys.argv[1:4]
    if mode == "vio":
        cmp_poses(a, b, "vio")
    elif mode == "ba":
        cmp_flat(json.load(open(f"{a}/ba.json")), json.load(open(f"{b}/ba.json")), "ba.json")
        cmp_poses(f"{a}/poses_ba.jsonl.gz", f"{b}/poses_ba.jsonl.gz", "poses_ba")
    elif mode == "json":
        cmp_flat(json.load(open(a)), json.load(open(b)), "json")
    else:
        print(__doc__); sys.exit(1)


if __name__ == "__main__":
    main()
