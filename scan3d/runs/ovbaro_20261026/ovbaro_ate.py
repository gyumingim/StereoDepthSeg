#!/usr/bin/env python3
"""ovbaro 2026-10-26: PC evaluation of --baro on the same ruler as integ2 (runs/integ2_20261026/integ2_ate.py = ov60eval
runner: 42 HS cells = 14 recordings x start offsets 0/1/2 s (+ 213955 drop, excluded), GT imu_gt_spl_cal1022, scorer
lens_switch_gtscore window ATE). Only the bridge (this branch's PC build, -DOV_FULL_SOURCE=ON) and the conditions change.
Profile for every condition = integ2 N30: "--profile rec20261025 --drop-late 0" (30 fps, bench template).

Conditions:
  OFF  no baro flag                          -> identity vs integ2 N30 (pose hashes, /tmp/integ2/ate/N30)
  B0   --baro 1, synthetic baro s0 (GT height + real static S24 baro fluctuation; gen_baro.py)
  B1   --baro 1, s1 = s0 + permanent 1.5 hPa step at 25 s (door)                       [offset 0 only]
  B2   --baro 1, s2 = s0 + 6 prop-wash bursts                                            [offset 0 only]
  K0   --test-blank 20:2 (cam0 black 2 s, 20 s after the first frame), no baro          [offset 0 only]
  K1   --test-blank 20:2 + --baro 1 s0                                                   [offset 0 only]
Height metrics (own, gravity-aligned, no alignment needed besides the start offset): e(t) = (z(t) - z(t0)) - (h(t) - h(t0)),
t0 = first tracking pose, z = pose "p"[2] (IMU position, VIO world z = up), h = GT IMU height (gen_baro.truth).

    python3 ovbaro_ate.py freeze | run hs:OFF,B0 [--par 4] | score
"""
import argparse
import gzip
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "integ2_20261026"))
os.environ.setdefault("OV60_OUT", "/tmp/ovbaro/ate")
import integ2_ate as ia  # noqa: E402  (chdir MAIN, sys.path imu_learn)
import gen_baro  # noqa: E402

WT = Path("/home/system/scanner/StereoDepthSeg-ovbaro/scan3d")
OUT = Path(os.environ["OV60_OUT"])
ia.WT, ia.RD, ia.OUT = WT, HERE, OUT
ia.SRC = WT / "ov_bridge" / "build_ovbaro_full" / "ov_bridge"
NEWP = "--profile rec20261025 --drop-late 0"
BLANK = "--test-blank 20:2"
CONDS = {
    "OFF": dict(fps=30.0, args=NEWP),
    "B0": dict(fps=30.0, args=NEWP, baro="s0"),
    "B1": dict(fps=30.0, args=NEWP, baro="s1"),
    "B2": dict(fps=30.0, args=NEWP, baro="s2"),
    "K0": dict(fps=30.0, args=NEWP + " " + BLANK),
    "K1": dict(fps=30.0, args=NEWP + " " + BLANK, baro="s0"),
}
ia.CONDS = CONDS
ia.PAIRS = [("OFF", "B0"), ("B0", "B1"), ("B0", "B2"), ("K0", "K1")]
O0_ONLY = {"B1", "B2", "K0", "K1"}


def freeze():
    (OUT / "bin").mkdir(parents=True, exist_ok=True)
    h = hashlib.sha256(open(ia.SRC, "rb").read()).hexdigest()
    dst = OUT / "bin" / f"ov_bridge_{h[:12]}"
    if not dst.exists():
        import shutil
        shutil.copy2(ia.SRC, dst)
    git = subprocess.run(["git", "-C", str(WT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(WT), "status", "--short", "ov_bridge"], capture_output=True, text=True).stdout
    json.dump(dict(src=str(ia.SRC), sha256=h, copy=str(dst), frozen=time.strftime("%F %T"), worktree_head=git,
                   ov_bridge_dirty=[x for x in dirty.splitlines() if "build_ovbaro_full" not in x]),
              open(OUT / "bin" / "bridge.json", "w"), indent=1)
    for c, d in CONDS.items():
        w = ia.wrapper(c)
        w.write_text(f"#!/bin/bash\nexec {dst} \"$@\" {d['args']} $OVBARO_EXTRA\n")
        w.chmod(0o755)
    print("frozen", dst, h)


def cells_for(cond):
    return [(r, o) for r, o in ia.cells("hs") if not (cond in O0_ONLY and o != 0)]


def job(name):
    key6, o, cond = name.split("_")
    d = CONDS[cond]
    extra = ""
    if d.get("baro"):
        rec = next(r for r, oo in ia.cells("hs") if r.split("_")[2] == key6)
        csv = OUT / "baro" / f"{rec}_{d['baro']}.csv"
        if not csv.exists():
            gen_baro.gen(rec, d["baro"], csv)
        extra = f"--baro 1 --baro-csv {csv} --baro-log {OUT / cond / (name + '.baro.csv')}"
    os.environ["OVBARO_EXTRA"] = extra
    (OUT / cond).mkdir(parents=True, exist_ok=True)
    ia.job(name)


def run_sched(spec, par):
    q = []
    for part in spec.split(";"):
        _, conds = part.split(":")
        q += [ia.name_of(r, o, c, "hs") for c in conds.split(",") for r, o in cells_for(c)]
    q = [n for n in q if not (OUT / n.split("_")[2] / f"{n}.json").exists()]
    running, t_all = {}, time.time()
    print(f"{time.strftime('%F %T')} start {spec} {len(q)} jobs PAR {par}", flush=True)
    while q or running:
        for n, (p, t0) in list(running.items()):
            if p.poll() is not None:
                del running[n]
                ok = (OUT / n.split("_")[2] / f"{n}.json").exists()
                print(f"{time.strftime('%F %T')} done {n} {time.time() - t0:.0f} s {'ok' if ok else 'FAILED'}", flush=True)
        while q and len(running) < par:
            n = q.pop(0)
            (OUT / n.split("_")[2]).mkdir(parents=True, exist_ok=True)
            err = open(OUT / n.split("_")[2] / f"{n}.err", "w")
            running[n] = (subprocess.Popen(["nice", "-n", "5", ia.PY, str(Path(__file__).resolve()), "job", n],
                                           stdout=subprocess.DEVNULL, stderr=err), time.time())
        time.sleep(2)
    print(f"{time.strftime('%F %T')} end {spec} wall {time.time() - t_all:.0f} s", flush=True)


def zmetrics(P, T, blank=None):
    tr = [m for m in P if m.get("status") == "tracking" and m.get("p")]
    if len(tr) < 10:
        return None
    t = np.array([m["t_ns"] for m in tr], np.int64)
    z = np.array([m["p"][2] for m in tr])
    h = np.interp(t, T["t_imu"], T["h"])
    e = (z - z[0]) - (h - h[0])
    de = np.abs(np.diff(z) - np.diff(h))
    res = dict(n=len(tr), rms_cm=round(float(np.sqrt(np.mean(e ** 2))) * 100, 3), max_cm=round(float(np.abs(e).max()) * 100, 3),
               final_cm=round(float(e[-1]) * 100, 3), jump_max_cm=round(float(de.max()) * 100, 3) if len(de) else None,
               t=t, z=z, e=e)
    bl = [m["t_ns"] for m in P if m.get("blank")]
    if bl:  # vision loss window + 3 s after
        w = (t >= min(bl)) & (t <= max(bl) + 3_000_000_000)
        if w.any():
            e0 = e[np.searchsorted(t, min(bl)) - 1] if np.searchsorted(t, min(bl)) > 0 else 0.0
            res["blank_max_cm"] = round(float(np.abs(e[w] - e0).max()) * 100, 3)
    return res


def phash(P):
    h = hashlib.sha256()
    for m in P:
        h.update(json.dumps({k: v for k, v in m.items() if k not in ("ms", "remap_ms", "wait_ms")}, sort_keys=True).encode())
    return h.hexdigest()[:16]


def score():
    import lens_switch_gtscore as gs
    gs.OUT = OUT / "score_tmp"
    gs.OUT.mkdir(parents=True, exist_ok=True)
    conds = list(CONDS)
    _, drop, _ = ia.hs_cells()
    rows = ia.score_hs_std("hs", conds, gs)
    S, Pp = ia.summarize(rows, conds, drop)
    # identity OFF vs integ2 N30 (same pose hash rule as integ2_ate)
    ident = []
    for r, o in ia.cells("hs"):
        a = OUT / "OFF" / ia.name_of(r, o, "OFF", "hs")
        b = Path("/tmp/integ2/ate/N30") / ia.name_of(r, o, "N30", "hs")
        if Path(str(a) + ".poses.jsonl.gz").exists() and Path(str(b) + ".poses.jsonl.gz").exists():
            ident.append((ia.key_of(r, o, "hs"), phash(ia.load_P(a)) == phash(ia.load_P(b))))
    # height metrics
    Z = {}
    truths = {}
    for r, o in ia.cells("hs"):
        if r not in truths:
            truths[r] = gen_baro.truth(r)
        k = ia.key_of(r, o, "hs")
        zz = {}
        for c in conds:
            pre = OUT / c / ia.name_of(r, o, c, "hs")
            if Path(str(pre) + ".poses.jsonl.gz").exists():
                zz[c] = zmetrics(ia.load_P(pre), truths[r])
        for a, b in (("B0", "B1"), ("B0", "B2")):  # disturbance leakage: same timestamps, |z_b - z_a|
            if zz.get(a) and zz.get(b):
                ta, tb = zz[a]["t"], zz[b]["t"]
                common, ia_, ib_ = np.intersect1d(ta, tb, return_indices=True)
                if len(common):
                    d = (zz[b]["z"][ib_] - zz[b]["z"][ib_][0]) - (zz[a]["z"][ia_] - zz[a]["z"][ia_][0])
                    zz[b]["leak_vs_B0_max_cm"] = round(float(np.abs(d).max()) * 100, 3)
                    zz[b]["leak_vs_B0_jump_max_cm"] = round(float(np.abs(np.diff(d)).max()) * 100, 3) if len(d) > 1 else None
        Z[k] = {c: {kk: vv for kk, vv in v.items() if kk not in ("t", "z", "e")} if v else None for c, v in zz.items()}
        Z[k]["rec"] = r
        Z[k]["div"] = {c: rows_div(rows, k, c) for c in conds}
    res = dict(made=time.strftime("%F %T"), protocol=__doc__, conds=CONDS, bridge=json.load(open(OUT / "bin" / "bridge.json")),
               identity_off_vs_integ2_N30=dict(n=len(ident), identical=sum(x for _, x in ident),
                                               differ=[k for k, x in ident if not x]),
               hs=dict(summary=S, paired=Pp, excluded=drop, cells=rows), height=Z)
    json.dump(res, open(OUT / "score.json", "w"), indent=1, ensure_ascii=False, default=float)
    json.dump(res, open(HERE / "score.json", "w"), indent=1, ensure_ascii=False, default=float)
    print("identity OFF vs integ2 N30:", res["identity_off_vs_integ2_N30"])
    print("| cond | n | div | ate_w med (mean) cm |")
    for c, s in S.items():
        print(f"| {c} | {s['n']} | {s['div']} {s['div_cells']} | {s['ate_w_med']} ({s['ate_w_mean']}) |")
    for k, p in Pp.items():
        print(k, {x: p[x] for x in ("n_both", "d_med", "d_mean", "n_b_better", "div_only_a", "div_only_b", "big")})
    print("\nheight (non-divergent in both of the pair):")
    for c in conds:
        v = [Z[k][c] for k in Z if Z[k].get(c) and not Z[k]["div"].get(c) and Z[k]["rec"] != drop]
        if not v:
            continue
        f = lambda key: round(float(np.median([x[key] for x in v if x.get(key) is not None])), 3) if any(x.get(key) is not None for x in v) else None
        print(c, "n", len(v), "rms med", f("rms_cm"), "max med", f("max_cm"), "jump med", f("jump_max_cm"),
              "jump max", round(max(x["jump_max_cm"] for x in v), 3), "leak med/max",
              f("leak_vs_B0_max_cm"), max([x.get("leak_vs_B0_max_cm") or 0 for x in v]), "blank med", f("blank_max_cm"))
    print("saved", OUT / "score.json", HERE / "score.json")


def rows_div(rows, k, c):
    for x in rows:
        if x["key"] == k and c in x:
            return bool(x[c]["div"])
    return None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["freeze", "run", "job", "score"])
    ap.add_argument("arg", nargs="?", default="")
    ap.add_argument("--par", type=int, default=4)
    a = ap.parse_args()
    {"freeze": lambda: freeze(), "run": lambda: run_sched(a.arg, a.par), "job": lambda: job(a.arg), "score": score}[a.cmd]()
