#!/usr/bin/env python3
"""ovloop 2026-10-26: place-recognition recall on the SAME keyframes for BoW (B30) vs BoQ (Q30) vs SALAD (S30).

  python3 recall.py [OUT]     (OUT = OVLOOP_OUT, default ~/scanner/tmp_ovloop/ate) -> OUT/recall.json + RD/recall.json
GT cells only (std, hs). Per keyframe (kf line of the --loop-log) the GT camera pose at t_ns (+ dt_ns of the run, same
lookup as ovloop_run.loop_edges_vs_gt). Eligible old keyframe j for current i (same rule as the bridge): same session,
t_j <= t_i - 3 s, path_i - path_j >= 1 m. "Same place" = GT camera centres < 0.5 m apart AND optical axes < 30 deg.
Revisit keyframe = has at least one eligible same-place old keyframe.
  cand_hit     : revisit KFs whose candidate list (<= max_cand, after the score threshold) contains a same-place KF
  cand_wrong   : candidates (all KFs) that are not same-place (verification must reject them)
  loop_recall  : revisit KFs that got an ACCEPTED loop edge (loop line cur == idx)
  r@1 / r@5    : (VPR only, offline from the .ovpr file, no threshold) top-1 / top-5 eligible by cosine contains same place
"""
import gzip
import json
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ovloop_run as R  # noqa: E402  (chdir to MAIN, sys.path for vio_bench_hs)
import vio_bench_hs as vb  # noqa: E402

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else R.OUT
D_M, A_DEG, GAP_S, PATH_M = 0.5, 30.0, 3.0, 1.0
VPR = {"Q30": "boq", "S30": "salad"}


def read_ovpr(f):
    b = open(f, "rb").read()
    assert b[:4] == b"OVPR"
    dim = struct.unpack("<I", b[4:8])[0]
    n = (len(b) - 8) // (8 + 4 * dim)
    out = {}
    for i in range(n):
        o = 8 + i * (8 + 4 * dim)
        t = struct.unpack("<q", b[o:o + 8])[0]
        out[t] = np.frombuffer(b[o + 8:o + 8 + 4 * dim], dtype="<f4")
    return out


def gt_lookup(rec, dt):
    g = dict(np.load(vb.pick_gt(R.MAIN / "recordings" / rec, "auto") / "gt.npz", allow_pickle=True))
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p, gt_R = g["t_ns"][ok], g["p_bc"][ok], g["R_bc"][ok]

    def at(t):
        t = t + dt
        j = int(np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1))
        j = j - 1 if abs(gt_t[j - 1] - t) < abs(gt_t[j] - t) else j
        return (gt_p[j], gt_R[j][:, 2]) if abs(gt_t[j] - t) < 2_000_000 else None
    return at


def cell(cond, name):
    r = json.load(open(OUT / cond / f"{name}.json"))
    L = [json.loads(x) for x in open(OUT / cond / f"{name}.loop.jsonl")]
    kfs = [x for x in L if x.get("type") == "kf"]
    acc = {(x["session"], x["cur"]) for x in L if x.get("type") == "loop"}
    at = gt_lookup(r["rec"], r.get("dt_ns") or 0)
    by = {(k["session"], k["idx"]): k for k in kfs}
    G = {key: at(k["t_ns"]) for key, k in by.items()}

    def same(a, b):
        ga, gb = G.get(a), G.get(b)
        if ga is None or gb is None:
            return None
        ang = np.degrees(np.arccos(np.clip(float(ga[1] @ gb[1]), -1, 1)))
        return bool(np.linalg.norm(ga[0] - gb[0]) < D_M and ang < A_DEG)

    def eligible(k):
        return [o for o in kfs if o["session"] == k["session"] and o["t"] <= k["t"] - GAP_S and k["path"] - o["path"] >= PATH_M]
    desc = read_ovpr(OUT / "vpr" / VPR[cond] / (name.rsplit("_", 1)[0] + ".ovpr")) if cond in VPR else None
    s = dict(n_kf=len(kfs), n_gt=sum(v is not None for v in G.values()), revisit=0, cand_hit=0, n_cand=0, cand_wrong=0,
             cand_unk=0, loop_hit=0, r1=0, r5=0)
    for k in kfs:
        key = (k["session"], k["idx"])
        if G.get(key) is None:
            continue
        el = eligible(k)
        truth = [o for o in el if same(key, (o["session"], o["idx"]))]
        cands = [c[0] for c in k["cand"] if c[1] >= 0]
        for c in cands:
            v = same(key, (k["session"], c))
            s["n_cand"] += 1
            s["cand_wrong"] += v is False
            s["cand_unk"] += v is None
        if not truth:
            continue
        s["revisit"] += 1
        s["cand_hit"] += any(same(key, (k["session"], c)) for c in cands)
        s["loop_hit"] += key in acc
        if desc is not None and k["t_ns"] in desc:
            q = desc[k["t_ns"]]
            sc = sorted(((float(desc[o["t_ns"]] @ q), o) for o in el if o["t_ns"] in desc), key=lambda x: -x[0])
            top = [same(key, (o["session"], o["idx"])) for _, o in sc[:5]]
            s["r1"] += bool(top[:1] and top[0])
            s["r5"] += any(top)
    return s


def main():
    rows, agg = {}, {}
    for cond in ("B30", "Q30", "S30", "B30w"):
        if not (OUT / cond).is_dir():
            continue
        for jf in sorted((OUT / cond).glob("*.json")):
            name = jf.stem
            if R.which_of(name) == "loop":
                continue
            s = cell(cond, name)
            rows[f"{cond}/{name}"] = s
            for setn in (R.which_of(name), "all"):
                a = agg.setdefault(f"{cond}/{setn}", {})
                for k, v in s.items():
                    a[k] = a.get(k, 0) + v
                a["cells"] = a.get("cells", 0) + 1
    for a in agg.values():
        rv = max(a["revisit"], 1)
        a["cand_hit_pct"] = round(100 * a["cand_hit"] / rv, 1)
        a["loop_recall_pct"] = round(100 * a["loop_hit"] / rv, 1)
        a["cand_wrong_pct"] = round(100 * a["cand_wrong"] / max(a["n_cand"], 1), 1)
        a["r1_pct"] = round(100 * a["r1"] / rv, 1)
        a["r5_pct"] = round(100 * a["r5"] / rv, 1)
    res = dict(made=time.strftime("%F %T"), out=str(OUT), rule=dict(same_place_m=D_M, same_place_deg=A_DEG, gap_s=GAP_S,
               path_m=PATH_M), agg=agg, rows=rows)
    json.dump(res, open(OUT / "recall.json", "w"), indent=1)
    (R.RD / "recall.json").write_text(json.dumps(res, indent=1))
    for k, a in sorted(agg.items()):
        print(k, {x: a[x] for x in ("cells", "n_kf", "revisit", "cand_hit_pct", "loop_recall_pct", "cand_wrong_pct", "r1_pct", "r5_pct")})


if __name__ == "__main__":
    main()
