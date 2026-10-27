#!/usr/bin/env python3
"""xfopt 2026-10-27: offline re-application of the loop gates + temporal consistency (ovloop.cpp process() steps 4-5) to
the per-candidate PnP results logged with --loop-cfg log_cand=1 ("cx" in each kf line).

Exact because the VIO (and so the keyframes, BoW candidates and PnP inputs) does not depend on the loop decisions (loop never
changes the VIO; --loop-sync 1). check(): the default gates must reproduce the run's own accepted loop lines exactly.
Gate dict keys: pick (b = BRIEF only, x = XFeat only, h = hybrid BRIEF-then-XFeat), x gates: inl, ratio, cells, hull, rp, yaw,
ntri_frac (max triangulated share of inliers), yaw_rate (deg per s between the keyframes, + yaw), consist_* (shared).
"""
import gzip
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import xfopt_run as XR  # noqa: E402

DEF = dict(pick="x", inl=0, ratio=0.0, cells=0, hull=0.0, rp=3.0, yaw=10.0, ntri_frac=1.0, yaw_a=None, yaw_b=0.0,
           max_corr_m=0.5, max_corr_rel=0.05, consist_kf=3, consist_old_s=4.0, consist_m=0.05, consist_yaw_deg=1.5, consist_n=1)


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Cell:
    """one run (cond / name): keyframes with their candidate entries, actual loops, GT."""

    def __init__(self, cond, name, out=XR.OUT, gt=True):
        self.cond, self.name = cond, name
        d = out / cond
        self.res = json.load(open(d / f"{name}.json"))
        L = [json.loads(x) for x in open(d / f"{name}.loop.jsonl")]
        self.kfs = [x for x in L if x.get("type") == "kf"]
        self.loops = [x for x in L if x.get("type") == "loop"]
        self.G = None
        if gt and self.res.get("ol_set") != "loop":
            poses = [json.loads(x) for x in gzip.open(d / f"{name}.poses.jsonl.gz", "rt")]
            self.G = XR.GT(self.res, self.res["rec"], poses)
        self._err = {}

    def err(self, k, e):
        """GT error of candidate entry e of keyframe k (cached) -> (cm, deg, corners) or None."""
        key = (k["session"], k["idx"], e["o"], e["p"])
        if key not in self._err:
            old = self.kf_of(k["session"], e["o"])
            self._err[key] = self.G.err(old["t_ns"], k["t_ns"], e["q"], e["t"]) if (self.G and e.get("q")) else None
        return self._err[key]

    def kf_of(self, session, idx):
        if not hasattr(self, "_kmap"):
            self._kmap = {(k["session"], k["idx"]): k for k in self.kfs}
        return self._kmap[(session, idx)]


def gate_ok(e, k, old, g):
    """re-applied gates of one logged path result (pose needed)."""
    if e["p"] == "b":
        return bool(e["ok"])          # BRIEF path: gates = defaults (as logged)
    if not e.get("q"):
        return False
    if e["why"] not in ("ok", "rollpitch", "drift", "yaw", "xinl", "xratio", "xcells"):
        return False
    path_between = k["path"] - old["path"]
    if e["rp"] > g["rp"]:
        return False
    if np.linalg.norm(e["drift"]) > g["max_corr_m"] + g["max_corr_rel"] * path_between:
        return False
    yaw_deg = abs(e["dyaw"]) * 180 / math.pi
    if yaw_deg > g["yaw"]:
        return False
    if g["yaw_a"] is not None and yaw_deg > g["yaw_a"] + g["yaw_b"] * (k["t"] - old["t"]):
        return False
    if e["ni"] < max(g["inl"], 0) or e["ni"] < g["ratio"] * e["nm"]:
        return False
    if e.get("cells", 0) < g["cells"] or e.get("hull", 0.0) < g["hull"]:
        return False
    if e["ni"] and e.get("ntri", 0) / e["ni"] > g["ntri_frac"]:
        return False
    return True


def simulate(c, gates):
    g = dict(DEF, **gates)
    acc = []          # (session, cur idx, chosen entry)
    recent = []       # [cur, old, entry, accepted, n_consistent]
    session = None
    for k in c.kfs:
        if k["session"] != session:
            session = k["session"]
            recent = []
        cand = {}
        order = []
        for e in k.get("cx", []):
            if e["o"] not in cand:
                cand[e["o"]] = {}
                order.append(e["o"])
            cand[e["o"]][e["p"]] = e
        best = None
        for o in order:
            old = c.kf_of(session, o)
            eb, ex = cand[o].get("b"), cand[o].get("x")
            okb = eb is not None and gate_ok(eb, k, old, g)
            okx = ex is not None and gate_ok(ex, k, old, g)
            if g["pick"] == "b":
                ch = eb if okb else None
            elif g["pick"] == "x":
                ch = ex if okx else None
            else:
                ch = eb if okb else (ex if okx else None)
            if ch is not None and (best is None or ch["ni"] > best["ni"]):
                best = ch
        accept = []
        if best is not None:
            hits = []
            for r in recent:
                if k["idx"] - r[0] > g["consist_kf"]:
                    continue
                if abs(c.kf_of(session, r[1])["t"] - c.kf_of(session, best["o"])["t"]) > g["consist_old_s"]:
                    continue
                if np.linalg.norm(np.array(r[2]["drift"]) - np.array(best["drift"])) > g["consist_m"]:
                    continue
                if abs(wrap(r[2]["dyaw"] - best["dyaw"])) * 180 / math.pi > g["consist_yaw_deg"]:
                    continue
                hits.append(r)
            consistent = len(hits) >= g["consist_n"]
            if consistent:
                for r in hits:
                    if not r[3]:
                        r[3] = True
                        accept.append((r[0], r[2]))
                accept.append((k["idx"], best))
            recent.append([k["idx"], best["o"], best, consistent])
        recent = [r for r in recent if k["idx"] - r[0] <= g["consist_kf"]]
        acc += [(session, cur, e) for cur, e in accept]
    return acc


def edges_score(c, acc, deg=3.0, cm=10.0):
    n = len(acc)
    chk = good = fg = fall = 0
    errs = []
    for s, cur, e in acc:
        k = c.kf_of(s, cur)
        r = c.err(k, e)
        if r is None:
            continue
        chk += 1
        f = r[0] > cm or r[1] > deg
        fall += f
        if r[2] >= XR.MIN_CORNERS:
            good += 1
            fg += f
            errs.append(r[1])
    return dict(n=n, checked=chk, good=good, false_good=fg, false_all=fall, deg_med=float(np.median(errs)) if errs else None)


def check(c, pick):
    """simulated default gates == the run's own loop lines (cur, old, n_inl)."""
    acc = simulate(c, dict(pick=pick))
    sim = sorted((s, cur, e["o"], e["ni"]) for s, cur, e in acc)
    act = sorted((x["session"], x["cur"], x["old"], x["n_inl"]) for x in c.loops)
    return sim == act, len(sim), len(act)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["check"])
    ap.add_argument("cond")
    ap.add_argument("--pick", default="x")
    ap.add_argument("--set", default="dev")
    o = ap.parse_args()
    bad = 0
    for r, off in XR.cells(o.set):
        name = f"{XR.key_of(r, off, o.set)}_{o.cond}"
        if not (XR.OUT / o.cond / f"{name}.json").exists():
            continue
        ok, ns, na = check(Cell(o.cond, name, gt=False), o.pick)
        bad += not ok
        print(name, "SAME" if ok else "DIFF", ns, na)
    print("RESULT", "PASS" if not bad else f"FAIL {bad}")
