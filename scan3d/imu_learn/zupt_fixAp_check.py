#!/usr/bin/env python3
"""Independent checker, ZUPT fix A' test (docs/zupt_fixAp_plan.md; criteria = docs/zupt_fixA_plan.md unchanged).
Copy of zupt_fixA_check.py; only changes: treatment = A' (runs/zupt_fixAp/<cell>_Ap), base from runs/zupt_fixA.

Written from the plan only (does NOT import zupt_fixA_analyse.py). numpy + stdlib.

  python imu_learn/zupt_fixAp_check.py            # check runs/zupt_fixAp/*_Ap vs runs/zupt_fixA/*_base -> runs/zupt_fixAp/check.json
  python imu_learn/zupt_fixA_check.py --selftest # parser check on old board_rotation / i3_confirm logs

Event definitions (plan, "로그에서 사건 읽는 법"):
  frame          = from a '[DIAGT] t' line up to the next '[DIAGT]' (ANSI codes stripped)
  ZUPT frame     = contains '[ZUPT]: accepted'
  normal update  = not ZUPT and contains 'seconds for SLAM update (N feats)'; N = SLAM count;
                   in/rej = sums over all '[CHI2] slam in X rej Y' lines of the frame
  all-reject     = normal update with in >= 10 and rej/in >= 0.9
  wipe-out       = normal update j with N_j <= 3 and some normal update i, t_j-0.2 <= t_i < t_j, N_i >= 20;
                   re-armed only after N >= 20 again
  collapse/wipe  = max |p - p_first| over tracked poses in [t, t+5] > 0.5 m
  run collapse   = any wipe collapse or ate_cm > 50
"""
import argparse, gzip, json, re, sys, time
from pathlib import Path

import numpy as np

OUT = Path("runs/zupt_fixAp")
BASE_DIR = Path("runs/zupt_fixA")
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
RE_DIAGT = re.compile(r"\[DIAGT\]\s+(-?[0-9.eE+-]+)")
RE_CHI2 = re.compile(r"\[CHI2\] slam in (\d+) rej (\d+)")
RE_SLAMUPD = re.compile(r"seconds for SLAM update \((\d+) feats\)")
RE_PASSDISP = re.compile(r"\[ZUPT\]: passed disparity \(.*?,\s*(\d+) features\)")

TARGET_RECS = ("133820", "134550")
CONTROL_RECS = ("134204",)


# ----------------------------------------------------------------------------- parsing
def parse_log(path):
    """-> list of frames: dict(t, zupt, upd, n, cin, crej, pdisp_feats)"""
    frames, cur = [], None
    with gzip.open(path, "rt", errors="replace") as f:
        for raw in f:
            line = ANSI.sub("", raw)
            m = RE_DIAGT.search(line)
            if m:
                cur = dict(t=float(m.group(1)), zupt=False, upd=False, n=None, cin=0, crej=0, pdisp=None)
                frames.append(cur)
                continue
            if cur is None:
                continue
            if "[ZUPT]: accepted" in line:
                cur["zupt"] = True
            m = RE_SLAMUPD.search(line)
            if m:
                cur["upd"] = True
                cur["n"] = int(m.group(1))
            m = RE_CHI2.search(line)
            if m:
                cur["cin"] += int(m.group(1))
                cur["crej"] += int(m.group(2))
            m = RE_PASSDISP.search(line)
            if m:
                cur["pdisp"] = int(m.group(1))
    for fr in frames:
        fr["normal"] = fr["upd"] and not fr["zupt"]
    return frames


def events(frames):
    normal = [f for f in frames if f["normal"]]
    allrej = [f["t"] for f in normal if f["cin"] >= 10 and f["crej"] / f["cin"] >= 0.9]
    wipes, armed = [], True
    for j, fj in enumerate(normal):
        if fj["n"] >= 20:
            armed = True
        if armed and fj["n"] <= 3:
            i = j - 1
            hit = False
            while i >= 0 and normal[i]["t"] >= fj["t"] - 0.2:
                if normal[i]["n"] >= 20:
                    hit = True
                    break
                i -= 1
            if hit:
                wipes.append(fj["t"])
                armed = False
    # NOT used for judging (extra record): variant that also counts a drop when the immediately preceding
    # normal update (ZUPT frames in between skipped, any time gap) had N >= 20. The pre-registered 0.2 s rule
    # misses e.g. 214550 base o0 88.77 s (last N>=20 update at 88.483 s, 0.28 s of ZUPT frames in between).
    wipes_alt, armed = [], True
    for j, fj in enumerate(normal):
        if fj["n"] >= 20:
            armed = True
        if armed and fj["n"] <= 3 and j > 0:
            hit = normal[j - 1]["n"] >= 20
            i = j - 1
            while not hit and i >= 0 and normal[i]["t"] >= fj["t"] - 0.2:
                hit = normal[i]["n"] >= 20
                i -= 1
            if hit:
                wipes_alt.append(fj["t"])
                armed = False
    zupt_t = [f["t"] for f in frames if f["zupt"]]
    lowfeat = sum(1 for f in frames if f["pdisp"] is not None and f["pdisp"] <= 20)
    return dict(allrej=allrej, wipes=wipes, wipes_alt=wipes_alt, zupt_t=zupt_t, normal_t=[f["t"] for f in normal],
                n_zupt=len(zupt_t), n_pdisp_le20=lowfeat)


def load_poses(path):
    first, tp = None, []
    with gzip.open(path, "rt") as f:
        for line in f:
            d = json.loads(line)
            if "t" not in d or "t_ns" not in d:
                continue
            if first is None:
                first = d
            if d.get("status") == "tracking" and d.get("p") is not None:
                tp.append((d["t"], *d["p"]))
    arr = np.array(tp, float) if tp else np.zeros((0, 4))
    return first, arr


def wipe_disp(poses, t0):
    sel = poses[(poses[:, 0] >= t0) & (poses[:, 0] <= t0 + 5.0)]
    if len(sel) == 0:
        return None
    return float(np.max(np.linalg.norm(sel[:, 1:] - sel[0, 1:], axis=1)))


# ----------------------------------------------------------------------------- GT still count
_gt_cache = {}


def gt_centres(rec):
    if rec in _gt_cache:
        return _gt_cache[rec]
    p = Path(f"recordings/{rec}/imu_gt_spl/gt.npz")
    if not p.exists():
        _gt_cache[rec] = None
        return None
    z = np.load(p)
    ok = z["ok"].astype(bool)
    p_bc = z["p_bc"].reshape(-1, 3)
    R_bc = z["R_bc"].reshape(-1, 3, 3)
    keep = ok & (np.linalg.norm(p_bc, axis=1) < 1.2)
    c = p_bc[keep]  # GT fix 2026-10-23: p_bc is already the camera centre (was -R_bc^T p_bc)
    _gt_cache[rec] = (z["t_ns"][keep].astype(np.int64), c)
    return _gt_cache[rec]


def still_zupt_count(rec, pose0, align_dt_ms, zupt_t):
    g = gt_centres(rec)
    if g is None or pose0 is None or align_dt_ms is None:
        return None, None
    t_ns, c = g
    off_ns = pose0["t_ns"] - pose0["t"] * 1e9 + align_dt_ms * 1e6
    tg = (t_ns - off_ns) / 1e9
    order = np.argsort(tg)
    tg, c = tg[order], c[order]

    def nearest(t):
        k = np.searchsorted(tg, t)
        best = None
        for kk in (k - 1, k):
            if 0 <= kk < len(tg) and (best is None or abs(tg[kk] - t) < abs(tg[best] - t)):
                best = kk
        if best is None or abs(tg[best] - t) > 0.02:
            return None
        return c[best]

    n_still, n_eval = 0, 0
    for t in zupt_t:
        a, b = nearest(t - 0.25), nearest(t + 0.25)
        if a is None or b is None:
            continue
        n_eval += 1
        if np.linalg.norm(b - a) / 0.5 < 0.0005:                   # 0.05 cm/s in m/s
            n_still += 1
    return n_still, n_eval


# ----------------------------------------------------------------------------- run analysis
def analyse_run(stem, rec_hint=None):
    stem = Path(stem)
    jp = Path(f"{stem}.json")
    r = dict(name=stem.name, exists=jp.exists())
    if not jp.exists():
        return r
    d = json.load(open(jp, encoding="utf-8"))
    r.update(rec=d.get("rec") or rec_hint, ate_cm=d.get("ate_cm"), error=d.get("error"),
             align_dt_ms=d.get("align_dt_ms"), json_mtime=time.strftime("%F %T", time.localtime(jp.stat().st_mtime)))
    lp, pp = Path(f"{stem}.stderr.log.gz"), Path(f"{stem}.poses.jsonl.gz")
    r["has_log"], r["has_poses"] = lp.exists(), pp.exists()
    if not lp.exists():
        return r
    fr = parse_log(lp)
    ev = events(fr)
    r.update(n_frames=len(fr), n_normal=len(ev["normal_t"]), allrej_t=ev["allrej"], n_allrej=len(ev["allrej"]),
             wipe_t=ev["wipes"], n_wipe=len(ev["wipes"]), wipe_alt_t_unjudged=ev["wipes_alt"], n_zupt=ev["n_zupt"], n_pdisp_le20=ev["n_pdisp_le20"])
    r["_zupt_t"], r["_normal_t"] = ev["zupt_t"], ev["normal_t"]
    r["_seq"] = [(f["t"], f["zupt"]) for f in fr]
    pose0, poses = (load_poses(pp) if pp.exists() else (None, np.zeros((0, 4))))
    disps = [wipe_disp(poses, t) for t in ev["wipes"]]
    r["wipe_disp5_m"] = disps
    r["wipe_collapse"] = [bool(x is not None and x > 0.5) for x in disps]
    ate = r["ate_cm"]
    r["has_result"] = ate is not None and not r.get("error")
    r["collapse"] = bool(any(r["wipe_collapse"]) or (ate is not None and ate > 50))
    s, e = still_zupt_count(r["rec"], pose0, r["align_dt_ms"], ev["zupt_t"])
    r["still_zupt"], r["still_zupt_evaluable"] = s, e
    return r


def still_windows(base):
    """Plan crit-4: merged base ZUPT runs (gap <= 0.3 s), length >= 1 s, base wipe inside or within 2 s after end."""
    segs, cur = [], None
    for t, is_z in base["_seq"]:                      # contiguous ZUPT-accepted frames in log order
        if is_z:
            cur = [t, t] if cur is None else [cur[0], t]
        else:
            if cur:
                segs.append(cur)
            cur = None
    if cur:
        segs.append(cur)
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] <= 0.3:
            merged[-1][1] = s[1]
        else:
            merged.append(list(s))
    out = []
    for a, b in merged:
        if b - a >= 1.0 and any(a <= w <= b + 2.0 for w in base["wipe_t"]):
            out.append((a, b))
    return out


def max_gap(normal_t, a, b):
    nt = np.array(sorted(normal_t))
    if len(nt) == 0:
        return None
    inside = nt[(nt >= a) & (nt <= b)]
    before, after = nt[nt < a], nt[nt > b]
    seq = ([before[-1]] if len(before) else []) + list(inside) + ([after[0]] if len(after) else [])
    if len(seq) < 2:
        return None
    return float(np.max(np.diff(seq)))


def cell_group(cell):
    if cell.startswith("BR_"):
        rec6 = cell.split("_")[1]
        if rec6 in TARGET_RECS:
            return "target"
        if rec6 in CONTROL_RECS:
            return "control"
    return "regress"


def public(r):
    return {k: v for k, v in r.items() if not k.startswith("_")}


# ----------------------------------------------------------------------------- main check
def main_check(outdir):
    cells = sorted({p.name[:-len("_base.json")] for p in BASE_DIR.glob("*_base.json")} |
                   {p.name[:-len("_Ap.json")] for p in outdir.glob("*_Ap.json")})
    res, missing = {}, []
    for c in cells:
        b, a = analyse_run(BASE_DIR / f"{c}_base"), analyse_run(outdir / f"{c}_Ap")
        if not (b["exists"] and a["exists"]):
            missing.append(dict(cell=c, base=b["exists"], Ap=a["exists"]))
            continue
        res[c] = dict(group=cell_group(c), base=b, Ap=a)
    expected_targets = [f"BR_{r}_o{k}" for r in TARGET_RECS for k in range(3)]
    expected_core = expected_targets + [f"BR_134204_o{k}" for k in range(3)] + [f"I3_214550_o{k}" for k in range(3)]
    core_missing = [c for c in expected_core if c not in res]

    # ---- criterion 1
    c1 = dict(items=[], missing=[c for c in expected_targets if c not in res])
    for c in expected_targets:
        if c not in res:
            continue
        A = res[c]["Ap"]
        it = dict(cell=c, n_allrej=A.get("n_allrej"), n_wipe=A.get("n_wipe"), ate_cm=A.get("ate_cm"),
                  base_ate_cm=res[c]["base"].get("ate_cm"))
        lim = 10.0 if c.startswith("BR_133820") else (5.0 if c == "BR_134550_o2" else None)
        it["ate_limit"] = lim
        ok = A.get("n_allrej") == 0 and A.get("n_wipe") == 0
        if lim is not None:
            ok = ok and A.get("ate_cm") is not None and A["ate_cm"] <= lim
        it["pass"] = bool(ok)
        c1["items"].append(it)
    c1["pass"] = (not c1["missing"]) and all(i["pass"] for i in c1["items"])

    # ---- criterion 2 (all non-target runs, control included)
    c2 = dict(items=[])
    for c, v in res.items():
        if v["group"] == "target":
            continue
        B, A = v["base"], v["Ap"]
        it = dict(cell=c, group=v["group"])
        if B.get("has_result") and not A.get("has_result"):
            it.update(pass_=False, why="base has result, A has none")
        else:
            new_w = [w for w in A.get("wipe_t", []) if not any(abs(w - x) <= 1.0 for x in B.get("wipe_t", []))]
            new_col = bool(A.get("collapse")) and not B.get("collapse")
            it.update(new_wipes=new_w, base_collapse=B.get("collapse"), A_collapse=A.get("collapse"),
                      new_collapse=new_col, pass_=not new_w and not new_col)
            if not B.get("has_result"):
                it["note"] = "base has no result"
        c2["items"].append(it)
    c2["failing"] = [i["cell"] for i in c2["items"] if not i["pass_"]]
    c2["pass"] = not c2["failing"]

    # ---- criterion 3
    c3a = []
    for k in range(3):
        c = f"BR_134204_o{k}"
        if c in res:
            b, a = res[c]["base"].get("ate_cm"), res[c]["Ap"].get("ate_cm")
            lim = None if b is None else b * 1.2 + 0.5
            c3a.append(dict(cell=c, base=b, Ap=a, limit=lim, pass_=lim is not None and a is not None and a <= lim))
    c3b = []
    for k in range(3):
        c = f"I3_214550_o{k}"
        if c in res:
            b, a = res[c]["base"].get("ate_cm"), res[c]["Ap"].get("ate_cm")
            lim = None if b is None else b * 1.1
            c3b.append(dict(cell=c, base=b, Ap=a, limit=lim, pass_=lim is not None and a is not None and a <= lim))
    c3c, c3c_listed = [], []
    for c, v in res.items():
        sb, sa = v["base"].get("still_zupt"), v["Ap"].get("still_zupt")
        row = dict(cell=c, base=sb, Ap=sa, base_eval=v["base"].get("still_zupt_evaluable"),
                   Ap_eval=v["Ap"].get("still_zupt_evaluable"))
        if sb is None or sb < 30:
            c3c_listed.append(row)
        else:
            row["pass_"] = sa is not None and sa >= 0.9 * sb
            c3c.append(row)
    c3 = dict(ate_134204=dict(items=c3a, pass_=len(c3a) == 3 and all(i["pass_"] for i in c3a)),
              ate_214550=dict(items=c3b, pass_=len(c3b) == 3 and all(i["pass_"] for i in c3b)),
              still_zupt=dict(items=c3c, listed_only=c3c_listed, pass_=all(i["pass_"] for i in c3c)))
    c3["pass"] = c3["ate_134204"]["pass_"] and c3["ate_214550"]["pass_"] and c3["still_zupt"]["pass_"]

    # ---- criterion 4
    c4 = dict(items=[])
    for c in expected_targets:
        if c not in res:
            continue
        B, A = res[c]["base"], res[c]["Ap"]
        if "_seq" not in B or "_normal_t" not in A:
            c4["items"].append(dict(cell=c, windows=[], pass_=False, why="log missing"))
            continue
        wins = still_windows(B)
        gaps = [max_gap(A["_normal_t"], a, b) for a, b in wins]
        ok = bool(wins) and all(g is not None and g < 0.72 for g in gaps)
        c4["items"].append(dict(cell=c, windows=[[round(a, 3), round(b, 3)] for a, b in wins],
                                A_max_gap_s=gaps, A_n_wipe=A.get("n_wipe"), A_ate_cm=A.get("ate_cm"), pass_=ok,
                                why=None if wins else "no base half-still window found (unevaluable)"))
    c4["pass_line1"] = len(c4["items"]) == 6 and all(i["pass_"] for i in c4["items"])
    zupt_broken = c4["pass_line1"]
    wipes_left = sum((i.get("A_n_wipe") or 0) for i in c4["items"])
    big_ate = [i["cell"] for i in c4["items"] if (i.get("A_ate_cm") or 0) > 10]
    if not zupt_broken:
        c4["line2_3"] = "ZUPT gap not removed in all 6 target runs -> lines 2/3 not applicable"
    elif wipes_left:
        c4["line2_3"] = f"ZUPT gap removed but {wipes_left} wipe-outs remain -> 'attitude locked during ZUPT' hypothesis REJECTED"
    elif big_ate:
        c4["line2_3"] = f"ZUPT gap removed, wipe-outs gone, but ATE > 10 cm in {big_ate} -> main cause: missing MSCKF"
    else:
        c4["line2_3"] = "ZUPT gap removed, no wipe-outs, ATE small -> consistent with hypothesis"

    fail_runs = sorted(set([i["cell"] for i in c1["items"] if not i["pass"]] + c2["failing"] +
                           [i["cell"] for i in c3a + c3b + c3c if not i["pass_"]] +
                           [i["cell"] for i in c4["items"] if not i["pass_"]]))
    out = dict(made=time.strftime("%F %T"), script="imu_learn/zupt_fixAp_check.py", plan="docs/zupt_fixAp_plan.md",
               n_cells=len(res), missing_cells=missing, core_missing=core_missing,
               runs_used=sorted(f"{c}_{k}" for c in res for k in ("base", "Ap")),
               criteria=dict(c1=c1, c2=c2, c3=c3, c4=c4), failing_runs=fail_runs,
               adopt_Ap=bool(c1["pass"] and c2["pass"] and c3["pass"] and c4["pass_line1"]),
               cells={c: dict(group=v["group"], base=public(v["base"]), Ap=public(v["Ap"])) for c, v in res.items()})
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "check.json").write_text(json.dumps(out, indent=1, default=float))

    # ---- summary
    print(f"cells with base+A: {len(res)}   missing/partial: {len(missing)}   core missing: {core_missing or '-'}")
    print(f"{'cell':22s} {'grp':7s} {'ATE b/A':>15s} {'rej b/A':>8s} {'wipe b/A':>9s} {'col b/A':>8s} {'zupt b/A':>11s} {'still b/A':>10s}")
    for c, v in sorted(res.items()):
        B, A = v["base"], v["Ap"]
        f = lambda x: "-" if x is None else (f"{x:.1f}" if isinstance(x, float) else str(x))
        print(f"{c:22s} {v['group']:7s} {f(B.get('ate_cm')):>7s}/{f(A.get('ate_cm')):<7s} "
              f"{f(B.get('n_allrej')):>3s}/{f(A.get('n_allrej')):<4s} {f(B.get('n_wipe')):>4s}/{f(A.get('n_wipe')):<4s} "
              f"{int(bool(B.get('collapse')))}/{int(bool(A.get('collapse'))):<6d} {f(B.get('n_zupt')):>5s}/{f(A.get('n_zupt')):<5s} "
              f"{f(B.get('still_zupt')):>4s}/{f(A.get('still_zupt')):<5s}")
    P = lambda b: "PASS" if b else "FAIL"
    alt = {c: (len(v["base"].get("wipe_alt_t_unjudged", [])), len(v["Ap"].get("wipe_alt_t_unjudged", [])))
           for c, v in res.items() if v["base"].get("wipe_alt_t_unjudged") != v["base"].get("wipe_t")
           or v["Ap"].get("wipe_alt_t_unjudged") != v["Ap"].get("wipe_t")}
    if alt:
        print(f"(unjudged) cells where variant wipe count differs, base/A: {alt}")
    print(f"C1 block target    : {P(c1['pass'])}" + (f"  (missing {c1['missing']})" if c1["missing"] else ""))
    print(f"C2 no new collapse : {P(c2['pass'])}  failing {c2['failing'] or '-'}")
    print(f"C3a 134204 ATE     : {P(c3['ate_134204']['pass_'])}  ({len(c3a)}/3 present)")
    print(f"C3b 214550 ATE     : {P(c3['ate_214550']['pass_'])}  ({len(c3b)}/3 present)")
    print(f"C3c still ZUPT     : {P(c3['still_zupt']['pass_'])}  ({len(c3c)} judged, {len(c3c_listed)} listed only)")
    for i in c4["items"]:
        print(f"C4 {i['cell']}: windows {i['windows']} A max gap {i.get('A_max_gap_s')} -> {P(i['pass_'])}")
    print(f"C4 line1           : {P(c4['pass_line1'])};  {c4['line2_3']}")
    print(f"failing runs: {fail_runs or '-'}")
    print(f"ADOPT Ap: {out['adopt_Ap']}   -> {outdir / 'check.json'}")


# ----------------------------------------------------------------------------- selftest
def selftest():
    cases = [("runs/board_rotation/133820_base_o0", "wipe ~37.148, all-reject ~37.031/37.115, ATE ~375"),
             ("runs/board_rotation/134204_base_o0", "no wipe-out"),
             ("runs/i3_confirm/214550_base_o0", "wipe ~88.77")]
    ok = True
    for stem, exp in cases:
        r = analyse_run(stem)
        if not r.get("exists"):
            print(f"{stem}: MISSING");
            ok = False
            continue
        print(f"{stem}  (expect: {exp})")
        print(f"  frames {r['n_frames']} normal {r['n_normal']} zupt {r['n_zupt']} pdisp<=20feat {r['n_pdisp_le20']}")
        print(f"  all-reject {r['n_allrej']} @ {[round(t, 3) for t in r['allrej_t']][:10]}")
        print(f"  wipe-outs {r['n_wipe']} @ {[round(t, 3) for t in r['wipe_t']]}  5s disp m {[None if d is None else round(d, 3) for d in r['wipe_disp5_m']]}")
        print(f"  unjudged variant wipe-outs @ {[round(t, 3) for t in r['wipe_alt_t_unjudged']]}")
        print(f"  ATE {r['ate_cm']} collapse {r['collapse']}  still-ZUPT {r['still_zupt']} (of {r['still_zupt_evaluable']} evaluable)")
        wins = still_windows(r)
        print(f"  crit-4 windows (base) {[[round(a, 3), round(b, 3)] for a, b in wins]} self max gap "
              f"{[max_gap(r['_normal_t'], a, b) for a, b in wins]}")
    r1 = analyse_run(cases[0][0]); r2 = analyse_run(cases[1][0]); r3 = analyse_run(cases[2][0])
    chk = [("133820 wipe at 37.148", any(abs(t - 37.148) < 0.01 for t in r1["wipe_t"])),
           ("133820 all-reject at 37.031 & 37.115", all(any(abs(t - x) < 0.01 for t in r1["allrej_t"]) for x in (37.031, 37.115))),
           ("133820 ATE ~375 & collapse", abs(r1["ate_cm"] - 375) < 1 and r1["collapse"]),
           ("134204 no wipe-out", r2["n_wipe"] == 0),
           # plan's literal 0.2 s rule cannot see 88.77 (last N>=20 normal update 0.28 s earlier); the
           # unjudged variant must see it, and the literal rule must NOT (documents the definition gap)
           ("214550 88.77 not a wipe under literal 0.2 s rule", not any(abs(t - 88.77) < 0.05 for t in r3["wipe_t"])),
           ("214550 wipe near 88.77 under unjudged variant", any(abs(t - 88.77) < 0.05 for t in r3["wipe_alt_t_unjudged"]))]
    for name, v in chk:
        print(f"  [{'OK' if v else 'FAIL'}] {name}")
        ok = ok and v
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--dir", default=str(OUT))
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())
    main_check(Path(a.dir))
