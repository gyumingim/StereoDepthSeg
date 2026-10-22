#!/usr/bin/env python3
"""Analysis of the ZUPT fix A2 test (docs/zupt_fixA2_plan.md = criteria of docs/zupt_fixA_plan.md unchanged).
Copy of zupt_fixA_analyse.py; only changes: treatment = A2 (runs/zupt_fixA2/<cell>_A2), base read from runs/zupt_fixA.

  ../venv_ffs/bin/python imu_learn/zupt_fixA2_analyse.py      # -> runs/zupt_fixA2/summary.json
"""
import gzip
import hashlib
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SCAN3D = HERE.parent
OUT = SCAN3D / "runs" / "zupt_fixA2"
BASE_DIR = SCAN3D / "runs" / "zupt_fixA"
ANSI = re.compile(r"\x1b\[[0-9;]*m")
RE_T = re.compile(r"\[DIAGT\] ([\d.]+)")
RE_SL = re.compile(r"\[CHI2\] slam in (\d+) rej (\d+)")
RE_SN = re.compile(r"seconds for SLAM update \((\d+) feats")
RE_MN = re.compile(r"seconds for MSCKF update \((\d+) feats")
RE_DF = re.compile(r"passed disparity \(([\d.]+) < [\d.]+, (\d+) features\)")
GAP_MAX = 0.72


def parse(path):
    fr, cur = [], None
    for line in gzip.open(path, "rt", errors="replace"):
        line = ANSI.sub("", line)
        m = RE_T.search(line)
        if m:
            cur = dict(t=float(m.group(1)), zacc=False, sin=0, srej=0, slam=None, msckf=None, dfeat=None)
            fr.append(cur)
            continue
        if cur is None:
            continue
        if "[ZUPT]: accepted" in line:
            cur["zacc"] = True
        elif (m := RE_SL.search(line)):
            cur["sin"] += int(m.group(1))
            cur["srej"] += int(m.group(2))
        elif (m := RE_SN.search(line)):
            cur["slam"] = int(m.group(1))
        elif (m := RE_MN.search(line)):
            cur["msckf"] = int(m.group(1))
        elif (m := RE_DF.search(line)):
            cur["dfeat"] = int(m.group(2))
    return fr


def updates(fr):
    return [f for f in fr if not f["zacc"] and f["slam"] is not None]


def all_reject(up):
    return [round(f["t"], 3) for f in up if f["sin"] >= 10 and f["srej"] / f["sin"] >= 0.9]


def wipeouts(up):
    out, armed = [], True
    for j, f in enumerate(up):
        if f["slam"] >= 20:
            armed = True
        if not armed or f["slam"] > 3:
            continue
        k = j - 1
        while k >= 0 and up[k]["t"] >= f["t"] - 0.2:
            if up[k]["slam"] >= 20:
                out.append(f["t"])
                armed = False
                break
            k -= 1
    return out


def pauses(fr):
    """ZUPT-accepted streaks merged across gaps <= 0.3 s: [start, end, n_accepted]."""
    st, s = [], None
    for f in fr:
        if f["zacc"]:
            if s is None:
                s = [f["t"], f["t"], 0]
            s[1] = f["t"]
            s[2] += 1
        elif s is not None and f["slam"] is not None:
            st.append(s)
            s = None
    if s:
        st.append(s)
    ps = []
    for a, b, k in st:
        if ps and a - ps[-1][1] <= 0.3:
            ps[-1][1] = b
            ps[-1][2] += k
        else:
            ps.append([a, b, k])
    return ps


def poses(prefix):
    P = [json.loads(x) for x in gzip.open(str(prefix) + ".poses.jsonl.gz", "rt") if x.strip()]
    tr = [p for p in P if p.get("status") == "tracking" and p.get("p")]
    return P, np.array([p["t"] for p in tr]), np.array([p["p"] for p in tr])


def gt_speed_fn(rec, j, P0):
    f = SCAN3D / "recordings" / rec / "imu_gt_spl" / "gt.npz"
    if not f.exists():
        return None
    g = np.load(f, allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    c = g["p_bc"][ok].reshape(-1, 3)  # GT fix 2026-10-23: p_bc is already the camera centre (was -R_bc^T p_bc)
    off = P0["t_ns"] - P0["t"] * 1e9 + j["align_dt_ms"] * 1e6
    tt = (g["t_ns"][ok] - off) / 1e9

    def near(x):
        i = int(np.clip(np.searchsorted(tt, x), 1, len(tt) - 1))
        i = i - 1 if abs(tt[i - 1] - x) < abs(tt[i] - x) else i
        return i if abs(tt[i] - x) <= 0.02 else None

    def sp(x):
        a, b = near(x - 0.25), near(x + 0.25)
        if a is None or b is None or tt[b] <= tt[a]:
            return None
        return float(np.linalg.norm(c[b] - c[a]) / (tt[b] - tt[a]) * 100)
    return sp


def run_info(name, d=OUT):
    pre = d / name
    if not Path(str(pre) + ".json").exists():
        return None
    j = json.load(open(str(pre) + ".json"))
    if j.get("error") or j.get("ate_cm") is None or not Path(str(pre) + ".stderr.log.gz").exists():
        return dict(name=name, result=False, error=j.get("error"), ate_cm=j.get("ate_cm"))
    fr = parse(str(pre) + ".stderr.log.gz")
    up = updates(fr)
    P, vt, vp = poses(pre)
    wo = wipeouts(up)
    wd = []
    for t in wo:
        m = (vt >= t) & (vt <= t + 5)
        d = float(np.max(np.linalg.norm(vp[m] - vp[m][0], axis=1))) if m.sum() >= 2 else None
        wd.append(dict(t=round(t, 3), maxdisp5_m=None if d is None else round(d, 3), collapse=bool(d is not None and d > 0.5)))
    rec = Path(j["rec"]).name
    sp = gt_speed_fn(rec, j, P[0])
    t_init = j.get("t_init_clip_s") or 0
    zacc = [f["t"] for f in fr if f["zacc"]]
    still = None
    if sp is not None:
        v = [sp(t) for t in zacc]                      # plan: all ZUPT-accepted frames (no init filter)
        still = int(sum(1 for x in v if x is not None and x < 0.05))
    return dict(name=name, result=True, ate_cm=j["ate_cm"], scale=j.get("scale"), t_init=t_init,
                n_update=len(up), n_zupt=len(zacc), zupt_true_still=still, gt=sp is not None,
                all_reject=all_reject(up), wipeouts=wd,
                collapse=bool(any(w["collapse"] for w in wd) or j["ate_cm"] > 50),
                n_disp_feat_le20=sum(1 for f in fr if f["dfeat"] is not None and f["dfeat"] <= 20),
                pauses=[[round(a, 3), round(b, 3), k] for a, b, k in pauses(fr)],
                upd_t=[f["t"] for f in up], lib=next((v for k, v in j.get("binary_sha256", {}).items() if k.endswith(".so")), None),
                cfg_sha=j.get("ov_config_sha256"), zfa_head=j.get("zfa_head"), zfa_dirty=j.get("zfa_dirty"))


def target_windows(b):
    """baseline pauses >= 1 s that contain a baseline wipe-out or have one within 2 s after their end."""
    out = []
    for a, e, k in b["pauses"]:
        if e - a >= 1.0 and any(a <= w["t"] <= e + 2 for w in b["wipeouts"]):
            out.append([a, e])
    return out


def max_gap(upd_t, a, e):
    t = np.asarray(upd_t)
    lo = np.searchsorted(t, a, side="right") - 1        # last update at/before a
    hi = np.searchsorted(t, e, side="left")              # first update at/after e
    seg = t[max(lo, 0):min(hi + 1, len(t))]
    return float(np.max(np.diff(seg))) if len(seg) >= 2 else None


def analyse():
    sys.path.insert(0, str(HERE))
    import zupt_fixA2 as zf
    cells = zf.cells()
    res = dict(made=time.strftime("%F %T"), plan="docs/zupt_fixA2_plan.md", cells={}, missing=[])
    for c in cells:
        b, a = run_info(f"{c['key']}_base", BASE_DIR), run_info(f"{c['key']}_A2")
        if b is None or a is None:
            res["missing"].append(c["key"] + ("" if b else " base") + ("" if a else " A2"))
            continue
        res["cells"][c["key"]] = dict(rec=c["rec"], group=c["group"], sub=c["sub"], base=b, A2=a)
    C = res["cells"]
    v = {}
    # criterion 1
    tg = {k: x for k, x in C.items() if x["group"] == "target"}
    c1 = []
    for k, x in sorted(tg.items()):
        A = x["A2"]
        lim = 10.0 if "133820" in k else (5.0 if k == "BR_134550_o2" else None)
        ok = A["result"] and not A["all_reject"] and not A["wipeouts"] and (lim is None or A["ate_cm"] <= lim)
        c1.append(dict(cell=k, ok=bool(ok), A2_all_reject=A.get("all_reject"), A2_wipeouts=A.get("wipeouts"),
                       A2_ate=A.get("ate_cm"), base_ate=x["base"].get("ate_cm"), ate_limit=lim,
                       base_all_reject=x["base"].get("all_reject"), base_wipeouts=x["base"].get("wipeouts")))
    v["c1"] = dict(ok=len(c1) == 6 and all(r["ok"] for r in c1), n=len(c1), rows=c1)
    # criterion 2 (all non-target runs)
    c2 = []
    for k, x in sorted(C.items()):
        if x["group"] == "target":
            continue
        b, A = x["base"], x["A2"]
        if not A["result"]:
            c2.append(dict(cell=k, ok=not b["result"], why="A2 no result" + ("" if b["result"] else " (base none too)")))
            continue
        if not b["result"]:
            c2.append(dict(cell=k, ok=True, why="base no result", A2_ate=A["ate_cm"]))
            continue
        bt = [w["t"] for w in b["wipeouts"]]
        new_w = [w for w in A["wipeouts"] if not any(abs(w["t"] - t) <= 1.0 for t in bt)]
        new_c = A["collapse"] and not b["collapse"]
        c2.append(dict(cell=k, ok=not new_w and not new_c, new_wipeouts=new_w, new_collapse=new_c,
                       base_wipeouts=b["wipeouts"], A2_wipeouts=A["wipeouts"], base_ate=b["ate_cm"], A2_ate=A["ate_cm"],
                       base_collapse=b["collapse"], A2_collapse=A["collapse"]))
    v["c2"] = dict(ok=all(r["ok"] for r in c2), n=len(c2), fails=[r for r in c2 if not r["ok"]],
                   base_collapse_cells=[r["cell"] for r in c2 if r.get("base_collapse")],
                   lost_collapse_cells=[r["cell"] for r in c2 if r.get("base_collapse") and not r.get("A2_collapse")])
    # criterion 3
    r34 = [dict(cell=k, base=x["base"].get("ate_cm"), A2=x["A2"].get("ate_cm"),
                lim=round(x["base"]["ate_cm"] * 1.2 + 0.5, 3) if x["base"].get("ate_cm") is not None else None)
           for k, x in sorted(C.items()) if x["group"] == "control"]
    for r in r34:
        r["ok"] = r["A2"] is not None and r["lim"] is not None and r["A2"] <= r["lim"]
    r45 = [dict(cell=k, base=x["base"].get("ate_cm"), A2=x["A2"].get("ate_cm"),
                lim=round(x["base"]["ate_cm"] * 1.1, 3) if x["base"].get("ate_cm") is not None else None)
           for k, x in sorted(C.items()) if x["rec"] == "hs_20261021_214550"]
    for r in r45:
        r["ok"] = r["A2"] is not None and r["lim"] is not None and r["A2"] <= r["lim"]
    rst, small = [], []
    for k, x in sorted(C.items()):
        b, A = x["base"], x["A2"]
        if not (b["result"] and A["result"]) or not b.get("gt"):
            small.append(dict(cell=k, why="no GT or no result"))
            continue
        if b["zupt_true_still"] < 30:
            small.append(dict(cell=k, why=f"base true-still ZUPT {b['zupt_true_still']} < 30", A2=A["zupt_true_still"]))
            continue
        rst.append(dict(cell=k, base=b["zupt_true_still"], A2=A["zupt_true_still"],
                        ratio=round(A["zupt_true_still"] / b["zupt_true_still"], 4),
                        ok=A["zupt_true_still"] >= 0.9 * b["zupt_true_still"]))
    v["c3"] = dict(ok=bool(len(r34) == 3 and len(r45) == 3 and all(r["ok"] for r in r34 + r45 + rst)),
                   ate_134204=r34, ate_214550=r45, still_ok=all(r["ok"] for r in rst), still_rows=rst,
                   still_fails=[r for r in rst if not r["ok"]], still_skipped=small,
                   still_ratio_min=min((r["ratio"] for r in rst), default=None),
                   still_ratio_median=float(np.median([r["ratio"] for r in rst])) if rst else None)
    # criterion 4
    c4 = []
    for k, x in sorted(tg.items()):
        wins = target_windows(x["base"])
        g = [dict(win=[round(a, 3), round(e, 3)], base_gap=max_gap(x["base"]["upd_t"], a, e),
                  A2_gap=max_gap(x["A2"]["upd_t"], a, e)) for a, e in wins]
        ok = bool(wins) and all(r["A2_gap"] is not None and r["A2_gap"] < GAP_MAX for r in g)
        c4.append(dict(cell=k, ok=ok, windows=g, A2_wipeouts=len(x["A2"]["wipeouts"]), A2_ate=x["A2"]["ate_cm"]))
    v["c4"] = dict(ok=len(c4) == 6 and all(r["ok"] for r in c4), rows=c4)
    v["all_ok"] = all(v[c]["ok"] for c in ("c1", "c2", "c3", "c4"))
    res["verdict"] = v
    # provenance
    libs = {k: sorted({x[k]["lib"] for x in C.values() if x[k].get("lib")}) for k in ("base", "A2")}
    cfg = sorted({json.dumps(x[c].get("cfg_sha"), sort_keys=True) for x in C.values() for c in ("base", "A2")
                  if x[c].get("cfg_sha")})
    res["provenance"] = dict(lib_sha256=libs, n_config_variants=len(cfg),
                             heads=sorted({x[c].get("zfa_head") for x in C.values() for c in ("base", "A2") if x[c].get("zfa_head")}),
                             any_dirty=any(x[c].get("zfa_dirty") for x in C.values() for c in ("base", "A2")),
                             recordings=sorted({x["rec"] for x in C.values()}),
                             analyse_sha256=hashlib.sha256(open(__file__, "rb").read()).hexdigest())
    for x in C.values():
        for c in ("base", "A2"):
            x[c].pop("upd_t", None)
    return res


if __name__ == "__main__":
    r = analyse()
    json.dump(r, open(OUT / "summary.json", "w"), indent=1, ensure_ascii=False)
    v = r["verdict"]
    print("missing", r["missing"])
    for c in ("c1", "c2", "c3", "c4"):
        print(c, "OK" if v[c]["ok"] else "FAIL")
    for row in v["c1"]["rows"]:
        print(" c1", row["cell"], row["ok"], "ate", row["base_ate"], "->", row["A2_ate"], "allrej", len(row["base_all_reject"] or []),
              "->", len(row["A2_all_reject"] or []), "wipe", len(row["base_wipeouts"] or []), "->", len(row["A2_wipeouts"] or []))
    print(" c2 fails", json.dumps(v["c2"]["fails"])[:1500])
    print(" c3 134204", v["c3"]["ate_134204"], "214550", v["c3"]["ate_214550"])
    print(" c3 still min", v["c3"]["still_ratio_min"], "median", v["c3"]["still_ratio_median"], "fails", v["c3"]["still_fails"])
    for row in v["c4"]["rows"]:
        print(" c4", row["cell"], row["ok"], row["windows"])
    print("ALL", v["all_ok"], r["provenance"])
