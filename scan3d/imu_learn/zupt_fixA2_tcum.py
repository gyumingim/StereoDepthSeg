#!/usr/bin/env python3
"""Choose T_cum for fix A2 from existing logs only (docs/zupt_fixA2_plan.md). No VIO runs.

In the fix A runs (runs/zupt_fixA/*_A) every ZUPT check logs '[ZUPT]: passed/failed disparity (d ...)' where d is the
mean disparity against the latest clone (= cumulative since the last normal update). Samples = those d at frames after
the first tracked pose (start still exempt in A2) where the GT camera speed < 0.05 cm/s (same GT speed as criterion 3).
T_cum = 95th percentile, rounded up to 0.1 px. Also reports, for information, d in the target semi-still windows and in
154320 o0 2.14-7.5 s.

  ../venv_ffs/bin/python imu_learn/zupt_fixA2_tcum.py   # -> runs/zupt_fixA2/tcum.json
"""
import gzip, json, re, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import zupt_fixA_analyse as an  # noqa: E402
import zupt_fixA as zf          # noqa: E402
RE = re.compile(r"\[ZUPT\]: (passed|failed) disparity \(([\d.]+) [<>] [\d.]+, (\d+) features\)")
RT = re.compile(r"\[DIAGT\] ([\d.]+)")
D = an.SCAN3D / "runs" / "zupt_fixA"


def disp(path):
    out, t = [], None
    for line in gzip.open(path, "rt", errors="replace"):
        line = an.ANSI.sub("", line)
        m = RT.search(line)
        if m:
            t = float(m.group(1)); continue
        m = RE.search(line)
        if m and t is not None:
            out.append((t, float(m.group(2)), int(m.group(3))))
    return out


def main():
    allv, per, tgt, d154 = [], {}, {}, None
    summ = json.load(open(D / "summary.json"))
    for c in zf.cells():
        k = c["key"]; pre = D / f"{k}_A"
        if not Path(str(pre) + ".stderr.log.gz").exists():
            continue
        j = json.load(open(str(pre) + ".json"))
        if j.get("ate_cm") is None:
            continue
        P = [json.loads(x) for x in gzip.open(str(pre) + ".poses.jsonl.gz", "rt") if x.strip()]
        tr = [p["t"] for p in P if p.get("status") == "tracking" and p.get("p")]
        t0 = tr[0] if tr else 1e9
        dd = disp(str(pre) + ".stderr.log.gz")
        sp = an.gt_speed_fn(Path(j["rec"]).name, j, P[0])
        if sp is not None:
            v = [d for t, d, n in dd if t >= t0 and n > 20 and (s := sp(t)) is not None and s < 0.05]
            if v:
                per[k] = dict(n=len(v), p50=float(np.percentile(v, 50)), p95=float(np.percentile(v, 95)), max=float(max(v)))
                allv += v
        if c["group"] == "target":
            b = summ["cells"][k]["base"]
            wins = [[a, e] for a, e, _ in b["pauses"] if e - a >= 1.0 and any(a <= w["t"] <= e + 2 for w in b["wipeouts"])]
            vv = [d for t, d, n in dd if any(a <= t <= e for a, e in wins)]
            gs = [sp(t) for t, d, n in dd if any(a <= t <= e for a, e in wins)] if sp else []
            tgt[k] = dict(windows=wins, n=len(vv), p50=float(np.median(vv)) if vv else None,
                          frac_gt_05=float(np.mean(np.array(vv) > 0.5)) if vv else None,
                          gt_speed_med_cms=float(np.median([g for g in gs if g is not None])) if gs else None, values=vv)
        if k == "Z_O_154320_o0":
            vv = [d for t, d, n in dd if t0 <= t <= 7.5]
            d154 = dict(n=len(vv), values=vv)
    p95 = float(np.percentile(allv, 95))
    T = float(np.ceil(p95 * 10 - 1e-9) / 10)
    for k, x in tgt.items():
        x["frac_gt_Tcum"] = float(np.mean(np.array(x["values"]) > T)) if x["values"] else None
    if d154:
        d154["frac_gt_Tcum"] = float(np.mean(np.array(d154["values"]) > T)) if d154["values"] else None
        d154["frac_gt_05"] = float(np.mean(np.array(d154["values"]) > 0.5)) if d154["values"] else None
    res = dict(made=time.strftime("%F %T"), source="runs/zupt_fixA/*_A logs", n_samples=len(allv), n_runs=len(per),
               p50=float(np.percentile(allv, 50)), p90=float(np.percentile(allv, 90)), p95=p95,
               p99=float(np.percentile(allv, 99)), max=float(max(allv)), T_cum=T, per_run=per, target=tgt, d154320_o0=d154)
    out = an.SCAN3D / "runs" / "zupt_fixA2"; out.mkdir(exist_ok=True)
    json.dump(res, open(out / "tcum.json", "w"), indent=1)
    print({k: v for k, v in res.items() if k not in ("per_run", "target", "d154320_o0")})
    for k, x in tgt.items():
        print(k, {a: b for a, b in x.items() if a != "values"})
    if d154:
        print("154320 o0", {a: b for a, b in d154.items() if a != "values"}, [round(v, 2) for v in d154["values"][:40]])


if __name__ == "__main__":
    main()
