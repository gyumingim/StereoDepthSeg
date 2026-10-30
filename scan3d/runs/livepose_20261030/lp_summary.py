#!/usr/bin/env python3
"""livepose summary: baseline table + paired comparisons (README 2.5 rules) -> summary.json + summary.md (this folder)."""
import json, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
RUNS = Path.home() / "scanner/tmp_livepose/runs"
import lp_batch as LB  # noqa: E402


def load(cond):
    out = {}
    for f in sorted((RUNS / cond).glob("*.json")):
        r = json.load(open(f))
        out[r["rec"]] = r
    return out


def metric(r, set_):
    s = r["score"]
    if set_ == "L":
        return s.get("loop_err_cm") if s.get("status") == "ok" else None
    return s.get("ate_vio_cm") if s.get("status") == "ok" else None


def fmt(v, d=2):
    return "—" if v is None else f"{v:.{d}f}"


def main():
    allc = {c: load(c) for c in LB.CONDS}
    setof = {r: s for s, rs in LB.SETS.items() for r in rs}
    B = allc["B"]
    md = [f"made {time.strftime('%F %T')} from {RUNS}", ""]
    # ---- baseline per recording
    md += ["## Baseline (B = live config) per recording, offsets 0/1/2 s", "",
           "| set | rec | ok/3 | VIO ATE cm o0/o1/o2 | map ATE o0 | path m | ATE % | drift cm/m | rot ° | init s | dt final ms | loop err cm (vio/map) | n_loop |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    base = []
    for s in "SLH":
        for r in LB.SETS[s]:
            rr = [allc[c].get(r) for c in ("B", "B_o1", "B_o2")]
            if rr[0] is None:
                continue
            sc = [x["score"] if x else {} for x in rr]
            okn = sum(1 for x in sc if x.get("status") == "ok")
            a = sc[0]
            row = dict(set=s, rec=r, ok=okn, ate=[x.get("ate_vio_cm") if x.get("status") == "ok" else None for x in sc],
                       stat=[x.get("status") for x in sc], **{k: a.get(k) for k in (
                           "ate_map_cm", "path_m", "ate_pct", "drift_cm_per_m", "rot_med_deg", "init_s", "dt_final_ms",
                           "loop_err_cm", "loop_err_map_cm", "loop_pct", "n_loop", "tracked_frac", "status", "scale_sim3")})
            base.append(row)
            ates = "/".join(fmt(v) if v is not None else (st or "?") for v, st in zip(row["ate"], row["stat"]))
            md.append(f"| {s} | {r} | {okn} | {ates} | {fmt(a.get('ate_map_cm'))} | {fmt(a.get('path_m'))} | {fmt(a.get('ate_pct'))} | "
                      f"{fmt(a.get('drift_cm_per_m'))} | {fmt(a.get('rot_med_deg'))} | {fmt(a.get('init_s'))} | {fmt(a.get('dt_final_ms'))} | "
                      f"{fmt(a.get('loop_err_cm'))}/{fmt(a.get('loop_err_map_cm'))} | {a.get('n_loop', '—')} |")
    # ---- paired comparisons vs B (o0)
    md += ["", "## Paired vs B (offset 0). Δ = cond − B (cm); H rule: |median Δ| ≥ 0.05 and same sign ≥ 10/12", "",
           "| cond | set | pairs (both ok) | median Δ | mean Δ | better/worse | B-only fail / cond-only fail | verdict (H) | per-rec Δ |",
           "|---|---|---|---|---|---|---|---|---|"]
    pairs = {}
    for c in LB.CONDS:
        if c == "B":
            continue
        for s in "SLH":
            if s not in LB.CONDS[c][2]:
                continue
            d, bo, co, per = [], 0, 0, []
            for r in LB.SETS[s]:
                if r not in allc[c] or r not in B:
                    continue
                mb, mc = metric(B[r], s), metric(allc[c][r], s)
                if mb is None and mc is not None:
                    bo += 1
                if mc is None and mb is not None:
                    co += 1
                if mb is not None and mc is not None:
                    d.append(mc - mb); per.append(f"{LB.short(r)} {mc - mb:+.2f}")
                elif mb is not None or mc is not None:
                    per.append(f"{LB.short(r)} {'B fail' if mb is None else 'cond fail'}")
            if not d and not bo and not co:
                continue
            d = np.array(d)
            better, worse = int((d < 0).sum()), int((d > 0).sum())
            med = float(np.median(d)) if len(d) else None
            verdict = ""
            if s == "H" and len(d):
                verdict = "영향 있음" if abs(med) >= 0.05 and max(better, worse) >= 10 else "잡음 수준"
            elif len(d):
                verdict = "추정 " + ("좋아짐" if med < 0 else "나빠짐" if med > 0 else "같음")
            pairs[f"{c}/{s}"] = dict(n=len(d), med=med, mean=float(d.mean()) if len(d) else None, better=better, worse=worse,
                                    b_only_fail=bo, c_only_fail=co, verdict=verdict, per=per)
            md.append(f"| {c} | {s} | {len(d)} | {fmt(med, 3)} | {fmt(pairs[f'{c}/{s}']['mean'], 3)} | {better}/{worse} | {bo}/{co} | {verdict} | {'; '.join(per)} |")
    # ---- loop closure share (map - vio) on GT sets
    lc = [(r["rec"], r["score"].get("ate_map_cm"), r["score"].get("ate_vio_cm")) for r in B.values()
          if r["score"].get("status") == "ok" and r["score"].get("ate_map_cm") is not None]
    md += ["", "## Loop closure share in B (map ATE − VIO ATE, cm)", "",
           "| rec | VIO | map | Δ |", "|---|---|---|---|"] + [f"| {a} | {v:.2f} | {m:.2f} | {m - v:+.2f} |" for a, m, v in lc]
    # Sim3 scale (VIO -> GT) per condition, GT sets (reference; < 1 = VIO motion too large)
    md += ["", "## Sim3 scale median (S+H ok runs) and alt-GT ATE (B)", "", "| cond | n | scale median | min | max |", "|---|---|---|---|---|"]
    scl = {}
    for c, runs in allc.items():
        v = [r["score"].get("scale_sim3") for r in runs.values() if r["score"].get("status") == "ok" and r["score"].get("scale_sim3")]
        if v:
            scl[c] = dict(n=len(v), med=float(np.median(v)), min=min(v), max=max(v))
            md.append(f"| {c} | {len(v)} | {np.median(v):.4f} | {min(v):.4f} | {max(v):.4f} |")
    alt = [(r["rec"], r["score"].get("ate_vio_cm"), r["score"].get("ate_vio_cm_imu_gt"), r["score"].get("ate_vio_cm_imu_gt_rs"))
           for r in B.values() if r["score"].get("ate_vio_cm")]
    md += ["", "| rec | ATE spl | ATE imu_gt | ATE imu_gt_rs |", "|---|---|---|---|"] + [f"| {a} | {b} | {c} | {d} |" for a, b, c, d in alt]
    # Lp0 identity
    ident = {}
    for r, x in allc.get("Lp0", {}).items():
        if r in B:
            ident[r] = (x["score"].get("ate_vio_cm"), B[r]["score"].get("ate_vio_cm"))
    md += ["", f"Lp0 (loop off) VIO ATE vs B VIO ATE: {ident}"]
    summ = dict(made=time.strftime("%F %T"), runs_dir=str(RUNS), baseline=base, pairs=pairs, loop_share=lc, lp0_identity=ident, scale=scl, alt_gt=alt,
                n_runs={c: len(v) for c, v in allc.items()})
    json.dump(summ, open(HERE / "summary.json", "w"), indent=1, ensure_ascii=False)
    (HERE / "summary.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
