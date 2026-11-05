#!/usr/bin/env python3
"""featloop 11-05: collect rec/<id>/{ba_V/ba.json, corner.json, mesh_V/summary.json} -> table.md + table.json, and the
pre-registered rule (README 4) on the H recordings: relative change vs S of pooled |dZ| p90 and multi-view corner reproj p90."""
import json, sys, time
from pathlib import Path
import numpy as np

D = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
H = ["20261022_133820", "20261022_134550", "20261022_134204", "20261021_214550", "20261021_213341", "20261021_215348",
     "20261020_214138", "20261020_221052", "20261020_221925", "20261020_222747", "20261020_223815", "20261020_224323"]
VARS = ["S", "X", "SL", "XL", "S2"]


def J(p):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return None


rows = []
for rid in ["165506"] + H:
    R = D / "rec" / rid
    ce = J(R / "corner.json") or {}
    for v in VARS + ["VIO"]:
        b = J(R / f"ba_{v}" / "ba.json") if v != "VIO" else None
        m = J(R / f"mesh_{v}" / "summary.json") if v != "VIO" else None
        c = (ce.get("sources") or {}).get(v)
        if b is None and c is None and m is None:
            continue
        r = dict(rec=rid, var=v)
        if b:
            ps = b.get("pair_stats", {}); tm = b.get("timing", {})
            r.update(n_kf=b["n_kf"], pairs=b["pairs"], verified=b["verified_pairs"], loop_pairs=ps.get("loop_pairs"),
                     loop_verified=ps.get("loop_verified"), loop_linked_obs=ps.get("loop_linked_obs"),
                     inl_med=ps.get("base_inliers_median"), tracks=b["tracks"], points=b["points"], obs=b["obs"],
                     track_len=b["obs"] / max(1, b["points"]), kf_obs_min=b["kf_obs_min"], reproj_med_px=b.get("final_med_px"),
                     sim3_to_vio=b["sim3_scale"], t_feat=tm.get("features_s"), t_match=tm.get("matching_s"), t_ba=tm.get("ba_s"),
                     t_gpu=tm.get("gpu_stage_s"), t_total=b["runtime_s"], loop_vio_dist=(b.get("loop") or {}).get("vio_dist_m_median"),
                     loop_out_gate=(b.get("loop") or {}).get("frac_outside_cov_gate"))
        if c:
            r.update(c_p2_med=c["pair2v_rigid_rms_mm"]["median"], c_p2_p90=c["pair2v_rigid_rms_mm"]["p90"],
                     c_scale_med=c["pair2v_abs_scale_err_pct"]["median"], c_mv_med=c["mv_reproj_px"]["median"],
                     c_mv_p90=c["mv_reproj_px"]["p90"], c_mv_scale=c["mv_scale_err_pct"], c_pairs=ce.get("n_pairs"))
        if m:
            pd, pa, po = m.get("pooled_dZ_cm", {}), m.get("pooled_absdZ_cm", {}), m.get("poses", {})
            r.update(dz_mean=pd.get("mean"), dz_p10=pd.get("p10"), dz_p90=pd.get("p90"), adz_p90=pa.get("p90"),
                     mesh_z=m.get("mesh_board_z_med_cm"), mesh_rms=m.get("mesh_board_z_rms_cm"), n_eval=m.get("n_kf_board_eval"),
                     k_gt=(1 / po["sim3_scale_would_be"] - 1) * 100 if po.get("sim3_scale_would_be") else None,
                     ate_cm=po.get("se3_rmse_cm"), t_mesh=m.get("runtime_s"))
        rows.append(r)

f = lambda x, d=2: "—" if x is None else (f"{x:.{d}f}" if isinstance(x, float) else str(x))
cols = [("rec", 0), ("var", 0), ("n_kf", 0), ("pairs", 0), ("verified", 0), ("loop_pairs", 0), ("loop_verified", 0), ("loop_linked_obs", 0),
        ("points", 0), ("obs", 0), ("track_len", 2), ("reproj_med_px", 3), ("c_p2_med", 2), ("c_p2_p90", 2), ("c_mv_med", 2), ("c_mv_p90", 2),
        ("c_mv_scale", 2), ("dz_mean", 2), ("dz_p10", 2), ("dz_p90", 2), ("adz_p90", 2), ("mesh_rms", 2), ("n_eval", 0), ("k_gt", 2), ("ate_cm", 2),
        ("t_feat", 0), ("t_match", 0), ("t_ba", 0), ("t_total", 0), ("t_mesh", 0)]
L = ["| " + " | ".join(c for c, _ in cols) + " |", "|" + "---|" * len(cols)]
for r in rows:
    L.append("| " + " | ".join(f(r.get(c), d) if d else f(r.get(c)) for c, d in cols) + " |")

# pre-registered rule on H
by = {(r["rec"], r["var"]): r for r in rows}
rule = {}
for v in ["X", "SL", "XL"]:
    rel = {"adz_p90": [], "c_mv_p90": []}
    n_imp = 0; n = 0; recs = []
    for rid in H:
        a, b = by.get((rid, "S")), by.get((rid, v))
        if not a or not b or a.get("adz_p90") is None or b.get("adz_p90") is None or a.get("c_mv_p90") is None or b.get("c_mv_p90") is None:
            continue
        n += 1; recs.append(rid)
        for k in rel:
            rel[k].append((b[k] - a[k]) / a[k] * 100)
        n_imp += b["adz_p90"] < a["adz_p90"]
    if n == 0:
        continue
    md = {k: float(np.median(x)) for k, x in rel.items()}
    if md["adz_p90"] <= -10 and md["c_mv_p90"] <= -10 and n_imp >= 8:
        verdict = "좋아짐 (추정)"
    elif md["adz_p90"] >= 10 and md["c_mv_p90"] >= 10 and (n - n_imp) >= 8:
        verdict = "나빠짐"
    else:
        verdict = "잡음 수준"
    extra = {}
    for k in ["c_p2_med", "dz_mean", "mesh_rms", "k_gt", "points", "obs", "t_total", "t_gpu"]:
        d_ = [by[(rid, v)][k] - by[(rid, "S")][k] for rid in recs if by[(rid, v)].get(k) is not None and by[(rid, "S")].get(k) is not None]
        extra[k + "_diff_median"] = float(np.median(d_)) if d_ else None
    rule[v] = dict(n=n, rel_change_median_pct=md, rel_change_all_pct={k: [round(x, 1) for x in xs] for k, xs in rel.items()},
                   adz_p90_improved=int(n_imp), verdict=verdict, **extra)
L += ["", "## 판정 (README 4, H 녹화)", "| 변형 | n | |dZ| p90 상대 변화 중앙 % | 모서리 다시점 p90 상대 변화 중앙 % | |dZ| p90 준 녹화 | 판정 | 점 수 차 중앙 | 전체 시간 차 중앙 s |",
      "|---|---|---|---|---|---|---|---|"]
for v, q in rule.items():
    L.append(f"| {v} | {q['n']} | {q['rel_change_median_pct']['adz_p90']:+.1f} | {q['rel_change_median_pct']['c_mv_p90']:+.1f} | "
             f"{q['adz_p90_improved']}/{q['n']} | {q['verdict']} | {f(q['points_diff_median'], 0)} | {f(q['t_total_diff_median'], 0)} |")
(D / "table.md").write_text("\n".join(L) + "\n")
(D / "table.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), rows=rows, rule=rule), indent=1))
print("\n".join(L))
