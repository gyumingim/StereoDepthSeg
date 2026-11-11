"""kfsel 11-11: table of the pre-registered comparison (runs/laptop/20261022_133820/kfsel_<TAG>/scan) -> table.json / table.md."""
import json, sys, time
from pathlib import Path

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H.parent / "kfonly_20261111"))
import mesh_layers  # noqa: E402

L = H.parent / "laptop" / "20261022_133820"
tags = [t for t in ("I", "M3", "M2", "M4", "M3g") if (L / f"kfsel_{t}" / "scan" / "scan_summary.json").exists()]
rows = {}
for t in tags:
    d = L / f"kfsel_{t}" / "scan"
    s = json.loads((d / "scan_summary.json").read_text())
    m = json.loads((d / "mesh" / "summary.json").read_text())
    se = s["sent"]; ks = se.get("kf_select") or {}
    lay = mesh_layers.layers(str(d / "mesh"))
    rows[t] = dict(rule=ks.get("rule"), dist_m=ks.get("dist_m"), rot_deg=ks.get("rot_deg"), blur=ks.get("blur"),
                   n_kf=se["n_keyframes"], kf_per_min=se["kf_per_min"], MB_per_min=se["jpeg_MB_per_min"],
                   kfba_s=s["step_s"].get("kfba"), reduce_s=s["step_s"].get("reduce"), mesh_s=s["step_s"].get("mesh"), total_s=s["total_s"],
                   n_kf_mesh=m["n_kf"], n_kf_depth=m["n_kf_depth"], ffs_cover=m["n_kf_depth"] / m["n_kf"],
                   p90_pooled=m["pooled_absdZ_cm"]["p90"], p90_kf_median=m["kf_board_absdZ_p90_cm_median"],
                   n_kf_board_eval=m.get("n_kf_board_eval"),
                   k_pct=(1 / m["poses"]["sim3_scale_would_be"] - 1) * 100, board_cover=m.get("mesh_board_cover_frac"),
                   double_layer=lay["ghost_frac"], near_rms_cm=lay["near_rms_cm"], board_v=lay["board_v"],
                   gyro_kf_median=ks.get("gyro_kf_median"), gyro_window_mean_median=ks.get("gyro_window_mean_median"),
                   moved_m_median=ks.get("moved_m_median"), rot_deg_median=ks.get("rot_deg_median"),
                   pos_in_window_hist=ks.get("pos_in_window_hist"), commit=s["commit"])
I = rows.get("I")
for t, r in rows.items():
    if I is None or t == "I":
        continue
    c = dict(p90_pooled=r["p90_pooled"] <= I["p90_pooled"] * 1.05, p90_kf=r["p90_kf_median"] <= I["p90_kf_median"] * 1.05,
             k=abs(r["k_pct"] - I["k_pct"]) <= 0.3, cover=r["board_cover"] >= I["board_cover"], n_kf=r["n_kf"] <= I["n_kf"])
    r["pass_parts"] = c; r["pass"] = all(c.values())
    r["d_p90_pooled_pct"] = (r["p90_pooled"] / I["p90_pooled"] - 1) * 100
    r["d_p90_kf_pct"] = (r["p90_kf_median"] / I["p90_kf_median"] - 1) * 100
(H / "table.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rec="20261022_133820", rows=rows), indent=1))
hd = "| tag | kf | kf/min | MB/min | kfba s | total s | FFS 짝 | p90 모음 cm | kf p90 중앙 cm | k % | 덮임 | 이중층 | 판정 |"
out = [hd, "|" + "---|" * 13]
for t, r in rows.items():
    pv = "" if "pass" not in r else ("합격" if r["pass"] else "불합격 (" + ", ".join(k for k, v in r["pass_parts"].items() if not v) + ")")
    out.append(f"| {t} | {r['n_kf']} | {r['kf_per_min']:.0f} | {r['MB_per_min']:.0f} | {r['kfba_s']} | {r['total_s']} | "
               f"{r['n_kf_depth']}/{r['n_kf_mesh']} ({r['ffs_cover']:.2f}) | {r['p90_pooled']:.3f} | {r['p90_kf_median']:.3f} | "
               f"{r['k_pct']:.3f} | {r['board_cover']:.2f} | {r['double_layer']:.3f} | {pv} |")
(H / "table.md").write_text("\n".join(out) + "\n")
print("\n".join(out))
print(json.dumps({t: {k: r[k] for k in ("moved_m_median", "rot_deg_median", "gyro_kf_median", "gyro_window_mean_median", "pos_in_window_hist", "reduce_s", "mesh_s", "n_kf_board_eval")} for t, r in rows.items()}, indent=0))
