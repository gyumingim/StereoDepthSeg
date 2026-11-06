#!/usr/bin/env python3
"""mvggt: results table (README 4) from rec/<id>/mesh_*/summary.json + rec/<id>/al/align_*.json -> table.md / table.json."""
import json
from pathlib import Path

H = Path(__file__).resolve().parent
rows, md = [], []
md.append("| rec | var | scale | n_eval kf | no depth kf | dZ mean cm | |dZ| p90 cm | kf |dZ| med (median) | mesh z / RMS cm | cover | vs B0 (|dZ| p90 / RMS) | verdict (fs only) |")
md.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
for rd in sorted((H / "rec").iterdir()):
    if not rd.is_dir():
        continue
    S = {}
    for m in sorted(rd.glob("mesh_*/summary.json")):
        S[m.parent.name[5:]] = json.loads(m.read_text())
    for ov, fs in [(o, f) for o in ("", "_ovV", "_ovM") for f in ("", "_fs")]:
        b = S.get("B0" + ov + fs)
        for v in {"": ("B0", "V1", "V2", "M1", "M2"), "_ovV": ("B0", "V1", "V2"), "_ovM": ("B0", "M1", "M2")}[ov]:
            s = S.get(v + ov + fs)
            if not s or "pooled_absdZ_cm" not in s:
                continue
            p90, rms, cov = s["pooled_absdZ_cm"]["p90"], s.get("mesh_board_z_rms_cm"), s.get("mesh_board_cover_frac")
            r = dict(rec=rd.name, var=v + ov, scale="oracle" if fs else "native", n_eval=s.get("n_kf_board_eval"),
                     n_nodepth=s.get("n_kf_no_pair"), dz_mean=s["pooled_dZ_cm"]["mean"], adz_p90=p90,
                     kf_adz_med=s.get("kf_board_absdZ_p50_cm_median"), mesh_z=s.get("mesh_board_z_med_cm"), mesh_rms=rms, cover=cov,
                     sim3=s["poses"].get("sim3_scale_would_be"), made=s.get("made"))
            rel = verdict = ""
            if b and v != "B0" and "pooled_absdZ_cm" in b:
                d1 = p90 / b["pooled_absdZ_cm"]["p90"] - 1
                d2 = (rms / b["mesh_board_z_rms_cm"] - 1) if (rms is not None and b.get("mesh_board_z_rms_cm")) else None
                r.update(rel_p90=d1, rel_rms=d2)
                rel = f"{d1 * 100:+.0f} % / " + (f"{d2 * 100:+.0f} %" if d2 is not None else "—")
                if fs and (ov or not any("_ov" in k for k in S)):
                    better = d1 <= -0.2 or (d2 is not None and d2 <= -0.2)
                    covok = (cov or 0) >= (b.get("mesh_board_cover_frac") or 0) - 0.05
                    verdict = "좋아짐" if (better and covok) else ("덮임 손실" if better else "아님")
                    r["verdict"] = verdict
            rows.append(r)
            f = lambda x, n=2: "—" if x is None else f"{x:.{n}f}"
            md.append(f"| {rd.name} | {v}{ov} | {r['scale']} | {r['n_eval']} | {r['n_nodepth']} | {f(r['dz_mean'])} | {f(p90)} | {f(r['kf_adz_med'])} | "
                      f"{f(r['mesh_z'])} / {f(rms)} | {f(cov)} | {rel} | {verdict} |")
md.append("")
md.append("| rec | pose source | n | ATE RMSE cm (Sim3 to GT) | k % | rot med / p90 ° | Sim3-to-BA centre rms cm | model focal / ours (960 px) |")
md.append("|---|---|---|---|---|---|---|---|")
for rd in sorted((H / "rec").iterdir()):
    for kind in ("vggt", "mast3r"):
        p = rd / "al" / f"align_{kind}.json"
        if not p.exists():
            continue
        a = json.loads(p.read_text())
        foc = a.get("focal_pred_x_960") or a.get("focal_final_960")
        for src, e in a["pose_eval"].items():
            if kind == "mast3r" and src in ("BA", "VIO_kf"):
                continue
            extra = f"{a['sim3_to_ba']['centre_rms_cm']:.2f}" if src in ("V1", "M1") else ""
            ff = f"{foc:.0f} / {a['focal_ours']:.0f}" if src in ("V1", "M1") else ""
            md.append(f"| {rd.name} | {src} | {e['n']} | {e['ate_rmse_cm']:.2f} | {e['k_pct']:+.2f} | {e['rot_med_deg']:.2f} / {e['rot_p90_deg']:.2f} | {extra} | {ff} |")
            rows.append(dict(rec=rd.name, pose=src, **e))
        if kind == "mast3r" and "mast3r_raw_metric_k_pct" in a:
            md.append(f"| {rd.name} | (MASt3R 자체 metric 축척, 맞추기 전) | | | {a['mast3r_raw_metric_k_pct']:+.1f} | | | |")
ovp = H / "overlap.md"
if ovp.exists():
    md += ["", "겹치는 키프레임만 (모든 변형에서 평가된 kf), 키프레임별 통계:", ""] + ovp.read_text().splitlines()
(H / "table.md").write_text("\n".join(md) + "\n")
(H / "table.json").write_text(json.dumps(rows, indent=1))
print("\n".join(md))
