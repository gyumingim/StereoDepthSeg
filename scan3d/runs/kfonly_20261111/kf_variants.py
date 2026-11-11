#!/usr/bin/env python3
"""kfonly 11-11 follow-ups (README 7-8): B95 / B1080 vs the same laptop A; same 3 metrics and thresholds as README 4.
Writes table_variants.md / table_variants.json. Light (json + mesh.ply reading only)."""
import json, time
from pathlib import Path
import numpy as np
from kf_score import IDS, LAP, summ, HERE
from mesh_layers import layers


def main():
    L = [f"# kfonly 후속 변형 표 (kf_variants.py, {time.strftime('%Y-%m-%d %H:%M')}, 미검증)"]
    out = {}
    f = lambda v: "—" if v is None else f"{v:.2f}"
    for var in ("", "95", "1080"):
        name = "B" + var
        rows = []
        for i in IDS:
            pa = LAP / i / "kfonly/A/mesh_S/summary.json"
            pv = LAP / i / (f"kfonly{var}/B{var}/mesh_S/summary.json" if var else "kfonly/B/mesh_S/summary.json")
            if not (pa.exists() and pv.exists()):
                continue
            A, V = summ(pa), summ(pv)
            for X, q in ((A, pa), (V, pv)):
                try:
                    X.update(layers(str(q.parent)))
                except Exception:
                    X.update(ghost_frac=None, near_rms_cm=None)
            man = json.loads(next((LAP / i / (f"kfonly{var}/red{var}" if var else "kfonly/red")).glob("*/manifest.json")).read_text())
            mn = man["duration_s"] / 60
            rows.append(dict(rec=i, A=A, V=V, rel_adz=(V["adz_p90"] / A["adz_p90"] - 1) * 100,
                             rel_rms=(V["mesh_rms"] / A["mesh_rms"] - 1) * 100, dk=abs(V["k"] - A["k"]),
                             jpeg_MB_min=man["jpeg_bytes"] / 1e6 / mn, kB_kf=man["jpeg_bytes"] / 1e3 / man["n_keyframes"],
                             image_size=man.get("image_size"), quality=man.get("jpeg_quality")))
        if not rows:
            continue
        md = float(np.median([r["rel_adz"] for r in rows])); mr = float(np.median([r["rel_rms"] for r in rows]))
        mk = float(np.median([r["dk"] for r in rows])); mb = float(np.median([r["jpeg_MB_min"] for r in rows]))
        verdict = "통과" if (md <= 5 and mr <= 10 and mk <= 0.3) else "불합격"
        out[name] = dict(n=len(rows), med_rel_adz_pct=md, med_rel_rms_pct=mr, med_dk_pp=mk, verdict=verdict, jpeg_MB_min_median=mb, rows=rows)
        L += ["", f"## {name} ({rows[0]['image_size']} JPEG q{rows[0]['quality']}) vs A (같은 노트북)",
              f"| 녹화 | A |dZ| p90 cm | {name} | 변화 % | A 메시 RMS | {name} | 변화 % | A k % | {name} k % | |Δk| %p | 이중층 A/{name} | 주 층 RMS A/{name} | JPEG MB/분 | kB/kf |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            A, V = r["A"], r["V"]
            L.append(f"| {r['rec']} | {f(A['adz_p90'])} | {f(V['adz_p90'])} | {r['rel_adz']:+.1f} | {f(A['mesh_rms'])} | {f(V['mesh_rms'])} | {r['rel_rms']:+.1f} | "
                     f"{A['k']:+.2f} | {V['k']:+.2f} | {r['dk']:.3f} | {f(A['ghost_frac'])}/{f(V['ghost_frac'])} | {f(A['near_rms_cm'])}/{f(V['near_rms_cm'])} | "
                     f"{r['jpeg_MB_min']:.1f} | {r['kB_kf']:.0f} |")
        L.append(f"**{name} (n {len(rows)})**: |dZ| p90 변화 중앙 {md:+.1f} % (≤ +5), 메시 RMS 변화 중앙 {mr:+.1f} % (≤ +10), |Δk| 중앙 {mk:.3f} %p (≤ 0.3) "
                 f"→ **{verdict}**; |dZ| 나빠진 녹화 {sum(r['rel_adz'] > 0 for r in rows)}/{len(rows)}; JPEG {mb:.1f} MB/분")
    (HERE / "table_variants.md").write_text("\n".join(L) + "\n")
    (HERE / "table_variants.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), variants=out), indent=1))
    print("\n".join(L))


if __name__ == "__main__":
    main()
