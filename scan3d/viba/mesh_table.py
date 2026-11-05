#!/usr/bin/env python3
"""viba M3: board depth table from scan_offline summaries RUN/mesh/<rec>_<ba|viba>/summary.json + README 4 rule."""
import json, sys, time
from pathlib import Path
import numpy as np
D = Path(sys.argv[1]) / "mesh"
recs = sorted({p.parent.name.rsplit("_", 1)[0] for p in D.glob("*/summary.json")})
L = ["| 녹화 | 자세 | 전체 화소 dZ 평균 / 중앙 / p10 / p90 cm | |dZ| p90 | 메시 z 중앙 / RMS cm | kf 평가 |", "|---|---|---|---|---|---|"]
pair = []
f = lambda x: "–" if x is None else f"{x:+.2f}"
for r in recs:
    row = {}
    for n in ("ba", "viba"):
        p = D / f"{r}_{n}" / "summary.json"
        if not p.exists():
            continue
        s = json.loads(p.read_text()); pd, pa = s.get("pooled_dZ_cm", {}), s.get("pooled_absdZ_cm", {})
        row[n] = pd
        L.append(f"| {r} | {n} | {f(pd.get('mean'))} / {f(pd.get('median'))} / {f(pd.get('p10'))} / {f(pd.get('p90'))} | "
                 f"{f(pa.get('p90'))} | {f(s.get('mesh_board_z_med_cm'))} / {f(s.get('mesh_board_z_rms_cm'))} | {s.get('n_kf_board_eval')} |")
    if "ba" in row and "viba" in row and row["ba"].get("mean") is not None and row["viba"].get("mean") is not None:
        pair.append((r, abs(row["ba"]["mean"]), abs(row["viba"]["mean"]), row["ba"]["p90"] - row["ba"]["p10"], row["viba"]["p90"] - row["viba"]["p10"]))
if pair:
    a = np.array([p[1:] for p in pair])
    mb, mv, wb, wv = np.median(a, 0)
    L += ["", f"짝 {len(pair)} 녹화: |dZ 평균| 중앙 ba {mb:.2f} → viba {mv:.2f} cm ({(mv / mb - 1) * 100:+.0f} %), "
          f"|평균| 줄어든 녹화 {int((a[:, 1] < a[:, 0]).sum())}/{len(pair)}; p90−p10 폭 중앙 ba {wb:.2f} → viba {wv:.2f} cm ({(wv / wb - 1) * 100:+.0f} %). "
          f"규칙 (README 4): {'좋아짐' if mv <= 0.8 * mb and wv <= 1.2 * wb else '미달'}. 만든 시각 {time.strftime('%F %T')}"]
print("\n".join(L)); (D.parent / "mesh_table.md").write_text("\n".join(L) + "\n")
