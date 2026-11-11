#!/usr/bin/env python3
"""ghost 11-11: table + pre-registered verdict (README 4) from runs/laptop/20261022_134550/ghost/<fuse tag>/variants.json files.
usage: ghost_table.py [FUSE_DIR ...]  -> table.md / table.json next to this file."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
G = HERE.parent / "laptop/20261022_134550/ghost"


def main():
    dirs = sys.argv[1:] or sorted(str(p.parent) for p in G.glob("*/variants.json"))
    res = {}
    for d in dirs:
        for k, v in json.loads((Path(d) / "variants.json").read_text()).items():
            if k == "dense_twide":   # name parse bug: twide not applied (= dense), left out
                continue
            res[k] = v
    b = res["base"]
    L = [f"# ghost 표 (ghost_table.py, {time.strftime('%Y-%m-%d %H:%M')}, 134550 1 회, 미검증)", "",
         "| 변형 | 이중층 비율 | |dZ| p90 cm | 덮임 | 주 층 RMS cm | 메시 RMS cm | 메시 |z| p90 cm | 보드 꼭짓점 | 버린 화소 (평균) | 융합 s | 판정 |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    rows = []
    for k, v in res.items():
        g_ok = v["ghost_frac"] <= b["ghost_frac"] / 3
        d_ok = v["pooled_absdZ_cm"]["p90"] <= b["pooled_absdZ_cm"]["p90"] * 1.05
        c_ok = v["mesh_board_cover_frac"] >= 0.9 * b["mesh_board_cover_frac"]
        ver = "기준" if k == "base" else ("통과" if g_ok and d_ok and c_ok else "불합격 (" + ",".join(
            n for n, o in (("이중층", g_ok), ("깊이", d_ok), ("덮임", c_ok)) if not o) + ")")
        rows.append(dict(variant=k, verdict=ver, **{x: v.get(x) for x in ("ghost_frac", "mesh_board_cover_frac", "near_rms_cm",
                                                                         "mesh_board_z_rms_cm", "mesh_board_absz_p90_cm", "fuse_s")},
                         adz_p90=v["pooled_absdZ_cm"]["p90"]))
        L.append(f"| {k} | {v['ghost_frac']:.3f} | {v['pooled_absdZ_cm']['p90']:.3f} | {v['mesh_board_cover_frac']:.3f} | {v['near_rms_cm']:.3f} | "
                 f"{v['mesh_board_z_rms_cm']:.3f} | {v['mesh_board_absz_p90_cm']:.3f} | {v['mesh_board_vertices']} | "
                 f"{v.get('cons_drop_frac_mean', 0):.3f} | {v['fuse_s']:.0f} | {ver} |")
    (HERE / "table.md").write_text("\n".join(L) + "\n")
    (HERE / "table.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), dirs=dirs, rows=rows), indent=1))
    print("\n".join(L))


if __name__ == "__main__":
    main()
