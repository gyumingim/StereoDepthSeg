#!/usr/bin/env python3
"""hybrid 11-07: results table (README 4-5) -> table.md / table.json.
B0 / V2 = mvggt runs (read only), B0r / B0c / H1-3 = this run. Verdict (README 5) on oracle scale, all keyframes, vs B0c."""
import json
from pathlib import Path

import numpy as np

H = Path(__file__).resolve().parent
M = Path.home() / "scanner/StereoDepthSeg-mvggt/scan3d/runs/mvggt_20261106/rec"
RECS = ("165506", "20261022_134204", "20261020_222747")


def load(d):
    s, k = d / "summary.json", d / "keyframes.json"
    if not s.exists():
        return None
    return json.loads(s.read_text()), json.loads(k.read_text())


def ident(r, compressed):
    return r.get("frame", r["i"]) if compressed else r["i"]


def stats(S, K):
    ev = [r for r in K if "board_absdZ_p50_cm" in r]
    vis = [r for r in K if r.get("board_px", 0) > 200]
    nopx = sum(1 for r in K if "board_px" not in r)
    fail = None if nopx else len([r for r in vis if "board_dZ_med_cm" not in r or abs(r["board_dZ_med_cm"]) > 1]) / max(1, len(vis))
    return dict(n_kf=len(K), n_eval=len(ev), n_nodepth=sum(1 for r in K if not r.get("pairs")),
                n_vggt=sum(1 for r in K if r.get("src") == "vggt"),
                adz_med=S["pooled_absdZ_cm"]["median"], adz_p90=S["pooled_absdZ_cm"]["p90"],
                kf_med=S.get("kf_board_absdZ_p50_cm_median"), kf_p90=S.get("kf_board_absdZ_p90_cm_median"),
                rms=S.get("mesh_board_z_rms_cm"), cover=S.get("mesh_board_cover_frac"),
                bias1=S.get("kf_frac_abs_bias_gt_1cm"), fail=fail, n_vis=len(vis), made=S.get("made"))


def main():
    rows, md = [], []
    md.append("| rec | scale | set | var | kf (eval / no depth / VGGT) | |dZ| med / p90 | kf |dZ| med / p90 (median) | mesh RMS | cover | kf |bias|>1 (eval) | 실패 kf (visible) | 판정 |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    filled_md = ["| rec | scale | n 메운 kf (평가됨) | var | kf |dZ| 중앙 (median) | kf |bias| 중앙 | |bias|>1 비율 |", "|---|---|---|---|---|---|---|"]
    verdict = {}
    for rid in RECS:
        D = H / "rec" / rid
        for fs in ("", "_fs"):
            sc = "oracle" if fs else "native"
            for st, names in (("all", [("B0", M / rid / f"mesh_B0{fs}"), ("V2", M / rid / f"mesh_V2{fs}")] +
                                [(v, D / f"mesh_{v}{fs}") for v in ("B0r", "B0c", "H1", "H2", "H3")]),
                              ("overlap", [("B0", M / rid / f"mesh_B0_ovV{fs}"), ("V2", M / rid / f"mesh_V2_ovV{fs}")] +
                                [(v, D / f"mesh_{v}_ov{fs}") for v in ("B0c", "H1", "H3")])):
                L = {v: load(p) for v, p in names}
                L = {v: x for v, x in L.items() if x}
                base = stats(*L["B0c"]) if "B0c" in L else None
                for v, (S, K) in L.items():
                    s = stats(S, K)
                    vd = ""
                    if st == "all" and fs and base and v in ("H1", "H2", "H3"):
                        c1 = (s["cover"] >= base["cover"] + 0.01) or (s["fail"] is not None and s["fail"] <= base["fail"] * 0.8)
                        c2 = s["rms"] <= base["rms"] * 1.2
                        c3 = s["kf_med"] <= base["kf_med"] * 1.1
                        vd = "통과" if (c1 and c2 and c3) else "실패 (" + ",".join(n for n, c in (("1", c1), ("2", c2), ("3", c3)) if not c) + ")"
                        verdict.setdefault(v, {})[rid] = dict(pass_=bool(c1 and c2 and c3), c1=bool(c1), c2=bool(c2), c3=bool(c3))
                    rows.append(dict(rec=rid, scale=sc, set=st, var=v, verdict=vd, **s))
                    f = lambda x, n=2: "—" if x is None else f"{x:.{n}f}"
                    md.append(f"| {rid} | {sc} | {st} | {v} | {s['n_eval']} / {s['n_nodepth']} / {s['n_vggt']} | {f(s['adz_med'])} / {f(s['adz_p90'])} | "
                              f"{f(s['kf_med'])} / {f(s['kf_p90'])} | {f(s['rms'])} | {f(s['cover'])} | {f(s['bias1'])} | {f(s['fail'])} | {vd} |")
                # filled keyframes only: H rows with src == vggt vs the same keyframes in V2
                if st == "all" and "V2" in L:
                    comp = bool(L["V2"][0]["args"].get("compress"))
                    v2 = {ident(r, comp): r for r in L["V2"][1] if "board_absdZ_p50_cm" in r}
                    for v in ("H1", "H2", "H3"):
                        if v not in L:
                            continue
                        hk = {ident(r, comp): r for r in L[v][1] if r.get("src") == "vggt" and "board_absdZ_p50_cm" in r}
                        common = sorted(set(hk) & set(v2))
                        for nm, src in ((v, hk), ("V2", v2)):
                            if not common:
                                continue
                            a50 = np.array([src[c]["board_absdZ_p50_cm"] for c in common])
                            b = np.array([src[c]["board_dZ_med_cm"] for c in common])
                            filled_md.append(f"| {rid} | {sc} | {len(common)} | {nm} (vs {v}) | {np.median(a50):.2f} | {np.median(np.abs(b)):.2f} | {np.mean(np.abs(b) > 1):.2f} |")
    md += ["", "판정 (README 5, 오라클 · 전체 kf, B0c 기준; 실패 (n) = 못 맞춘 조건 번호):", ""]
    for v, d in verdict.items():
        k = sum(x["pass_"] for x in d.values())
        md.append(f"- {v}: 통과 {k}/{len(d)} → {'추천 후보' if k >= 2 else '채택 안 함'}")
    md += ["", "메운 키프레임만 (H 에서 src=vggt 이고 평가된 kf, 같은 kf 의 V2 와 비교, 판정 안 씀):", ""] + filled_md
    (H / "table.md").write_text("\n".join(md) + "\n")
    (H / "table.json").write_text(json.dumps(dict(rows=rows, verdict=verdict), indent=1, ensure_ascii=False))
    print("\n".join(md))


if __name__ == "__main__":
    main()
