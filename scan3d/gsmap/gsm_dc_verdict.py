#!/usr/bin/env python3
"""gsmap 11-27 dc: GO / NO-GO table from the probe dirs (probe.json of each variant + probe_dist.json). Writes dc/verdict.json.
GO rule (set before the probe): pooled hidden-pixel AbsRel median <= 2 x the FFS reference (fused FFS vs BA2 on FFS pixels, 0.73 %)
AND BA2-on-FFS-empty median AND > 5 % share both lower than DA3 affine by >= 25 % (relative), pooled over all 10 keyframes and in the aisle.
"""
import json, time
from pathlib import Path

DC = Path(__file__).resolve().parent.parent / "runs/gsmap_20261125_121801_v2/dc"
VAR = dict(probe="PriorDA v1.1 vitb (frozen DA-v2 vitb)", probe_g="PriorDA v1.1, geometric = DA3-LARGE depth",
           probe_gdg="PriorDA v1.1, geometric = DA3, double_global", probe_mdc="Marigold-DC (marigold-depth-v1-0, 50 steps, 768, bf16)")


def main():
    rows = {}; ref = None
    for n, desc in VAR.items():
        p = json.loads((DC / n / "probe.json").read_text()); ref = p["ffs_reference"]["med_abs_rel_pct"]; P = p["pooled"]
        r = dict(desc=desc)
        for g in ("all", "aisle", "other"):
            for t in ("hidden", "ba_empty"):
                for k, kk in (("dc", "pda"), ("da3", "da3_affine")):
                    e = P[g][t][kk]; r[f"{g}_{t}_{k}"] = dict(n=e["n"], med=round(e["absrel_med_pct"], 2), p90=round(e["p90_pct"], 1), gt5=round(e["gt5_pct"], 1))
        hid_ok = r["all_hidden_dc"]["med"] <= 2 * ref
        better = lambda g: (r[f"{g}_ba_empty_dc"]["med"] <= 0.75 * r[f"{g}_ba_empty_da3"]["med"]) and (r[f"{g}_ba_empty_dc"]["gt5"] <= 0.75 * r[f"{g}_ba_empty_da3"]["gt5"])
        r["gate"] = dict(hidden_le_2x_ffs=hid_ok, ba_empty_better_all=better("all"), ba_empty_better_aisle=better("aisle"))
        r["GO"] = bool(hid_ok and better("all") and better("aisle")); rows[n] = r
    dist = json.loads((DC / "probe_dist.json").read_text())["variants"]
    res = dict(made=time.strftime("%F %T"), n_recordings=1, n_keyframes=10, kf_idx="50,150,250,400,470 + aisle 314,318,322,326,330",
               inputs=[str(DC / n / "probe.json") for n in VAR] + [str(DC / "probe_dist.json")],
               go_rule=__doc__.split("GO rule")[1].strip(), ffs_reference_med_pct=ref, variants=rows,
               ba_empty_by_px_to_nearest_ffs=dist, verdict="GO" if any(r["GO"] for r in rows.values()) else "NO-GO",
               notes=["DA3 affine = last night's fill rule (per-kf robust scale+shift of DA3-LARGE to FFS, align_fuse.robust_fit), same pixels",
                      "BA2 targets = w5/sparse (3x3 splat of BA2 track points, F == 0 only); the same targets score both methods",
                      "hidden pixels = 20 % of valid FFS in random 24x24 blocks (seed 0); these lie inside FFS-covered surfaces, easier than true FFS holes",
                      "probe_mdc pictures were made before the label fix: their panels say 'PriorDA' but show Marigold-DC",
                      "n = 1 recording, 10 keyframes; one seed per variant"])
    (DC / "verdict.json").write_text(json.dumps(res, indent=1))
    for n, r in rows.items():
        print(n, r["GO"], "hidden", r["all_hidden_dc"]["med"], r["all_hidden_da3"]["med"], "| ba_empty all", r["all_ba_empty_dc"], r["all_ba_empty_da3"],
              "| aisle", r["aisle_ba_empty_dc"]["med"], r["aisle_ba_empty_da3"]["med"])
    print("VERDICT", res["verdict"])


if __name__ == "__main__":
    main()
