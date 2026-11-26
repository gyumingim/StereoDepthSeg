#!/usr/bin/env python3
"""gsmap W5 2026-11-27: the W4 F ruler (gsm_f_ruler.py, code unchanged, imported) on ffsonly2 + W5, written to w5/fruler (never into w4).

  python scan3d/gsmap/gsm_w5_ruler.py all      -> w5/fruler/{f1,sanity,f2f3,baselines}.json + sheets, w5/fruler/w5_compare.json

Meshes (all ba2 frame): ffsonly2_e3c / ffsonly2_carve (W4 inputs, re-measured in this run), W2_final / W3A_fill (only so that the W4
compare sheet code runs), W5_e3c (votes: gsm_ffs_support --n 2, w5/aisle/w5_e3c), W5_carve (votes: w5/b/support/support_votes.npz, the
prune step, same as ffsonly2_carve). CAVEAT: F1 uses the same BA2 tracks that W5 was trained on (all kept points, odd and even) -> F1 is
NOT an independent ruler for W5. F2 / F3 / the FFS-unsupported area do not use BA tracks.
"""
import json, sys, time
from argparse import Namespace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gsm_f_ruler as fr  # noqa: E402

V2 = fr.V2; W5 = V2 / "w5"
fr.OUT = W5 / "fruler"
fr.MESHES = dict(fr.MESHES)
fr.MESHES["W5_e3c"] = dict(ply=W5 / "b/e3c.ply", votes=W5 / "aisle/w5_e3c/support_votes.npz", frame="ba2", glb=W5 / "b/tex_121801_w5.glb")
fr.MESHES["W5_carve"] = dict(ply=W5 / "b/carve.ply", votes=W5 / "b/support/support_votes.npz", frame="ba2", glb=None)
fr.F1_MESHES = ["ffsonly2_e3c", "ffsonly2_carve", "W5_e3c", "W5_carve"]


def compare():
    f1 = json.loads((fr.OUT / "f1.json").read_text()); f23 = json.loads((fr.OUT / "f2f3.json").read_text())
    rows = {}
    for nm in ("ffsonly2_e3c", "W5_e3c", "ffsonly2_carve", "W5_carve"):
        o = f23["meshes"][nm]; e = f1["meshes"][nm]["even"]
        rows[nm] = dict(area_m2=o["area_m2"], area_roi_m2=o["area_roi_m2"], unsupported_lt2_m2=o["unsupported_area_m2"], unsupported_lt2_roi_m2=o["unsupported_area_roi_m2"],
                        F1_pierced_m2=e["pierced_area_m2"], F1_pierced_roi_m2=e["pierced_area_roi_m2"], F1_blocked_frac=e["blocked_frac"],
                        F1_pt_mesh_cm_med=f1["meshes"][nm]["even_point_to_mesh_cm_p25_50_75"][1], F2_tube_m2=o["f2_tube_area_m2"],
                        F3_red_px_9views=o["red_px_total_9views"], F3_audit_red_px=o["views"]["audit"]["red_px"], F3_audit_red_pct=o["views"]["audit"]["red_pct_of_hit"],
                        F3_tex_red_px_audit=o["tex_red_px_audit"], F3_red_px_per_view={k: v["red_px"] for k, v in o["views"].items()},
                        F3_hit_px_per_view={k: v["hit_px"] for k, v in o["views"].items()}, sheet=o["sheet"])
    ch = {}
    for kind in ("e3c", "carve"):
        b, w = rows[f"ffsonly2_{kind}"], rows[f"W5_{kind}"]
        pc = lambda x, y: round((y - x) / x * 100, 1) if x else None  # noqa: E731
        ch[kind] = dict(unsupported_roi_change_pct=pc(b["unsupported_lt2_roi_m2"], w["unsupported_lt2_roi_m2"]),
                        unsupported_whole_change_pct=pc(b["unsupported_lt2_m2"], w["unsupported_lt2_m2"]),
                        F3_red_px_9views_change_pct=pc(b["F3_red_px_9views"], w["F3_red_px_9views"]),
                        F3_audit_red_px_change_pct=pc(b["F3_audit_red_px"], w["F3_audit_red_px"]),
                        roi_area_change_pct=pc(b["area_roi_m2"], w["area_roi_m2"]))
    res = dict(made=time.strftime("%F %T"), n_recordings=1, sources=[str(fr.OUT / "f1.json"), str(fr.OUT / "f2f3.json")], roi=fr.ROI,
               caveat="F1 not independent for W5 (trained on the same BA2 tracks); n=1, retrain noise unknown", rows=rows, change_W5_vs_ffsonly2=ch)
    (fr.OUT / "w5_compare.json").write_text(json.dumps(res, indent=1)); print(json.dumps(ch, indent=1))


def pic(dst):
    """ffsonly2 vs W5 picture from the F3 sheets: rows ffsonly2_e3c, W5_e3c, ffsonly2_carve, W5_carve; columns audit (aisle) class,
    audit texture, 2 keyframe views (sheet tiles 0, 1, 4, 7 = the W4 compare-sheet columns)"""
    import cv2
    w, h = fr.W // 2, fr.H // 2; rows = []
    for nm in ("ffsonly2_e3c", "W5_e3c", "ffsonly2_carve", "W5_carve"):
        sh = cv2.imread(str(fr.OUT / f"sheet_{nm}.jpg")); t = [sh[(i // 3) * h:(i // 3 + 1) * h, (i % 3) * w:(i % 3 + 1) * w] for i in range(9)]
        rows.append(np.hstack([t[c] for c in (0, 1, 4, 7)]))
    leg = np.full((40, rows[0].shape[1], 3), 20, np.uint8)
    cv2.putText(leg, "W5 (BA sparse depth retrain) vs ffsonly2 | grey = FFS-supported  red = no FFS support  blue = contradicted  dark = hole",
                (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(str(dst), np.vstack([leg] + rows), [cv2.IMWRITE_JPEG_QUALITY, 88]); print("wrote", dst)


def main():
    if sys.argv[1] == "pic":
        pic(sys.argv[2]); return
    fr.OUT.mkdir(parents=True, exist_ok=True)
    assert not (fr.OUT / "w5_compare.json").exists(), "never overwrite"
    a = Namespace(reproj=1.0, min_obs=3, min_tri=2.0, min_kf=0, tag="")
    fr.cmd_f1(a); fr.cmd_sanity(a); fr.cmd_f2f3(a); compare()


if __name__ == "__main__":
    main()
