#!/usr/bin/env python3
"""Copy the more-recordings meshes (PLY) next to the scanA viewer and append them to meshes.json (existing 165506 entries kept).
Also writes more/summary_table.json (numbers vs GT imu_gt_spl_cal1022, unverified)."""
import json, shutil, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent
VIEW = Path("/mnt/c/Users/a3162/scan3d_handoff/changes_0620/scanA")
REPO_VIEW = HERE.parent / "step3"
NOTE = {"20261022_134204": "모니터 보드 장면", "20261021_213341": "노트북 보드 장면 (10-21)",
        "20261020_222747": "노트북 보드 장면 (10-20)", "20261020_214138": "노트북 보드 장면 (10-20, 가장 김)"}
rows, add = [], []
for rec in sys.argv[1:]:
    short = rec.split("_")[1]
    for kind, lab in (("ba", "BA 자세 (실제 상황, 축척 VIO 그대로), 렌즈 2.22 dpt"), ("gt", "정답 자세 (비교용, 같은 키프레임), 렌즈 2.22 dpt")):
        d = HERE / "mesh" / f"{rec}_{kind}"
        if not (d / "summary.json").exists():
            print("skip (no summary)", d); continue
        s = json.loads((d / "summary.json").read_text())
        mb = (d / "mesh.ply").stat().st_size / 2 ** 20
        if mb > 30:
            sys.exit(f"{d}/mesh.ply {mb:.0f} MB > 30 MB: decimate first")
        f = f"h{short}_{kind}.ply"
        shutil.copy2(d / "mesh.ply", VIEW / f)
        add.append(dict(file=f, label=f"{short} · {lab} · {NOTE.get(rec, )}", rec=short, board_mm=s["board_mm"]))
        p = s["poses"]
        rows.append(dict(rec=rec, kind=kind, mesh_mb=round(mb, 2), vertices=s["mesh_vertices"], n_kf=s["n_kf"], n_kf_depth=s["n_kf_depth"],
                         se3_rmse_cm=p.get("se3_rmse_cm"), sim3_scale=p.get("sim3_scale_would_be"),
                         board_dZ_bias_cm=s.get("kf_board_dZ_bias_cm_median"), board_absdZ_p50_cm=s.get("kf_board_absdZ_p50_cm_median"),
                         pooled_dZ_p10_p90_cm=[s["pooled_dZ_cm"]["p10"], s["pooled_dZ_cm"]["p90"]] if "pooled_dZ_cm" in s else None,
                         board_Z_m=s.get("kf_board_Z_m_median"), mesh_board_z_med_cm=s.get("mesh_board_z_med_cm"),
                         mesh_board_z_rms_cm=s.get("mesh_board_z_rms_cm"), mesh_board_cover=s.get("mesh_board_cover_frac"),
                         runtime_s=s["runtime_s"]))
for mj in (VIEW / "meshes.json", REPO_VIEW / "meshes.json"):
    cur = [m for m in json.loads(mj.read_text(encoding="utf-8")) if not m["file"].startswith("h")]
    mj.write_text(json.dumps(cur + add, indent=1, ensure_ascii=False), encoding="utf-8")
shutil.copy2(REPO_VIEW / "viewer.html", VIEW / "viewer.html")
(HERE / "summary_table.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), gt="imu_gt_spl_cal1022",
    note="미검증 (독립 검사 전). sim3_scale = VIO/BA 위치를 GT 에 맞출 축척 (1 보다 작으면 VIO 가 김)", rows=rows), indent=1, ensure_ascii=False))
for r in rows:
    print(r)
