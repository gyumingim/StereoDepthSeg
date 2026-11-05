#!/usr/bin/env python3
"""gsplat 2026-11-05: table (markdown + json) of TSDF vs 2DGS scores + the pre-registered rule (README 3).
  python3 scan3d/gsplat/gs_table.py scan3d/runs/gsplat_20261105"""
import json
import sys
from pathlib import Path

O = Path(sys.argv[1])


def ld(p):
    return json.loads(p.read_text()) if p.exists() else None


def gpu_smi(name):
    p = O / f"{name}.gpu.txt"
    if not p.exists():
        return None
    v = [int(x) for x in p.read_text().split() if x.strip().isdigit()]
    return round(max(v) / 1024, 1) if v else None


rows, tab = [], {}
for r in ("ba", "ba_fs"):
    for m in ("tsdf", "gsA", "gsB"):
        name = f"{m}_{r}"
        sc = ld(O / "score" / f"{name}.json")
        sm = ld(O / ("tsdf" if m == "tsdf" else "gs") / (r if m == "tsdf" else name) / "summary.json")
        if not sc or not sm:
            rows.append(f"| {name} | (없음) |")
            continue
        if m == "tsdf":   # runtime/GPU of the scanA run (our re-run also wrote the dump PNG/npz, slower); numbers are identical
            sa = ld(Path.home() / "scanner/StereoDepthSeg-scanA/scan3d/runs/scanA_20261105/step3" / r / "summary.json")
            if sa:
                sm = dict(sm, runtime_s=sa.get("runtime_s"), runtime_rerun_with_dump_s=sm.get("runtime_s"))
        mr, dm = sc["mesh_raycast"], sc.get("depth_map", {})
        e = dict(mesh_dZ=mr["pooled_dZ_cm"], mesh_abs=mr["pooled_absdZ_cm"], mesh_valid=mr["board_valid_px_total"] / max(1, mr["board_px_total"]),
                 dm_dZ=dm.get("pooled_dZ_cm"), dm_abs=dm.get("pooled_absdZ_cm"),
                 dm_valid=dm.get("board_valid_px_total", 0) / max(1, dm.get("board_px_total", 1)),
                 z_med=sc.get("mesh_board_z_med_cm"), z_rms=sc.get("mesh_board_z_rms_cm"), cover=sc.get("mesh_board_cover_frac"),
                 runtime=sm.get("runtime_s"), gpu_torch=sm.get("gpu_max_alloc_gb"),
                 gpu_smi=gpu_smi(f"tsdf_{r}" if m == "tsdf" else name), n_gauss=sm.get("n_gauss"), psnr=sm.get("psnr_mean"),
                 n_kf=mr["n_kf_eval"], mesh_faces=sc.get("mesh_faces"))
        tab[name] = e
        f = lambda d, k: "—" if not d else f"{d[k]:+.2f}"  # noqa: E731
        g = lambda d, k: "—" if not d else f"{d[k]:.2f}"  # noqa: E731
        rows.append(f"| {name} | {f(e['mesh_dZ'], 'mean')} / {f(e['mesh_dZ'], 'p10')} / {f(e['mesh_dZ'], 'p90')} | "
                    f"{g(e['mesh_abs'], 'mean')} / {g(e['mesh_abs'], 'p90')} | {e['mesh_valid']:.2f} | "
                    f"{f(e['dm_dZ'], 'mean')} / {f(e['dm_dZ'], 'p10')} / {f(e['dm_dZ'], 'p90')} | {g(e['dm_abs'], 'p90')} | {e['dm_valid']:.2f} | "
                    f"{e['z_med']:+.2f} / {e['z_rms']:.2f} | {e['cover']:.2f} | {e['runtime']:.0f} | {e['gpu_torch']} / {e['gpu_smi']} | "
                    f"{e['n_gauss'] or '—'} | {'—' if e['psnr'] is None else round(e['psnr'], 1)} |")
head = ("| run | (b) 메시 깊이 dZ 평균 / p10 / p90 cm | (b) 메시 |dZ| 평균 / p90 cm | (b) 보드 화소 덮임 | (a) 깊이 지도 dZ 평균 / p10 / p90 cm | "
        "(a) |dZ| p90 | (a) 유효 화소 | (c) 메시 z 중앙 / RMS cm | (c) 덮임 1 cm 칸 | 시간 s | GPU torch / smi GB | 가우시안 수 | PSNR |\n"
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|")
verdict = []
for r in ("ba_fs", "ba"):
    t = tab.get(f"tsdf_{r}")
    for m in ("gsA", "gsB"):
        g = tab.get(f"{m}_{r}")
        if not t or not g:
            continue
        dp = (g["mesh_abs"]["p90"] - t["mesh_abs"]["p90"]) / t["mesh_abs"]["p90"]
        dr = (g["z_rms"] - t["z_rms"]) / t["z_rms"]
        dc = g["cover"] - t["cover"]
        if (dp <= -0.2 or dr <= -0.2) and dp <= 0.2 and dr <= 0.2 and dc >= -0.05:
            v = "좋아짐 (녹화 1개, 추정)"
        elif dp >= 0.2 or dr >= 0.2 or dc < -0.05:
            v = "나빠짐"
        else:
            v = "잡음 수준"
        slow = " (시간 TSDF 의 10 배 넘음)" if g["runtime"] > 10 * t["runtime"] else ""
        verdict.append(dict(run=f"{m}_{r}", d_absp90=round(dp * 100, 1), d_rms=round(dr * 100, 1), d_cover=round(dc, 3), verdict=v + slow))
md = head + "\n" + "\n".join(rows) + "\n\n판정 (README 3절, TSDF 대비 %):\n" + "\n".join(
    f"- {v['run']}: 메시 |dZ| p90 {v['d_absp90']:+.1f} %, 메시 RMS {v['d_rms']:+.1f} %, 덮임 {v['d_cover']:+.3f} → **{v['verdict']}**" for v in verdict)
(O / "table.md").write_text(md + "\n")
(O / "table.json").write_text(json.dumps(dict(rows=tab, verdict=verdict), indent=1))
print(md)
