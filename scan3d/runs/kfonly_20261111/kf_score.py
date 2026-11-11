#!/usr/bin/env python3
"""kfonly 11-11: score A vs B (README 3-4) from runs/laptop/<id>/kfonly/{A,B}/mesh_S/summary.json + red/*/manifest.json.
Writes table.md / table.json next to this file. Light (json reading only)."""
import glob, json, time
from pathlib import Path
import numpy as np
from mesh_layers import layers

HERE = Path(__file__).resolve().parent
LAP = HERE.parent / "laptop"
FL = Path.home() / "scanner/StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/table.json"
IDS = "20261022_133820 20261022_134550 20261022_134204 20261021_214550 20261021_213341 20261021_215348 20261020_221052 20261020_221925 20261020_222747 20261020_223815 20261020_224323".split()


def summ(p):
    m = json.loads(p.read_text())
    po = m["poses"]
    return dict(adz_p90=m["pooled_absdZ_cm"]["p90"], mesh_rms=m.get("mesh_board_z_rms_cm"),
                k=(1 / po["sim3_scale_would_be"] - 1) * 100, n_eval=m.get("n_kf_board_eval"), n_kf=m.get("n_kf"),
                n_depth=m.get("n_kf_depth"), cover=m.get("mesh_board_cover_frac"), t_mesh=m.get("runtime_s"))


def ba(p):
    b = json.loads(p.read_text())
    out = {}
    for k in ("n_points", "points", "n_obs", "obs"):
        if k in b:
            out[k] = b[k]
    return out


def main():
    ref = {}
    try:
        tj = json.loads(FL.read_text())
        for r in tj.get("rows", tj if isinstance(tj, list) else []):
            if isinstance(r, dict) and r.get("var") == "S":
                ref[r["rec"]] = r
    except Exception as e:
        print("featloop ref not read:", e)
    rows, rel_dz, rel_rms, dk = [], [], [], []
    sizes = []
    for i in IDS:
        d = LAP / i / "kfonly"
        pa, pb = d / "A/mesh_S/summary.json", d / "B/mesh_S/summary.json"
        if not (pa.exists() and pb.exists()):
            rows.append(dict(rec=i, missing=True)); continue
        A, B = summ(pa), summ(pb)
        for X, q in ((A, pa), (B, pb)):
            try:
                X.update(layers(str(q.parent)))
            except Exception as e:
                X.update(board_v=None, ghost_frac=None, near_rms_cm=None)
        man = json.loads(next((d / "red").glob("*/manifest.json")).read_text())
        r = dict(rec=i, A=A, B=B, rel_adz_pct=(B["adz_p90"] / A["adz_p90"] - 1) * 100,
                 rel_rms_pct=(B["mesh_rms"] / A["mesh_rms"] - 1) * 100, dk_pp=abs(B["k"] - A["k"]),
                 ref_S=dict(adz_p90=ref.get(i, {}).get("adz_p90"), mesh_rms=ref.get(i, {}).get("mesh_rms"), k=ref.get(i, {}).get("k_gt")))
        rel_dz.append(r["rel_adz_pct"]); rel_rms.append(r["rel_rms_pct"]); dk.append(r["dk_pp"])
        mn = man["duration_s"] / 60
        rec_dir = Path(man["rec"])
        a_bytes = man["mp4_bytes"] + man["frames_csv_bytes"] + man["imu_csv_bytes"]
        # keyframe poses only: 16 float64 + t = 136 B (binary); full live pose stream: file as is (gz jsonl)
        b_kf_pose = man["n_keyframes"] * 136
        b_min = man["jpeg_bytes"] + b_kf_pose + man["imu_csv_zlib_bytes"] + man["frames_csv_bytes"] * 0  # frame csv not needed
        b_full = man["jpeg_bytes"] + man["live_pose_file_bytes"] + man["imu_csv_bytes"] + man["frames_csv_bytes"]
        ss = man["size_sample"]; nS = max(ss["n"], 1)
        sz = dict(rec=i, min=mn, n_kf=man["n_keyframes"], kf_per_min=man["n_keyframes"] / mn,
                  A_MB_min=a_bytes / 1e6 / mn, B_jpeg_MB_min=man["jpeg_bytes"] / 1e6 / mn,
                  B_full_MB_min=b_full / 1e6 / mn, B_min_MB_min=b_min / 1e6 / mn,
                  jpeg_kB_per_kf=man["jpeg_bytes"] / 1e3 / man["n_keyframes"],
                  sample_kB=dict(q80=ss["q80"] / nS / 1e3, q90=ss["q90"] / nS / 1e3, q95=ss["q95"] / nS / 1e3,
                                 png=ss["png"] / nS / 1e3, q90_1080=ss["q90_full1080"] / nS / 1e3),
                  live_pose_MB_min=man["live_pose_file_bytes"] / 1e6 / mn, imu_MB_min=man["imu_csv_bytes"] / 1e6 / mn,
                  imu_zlib_MB_min=man["imu_csv_zlib_bytes"] / 1e6 / mn, check_A_equal=(man.get("check_A") or {}).get("equal"))
        r["size"] = sz; sizes.append(sz); rows.append(r)
    n = len(rel_dz)
    rule = None
    if n:
        md, mr, mk = float(np.median(rel_dz)), float(np.median(rel_rms)), float(np.median(dk))
        rule = dict(n=n, med_rel_adz_pct=md, med_rel_rms_pct=mr, med_dk_pp=mk, pass_adz=md <= 5, pass_rms=mr <= 10, pass_k=mk <= 0.3)
        rule["verdict"] = "통과" if (rule["pass_adz"] and rule["pass_rms"] and rule["pass_k"]) else "불합격"
    L = [f"# kfonly 표 (kf_score.py, {time.strftime('%Y-%m-%d %H:%M')}, 미검증)", "",
         "| 녹화 | A |dZ| p90 cm | B | 변화 % | A 메시 RMS cm | B | 변화 % | A k % | B k % | |Δk| %p | 평가 kf A/B | 참고 featloop S |dZ|·RMS·k | 진단: 이중층 비율 A/B | 진단: 주 층 RMS cm A/B |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    f = lambda v, p=2: "—" if v is None else f"{v:.{p}f}"
    for r in rows:
        if r.get("missing"):
            L.append(f"| {r['rec']} | 없음 |||||||||||"); continue
        A, B, S = r["A"], r["B"], r["ref_S"]
        L.append(f"| {r['rec']} | {f(A['adz_p90'])} | {f(B['adz_p90'])} | {r['rel_adz_pct']:+.1f} | {f(A['mesh_rms'])} | {f(B['mesh_rms'])} | "
                 f"{r['rel_rms_pct']:+.1f} | {A['k']:+.2f} | {B['k']:+.2f} | {r['dk_pp']:.3f} | {A['n_eval']}/{B['n_eval']} | "
                 f"{f(S['adz_p90'])} · {f(S['mesh_rms'])} · {f(S['k'])} | {f(A['ghost_frac'])}/{f(B['ghost_frac'])} | "
                 f"{f(A['near_rms_cm'])}/{f(B['near_rms_cm'])} |")
    if rule:
        L += ["", f"**판정 (README 4, n {rule['n']})**: |dZ| p90 변화 중앙 {rule['med_rel_adz_pct']:+.1f} % (≤ +5: {rule['pass_adz']}), "
              f"메시 RMS 변화 중앙 {rule['med_rel_rms_pct']:+.1f} % (≤ +10: {rule['pass_rms']}), |Δk| 중앙 {rule['med_dk_pp']:.3f} %p (≤ 0.3: {rule['pass_k']}) → **{rule['verdict']}**"]
    L += ["", "## 데이터 크기 (MB / 분)", "| 녹화 | 분 | kf/분 | A (mp4+csv+imu) | B JPEG q90 | B 전체 (JPEG+실시간 자세 파일+imu.csv+frames csv) | B 최소 (JPEG+kf 자세+imu zlib) | kB/kf q90 | 표본 kB q80/q90/q95/PNG/1080p q90 |", "|---|---|---|---|---|---|---|---|---|"]
    for s in sizes:
        q = s["sample_kB"]
        L.append(f"| {s['rec']} | {s['min']:.2f} | {s['kf_per_min']:.0f} | {s['A_MB_min']:.0f} | {s['B_jpeg_MB_min']:.1f} | {s['B_full_MB_min']:.1f} | {s['B_min_MB_min']:.1f} | "
                 f"{s['jpeg_kB_per_kf']:.0f} | {q['q80']:.0f}/{q['q90']:.0f}/{q['q95']:.0f}/{q['png']:.0f}/{q['q90_1080']:.0f} |")
    if sizes:
        L.append(f"| 중앙 | | {np.median([s['kf_per_min'] for s in sizes]):.0f} | {np.median([s['A_MB_min'] for s in sizes]):.0f} | {np.median([s['B_jpeg_MB_min'] for s in sizes]):.1f} | "
                 f"{np.median([s['B_full_MB_min'] for s in sizes]):.1f} | {np.median([s['B_min_MB_min'] for s in sizes]):.1f} | {np.median([s['jpeg_kB_per_kf'] for s in sizes]):.0f} | |")
    (HERE / "table.md").write_text("\n".join(L) + "\n")
    (HERE / "table.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), rows=rows, rule=rule), indent=1))
    print("\n".join(L))


if __name__ == "__main__":
    main()
