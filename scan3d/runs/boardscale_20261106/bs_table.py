#!/usr/bin/env python3
"""boardscale 2026-11-06 scoring (README 2-3). Reads board_scale.json + depth summaries; the GT poses are read HERE only
(evaluation), never by board_scale.py.  python scan3d/runs/boardscale_20261106/bs_table.py -> table.json, table.md"""
import gzip, json, time
from pathlib import Path
import numpy as np

H = Path.home() / "scanner"
O = Path(__file__).resolve().parent
RD = H / "StereoDepthSeg/scan3d/recordings"
SA = H / "StereoDepthSeg-scanA/scan3d/runs/scanA_20261105"
FL = H / "StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec"
MAIN = ["165506", "20261022_134204", "20261021_213341", "20261020_222747", "20261020_214138"]
EXTRA = ["165506", "20261022_134204", "20261021_213341", "20261020_224323", "20261021_214550", "20261022_133820", "20261022_134550"]


def recdir(r): return RD / ("hs_20261024_165506_std60" if r == "165506" else f"hs_{r}")
def gtl(r): return "imu_gt_spl" if r == "165506" else "imu_gt_spl_cal1022"


def gt_scale(r, poses):
    """Sim(3) scale of the BA keyframe centres onto the GT centres (same as scan_offline sim3_scale_would_be)."""
    g = np.load(recdir(r) / gtl(r) / "gt.npz")
    A, B = [], []
    for line in gzip.open(poses, "rt"):
        d = json.loads(line)
        f = int(d["frame"])
        if g["ok"][f] and abs(int(g["t_ns"][f]) - int(d["t_ns"])) < 1e6:
            A.append(np.array(d["T_world_cam0"]).reshape(4, 4)[:3, 3]); B.append(g["p_bc"][f])
    A, B = np.array(A), np.array(B)
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()), len(A), str(g["board_json"]), float(np.load(recdir(r) / gtl(r) / "gt.npz")["square_mm"])


def summ(p):
    p = Path(p) / "summary.json"
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    return dict(dZ_mean=d["pooled_dZ_cm"]["mean"], dZ_med=d["pooled_dZ_cm"]["median"], absdZ_p90=d["pooled_absdZ_cm"]["p90"],
                kf_bias_med=d.get("kf_board_dZ_bias_cm_median"), mesh_z_med=d.get("mesh_board_z_med_cm"), n_eval=d.get("n_kf_board_eval"),
                sim3=d["poses"].get("sim3_scale_would_be"), free_scale=d["poses"].get("free_scale_applied"), made=d["made"])


def main():
    out = dict(made=time.strftime("%F %T"), main={}, extra={})
    md = ["| 녹화 | 보드 칸 mm | 보드 키프레임 | 보드 축척 c (부트 σ, 95 %) | GT 축척 | 차 % | 기준 ≤0.5 % | 95 % 가 GT 덮음 | 검출 vs GT 검출 px |",
          "|---|---|---|---|---|---|---|---|---|"]
    for grp, L, src in (("main", MAIN, "scanA"), ("extra", EXTRA, "S")):
        if grp == "extra":
            md += ["", "추가 (featloop 시드 0 SIFT BA, 축척만):", "", md[0], md[1]]
        for r in L:
            bj = O / "scale" / f"{r}_{src}" / "board_scale.json"
            if not bj.exists():
                continue
            b = json.loads(bj.read_text())
            cg, n, gboard, gsq = gt_scale(r, b["poses"])
            diff = (b["c"] / cg - 1) * 100
            cover = b["boot"]["p2_5"] <= cg <= b["boot"]["p97_5"]
            row = dict(c=b["c"], boot=b["boot"], c_gt=cg, n_gt=n, diff_pct=diff, pass_scale=abs(diff) <= 0.5, boot_covers_gt=cover,
                       n_kf_board=b["n_kf_board"], n_poses=b["n_poses"], square_mm=b["square_mm"], board=b["board"],
                       board_matches_gt_file=Path(gboard).name == Path(b["board"]).name and abs(gsq - b["square_mm"]) < 1e-6,
                       pair2v_median=b["pair2v"]["median"], best=b["best"], check=b.get("check_vs_gt_detections"))
            if grp == "main":
                base = SA / "step3/ba" if r == "165506" else SA / f"more/mesh/{r}_ba"
                orc = SA / "step3/ba_fs" if r == "165506" else O / "depth" / f"{r}_oracle"
                row["depth"] = dict(ba=summ(base), board=summ(O / "depth" / f"{r}_board"), oracle=summ(orc),
                                    ba_rerun=summ(O / "depth" / f"{r}_ba_rerun"))
                dd = row["depth"]
                if dd["board"] and dd["oracle"]:
                    row["pass_depth"] = bool(abs(dd["board"]["dZ_mean"] - dd["oracle"]["dZ_mean"]) <= 0.3 and
                                             dd["board"]["absdZ_p90"] <= 1.1 * dd["oracle"]["absdZ_p90"] + 0.1)
            out[grp][r] = row
            ck = row["check"]
            md.append(f"| {r} | {b['square_mm']:.3f} | {b['n_kf_board']}/{b['n_poses']} | {b['c']:.4f} ({b['boot']['sd']:.4f}, "
                      f"{b['boot']['p2_5']:.4f}–{b['boot']['p97_5']:.4f}) | {cg:.4f} | {diff:+.2f} | {'예' if row['pass_scale'] else '아니오'} | "
                      f"{'예' if cover else '아니오'} | {ck['median_px']:.3f} (n {ck['n']}) |" if ck else "")
    md += ["", "깊이 (전체 화소, cm; BA = 원래 VIO 축척, 보드 = 보드 축척, 오라클 = GT 축척):", "",
           "| 녹화 | dZ 평균 BA / 보드 / 오라클 | dZ 중앙 BA / 보드 / 오라클 | abs dZ p90 BA / 보드 / 오라클 | kf 치우침 중앙 BA / 보드 / 오라클 | 메시 보드 높이 BA / 보드 / 오라클 | 깊이 기준 |",
           "|---|---|---|---|---|---|---|"]
    f = lambda x: "—" if x is None else f"{x:+.2f}"
    for r, row in out["main"].items():
        d = row["depth"]
        g = lambda k: " / ".join(f(d[s][k]) if d[s] else "—" for s in ("ba", "board", "oracle"))
        md.append(f"| {r} | {g('dZ_mean')} | {g('dZ_med')} | {g('absdZ_p90')} | {g('kf_bias_med')} | {g('mesh_z_med')} | "
                  f"{ {True: '통과', False: '못 함'}.get(row.get('pass_depth'), '—') } |")
        if d.get("ba_rerun"):
            md.append(f"| {r} (BA 다시, 결정성) | {f(d['ba_rerun']['dZ_mean'])} | {f(d['ba_rerun']['dZ_med'])} | {f(d['ba_rerun']['absdZ_p90'])} | "
                      f"{f(d['ba_rerun']['kf_bias_med'])} | {f(d['ba_rerun']['mesh_z_med'])} | |")
    (O / "table.json").write_text(json.dumps(out, indent=1))
    (O / "table.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
