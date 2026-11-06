#!/usr/bin/env python3
"""night1107 scoring (README 2-3): 173432 vs 165506, same rulers. GT poses read HERE only (evaluation).
k_fwd / k_pair = viba_score.score (copied, GT imu_gt_spl, tol 2 ms); board c vs GT = bs_table.gt_scale; depth = scan_offline summary."""
import gzip, json, time
from pathlib import Path
import numpy as np

H = Path.home() / "scanner"; O = Path(__file__).resolve().parent
RD = H / "StereoDepthSeg/scan3d/recordings"
FL = H / "StereoDepthSeg-featloop/scan3d/runs/featloop_20261105/rec"
BS = H / "StereoDepthSeg-boardscale/scan3d/runs/boardscale_20261106/scale"
SA = H / "StereoDepthSeg-boardscale/scan3d/runs/scanA_20261105/step1"
RECS = {"165506": dict(rec=RD / "hs_20261024_165506_std60", ba=FL / "165506/ba_S", board=BS / "165506_S/board_scale.json",
                       vio_json=SA / "B_o0.json", d_ba=FL / "165506/mesh_S", d_board=O / "depth/165506_S_board",
                       d_oracle=O / "depth/165506_S_oracle"),
        "173432": dict(rec=RD / "hs_20261024_173432_std60", ba=O / "rec/173432/ba_S", board=O / "scale/173432_S/board_scale.json",
                       vio_json=O / "vio/173432_B_o0.json", d_ba=O / "depth/173432_ba", d_board=O / "depth/173432_board",
                       d_oracle=O / "depth/173432_oracle"),
        "173432A": dict(rec=RD / "hs_20261024_173432_std60", ba=O / "rec/173432/ba_SA", board=O / "scale/173432_SA/board_scale.json",
                        vio_json=O / "vio/173432_A_o0.json", d_ba=O / "depth/173432A_ba", d_board=O / "depth/173432A_board",
                        d_oracle=O / "depth/173432A_oracle")}
for _n in ("173432_A_o1", "173432_A_o2", "165506_A_o0"):   # README 5.1 (exploratory, no oracle run)
    RECS[_n] = dict(rec=RECS[_n[:6]]["rec"], ba=O / f"rec/{_n}/ba_S", board=O / f"scale/{_n}_S/board_scale.json",
                    vio_json=O / f"vio/{_n}.json", d_ba=O / f"depth/{_n}_ba", d_board=O / f"depth/{_n}_board", d_oracle=O / "none")
VIOS = [("165506", "B_o0 (기본, scanA)", SA / "B_o0"), ("165506", "B_o1 (기본, scanA)", SA / "B_o1"), ("165506", "B_o2 (기본, scanA)", SA / "B_o2"),
        ("165506", "A_o0 (감사 초기화)", O / "vio/165506_A_o0")] + \
       [("173432", f"{v}_o{o} ({'기본' if v == 'B' else '감사 초기화'})", O / f"vio/173432_{v}_o{o}") for v in "BA" for o in range(3)]


def vio_rows():
    rows = []
    for r, name, base in VIOS:
        js, pz = Path(str(base) + ".json"), Path(str(base) + ".poses.jsonl.gz")
        if not js.exists():
            continue
        s = json.loads(js.read_text()); s = s.get("score", s)
        t, _ = load(pz) if pz.exists() else ([], None)
        row = dict(rec=r, name=name, status=s.get("status"), tracked=int(len(t)), init_s=s.get("init_s"), ate_vio_cm=s.get("ate_vio_cm"),
                   scale_sim3=s.get("scale_sim3"), path_m=s.get("path_m"))
        if len(t) > 50:
            row.update({"k_" + k: v for k, v in kscore(RECS[r]["rec"], pz).items() if k in ("k_fwd", "k_pair", "n")})
        rows.append(row)
    return rows


def gt(rec):
    return np.load(rec / "imu_gt_spl/gt.npz", allow_pickle=True)


def load(p):
    t, P = [], []
    for line in gzip.open(p, "rt"):
        m = json.loads(line)
        if m.get("status", "tracking") == "tracking" and m.get("T_world_cam0"):
            t.append(int(m["t_ns"])); P.append(np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3])
    return np.array(t, np.int64), np.array(P)


def umeyama_s(A, B):
    a, b = A - A.mean(0), B - B.mean(0)
    U, S, Vt = np.linalg.svd(b.T @ a / len(A)); d = np.sign(np.linalg.det(U @ Vt))
    return float((S[0] + S[1] + d * S[2]) / np.mean((a ** 2).sum(1)))


def kscore(rec, poses, tol_ms=2.0):
    g = gt(rec); ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gtt, gp = g["t_ns"][ok], g["p_bc"][ok]
    t, P = load(poses)
    j = np.clip(np.searchsorted(gtt, t), 1, len(gtt) - 1)
    j = np.where(np.abs(gtt[j - 1] - t) < np.abs(gtt[j] - t), j - 1, j)
    mm = np.abs(gtt[j] - t) < tol_ms * 1e6
    A, G, tt = P[mm], gp[j[mm]], t[mm]
    idx = np.searchsorted(tt, tt + 1_000_000_000); keep = idx < len(tt)
    dv = np.linalg.norm(A[idx[keep]] - A[keep], axis=1); dg = np.linalg.norm(G[idx[keep]] - G[keep], axis=1); s = dg > 0.10
    return dict(n=int(len(A)), k_fwd=(1 / umeyama_s(A, G) - 1) * 100, k_rev=(umeyama_s(G, A) - 1) * 100,
                k_pair=float((np.median(dv[s] / dg[s]) - 1) * 100) if s.sum() > 20 else None, n_pair=int(s.sum()),
                gt_std_cm=(G.std(0) * 100).round(2).tolist())


def gt_scale(rec, poses):
    g = gt(rec); A, B = [], []
    for line in gzip.open(poses, "rt"):
        d = json.loads(line); f = int(d["frame"])
        if g["ok"][f] and abs(int(g["t_ns"][f]) - int(d["t_ns"])) < 1e6:
            A.append(np.array(d["T_world_cam0"]).reshape(4, 4)[:3, 3]); B.append(g["p_bc"][f])
    A, B = np.array(A), np.array(B); ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()), len(A)


def summ(p):
    p = Path(p) / "summary.json"
    if not p.exists():
        return None
    d = json.loads(p.read_text())
    return dict(dZ_mean=d["pooled_dZ_cm"]["mean"], dZ_med=d["pooled_dZ_cm"]["median"], absdZ_p90=d["pooled_absdZ_cm"]["p90"],
                kf_bias_med=d.get("kf_board_dZ_bias_cm_median"), mesh_z_med=d.get("mesh_board_z_med_cm"),
                mesh_rms=d.get("mesh_board_z_rms_cm"), cover=d.get("mesh_board_cover_frac"), n_eval=d.get("n_kf_board_eval"),
                n_kf=d.get("n_kf"), n_no_pair=d.get("n_kf_no_pair"), sim3=d["poses"].get("sim3_scale_would_be"),
                se3_rmse_cm=d["poses"].get("se3_rmse_cm"), free_scale=d["poses"].get("free_scale_applied"),
                vio=d["args"].get("vio"), made=d["made"])


def main():
    out = dict(made=time.strftime("%F %T"), recs={})
    for r, c in RECS.items():
        row = dict(inputs={k: str(v) for k, v in c.items()})
        if (c["ba"] / "poses_ba.jsonl.gz").exists():
            row["k_vio_kf"] = kscore(c["rec"], c["ba"] / "poses_vio_kf.jsonl.gz")
            row["k_ba"] = kscore(c["rec"], c["ba"] / "poses_ba.jsonl.gz")
            bj = json.loads((c["ba"] / "ba.json").read_text())
            row["ba"] = {k: bj.get(k) for k in ("final_med_px", "sim3_scale", "n_kf", "n_points", "n_obs")}
        if c["vio_json"].exists():
            vj = json.loads(c["vio_json"].read_text())
            row["vio_json"] = {k: v for k, v in vj.get("score", {}).items() if not isinstance(v, (dict, list))}
            row["vio_json"].update(wall_s=vj.get("wall_s"), crashed=vj.get("crashed"), bridge_sha=vj.get("bridge_sha"))
        if c["board"].exists():
            b = json.loads(c["board"].read_text()); cg, n = gt_scale(c["rec"], b["poses"])
            row["board"] = dict(c=b["c"], boot=b["boot"], c_gt=cg, n_gt=n, diff_pct=(b["c"] / cg - 1) * 100,
                                pass_scale=abs(b["c"] / cg - 1) * 100 <= 0.5, covers=b["boot"]["p2_5"] <= cg <= b["boot"]["p97_5"],
                                n_kf_board=b["n_kf_board"], n_poses=b["n_poses"], pair2v_median=b["pair2v"]["median"],
                                check=b.get("check_vs_gt_detections"), n_corners=b.get("n_corners"))
        d = row["depth"] = {s: summ(c["d_" + s]) for s in ("ba", "board", "oracle")}
        if d["board"] and d["oracle"]:
            row["pass_depth"] = bool(abs(d["board"]["dZ_mean"] - d["oracle"]["dZ_mean"]) <= 0.3 and
                                     d["board"]["absdZ_p90"] <= 1.1 * d["oracle"]["absdZ_p90"] + 0.1)
        kb = row.get("k_ba")
        if kb:
            kf, kp = kb["k_fwd"], kb["k_pair"]
            row["verdict_2pct"] = ("다시 나옴" if 1.0 <= kf <= 3.0 and kp is not None and kp >= 0.5 else
                                   "안 나옴" if abs(kf) < 1.0 else "다른 크기")
        out["recs"][r] = row
    out["vio"] = vio_rows()
    (O / "table.json").write_text(json.dumps(out, indent=1, ensure_ascii=False))
    f = lambda x, n=2: "—" if x is None else f"{x:+.{n}f}"
    vmd = ["", "VIO 재생 (c4f60; 기본 = 사전 등록, 감사 초기화 = 사후 변형 init_window_time 1.5 / init_max_disparity 2.30):", "",
           "| 녹화 | run | 상태 | tracking 프레임 | 초기화 s | ATE cm | lp sim3 축척 | k_fwd % (전체 프레임) | k_pair % |", "|---|---|---|---|---|---|---|---|---|"]
    for v in out["vio"]:
        vmd.append(f"| {v['rec']} | {v['name']} | {v['status']} | {v['tracked']} | {v['init_s']} | {v['ate_vio_cm']} | {v['scale_sim3']} | "
                   f"{f(v.get('k_k_fwd'))} | {f(v.get('k_k_pair'))} |")
    md = [f"night1107 채점 {out['made']} (미검증 · 독립 검사 전)", "",
          "| 녹화 | VIO ATE cm | k_fwd VIO kf / BA % | k_pair VIO kf / BA % | 판정 (+2 %) | BA 재투영 px | 보드 c (σ) | GT 축척 | c 차 % | 보드 kf |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r, row in out["recs"].items():
        kv, kb, b = row.get("k_vio_kf", {}), row.get("k_ba", {}), row.get("board")
        ate = row.get("vio_json", {}).get("ate_vio_cm")
        cells = (f"{b['c']:.4f} ({b['boot']['sd']:.4f}) | {b['c_gt']:.4f} | {b['diff_pct']:+.2f} | {b['n_kf_board']}/{b['n_poses']} |"
                 if b else "— | — | — | — |")
        md.append(f"| {r} | {'—' if ate is None else format(ate, '.2f')} | {f(kv.get('k_fwd'))} / {f(kb.get('k_fwd'))} | "
                  f"{f(kv.get('k_pair'))} / {f(kb.get('k_pair'))} | {row.get('verdict_2pct', '—')} | "
                  f"{row.get('ba', {}).get('final_med_px', '—')} | " + cells)
    md += ["", "깊이 (cm; BA = 원래 축척 / 보드 = 보드 축척 / 오라클 = GT 축척):", "",
           "| 녹화 | dZ 평균 | abs dZ p90 | kf 치우침 중앙 | 메시 보드 높이 중앙 | 메시 RMS | 덮임 | 평가 kf | 깊이 기준 |",
           "|---|---|---|---|---|---|---|---|---|"]
    for r, row in out["recs"].items():
        d = row["depth"]
        g = lambda k, fm=f: " / ".join(fm(d[s][k]) if d[s] and d[s][k] is not None else "—" for s in ("ba", "board", "oracle"))
        pd = {True: "통과", False: "못 함"}.get(row.get("pass_depth"), "—")
        md.append(f"| {r} | {g('dZ_mean')} | {g('absdZ_p90', lambda x: format(x, '.2f'))} | {g('kf_bias_med')} | {g('mesh_z_med')} | "
                  f"{g('mesh_rms', lambda x: format(x, '.2f'))} | {g('cover', lambda x: format(x, '.3f'))} | {g('n_eval', str)} | {pd} |")
    md += vmd
    (O / "table.md").write_text("\n".join(md) + "\n"); print("\n".join(md))


if __name__ == "__main__":
    main()
