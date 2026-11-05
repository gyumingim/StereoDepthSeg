#!/usr/bin/env python3
"""scalehunt rulers R1-R4 (README 2) on replay pose files; summary over conditions.

  python sh_score.py score [COND ...]        -> ROOT/score/<COND>.json (per recording)
  python sh_score.py table OUT.json OUT.md   -> paired deltas vs B (README 3 rule)
k = VIO length / GT length - 1 (%), + = VIO too long.
"""
import gzip, json, sys, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sh_batch import ROOT, REC, RECS, ORDER, short  # noqa: E402

BEST = {"hs_20261024_165506_std60": "imu_gt_spl"}   # H: imu_gt_spl_cal1022


def load_poses(f):
    t, P = [], []
    for line in gzip.open(f, "rt"):
        m = json.loads(line)
        if m.get("status") == "tracking" and m.get("T_world_cam0"):
            T = np.asarray(m["T_world_cam0"], float).reshape(4, 4)
            t.append(int(m["t_ns"])); P.append(T[:3, 3])
    return np.array(t, np.int64), np.array(P)


def match(rec, gdir, vt, shift=0):
    g = np.load(REC / rec / gdir / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt_t, gt_p = g["t_ns"][ok], g["p_bc"][ok]
    v = vt + shift
    j = np.clip(np.searchsorted(gt_t, v), 1, len(gt_t) - 1)
    j = np.where(np.abs(gt_t[j - 1] - v) < np.abs(gt_t[j] - v), j - 1, j)
    mm = np.abs(gt_t[j] - v) < 2_000_000
    return mm, gt_p[j[mm]], float(g["K"][0, 0])


def sim3(A, B):
    """B ~ s R A + t (Umeyama). returns s, R, t"""
    ma, mb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma) / len(A))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / np.mean(np.sum((A - ma) ** 2, 1)))
    return s, R, mb - s * R @ ma


def rulers(P, G):
    s_f, R, t = sim3(P, G)                       # VIO -> GT
    s_r, _, _ = sim3(G, P)                       # GT -> VIO
    Pc, Gc = P - P.mean(0), G - G.mean(0)
    Pa = Pc @ R.T                                # VIO rotated into GT (board) frame, not scaled
    k_ax = [float((Pa[:, a] * Gc[:, a]).sum() / (Gc[:, a] ** 2).sum() - 1) * 100 for a in range(3)]
    k_rR = float((Pa * Gc).sum() / (Gc ** 2).sum() - 1) * 100
    Rse, tse = sim3(P, G)[1], None
    # SE3 ATE (no scale)
    U, _, Vt = np.linalg.svd(Gc.T @ Pc / len(P)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); R0 = U @ D @ Vt
    e = np.linalg.norm(Pc @ R0.T - Gc, axis=1)
    ef = np.linalg.norm(s_f * Pc @ R.T - Gc, axis=1)
    return dict(k_fwd=round((1 / s_f - 1) * 100, 3), k_rev=round((s_r - 1) * 100, 3), k_revR=round(k_rR, 3),
                k_x=round(k_ax[0], 3), k_y=round(k_ax[1], 3), k_z=round(k_ax[2], 3),
                gt_std_cm=[round(float(v) * 100, 2) for v in Gc.std(0)], ate_se3_cm=round(float(np.sqrt((e ** 2).mean())) * 100, 3),
                ate_sim3_cm=round(float(np.sqrt((ef ** 2).mean())) * 100, 3), n=int(len(P)))


def score_cell(f, rec):
    r = json.load(open(f))
    vt, P = load_poses(str(f)[:-5] + ".poses.jsonl.gz")
    shift = int(r["gt_align_dt_ms"] * 1e6) if r["args"]["time"] == "oracle" else 0
    out = dict(status=r["score"].get("status"), init_s=r["score"].get("init_s"), lp_scale=r["score"].get("scale_sim3"),
               lp_ate=r["score"].get("ate_vio_cm"))
    if len(P) < 50:
        return out
    for tag, gd in (("old", "imu_gt_spl"), ("best", BEST.get(rec, "imu_gt_spl_cal1022"))):
        if not (REC / rec / gd / "gt.npz").exists():
            continue
        mm, G, fx = match(rec, gd, vt, shift)
        if mm.sum() < 50:
            continue
        out[tag] = dict(gt=gd, gt_fx=round(fx, 2), **rulers(P[mm], G))
    return out


def cmd_score(conds):
    (ROOT / "score").mkdir(exist_ok=True)
    for c in conds:
        res = {}
        for rec in RECS:
            f = ROOT / "runs" / c / (short(rec) + ".json")
            if f.exists():
                res[rec] = score_cell(f, rec)
        json.dump(dict(cond=c, made=time.strftime("%F %T"), cells=res), open(ROOT / "score" / f"{c}.json", "w"), indent=1)
        print(c, len(res), flush=True)


def cmd_table(out_json, out_md):
    S = {}
    for c in ORDER:
        f = ROOT / "score" / f"{c}.json"
        if f.exists():
            S[c] = json.load(open(f))["cells"]
    keys = ["k_fwd", "k_rev", "k_x", "k_y", "k_z"]
    B = S["B"]
    B60 = S.get("B60", {})          # README 3.1: capped conditions are paired with the capped baseline
    rows, lines = {}, []
    lines.append("| cond | n ok | R2 k_fwd med | Δk_fwd med (vs B60; B60 vs B) | sign +/− | Δk_rev med | Δk_z med | Δk_xy med | R1 old-GT Δk_fwd med | ATE med cm |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for c, cells in S.items():
        d = {k: [] for k in keys + ["old"]}
        kb, ate = [], []
        for rec, v in cells.items():
            b = (B if c in ("B", "B60") else B60).get(rec, {})
            if "best" not in v or v.get("status") != "ok":
                continue
            kb.append(v["best"]["k_fwd"]); ate.append(v["best"]["ate_se3_cm"])
            if "best" not in b or b.get("status") != "ok":
                continue
            for k in keys:
                d[k].append(v["best"][k] - b["best"][k])
            if "old" in v and "old" in b:
                d["old"].append(v["old"]["k_fwd"] - b["old"]["k_fwd"])
        med = lambda x: round(float(np.median(x)), 2) if len(x) else None
        dxy = [(a + b2) / 2 for a, b2 in zip(d["k_x"], d["k_y"])]
        pos = sum(1 for x in d["k_fwd"] if x > 0); neg = sum(1 for x in d["k_fwd"] if x < 0)
        rows[c] = dict(n_ok=len(kb), k_fwd_med=med(kb), dk_fwd_med=med(d["k_fwd"]), pos=pos, neg=neg, dk_rev_med=med(d["k_rev"]),
                       dk_z_med=med(d["k_z"]), dk_xy_med=med(dxy), dk_old_med=med(d["old"]), ate_med=med(ate),
                       effect=bool(len(d["k_fwd"]) and abs(np.median(d["k_fwd"])) >= 0.5 and max(pos, neg) >= 10))
        r = rows[c]
        lines.append(f"| {c} | {r['n_ok']} | {r['k_fwd_med']} | {r['dk_fwd_med']} | {pos}/{neg} | {r['dk_rev_med']} | {r['dk_z_med']} | "
                     f"{r['dk_xy_med']} | {r['dk_old_med']} | {r['ate_med']} |" + (" **영향**" if r["effect"] else ""))
    # baseline per recording
    lines += ["", "| rec (B) | GT old fx | old k_fwd | best GT | best fx | k_fwd | k_rev | k_x | k_y | k_z | GT std cm (x,y,z) | ATE se3 |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for rec, v in B.items():
        if "best" not in v:
            lines.append(f"| {rec} | {v.get('status')} |"); continue
        o, b = v.get("old", {}), v["best"]
        lines.append(f"| {rec} | {o.get('gt_fx')} | {o.get('k_fwd')} | {b['gt']} | {b['gt_fx']} | {b['k_fwd']} | {b['k_rev']} | {b['k_x']} | "
                     f"{b['k_y']} | {b['k_z']} | {b['gt_std_cm']} | {b['ate_se3_cm']} |")
    agg = {}
    for tag in ("old", "best"):
        for k in keys:
            x = [v[tag][k] for v in B.values() if tag in v and v.get("status") == "ok"]
            agg[f"{tag}_{k}"] = [round(float(np.median(x)), 2), int(sum(1 for y in x if y > 1)), len(x)] if x else None
    lines += ["", "B 중앙값 [median %, n(k>1 %), n]: " + json.dumps(agg)]
    json.dump(dict(made=time.strftime("%F %T"), rows=rows, B_agg=agg), open(out_json, "w"), indent=1)
    Path(out_md).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    if sys.argv[1] == "score":
        cmd_score(sys.argv[2:] or ORDER)
    else:
        cmd_table(sys.argv[2], sys.argv[3])
