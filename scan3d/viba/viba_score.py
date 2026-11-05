#!/usr/bin/env python3
"""viba M1 scale rulers (README 3) on keyframe pose files, same rule for every source.

  python scan3d/viba/viba_score.py RUN_DIR [--out table.json --md table.md]
RUN_DIR/ba/<rec>/poses_vio_kf.jsonl.gz, poses_ba.jsonl.gz, ba.json; RUN_DIR/viba/<rec>/<variant>/poses_viba.jsonl.gz, viba.json.
k = length / GT length - 1 (%), + = too long. k_fwd = 1/s - 1 of the forward Umeyama (poses -> GT, scalehunt R2),
k_rev = reverse Umeyama s - 1, k_pair = median |dP|/|dGT| - 1 over keyframe pairs 1 s apart with |dGT| > 10 cm (no alignment).
"""
import argparse, gzip, json, time
from pathlib import Path
import numpy as np

REC = Path.home() / "scanner/StereoDepthSeg/scan3d/recordings"


def gt_dir(rec):
    return "imu_gt_spl" if rec.endswith("165506_std60") else "imu_gt_spl_cal1022"   # H: never imu_gt_spl (old fx 1267)


def load(p):
    t, P = [], []
    for line in gzip.open(p, "rt"):
        m = json.loads(line)
        if m.get("status") == "tracking" and m.get("T_world_cam0"):
            t.append(int(m["t_ns"])); P.append(np.asarray(m["T_world_cam0"], float).reshape(4, 4)[:3, 3])
    return np.array(t, np.int64), np.array(P)


def umeyama_s(A, B):
    a, b = A - A.mean(0), B - B.mean(0)
    U, S, Vt = np.linalg.svd(b.T @ a / len(A)); d = np.sign(np.linalg.det(U @ Vt))
    return float((S[0] + S[1] + d * S[2]) / np.mean((a ** 2).sum(1)))


def ate_se3(A, B):
    a, b = A - A.mean(0), B - B.mean(0)
    U, _, Vt = np.linalg.svd(b.T @ a); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return float(np.sqrt((np.linalg.norm(a @ (U @ D @ Vt).T - b, axis=1) ** 2).mean()))


def score(rec, poses, tol_ms):
    g = np.load(REC / rec / gt_dir(rec) / "gt.npz", allow_pickle=True)
    ok = g["ok"].astype(bool) & (np.linalg.norm(g["p_bc"], axis=1) < 1.2)
    gt, gp = g["t_ns"][ok], g["p_bc"][ok]
    t, P = load(poses)
    j = np.clip(np.searchsorted(gt, t), 1, len(gt) - 1)
    j = np.where(np.abs(gt[j - 1] - t) < np.abs(gt[j] - t), j - 1, j)
    mm = np.abs(gt[j] - t) < tol_ms * 1e6
    A, G, tt = P[mm], gp[j[mm]], t[mm]
    if len(A) < 20:
        return dict(n=int(len(A)))
    idx = np.searchsorted(tt, tt + 1_000_000_000); keep = idx < len(tt)
    dv = np.linalg.norm(A[idx[keep]] - A[keep], axis=1); dg = np.linalg.norm(G[idx[keep]] - G[keep], axis=1); s = dg > 0.10
    return dict(n=int(len(A)), gt=gt_dir(rec), gt_fx=float(g["K"][0, 0]), k_fwd=(1 / umeyama_s(A, G) - 1) * 100,
                k_rev=(umeyama_s(G, A) - 1) * 100, k_pair=float((np.median(dv[s] / dg[s]) - 1) * 100) if s.sum() > 20 else None,
                n_pair=int(s.sum()), ate_se3_cm=ate_se3(A, G) * 100, gt_std_cm=(G.std(0) * 100).round(2).tolist(),
                t_ns_set=int(np.bitwise_xor.reduce(tt)) if len(tt) else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--out", default=""); ap.add_argument("--md", default="")
    a = ap.parse_args()
    run = Path(a.run)
    res = {}
    for bd in sorted((run / "ba").iterdir()):
        if not bd.is_dir() or not (bd / "poses_ba.jsonl.gz").exists():
            continue
        rec = "hs_" + bd.name if not bd.name.endswith("165506") else "hs_" + bd.name + "_std60"
        tol = 2.0 if rec.endswith("_std60") else 1.0
        r = {"vio_kf": score(rec, bd / "poses_vio_kf.jsonl.gz", tol), "ba": score(rec, bd / "poses_ba.jsonl.gz", tol)}
        bj = json.loads((bd / "ba.json").read_text())
        r["ba"].update(reproj_med_px=bj.get("final_med_px"), sim3_scale=bj.get("sim3_scale"))
        for vd in sorted((run / "viba" / bd.name).glob("*/poses_viba.jsonl.gz")) if (run / "viba" / bd.name).is_dir() else []:
            r[vd.parent.name] = score(rec, vd, tol)
            vj = json.loads((vd.parent / "viba.json").read_text())
            r[vd.parent.name].update(reproj_med_px=vj.get("reproj_med_px"), lm_iters=vj.get("lm_iters"))
        res[bd.name] = r
    srcs = ["vio_kf", "ba", "ba_g", "viba", "viba_on", "viba_x103", "x_viba_it300", "x_ba_g_it300"]
    lines = ["| 녹화 | n kf (GT 짝) | " + " | ".join(f"{s} k_fwd / k_pair / k_rev" for s in srcs) + " | 재투영 중앙 px ba / viba |",
             "|---" * (len(srcs) + 3) + "|"]
    for rec, r in res.items():
        cells = []
        for s in srcs:
            v = r.get(s, {})
            cells.append(f"{v['k_fwd']:+.2f} / {v['k_pair']:+.2f} / {v['k_rev']:+.2f}" if "k_fwd" in v and v.get("k_pair") is not None
                         else (f"{v['k_fwd']:+.2f} / – / {v['k_rev']:+.2f}" if "k_fwd" in v else "–"))
        rp = lambda s: f"{r[s]['reproj_med_px']:.3f}" if s in r and r[s].get("reproj_med_px") is not None else "–"
        lines.append(f"| {rec} | {r['ba'].get('n')} | " + " | ".join(cells) + f" | {rp('ba')} / {rp('viba')} |")
    summ = {}
    for s in srcs:
        k = [r[s]["k_fwd"] for r in res.values() if s in r and "k_fwd" in r[s]]
        kp = [r[s]["k_pair"] for r in res.values() if s in r and r[s].get("k_pair") is not None]
        if k:
            summ[s] = dict(n=len(k), k_fwd_med=float(np.median(k)), absk_fwd_med=float(np.median(np.abs(k))),
                           k_pair_med=float(np.median(kp)) if kp else None, absk_pair_med=float(np.median(np.abs(kp))) if kp else None)
    # pre-registered rule (README 4): paired vs ba
    for s in ("viba", "viba_on", "viba_x103"):
        pr = [(abs(r[s]["k_fwd"]), abs(r["ba"]["k_fwd"])) for r in res.values() if s in r and "k_fwd" in r[s] and "k_fwd" in r["ba"]]
        if pr and s in summ:
            summ[s].update(n_abs_smaller=sum(1 for x, y in pr if x < y), n_paired=len(pr),
                           d_absk_med=float(np.median([x for x, _ in pr]) - np.median([y for _, y in pr])))
    lines += ["", "| 출처 | n | k_fwd 중앙 | \\|k_fwd\\| 중앙 | k_pair 중앙 | \\|k_pair\\| 중앙 | \\|k\\| 줄어든 녹화 (vs ba) | Δ\\|k_fwd\\| 중앙 %p |", "|---|---|---|---|---|---|---|---|"]
    for s, v in summ.items():
        f = lambda x: "–" if x is None else f"{x:+.2f}"
        lines.append(f"| {s} | {v['n']} | {f(v['k_fwd_med'])} | {v['absk_fwd_med']:.2f} | {f(v['k_pair_med'])} | "
                     f"{'–' if v['absk_pair_med'] is None else format(v['absk_pair_med'], '.2f')} | "
                     f"{str(v.get('n_abs_smaller', '–')) + '/' + str(v.get('n_paired', '–'))} | {f(v.get('d_absk_med'))} |")
    md = "\n".join(lines)
    print(md)
    if a.out:
        Path(a.out).write_text(json.dumps(dict(made=time.strftime("%F %T"), run=str(run), per_rec=res, summary=summ), indent=1))
    if a.md:
        Path(a.md).write_text(md + "\n")


if __name__ == "__main__":
    main()
