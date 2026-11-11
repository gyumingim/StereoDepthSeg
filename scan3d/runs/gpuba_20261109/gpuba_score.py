#!/usr/bin/env python3
"""gpuba 11-11: collect laptop results runs/laptop/<rid>/gpuba/{cpu_db,cuda_db,mesh_cpu,mesh_cuda} -> table.md / table.json and the
pre-registered rule (README 3): every H |k_new - k_cpu| <= 0.1 %p, every H (|dZ|p90 new - cpu)/cpu <= +5 %, sum of runtimes smaller."""
import gzip, json, sys, time
from pathlib import Path
import numpy as np

W = Path(__file__).resolve().parents[3]
L = W / "scan3d/runs/laptop"
D = Path(__file__).resolve().parent
H = ["20261022_133820", "20261022_134550", "20261022_134204", "20261021_214550", "20261021_213341", "20261021_215348",
     "20261020_221052", "20261020_221925", "20261020_222747", "20261020_223815", "20261020_224323"]


def J(p):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return None


def poses(p):
    try:
        return np.array([json.loads(l)["T_world_cam0"] for l in gzip.open(p, "rt")])
    except Exception:
        return None


def kofm(m):
    s = (m or {}).get("poses", {}).get("sim3_scale_would_be")
    return (1 / s - 1) * 100 if s else None


rows = []
for rid in H:
    O = L / rid / "gpuba"
    r = dict(rec=rid)
    for v in ("cpu", "cuda"):
        b = J(O / f"{v}_db/ba.json"); m = J(O / f"mesh_{v}/summary.json")
        if b:
            tm = b["timing"]; ph = tm.get("ba_phases", {})
            r[f"t_{v}"] = round(b["runtime_s"] - tm.get("images_s", 0), 1)
            r[f"verify_{v}"] = tm.get("matching_s"); r[f"tracks_{v}"] = tm.get("tracks_s")
            r[f"tri_{v}"] = tm.get("tri_s", round(tm.get("tracks_tri_s", 0) - tm.get("tracks_s", 0), 1)); r[f"ba_{v}"] = tm.get("ba_s")
            r[f"schur_{v}"] = ph.get("schur_s"); r[f"pairs"] = ph.get("n_obs_pairs"); r["points"] = b["points"]; r["obs"] = b["obs"]
            r[f"sim3_{v}"] = b["sim3_scale"]
        if m:
            r[f"k_{v}"] = kofm(m); r[f"adz_{v}"] = (m.get("pooled_absdZ_cm") or {}).get("p90")
    a, c = poses(O / "cpu_db/poses_ba.jsonl.gz"), poses(O / "cuda_db/poses_ba.jsonl.gz")
    r["pose_maxdiff"] = float(np.abs(a - c).max()) if a is not None and c is not None and a.shape == c.shape else None
    r["tri_equal"] = all((J(O / "tri_equal.json") or {"x": False}).values())
    if r.get("k_cpu") is not None and r.get("k_cuda") is not None:
        r["dk_pp"] = r["k_cuda"] - r["k_cpu"]
    if r.get("adz_cpu") and r.get("adz_cuda") is not None:
        r["dadz_pct"] = (r["adz_cuda"] - r["adz_cpu"]) / r["adz_cpu"] * 100
    rows.append(r)

f = lambda x, d=2: "—" if x is None else (f"{x:.{d}f}" if isinstance(x, float) else str(x))
cols = [("rec", 0), ("points", 0), ("obs", 0), ("pairs", 0), ("tri_equal", 0), ("pose_maxdiff", 0), ("k_cpu", 3), ("k_cuda", 3), ("dk_pp", 4),
        ("adz_cpu", 3), ("adz_cuda", 3), ("dadz_pct", 2), ("verify_cpu", 1), ("tracks_cpu", 1), ("tri_cpu", 1), ("ba_cpu", 1), ("schur_cpu", 1),
        ("t_cpu", 1), ("verify_cuda", 1), ("tracks_cuda", 1), ("tri_cuda", 1), ("ba_cuda", 1), ("schur_cuda", 1), ("t_cuda", 1)]
out = ["| " + " | ".join(c for c, _ in cols) + " |", "|" + "---|" * len(cols)]
for r in rows:
    out.append("| " + " | ".join((f"{r[c]:.1e}" if c == "pose_maxdiff" and r.get(c) is not None else f(r.get(c), d) if d else f(r.get(c)))
                                 for c, d in cols) + " |")
done = [r for r in rows if r.get("dk_pp") is not None and r.get("dadz_pct") is not None and r.get("t_cpu") and r.get("t_cuda")]
rule = dict(n=len(done), n_needed=len(H),
            k_ok=all(abs(r["dk_pp"]) <= 0.1 for r in done), k_max_abs_pp=max((abs(r["dk_pp"]) for r in done), default=None),
            adz_ok=all(r["dadz_pct"] <= 5 for r in done), adz_max_pct=max((r["dadz_pct"] for r in done), default=None),
            t_cpu_sum=round(sum(r["t_cpu"] for r in done), 1), t_cuda_sum=round(sum(r["t_cuda"] for r in done), 1))
rule["time_ok"] = rule["t_cuda_sum"] < rule["t_cpu_sum"]
rule["speedup"] = round(rule["t_cpu_sum"] / rule["t_cuda_sum"], 2) if rule["t_cuda_sum"] else None
rule["pass_1_2_3a"] = rule["n"] == len(H) and rule["k_ok"] and rule["adz_ok"] and rule["time_ok"]
out += ["", f"rule (README 3, n {rule['n']}/{len(H)}): {json.dumps(rule)}", f"made {time.strftime('%F %T')}, laptop results under {L}"]
(D / "table.md").write_text("\n".join(out) + "\n")
(D / "table.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), rows=rows, rule=rule), indent=1))
print("\n".join(out))
