#!/usr/bin/env python3
"""mvggt (coordinator 11-06): overlap-only comparison. Per recording: the keyframes evaluated on the board in EVERY variant
present (B0, V1, V2, M1, M2 at that scale) -> (a) per-keyframe stats on that set from keyframes.json (median over kf of the kf
median |dZ|, of the kf p90 |dZ|, of |kf bias|) and (b) kf list rec/<id>/overlap_<scale>.txt (recording-row t_ns) for re-fused
meshes on the same set (mesh_<V>_ov[_fs], pooled + mesh metrics). usage: overlap.py [--write-lists]"""
import json
import sys
from pathlib import Path

import numpy as np

H = Path(__file__).resolve().parent
VARS = ("B0", "V1", "V2", "M1", "M2")


def kf(rd, name):
    p = rd / f"mesh_{name}" / "keyframes.json"
    if not p.exists():
        return None
    return {r["i"]: r for r in json.loads(p.read_text()) if "board_absdZ_p50_cm" in r}


def main():
    out = {}
    for rd in sorted((H / "rec").iterdir()):
        if not (rd / "frames" / "frames.npz").exists():
            continue
        fr = np.load(rd / "frames" / "frames.npz")
        summ = json.loads((rd / "mesh_B0" / "summary.json").read_text()) if (rd / "mesh_B0" / "summary.json").exists() else None
        if summ is None:
            continue
        compressed = bool(summ["args"].get("compress"))
        for fs in ("", "_fs"):
            K = {v: kf(rd, v + fs) for v in VARS}
            K = {v: k for v, k in K.items() if k is not None}
            if "B0" not in K or len(K) < 2:
                continue
            # keyframe identity: recording frame (compressed runs store it as 'frame'), else the row index i
            ident = {v: {(r.get("frame", i) if compressed else i): r for i, r in k.items()} for v, k in K.items()}
            common = sorted(set.intersection(*[set(d) for d in ident.values()]))
            res = dict(n_common=len(common), n_each={v: len(d) for v, d in ident.items()}, vars={})
            for v, d in ident.items():
                a50 = np.array([d[c]["board_absdZ_p50_cm"] for c in common])
                a90 = np.array([d[c]["board_absdZ_p90_cm"] for c in common])
                bias = np.array([d[c]["board_dZ_med_cm"] for c in common])
                res["vars"][v] = dict(kf_absdZ_med_median=float(np.median(a50)), kf_absdZ_p90_median=float(np.median(a90)),
                                      kf_absbias_median=float(np.median(np.abs(bias))), kf_bias_mean=float(bias.mean()),
                                      kf_frac_absbias_gt1cm=float(np.mean(np.abs(bias) > 1)))
            out[f"{rd.name}{fs or '_native'}"] = res
            if "--write-lists" in sys.argv:
                rows = np.asarray(fr["row"]); trow = np.asarray(fr["t_row"])
                m = {int(r): int(t) for r, t in zip(rows, trow)}
                lst = [m[c] for c in common]          # common ids are recording rows (165506) or frames (compressed)
                (rd / f"overlap{fs or '_native'}.txt").write_text("\n".join(map(str, lst)) + "\n")
    (H / "overlap.json").write_text(json.dumps(out, indent=1))
    md = ["| rec · scale | kf in all variants | var | kf median |dZ| (median over kf) | kf p90 |dZ| (median) | |kf bias| median | kf |bias|>1 cm |",
          "|---|---|---|---|---|---|---|"]
    for k, r in out.items():
        for v, s in r["vars"].items():
            md.append(f"| {k} | {r['n_common']} | {v} | {s['kf_absdZ_med_median']:.2f} | {s['kf_absdZ_p90_median']:.2f} | "
                      f"{s['kf_absbias_median']:.2f} | {s['kf_frac_absbias_gt1cm']:.2f} |")
    (H / "overlap.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
