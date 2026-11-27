#!/usr/bin/env python3
"""2x2 {60, 30 fps} x {VIO poses, BA poses} on the SAME 20 keyframe timestamps (night 11-27, after the Fable direction critic).
Keyframes: 20 evenly spread over the ba_partners2 keyframes whose 60 fps frame index is even (so the frame exists at 30 fps = index / 2)
and that have >= 1 strict partner in the official 60 fps plan. Poses: BA = ba_partners2 cams T (same for both rates, same timestamps);
VIO = the official 60 fps VIO (60 arms) / the c4 30 fps VIO (30 arms), at the keyframe timestamp. Partners are always VIO-relative
(as in the official pipeline). Writes arm/run/dump_ba/cams.npz for A60_BA, A60_VIO, A30_BA, A30_VIO + arms.json."""
import gzip, json, time
from pathlib import Path

import numpy as np

D = Path(__file__).resolve().parent; H = Path.home() / "scanner/StereoDepthSeg-gsmap/scan3d/runs"
V2 = H / "gsmap_20261125_121801_v2"; REC60 = H / "dryrun30_121801_20261127/rec_hs_20261112_121801_std60"; REC30 = H / "dryrun30_121801_20261127/rec_hs_20261112_121801_std30"
VIO60 = Path("/home/system/scanner/StereoDepthSeg-fixes/scan3d/runs/laptop/20261112_121801_std60/fix_u8/scan/vio/B_o0.poses.jsonl.gz")
VIO30 = H / "night_20261127/rehearsal30/c4/scan/vio/B_o0.poses.jsonl.gz"


def poses(p):
    t, T = [], []
    for line in gzip.open(p, "rt"):
        d = json.loads(line)
        if d.get("type") == "pose" and d.get("T_world_cam0") and d.get("status") == "tracking":
            t.append(int(d["t_ns"])); T.append(np.array(d["T_world_cam0"], float).reshape(4, 4))
    return np.array(t), np.array(T)


def main():
    c = np.load(V2 / "ba_partners2/run/dump_ba/cams.npz"); kf = c["kf"]; T = c["T"]
    g60 = np.load(REC60 / "lensonly/gt.npz"); g30 = np.load(REC30 / "lensonly/gt.npz")
    plan = json.loads((V2 / "ffs_pairs2/plan.json").read_text())["chosen"]["plan"]
    t60, P60 = poses(VIO60); t30, P30 = poses(VIO30)
    hasv = lambda tv, t: np.min(np.abs(tv - t)) < 8e6   # noqa: E731  (VIO tracking pose at that time in BOTH rates)
    cand = [i for i, f in enumerate(kf) if int(f) % 2 == 0 and any(q[1] != "fallback" for q in plan.get(str(int(f)), []))
            and hasv(t60, g60["t_ns"][int(f)]) and hasv(t30, g60["t_ns"][int(f)])]
    sel = [cand[int(k)] for k in np.linspace(0, len(cand) - 1, 20).round().astype(int)]
    f60 = np.array([int(kf[i]) for i in sel]); f30 = f60 // 2
    assert np.all(g60["t_ns"][f60] == g30["t_ns"][f30]), "30 fps frame k must be 60 fps frame 2k"
    out = {}
    for arm, rec, fr, g, (tv, Tv) in (("A60_BA", REC60, f60, g60, (None, None)), ("A60_VIO", REC60, f60, g60, (t60, P60)),
                                      ("A30_BA", REC30, f30, g30, (None, None)), ("A30_VIO", REC30, f30, g30, (t30, P30))):
        if tv is None:
            TT = T[sel]
        else:
            TT = []
            for f in fr:
                k = int(np.argmin(np.abs(tv - g["t_ns"][f]))); assert abs(tv[k] - g["t_ns"][f]) < 8e6, (arm, f); TT.append(Tv[k])
            TT = np.array(TT)
        K = g["K"].copy(); Kr = K.copy(); Kr[:2] *= 0.5
        (D / arm / "run/dump_ba").mkdir(parents=True, exist_ok=True)
        np.savez(D / arm / "run/dump_ba/cams.npz", kf=fr, T=TT, t_ns=g["t_ns"][fr], Kr=Kr, K=K, dist=g["dist"], size=np.array([960, 540]),
                 made=np.array(time.strftime("%F %T")), poses=np.array(arm))
        out[arm] = dict(rec=str(rec), vio=str(VIO60 if "60" in arm else VIO30), kf=fr.tolist(), poses="BA ba_partners2" if tv is None else "VIO")
    (D / "arms.json").write_text(json.dumps(dict(made=time.strftime("%F %T"), kf_idx=sel, arms=out), indent=1)); print(json.dumps({k: v["kf"][:5] for k, v in out.items()}))


if __name__ == "__main__":
    main()
