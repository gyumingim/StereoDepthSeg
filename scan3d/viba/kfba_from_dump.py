#!/usr/bin/env python3
"""viba (README 2.5): the kf_ba vision-only BA (same ba_lm Schur LM, Huber 1 px, same fixed keyframes, Sim(3) onto the VIO
centres) run on the kf_ba dump (= same observations as VI-BA), without recomputing features.
  python scan3d/viba/kfba_from_dump.py BA_DIR [--iters 40]  -> BA_DIR/poses_ba.jsonl.gz, poses_vio_kf.jsonl.gz, ba.json"""
import argparse, gzip, json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from kf_ba import ba_lm, umeyama  # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("bdir"); ap.add_argument("--iters", type=int, default=40); a = ap.parse_args()
t0 = time.time(); out = Path(a.bdir); D = np.load(out / "viba_input.npz")
logf = open(out / "ba_from_dump.log", "w")
def log(s):
    print(s, flush=True); logf.write(s + "\n"); logf.flush()
kfs, tk, Tv, Kr = D["kfs"], D["t_ns"], D["Tv"], D["Kr"]; fixed = [int(f) for f in D["fixed"]]
R1, t1, X1, st = ba_lm(D["Rcw"], D["tcw"], D["X0"], D["cam"], D["pt"], D["uv"], Kr, fixed=fixed, iters=a.iters, log=log)
Cb = np.array([-R1[k].T @ t1[k] for k in range(len(kfs))]); Cv = Tv[:, :3, 3]
fr = np.array([k not in set(fixed) or k == fixed[0] for k in range(len(kfs))])
sc, Ra, ta = umeyama(Cb[fr], Cv[fr]); res = np.linalg.norm((sc * (Ra @ Cb.T).T + ta) - Cv, axis=1)
log(f"[sim3] scale {sc:.5f}, BA-vs-VIO centre rms {np.sqrt((res[fr] ** 2).mean()) * 100:.2f} cm")
for nm, src in (("poses_ba.jsonl.gz", "ba"), ("poses_vio_kf.jsonl.gz", "vio")):
    with gzip.open(out / nm, "wt") as f:
        for k in range(len(kfs)):
            T = Tv[k]
            if src == "ba" and fr[k]:
                T = np.eye(4); T[:3, :3] = Ra @ R1[k].T; T[:3, 3] = sc * Ra @ Cb[k] + ta
            f.write(json.dumps(dict(type="pose", status="tracking", t_ns=int(tk[k]), frame=int(kfs[k]), T_world_cam0=T.ravel().tolist())) + "\n")
info = dict(made=time.strftime("%F %T"), source="kfba_from_dump (kf_ba.ba_lm on viba_input.npz)", n_kf=int(len(kfs)), points=int(len(D["X0"])),
            obs=int(len(D["cam"])), fixed_kf=[int(kfs[k]) for k in fixed], sim3_scale=sc, sim3_rms_cm=float(np.sqrt((res[fr] ** 2).mean()) * 100),
            runtime_s=round(time.time() - t0, 1), **{k: v for k, v in st.items() if k != "hist"}, hist=st["hist"])
(out / "ba.json").write_text(json.dumps(info, indent=1)); log(json.dumps({k: v for k, v in info.items() if k != "hist"}))
