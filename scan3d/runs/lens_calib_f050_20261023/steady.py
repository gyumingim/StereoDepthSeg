"""Pick steady frames per tripod pose (views/<rec>.pkl → results/steady.json + per-pose views pickle).

Motion of frame i = max over shared corner ids of |c_i - c_{i-1}| (px). A frame is steady when its motion and the next
frame's motion are both < THR (0.1 px). From the steady frames we take the last continuous run (the phone was touched
at the start; heat stops cut the end), use its last ~1 s (60 frames), and pick N_PER_POSE frames evenly from it.
Poses with < MIN_CORNERS corners are dropped. Identical-looking poses (same board position, < 3 px mean corner
difference) are reported as duplicates and merged (frames split between them) so each pose has similar weight.
  ../venv_ffs/bin/python runs/lens_calib_f050_20261023/steady.py
"""
import json
import pickle
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
THR = 0.1
N_PER_POSE = 5
WIN = 60
MIN_CORNERS = 6


def motion(frames):
    m = [np.inf]
    for a, b in zip(frames[:-1], frames[1:]):
        ia, ib = a[2], b[2]
        common = np.intersect1d(ia, ib)
        if len(common) < 4:
            m.append(np.inf)
            continue
        pa = a[3][[np.where(ia == k)[0][0] for k in common]]
        pb = b[3][[np.where(ib == k)[0][0] for k in common]]
        m.append(float(np.max(np.linalg.norm(pb - pa, axis=1))))
    return np.array(m)


def main():
    recs = [r.strip() for r in open(HERE / "recs.txt") if r.strip()]
    out, poses = {}, []
    for rec in recs:
        d = pickle.load(open(HERE / "views" / f"{rec}.pkl", "rb"))
        fr = d["frames"]
        m = motion(fr)
        steady = np.zeros(len(fr), bool)
        steady[:-1] = (m[:-1] < THR) & (m[1:] < THR)
        steady[-1] = m[-1] < THR
        nc = np.array([len(f[2]) for f in fr])
        steady &= nc >= max(MIN_CORNERS, int(np.median(nc)))
        # last continuous run of steady frames
        idx = np.where(steady)[0]
        runs, run = [], []
        for k in idx:
            if run and k != run[-1] + 1:
                runs.append(run)
                run = []
            run.append(k)
        if run:
            runs.append(run)
        best = max(runs, key=len) if runs else []
        last = runs[-1] if runs else []
        use = last if len(last) >= 20 else best        # prefer the end, unless the last run is tiny
        use = use[-WIN:]
        pick = [int(use[int(round(x))]) for x in np.linspace(0, len(use) - 1, min(N_PER_POSE, len(use)))] if use else []
        fin = m[np.isfinite(m)]
        info = dict(n_frames=len(fr), board=d["board"], corners_median=int(np.median(nc)),
                    motion_p50=float(np.median(fin)) if len(fin) else None,
                    motion_p90=float(np.quantile(fin, 0.9)) if len(fin) else None,
                    steady_frames=int(steady.sum()), n_runs=len(runs), run_used=[int(use[0]), int(use[-1])] if use else None,
                    run_len=len(use), picked=pick,
                    motion_in_run_max=float(m[use[1:]].max()) if len(use) > 1 else None,
                    sharp=float(np.median([fr[k][4] for k in pick])) if pick else None,
                    sat_frac=float(np.median([fr[k][7] for k in pick])) if pick else None)
        out[rec] = info
        if pick:
            # mean corner position over the run (for duplicate check and coverage)
            ids = fr[pick[0]][2]
            mean = np.mean([fr[k][3][[np.where(fr[k][2] == i)[0][0] for i in ids]] for k in pick
                            if set(ids) <= set(fr[k][2])], 0)
            poses.append(dict(rec=rec, board=d["board"], size=d["size"], ids=ids, mean=mean,
                              views=[(fr[k][0], fr[k][2], fr[k][3]) for k in pick]))
        print(f"{rec[12:18]} {d['board'][13:16]} n={len(fr):3d} corners {info['corners_median']:2d} motion p50 "
              f"{info['motion_p50']:.3f} p90 {info['motion_p90']:.3f} steady {info['steady_frames']:3d} runs {len(runs)} "
              f"use {info['run_used']} ({len(use)}) maxmot {info['motion_in_run_max']} pick {pick}")
    # duplicates: same board, same ids, mean corner distance < 3 px
    dup = []
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            a, b = poses[i], poses[j]
            if a["board"] == b["board"] and len(a["ids"]) == len(b["ids"]) and (a["ids"] == b["ids"]).all():
                dist = float(np.linalg.norm(a["mean"] - b["mean"], axis=1).mean())
                if dist < 3:
                    dup.append((a["rec"], b["rec"], dist))
    for a, b, dist in dup:
        print(f"duplicate pose: {a} ~ {b} mean corner diff {dist:.2f} px")
    # also report nearest-pose distances for near duplicates (< 30 px)
    near = []
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            a, b = poses[i], poses[j]
            if a["board"] == b["board"] and len(a["ids"]) == len(b["ids"]) and (a["ids"] == b["ids"]).all():
                dist = float(np.linalg.norm(a["mean"] - b["mean"], axis=1).mean())
                if dist < 30:
                    near.append((a["rec"], b["rec"], round(dist, 2)))
    print("near poses (<30 px):", near)
    (HERE / "results").mkdir(exist_ok=True)
    json.dump(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), thr_px=THR, n_per_pose=N_PER_POSE, win=WIN, recs=out,
                   duplicates=dup, near=near), open(HERE / "results" / "steady.json", "w"), indent=1)
    pickle.dump(dict(poses=poses, duplicates=dup, near=near), open(HERE / "views" / "poses.pkl", "wb"))


if __name__ == "__main__":
    main()
