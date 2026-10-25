# Accuracy of ov_bridge --predict lines: the predicted pose at time t (pose-line time axis) vs the VIO pose of the same
# run at the same t (linear interpolation of position / normalised lerp of orientation between the two bracketing
# tracking pose lines = the pose the filter computes later with that frame's camera update).
# Baselines at the same t: "no-pred" = the last frame pose (state time) used as is (what the app had before),
# "const-vel" = last frame pose + v * h.
# usage: pred_acc.py RUN.jsonl [more ...]
import json, sys
import numpy as np


def qang(a, b):  # angle between Hamilton xyzw quaternions (deg)
    d = abs(float(np.dot(a, b)))
    return np.degrees(2 * np.arccos(min(1.0, d)))


def pc(a):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return "n 0"
    return f"n {len(a)} p50 {np.percentile(a, 50):.2f} p95 {np.percentile(a, 95):.2f} max {a.max():.2f}"


for path in sys.argv[1:]:
    P, Q = [], []
    for l in open(path):
        if not l.startswith("{"):
            continue
        j = json.loads(l)
        if j.get("type") == "pose" and j.get("status") == "tracking" and j.get("p"):
            P.append((j["t"], np.array(j["p"]), np.array(j["q"]), np.array(j["v"])))
        elif j.get("type") == "pred" and j.get("status") == "tracking":
            Q.append(j)
    tp = np.array([x[0] for x in P])
    rows = {"frame": [], "rate": []}
    for j in Q:
        t = j["t"]
        k = int(np.searchsorted(tp, t))
        if k == 0 or k >= len(tp) or tp[k] - tp[k - 1] > 0.1:
            continue
        f = (t - tp[k - 1]) / (tp[k] - tp[k - 1])
        p = (1 - f) * P[k - 1][1] + f * P[k][1]
        q0, q1 = P[k - 1][2], P[k][2]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = (1 - f) * q0 + f * q1
        q /= np.linalg.norm(q)
        ks = min(int(np.searchsorted(tp, j["t_state"] - 1e-7)), len(tp) - 1)
        okb = abs(tp[ks] - j["t_state"]) < 1e-6
        base = np.linalg.norm(P[ks][1] - p) * 100 if okb else np.nan
        base_o = qang(P[ks][2], q) if okb else np.nan
        cv = np.linalg.norm(P[ks][1] + P[ks][3] * j["h"] - p) * 100 if okb else np.nan
        rows[j["src"]].append((j["h"] * 1e3, np.linalg.norm(np.array(j["p"]) - p) * 100, qang(np.array(j["q"]), q), base,
                               base_o, cv, np.linalg.norm(P[k][3])))
    print(f"== {path}: pose(tracking) {len(P)} pred {len(Q)}")
    for src, r in rows.items():
        if not r:
            continue
        r = np.array(r)
        print(f"  [{src}] matched {len(r)} | horizon ms {pc(r[:, 0])} | speed m/s {pc(r[:, 6])}")
        print(f"     predicted       pos err cm {pc(r[:, 1])} | ori err deg {pc(r[:, 2])}")
        print(f"     no-pred (last)  pos err cm {pc(r[:, 3])} | ori err deg {pc(r[:, 4])}")
        print(f"     const-velocity  pos err cm {pc(r[:, 5])}")
