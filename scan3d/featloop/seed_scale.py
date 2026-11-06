# featloop: BA scale k vs GT per run (camera centres, Sim3 BA->GT, k = 1/s - 1 in %, same as fl_score k_gt), CPU only
import gzip, json, sys, numpy as np
rec, runs = sys.argv[1], sys.argv[2:]
g = np.load(rec + "/imu_gt_spl/gt.npz"); t, ok, P = g["t_ns"], g["ok"], g["p_bc"]
def ume(A, B):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]); return float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum())
for r in runs:
    A, B = [], []
    for l in gzip.open(r, "rt"):
        d = json.loads(l); i = int(np.argmin(np.abs(t - d["t_ns"])))
        if abs(t[i] - d["t_ns"]) < 2e6 and ok[i]:
            A.append(np.array(d["T_world_cam0"]).reshape(4, 4)[:3, 3]); B.append(P[i])
    s = ume(np.array(A), np.array(B)); print(f"{r.split('/')[-2]:22s} n {len(A)} k {(1 / s - 1) * 100:+.2f} %")
