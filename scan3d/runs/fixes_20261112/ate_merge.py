# ATE of merged keyframe poses vs the single full-recording run (fix_u8), centres only; light analysis (desktop)
import gzip, json, sys, numpy as np
merge_dir, full_ba = sys.argv[1], sys.argv[2]
def ld(p):
    d = {}
    for l in gzip.open(p, "rt"):
        m = json.loads(l); d[int(m["frame"])] = np.array(m["T_world_cam0"]).reshape(4, 4)
    return d
mj = json.load(open(merge_dir + "/merged/" + [x for x in __import__("os").listdir(merge_dir + "/merged") if x.endswith("_kf")][0] + "/merge.json"))
off = mj["frame_offset"]
full = ld(full_ba); fk = np.array(sorted(full)); FC = np.array([full[i][:3, 3] for i in fk])
def full_at(f):
    if f in full: return full[f][:3, 3]
    j = np.searchsorted(fk, f)
    if j == 0 or j >= len(fk) or fk[j] - fk[j - 1] > 30: return None
    w = (f - fk[j - 1]) / (fk[j] - fk[j - 1]); return (1 - w) * FC[j - 1] + w * FC[j]
def umeyama(A, B, scale):
    ma, mb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((B - mb).T @ (A - ma)); D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt; s = float(np.trace(np.diag(S) @ D) / ((A - ma) ** 2).sum()) if scale else 1.0
    return s, R, mb - s * R @ ma
out = {}
for nm in ("poses_ba", "poses_vio_kf"):
    M = ld(merge_dir + f"/scan/ba/{nm}.jsonl.gz")
    rows = []
    for f, T in sorted(M.items()):
        k = max(i for i, o in enumerate(off) if f >= o); fo = f - off[k]; c = full_at(fo)
        if c is not None: rows.append((k, fo, T[:3, 3], c, fo in full))
    A = np.array([r[2] for r in rows]); B = np.array([r[3] for r in rows]); part = np.array([r[0] for r in rows])
    res = {}
    for scale in (False, True):
        s, R, t = umeyama(A, B, scale); e = np.linalg.norm((s * (R @ A.T).T + t) - B, axis=1) * 100
        res["sim3" if scale else "se3"] = dict(scale=s, rms_cm=float(np.sqrt((e ** 2).mean())), median_cm=float(np.median(e)), max_cm=float(e.max()),
                                                per_part_rms_cm=[float(np.sqrt((e[part == k] ** 2).mean())) for k in range(len(off))])
    # no re-alignment at all (both runs are in part-0 / full VIO world; part 0 = first 35 s of the same VIO)
    e0 = np.linalg.norm(A - B, axis=1) * 100
    res["raw"] = dict(rms_cm=float(np.sqrt((e0 ** 2).mean())), per_part_rms_cm=[float(np.sqrt((e0[part == k] ** 2).mean())) for k in range(len(off))])
    res["n"] = len(rows); res["n_per_part"] = [int((part == k).sum()) for k in range(len(off))]; res["n_exact_frame"] = int(sum(r[4] for r in rows))
    out[nm] = res
print(json.dumps(out, indent=1))
