"""Shared helpers for round-2 lens calibration (selection, coverage)."""
import pickle
from pathlib import Path
import numpy as np, cv2

D = Path(__file__).resolve().parent
S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
SIZE = (1920, 1080)
FOCUS = {"2.22": ["162218", "163644", "171455"], "1.00": ["162754", "172351"],
         "3.33": ["163008", "173113", "173834"], "0.37": ["163228", "174512"]}
ROUND = {"162218": 1, "163644": 1, "171455": 2, "162754": 1, "172351": 2, "163008": 1, "173113": 2, "173834": 2,
         "163228": 1, "174512": 2}
REC2F = {r: f for f, rs in FOCUS.items() for r in rs}
DIRS4 = ["right", "down", "left", "up"]   # far edge direction of the board in the image


def load_views(rec, wmax=0.05, sharp_frac=0.5, stand_w=0.015, thin_s=0.1):
    """Usable views of one recording + filter counts."""
    P = pickle.load(open(D / "views" / f"{rec}.pkl", "rb"))
    V = sorted(P["views"], key=lambda v: v["fi"])
    cnt = dict(n_frames=P["n_frames"], n_still=P["n_still"], n_detected_ge12=len(V), step=P["step"],
               mirror_dup_markers_dropped=P["stats"]["dup_dropped"], mirror_small_dropped=P["stats"]["small_dropped"],
               ransac_markers_dropped=P["stats"]["ransac_dropped"], corners_dropped=P["stats"]["corner_dropped"])
    if not V:
        return [], cnt
    t = np.array([v["t_ns"] for v in V]) * 1e-9
    w = np.array([v["w"] for v in V])
    # phone resting on a stand: every detected still view within +-0.5 s has |w| < stand_w and the run spans >= 1 s
    stand = np.zeros(len(V), bool)
    for k in range(len(V)):
        m = np.abs(t - t[k]) <= 0.5
        stand[k] = (w[m] < stand_w).all() and (t[m].max() - t[m].min()) > 0.9
    sh = np.array([v["sharp"] for v in V])
    smin = sharp_frac * float(np.median(sh[~stand])) if (~stand).any() else 0
    ok = (w < wmax) & ~stand & (sh >= smin)
    cnt.update(n_stand=int(stand.sum()), sharp_min=round(smin, 1), n_blurry=int(((sh < smin) & ~stand).sum()),
               n_w_ok=int((w < wmax).sum()))
    U = [v for v, o in zip(V, ok) if o]
    # thin: at most one view per thin_s seconds (neighbouring frames are near-identical)
    out, last = [], -1e9
    for v in U:
        if v["t_ns"] * 1e-9 - last >= thin_s:
            out.append(v); last = v["t_ns"] * 1e-9
    for v in out:
        v["rec"] = rec
        v["block"] = f"{rec}_{v['fi'] // 240}"
    cnt.update(n_usable=len(U), n_after_thin=len(out))
    return out, cnt


def bin_key(v):
    W, H = SIZE
    gx, gy = min(int(v["cx"] / W * 4), 3), min(int(v["cy"] / H * 3), 2)
    tl = v["tilt"]
    tb = 0 if tl < 15 else 1 if tl < 30 else 2 if tl < 45 else 3
    dr = -1 if tb == 0 else int(((v["dir_ang"] + 45) % 360) // 90)
    return (gx, gy, tb, dr)


def spread(views, n, seed=0):
    """Round-robin over (4x3 image cell, tilt bin, far-edge direction) bins -> at most n views (indices)."""
    rng = np.random.default_rng(seed)
    bins = {}
    for k, v in enumerate(views):
        bins.setdefault(bin_key(v), []).append(k)
    lists = [list(rng.permutation(b)) for _, b in sorted(bins.items())]
    pick = []
    while len(pick) < n and any(lists):
        for b in lists:
            if b and len(pick) < n:
                pick.append(int(b.pop()))
    return sorted(pick), len(bins)


def coverage(views, gx=32, gy=18):
    W, H = SIZE
    occ = np.zeros((gy, gx), bool)
    if not views:
        return 0.0, occ
    pts = np.concatenate([v["img"].reshape(-1, 2) for v in views])
    occ[np.clip((pts[:, 1] / H * gy).astype(int), 0, gy - 1), np.clip((pts[:, 0] / W * gx).astype(int), 0, gx - 1)] = True
    return float(occ.mean()), occ


def tilt_table(views):
    tab = {}
    for v in views:
        tl = v["tilt"]
        tb = "0-15" if tl < 15 else "15-30" if tl < 30 else "30-45" if tl < 45 else ">=45"
        dr = "front" if tl < 15 else DIRS4[int(((v["dir_ang"] + 45) % 360) // 90)]
        tab[f"{dr}:{tb}"] = tab.get(f"{dr}:{tb}", 0) + 1
    return tab


def cvtup(v):
    return (v["obj"], v["img"], v["cx"], v["cy"], v["tilt"])
