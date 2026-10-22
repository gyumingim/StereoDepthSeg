"""Pose coverage check for a lens-calibration recording (ChArUco monitor board).

usage: pose_coverage.py REC_DIR OUT.json [--step 4] [--wmax 0.05]

- still frames: |raw gyro (type 16)| < wmax rad/s, interpolated at frames_cam5.csv t_mid_ns
- board detection on every `step`-th still frame (>= 12 corners kept)
- pose: solvePnP with the lens file whose focus is closest to meta focus_dpt_request
- still segments (runs of still frames, gap < 0.15 s, >= 0.3 s long) = poses; one representative per segment
Tilt direction convention (camera frame, x right, y down): direction = where the board's FAR edge is in the image.
 'up' = board top edge farther (phone looks up at the board from below), 'right' = right edge farther, etc.
"""
import csv, json, sys, time, argparse
from pathlib import Path
import cv2, numpy as np

S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import aruco_scale
from imu_gt import read_imu

DIRS = ["right", "down-right", "down", "down-left", "left", "up-left", "up", "up-right"]  # 0 deg = right, CW in image

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("out")
ap.add_argument("--step", type=int, default=4); ap.add_argument("--wmax", type=float, nargs="+", default=[0.05, 0.1])
a = ap.parse_args()
rec = Path(a.rec)
meta = json.loads((rec / "meta.json").read_text())
focus = float(meta.get("focus_dpt_request", -1))
lens_files = sorted((S3 / "config/lens").glob("hs_1920x1080_f[0-9]*.json"))
lens_path = min(lens_files, key=lambda p: abs(json.loads(p.read_text())["focus_dpt"] - focus))
L = json.loads(lens_path.read_text())
K, D = np.array(L["K"], float), np.array(L["dist"], float)

spec = aruco_scale.load_board_spec(str(S3 / "config/charuco/board_monitor_6x6.json"))
board = aruco_scale.Board(spec)
rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
t = np.array([int(r["t_mid_ns"]) for r in rows])
_, gyr = read_imu(rec / "imu.csv", raw=True)
w = np.linalg.norm(np.stack([np.interp(t, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1), axis=1)
still = w < max(a.wmax)
still_idx = np.where(still)[0]
want = set(still_idx[::a.step].tolist())

cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
dets_all, i, W, H = [], 0, 1920, 1080
t0 = time.time()
while True:
    if i > (max(want) if want else -1):
        break
    if i in want:
        r, f = cap.read()
        if not r:
            break
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        H, W = g.shape
        c, ids, _ = board.detect(g)
        if len(ids) >= 12:
            obj = board.obj[ids]
            ok, rv, tv = cv2.solvePnP(obj, c.reshape(-1, 1, 2), K, D, flags=cv2.SOLVEPNP_IPPE)
            if ok:
                ok, rv, tv = cv2.solvePnP(obj, c.reshape(-1, 1, 2), K, D, rv, tv, True, cv2.SOLVEPNP_ITERATIVE)
            if ok:
                R, _ = cv2.Rodrigues(rv)
                pc = R @ np.array([board.size_m[0] / 2, board.size_m[1] / 2, 0.0]) + tv.ravel()
                n = R[:, 2] * (-1 if R[2, 2] > 0 else 1)        # board normal pointing toward the camera
                tilt = float(np.degrees(np.arccos(min(1.0, abs(n[2])))))
                # far edge direction = image-plane component of the toward-camera normal (checked: top farther -> ny<0 -> 'up')
                ang = float(np.degrees(np.arctan2(n[1], n[0]))) % 360
                ax = R[:, 0]
                roll = float(np.degrees(np.arctan2(ax[1], ax[0])))  # board x axis angle in the image
                dets_all.append(dict(fi=i, t=float(t[i] * 1e-9), w=float(w[i]), n=int(len(ids)), tilt=tilt, dir_ang=ang,
                                 roll=roll, dist=float(np.linalg.norm(pc)), pts=c.tolist(),
                                 cx=float(c[:, 0].mean()), cy=float(c[:, 1].mean())))
    else:
        if not cap.grab():
            break
    i += 1
print(rec.name, "frames", len(rows), "still", len(still_idx), "sampled", len(want), "det>=12", len(dets_all),
      f"{time.time() - t0:.0f}s", file=sys.stderr)

# ---- segments of still frames
def summarize(wmax):
    still_idx = np.where(w < wmax)[0]
    dets = [d for d in dets_all if d["w"] < wmax]
    segs, cur = [], []
    for k in still_idx:
        if cur and t[k] - t[cur[-1]] > 0.15e9:
            segs.append(cur); cur = []
        cur.append(k)
    if cur:
        segs.append(cur)
    segs = [s for s in segs if (t[s[-1]] - t[s[0]]) >= 0.3e9]
    by_fi = {d["fi"]: d for d in dets}
    poses = []
    for s in segs:
        ds_all = [by_fi[k] for k in s if k in by_fi]
        # split a still run when the board view changes (slow drift / translation without rotation)
        groups, cur = [], []
        for d in ds_all:
            if cur and (abs(d["cx"] - cur[0]["cx"]) + abs(d["cy"] - cur[0]["cy"]) > 150
                        or abs(d["tilt"] - cur[0]["tilt"]) > 8 or abs(d["roll"] - cur[0]["roll"]) > 20):
                groups.append(cur); cur = []
            cur.append(d)
        if cur:
            groups.append(cur)
        for ds in groups:
            dur = (ds[-1]["t"] - ds[0]["t"])
            if dur < 0.25 and len(groups) > 1:
                continue
            d = ds[len(ds) // 2]
            poses.append(dict(t0=float(ds[0]["t"] - t[0] * 1e-9), dur=float(dur), n_det=len(ds),
                              **{k: d[k] for k in ("fi", "n", "tilt", "dir_ang", "roll", "dist", "cx", "cy")}))


    def dir_name(ang):
        return DIRS[int(((ang + 22.5) % 360) // 45)]


    def tbin(tl):
        return "0-15" if tl < 15 else ("15-35" if tl < 35 else ">=35")


    for p in poses:
        p["dir"] = dir_name(p["dir_ang"]); p["tbin"] = tbin(p["tilt"])
        p["portrait"] = bool(45 <= abs(p["roll"]) <= 135)

    # ---- tilt table (all still detections and poses)
    tab_frames = {d: {b: 0 for b in ("0-15", "15-35", ">=35")} for d in DIRS}
    for d in dets:
        tab_frames[dir_name(d["dir_ang"])][tbin(d["tilt"])] += 1
    tab_poses = {d: {b: 0 for b in ("0-15", "15-35", ">=35")} for d in DIRS}
    for p in poses:
        tab_poses[p["dir"]][p["tbin"]] += 1

    # ---- coverage
    allpts = np.array([q for d in dets for q in d["pts"]]) if dets else np.zeros((0, 2))
    g3 = np.zeros((3, 3), int); g32 = np.zeros((18, 32), bool)
    for x, y in allpts:
        g3[min(2, int(y / H * 3)), min(2, int(x / W * 3))] += 1
        g32[min(17, int(y / H * 18)), min(31, int(x / W * 32))] = True
    # hull-based coverage per frame (fraction of grid cells inside any board quad), closer to "board occupies"
    g32h = np.zeros((18, 32), bool)
    yy, xx = np.mgrid[0:18, 0:32]
    cell_c = np.stack([(xx + 0.5) * W / 32, (yy + 0.5) * H / 18], -1).reshape(-1, 2)
    for d in dets:
        hull = cv2.convexHull(np.array(d["pts"], np.float32))
        inside = np.array([cv2.pointPolygonTest(hull, (float(x), float(y)), False) >= 0 for x, y in cell_c])
        g32h |= inside.reshape(18, 32)
    regions = {  # (rows, cols) on the 32x18 grid; corner = 1/8 width x 1/6 height
        "top-left": (slice(0, 3), slice(0, 4)), "top-right": (slice(0, 3), slice(28, 32)),
        "bottom-left": (slice(15, 18), slice(0, 4)), "bottom-right": (slice(15, 18), slice(28, 32)),
        "bottom": (slice(15, 18), slice(0, 32)), "top": (slice(0, 3), slice(0, 32)),
        "left": (slice(0, 18), slice(0, 4)), "right": (slice(0, 18), slice(28, 32)),
    }
    reg_cov = {k: float(g32[r].mean()) for k, r in regions.items()}
    dists = [p["dist"] for p in poses]
    out = dict(
        rec=rec.name, made=time.strftime("%Y-%m-%d %H:%M:%S"), script="runs/lens_calib_20261022/pose_coverage.py",
        focus_dpt=focus, lens_file=lens_path.name, wmax=wmax, step=a.step, gyro="raw type 16 (bias not removed)",
        n_frames=len(rows), n_still=int(len(still_idx)), n_still_det12=len(dets),
        n_still_det12_est_allframes=len(dets) * a.step,
        n_poses=len(poses), poses=poses, tilt_poses=tab_poses, tilt_frames=tab_frames,
        grid3_counts=g3.tolist(), grid3_covered=(g3 > 0).tolist(),
        grid32_cov_corners=float(g32.mean()), grid32_cov_hull=float(g32h.mean()), grid32=g32.astype(int).tolist(),
        region_cov=reg_cov, n_portrait=sum(p["portrait"] for p in poses),
        dist_m=dict(min=min(dists), med=float(np.median(dists)), max=max(dists)) if dists else None,
    )
    return out


res = {f"w<{wm}": summarize(wm) for wm in a.wmax}
Path(a.out).write_text(json.dumps(res, indent=1))
for k, o in res.items():
    print(k, "poses", o["n_poses"], "cov", round(o["grid32_cov_corners"], 3), file=sys.stderr)
