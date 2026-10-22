"""Round-2 lens calibration view collection with mirror-duplicate rejection.

usage: collect2.py REC OUT.pkl [--step 3] [--wmax 0.05]

- still frames: |raw gyro (type 16)| < wmax rad/s, linearly interpolated at frames_cam5.csv t_mid_ns
- every `step`-th still frame: ArUco markers (same dict / params as aruco_scale.Board), then
  mirror rejection (docs: runs/lens_calib_20261022/MIRROR_NOTE.md):
    1. same-ID duplicates -> keep the larger marker only
    2. markers whose perimeter < 1/3 of the median marker perimeter -> drop
    3. one RANSAC homography (board marker corners -> image) for the main board; markers with any corner
       farther than max(6 px, 0.2 * median side) from it -> drop
  then ChArUco corners from the kept markers (CharucoDetector.detectBoard with given markers),
  and corners farther than max(6 px, 0.15 * square px) from the kept-marker homography -> drop.
- keeps views with >= 12 corners; stores frame index, time, gyro rate, sharpness, rough tilt.
"""
import argparse, csv, json, pickle, sys, time
from pathlib import Path
import cv2, numpy as np

S3 = Path.home() / "scanner/StereoDepthSeg/scan3d"
sys.path[:0] = [str(S3), str(S3 / "imu_learn")]
import aruco_scale
from imu_gt import read_imu

ap = argparse.ArgumentParser()
ap.add_argument("rec"); ap.add_argument("out")
ap.add_argument("--step", type=int, default=3)
ap.add_argument("--wmax", type=float, default=0.05)
ap.add_argument("--min-corners", type=int, default=12)
a = ap.parse_args()
cv2.setNumThreads(2)
rec = Path(a.rec)
board = aruco_scale.Board(aruco_scale.load_board_spec(str(S3 / "config/charuco/board_monitor_6x6.json")))
mk_obj = {int(i): np.asarray(p, np.float64)[:, :2] for i, p in zip(board.board.getIds(), board.board.getObjPoints())}
sq_m = board.square_m
dp = board.detector.getDetectorParameters()
adet = cv2.aruco.ArucoDetector(board.dictionary, dp)

rows = list(csv.DictReader(open(rec / "frames_cam5.csv")))
t = np.array([int(r["t_mid_ns"]) for r in rows])
_, gyr = read_imu(rec / "imu.csv", raw=True)
w = np.linalg.norm(np.stack([np.interp(t, gyr[:, 0], gyr[:, 1 + j]) for j in range(3)], 1), axis=1)
still_idx = np.where(w < a.wmax)[0]
want = set(still_idx[::a.step].tolist())
last = max(want) if want else -1
K0 = np.array([[1300.0, 0, 960], [0, 1300.0, 540], [0, 0, 1]])

stats = dict(frames_read=0, det_tried=0, dup_dropped=0, small_dropped=0, ransac_dropped=0, corner_dropped=0,
             frames_with_dup=0, frames_with_any_drop=0)
corner_id_count = np.zeros(board.n_corners, int)
views = []
cap = cv2.VideoCapture(str(rec / "cam5.mp4"))
i, size, t0 = 0, None, time.time()
while i <= last:
    if i not in want:
        if not cap.grab():
            break
        i += 1
        continue
    r, f = cap.read()
    if not r:
        break
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    size = g.shape[::-1]
    stats["det_tried"] += 1
    mc, mid, _ = adet.detectMarkers(g)
    i += 1
    fi = i - 1
    if mid is None or len(mid) < 4:
        continue
    mc = [np.asarray(c, np.float64).reshape(4, 2) for c in mc]
    mid = np.asarray(mid).ravel().astype(int)
    per = np.array([cv2.arcLength(c.astype(np.float32), True) for c in mc])
    keep = np.array([m in mk_obj for m in mid])
    nd = ns = nr = 0
    # 1. duplicates
    for m in set(mid[keep].tolist()):
        js = np.where((mid == m) & keep)[0]
        if len(js) > 1:
            best = js[np.argmax(per[js])]
            for j in js:
                if j != best:
                    keep[j] = False; nd += 1
    # 2. small
    med = np.median(per[keep]) if keep.any() else 0
    small = keep & (per < med / 3)
    ns = int(small.sum()); keep &= ~small
    if keep.sum() < 4:
        continue
    # 3. RANSAC homography of the main board
    js = np.where(keep)[0]
    src = np.concatenate([mk_obj[mid[j]] for j in js]); dst = np.concatenate([mc[j] for j in js])
    side = med / 4
    thr = max(6.0, 0.2 * side)
    H, inl = cv2.findHomography(src, dst, cv2.RANSAC, thr)
    if H is None:
        continue
    pr = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(pr - dst, axis=1).reshape(-1, 4).max(1)
    bad = err > thr
    nr = int(bad.sum())
    keep[js[bad]] = False
    if keep.sum() < 4:
        continue
    js = np.where(keep)[0]
    src = np.concatenate([mk_obj[mid[j]] for j in js]); dst = np.concatenate([mc[j] for j in js])
    H, _ = cv2.findHomography(src, dst, 0)
    kc = tuple(mc[j].reshape(1, 4, 2).astype(np.float32) for j in js)
    kid = mid[js].reshape(-1, 1).astype(np.int32)
    cc, cid, _, _ = board.detector.detectBoard(g, markerCorners=kc, markerIds=kid)
    if cid is None or len(cid) < a.min_corners:
        stats["dup_dropped"] += nd; stats["small_dropped"] += ns; stats["ransac_dropped"] += nr
        continue
    cc = np.asarray(cc, np.float64).reshape(-1, 2); cid = np.asarray(cid).ravel().astype(int)
    obj = board.obj[cid]
    pc = cv2.perspectiveTransform(obj[:, :2].reshape(-1, 1, 2), H).reshape(-1, 2)
    sq_px = np.sqrt(abs(np.linalg.det(H[:2, :2] / H[2, 2]))) * sq_m if H is not None else 100
    cthr = max(6.0, 0.15 * sq_px)
    ok = np.linalg.norm(pc - cc, axis=1) < cthr
    nc = int((~ok).sum())
    stats["dup_dropped"] += nd; stats["small_dropped"] += ns; stats["ransac_dropped"] += nr; stats["corner_dropped"] += nc
    stats["frames_with_dup"] += int(nd > 0); stats["frames_with_any_drop"] += int(nd + ns + nr + nc > 0)
    cc, cid, obj = cc[ok], cid[ok], obj[ok]
    if len(cid) < a.min_corners:
        continue
    corner_id_count[cid] += 1
    img = cc.astype(np.float32).reshape(-1, 1, 2)
    # rough pose with nominal K (binning only)
    okp, rv, tv = cv2.solvePnP(obj.astype(np.float32), img, K0, None, flags=cv2.SOLVEPNP_IPPE)
    R = cv2.Rodrigues(rv)[0]
    nrm = R[:, 2] * (-1 if R[2, 2] > 0 else 1)
    tilt = float(np.degrees(np.arccos(min(1.0, abs(nrm[2])))))
    dang = float(np.degrees(np.arctan2(nrm[1], nrm[0])) % 360)
    roll = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    x0, y0 = np.floor(cc.min(0)).astype(int); x1, y1 = np.ceil(cc.max(0)).astype(int)
    roi = g[max(y0, 0):y1, max(x0, 0):x1]
    sharp = float(cv2.Laplacian(roi, cv2.CV_64F).var()) if roi.size else 0.0
    views.append(dict(fi=fi, t_ns=int(t[fi]), w=float(w[fi]), obj=obj.astype(np.float32), img=img, ids=cid,
                      cx=float(cc[:, 0].mean()), cy=float(cc[:, 1].mean()), tilt=tilt, dir_ang=dang, roll=roll,
                      dist_m0=float(np.linalg.norm(tv)), sharp=sharp, n_markers=int(len(js)),
                      drops=(nd, ns, nr, nc)))
stats["frames_read"] = i
out = dict(rec=rec.name, views=views, size=size, n_frames=len(rows), n_still=int(len(still_idx)), wmax=a.wmax,
           step=a.step, stats=stats, corner_id_count=corner_id_count.tolist(),
           made=time.strftime("%Y-%m-%d %H:%M:%S"), script="runs/lens_calib_round2_20261022/collect2.py",
           gyro="raw type 16 (bias not removed)")
pickle.dump(out, open(a.out, "wb"))
print(rec.name, "frames", len(rows), "still", len(still_idx), "views", len(views), json.dumps(stats),
      f"{time.time() - t0:.0f}s", flush=True)
