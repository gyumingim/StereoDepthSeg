#!/usr/bin/env python3
"""정밀 복원 (녹화 전용): SfM / 번들 조정으로 렌즈 2·5 모든 사진의 자세를 따로 구한 뒤 깊이 → TSDF.

왜
- 실시간 파이프라인은 모노 VIO(렌즈 2) 포즈를 그대로 써서 누적 오차가 남는다 → 같은 벽을 다시 보면 벽이 두 장.
- 렌즈 2·5 는 22~33 ms 어긋나 찍힌다 → '동시 촬영' 가정 없이 사진마다 그 시각의 독립 자세 (COLMAP 사진 = 독립 카메라).
- 다시 본 곳끼리 특징 매칭 (검색 = SIFT 단어 주머니 TF-IDF) → 번들 조정이 루프를 저절로 닫는다.

단계 (진행은 <out>/sfm_progress.json)
  1. 사진 풀기: 녹화 mp4 → <out>/sfm/images/<렌즈>/<frame_idx>.jpg (--step 장마다 하나), 사진마다 노출 가운데 시각 t_mid.
  2. 특징: pycolmap (CUDA) SIFT. 카메라 = 렌즈마다 하나, FULL_OPENCV (cv2 5계수 k1 k2 p1 p2 k3 그대로),
     보정값 calib_phone_pair.json 을 녹화 크기로 비례 변환 (calib_scale, 640 → 1440 = ×2.25). 내부값 고정 (기본).
  3. 매칭 쌍: 같은 렌즈 시간 이웃 (--seq-window) + 같은 시각 근처 두 렌즈 (±2.5 프레임 주기) + 검색 상위 --retrieval-k
     (시간 차 ≥ --loop-min-dt 인 것만 = 루프 후보). 매칭 + 기하 검증 = pycolmap.match_image_pairs.
  4. 지도: pycolmap 전역 SfM (GLOMAP 통합판, --mapper global, 기본 — 779 장 3 분) 또는 증분 (--mapper incremental, 17 분 넘게 걸려 기본 아님). 가장 큰 모델.
  5. 축척·중력: 단안 SfM 은 축척이 없다 → 같은 녹화의 기존 복원 timeline.json 궤적 (모노 VIO, T_world_rect0) 의
     렌즈 2 중심에 Sim(3) (RANSAC + Umeyama) → VIO 와 같은 미터·중력 좌표.
     검증: 두 렌즈 기선 — 렌즈 5 사진 j 의 중심 vs 렌즈 2 궤적을 t_j 로 보간한 중심 사이 거리 = 보정 기선 15.53 mm 여야.
     (COLMAP rig 는 '같은 순간 촬영' 가정이라 22~33 ms 어긋난 S24 렌즈엔 맞지 않고, pose prior 는 위치만 받아
      VIO 표류를 다시 끌어들이므로 쓰지 않음 → SfM 뒤 Sim(3) + 기선 검증만.) --scale baseline 이면 축척은 기선 쪽.
  6. 조밀: 렌즈 2 키프레임 i (kf_trans 1.5 cm / kf_rot 2.5°, SfM 자세 기준) + 시각이 가장 가까운 렌즈 5 사진 j.
     실제 상대자세 (--rel, 143702 키프레임 108 개의 정류 뒤 SIFT |dy| 중앙값으로 고름):
       hybrid (기본) = 회전·기선 방향은 BA (렌즈 5 사진 j 자세 그대로), 기선 길이는 보정 기선 + 렌즈 2 움직임  → 0.58 px
                       (BA 렌즈 5 중심은 ±수 mm 흔들려 기선 길이가 p10~p90 13~26 mm → 길이만 보정값 쪽).
                       기선 방향이 25° 넘게 돌거나 정류가 안 되면 gyro 로 (143702 에서 키프레임 40 %).
       gyro          = 보정 (R,T) ∘ [자이로 적분 회전 (처음 1 s 정지 구간 바이어스) + BA 렌즈 2 궤적 보간 이동]  → 0.99 px
       direct        = BA 렌즈 5 자세 그대로 (기선 길이도 BA)  → 0.58 px, 하지만 깊이 축척이 키프레임마다 흔들림
       interp        = 보정 (R,T) ∘ BA 렌즈 2 궤적 보간 (회전도 보간)  → 1.44 px (BA 사진별 회전 잡음 0.1~0.3°)
       (참고: 고정 보정만 = 2.54 px)
     --depth mono = 렌즈 2 만 (BA 자세 모션 스테레오, motion_stereo.py), fused = 두 렌즈 + 모션 합치기 — 비교용.
     --poses vio  = SfM 없이 기존 복원의 VIO 궤적으로 같은 조밀 단계 (전후 비교용).
     → stereoRectify → FFS (depth_seg.DepthSeg, scan3d 와 같은 축소 규칙 ffs_max_px, 반복 8) → 깊이
     → (3 렌즈 녹화면 --six auto: 2+6 깊이를 같은 렌즈 2 정류 격자로 옮겨 역분산 합침, depth_fuse.py)
     → (--consist auto = 2, 2+6 합칠 때는 0: 앞뒤 이웃 키프레임 2 장씩과 깊이가 어긋난 화소 버림, depth_fuse.ConsistencyFilter)
     → 복셀 지도 + TSDF (voxel_map.VoxelMap → tsdf_map.TSDFMap) → mesh.ply.
     근거 (2026-10-17, 같은 SfM 자세로 조밀 단계만, eval_planes 메시 국소 두께 cm, 평면 6 개 · seed 4 개 평균):
       --consist 2 (drop): 143702 11.5 → 10.0 · 175446 10.6 → 6.8 · 165645 12.7 → 12.0 (정점 −4~−18 %, +5 s)
       --six fuse (165645 만 3 렌즈): 12.7 → 10.8, RMS 7.0 → 6.2, 다시 본 곳 어긋남 4.2 → 3.3 cm (+28 s).
         둘 다 켜면 11.7 (더해지지 않음) → 2+6 합칠 때는 일관성 끔.
       효과 없거나 비용이 커서 기본 아님: --iters 16 (11.3 / 11.4 / 11.9, FFS +40 %), FFS 입력 키우기 (scale 0.6·0.75:
       이웃 잔차로 잰 깊이 노이즈 (m) 가 그대로 — 노이즈가 FFS 해상도가 아니라 원본 1.26 px 수준), --lr-check
       (두께 −4~−36 % 지만 남는 화소 43~96 %, FFS 2 배), --sigma-floor 1.5 cm (−3 %, 잡음 수준).
     · 모델 조각 합치기 (--merge-models 1): SfM 이 여러 모델로 갈라지면 모델마다 VIO 에 Sim(3) (merge_models).
       175446 의 빠진 17 % 는 시작 정지 2.7 s (렌즈 2 만 36 장 모델, VIO 자세 없음) 라 합칠 수 없음 — 합성 시험만 통과.
  7. 저장: <녹화>/recon_<시각>_sfm/ — scan3d 복원과 같은 파일 (summary.json, voxels*, mesh*, trajectory.json, timeline.json,
     voxels_t.bin, kf/, 평가용 kf_points.npz) → 기존 웹 리플레이에서 그대로 열림. sfm/ 에 sparse/ (COLMAP 모델) · poses.json
     (미터 자세, --from-sfm 로 다시 씀). 사진·database.db 는 --keep-images / --keep-db 일 때만 남김.

명령줄 (WSL, 서버가 복원·스캔 중이 아닐 때 — GPU 무거운 일은 한 번에 하나):
  venv_ffs/bin/python scan3d/sfm_recon.py scan3d/recordings/rec_20261016_143702 [--step 1] [--rel interp] [--pair 2+5]
"""
import argparse
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for p in (str(HERE), str(ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

PROGRESS_FILE = "sfm_progress.json"
SFM_FORMAT = "scan3d-sfm/1"


# ---------------- 작은 도구 ----------------
def make_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rot_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


class Progress:
    """진행 상태를 파일 하나에 (서버·웹이 읽음). stage, pct (0~100), msg, 시작 시각, 오류."""

    def __init__(self, path, log):
        self.path = Path(path) if path else None
        self.log = log
        self.d = dict(format="scan3d-sfm-progress/1", state="running", stage="시작", pct=0.0, msg="", started=time.time(),
                      pid=os.getpid(), error=None, out=None)

    def __call__(self, stage=None, pct=None, msg=None, **kw):
        if stage is not None:
            self.d["stage"] = stage
        if pct is not None:
            self.d["pct"] = round(float(pct), 1)
        if msg is not None:
            self.d["msg"] = msg
            self.log(f"[sfm] {stage or self.d['stage']}: {msg}")
        self.d.update(kw)
        self.d["elapsed_s"] = round(time.time() - self.d["started"], 1)
        if self.path:
            tmp = self.path.with_suffix(".part")
            try:
                tmp.write_text(json.dumps(self.d, default=str))
                tmp.replace(self.path)
            except OSError:
                pass


def keep_kf_pts(shim, t, pts_w, z, n=4000, zmax=1.5):
    """평가용 (tools/eval_planes.py --revisit): 키프레임마다 가까운 (z ≤ zmax) 월드 점 최대 n 개."""
    sel = np.nonzero((z > 0.3) & (z <= zmax))[0]
    if len(sel) > n:
        sel = np.random.default_rng(int(t * 1000)).choice(sel, n, replace=False)
    shim.kf_pts.append((float(t), np.asarray(pts_w[sel], np.float32)))


def save_kf_pts(shim, out):
    if not shim.kf_pts:
        return
    t = np.array([x[0] for x in shim.kf_pts], np.float64)
    n = np.array([len(x[1]) for x in shim.kf_pts], np.int64)
    np.savez_compressed(Path(out) / "kf_points.npz", t=t, n=n, pts=np.concatenate([x[1] for x in shim.kf_pts]))


def vio_pose_set(rec, g, info, a, log):
    """비교용 (--poses vio): 같은 녹화의 기존 복원 궤적 (모노 VIO) 으로 렌즈 2·5 프레임 자세를 만듦 (SfM 없음).
    렌즈 2 = T_world_rect0(t) · [R1|0] 보간, 렌즈 5 = 렌즈 2(t_j) · 보정⁻¹."""
    vname, vt, vT = find_vio_traj(rec.path, info["pair"], log)
    if vname is None:
        raise RuntimeError("VIO 궤적이 있는 기존 복원 없음")
    R1 = np.asarray(g["rp"]["R1"], float)
    TL = [T @ make_T(R1, np.zeros(3)) for T in vT]
    T_cal = make_T(np.asarray(g["st"]["R"], float), np.asarray(g["st"]["T"], float))
    images, poses = [], {}
    for s in sorted(rec.streams, key=lambda s: s.index):
        side = "L" if s.index == info["iL"] else "R"
        for k, r in enumerate(s.recs):
            if k % a.step:
                continue
            fidx, t_ns = int(r[0]), int(r[1])
            e = dict(name=f"{s.physical}/{fidx:06d}.jpg", lens=str(s.physical), side=side, stream=s.index, t_ns=t_ns, fidx=fidx)
            T = interp_pose(vt, TL, (t_ns - rec.t_first) * 1e-9, 0.5)
            if T is None:
                continue
            images.append(e)
            poses[e["name"]] = T if side == "L" else T @ inv_T(T_cal)
    images.sort(key=lambda e: e["t_ns"])
    log(f"[sfm] VIO 자세 {len(poses)} 개 ({vname}) — SfM 없이 비교용")
    return images, poses, dict(vio_recon=vname, scale_used="vio poses")


class Shim:
    """replay_view.write_replay_extras 가 쓰는 Scanner 일부 (vmap, replay_tl, replay_depth, traj, t0_ns, rp)."""

    def __init__(self, vmap, rp, t0_ns):
        import threading
        self.vmap, self.rp, self.t0_ns = vmap, rp, t0_ns
        self.replay_tl, self.replay_depth, self.traj = [], {}, []
        self.kf_pts = []
        self.traj_lock = threading.Lock()


# ---------------- 1. 사진 ----------------
def load_inputs(rec_dir, pair, log):
    import calib_scale
    import recording
    pair = (pair or "2+5").replace(",", "+")
    rec = recording.Recording(rec_dir, log, pair=pair.split("+"))
    phys = [str(p) for p in rec.physical[:2]]
    calib = ROOT / ("calib_phone_pair.json" if pair == "2+5" else "calib_phone_pair_2_6.json")
    g = calib_scale.load_geometry(str(calib), phys, rec.size)
    # 정류 왼쪽 = 렌즈 2 (K1), 오른쪽 = 렌즈 5 (K2). swap 이면 녹화 스트림 1 이 왼쪽
    iL, iR = (1, 0) if g["swap"] else (0, 1)
    lensL, lensR = phys[iL], phys[iR]
    return rec, g, dict(pair=pair, calib=str(calib), phys=phys, iL=iL, iR=iR, lensL=lensL, lensR=lensR)


def extract_images(rec, info, img_dir, step, prog, log):
    """mp4 → JPEG. 반환 [dict(name, lens, side ('L'|'R'), stream, t_ns, fidx)] (시각 순)."""
    import cv2
    idx_path = img_dir / "index.json"
    if idx_path.is_file():
        try:
            j = json.loads(idx_path.read_text())
            if j.get("step") == step and j.get("images"):
                log(f"[sfm] 사진 이미 있음 ({len(j['images'])} 장)")
                return j["images"]
        except (OSError, ValueError):
            pass
    out = []
    n_total = sum(len(s) for s in rec.streams)
    done = 0
    for s in sorted(rec.streams, key=lambda s: s.index):
        side = "L" if s.index == info["iL"] else "R"
        lens = str(s.physical)
        (img_dir / lens).mkdir(parents=True, exist_ok=True)
        rows = {int(r[1]): int(r[0]) for r in s.recs}
        for k, f in enumerate(s.iter_frames(rec.size, lambda m: None)):
            done += 1
            if k % step:
                continue
            fidx = rows.get(int(f.timestamp_ns), k)
            name = f"{lens}/{fidx:06d}.jpg"
            cv2.imwrite(str(img_dir / name), f.bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            out.append(dict(name=name, lens=lens, side=side, stream=s.index, t_ns=int(f.timestamp_ns), fidx=fidx))
            if done % 50 == 0:
                prog(pct=10.0 * done / max(1, n_total))
    out.sort(key=lambda e: e["t_ns"])
    idx_path.write_text(json.dumps(dict(step=step, images=out)))
    log(f"[sfm] 사진 {len(out)} 장 풀기 끝")
    return out


# ---------------- 2~4. SfM ----------------
def colmap_params(K, dist):
    """cv2 K, dist(k1 k2 p1 p2 k3) → COLMAP FULL_OPENCV params (화소 중심 규약: COLMAP 은 첫 화소 중심이 0.5)."""
    d = list(np.asarray(dist, float).reshape(-1)) + [0.0] * 8
    K = np.asarray(K, float)
    return [K[0, 0], K[1, 1], K[0, 2] + 0.5, K[1, 2] + 0.5, d[0], d[1], d[2], d[3], d[4], 0.0, 0.0, 0.0]


def extract_features(db_path, img_dir, images, g, info, a, log):
    import pycolmap
    st = g["st"]
    cams = {"L": colmap_params(st["K1"], st["dist1"]), "R": colmap_params(st["K2"], st["dist2"])}
    eo = pycolmap.FeatureExtractionOptions()
    if a.features == "aliked":
        eo.type = pycolmap.FeatureExtractorType.ALIKED_N16ROT
    else:
        eo.sift.max_num_features = a.max_features
    eo.max_image_size = max(int(x) for x in g["size"])
    for side in ("L", "R"):
        names = [e["name"] for e in images if e["side"] == side]
        ro = pycolmap.ImageReaderOptions()
        ro.camera_model = "FULL_OPENCV"
        ro.camera_params = ",".join(f"{v:.10g}" for v in cams[side])
        pycolmap.extract_features(db_path, img_dir, image_names=names, camera_mode=pycolmap.CameraMode.SINGLE,
                                  reader_options=ro, extraction_options=eo, device=pycolmap.Device.auto)
        log(f"[sfm] 특징 {side} ({len(names)} 장)")


def _pair_ids(pid):
    """COLMAP pair_id → (image_id1, image_id2) (pair_id = id1 · 2147483647 + id2)."""
    import pycolmap
    f = getattr(pycolmap, "pair_id_to_image_pair", None) or getattr(pycolmap.Database, "pair_id_to_image_pair", None)
    if f is not None:
        return tuple(f(pid))
    return int(pid) // 2147483647, int(pid) % 2147483647


def read_db(db_path):
    import pycolmap
    db = pycolmap.Database.open(str(db_path))
    try:
        ims = {im.name: im.image_id for im in db.read_all_images()}
        desc = {}
        for n, i in ims.items():
            d = db.read_descriptors(i)
            desc[n] = np.asarray(d.data if hasattr(d, "data") else d)
        tvg = {}
        try:
            pairs = db.read_two_view_geometries()
            ids, geoms = pairs if isinstance(pairs, tuple) else (None, None)
        except Exception:
            ids, geoms = None, None
        if ids is not None:
            inv = {v: k for k, v in ims.items()}
            for pid, gm in zip(ids, geoms):
                i1, i2 = pid if isinstance(pid, tuple) else _pair_ids(pid)
                tvg[(inv.get(i1), inv.get(i2))] = int(len(gm.inlier_matches))
        return ims, desc, tvg
    finally:
        db.close()


def bow_retrieval(images, desc, a, log):
    """SIFT 단어 주머니 (k-means 단어 a.bow_words, TF-IDF, 코사인) → 사진마다 시간 차 ≥ loop_min_dt 인 상위 k."""
    import torch
    t0 = time.perf_counter()
    rng = np.random.default_rng(0)
    names = [e["name"] for e in images]
    D = {n: np.asarray(desc[n], np.float32) for n in names if n in desc and len(desc[n])}
    samp = np.concatenate([d[rng.choice(len(d), min(len(d), 150), replace=False)] for d in D.values()])
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    X = torch.from_numpy(samp).to(dev)
    X = X / X.norm(dim=1, keepdim=True).clamp_min(1e-6)
    K = min(a.bow_words, len(X) // 4)
    C = X[torch.from_numpy(rng.choice(len(X), K, replace=False)).to(dev)].clone()
    for _ in range(15):                                         # 구면 k-means
        lab = (X @ C.T).argmax(1)
        Cn = torch.zeros_like(C).index_add_(0, lab, X)
        cnt = torch.bincount(lab, minlength=K).float()
        keep = cnt > 0
        C[keep] = Cn[keep] / Cn[keep].norm(dim=1, keepdim=True).clamp_min(1e-6)
    H = np.zeros((len(names), K), np.float32)
    for k, n in enumerate(names):
        if n not in D:
            continue
        d = torch.from_numpy(D[n][:2000]).to(dev)
        d = d / d.norm(dim=1, keepdim=True).clamp_min(1e-6)
        H[k] = torch.bincount((d @ C.T).argmax(1), minlength=K).float().cpu().numpy()
    df = (H > 0).sum(0)
    idf = np.log(len(names) / np.maximum(df, 1))
    W = H * idf
    W /= np.maximum(np.linalg.norm(W, axis=1, keepdims=True), 1e-9)
    S = W @ W.T
    t = np.array([e["t_ns"] for e in images], np.float64) * 1e-9
    far = np.abs(t[:, None] - t[None, :]) >= a.loop_min_dt
    S = np.where(far, S, -1.0)
    out = set()
    for i in range(len(names)):
        js = np.argsort(-S[i])[:a.retrieval_k]
        for j in js:
            if S[i, j] > 0:
                out.add(tuple(sorted((names[i], names[int(j)]))))
    log(f"[sfm] 검색 (단어 {K}) 루프 후보 쌍 {len(out)} ({time.perf_counter() - t0:.1f} s)")
    return out


def make_pairs(images, fps, a):
    names = [e["name"] for e in images]
    t = np.array([e["t_ns"] for e in images], np.float64) * 1e-9
    side = np.array([e["side"] for e in images])
    pairs = set()
    for s in ("L", "R"):
        idx = np.nonzero(side == s)[0]
        for k, i in enumerate(idx):
            for j in idx[k + 1:k + 1 + a.seq_window]:
                pairs.add(tuple(sorted((names[i], names[j]))))
    # 같은 시각 근처 두 렌즈 (±2.5 주기, 사진 간격 step 고려)
    win = 2.5 * a.step / fps
    iL, iR = np.nonzero(side == "L")[0], np.nonzero(side == "R")[0]
    tR = t[iR]
    for i in iL:
        for j in iR[np.abs(tR - t[i]) <= win]:
            pairs.add(tuple(sorted((names[i], names[j]))))
    return pairs


def run_sfm(work, img_dir, images, g, info, fps, a, prog, log):
    import pycolmap
    db_path = work / "database.db"
    sparse = work / "sparse"
    stats = {}
    t0 = time.perf_counter()
    if db_path.exists() and not a.reuse_db:
        db_path.unlink()
    if not db_path.exists():
        prog("특징", 12, "SIFT (GPU)")
        extract_features(db_path, img_dir, images, g, info, a, log)
        stats["t_features_s"] = round(time.perf_counter() - t0, 1)
        _, desc, _ = read_db(db_path)
        seq = make_pairs(images, fps, a)
        prog("매칭", 25, "검색으로 루프 후보 찾기")
        loop = bow_retrieval(images, desc, a, log) if a.retrieval_k > 0 else set()
        allp = seq | loop
        stats.update(pairs_seq=len(seq), pairs_loop_candidates=len(loop - seq), pairs_total=len(allp))
        pl = work / "pairs.txt"
        pl.write_text("\n".join(f"{x} {y}" for x, y in sorted(allp)) + "\n")
        prog("매칭", 30, f"쌍 {len(allp)} 개 매칭 + 기하 검증")
        t1 = time.perf_counter()
        mo = pycolmap.FeatureMatchingOptions()
        if a.features == "aliked":
            mo.type = pycolmap.FeatureMatcherType.ALIKED_LIGHTGLUE
        po = pycolmap.ImportedPairingOptions()
        po.match_list_path = str(pl)
        pycolmap.match_image_pairs(str(db_path), matching_options=mo, pairing_options=po, device=pycolmap.Device.auto)
        stats["t_match_s"] = round(time.perf_counter() - t1, 1)
    # 지도
    prog("지도", 45, f"{a.mapper} SfM")
    t2 = time.perf_counter()
    if sparse.exists():
        shutil.rmtree(sparse)
    sparse.mkdir(parents=True)
    if a.mapper == "global":
        op = pycolmap.GlobalPipelineOptions()
        ba = op.mapper.bundle_adjustment
        ba.refine_focal_length = a.refine_intrinsics
        ba.refine_extra_params = a.refine_intrinsics
        ba.refine_principal_point = False
        recs = pycolmap.global_mapping(str(db_path), str(img_dir), str(sparse), op)
    else:
        op = pycolmap.IncrementalPipelineOptions()
        op.ba_refine_focal_length = a.refine_intrinsics
        op.ba_refine_extra_params = a.refine_intrinsics
        op.ba_refine_principal_point = False
        op.mapper.abs_pose_refine_focal_length = False
        op.mapper.abs_pose_refine_extra_params = False
        op.ba_use_gpu = True
        recs = pycolmap.incremental_mapping(str(db_path), str(img_dir), str(sparse), op)
    stats["t_map_s"] = round(time.perf_counter() - t2, 1)
    if not recs:
        raise RuntimeError("SfM 실패: 모델 없음")
    sizes = {k: r.num_reg_images() for k, r in recs.items()}
    best = max(sizes, key=sizes.get)
    R = recs[best]
    stats.update(models=len(recs), model_sizes=sorted(sizes.values(), reverse=True), model=int(best),
                 registered=int(R.num_reg_images()), images=len(images),
                 registered_frac=round(R.num_reg_images() / max(1, len(images)), 4),
                 points3D=int(R.num_points3D()), reproj_px=round(float(R.compute_mean_reprojection_error()), 3),
                 track_len=round(float(R.compute_mean_track_length()), 2),
                 cameras={str(c): dict(model=str(cam.model), params=[round(float(v), 4) for v in cam.params])
                          for c, cam in R.cameras.items()})
    R.write(str(sparse / str(best)))
    log(f"[sfm] 모델 {len(recs)} 개, 가장 큰 것 {stats['registered']}/{len(images)} 장, 재투영 {stats['reproj_px']} px")
    stats["_recs"] = dict(recs)                  # 모델 조각 합치기 (merge_models) 용, sparse_stage 가 꺼냄
    return R, stats


def merge_models(poses_m, recs, R_best, images, g, info, rec, fps, a, log, sstats, astats):
    """SfM 이 여러 모델로 갈라졌을 때 (예: 175446 431 + 36 + 4 장) 작은 모델도 모델마다 VIO 궤적에 Sim(3) 을 따로 맞춰 합침.
    모델끼리는 공통 점이 없어 서로 직접 못 잇지만, 같은 VIO (미터·중력) 좌표로 옮기면 한 지도가 된다 (그 부분 정확도 = VIO 수준).
    받아들이는 조건: 등록 ≥ --merge-min-images, VIO 대응 ≥ 10 이고 인라이어 비율 ≥ 0.5, 축척이 VIO 에서 나옴 (중력 방향 있음)."""
    by = {e["name"]: e for e in images}
    out = dict(poses_m)
    extra = []
    for k, Rk in sorted(recs.items(), key=lambda kv: -kv[1].num_reg_images()):
        if Rk is R_best:
            continue
        n_reg = int(Rk.num_reg_images())
        ent = dict(model=int(k), registered=n_reg)
        if n_reg < a.merge_min_images:
            ent["why"] = "작음"
            extra.append(ent)
            continue
        pk = {n: T for n, T in image_poses(Rk, images).items() if n not in out}
        try:
            pm, st = align_and_check(pk, images, g, info, rec, fps, a, lambda m: None)
        except Exception as e:
            ent["why"] = f"맞춤 실패 {e!r}"[:80]
            extra.append(ent)
            continue
        ent.update({kk: st.get(kk) for kk in ("vio_matched", "vio_inliers", "vio_inlier_frac", "vio_rmse_m", "scale_used",
                                               "baseline_mm_med")})
        ok = st.get("scale_used") == "vio" and (st.get("vio_inliers") or 0) >= 10 and (st.get("vio_inlier_frac") or 0) >= 0.5
        if not ok:
            ent["why"] = "VIO 맞춤 약함"
            extra.append(ent)
            continue
        out.update(pm)
        ent["used"] = True
        extra.append(ent)
    nL = sum(1 for e in images if e["side"] == "L")
    used = [e for e in extra if e.get("used")]
    sstats["merged_models"] = extra
    sstats["registered_merged"] = len(out)
    sstats["registered_frac_merged"] = round(len(out) / max(1, len(images)), 4)
    sstats["registered_frac_L_merged"] = round(sum(1 for n in out if by[n]["side"] == "L") / max(1, nL), 4)
    log(f"[sfm] 모델 조각 합치기: {len(used)} 개 더함 → 사진 {len(poses_m)} → {len(out)}/{len(images)} 장 ({extra})")
    return out


def image_poses(R, images):
    """name → T_w_c (4x4, SfM 좌표)."""
    by = {e["name"]: e for e in images}
    out = {}
    for _, im in R.images.items():
        if not (im.has_pose if isinstance(im.has_pose, bool) else im.has_pose()):
            continue
        cfw = im.cam_from_world() if callable(im.cam_from_world) else im.cam_from_world
        T_cw = np.eye(4)
        T_cw[:3, :] = np.asarray(cfw.matrix())
        if im.name in by:
            out[im.name] = inv_T(T_cw)
    return out


def loop_pairs(R, images, db_path, a):
    """루프 쌍 = 둘 다 등록된 사진 쌍 중 시간 차 ≥ loop_min_dt 이고 검증 인라이어 ≥ 30. 모델의 대응 그래프로 셈."""
    by = {e["name"]: e for e in images}
    try:
        cg = R.compute_correspondence_graph() if hasattr(R, "compute_correspondence_graph") else None
    except Exception:
        cg = None
    n = 0
    try:
        _, _, tvg = read_db(db_path)
    except Exception:
        tvg = {}
    reg = {im.name for _, im in R.images.items()}
    for (x, y), ni in tvg.items():
        if x in by and y in by and x in reg and y in reg and ni >= 30 \
                and abs(by[x]["t_ns"] - by[y]["t_ns"]) * 1e-9 >= a.loop_min_dt:
            n += 1
    del cg
    return n, sum(1 for v in tvg.values() if v >= 30)


# ---------------- 5. 축척·중력 ----------------
def find_vio_traj(rec_dir, pair, log):
    """같은 녹화의 기존 (SfM 아닌) 복원 timeline.json → (복원 이름, t[s] 배열, T_world_rect0 (N,4,4))."""
    import replay_view
    for r in replay_view.list_recons(rec_dir):
        if "_sfm" in r["name"] or not r["timeline"]:
            continue
        if r.get("pair") and pair and r["pair"] != pair:
            continue
        tl = replay_view.read_timeline(Path(rec_dir) / r["name"])
        tr = (tl or {}).get("trajectory") or []
        if len(tr) < 10:
            continue
        A = np.asarray(tr, float)
        T = np.tile(np.eye(4), (len(A), 1, 1))
        T[:, :3, :] = A[:, 1:13].reshape(-1, 3, 4)
        log(f"[sfm] VIO 궤적: {r['name']} ({len(A)} 자세)")
        return r["name"], A[:, 0], T
    return None, None, None


def umeyama(src, dst, with_scale=True):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    C = xd.T @ xs / len(src)
    U, S, Vt = np.linalg.svd(C)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    Rm = U @ D @ Vt
    var = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(S) @ D) / var) if with_scale else 1.0
    t = mu_d - s * Rm @ mu_s
    return s, Rm, t


def sim3_ransac(src, dst, thr, iters=2000, seed=0):
    rng = np.random.default_rng(seed)
    best, best_in = None, None
    n = len(src)
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        try:
            s, Rm, t = umeyama(src[idx], dst[idx])
        except np.linalg.LinAlgError:
            continue
        if not np.isfinite(s) or s <= 0:
            continue
        err = np.linalg.norm((s * (Rm @ src.T)).T + t - dst, axis=1)
        inl = err < thr
        if best_in is None or inl.sum() > best_in.sum():
            best_in = inl
    for _ in range(3):
        s, Rm, t = umeyama(src[best_in], dst[best_in])
        err = np.linalg.norm((s * (Rm @ src.T)).T + t - dst, axis=1)
        best_in = err < thr
        best = (s, Rm, t, err)
    return best[0], best[1], best[2], best_in, best[3]


def interp_pose(ts, Ts, t, max_gap):
    """시각 순 (ts, Ts) 에서 t 의 자세 (회전 slerp, 위치 선형). 앞뒤 사진 간격이 max_gap 보다 크면 None."""
    from scipy.spatial.transform import Rotation, Slerp
    k = int(np.searchsorted(ts, t))
    if k <= 0 or k >= len(ts):
        if k == 0 and abs(ts[0] - t) < 1e-6:
            return Ts[0]
        return None
    t0, t1 = ts[k - 1], ts[k]
    if t1 - t0 > max_gap:
        return None
    u = (t - t0) / max(t1 - t0, 1e-12)
    r = Slerp([0, 1], Rotation.from_matrix(np.stack([Ts[k - 1][:3, :3], Ts[k][:3, :3]])))(u).as_matrix()
    return make_T(r, (1 - u) * Ts[k - 1][:3, 3] + u * Ts[k][:3, 3])


def align_and_check(poses, images, g, info, rec, fps, a, log):
    """SfM 자세 → 미터·중력 좌표. 반환 (poses_metric, stats)."""
    st, rp0 = g["st"], g["rp"]
    by = {e["name"]: e for e in images}
    R1 = np.asarray(rp0["R1"], float)
    Rcal, Tcal = np.asarray(st["R"], float), np.asarray(st["T"], float).reshape(3)
    B_cal = float(np.linalg.norm(Tcal))
    L = sorted([n for n in poses if by[n]["side"] == "L"], key=lambda n: by[n]["t_ns"])
    Rn = sorted([n for n in poses if by[n]["side"] == "R"], key=lambda n: by[n]["t_ns"])
    tL = np.array([(by[n]["t_ns"] - rec.t_first) * 1e-9 for n in L])
    TL = [poses[n] for n in L]
    gap = 2.6 * a.step / fps
    # 기선 (SfM 단위): 렌즈 5 중심 vs 렌즈 2 궤적을 그 시각으로 보간
    base, rerr = [], []
    for n in Rn:
        tj = (by[n]["t_ns"] - rec.t_first) * 1e-9
        Ti = interp_pose(tL, TL, tj, gap)
        if Ti is None:
            continue
        T5 = poses[n]
        base.append(float(np.linalg.norm(T5[:3, 3] - Ti[:3, 3])))
        # 보정: p_c5 = Rcal p_c2 + Tcal → R_c5_c2 = Rcal. SfM: R_c5_c2 = R_c5_w R_w_c2
        rerr.append(rot_deg(Rcal.T @ (T5[:3, :3].T @ Ti[:3, :3])))
    base, rerr = np.array(base), np.array(rerr)
    s_base = B_cal / float(np.median(base)) if len(base) else None
    stats = dict(baseline_pairs=int(len(base)), rel_rot_err_deg_med=round(float(np.median(rerr)), 4) if len(rerr) else None,
                 rel_rot_err_deg_p90=round(float(np.percentile(rerr, 90)), 4) if len(rerr) else None)
    # VIO
    vname, vt, vT = find_vio_traj(rec.path, info["pair"], log)
    s_vio = None
    S = None
    if vname is not None:
        src, dst = [], []
        for n, t in zip(L, tL):
            k = int(np.searchsorted(vt, t))
            cand = [i for i in (k - 1, k) if 0 <= i < len(vt)]
            i = min(cand, key=lambda i: abs(vt[i] - t))
            if abs(vt[i] - t) > 0.6 / fps:
                continue
            src.append(poses[n][:3, 3])
            dst.append(vT[i][:3, 3])
        src, dst = np.array(src), np.array(dst)
        if len(src) >= 10:
            s, Rm, t, inl, err = sim3_ransac(src, dst, a.sim3_thr)
            s_vio = s
            S = (s, Rm, t)
            stats.update(vio_recon=vname, vio_matched=int(len(src)), vio_inliers=int(inl.sum()),
                         vio_inlier_frac=round(float(inl.mean()), 3),
                         vio_rmse_m=round(float(np.sqrt(np.mean(err[inl] ** 2))), 4),
                         vio_err_med_m=round(float(np.median(err)), 4), vio_err_max_m=round(float(err.max()), 4))
    if s_base is not None:
        stats["scale_baseline"] = s_base
    if s_vio is not None:
        stats["scale_vio"] = s_vio
        if s_base is not None:
            stats["scale_ratio_baseline_over_vio"] = round(s_base / s_vio, 4)
    if S is None:
        if s_base is None:
            raise RuntimeError("축척을 정할 수 없음 (VIO 복원도, 두 렌즈 기선도 없음)")
        log("[sfm] VIO 궤적 없음 → 축척은 두 렌즈 기선, 중력 방향은 모름 (SfM 좌표 그대로)")
        S = (s_base, np.eye(3), np.zeros(3))
        stats["scale_used"] = "baseline (no VIO)"
    elif a.scale == "baseline" and s_base is not None:
        S = (s_base, S[1], S[2] * (s_base / S[0]))
        stats["scale_used"] = "baseline"
    else:
        stats["scale_used"] = "vio"
    s, Rm, t = S
    out = {}
    for n, T in poses.items():
        out[n] = make_T(Rm @ T[:3, :3], s * (Rm @ T[:3, 3]) + t)
    if len(base):
        bm = base * s * 1000
        stats.update(baseline_mm_med=round(float(np.median(bm)), 3), baseline_mm_p10=round(float(np.percentile(bm, 10)), 3),
                     baseline_mm_p90=round(float(np.percentile(bm, 90)), 3), baseline_cal_mm=round(B_cal * 1000, 3))
    stats["R1"] = R1.tolist()
    log(f"[sfm] 축척: VIO {s_vio} · 기선 {s_base} → 사용 {stats['scale_used']}, 기선 중앙값 {stats.get('baseline_mm_med')} mm "
        f"(보정 {B_cal * 1000:.2f}), 렌즈 상대 회전 오차 중앙값 {stats.get('rel_rot_err_deg_med')}°")
    return out, stats


# ---------------- 6. 조밀 ----------------
def select_keyframes(L, by, poses, a):
    kfs, last = [], None
    for n in L:
        T = poses[n]
        if last is not None:
            dtr = np.linalg.norm(T[:3, 3] - last[:3, 3])
            drot = rot_deg(last[:3, :3].T @ T[:3, :3])
            if dtr < a.kf_trans and drot < a.kf_rot:
                continue
        kfs.append(n)
        last = T
    return kfs


def rect_dy(bgrL, bgrR, rp, n=800):
    """정류 쌍의 SIFT 대응점 세로 어긋남 |dy| 중앙값 (px), 대응 수. 상대자세 정확도 지표."""
    import cv2
    import lib_rectify as lr
    a, b = lr.rectify_pair(bgrL, bgrR, rp)
    sift = cv2.SIFT_create(n)
    ka, da = sift.detectAndCompute(cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), None)
    kb, db = sift.detectAndCompute(cv2.cvtColor(b, cv2.COLOR_BGR2GRAY), None)
    if da is None or db is None or len(ka) < 10 or len(kb) < 10:
        return None, 0
    m = cv2.BFMatcher().knnMatch(da, db, k=2)
    good = [x[0] for x in m if len(x) == 2 and x[0].distance < 0.7 * x[1].distance]
    if len(good) < 10:
        return None, len(good)
    pa = np.float32([ka[x.queryIdx].pt for x in good])
    pb = np.float32([kb[x.trainIdx].pt for x in good])
    dy = np.abs(pa[:, 1] - pb[:, 1])
    dy = dy[dy < 8]
    return (round(float(np.median(dy)), 3) if len(dy) >= 10 else None), len(good)


BANDS = ((0.3, 1.0, "0.3-1"), (1.0, 2.0, "1-2"), (2.0, 4.0, "2-4"))


def band_frac(Z, valid):
    """깊이 유효 화소 비율 (전체 화소 대비) — 거리 구간별."""
    n = float(Z.size)
    Zv = Z[valid & np.isfinite(Z)]
    out = dict(all=round(len(Zv) / n, 4))
    for lo, hi, nm in BANDS:
        out[nm] = round(float(((Zv >= lo) & (Zv < hi)).sum()) / n, 4)
    return out


def motion_kind(tL, TL, t, win=0.5):
    """키프레임 시각 t 앞뒤 win 초 동안 렌즈 2 이동 (m)·회전 (°) → 'rot_only' (이동 < 3 cm 이고 회전 > 8°) 여부."""
    k0, k1 = int(np.searchsorted(tL, t - win)), int(np.searchsorted(tL, t + win)) - 1
    k0, k1 = max(0, k0), min(len(tL) - 1, max(k0, k1))
    tr = float(np.linalg.norm(TL[k1][:3, 3] - TL[k0][:3, 3]))
    rot = rot_deg(TL[k0][:3, :3].T @ TL[k1][:3, :3])
    return dict(move_m=round(tr, 4), rot_deg=round(rot, 2), rot_only=bool(tr < 0.03 and rot > 8.0))


def summarize_kfs(kfl):
    """키프레임별 band_frac · rot_only → 평균 (전체, 제자리 회전 구간)."""
    def mean_b(rows):
        if not rows:
            return None
        return {k: round(float(np.mean([r[k] for r in rows])), 4) for k in rows[0]}
    allb = [k["bands"] for k in kfl]
    rotb = [k["bands"] for k in kfl if k["mk"]["rot_only"]]
    return dict(n=len(kfl), bands=mean_b(allb), n_rot_only=len(rotb), bands_rot_only=mean_b(rotb))


def gyro_rel(gyro_s, bg, Rs, T_c5_c2_cal, Ti, Tj, ti_ns, tj_ns):
    """보정 (R,T) ∘ [렌즈 2 의 t_i → t_j 움직임: 회전 = 자이로 적분 (바이어스 bg), 이동 = BA 렌즈 2 궤적 보간]."""
    import motion_rect
    lo, hi = (ti_ns, tj_ns) if tj_ns >= ti_ns else (tj_ns, ti_ns)
    Rlh = motion_rect.gyro_delta_R(gyro_s, lo, hi, bg)
    if Rlh is None:
        return None
    R_Ii_Ij = Rlh if tj_ns >= ti_ns else Rlh.T
    R_cj_ci = (Rs @ R_Ii_Ij @ Rs.T).T                       # p_c = R_sensor_to_left p_I
    t = R_cj_ci @ Ti[:3, :3].T @ (Ti[:3, 3] - Tj[:3, 3])
    return T_c5_c2_cal @ make_T(R_cj_ci, t)


def mono_depth(ms, cfg, rp, t_ns, bgr, T_rect0):
    """렌즈 2 만 깊이: MotionStereo.motion_depth (BA 자세 쌍들) → combine_motion (쌍 둘이면 서로 어긋나는 화소 버림).
    fuse_keyframe 은 두 렌즈 anchor 가 없으면 검증 안 된 모션 화소를 버리므로 여기서는 BA 자세를 믿고 모두 씀."""
    import cv2
    import frames
    import motion_stereo
    W, H = (int(x) for x in rp["size"])
    rect0 = cv2.remap(bgr, *rp["mapL"], cv2.INTER_LINEAR)
    md = ms.motion_depth(t_ns, bgr, T_rect0)
    ms._last = md
    if md is None:
        Z = np.full((H, W), np.nan, np.float32)
        return dict(Z=Z, valid=np.zeros((H, W), bool), pts_rect0=np.zeros((0, 3), np.float32), rgb=np.zeros((0, 3), np.uint8),
                    cls=np.zeros(0, np.int16), rectL=rect0)
    Zm, Vm, Sm, ver = motion_stereo.combine_motion(md["pairs"], cfg)
    Vm = Vm & np.isfinite(Zm)
    s = cfg.stride
    sub = np.zeros((H, W), bool)
    sub[::s, ::s] = True
    vs, us = np.nonzero(Vm & sub)
    pts = frames.rect0_points(Zm[vs, us], rp, us.astype(np.float32), vs.astype(np.float32))
    return dict(Z=Zm, valid=Vm, pts_rect0=pts, rgb=rect0[vs, us][:, ::-1].copy(), cls=np.full(len(vs), -1, np.int16),
                rectL=rect0)


def dense_mono(rec, g, info, images, poses, out_dir, a, prog, log, align_stats):
    """렌즈 2 만: BA 자세로 같은 렌즈 과거 프레임과 모션 스테레오 (motion_stereo.MotionStereo, 두 렌즈 anchor 없음) → TSDF."""
    import calib_scale
    import depth_seg
    import lib_ffs
    import motion_stereo
    import replay_view
    import voxel_map
    import torch
    st, rp0 = g["st"], g["rp"]
    by = {e["name"]: e for e in images}
    R1n = np.asarray(rp0["R1"], float)
    L = sorted([n for n in poses if by[n]["side"] == "L"], key=lambda n: by[n]["t_ns"])
    tL = np.array([by[n]["t_ns"] * 1e-9 for n in L])
    TL = [poses[n] for n in L]
    kfs = set(select_keyframes(L, by, poses, a))
    pose_at = {by[n]["t_ns"]: (n, poses[n]) for n in L}
    ffs_scale = calib_scale.auto_ffs_scale(*g["size"], a.ffs_max_px)
    voxel_map.configure_tsdf(enabled=True)
    vmap = voxel_map.VoxelMap(a.voxel, a.min_hits)
    ds = depth_seg.DepthSeg(rp0, a.iters, ffs_scale, tuple(a.depth_range), a.stride, no_yolo=True, log=log)
    ds.warmup()
    cfg = motion_stereo.MSConfig(pose=a.mono_pose, anchor="none", n_pairs=a.mono_pairs,
                                 sigma_rot_deg_free=a.mono_sigma_rot, keep_unverified=True, z_max=4.0)
    ms = motion_stereo.MotionStereo(st, rp0, ds.model, cfg=cfg, iters=a.iters, infer=lib_ffs.infer, log=log)
    shim = Shim(vmap, rp0, rec.t_first)
    for n in L:
        shim.traj.append(((by[n]["t_ns"] - rec.t_first) * 1e-9, poses[n] @ make_T(R1n.T, np.zeros(3))))
    sL = next(s for s in rec.streams if s.index == info["iL"])
    dstats = dict(kf=len(kfs), ok=0, no_pair=0, ffs_ms=[], dy_px=[], B_mm=[], kfl=[])
    t0 = time.perf_counter()
    torch.cuda.reset_peak_memory_stats()
    done = 0
    for f in sL.iter_frames(rec.size, lambda m: None):
        tn = int(f.timestamp_ns)
        if tn not in pose_at:
            continue
        n, T_w_c2 = pose_at[tn]
        T_rect0 = T_w_c2 @ make_T(R1n.T, np.zeros(3))
        if n in kfs:
            done += 1
            out = mono_depth(ms, cfg, rp0, tn, f.bgr, T_rect0)
            md = ms._last
            t_map = (tn - rec.t_first) * 1e-9
            mk = motion_kind(tL, TL, tn * 1e-9)
            if md is None or not out["valid"].any():
                dstats["no_pair"] += 1
                dstats["kfl"].append(dict(t=t_map, bands=band_frac(out["Z"], out["valid"]), mk=mk))
            else:
                for p in md["pairs"]:
                    dstats["ffs_ms"].append(p["ms"]["ffs"])
                    dstats["B_mm"].append(p["B"] * 1000)
                    if p.get("gate"):
                        dstats["dy_px"].append(p["gate"]["dy_med"])
                B = float(np.mean([p["B"] for p in md["pairs"]]))
                rpm = dict(rp0, baseline_m=B)
                pts_w = out["pts_rect0"] @ T_rect0[:3, :3].T.astype(np.float32) + T_rect0[:3, 3].astype(np.float32)
                keep_kf_pts(shim, t_map, pts_w, out["pts_rect0"][:, 2])
                nf0 = vmap.frames
                vmap.integrate(pts_w.astype(np.float32), out["rgb"], out["cls"], np.full(len(pts_w), -1, np.int32), t_map)
                Zi = np.where(out["valid"], out["Z"], 0).astype(np.float32)
                vmap.integrate_depth(Zi, out["valid"], out["rectL"], rpm, T_rect0,
                                     ffs_scale=float(np.mean([p["scale"] for p in md["pairs"]])), t=t_map, log=log)
                dstats["ok"] += 1
                dstats["kfl"].append(dict(t=t_map, bands=band_frac(out["Z"], out["valid"]), mk=mk))
                if vmap.frames != nf0:
                    ent = dict(i=vmap.frames, t=t_map, fr=[(f.index, tn)], T=T_rect0)
                    try:
                        ent["q"] = replay_view.kf_quality(out["pts_rect0"], out["valid"], seed=tn & 0xFFFFFFFF)
                    except Exception:
                        pass
                    shim.replay_tl.append(ent)
                    try:
                        jpg = replay_view.depth_jpeg(np.nan_to_num(out["Z"]), out["valid"], 0.3, 4.0)
                        if jpg:
                            shim.replay_depth[ent["i"]] = jpg
                    except Exception:
                        pass
            if done % 5 == 0:
                prog("깊이", 60 + 35 * done / max(1, len(kfs)), f"키프레임 {done}/{len(kfs)} (렌즈 2 모션)")
        ms.add_frame(tn, f.bgr, T_rect0)
    dstats["t_dense_s"] = round(time.perf_counter() - t0, 1)
    dstats["gpu_peak_mb"] = round(torch.cuda.max_memory_allocated() / 2 ** 20, 1)
    dstats["ms_stats"] = {k: v for k, v in ms.stats.items() if k != "rejects"}
    for k in ("ffs_ms", "dy_px", "B_mm"):
        v = dstats.pop(k)
        dstats[k + "_med"] = round(float(np.median(v)), 3) if v else None
        if v:
            dstats[k + "_p90"] = round(float(np.percentile(v, 90)), 3)
            dstats[k + "_n"] = len(v)
    dstats["coverage"] = summarize_kfs(dstats.pop("kfl"))
    return vmap, shim, dstats


def dense(rec, g, info, images, poses, out_dir, a, prog, log, align_stats):
    if a.depth == "mono":
        return dense_mono(rec, g, info, images, poses, out_dir, a, prog, log, align_stats)
    import lib_rectify as lr
    import calib_scale
    import depth_seg
    import replay_view
    import voxel_map
    st, rp0 = g["st"], g["rp"]
    fps = float(g["fps"])
    by = {e["name"]: e for e in images}
    Rcal, Tcal = np.asarray(st["R"], float), np.asarray(st["T"], float).reshape(3)
    T_c5_c2_cal = make_T(Rcal, Tcal)
    R1n = np.asarray(rp0["R1"], float)
    L = sorted([n for n in poses if by[n]["side"] == "L"], key=lambda n: by[n]["t_ns"])
    Rn = sorted([n for n in poses if by[n]["side"] == "R"], key=lambda n: by[n]["t_ns"])
    tL = np.array([by[n]["t_ns"] * 1e-9 for n in L])
    TL = [poses[n] for n in L]
    tR = np.array([by[n]["t_ns"] for n in Rn], np.int64)
    kfs = select_keyframes(L, by, poses, a)
    ffs_scale = calib_scale.auto_ffs_scale(*g["size"], a.ffs_max_px)
    log(f"[sfm] 키프레임 {len(kfs)} (렌즈 2 등록 {len(L)}), FFS scale {ffs_scale}, 상대자세 {a.rel}")
    gyro_s = [(s.timestamp_ns, s.values[:3]) for s in rec.sensors if s.type == 4]
    gt = np.array([x[0] for x in gyro_s], np.int64) if gyro_s else np.zeros(0, np.int64)
    first = (gt > rec.t_first) & (gt < rec.t_first + 1_000_000_000)     # 녹화 처음 1 s 는 정지 (녹화 요령) → 자이로 바이어스
    bg = np.median(np.array([x[1] for x in gyro_s])[first], axis=0) if first.sum() > 20 else np.zeros(3)
    Rs = np.asarray(st["R_sensor_to_left"], float)
    # 필요한 프레임 (스트림, t_ns) → 키프레임 짝
    jobs = []
    gap = 2.6 * a.step / fps
    for n in kfs:
        ti = by[n]["t_ns"]
        if not len(tR):
            break
        k = int(np.argmin(np.abs(tR - ti)))
        if abs(int(tR[k]) - ti) * 1e-9 > 0.55 * a.step / fps + 1e-4:
            continue
        m = Rn[k]
        T_w_c2 = poses[n]
        Tj = interp_pose(tL, TL, by[m]["t_ns"] * 1e-9, gap)
        direct = inv_T(poses[m]) @ T_w_c2
        interp = T_c5_c2_cal @ inv_T(Tj) @ T_w_c2 if Tj is not None else None
        gyro = None
        if Tj is not None and a.rel in ("gyro", "hybrid"):
            gyro = gyro_rel(gyro_s, bg, Rs, T_c5_c2_cal, T_w_c2, Tj, by[n]["t_ns"], by[m]["t_ns"])
        if a.rel == "direct":
            alts = [("direct", direct)]
        elif a.rel == "interp":
            alts = [("interp", interp)]
        elif a.rel == "gyro":
            alts = [("gyro", gyro)]
        else:
            # hybrid: 회전·기선 방향 = BA (렌즈 5 사진 j 자세 그대로 — 정류 |dy| 가 가장 작음),
            #         기선 길이 = 보정 기선 + 렌즈 2 움직임 (BA 렌즈 5 중심은 ±수 mm 흔들려 길이가 p10~p90 13~26 mm)
            hyb = None
            if interp is not None:
                hyb = make_T(direct[:3, :3], direct[:3, 3] / max(np.linalg.norm(direct[:3, 3]), 1e-9)
                             * np.linalg.norm(interp[:3, 3]))
            alts = [("hybrid", hyb), ("gyro", gyro)]
        alts = [(nm, T) for nm, T in alts if T is not None]
        if not alts:
            continue
        jobs.append(dict(n=n, m=m, T=T_w_c2, alts=alts))
    need = {}
    for j in jobs:
        need.setdefault((by[j["n"]]["stream"], by[j["n"]]["t_ns"]), []).append(j)
        need.setdefault((by[j["m"]]["stream"], by[j["m"]]["t_ns"]), []).append(j)
    voxel_map.configure_tsdf(enabled=True, sigma_px=a.sigma_px, sigma_floor=a.sigma_floor)
    vmap = voxel_map.VoxelMap(a.voxel, a.min_hits)
    ds = depth_seg.DepthSeg(rp0, a.iters, ffs_scale, tuple(a.depth_range), a.stride, no_yolo=True, log=log)
    vmap.names = ds.names
    ds.warmup()
    import depth_fuse
    sig_px = float(voxel_map.tsdf_cfg()["sigma_px"])
    ms = None
    pose_atL = {}
    if a.depth == "fused":
        # 두 렌즈 깊이 + 렌즈 2 모션 스테레오 (BA 자세) 합치기 — motion_stereo.MotionStereo.fuse_keyframe (anchor 포함)
        import lib_ffs
        import motion_stereo
        ms = motion_stereo.MotionStereo(st, rp0, ds.model, cfg=motion_stereo.MSConfig(pose=a.mono_pose, n_pairs=a.mono_pairs),
                                        iters=a.iters, infer=lib_ffs.infer, log=log)
        pose_atL = {by[n]["t_ns"]: poses[n] @ make_T(R1n.T, np.zeros(3)) for n in L}
    six = None
    if a.six == "auto":                                 # 3 렌즈 녹화 + 2+5 쌍이면 2+6 깊이 합치기 (165645: 국소 두께 13.3 → 10.3 cm)
        a.six = "fuse" if info["pair"] == "2+5" and "6" in [str(p) for p in rec.physical_all] else "off"
    if a.six != "off":
        six = load_six(rec, info, jobs, by, a, log)
        for j in jobs:                                    # 렌즈 6 시각의 렌즈 2 자세 (2+6 상대자세의 이동 부분)
            f6 = six["frames"].get(j["n"])
            if f6 is not None:
                j["T6"] = interp_pose(tL, TL, int(f6.timestamp_ns) * 1e-9, gap)
    if a.consist < 0:
        # auto: 2 장. 단 2+6 을 합칠 때는 끔 — 165645 (seed 4 개 평균 국소 두께) six 만 10.8, six + 일관성 11.7 cm
        a.consist = 0 if six is not None else 2
    cf = None
    if a.consist > 0:
        cf = depth_fuse.ConsistencyFilter(k=a.consist_k, ds=a.consist_ds, sigma_px=sig_px, mode=a.consist_mode)
    shim = Shim(vmap, rp0, rec.t_first)
    for n in L:
        shim.traj.append(((by[n]["t_ns"] - rec.t_first) * 1e-9, poses[n] @ make_T(R1n.T, np.zeros(3))))
    cal = [dict(K=np.array(st[f"K{i}"], float), dist=np.array(st[f"dist{i}"], float), image_size=st["image_size"])
           for i in (1, 2)]
    got = {}
    done = 0
    dstats = dict(kf=len(jobs), ok=0, skipped=0, ffs_ms=[], tsdf_ms=[], dy_px=[], base_mm=[], why={}, kfl=[], used={},
                  lr_keep=[], six=[], consist=[])
    zero = dict(all=0.0, **{nm: 0.0 for _, _, nm in BANDS})
    pending, hist = [], []                                # 일관성 거르기: 앞뒤 이웃이 다 계산될 때까지 기다리는 키프레임

    def finish(e, nbs):
        """깊이 한 장 → (일관성 거르기) → 복셀·TSDF·시간 정보."""
        r, rp, T_cam = e["r"], e["rp"], e["T_cam"]
        valid, wmul = r["valid"], None
        if cf is not None:
            keep, wmul, ci = cf.check(e["view"], nbs)
            dstats["consist"].append(ci)
            valid = valid & keep
            if wmul is None and not keep.all():
                pts, rgb, cls = depth_fuse.points_from(r["Z"], valid, r["rectL"], rp, a.stride)
                r = dict(r, valid=valid, pts_rect0=pts, rgb=rgb, cls=cls)
        pts_w = (r["pts_rect0"] @ T_cam[:3, :3].T.astype(np.float32) + T_cam[:3, 3].astype(np.float32))
        t_map = e["t_map"]
        keep_kf_pts(shim, t_map, pts_w, r["pts_rect0"][:, 2])
        nf0 = vmap.frames
        vmap.integrate(pts_w.astype(np.float32), r["rgb"], r["cls"], np.full(len(pts_w), -1, np.int32), t_map)
        tsr = depth_fuse.integrate_tsdf(vmap, r["Z"], valid, r["rectL"], rp, T_cam, ffs_scale, t_map, log=log,
                                        sigma=e.get("sigma"), wmul=wmul)
        if tsr:
            dstats["tsdf_ms"].append(tsr["ms"])
        dstats["ffs_ms"].append(r["ffs_ms"])
        dstats["ok"] += 1
        if vmap.frames != nf0:
            fL, fR = e["fL"], e["fR"]
            pts_rect0 = r["pts_rect0"] @ e["R_q"].T.astype(np.float32)
            ent = dict(i=vmap.frames, t=t_map, fr=[(fL.index, fL.timestamp_ns), (fR.index, fR.timestamp_ns)], T=e["T_rect0"])
            try:
                ent["q"] = replay_view.kf_quality(pts_rect0, valid, seed=int(fL.timestamp_ns) & 0xFFFFFFFF)
            except Exception as ex:
                log(f"[sfm] 비교 숫자 실패 {ex!r}")
            shim.replay_tl.append(ent)
            try:
                jpg = replay_view.depth_jpeg(r["Z"], valid, a.depth_range[0], a.depth_range[1])
                if jpg:
                    shim.replay_depth[ent["i"]] = jpg
            except Exception:
                pass

    def drain(final=False):
        while pending and (final or len(pending) > a.consist):
            e = pending.pop(0)
            nbs = (hist[-a.consist:] + [p["view"] for p in pending[:a.consist]]) if cf is not None else []
            finish(e, nbs)
            if cf is not None:
                hist.append(e["view"])
                del hist[:-max(1, a.consist)]
            e.clear()

    t0 = time.perf_counter()
    import torch
    torch.cuda.reset_peak_memory_stats()
    for f in rec.iter_frames():
        key = (f.index, int(f.timestamp_ns))
        add = None
        if ms is not None and f.index == info["iL"] and int(f.timestamp_ns) in pose_atL:
            add = (int(f.timestamp_ns), f.bgr, pose_atL[int(f.timestamp_ns)])
        if key not in need:
            if add:
                ms.add_frame(*add)
            continue
        got[key] = f
        for j in need[key]:
            kn, km = (by[j["n"]]["stream"], by[j["n"]]["t_ns"]), (by[j["m"]]["stream"], by[j["m"]]["t_ns"])
            if j.get("done") or kn not in got or km not in got:
                continue
            j["done"] = True
            done += 1
            fL, fR = got[kn], got[km]
            mk = motion_kind(tL, TL, fL.timestamp_ns * 1e-9)
            rp, why = None, None
            for nm, rel in j["alts"]:                     # 앞 방식이 기선 검사·정류에 떨어지면 다음 방식
                B = float(np.linalg.norm(rel[:3, 3]))
                turn = float(np.degrees(np.arccos(np.clip(rel[:3, 3] @ Tcal / max(B * np.linalg.norm(Tcal), 1e-12),
                                                          -1, 1))))
                if turn > 25.0 or B < 0.6 * np.linalg.norm(Tcal):
                    why = "baseline"
                    continue
                try:
                    rp = lr.rectify_maps(cal[0], cal[1], rel[:3, :3], rel[:3, 3])
                except ValueError as e:
                    why = str(e)[:30]
                    continue
                dstats["used"][nm] = dstats["used"].get(nm, 0) + 1
                dstats["base_mm"].append(B * 1000)
                break
            if rp is None:
                dstats["kfl"].append(dict(bands=zero, mk=mk))
                dstats["skipped"] += 1
                dstats["why"][why] = dstats["why"].get(why, 0) + 1
                continue
            if a.check_dy and done % a.check_dy == 0:
                dy, _ = rect_dy(fL.bgr, fR.bgr, rp)
                if dy is not None:
                    dstats["dy_px"].append(dy)
            r = ds(fL.bgr, fR.bgr, rp=rp, scale=ffs_scale)
            if a.lr_check:
                # 좌우 일관성: 오른쪽 기준 시차를 한 번 더 (FFS 2 배) → 왼쪽 시차와 lr_tol (FFS 해상도 px) 넘게 다르면 버림
                _, rectR = lr.rectify_pair(fL.bgr, fR.bgr, rp)
                dR = depth_fuse.infer_right(ds.model, r["rectL"], rectR, a.iters, ffs_scale)
                lrm = depth_fuse.lr_check_mask(r["disp"], dR, a.lr_tol / ffs_scale)
                nv0 = max(1, int(r["valid"].sum()))
                valid = r["valid"] & lrm
                dstats["lr_keep"].append(int(valid.sum()) / nv0)
                pts, rgb, cls = depth_fuse.points_from(r["Z"], valid, r["rectL"], rp, a.stride)
                r = dict(r, valid=valid, pts_rect0=pts, rgb=rgb, cls=cls)
            R1p = np.asarray(rp["R1"], float)
            T_cam = j["T"] @ make_T(R1p.T, np.zeros(3))                  # 쌍별 정류 카메라 → 월드
            T_rect0 = j["T"] @ make_T(R1n.T, np.zeros(3))                # 기본 rect0 (웹 절두체)
            R_q = R1n @ R1p.T                                              # 쌍별 정류 점 → 기본 rect0 (비교 숫자용)
            sigma = None
            if six is not None:
                r, rp, T_cam, R_q, sigma = six_depth(six, j, r, rp, T_cam, R_q, fL, ds, ffs_scale, sig_px, a, dstats,
                                                     depth_fuse, R1n)
            if ms is not None:
                out = ms.fuse_keyframe(int(fL.timestamp_ns), fL.bgr, T_rect0, r2=r, rp_pair=rp, R_fix=R_q)
                md = ms._last
                Bm = max([p["B"] for p in md["pairs"]]) if md else 0.0
                dstats["fused_pairs"] = dstats.get("fused_pairs", 0) + (1 if md else 0)
                r = dict(r, Z=out["Z"], valid=out["valid"], pts_rect0=out["pts_rect0"], rgb=out["rgb"], cls=out["cls"],
                         rectL=out["rectL"])
                rp = dict(rp0, baseline_m=max(float(rp0["baseline_m"]), Bm))
                T_cam, R_q = T_rect0, np.eye(3)
            dstats["kfl"].append(dict(bands=band_frac(r["Z"], r["valid"]), mk=mk))
            e = dict(r=r, rp=rp, T_cam=T_cam, T_rect0=T_rect0, R_q=R_q, fL=fL, fR=fR, sigma=sigma,
                     t_map=(fL.timestamp_ns - rec.t_first) * 1e-9)
            if cf is not None:
                e["view"] = cf.view(r["Z"], r["valid"], rp, T_cam, ffs_scale, t=e["t_map"])
            pending.append(e)
            drain()
            if done % 5 == 0:
                prog("깊이", 60 + 35 * done / max(1, len(jobs)), f"키프레임 {done}/{len(jobs)}")
        if add:
            ms.add_frame(*add)
        # 메모리: 지난 프레임 버림
        for kk in [kk for kk in got if kk[1] < int(f.timestamp_ns) - 3_000_000_000]:
            del got[kk]
    drain(final=True)
    if cf is not None:
        dstats["consist_summary"] = cf.summary()
        log(f"[sfm] 일관성 거르기 {dstats['consist_summary']}")
    dstats.pop("consist")
    if dstats["lr_keep"]:
        dstats["lr_keep_med"] = round(float(np.median(dstats["lr_keep"])), 4)
    dstats.pop("lr_keep")
    if dstats["six"]:
        sx = dstats["six"]
        dstats["six_summary"] = {k: round(float(np.median([x[k] for x in sx if x.get(k) is not None])), 5)
                                 for k in ("agree_frac", "rel_bias", "conflict", "b_only", "both", "a", "b", "ffs6_ms")
                                 if any(x.get(k) is not None for x in sx)}
        dstats["six_summary"]["n"] = len(sx)
        log(f"[sfm] 2+6 {six['mode']} {dstats['six_summary']}")
    dstats.pop("six")
    dstats["t_dense_s"] = round(time.perf_counter() - t0, 1)
    dstats["gpu_peak_mb"] = round(torch.cuda.max_memory_allocated() / 2 ** 20, 1)
    dstats["coverage"] = summarize_kfs(dstats.pop("kfl"))
    for k in ("ffs_ms", "tsdf_ms", "dy_px", "base_mm"):
        v = dstats.pop(k)
        dstats[k + "_med"] = round(float(np.median(v)), 3) if v else None
        if k in ("dy_px", "base_mm") and v:
            dstats[k + "_p90"] = round(float(np.percentile(v, 90)), 3)
            dstats[k + "_n"] = len(v)
    return vmap, shim, dstats


# ---------------- 6b. 3 렌즈 녹화: 2+6 깊이 (망원 가운데) ----------------
def load_six(rec, info, jobs, by, a, log):
    """3 렌즈 녹화에서 키프레임 (렌즈 2 사진) 마다 시각이 가장 가까운 렌즈 6 사진 + 2+6 보정 기하.
    렌즈 2·6 은 1~3 ms 로 거의 동시 → 상대자세 = 보정 (R,T) ∘ [자이로 회전 + 렌즈 2 궤적 이동] (gyro_rel), 못 구하면 보정 그대로."""
    import calib_scale
    import recording
    rec6 = recording.Recording(rec.path, log, pair=["2", "6"])
    phys6 = [str(p) for p in rec6.physical[:2]]
    if "6" not in phys6:
        raise RuntimeError("렌즈 6 이 없는 녹화")
    g6 = calib_scale.load_geometry(str(ROOT / "calib_phone_pair_2_6.json"), phys6, rec6.size)
    i6 = 0 if g6["swap"] else 1
    s6 = [s for s in rec6.streams if s.index == i6][0]
    t6 = np.array(sorted(int(r[1]) for r in s6.recs), np.int64)
    want = {}
    for j in jobs:
        ti = by[j["n"]]["t_ns"]
        k = int(np.argmin(np.abs(t6 - ti)))
        if abs(int(t6[k]) - ti) <= 8_000_000:
            want.setdefault(int(t6[k]), []).append(j["n"])
    frames6 = {}
    for f in rec6.iter_frames():
        if f.index == i6 and int(f.timestamp_ns) in want:
            for n in want[int(f.timestamp_ns)]:
                frames6[n] = f
    dt = [abs(int(frames6[n].timestamp_ns) - by[n]["t_ns"]) * 1e-6 for n in frames6]
    log(f"[sfm] 렌즈 6 사진 {len(frames6)}/{len(jobs)} (렌즈 2 와 시각 차 중앙값 {float(np.median(dt)) if dt else -1:.2f} ms)")
    st6 = g6["st"]
    cal6 = [dict(K=np.array(st6[f"K{i}"], float), dist=np.array(st6[f"dist{i}"], float), image_size=st6["image_size"])
            for i in (1, 2)]
    gyro_s = [(s.timestamp_ns, s.values[:3]) for s in rec.sensors if s.type == 4]
    gt = np.array([x[0] for x in gyro_s], np.int64) if gyro_s else np.zeros(0, np.int64)
    first = (gt > rec.t_first) & (gt < rec.t_first + 1_000_000_000)
    bg = np.median(np.array([x[1] for x in gyro_s])[first], axis=0) if first.sum() > 20 else np.zeros(3)
    return dict(frames=frames6, g=g6, cal=cal6, gyro=gyro_s, bg=bg, mode=a.six,
                T_cal=make_T(np.asarray(st6["R"], float), np.asarray(st6["T"], float)),
                Rs=np.asarray(st6["R_sensor_to_left"], float))


def six_depth(six, j, r, rp, T_cam, R_q, fL, ds, ffs_scale, sig_px, a, dstats, depth_fuse, R1n):
    """키프레임 하나: 2+6 깊이 → only = 2+6 깊이만 (그 정류 그대로), fuse = 2+5 정류 격자로 옮겨 역분산 합침.
    반환 (r, rp, T_cam, R_q, sigma 지도 또는 None)."""
    import lib_rectify as lr
    f6 = six["frames"].get(j["n"])
    if f6 is None:
        return r, rp, T_cam, R_q, None
    ti, tj = int(fL.timestamp_ns), int(f6.timestamp_ns)
    rel = None
    Tj = j.get("T6")
    if Tj is not None:
        rel = gyro_rel(six["gyro"], six["bg"], six["Rs"], six["T_cal"], j["T"], Tj, ti, tj)
    if rel is None:
        rel = six["T_cal"]
    try:
        rp6 = lr.rectify_maps(six["cal"][0], six["cal"][1], rel[:3, :3], rel[:3, 3])
    except ValueError:
        return r, rp, T_cam, R_q, None
    r6 = ds(fL.bgr, f6.bgr, rp=rp6, scale=ffs_scale)
    info = dict(ffs6_ms=r6["ffs_ms"])
    if six["mode"] == "only":
        R1p = np.asarray(rp6["R1"], float)
        T6 = j["T"] @ make_T(R1p.T, np.zeros(3))
        info["b"] = int(r6["valid"].sum())
        dstats["six"].append(info)
        return r6, rp6, T6, R1n @ R1p.T, None
    Za, Va = r["Z"], r["valid"]
    Zb, Vb, _ = depth_fuse.warp_same_center(r6["Z"], r6["valid"], rp6, rp)
    Sa = depth_fuse.sigma_map(Za, Va, depth_fuse.f_eff_B(rp, ffs_scale), sig_px)
    Sb = depth_fuse.sigma_map(Zb, Vb, depth_fuse.f_eff_B(rp6, ffs_scale), a.six_sigma_px or sig_px)
    Z, V, S, fst = depth_fuse.fuse_pairs(Za, Va, Sa, Zb, Vb, Sb, k=a.six_k)
    info.update(fst)
    dstats["six"].append(info)
    pts, rgb, cls = depth_fuse.points_from(Z, V, r["rectL"], rp, a.stride)
    return dict(r, Z=Z, valid=V, pts_rect0=pts, rgb=rgb, cls=cls), rp, T_cam, R_q, S


# ---------------- 메인 ----------------
def parse_args(argv=None):
    p = argparse.ArgumentParser(description="정밀 복원 (SfM/BA + FFS + TSDF)")
    p.add_argument("rec")
    p.add_argument("--out", default=None, help="결과 폴더 (기본 <녹화>/recon_<시각>_sfm)")
    p.add_argument("--pair", default="2+5")
    p.add_argument("--step", type=int, default=1, help="사진 간격 (1 = 모든 프레임)")
    p.add_argument("--features", choices=["sift", "aliked"], default="sift")
    p.add_argument("--max-features", type=int, default=4096)
    p.add_argument("--seq-window", type=int, default=8)
    p.add_argument("--retrieval-k", type=int, default=8)
    p.add_argument("--loop-min-dt", type=float, default=2.0, help="루프 후보로 볼 최소 시간 차 (s)")
    p.add_argument("--bow-words", type=int, default=1024)
    p.add_argument("--mapper", choices=["incremental", "global"], default="global")
    p.add_argument("--refine-intrinsics", action="store_true", help="초점·왜곡을 BA 에서 다듬음 (기본 고정)")
    p.add_argument("--scale", choices=["vio", "baseline"], default="vio")
    p.add_argument("--sim3-thr", type=float, default=0.05, help="VIO Sim(3) RANSAC 인라이어 거리 (m)")
    p.add_argument("--merge-models", type=int, choices=[0, 1], default=1,
                   help="1 = SfM 이 여러 모델로 갈라지면 작은 모델도 모델마다 VIO 에 Sim(3) 맞춰 합침 (merge_models)")
    p.add_argument("--merge-min-images", type=int, default=15, help="합칠 모델의 최소 등록 사진 수")
    p.add_argument("--rel", choices=["hybrid", "direct", "interp", "gyro"], default="hybrid",
                   help="키프레임 쌍 상대자세 (143702 정류 |dy| 중앙값: direct 0.58 · hybrid 0.58 · gyro 0.99 · interp 1.44 · 고정 보정 2.54 px)")
    p.add_argument("--kf-trans", type=float, default=0.015)
    p.add_argument("--kf-rot", type=float, default=2.5)
    p.add_argument("--iters", type=int, default=8)
    p.add_argument("--ffs-max-px", type=int, default=307200)
    p.add_argument("--voxel", type=float, default=0.03)
    p.add_argument("--min-hits", type=int, default=2)
    p.add_argument("--depth-range", type=float, nargs=2, default=[0.3, 2.5])
    p.add_argument("--stride", type=int, default=3)
    p.add_argument("--check-dy", type=int, default=3, help="키프레임 N 개마다 정류 |dy| 측정 (0 = 끔)")
    p.add_argument("--depth", choices=["stereo", "mono", "fused"], default="stereo",
                   help="stereo = 렌즈 2+5 (기본), mono = 렌즈 2 만 모션 스테레오 (비교용, 꼬리 _sfmmono), "
                        "fused = 두 렌즈 + 렌즈 2 모션 합치기 (꼬리 _sfmfused)")
    p.add_argument("--mono-pose", choices=["vio", "e", "auto"], default="vio",
                   help="모션 쌍 상대자세: vio = BA 자세 그대로 (이름은 motion_stereo 규약), e = 기본행렬, auto = dy 작은 쪽")
    p.add_argument("--mono-pairs", type=int, default=2)
    p.add_argument("--mono-sigma-rot", type=float, default=0.1, help="BA 상대 회전 오차 가정 (°)")
    p.add_argument("--poses", choices=["sfm", "vio"], default="sfm",
                   help="vio = 기존 복원의 VIO 궤적으로 같은 조밀 단계 (비교용, 결과 꼬리 _sfmvio, 상대자세 gyro)")
    p.add_argument("--from-sfm", default=None, help="다른 정밀 복원 폴더의 sfm/poses.json 자세를 다시 씀 (조밀 단계만 비교용)")
    p.add_argument("--six", choices=["auto", "off", "fuse", "only"], default="auto",
                   help="3 렌즈 녹화 (2·5·6): fuse = 2+5 깊이에 2+6 깊이 (망원 가운데) 역분산 합침, only = 2+6 깊이만 (비교용), "
                        "auto = 렌즈 6 이 있고 쌍이 2+5 면 fuse")
    p.add_argument("--six-k", type=float, default=3.0, help="2+5 와 2+6 깊이가 k·σ 넘게 다르면 버림")
    p.add_argument("--six-sigma-px", type=float, default=None,
                   help="2+6 깊이 σ_d (FFS px, 기본 = 2+5 와 같은 sigma_px). 이웃 잔차 실측은 0.79~0.95 (2+5 0.52~0.61) 이지만 "
                        "0.9 로 두면 165645 국소 두께 10.8 → 12.0 cm 로 나빠짐 (2+5 깊이 몫이 커짐)")
    p.add_argument("--consist", type=int, default=-1,
                   help="여러 키프레임 일관성: 앞뒤 이웃 키프레임 수 (0 = 끔, -1 = auto: 2, 2+6 합칠 때는 0; depth_fuse.py). "
                        "143702 11.5 → 10.0 · 175446 10.6 → 6.8 · 165645 12.7 → 12.0 cm (국소 두께, seed 4 개 평균)")
    p.add_argument("--consist-mode", choices=["drop", "strict", "weight", "measure"], default="drop",
                   help="measure = 거르지 않고 이웃 잔차로 깊이 노이즈만 잼 (summary dense.consist_summary.sigma_px_by_z)")
    p.add_argument("--consist-k", type=float, default=2.5)
    p.add_argument("--consist-ds", type=int, default=4, help="일관성 비교 격자 축소 (화소)")
    p.add_argument("--lr-check", action="store_true", help="FFS 좌우 일관성 (오른쪽 기준 시차 한 번 더, FFS 2 배)")
    p.add_argument("--lr-tol", type=float, default=1.0, help="좌우 시차 허용 (FFS 입력 해상도 px)")
    p.add_argument("--sigma-px", type=float, default=None, help="TSDF 시차 노이즈 σ_d (FFS px, 기본 tsdf_map.DEFAULTS)")
    p.add_argument("--sigma-floor", type=float, default=None, help="TSDF 깊이 노이즈 바닥 (m): σ = √(바닥² + σ_d² Z⁴/(fB)²)")
    p.add_argument("--no-dense", action="store_true")
    p.add_argument("--reuse-db", action="store_true", help="sfm/database.db 가 있으면 특징·매칭 건너뜀 (시험용)")
    p.add_argument("--keep-images", action="store_true")
    p.add_argument("--keep-db", action="store_true", help="sfm/database.db 남김 (기본 지움, 0.3~0.5 GB)")
    p.add_argument("--progress", default=None, help="진행 파일 (기본 <out>/sfm_progress.json)")
    return p.parse_args(argv)


def sparse_stage(rec_dir, rec, g, info, fps, images, work, img_dir, a, prog, log, times):
    """특징·매칭·SfM·축척 → (미터 자세 {name: T_w_c}, sfm 숫자, 축척 숫자). sfm/poses.json 저장."""
    if True:
        R, sstats = run_sfm(work, img_dir, images, g, info, fps, a, prog, log)
        recs_all = sstats.pop("_recs", {})
        times.update(features_s=sstats.get("t_features_s"), match_s=sstats.get("t_match_s"), map_s=sstats.get("t_map_s"))
        poses = image_poses(R, images)
        nloop, nver = loop_pairs(R, images, work / "database.db", a)
        sstats.update(loop_pairs=nloop, verified_pairs=nver)
        by = {e["name"]: e for e in images}
        nL = sum(1 for e in images if e["side"] == "L")
        sstats["registered_L"] = sum(1 for n in poses if by[n]["side"] == "L")
        sstats["registered_R"] = sum(1 for n in poses if by[n]["side"] == "R")
        sstats["registered_frac_L"] = round(sstats["registered_L"] / max(1, nL), 4)
        sstats["registered_frac_R"] = round(sstats["registered_R"] / max(1, len(images) - nL), 4)
        prog("축척", 55, "VIO 궤적에 Sim(3) + 두 렌즈 기선 검증")
        poses_m, astats = align_and_check(poses, images, g, info, rec, fps, a, log)
        if a.merge_models and len(recs_all) > 1:
            poses_m = merge_models(poses_m, recs_all, R, images, g, info, rec, fps, a, log, sstats, astats)
        (work / "poses.json").write_text(json.dumps(dict(
            format=SFM_FORMAT, recording=rec_dir.name, t_first=rec.t_first,
            images=[dict(by[n], T_w_c=np.asarray(T).reshape(-1).round(7).tolist()) for n, T in
                    sorted(poses_m.items(), key=lambda kv: by[kv[0]]["t_ns"])]), separators=(",", ":")))
        return poses_m, sstats, astats




def main(argv=None):
    a = parse_args(argv)
    rec_dir = Path(a.rec).resolve()
    tail = "_sfm" + ("" if a.depth == "stereo" else a.depth) + ("vio" if a.poses == "vio" else "")
    out = Path(a.out) if a.out else rec_dir / time.strftime("recon_%Y%m%d-%H%M%S" + tail)
    out.mkdir(parents=True, exist_ok=True)
    work = out / "sfm"
    img_dir = work / "images"
    img_dir.mkdir(parents=True, exist_ok=True)
    logf = open(out / "sfm_log.txt", "a", encoding="utf-8")

    def log(m):
        line = time.strftime("%H:%M:%S ") + str(m)
        print(line, flush=True)
        logf.write(line + "\n")
        logf.flush()

    prog = Progress(a.progress or out / PROGRESS_FILE, log)
    prog(out=str(out), recording=rec_dir.name)
    T0 = time.perf_counter()
    try:
        rec, g, info = load_inputs(rec_dir, a.pair, log)
        fps = float(rec.fps or 15)
        g["fps"] = fps
        prog("사진", 1, f"{rec_dir.name} {rec.size[0]}x{rec.size[1]} {rec.duration_s:.1f} s · 렌즈 {info['pair']}")
        t = time.perf_counter()
        if a.poses == "vio":
            images, poses_m, astats = vio_pose_set(rec, g, info, a, log)
            sstats, times = {}, {}
            a.rel = "gyro"
        elif a.from_sfm:
            src = Path(a.from_sfm)
            pj = json.loads((src / "sfm" / "poses.json").read_text())
            (work / "poses.json").write_text(json.dumps(pj, separators=(",", ":")))
            images = [{k: v for k, v in e.items() if k != "T_w_c"} for e in pj["images"]]
            poses_m = {e["name"]: np.asarray(e["T_w_c"], float).reshape(4, 4) for e in pj["images"]}
            sj = json.loads((src / "summary.json").read_text())
            sstats, astats = sj.get("sfm", {}), sj.get("align", {})
            times = dict(sj.get("times") or {}, from_sfm=src.name)
            log(f"[sfm] 자세 {len(poses_m)} 개를 {src} 에서 다시 씀 (사진·SfM 건너뜀)")
        elif a.poses != "vio":
            images = extract_images(rec, info, img_dir, a.step, prog, log)
            times = dict(images_s=round(time.perf_counter() - t, 1))
        if not a.from_sfm and a.poses != "vio":
            poses_m, sstats, astats = sparse_stage(rec_dir, rec, g, info, fps, images, work, img_dir, a, prog, log, times)
        summary = dict(format=SFM_FORMAT, created=time.strftime("%Y-%m-%d %H:%M:%S"), recording=str(rec_dir),
                       args=vars(a), size=list(rec.size), fps=fps, pair=info["pair"], calib=info["calib"],
                       calib_note=g["note"], sfm=sstats, align=astats, times=times)
        nloop = sstats.get("loop_pairs")
        if not a.no_dense:
            prog("깊이", 60, "FFS + TSDF")
            vmap, shim, dstats = dense(rec, g, info, images, poses_m, out, a, prog, log, astats)
            summary["dense"] = dstats
            times["dense_s"] = dstats["t_dense_s"]
            prog("저장", 96, "결과 저장")
            import replay_view
            times["total_s"] = round(time.perf_counter() - T0, 1)
            summary["gpu_peak_mb_torch"] = dstats.get("gpu_peak_mb")
            cmp = replay_view.quality_summary(list(shim.replay_tl), vmap.count_shown(), int(vmap.n))
            cmp.update(pair=info["pair"], vio_reused=None, mode="sfm")
            summary["compare"] = cmp
            summary["replay"] = dict(state="완료", recording=str(rec_dir), keyframes=len(shim.replay_tl), sfm=True)
            save_kf_pts(shim, out)
            vmap.save(out, trajectory=[T for _, T in shim.traj[::max(1, len(shim.traj) // 2000)]], extra=summary)
            (out / "trajectory.json").write_text(json.dumps(
                [dict(t=t, T_world_rect0=np.asarray(T).reshape(-1).round(6).tolist()) for t, T in shim.traj]))
            tl = replay_view.write_replay_extras(out, shim, rec, complete=True, pair=info["pair"], compare=cmp)
            tl_path = out / "timeline.json"
            j = json.loads(tl_path.read_text())
            j["mode"] = "sfm"
            j["sfm"] = dict(registered_frac=sstats.get("registered_frac"), loop_pairs=nloop, reproj_px=sstats.get("reproj_px"),
                            scale_used=astats.get("scale_used"), baseline_mm_med=astats.get("baseline_mm_med"))
            tl_path.write_text(json.dumps(j, separators=(",", ":")))
            try:
                replay_view.ensure_thumbs(rec_dir, log)
            except Exception as e:
                log(f"[sfm] 카메라 화면 그림 실패 {e!r}")
            log(f"[sfm] 저장: 키프레임 {len(tl['keyframes'])} · 복셀 {tl['voxels']} → {out}")
        else:
            times["total_s"] = round(time.perf_counter() - T0, 1)
            (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
        if not a.keep_images:
            shutil.rmtree(img_dir, ignore_errors=True)
        if not a.keep_db:                                   # 특징·매칭 DB 는 녹화당 0.3~0.5 GB → 기본 지움 (sparse/ · poses.json 은 남김)
            (work / "database.db").unlink(missing_ok=True)
        prog("완료", 100, f"끝 ({times['total_s']} s)", state="done")
        return 0
    except Exception as e:
        log(f"[sfm] 오류 {e!r}")
        traceback.print_exc()
        prog(msg=f"오류: {e!r}", state="error", error=repr(e))
        return 1
    finally:
        logf.close()


if __name__ == "__main__":
    sys.exit(main())
