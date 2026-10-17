"""복원 리플레이 자료 (시간 따라 채워지는 맵 + 그 시각의 카메라 화면).

복원(replay.py) 이 끝나면 recon_<시각>/ 에 아래를 더 쓴다 (write_replay_extras):
  timeline.json   format "scan3d-timeline/1"
                  t 는 모두 "녹화 기준 초" = (프레임 시각 − 녹화 첫 프레임 시각) (recording.Recording.t_first, 노출 가운데)
                  keyframes: [{i (1부터, voxels_t.bin 의 kf 와 같은 번호), t, t_map (맵 시각 = 첫 IMU 기준 초),
                               frames: {"2": frame_idx, "5": frame_idx} (frames_camN.csv 의 frame_idx),
                               T: T_world_rect0 4x4 행 우선 16개, n_shown: 이 키프레임까지 보이는 복셀 수, depth: 파일 이름}]
                  trajectory: [[t, r00 r01 r02 x  r10 r11 r12 y  r20 r21 r22 z], ...] (T_world_rect0 위 3행, 솎음 ≤ 6000)
                  cam: {f, cx, cy, w, h} (rect0 핀홀, 절두체용), lenses: [{lens, physical, name}], complete, voxels, voxel_m ...
                  pair: "2+5" | "2+6" (깊이 렌즈 쌍), voxels_total (표시 기준 아래 포함), compare: 비교 숫자 (quality_summary),
                  vio_reused: 다른 쌍 복원의 위치 추적(포즈)을 그대로 썼으면 그 복원 이름.
                  키프레임마다 q: {valid (깊이 유효 화소 비율), n (점 수), plane: {res_mm, inlier, n, bins: {"0-1": {res_mm, n}, ...}}}
비교용 3렌즈 녹화 (meta physical 2,5,6): 쌍마다 복원 → recon_<시각>_p25 / recon_<시각>_p26 (rec_pairs, best_recon(pair)).
  voxels_t.bin    헤더 <4sIIfff = b"S3VT", 버전 1, M, voxel_m, t_min, t_max (24 B)
                  + M × 17 B 복셀 레코드 (웹 /api/voxels 와 같은 <hhhBBBhiH, 처음 보인 시각 순 정렬)
                  + 0~3 B 채움 (4 B 정렬) + M × float32 처음 보인 시각 (녹화 기준 초) + M × int32 키프레임 번호
  kf/d_00001.jpg  키프레임 깊이 그림 (작게)
녹화 폴더에는 thumbs/ (ensure_thumbs): cam2/00000.jpg … + index.json {lenses: [{physical, dir, t: [...]}]} — 카메라 창용.
기존 voxels.npz / voxels.json (17 B 레코드) 은 그대로이고 voxels.npz 에 t_first·t_shown·kf_shown 배열이 붙는다 (scan3d-voxels/2).
"""
import bisect
import json
import re
import shutil
import struct
import threading
import time
from pathlib import Path

import numpy as np

TIMELINE_FORMAT = "scan3d-timeline/1"
THUMBS_FORMAT = "scan3d-thumbs/1"
VOXT_MAGIC, VOXT_VERSION = b"S3VT", 1
NAME_RE = re.compile(r"^[\w.\-]+$")
LENS_NAMES = {"2": "초광각", "5": "광각", "6": "망원"}
THUMB_W = 480
DEPTH_W = 320


def safe_name(n):
    n = str(n or "")
    if not NAME_RE.match(n) or n in (".", ".."):
        raise ValueError(f"이름이 이상함: {n!r}")
    return n


# ---------------- 복원 결과 찾기 ----------------
_tl_cache = {}


def read_timeline(recon_dir):
    """timeline.json (파일 시각이 같으면 캐시 — 웹 목록이 2 초마다 읽음)."""
    p = Path(recon_dir) / "timeline.json"
    try:
        mt = p.stat().st_mtime_ns
    except OSError:
        return None
    c = _tl_cache.get(str(p))
    if c and c[0] == mt:
        return c[1]
    try:
        j = json.loads(p.read_text())
    except (OSError, ValueError):
        return None
    j = j if isinstance(j, dict) and j.get("format") == TIMELINE_FORMAT else None
    _tl_cache[str(p)] = (mt, j)
    return j


PAIR_SUFFIX = {"2+5": "_p25", "2+6": "_p26"}
COMPARE_PAIRS = ("2+6", "2+5")         # 3렌즈 녹화 자동 복원 순서 (첫 쌍의 위치 추적을 둘째 쌍이 다시 씀)


def rec_physical(rec_dir):
    try:
        meta = json.loads((Path(rec_dir) / "meta.json").read_text())
    except (OSError, ValueError):
        meta = {}
    phys = meta.get("physical") or ["2", "5"]
    if isinstance(phys, str):
        phys = [p.strip() for p in phys.split(",") if p.strip()]
    return [str(p) for p in phys]


def rec_pairs(rec_dir):
    """복원할 렌즈 쌍 목록: 3렌즈(2,5,6) 녹화 = ["2+6", "2+5"], 그 밖에는 [None] (녹화한 두 렌즈 그대로)."""
    ph = rec_physical(rec_dir)
    if len(ph) > 2 and all(p in ph for p in ("2", "5", "6")):
        return list(COMPARE_PAIRS)
    return [None]


def recon_pair(name, tl=None):
    """복원 폴더의 렌즈 쌍: timeline pair → 폴더 이름 꼬리 _p25/_p26 → timeline lenses → None."""
    if tl and tl.get("pair"):
        return tl["pair"]
    for pr, suf in PAIR_SUFFIX.items():
        if str(name).endswith(suf):
            return pr
    ls = (tl or {}).get("lenses") or []
    if len(ls) >= 2:
        return "+".join(str(x.get("physical")) for x in ls[:2])
    return None


def list_recons(rec_dir):
    """[{name, timeline, complete, voxels, keyframes, created, pair}] 최근 것 먼저."""
    out = []
    d = Path(rec_dir)
    if not d.is_dir():
        return out
    for r in sorted(d.glob("recon_*"), reverse=True):
        if not r.is_dir():
            continue
        tl = read_timeline(r)
        out.append(dict(name=r.name, timeline=tl is not None, complete=bool(tl and tl.get("complete")),
                        voxels=(tl or {}).get("voxels"), keyframes=len((tl or {}).get("keyframes") or []),
                        created=(tl or {}).get("created"), pair=recon_pair(r.name, tl),
                        ready=bool(tl and tl.get("complete") and (r / "voxels_t.bin").is_file())))
    return out


def best_recon(rec_dir, pair=None):
    """리플레이할 수 있는 (시간 정보가 있고 끝까지 된) 가장 최근 복원 이름 또는 None. pair 를 주면 그 쌍의 복원만."""
    # 정밀 복원(recon_*_sfm)은 웹 [정밀 ▶] 이 이름을 직접 넘기므로, 기본 [▶ 보기]·자동 복원 판단은 일반 복원을 먼저 고른다
    # (일반 복원이 없을 때만 정밀 복원).
    sfm = None
    for r in list_recons(rec_dir):
        if r["ready"] and (pair is None or r["pair"] == pair):
            if not r["name"].endswith("_sfm"):
                return r["name"]
            sfm = sfm or r["name"]
    return sfm


def pair_recons(rec_dir):
    """3렌즈 녹화: {"2+6": 복원 이름 또는 None, "2+5": ...}. 2렌즈 녹화는 None."""
    prs = rec_pairs(rec_dir)
    if prs == [None]:
        return None
    return {p: best_recon(rec_dir, p) for p in prs}


# ---------------- 비교 숫자 (깊이 렌즈 쌍 비교) ----------------
Z_BINS = ((0.0, 1.0, "0-1"), (1.0, 2.0, "1-2"), (2.0, 99.0, "2+"))
PLANE_MAX_PTS = 6000
PLANE_ITERS = 160


def _plane_tau(z):
    """평면 안쪽으로 보는 거리 (m): 1 cm·(1 + z²) — 스테레오 깊이 오차가 거리 제곱으로 커지므로 거리마다 넓힘."""
    return 0.01 * (1.0 + z * z)


def kf_quality(pts, valid, seed=0):
    """키프레임 하나의 비교 숫자. pts = rect0 점 (N,3) m (FFS 유효 화소, stride 간격), valid = 화소 유효 마스크.
    평면: RANSAC (점 ≤ 6000, 160 번) 으로 가장 큰 평면 → 안쪽 점으로 최소제곱 다시 맞춤 (2 번) →
    안쪽 점(거리 ≤ 1 cm·(1+z²))의 평면 거리 중앙값 (mm), 거리(z) 구간별로도. 쌍끼리 같은 규칙이라 서로 비교만 의미 있음."""
    q = dict(valid=round(float(np.mean(valid)), 4) if valid is not None and np.size(valid) else None,
             n=int(len(pts)) if pts is not None else 0, plane=None)
    P = np.asarray(pts, np.float64).reshape(-1, 3) if pts is not None else np.zeros((0, 3))
    P = P[np.isfinite(P).all(1)]
    if len(P) < 200:
        return q
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), PLANE_MAX_PTS, replace=False)] if len(P) > PLANE_MAX_PTS else P
    tauS = _plane_tau(S[:, 2])
    idx = rng.integers(0, len(S), (PLANE_ITERS, 3))
    a, b, c = S[idx[:, 0]], S[idx[:, 1]], S[idx[:, 2]]
    nrm = np.cross(b - a, c - a)
    ln = np.linalg.norm(nrm, axis=1)
    ok = ln > 1e-9
    if not ok.any():
        return q
    nrm, a = nrm[ok] / ln[ok, None], a[ok]
    d = -(nrm * a).sum(1)
    dist = np.abs(S @ nrm.T + d[None, :])                       # (점, 가설)
    cnt = (dist <= tauS[:, None]).sum(0)
    k = int(np.argmax(cnt))
    n_, d_ = nrm[k], d[k]
    tau = _plane_tau(P[:, 2])
    for _ in range(2):                                          # 안쪽 점으로 다시 맞춤 (SVD)
        inl = np.abs(P @ n_ + d_) <= tau
        if inl.sum() < 50:
            return q
        Q = P[inl]
        cen = Q.mean(0)
        _, _, vt = np.linalg.svd(Q - cen, full_matrices=False)
        n_ = vt[-1]
        d_ = -float(n_ @ cen)
    r = np.abs(P @ n_ + d_)
    inl = r <= tau
    if inl.sum() < 50:
        return q
    ri, zi = r[inl], P[inl, 2]
    bins = {}
    for lo, hi, nm in Z_BINS:
        sel = (zi >= lo) & (zi < hi)
        if sel.sum() >= 30:
            bins[nm] = dict(res_mm=round(float(np.median(ri[sel])) * 1000, 2), n=int(sel.sum()))
    q["plane"] = dict(res_mm=round(float(np.median(ri)) * 1000, 2), inlier=round(float(inl.mean()), 3), n=int(inl.sum()),
                      dist_m=round(abs(d_), 3), bins=bins)
    return q


def _med(v):
    v = [x for x in v if x is not None]
    return round(float(np.median(v)), 3) if v else None


def quality_summary(kfs, voxels_shown=None, voxels_total=None):
    """키프레임 (q 가 붙은) 목록 → 한 쌍 복원의 비교 요약 (summary.json compare, timeline compare)."""
    qs = [k.get("q") for k in kfs if k.get("q")]
    pl = [q["plane"] for q in qs if q.get("plane")]
    bins = {}
    for _, _, nm in Z_BINS:
        v = [p["bins"][nm]["res_mm"] for p in pl if nm in (p.get("bins") or {})]
        if v:
            bins[nm] = dict(res_mm=_med(v), kf=len(v))
    return dict(keyframes=len(kfs), voxels_shown=voxels_shown, voxels_total=voxels_total,
                valid_med=_med([q.get("valid") for q in qs]), plane_kf=len(pl),
                plane_res_mm=_med([p["res_mm"] for p in pl]), plane_inlier=_med([p["inlier"] for p in pl]),
                plane_bins=bins,
                note="평면 오차 = 키프레임마다 가장 큰 평면(RANSAC)에서 안쪽 점의 평면 거리 중앙값 → 키프레임들의 중앙값. "
                     "깊이 유효 비율은 그 쌍의 정류 화면 기준 (2+6 은 망원 화각).")


def pair_compare(rec_dir):
    """3렌즈 녹화의 두 쌍 복원 비교. 같은 키프레임 = 같은 렌즈 2 프레임 (frames["2"]) 끼리.
    반환 {pairs: {pair: {recon, compare, vio_reused} 또는 None}, common: {n, by_pair: {pair: quality_summary}}} 또는 None."""
    pr = pair_recons(rec_dir)
    if not pr:
        return None
    out, kfm = {}, {}
    for p, rn in pr.items():
        tl = read_timeline(Path(rec_dir) / rn) if rn else None
        if not tl:
            out[p] = None
            continue
        out[p] = dict(recon=rn, compare=tl.get("compare"), vio_reused=tl.get("vio_reused"))
        kfm[p] = {k["frames"].get("2"): k for k in tl.get("keyframes") or []
                  if k.get("q") and (k.get("frames") or {}).get("2") is not None}
    common = None
    if len(kfm) >= 2:
        keys = set.intersection(*[set(m) for m in kfm.values()])
        common = dict(n=len(keys), by_pair={p: quality_summary([m[k] for k in keys]) for p, m in kfm.items()})
    return dict(pairs=out, common=common)


# ---------------- 복원 끝: 시간 정보 쓰기 ----------------
def depth_jpeg(Z, valid, zmin, zmax):
    import cv2
    from lib_phone_depth import depth_colormap
    img = depth_colormap(Z, valid, zmin, zmax)
    if img.shape[1] > DEPTH_W:
        img = cv2.resize(img, (DEPTH_W, int(round(img.shape[0] * DEPTH_W / img.shape[1]))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return buf.tobytes() if ok else None


def write_replay_extras(d, sc, rec, complete=True, pair=None, compare=None, vio_reused=None):
    """sc = Scanner (vmap, replay_tl, replay_depth, traj, t0_ns, rp, cams_info), rec = recording.Recording.
    pair = 깊이 렌즈 쌍 "2+6" 등, compare = quality_summary (없으면 여기서 계산), vio_reused = 포즈를 빌려온 복원 이름."""
    d = Path(d)
    t_off = ((sc.t0_ns or rec.t_first) - rec.t_first) * 1e-9        # 맵 시각 + t_off = 녹화 기준 초
    _, _, recs, tm = sc.vmap.delta(0, with_time=True)
    t = tm["t_shown"].astype(np.float64) + t_off
    kfv = tm["kf_shown"].astype(np.int32)
    order = np.lexsort((kfv, t))                                    # 시각 순 (같으면 키프레임 번호 순) — 웹이 이분 탐색
    recs, t, kfv = recs[order], t[order].astype(np.float32), kfv[order]
    M = len(recs)
    tmin = float(t[0]) if M else 0.0
    tmax = float(t[-1]) if M else 0.0
    body = recs.tobytes()
    pad = (-(24 + len(body))) % 4
    tmp = d / "voxels_t.bin.part"
    with open(tmp, "wb") as fo:
        fo.write(struct.pack("<4sIIfff", VOXT_MAGIC, VOXT_VERSION, M, float(sc.vmap.voxel_m), tmin, tmax))
        fo.write(body)
        fo.write(b"\0" * pad)
        fo.write(t.astype("<f4").tobytes())
        fo.write(kfv.astype("<i4").tobytes())
    tmp.replace(d / "voxels_t.bin")
    # 키프레임 목록
    rows = {s.index: {int(r[1]): int(r[0]) for r in s.recs} for s in rec.streams}
    phys = {s.index: str(s.physical) for s in rec.streams}
    kdir = d / "kf"
    kfs = []
    depth = dict(getattr(sc, "replay_depth", {}) or {})
    for k in list(getattr(sc, "replay_tl", []) or []):
        frames = {}
        for si, t_ns in k["fr"]:
            if si in rows:
                frames[phys[si]] = rows[si].get(int(t_ns))
        e = dict(i=int(k["i"]), t=round(float(k["t"]) + t_off, 4), t_map=round(float(k["t"]), 4), frames=frames,
                 T=np.asarray(k["T"], float).reshape(-1).round(5).tolist(),
                 n_shown=int(np.count_nonzero(kfv <= int(k["i"]))))
        if k.get("q"):
            e["q"] = k["q"]
        jpg = depth.get(int(k["i"]))
        if jpg:
            kdir.mkdir(exist_ok=True)
            fn = f"d_{int(k['i']):05d}.jpg"
            (kdir / fn).write_bytes(jpg)
            e["depth"] = fn
        kfs.append(e)
    with sc.traj_lock:
        tr = list(sc.traj)
    step = max(1, len(tr) // 6000)
    traj = [[round(tt + t_off, 4)] + np.asarray(T, float)[:3, :].reshape(-1).round(5).tolist() for tt, T in tr[::step]]
    W, H = sc.rp["size"]
    tl = dict(format=TIMELINE_FORMAT, created=time.strftime("%Y-%m-%d %H:%M:%S"), complete=bool(complete),
              recording=rec.path.name, recon=d.name, duration_s=round(rec.duration_s, 3),
              t_ref="녹화 기준 초 (첫 프레임 노출 가운데 = 0)", t_offset_s=round(t_off, 6),
              voxels=M, voxel_m=float(sc.vmap.voxel_m), min_hits=int(sc.vmap.min_hits), t_min=tmin, t_max=tmax,
              voxels_file="voxels_t.bin", voxels_format=f"S3VT/{VOXT_VERSION}",
              cam=dict(f=float(sc.rp["f"]), cx=float(sc.rp["cx"]), cy=float(sc.rp["cy"]), w=int(W), h=int(H)),
              lenses=[dict(lens=i, physical=str(s.physical), name=LENS_NAMES.get(str(s.physical), f"렌즈 {s.physical}"))
                      for i, s in enumerate(sorted(rec.streams, key=lambda s: s.index))],
              keyframes=kfs, trajectory=traj,
              pair=pair or "+".join(str(s.physical) for s in sorted(rec.streams, key=lambda s: s.index)[:2]),
              rec_lenses=list(getattr(rec, "physical_all", rec.physical)), voxels_total=int(sc.vmap.n),
              compare=compare or quality_summary(kfs, M, int(sc.vmap.n)), vio_reused=vio_reused)
    (d / "timeline.json").write_text(json.dumps(tl, separators=(",", ":")))
    return tl


# ---------------- 녹화 프레임 그림 (thumbs) ----------------
_thumb_locks = {}
_thumb_glock = threading.Lock()
_idx_cache = {}


def thumbs_ready(rec_dir):
    return (Path(rec_dir) / "thumbs" / "index.json").is_file()


def ensure_thumbs(rec_dir, log=print):
    """녹화 영상 두 개를 한 번 풀어 thumbs/camN/NNNNN.jpg + index.json 을 만든다 (있으면 그대로). 반환 index dict 또는 None."""
    import cv2
    import recording
    d = Path(rec_dir)
    out = d / "thumbs"
    if (out / "index.json").is_file():
        return load_thumb_index(d)
    with _thumb_glock:
        lk = _thumb_locks.setdefault(str(d), threading.Lock())
    with lk:
        if (out / "index.json").is_file():
            return load_thumb_index(d)
        t_start = time.monotonic()
        rec = recording.Recording(d, lambda m: None, pair="all")    # 3렌즈 녹화면 세 렌즈 모두
        tmp = d / ".thumbs.part"
        if tmp.exists():
            shutil.rmtree(tmp)
        tmp.mkdir()
        lenses = []
        for s in sorted(rec.streams, key=lambda s: s.index):
            sub = f"cam{s.physical}"
            (tmp / sub).mkdir()
            ts = []
            for k, f in enumerate(s.iter_frames(rec.size, lambda m: None)):
                img = f.bgr
                if img.shape[1] > THUMB_W:
                    img = cv2.resize(img, (THUMB_W, int(round(img.shape[0] * THUMB_W / img.shape[1]))),
                                     interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok:
                    continue
                (tmp / sub / f"{len(ts):05d}.jpg").write_bytes(buf.tobytes())
                ts.append(round((f.timestamp_ns - rec.t_first) * 1e-9, 4))
            lenses.append(dict(lens=s.index, physical=str(s.physical),
                               name=LENS_NAMES.get(str(s.physical), f"렌즈 {s.physical}"), dir=sub, t=ts))
        idx = dict(format=THUMBS_FORMAT, created=time.strftime("%Y-%m-%d %H:%M:%S"), width=THUMB_W,
                   duration_s=round(rec.duration_s, 3), lenses=lenses)
        (tmp / "index.json").write_text(json.dumps(idx, separators=(",", ":")))
        if out.exists():
            shutil.rmtree(out)
        tmp.rename(out)
        log(f"[replay] 카메라 화면 그림 {sum(len(x['t']) for x in lenses)} 장 ({time.monotonic() - t_start:.1f} s) → {out}")
        return idx


def load_thumb_index(rec_dir):
    p = Path(rec_dir) / "thumbs" / "index.json"
    try:
        mt = p.stat().st_mtime
    except OSError:
        return None
    c = _idx_cache.get(str(p))
    if c and c[0] == mt:
        return c[1]
    try:
        j = json.loads(p.read_text())
    except (OSError, ValueError):
        return None
    _idx_cache[str(p)] = (mt, j)
    return j


def thumb_at(rec_dir, lens, t, phys=None):
    """lens 0|1|2 (녹화 렌즈 순서: 0 = 렌즈 2 초광각, 1 = 렌즈 5 광각, 3렌즈 녹화면 2 = 렌즈 6 망원), t = 녹화 기준 초.
    phys ("2"|"5"|"6") 를 주면 그 물리 렌즈 (리플레이 중인 쌍의 렌즈). → (jpeg bytes, 프레임 시각) 또는 (None, None)."""
    idx = load_thumb_index(rec_dir)
    if not idx:
        return None, None
    ls = idx.get("lenses") or []
    if not ls:
        return None, None
    L = next((x for x in ls if phys is not None and str(x.get("physical")) == str(phys)), None)
    if L is None:
        L = ls[min(max(0, int(lens)), len(ls) - 1)]
    ts = L.get("t") or []
    if not ts:
        return None, None
    k = bisect.bisect_right(ts, float(t)) - 1
    k = min(max(0, k), len(ts) - 1)
    try:
        return (Path(rec_dir) / "thumbs" / L["dir"] / f"{k:05d}.jpg").read_bytes(), ts[k]
    except OSError:
        return None, None
