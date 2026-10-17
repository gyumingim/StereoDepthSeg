"""저장 지도 (persistent map, 2026-10-17): 복원·실시간 결과를 '지도' 로 저장 → 다음 세션이 그 지도 기준으로 위치를 찾고
(재위치 reloc.py 의 PnP + 두 키프레임 합의) 이어 쌓거나 (append) 위치만 추정 (localize).

폴더 scan3d/maps/<이름>/ (형식 scan3d-map/1):
  map.json            메타: 이름, 만든 시각, 출처 (녹화·복원·실시간), 좌표계, 숫자 (복셀·TSDF 블록·기준 키프레임), 설정
  voxels_state.npz    VoxelMap 전체 상태 (VoxelMap.save_state — 표시 기준 아래 복셀까지)
  tsdf.npz            TSDF 블록 (TSDFMap.save_npz)
  free.npz            관측된 빈 칸 (esdf_map.FreeSpace, 있으면)
  reloc_db.npz        재위치 기준 키프레임: 카메라 중심·자세, SIFT 기술자, 특징점 3D (지도 좌표), 확인용 점 표본
  trajectory.json     만든 세션의 궤적 (지도 좌표)
  mesh.ply            메시 (보기용) · esdf.npz · esdf2d.png · esdf.json (esdf_map.EsdfMap.save)
좌표계: 지도 = 처음 만든 세션의 OpenVINS 월드 (중력 정렬 z 위, 미터, 원점 = 그 세션 VIO 시작 위치). 지도에서 이어 쌓은
세션을 다시 저장하면 같은 좌표계 (map.json frame.base_map 에 기록).
"""
import json
import re
import shutil
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MAP_ROOT = HERE / "maps"
FORMAT = "scan3d-map/1"
MODES = ("append", "localize")
_NAME = re.compile(r"^[\w\-. ]{1,64}$")


def safe_name(name):
    n = str(name or "").strip()
    if not _NAME.match(n) or n.startswith(".") or ".." in n:
        raise ValueError(f"지도 이름이 잘못됨: {name!r} (글자·숫자·_ - . 공백, 64 자까지)")
    return n


def root_dir(root=None):
    return Path(root) if root else MAP_ROOT


def map_dir(name, root=None):
    return root_dir(root) / safe_name(name)


def _du(d):
    return sum(f.stat().st_size for f in Path(d).rglob("*") if f.is_file())


def list_maps(root=None):
    """[{name, created, source, counts, size_mb, png}] 최근 것 먼저."""
    out = []
    r = root_dir(root)
    if not r.is_dir():
        return out
    for d in r.iterdir():
        f = d / "map.json"
        if not d.is_dir() or not f.is_file():
            continue
        try:
            m = json.loads(f.read_text())
        except (OSError, ValueError):
            continue
        out.append(dict(name=d.name, created=m.get("created"), source=m.get("source"), counts=m.get("counts"),
                        frame=m.get("frame"), size_mb=round(_du(d) / 2 ** 20, 1), esdf2d=(d / "esdf2d.png").is_file()))
    out.sort(key=lambda x: str(x.get("created") or ""), reverse=True)
    return out


def delete_map(name, root=None):
    d = map_dir(name, root)
    if not (d / "map.json").is_file():
        raise FileNotFoundError(f"지도 없음: {name}")
    shutil.rmtree(d)
    return True


def save_map(name, vmap, reloc=None, traj=None, source=None, frame=None, extra=None, root=None, log=None,
             overwrite=True):
    """지도 저장. vmap = voxel_map.VoxelMap (전역 좌표), reloc = reloc.Relocalizer (기준 키프레임 DB, 없으면 위치 찾기 불가),
    traj = [(t, T 4x4)], source = dict (출처), frame = dict (좌표계 설명). 반환 map.json dict."""
    name = safe_name(name)
    r = root_dir(root)
    r.mkdir(parents=True, exist_ok=True)
    d = r / name
    if d.exists() and not overwrite:
        raise FileExistsError(f"이미 있는 지도: {name}")
    tmp = r / f".tmp_{name}_{int(time.time() * 1000)}"
    tmp.mkdir()
    t0 = time.perf_counter()
    try:
        counts = {}
        with vmap.lock:
            counts["voxels"] = vmap.save_state(tmp / "voxels_state.npz")
            ts = vmap.tsdf
            fs = getattr(vmap, "free", None)
        counts["tsdf_blocks"] = ts.save_npz(tmp / "tsdf.npz") if ts is not None else 0
        if fs is not None:
            fs.save_npz(tmp / "free.npz")
            counts["free_cells"] = fs.stats()["free_cells"]
        db = reloc.export_db() if reloc is not None else None
        if db is not None and int(db["n"]):
            np.savez_compressed(tmp / "reloc_db.npz", **db)
            counts["reloc_keyframes"] = int(db["n"])
            counts["reloc_features"] = int(len(db["desc"]))
        else:
            counts["reloc_keyframes"] = 0
        tr = list(traj or [])
        (tmp / "trajectory.json").write_text(json.dumps(
            [dict(t=round(float(t), 4), T=np.asarray(T, float).reshape(-1).round(6).tolist()) for t, T in tr]))
        if ts is not None:
            import tsdf_map
            V, C, F = ts.mesh_arrays()
            tsdf_map.write_mesh_ply(tmp / "mesh.ply", V, C, F)
            counts["mesh_vertices"] = int(len(V))
        esdf_meta = None
        try:
            import esdf_map
            e = esdf_map.for_vmap(vmap)
            xy = [tuple(float(v) for v in np.asarray(T)[:3, 3]) for _, T in tr[::max(1, len(tr) // 2000)]]
            esdf_meta = e.save(tmp, traj_xy=xy)
        except Exception as ex:                  # ESDF 는 덤 — 실패해도 지도는 저장
            if log:
                log(f"[map] ESDF 저장 실패 {ex!r}")
        meta = dict(format=FORMAT, name=name, created=time.strftime("%Y-%m-%d %H:%M:%S"), source=source or {},
                    frame=dict(dict(world="OpenVINS 월드: 중력 정렬 z 위, 미터, 원점 = 지도를 처음 만든 세션의 VIO 시작 위치"),
                               **(frame or {})),
                    counts=counts, voxel_m=vmap.voxel_m, min_hits=vmap.min_hits,
                    tsdf_cfg=None if ts is None else ts.cfg, esdf=esdf_meta and dict(res=esdf_meta["res"],
                                                                                    slice2d=esdf_meta["slice2d"]),
                    save_ms=round(1000 * (time.perf_counter() - t0)), **(extra or {}))
        (tmp / "map.json").write_text(json.dumps(meta, indent=1, default=str))
        if d.exists():
            shutil.rmtree(d)
        tmp.rename(d)
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    meta["size_mb"] = round(_du(d) / 2 ** 20, 1)
    if log:
        log(f"[map] 지도 '{name}' 저장: 복셀 {counts['voxels']} · TSDF 블록 {counts['tsdf_blocks']} · 기준 키프레임 "
            f"{counts['reloc_keyframes']} · {meta['size_mb']} MB · {meta['save_ms']} ms → {d}")
    return meta


def load_map(name, root=None, log=None, device=None, tsdf_cfg=None):
    """반환 dict(meta, vmap (VoxelMap, TSDF·빈 칸 포함), db (export_db 형식 dict 또는 None), traj [(t, T)], dir)."""
    import voxel_map
    d = map_dir(name, root)
    f = d / "map.json"
    if not f.is_file():
        raise FileNotFoundError(f"지도 없음: {name}")
    t0 = time.perf_counter()
    meta = json.loads(f.read_text())
    vm = voxel_map.VoxelMap.load_state(d / "voxels_state.npz")
    if (d / "tsdf.npz").is_file():
        import tsdf_map
        vm._tsdf = tsdf_map.TSDFMap.load_npz(d / "tsdf.npz", device=device, log=log, cfg=tsdf_cfg)
    if (d / "free.npz").is_file():
        import esdf_map
        vm.free = esdf_map.FreeSpace.load_npz(d / "free.npz")
    db = None
    if (d / "reloc_db.npz").is_file():
        z = np.load(d / "reloc_db.npz", allow_pickle=False)
        db = {k: z[k] for k in z.files}
    traj = []
    try:
        traj = [(e["t"], np.asarray(e["T"], float).reshape(4, 4)) for e in json.loads((d / "trajectory.json").read_text())]
    except (OSError, ValueError, KeyError):
        pass
    ms = round(1000 * (time.perf_counter() - t0))
    if log:
        log(f"[map] 지도 '{name}' 불러옴: 복셀 {vm.n} · TSDF 블록 {0 if vm.tsdf is None else vm.tsdf.nb} · 기준 키프레임 "
            f"{0 if db is None else int(db['n'])} ({ms} ms)")
    return dict(meta=meta, vmap=vm, db=db, traj=traj, dir=d, load_ms=ms)
