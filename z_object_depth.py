#!/usr/bin/env python3
"""
z_object_depth.py — 조밀 스테레오(Fast-FoundationStereo) + YOLO 로 객체별 거리·크기

  ./venv_ffs/bin/python z_object_depth.py --mode seg     # yolo11m-seg 마스크로 집계 (기본 권장)
  ./venv_ffs/bin/python z_object_depth.py --mode bbox    # yolo11m 박스로 집계 (비교 기준선)
  ./venv_ffs/bin/python z_object_depth.py --mode both    # seg 한 번 -> 같은 인스턴스를 mask/bbox 둘 다 집계
  옵션: --scale 0.5 --iters 4 (빠르게)  --lr-check (좌우 일관성으로 오매칭 제거, 추론 2회)
        --known-m 0.62 --known-idx 1 (실측 대조)

※ venv_ffs 로 실행해야 한다 (torch 2.6 고정). 메인 venv 에서는 lib_ffs import 가 실패한다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
파이프라인 (설계 근거: doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md §4, §12)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  L/R 원본 + calib_stereo.json(K1,D1,K2,D2,R,T)
    -> lib_rectify 로 두 영상 정류 (alpha=0, CALIB_ZERO_DISPARITY)
    -> Fast-FoundationStereo: 정류쌍 -> 시차 -> Z_rect = f*baseline/d
    -> (--lr-check) 좌우를 바꾸고 각각 가로 반전해 한 번 더 추론, 두 시차가 안 맞는 화소 제외
    -> YOLO 를 "정류된 왼쪽 영상"에서 실행 -> 마스크/박스가 시차와 같은 좌표계
    -> 객체별 (영역 ∩ 유효화소) -> 깊이 통계 + 3D 점 -> 중심·크기(평면 모델)
  유효 화소: finite(d) & d>0.5 & 우측 대응점 (u-d, v) 가 roi2 안 & Z 가 z_range 안.

  희소(SIFT) 경로와의 차이: 마스크 안 "모든" 화소에 깊이가 있으므로 질감 없는 물체도
  되고, 경계 특징점이 부족해서 생기던 크기 과소평가는 없다. 다만 세그멘테이션 누락·
  가림·무효 시차·평면 모델 부적합으로 경계가 사라질 수는 있다. 정류 품질과 캘리
  왜곡계수(폰 초광각 가장자리 커버리지 8/16)에 민감하다.

  깊이 필드 셋을 구분한다: depth_rect_m(정류 광축 Z 중앙값), z_cam1_med_m(원 카메라1 광축),
  dist_m(카메라 중심까지 유클리드). 회전이 있으면 서로 다르다. 전부 **최종 선택 화소**에서
  계산하고, 영역 전체의 원시 중앙값은 depth_raw_m 으로 따로 둔다 (R03).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
상태·검사 (R06/R07)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  meta.checks.epipolar : pass | fail | unavailable (텍스처 부족은 실패가 아니라 판정 불가)
  객체 status          : ok | ambiguous(선택 화소 < 50%, 배경 혼입 의심) | insufficient_valid_pixels | no_mask
  객체 warnings        : rectification_suspect(정류 검사 fail), scale_provisional, non_planar
  NaN 은 JSON 에 쓰지 않는다 (None 으로 바꾼다).
"""
import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import lib_calib as lc
import lib_detect as ld
import lib_ffs
import lib_rectify as lr
import lib_stereo as ls

HERE = Path(__file__).parent
EPI_GATE_PX = 1.0          # |dy| 중앙값 합격선(px)
EPI_INLIER_FRAC = 0.7      # |dy|<2px 인 대응점 비율 합격선


def _clean(x):
    """JSON 저장 전 NaN/Inf/ndarray 정리."""
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return _clean(x.tolist())
    if isinstance(x, (np.floating, float)):
        return None if not np.isfinite(x) else float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def valid_mask(disp, rp, z_range, min_disp=0.5):
    """유효 화소: 시차 양수·유한, 우측 대응점 (u-d, v) 가 roi2 안(x·y 모두), 깊이 범위 안, 좌측 roi1 안."""
    H, W = disp.shape
    u = np.arange(W, dtype=np.float32)[None, :].repeat(H, 0)
    v = np.arange(H, dtype=np.float32)[:, None].repeat(W, 1)
    ur = u - disp
    x2, y2, w2, h2 = rp["roi2"]
    Z = lr.disparity_to_depth(disp, rp)
    ok = np.isfinite(disp) & (disp > min_disp) & (ur >= x2) & (ur < x2 + w2) & (v >= y2) & (v < y2 + h2)
    ok &= np.isfinite(Z) & (Z >= z_range[0]) & (Z <= z_range[1])
    x1, y1, w1, h1 = rp["roi1"]
    box = np.zeros_like(ok); box[y1:y1 + h1, x1:x1 + w1] = True
    return ok & box, Z


def lr_consistency(model, rgbL, rgbR, dispL, iters, scale, thr_px):
    """좌우 일관성 (Project Aria exporter 방식): 오른쇽/왼쪽을 바꾸고 각각 가로 반전해 한 번 더
    추론하면 오른쪽 기준 양의 시차가 나온다. 왼쪽 화소 (u,v,d) 가 가리키는 오른쪽 위치 (u-d, v)
    의 시차가 d 와 thr_px 이상 다르면 오매칭/가림으로 본다. 반환: 일관 마스크(bool HxW)."""
    dispR = lib_ffs.infer(model, rgbR[:, ::-1].copy(), rgbL[:, ::-1].copy(), valid_iters=iters, scale=scale)[:, ::-1]
    H, W = dispL.shape
    u = np.arange(W, dtype=np.float32)[None, :].repeat(H, 0)
    ur = np.rint(u - dispL).astype(int)
    inb = (ur >= 0) & (ur < W) & np.isfinite(dispL)
    dR = np.full_like(dispL, np.nan)
    rows = np.arange(H)[:, None].repeat(W, 1)
    dR[inb] = dispR[rows[inb], ur[inb]]
    return inb & np.isfinite(dR) & (np.abs(dR - dispL) <= thr_px), dispR


def _empty(label, instance, det, method):
    x, y, w, h = det["box"]
    return dict(instance=instance, label=label, det_conf=round(float(det["conf"]), 3), method=method,
                bbox_xyxy=[int(x), int(y), int(x + w), int(y + h)], status="insufficient_valid_pixels",
                warnings=[], region_count=0, valid_count=0, valid_fraction=0.0, selected_count=0,
                selected_fraction=None, depth_raw_m=None, depth_rect_m=None, z_cam1_med_m=None,
                dist_m=None, p10_m=None, p90_m=None, mad_m=None, mixed_depth=None,
                center_cam1_m=None, center_map_m=None, dims_mm=None, size_status=None,
                planarity=None, selection_policy=None, depth_clusters=None, minority_fraction=None,
                coordinate_frame="cam1(원 카메라1 광학계) / map(Z-up)")


def split_depth(z, jump=1.3):
    """1D 깊이 분포를 최대 간격으로 두 군집으로 나눈다. 반환: 라벨(0/1) 또는 None(단일 군집).

    log(z) 를 정렬해 인접 간격이 log(jump) 를 넘는 가장 큰 곳에서 자른다. 깊이 노이즈 2% 는
    간격이 작아 안 잘리고, 물체(1.5m)와 배경(4m)처럼 30% 이상 떨어진 두 표면은 잘린다.
    """
    if len(z) < 20:
        return None
    lz = np.log(np.asarray(z, float))
    order = np.argsort(lz)
    gaps = np.diff(lz[order])
    k = int(np.argmax(gaps))
    if gaps[k] < np.log(jump):
        return None
    lab = np.zeros(len(z), int)
    lab[order[k + 1:]] = 1
    return lab


def choose_cluster(lab, uv, bbox_xyxy, mode):
    """두 깊이 군집 중 물체를 고른다. 반환 (선택 라벨, 정책 문자열, 군집 요약 리스트).

    seg  : 마스크가 곧 물체이므로 큰 군집. 작은 군집은 경계 혼입(배경/가림)으로 본다.
    bbox : 박스에는 배경이 같이 들어오고 배경이 과반일 수도 있다. "가장 큰 군집"이나
           "가장 가까운 군집"을 무조건 고르면 배경/앞 물체를 고른다(doc_DEPTH_CODE_REVIEW §6).
           검출 박스의 중앙 절반 영역에 더 많이 있는 군집을 고른다(center support).
    어느 쪽이든 선택 근거를 clusters 로 남겨 사람이 확인할 수 있게 한다.
    """
    x0, y0, x1, y1 = bbox_xyxy
    cx0, cx1 = x0 + 0.25 * (x1 - x0), x1 - 0.25 * (x1 - x0)
    cy0, cy1 = y0 + 0.25 * (y1 - y0), y1 - 0.25 * (y1 - y0)
    central = (uv[:, 0] >= cx0) & (uv[:, 0] < cx1) & (uv[:, 1] >= cy0) & (uv[:, 1] < cy1)
    summ = []
    for l in (0, 1):
        m = lab == l
        summ.append(dict(label=l, n=int(m.sum()), frac=float(m.mean()),
                         center_support=float((m & central).sum() / max(1, central.sum()))))
    if mode == "seg":
        pick = max(summ, key=lambda c: c["n"])["label"]; policy = "seg:larger-cluster"
    else:
        pick = max(summ, key=lambda c: c["center_support"])["label"]; policy = "bbox:center-support"
    return pick, policy, summ


def aggregate(rec, region, valid, Z, pts_cam1, rp=None, outline=False):
    """영역 ∩ 유효화소 -> planar_box 로 물체 화소를 고르고, 그 **같은 집합**에서 깊이·중심·치수."""
    sel0 = (region > 0) & valid
    n, n_reg = int(sel0.sum()), int((region > 0).sum())
    rec.update(region_count=n_reg, valid_count=n, valid_fraction=(n / n_reg) if n_reg else 0.0)
    if n < 50:
        rec["status"] = "insufficient_valid_pixels"
        return rec
    z_raw = Z[sel0]
    rec["depth_raw_m"] = float(np.median(z_raw))
    rec["mixed_depth"] = bool(np.percentile(z_raw, 90) / max(np.percentile(z_raw, 10), 1e-6) > 1.3)
    P = pts_cam1[sel0]
    vv, uu = np.where(sel0)
    # 깊이가 두 덩어리면 물체 후보를 고르고 근거를 남긴다. 아니면 전체를 쓴다.
    lab = split_depth(z_raw)
    rec["selection_policy"] = "single-cluster"; rec["depth_clusters"] = None
    if lab is not None:
        pick, policy, summ = choose_cluster(lab, np.column_stack([uu, vv]), rec["bbox_xyxy"], rec["method"])
        rec["selection_policy"] = policy
        rec["depth_clusters"] = [dict(c, z_med=float(np.median(z_raw[lab == c["label"]]))) for c in summ]
        keep = lab == pick
        if keep.sum() < 50:
            rec["status"] = "insufficient_valid_pixels"; return rec
        z_raw, P = z_raw[keep], P[keep]
        # 선택되지 않은 쪽이 작지 않으면 물체를 확정할 수 없다 -> ambiguous
        rec["minority_fraction"] = float(1.0 - keep.mean())
    else:
        rec["minority_fraction"] = 0.0
    rays = None
    if outline and rp is not None:
        cs, _ = cv2.findContours((region > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if cs:
            rays = lr.pixel_rays_cam1(rp, max(cs, key=cv2.contourArea).reshape(-1, 2))
    pb = ls.planar_box(P, boundary_rays=rays)
    if pb is None:
        rec["status"] = "insufficient_valid_pixels"
        return rec
    idx = pb["idx"]
    zs = z_raw[idx]; Ps = P[idx]
    frac = len(idx) / n
    rec.update(selected_count=int(len(idx)), selected_fraction=float(frac),
               depth_rect_m=float(np.median(zs)), p10_m=float(np.percentile(zs, 10)),
               p90_m=float(np.percentile(zs, 90)), mad_m=float(np.median(np.abs(zs - np.median(zs)))),
               z_cam1_med_m=float(np.median(Ps[:, 2])),
               center_cam1_m=np.asarray(pb["center"]).round(4).tolist(),
               dist_m=float(np.linalg.norm(pb["center"])),
               center_map_m=(ls.CAM_TO_MAP @ np.asarray(pb["center"])).round(4).tolist(),
               planarity=pb["planarity"])
    if pb["planar_ok"]:
        rec.update(dims_mm=(pb["dims_sorted"] * 1000).round(1).tolist(),
                   size_status=f"planar+{pb['size_from']}")
    else:
        rec.update(dims_mm=None, size_status="non_planar"); rec["warnings"].append("non_planar")
    # seg: 마스크 안 소수 군집 20% 초과면 혼입/가림 의심.
    # bbox: 소수 군집이 40% 를 넘으면(예: 물체 55% / 배경 45%) 어느 쪽이 물체인지 확정할 수 없다.
    #       중앙 지지로 고르긴 하지만 status 는 ambiguous 로 두고 depth_clusters 를 보게 한다.
    # bbox: 깊이 군집이 갈라지지 않는데(single-cluster) 상자 안 깊이가 p90/p10 > 1.3 로 섞여 있으면 배경이 연속으로
    #       들어온 것(예: 화면 대부분을 차지한 사람 + 뒤 벽·바닥 — 폰 실쌍에서 dims 2.5m, dist 1.57m 가 ok 로 나왔다).
    #       seg 는 마스크가 물체만 잡으므로 깊이가 긴 물체(탁자)를 위해 이 조건을 걸지 않는다.
    thr = 0.2 if rec["method"] == "seg" else 0.4
    ambiguous = rec["minority_fraction"] > thr or frac < 0.5
    if rec["method"] == "bbox" and rec["mixed_depth"] and rec["selection_policy"] == "single-cluster":
        ambiguous = True; rec["warnings"].append("bbox_mixed_depth_no_split")
    rec["status"] = "ambiguous" if ambiguous else "ok"
    return rec


def draw_overlay(rect_bgr, dets, results, mode):
    vis = ld.draw(rect_bgr, dets, color=(0, 220, 80) if mode != "bbox" else (255, 160, 0))
    for d, r in zip(dets, results):
        x, y, w, h = d["box"]
        t = (f"{r['depth_rect_m']:.2f}m {r['valid_fraction']*100:.0f}%v {r['selected_fraction']*100:.0f}%s"
             if r["status"] in ("ok", "ambiguous") else r["status"])
        if r["status"] == "ambiguous":
            t += " ?"
        yy = min(rect_bgr.shape[0] - 6, y + h + 16)
        cv2.putText(vis, t, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(vis, t, (x, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return vis


def load_pose_any(a, imgL, imgR):
    """포즈 출처: 기본은 shots/meta.json 의 mode 를 따른다 (z_reconstruct 와 동일 규칙).
    stereo -> calib_stereo.json,  board/mono -> z_reconstruct.load_pose (같은 카메라 K 둘).
    반환 (calL, calR, R, T, st_dict)"""
    meta_p = Path(a.left).parent / "meta.json"
    mode = "stereo"
    if a.pose == "auto" and meta_p.exists():
        mode = json.loads(meta_p.read_text()).get("mode", "stereo")
    if mode in ("board", "mono"):
        import z_reconstruct as zr                     # 지연 import (open3d 포함)
        R, T, calL, calR, info = zr.load_pose(json.loads(meta_p.read_text()), imgL, imgR)
        st = dict(K1=calL["K"].tolist(), dist1=calL["dist"].ravel().tolist(),
                  K2=calR["K"].tolist(), dist2=calR["dist"].ravel().tolist(), R=np.asarray(R).tolist(),
                  T=np.asarray(T).ravel().tolist(), image_size=list(calL["image_size"]),
                  baseline_m=info["baseline_m"], scale_status=info["scale_status"], method=f"shots:{mode}")
        return calL, calR, np.asarray(R), np.asarray(T).reshape(3, 1), st
    st = json.loads(Path(a.calib_stereo).read_text())
    calL = dict(K=np.array(st["K1"]), dist=np.array(st["dist1"]), image_size=tuple(st["image_size"]))
    calR = dict(K=np.array(st["K2"]), dist=np.array(st["dist2"]), image_size=tuple(st["image_size"]))
    return calL, calR, np.array(st["R"]), np.array(st["T"]).reshape(3, 1), st


def main(a):
    imgL, imgR = cv2.imread(a.left), cv2.imread(a.right)
    if imgL is None or imgR is None:
        sys.exit(f"이미지 없음: {a.left} / {a.right}")
    calL, calR, R, T, st = load_pose_any(a, imgL, imgR)
    # 별도 캘리 파일이 있으면 stereo JSON 의 K 와 같은 세트인지 확인한다 (R11).
    for path, cal, nm in ((a.calib_l, calL, "L"), (a.calib_r, calR, "R")):
        if str(st.get("method", "")).startswith("shots:"):
            break                                          # board/mono: K 는 calib.json 하나뿐
        if path and Path(path).exists():
            c = lc.load_calib(path)
            if not np.allclose(c["K"], cal["K"], rtol=1e-6) or not np.allclose(c["dist"], cal["dist"], atol=1e-9):
                sys.exit(f"{nm}: {path} 의 K/dist 가 {a.calib_stereo} 안의 K{1 if nm=='L' else 2} 와 다름 — "
                         f"다른 캘리 세트가 섞였다. z_stereo_pose.py 를 다시 돌릴 것")
    scale_status = st.get("scale_status", "unknown")
    warnings_global = []
    if scale_status == "provisional":
        warnings_global.append("scale_provisional")
        print("  [WARN] calib_stereo.json 스케일이 임시값 — 거리·크기는 비율만 맞다 (z_stereo_pose.py --rescale)")

    lr.assert_size(imgL, calL, "L"); lr.assert_size(imgR, calR, "R")
    out_dir = Path(a.out); out_dir.mkdir(exist_ok=True)

    # ── 정류 ─────────────────────────────────────────────────────────────────
    rp = lr.rectify_maps(calL, calR, R, T, alpha=0.0)
    rL, rR = lr.rectify_pair(imgL, imgR, rp)
    ep = lr.epipolar_check(rL, rR)
    if ep["status"] == "ok":
        ep_status = "pass" if (ep["dy_med"] < EPI_GATE_PX and ep["inlier_frac"] >= EPI_INLIER_FRAC) else "fail"
        print(f"정류: f {rp['f']:.1f}px baseline {rp['baseline_m']*1000:.1f}mm roi1 {rp['roi1']} roi2 {rp['roi2']}")
        print(f"  [{ep_status.upper()}] 에피폴라 잔차 |dy| 중앙값 {ep['dy_med']:.2f}px (< {EPI_GATE_PX}), "
              f"|dy|<2px 비율 {ep['inlier_frac']*100:.0f}% (>= {EPI_INLIER_FRAC*100:.0f}%)  "
              f"[전체 RMS {ep['dy_rms']:.2f}, 인라이어 RMS {ep['dy_rms_inlier'] or 0:.2f}, n={ep['n']}]")
        if ep_status == "fail":
            warnings_global.append("rectification_suspect")
            print("         (R,T) 나 캘리가 틀렸을 수 있다. 결과에 rectification_suspect 를 남긴다.")
    else:
        ep_status = "unavailable"
        print(f"정류: 대응점 {ep['n']}개 — 정류 검사 불가(텍스처 부족). 실패가 아니라 판정 불가.")

    # ── 조밀 시차 ────────────────────────────────────────────────────────────
    model = lib_ffs.load(a.ckpt, valid_iters=a.iters)
    rgbL, rgbR = cv2.cvtColor(rL, cv2.COLOR_BGR2RGB), cv2.cvtColor(rR, cv2.COLOR_BGR2RGB)
    t0 = time.perf_counter()
    disp = lib_ffs.infer(model, rgbL, rgbR, valid_iters=a.iters, scale=a.scale)
    t_ffs = time.perf_counter() - t0
    valid, Z = valid_mask(disp, rp, tuple(a.z_range))
    lr_frac = None
    if a.lr_check:
        t1 = time.perf_counter()
        # 임계는 "추론 해상도" 픽셀로 정의한다 (doc_YOLO_..._RESEARCH §12). 시차는 원해상도 단위로
        # 되돌려져 있으므로 비교 임계도 1/scale 배 한다. scale 0.5 에서 1.5px 그대로 쓰면 추론 기준
        # 0.75px 로 지나치게 엄격해 유효 화소의 30% 가 떨어져 나갔다 (합성 검증).
        thr_full = a.lr_thr_px / a.scale
        cons, _ = lr_consistency(model, rgbL, rgbR, disp, a.iters, a.scale, thr_full)
        lr_frac = float(cons[valid].mean()) if valid.any() else None
        valid &= cons
        print(f"좌우 일관성: 유효화소 중 {lr_frac*100:.1f}% 통과 (|dL-dR| <= {a.lr_thr_px}px@추론해상도 = {thr_full:.2f}px@원해상도), "
              f"+{(time.perf_counter()-t1)*1000:.0f}ms")
    pts = lr.depth_to_points_cam1(np.where(valid, Z, np.nan).astype(np.float32), rp)
    print(f"FFS: {t_ffs*1000:.0f}ms (scale {a.scale}, iters {a.iters}, 첫 호출은 컴파일 포함)  "
          f"유효화소 {valid.mean()*100:.1f}%  시차 중앙값 {np.median(disp[valid]) if valid.any() else float('nan'):.1f}px")
    np.save(out_dir / "depth_rect_m.npy", np.where(valid, Z, np.nan).astype(np.float32))
    cv2.imwrite(str(out_dir / "valid_mask.png"), (valid * 255).astype(np.uint8))
    if valid.any():
        dn = (255 * np.where(valid, disp, 0) / max(1e-6, np.percentile(disp[valid], 99))).clip(0, 255).astype(np.uint8)
        cv2.imwrite(str(out_dir / "disp_vis.png"), cv2.applyColorMap(dn, cv2.COLORMAP_TURBO))

    # ── 검출 (정류된 왼쪽에서) ───────────────────────────────────────────────
    seg = a.mode in ("seg", "both")
    model_name = a.model or (ld.DEFAULT_SEG_MODEL if seg else ld.DEFAULT_MODEL)
    dets = ld.detect_yolo(rL, model=model_name, conf=a.conf, classes=a.classes, device=a.device)
    print(f"YOLO({model_name}, {getattr(ld.detect_yolo, 'last_device', '?')}) 검출 {len(dets)}개")

    # ── 집계 ─────────────────────────────────────────────────────────────────
    H, W = disp.shape
    results = {}
    for m in (["seg", "bbox"] if a.mode == "both" else [a.mode]):
        rows = []
        for i, d in enumerate(dets):
            rec = _empty(d["label"], i + 1, d, m)
            rec["warnings"] = list(warnings_global)
            if m == "seg":
                region = d.get("mask")
                if region is None:
                    rec["status"] = "no_mask"; rows.append(rec); continue
            else:
                region = np.zeros((H, W), np.uint8); x, y, w, h = d["box"]
                region[max(0, y):y + h, max(0, x):x + w] = 255
            rows.append(aggregate(rec, region, valid, Z, pts, rp=rp, outline=(m == "seg")))
        results[m] = rows
        cv2.imwrite(str(out_dir / f"overlay_{m}.png"), draw_overlay(rL, dets, rows, m))

    # ── 출력 ─────────────────────────────────────────────────────────────────
    print(f"\n{'='*78}\n{'#':>2s} {'label':12s} {'meth':4s} {'status':10s} {'Zrect':>6s} {'dist':>6s} "
          f"{'p10~p90':>13s} {'val':>4s} {'sel':>4s} {'크기 mm (W x H x 두께)':>22s}\n{'='*78}")
    for m, rows in results.items():
        for r in rows:
            if r["status"] not in ("ok", "ambiguous"):
                print(f"{r['instance']:>2} {r['label']:12s} {m:4s} {r['status']}"); continue
            dims = r["dims_mm"]; ds = " x ".join(f"{v:.0f}" for v in dims) if dims else f"-({r['size_status']})"
            print(f"{r['instance']:>2} {r['label']:12s} {m:4s} {r['status']:10s} {r['depth_rect_m']:6.3f} {r['dist_m']:6.3f} "
                  f"{r['p10_m']:6.3f}~{r['p90_m']:6.3f} {r['valid_fraction']*100:3.0f}% {r['selected_fraction']*100:3.0f}% {ds:>22s}")
    if a.known_m:
        for m, rows in results.items():
            for r in rows:
                if r["status"] == "ok" and r["instance"] == a.known_idx:
                    e = abs(r["dist_m"] - a.known_m) / a.known_m * 100
                    print(f"  [{'PASS' if e < 10 else 'FAIL'}] {m}: 객체 {a.known_idx} 거리 {r['dist_m']:.3f}m vs 실측 {a.known_m:.3f}m -> 오차 {e:.1f}%")

    meta = dict(time=datetime.now().isoformat(timespec="seconds"), mode=a.mode, model=model_name,
                ckpt=str(a.ckpt), scale=a.scale, iters=a.iters, lr_check=bool(a.lr_check), lr_pass_frac=lr_frac,
                scale_status=scale_status, stereo_baseline_m=float(st.get("baseline_m", np.linalg.norm(T))),
                checks=dict(epipolar=ep_status, epipolar_n=ep["n"], epipolar_dy_med_px=ep["dy_med"],
                            epipolar_dy_rms_px=ep["dy_rms"], epipolar_inlier_frac=ep["inlier_frac"]),
                f_rect=rp["f"], cx=rp["cx"], cy=rp["cy"], baseline_rect_m=rp["baseline_m"],
                roi1=list(rp["roi1"]), roi2=list(rp["roi2"]), R1=rp["R1"], Q=rp["Q"],
                image_size=[W, H], ffs_ms=round(t_ffs * 1000, 1), valid_fraction=float(valid.mean()))
    tmp = out_dir / "objects.json.tmp"
    tmp.write_text(json.dumps(_clean(dict(meta=meta, objects=results)), indent=2, allow_nan=False))
    os.replace(tmp, out_dir / "objects.json")
    print(f"\n저장: {out_dir}/objects.json, overlay_*.png, depth_rect_m.npy, disp_vis.png, valid_mask.png")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["seg", "bbox", "both"], default="both")
    ap.add_argument("--left", default=str(HERE / "shots" / "L.png"))
    ap.add_argument("--right", default=str(HERE / "shots" / "R.png"))
    ap.add_argument("--calib-l", default=str(HERE / "calib.json"), help="K 일치 확인용 (없으면 건너뜀)")
    ap.add_argument("--calib-r", default=str(HERE / "calib_phone.json"))
    ap.add_argument("--calib-stereo", default=str(HERE / "calib_stereo.json"))
    ap.add_argument("--pose", choices=["auto", "stereo"], default="auto",
                    help="auto: shots/meta.json 의 mode 를 따름 (board/mono/stereo). stereo: calib_stereo.json 강제")
    ap.add_argument("--ckpt", default=str(lib_ffs.DEFAULT_CKPT))
    ap.add_argument("--scale", type=float, default=1.0, help="FFS 입력 축소 배율 (0<s<=1, 0.5 면 빠름)")
    ap.add_argument("--iters", type=int, default=8, help="refinement 반복 (4 or 8)")
    ap.add_argument("--lr-check", action="store_true", help="좌우 일관성 검사 (추론 2회)")
    ap.add_argument("--lr-thr-px", type=float, default=1.5, help="좌우 일관성 임계 (추론 해상도 px)")
    ap.add_argument("--z-range", type=float, nargs=2, default=(0.1, 20.0), metavar=("MIN", "MAX"))
    ap.add_argument("--model", help="YOLO 가중치 (기본: seg 면 yolo11m-seg.pt, bbox 면 yolo11m.pt)")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--classes", nargs="+")
    ap.add_argument("--device", default=None)
    ap.add_argument("--known-m", type=float, help="객체 하나의 실측 거리(m) — 검증용")
    ap.add_argument("--known-idx", type=int, default=1)
    ap.add_argument("--out", default=str(HERE / "out_dense"))
    main(ap.parse_args())
