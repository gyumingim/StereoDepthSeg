#!/usr/bin/env python3
"""폰 두 렌즈(초광각 2 + 광각 5) 스테레오 캘리 -> calib_phone_pair.json

  venv/bin/python z_phone_calib.py --factory                                   # 공장값만 (dy 3~5px, 쓰지 말 것)
  venv/bin/python z_phone_calib.py --refine phone_stereo/runs/*                # 장면 쌍으로 회전 보정, baseline 공장 고정
  venv/bin/python z_phone_calib.py --board phone_stereo/runs/board --screen laptop --ruler-mm 179.1   # 체커보드 (권장)
  venv/bin/python z_phone_calib.py --check phone_stereo/runs/probe_now         # 기존 json 을 쌍에서 검사

번들 = z_phone_stereo.py 가 저장한 camera_<id>.png 두 장이 있는 폴더 (S 키 / --save-interval 로 captures/<ts>/ 에 쌓임).
--physical 은 (왼, 오른) 순서. 공장 포즈로 보아 오른쪽이 왼쪽에 있으면 자동으로 뒤집고 알려준다.
결과 json 의 phone.physical 순서와 같게 z_phone_stereo.py --physical 을 주면 된다 (뒤집혀 있으면 수신 쪽이 스왑한다).

scale_status : factory_unverified | factory_baseline_refined | metric | board_unreliable
  metric 은 --board 가 rms<=1px, 뷰>=8, baseline 이 공장값 ±15% 안일 때만 붙는다.
"""
import argparse
import json
from pathlib import Path

import numpy as np

import lib_calib
import lib_phone_calib as pc

EPI_GATE_PX, EPI_INLIER_FRAC = 1.0, 0.7          # z_object_depth.py 와 같은 합격선 (torch 임포트를 피하려 여기에 복제)
BOARD_RMS_MAX_PX, BOARD_MIN_VIEWS, BOARD_BASELINE_TOL = 1.0, 8, 0.15


def scale_to(st, size):
    """K 만 해상도에 비례해 스케일. dist 는 정규화 좌표 기준이라 불변."""
    if list(size) == list(st["image_size"]):
        return st
    sx, sy = size[0] / st["image_size"][0], size[1] / st["image_size"][1]
    if abs(sx / sy - 1) > 0.01:
        raise ValueError(f"--target-size {size} 와 캡처 {st['image_size']} 비율이 다름")
    S = np.diag([sx, sy, 1.0])
    new = dict(st, K1=S @ np.asarray(st["K1"]), K2=S @ np.asarray(st["K2"]), image_size=[int(size[0]), int(size[1])])
    new["scaled_from"] = list(st["image_size"])
    return new


def report(rows, label):
    print(f"[{label}] 정류 후 |dy| (합격: 중앙값 < {EPI_GATE_PX}px 이고 인라이어(<2px) >= {EPI_INLIER_FRAC})")
    ok_all = True
    for r in rows:
        if r["status"] != "ok":
            print(f"   {Path(r['pair']).name:24s} 대응점 {r['n']}개 — 판정 불가(텍스처 부족)")
            continue
        ok = r["dy_med"] < EPI_GATE_PX and r["inlier_frac"] >= EPI_INLIER_FRAC
        ok_all &= ok
        print(f"   {Path(r['pair']).name:24s} n {r['n']:4d}  dy 중앙값 {r['dy_med']:5.2f}px  인라이어 {r['inlier_frac']:.2f}  "
              f"dx>0 {r['pos_dx_frac']:.2f}  {'OK' if ok else 'FAIL'}")
    return ok_all


def main(a):
    inv = pc.load_inventory(a.inventory)
    L, R = pc.order_left_right(inv, a.physical, a.size)
    if [L, R] != [str(v) for v in a.physical]:
        print(f"* 공장 포즈상 물리 {R} 가 오른쪽에 있음 — (왼,오른) = ({L},{R}) 로 뒤집음")
    if a.check:
        st = pc.load(a.out)
        if st["phone"]["physical"] != [L, R]:
            raise ValueError(f"{a.out} 은 {st['phone']['physical']} 용 — --physical 불일치")
        pairs = pc.load_pairs(a.check, L, R)
        rows, rp = pc.evaluate_pairs(pairs, st)
        report(rows, f"check {a.out} ({st.get('scale_status')})")
        return
    st = pc.factory_stereo(inv, L, R, a.size, a.logical)
    print(f"공장값: baseline {st['baseline_m']*1000:.2f}mm, 상대회전 {st['factory_rotation_deg']:.2f}°, "
          f"fx L {st['K1'][0,0]:.1f} R {st['K2'][0,0]:.1f} @ {a.size}")
    if a.base and (a.board or a.refine):
        # 기존 보정본(예: 광각 fx 0.991 스케일이 들어간 refine 결과)의 K/dist 를 초기값·고정값으로. 캡처 해상도에 맞게 비례 조정.
        base = scale_to(pc.load(a.base), a.size)
        if base["phone"]["physical"] != [L, R]:
            raise ValueError(f"--base {a.base} 는 {base['phone']['physical']} 용 — --physical 불일치")
        for k in ("K1", "K2", "dist1", "dist2", "R", "T", "focal_scale_wide"):
            if k in base:
                st[k] = base[k]
        st["intrinsics_source"] = f"base:{a.base}"
        print(f"--base 적용: fx L {st['K1'][0,0]:.1f} R {st['K2'][0,0]:.1f} @ {a.size} (광각 스케일 {st.get('focal_scale_wide', 1.0):.4f})")
    if a.refine:
        pairs = pc.load_pairs(a.refine, L, R)
        nL, nR, counts = pc.collect_matches(pairs, st)
        print(f"장면 쌍 {len(pairs)}개, SIFT 대응점 {counts} (합 {len(nL)})")
        if len(nL) < 60:
            raise ValueError("대응점이 60개 미만 — 텍스처 있는 장면(방 전체, 책장 등) 쌍을 더 저장할 것")
        rows0, _ = pc.evaluate_pairs(pairs, st)
        report(rows0, "factory")
        Rn, Tn, s_wide, info = pc.refine_pose(nL, nR, st["R"], st["T"], float(st["K1"][0, 0]), fit_t=a.fit_t,
                                              fix_yaw=not a.free_yaw, fit_scale=not a.no_scale)
        st = pc.apply_scale(st, s_wide)
        st.update(R=Rn, T=Tn, pose_source="factory_baseline + scene_refined(roll,pitch,wide_scale)",
                  scale_status="factory_baseline_refined",
                  refine=dict(info, fit_t=bool(a.fit_t), pairs=[p[0] for p in pairs], counts=counts),
                  warnings=["baseline 15.76mm 는 공장값 그대로 (검증 안 됨)",
                            "yaw 는 공장값 그대로 (dy 로 관측 불가) -> 깊이 편향 가능. --known (실측 거리 1개) 또는 --board 로 고정",
                            "정류 dy 만 개선된 상태"])
        print(f"회전 보정 (x,y,z)={info['delta_rot_deg_xyz']}°  광각 fx 스케일 {s_wide:.4f}  T 방향 보정 {info['delta_t_dir_deg']:.3f}°  "
              f"Sampson 중앙값 {info['sampson_med_px']:.2f}px, <1px {info['inlier_frac_1px']:.2f}")
        rows1, _ = pc.evaluate_pairs(pairs, st)
        report(rows1, "refined")
        pos = np.median([r["pos_dx_frac"] for r in rows1 if r["status"] == "ok"] or [1.0])
        if pos < 0.85:
            st["warnings"].append(f"시차>0 비율 중앙값 {pos:.2f} — yaw 가 튄 징후. --free-yaw 를 쓰지 말고 기본으로 다시")
            print(f"   ! 시차>0 비율 {pos:.2f} < 0.85: yaw 가 잘못 움직였을 가능성")
    if a.known:
        if a.known_m is None or a.roi is None:
            raise ValueError("--known 은 --known-m <m> 과 --roi x y w h (왼쪽=초광각 정류영상 기준) 가 필요")
        if not a.refine and not a.board:
            st = pc.load(a.base or a.out)            # 기존 보정본 위에 yaw 만 고정
        (name, imL, imR), = pc.load_pairs([a.known], L, R)[:1]
        st, info = pc.pin_yaw_known_distance(st, imL, imR, a.roi, a.known_m)
        st["pose_source"] = st["pose_source"] + " + yaw_pinned(known_distance)"
        st["scale_status"] = "known_distance_pinned"
        st["warnings"] = ["baseline 15.76mm 는 공장값 그대로", f"yaw 를 실측 {a.known_m}m 물체 하나로 고정 — 다른 거리 1개로 교차 확인 권장"]
        print(f"yaw 고정: ROI 시차 {info['disparity_before_px']:.2f}px (깊이 {info['depth_before_m']}m) -> {info['disparity_after_px']:.2f}px "
              f"(목표 {info['target_px']:.2f}px = {a.known_m}m), yaw 보정 {info['yaw_delta_deg']:+.3f}°, 대응점 {info['n_matches']}")
    if a.board:
        if a.ruler_mm is None:
            raise ValueError("--board 는 --ruler-mm (기준 막대 실측 mm) 필수 — z_calibrate.py --make-target 의 막대")
        square_m = lib_calib.square_m_from_ruler(a.ruler_mm, a.screen)
        pairs = pc.load_pairs(a.board, L, R)
        st = pc.board_stereo_calibrate(pairs, st, square_m, fix_intrinsics=not a.free_intrinsics)
        b = st["board"]
        ratio = st["baseline_m"] / st["factory_baseline_m"]
        if b["sync_dropped"]:
            print(f"   동기 불일치(움직이는 중 촬영)로 제외 {len(b['sync_dropped'])}쌍 (게이트 {b['sync_gate_px']:.1f}px): {[Path(n).name for n, _ in b['sync_dropped']][:6]}")
        print(f"stereoCalibrate: 뷰 {b['n_views']} (건너뜀 {len(b['skipped'])}), RMS {b['rms_px']:.3f}px, "
              f"baseline {st['baseline_m']*1000:.2f}mm (공장 {st['factory_baseline_m']*1000:.2f}mm, 비 {ratio:.3f}), "
              f"상대회전 {pc.rotation_deg(st['R']):.2f}°")
        if b["per_view_px"]:
            worst = sorted(zip(b["per_view_px"], b["used"]), reverse=True)[:3]
            print("   뷰별 오차 최악 3:", [(round(e[0], 2), round(e[1], 2), Path(n).name) for e, n in worst])
        reasons = []
        if b["rms_px"] > BOARD_RMS_MAX_PX: reasons.append(f"rms {b['rms_px']:.2f}px > {BOARD_RMS_MAX_PX}")
        if b["n_views"] < BOARD_MIN_VIEWS: reasons.append(f"뷰 {b['n_views']} < {BOARD_MIN_VIEWS}")
        if abs(ratio - 1) > BOARD_BASELINE_TOL: reasons.append(f"baseline 이 공장값과 {abs(ratio-1)*100:.0f}% 차이")
        st["scale_status"] = "metric" if not reasons else "board_unreliable"
        st["warnings"] = reasons or ["체커보드 stereoCalibrate 통과 — 실측 거리 1개로 최종 확인 권장"]
        rows, _ = pc.evaluate_pairs(pairs, st)
        report(rows, "board")
    if a.refine or a.board:
        focus, why = pc.common_focus([p[0] for p in pairs])
        st["focus_m"] = focus
        if why:
            st.setdefault("warnings", []).append(f"초점 고정값 불명: {why} — 자동초점 번들이면 광각 fx 가 쌍마다 달라 dy 가 흔들린다")
    if a.target_size:
        st = scale_to(st, a.target_size)
    st["history"] = st.get("history", []) + [dict(pose_source=st["pose_source"], scale_status=st["scale_status"])]
    if st["scale_status"] == "board_unreliable":
        # 실패한 보드 결과로 멀쩡한 보정본을 덮어쓰면 라이브 정류가 바로 깨진다 (실제로 dy 5px 로 죽었다) → 옆 파일에만 남긴다
        alt = str(Path(a.out).with_suffix(".unreliable.json"))
        pc.save(alt, st)
        print(f"!! board_unreliable → {a.out} 은 그대로 두고 {alt} 에만 저장. 이유: {st['warnings']}")
        return
    pc.save(a.out, st)
    print(f"저장: {a.out}  scale_status={st['scale_status']}  phone={st['phone']}  image_size={st['image_size']}")
    for w in st.get("warnings", []):
        print("  !", w)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", default=str(pc.DEFAULT_INVENTORY), help="z_phone_stereo.py --list 결과 cameras.json")
    ap.add_argument("--logical", default="0")
    ap.add_argument("--physical", nargs=2, default=["2", "5"], help="(왼, 오른) 물리 id. 기본 초광각 2, 광각 5")
    ap.add_argument("--size", type=int, nargs=2, default=[640, 480], help="번들/스트림 해상도")
    ap.add_argument("--target-size", type=int, nargs=2, help="저장할 캘리 해상도 (고해상도로 보드 캘리 후 640 480 로 축소할 때)")
    ap.add_argument("--out", default="calib_phone_pair.json")
    ap.add_argument("--factory", action="store_true", help="공장값만 저장")
    ap.add_argument("--refine", nargs="+", metavar="DIR", help="장면 쌍 번들로 회전 보정")
    ap.add_argument("--fit-t", action="store_true", help="T 방향도 적합 (기본 끔: 15.76mm baseline 에서는 관측이 약해 흔들림)")
    ap.add_argument("--free-yaw", action="store_true", help="yaw 도 적합 (기본 끔: dy 로 관측 불가, 실쌍에서 -0.87° 로 튀어 시차 반전)")
    ap.add_argument("--no-scale", action="store_true", help="광각 fx 스케일 적합 끔 (기본 켬: AF 렌즈 초점 위치별 fx 변화 보상)")
    ap.add_argument("--known", metavar="DIR", help="실측 거리로 yaw 고정: 이 번들의 --roi 안 물체가 --known-m 에 있음")
    ap.add_argument("--known-m", type=float)
    ap.add_argument("--base", help="바탕 json: --known 단독이면 그 위에 yaw 만 고정, --board/--refine 이면 그 K/dist(광각 fx 스케일 포함)를 초기값으로")
    ap.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"), help="초광각(왼쪽) 정류영상 기준 ROI")
    ap.add_argument("--board", nargs="+", metavar="DIR", help="체커보드 번들로 stereoCalibrate")
    ap.add_argument("--screen", default="laptop", choices=sorted(lib_calib.SCREENS))
    ap.add_argument("--ruler-mm", type=float, help="화면 기준 막대 실측 길이(mm)")
    ap.add_argument("--free-intrinsics", action="store_true", help="K/dist 도 재추정 (뷰 20장 이상일 때만)")
    ap.add_argument("--check", nargs="+", metavar="DIR", help="--out 의 json 을 번들에서 검사만")
    a = ap.parse_args()
    if not (a.factory or a.refine or a.board or a.check or a.known):
        ap.error("--factory / --refine / --board / --known / --check 중 하나 필요")
    main(a)
