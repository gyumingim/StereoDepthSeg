#!/usr/bin/env python3
"""
z_live.py — 실시간 객체 검출 + 거리·크기 (카메라 1대, 체커보드 평면 기준)

  python z_live.py
  python z_live.py --classes bottle cup --conf 0.4

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
세팅
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 폰에 target_s24.png 를 띄워 책상에 눕혀 둔다 (손에 들지 말 것).
  2. 측정할 물체를 "같은 책상 위"에 놓는다.
  3. 카메라(노트북)를 그쪽으로 향하게 두면 끝. 움직일 필요 없다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
되는 것과 안 되는 것
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  체커보드가 책상 평면을 정의하고, 물체가 그 평면에 "닿아 있다"는 조건을 쓴다.
  픽셀 하나가 정하는 광선과 평면의 교점은 하나뿐이므로 한 장으로 3D가 나온다.
  합성 검증(왜곡 없는 이상적 조건): 거리 오차 0.08~0.23%, 폭 0.04~0.92%,
  높이 0.30~1.06%.

  안 되는 것:
    - 공중에 뜬 물체, 다른 평면(선반·바닥) 위의 물체 -> 값이 틀린다
    - 박스 아랫변이 물체가 바닥에 닿는 선이 아닌 경우(가려짐 등)
    - 카메라가 평면을 정면으로 내려다보면 "높이"가 관측 불가해진다
      (HUD 의 hcond 가 작아지면 높이를 믿지 말 것)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
조작
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  G     자로 잰 정답값 입력 (터미널에 입력) -> truth_log.csv 에 쌓임
  S     현재 화면 저장
  Q/ESC 종료 (종료 시 누적 오차 통계 출력)
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import lib_calib as lc
import lib_cam
import lib_detect as ld
import lib_plane as lp

HERE = Path(__file__).parent
CALIB_JSON = HERE / "calib.json"
TRUTH_CSV = HERE / "truth_log.csv"
BOARD_EVERY = 3          # 보드는 고정돼 있으므로 매 프레임 찾을 필요가 없다


def _fmt(r):
    if r is None:
        return "평면 밖"
    h = "  ?" if not np.isfinite(r["height_m"]) else f"{r['height_m']*1000:.0f}"
    return f"{r['distance_m']*1000:.0f}mm  {r['width_m']*1000:.0f}x{h}mm"


def ask_truth(rows, log):
    """터미널에서 실측값을 받아 로그에 쌓는다. 화면은 잠시 멈춘다."""
    if not rows:
        print("  측정된 객체가 없다 — 입력할 것이 없음")
        return
    print("\n  현재 추정값:")
    for i, (det, r) in enumerate(rows, 1):
        print(f"    {i}. {det['label']:14s} {_fmt(r)}")
    print("  입력 형식:  <번호> <실측거리mm> [실측폭mm] [실측높이mm]   (엔터만 치면 취소)")
    try:
        line = input("  > ").strip()
    except EOFError:
        return
    if not line:
        return
    f = line.split()
    try:
        idx = int(f[0]) - 1
        assert 0 <= idx < len(rows)
        vals = [float(v) for v in f[1:]]
        assert vals
    except Exception:
        print("  형식이 맞지 않음 — 무시")
        return

    det, r = rows[idx]
    if r is None:
        print("  그 객체는 평면 밖이라 추정값이 없음 — 무시")
        return
    rec = dict(time=datetime.now().isoformat(timespec="seconds"), label=det["label"],
               est_dist_mm=round(r["distance_m"] * 1000, 1), true_dist_mm=vals[0],
               est_w_mm=round(r["width_m"] * 1000, 1),
               true_w_mm=vals[1] if len(vals) > 1 else "",
               est_h_mm=round(r["height_m"] * 1000, 1) if np.isfinite(r["height_m"]) else "",
               true_h_mm=vals[2] if len(vals) > 2 else "")
    log.append(rec)
    new = not TRUTH_CSV.exists()
    with TRUTH_CSV.open("a", newline="") as fp:
        wr = csv.DictWriter(fp, fieldnames=list(rec))
        if new:
            wr.writeheader()
        wr.writerow(rec)
    e = abs(rec["est_dist_mm"] - vals[0]) / vals[0] * 100
    print(f"  기록됨 — 거리 오차 {e:.2f}%  ({TRUTH_CSV.name} 에 누적 {len(log)}건)")


def stats(log):
    if not log:
        return
    print(f"\n{'='*58}\n누적 오차 ({len(log)}건)\n{'='*58}")
    for key, est, tru in (("거리", "est_dist_mm", "true_dist_mm"),
                          ("폭", "est_w_mm", "true_w_mm"),
                          ("높이", "est_h_mm", "true_h_mm")):
        e = [abs(r[est] - r[tru]) / r[tru] * 100
             for r in log if r[tru] != "" and r[est] != "" and float(r[tru]) > 0]
        if e:
            print(f"  {key:4s} n={len(e):3d}  평균 {np.mean(e):6.2f}%  "
                  f"중앙값 {np.median(e):6.2f}%  최대 {max(e):6.2f}%")


def main(a):
    if not CALIB_JSON.exists():
        sys.exit(f"{CALIB_JSON} 없음 — z_calibrate.py 를 먼저 돌릴 것")
    calib = lc.load_calib(CALIB_JSON)
    K, square_m = calib["K"], calib["square_m"]
    print(f"calib: fx {K[0,0]:.1f}px, 사각형 {square_m*1000:.4f}mm, "
          f"해상도 {calib['image_size']}")

    cap = lib_cam.open_camera(a.index)
    if tuple(calib["image_size"]) != lib_cam.SIZE:
        cap.release()
        sys.exit(f"캘리 해상도 {tuple(calib['image_size'])} 와 카메라 {lib_cam.SIZE} 불일치")

    state = dict(plane=None, frame_i=0, rows=[], fps=0.0, t=time.perf_counter())
    log = []

    def on_frame(frame):
        st = state
        st["frame_i"] += 1
        if st["frame_i"] % BOARD_EVERY == 1:
            c = lc.find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), fast=True)
            st["plane"] = (lp.board_plane(calib, c, square_m, lc.PATTERN)
                           if c is not None else None)

        dets = ld.detect_yolo(frame, model=a.model, conf=a.conf,
                              classes=a.classes, device=a.device)
        vis = frame.copy()
        rows = []
        for d in dets:
            r = (lp.object_from_box(K, d["box"], st["plane"], dist_coeffs=calib["dist"])
                 if st["plane"] else None)
            rows.append((d, r))
            x, y, w, h = d["box"]
            col = (0, 220, 80) if r else (0, 165, 255)
            cv2.rectangle(vis, (x, y), (x + w, y + h), col, 2)
            cv2.line(vis, (x, y + h), (x + w, y + h), col, 3)   # 평면에 닿는다고 본 선
            for t_, yy in ((f"{d['label']} {d['conf']:.2f}", y - 24), (_fmt(r), y - 6)):
                cv2.putText(vis, t_, (x, max(14, yy)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0), 3)
                cv2.putText(vis, t_, (x, max(14, yy)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, col, 1)
        st["rows"] = rows

        now = time.perf_counter()
        st["fps"] = 0.9 * st["fps"] + 0.1 / max(1e-6, now - st["t"])
        st["t"] = now
        pl = st["plane"]
        hud = [f"board: {'OK' if pl else '--'}", f"objects: {len(dets)}",
               f"{st['fps']:.0f} fps", "[G] truth  [S] save  [Q] quit"]
        if pl:
            hud.insert(1, f"board {pl['board_dist_m']*1000:.0f}mm "
                          f"pnp {pl['pnp_reproj_px']:.2f}px")
            if rows and rows[0][1]:
                hud.insert(2, f"hcond {rows[0][1]['height_cond']:.0f}")
        return lib_cam.put_lines(vis, hud,
                                 color=(0, 255, 0) if pl else (0, 165, 255))

    def on_grab(frame):
        ask_truth(state["rows"], log)
        return None

    try:
        print("  [G] 정답값 입력   [S] 화면 저장   [Q/ESC] 종료")
        while True:
            ok, frame = cap.read()
            if not ok:
                print("  프레임 읽기 실패")
                break
            vis = on_frame(frame)
            cv2.imshow("live depth", vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k in (ord("g"), ord("G")):
                ask_truth(state["rows"], log)
            if k in (ord("s"), ord("S")):
                p = HERE / f"live_{datetime.now():%H%M%S}.png"
                cv2.imwrite(str(p), vis)
                print(f"  저장 {p.name}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        stats(log)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=ld.DEFAULT_MODEL)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default=ld.DEFAULT_DEVICE,
                    help="0/cuda:0 은 GPU, cpu 는 CPU. 생략 시 자동")
    ap.add_argument("--classes", nargs="+", help="COCO 클래스 이름으로 필터")
    ap.add_argument("--index", type=int, default=0)
    main(ap.parse_args())
