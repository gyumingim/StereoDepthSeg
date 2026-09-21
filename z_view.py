#!/usr/bin/env python3
"""
z_view.py — 두 카메라를 나란히 보여준다 (촬영 없이 보기만)

  python z_view.py                          # 노트북 웹캠 + 폰(/dev/video9)
  python z_view.py --index-l 0 --index-r 9

폰을 노트북에 고정할 때 쓴다. 두 카메라가 같은 장면을 충분히 겹쳐 보고 있어야
스테레오가 되는데, 그걸 눈으로 확인할 방법이 필요하다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
화면에 나오는 것
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  match : 두 화면에서 같은 지점으로 매칭된 특징점 수 (ORB, 몇 프레임마다 갱신).
          이 값이 크면 겹침이 충분하다. 0에 가까우면 두 카메라가 서로 다른 곳을
          보고 있거나 장면에 질감이 없는 것이다.
  fps   : 표시 속도

조작: S 현재 화면 저장 / Q,ESC 종료

먼저 폰 카메라를 띄워둘 것:  ./tools/phonecam.sh run 2
"""
import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import lib_cam

HERE = Path(__file__).parent
MATCH_EVERY = 10          # ORB 매칭은 매 프레임 할 필요가 없다


def main(a):
    capL = lib_cam.open_camera(a.index_l, set_format=not a.loopback_l)
    capR = lib_cam.open_camera(a.index_r, set_format=not a.loopback_r)
    orb = cv2.ORB_create(600)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    st = dict(i=0, n_match=-1, fps=0.0, t=time.perf_counter())

    print("  [S] 저장   [Q/ESC] 종료")
    try:
        while True:
            ok, fL, fR, skew = lib_cam.read_pair(capL, capR)
            if not ok:
                print("  프레임 읽기 실패")
                break

            st["i"] += 1
            if st["i"] % MATCH_EVERY == 1:
                kL, dL = orb.detectAndCompute(cv2.cvtColor(fL, cv2.COLOR_BGR2GRAY), None)
                kR, dR = orb.detectAndCompute(cv2.cvtColor(fR, cv2.COLOR_BGR2GRAY), None)
                if dL is None or dR is None or len(kL) < 2 or len(kR) < 2:
                    st["n_match"] = 0
                else:
                    knn = bf.knnMatch(dL, dR, k=2)
                    st["n_match"] = sum(1 for m, n in knn if m.distance < 0.75 * n.distance)

            h = 400
            sL = cv2.resize(fL, (int(fL.shape[1] * h / fL.shape[0]), h))
            sR = cv2.resize(fR, (int(fR.shape[1] * h / fR.shape[0]), h))
            pair = np.hstack([sL, np.full((h, 4, 3), 60, np.uint8), sR])

            now = time.perf_counter()
            st["fps"] = 0.9 * st["fps"] + 0.1 / max(1e-6, now - st["t"])
            st["t"] = now
            col = (0, 255, 0) if st["n_match"] >= 30 else (0, 165, 255)
            lib_cam.put_lines(pair, [
                f"L=/dev/video{a.index_l} {fL.shape[1]}x{fL.shape[0]}   "
                f"R=/dev/video{a.index_r} {fR.shape[1]}x{fR.shape[0]}",
                f"match: {st['n_match']}   {st['fps']:.0f} fps   grab skew {skew:.1f} ms",
                "[S] save  [Q] quit"], color=col)
            cv2.imshow("two cameras", pair)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k in (ord("s"), ord("S")):
                p = HERE / f"view_{datetime.now():%H%M%S}.png"
                cv2.imwrite(str(p), pair)
                print(f"  저장 {p.name}")
    finally:
        capL.release()
        capR.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--index-l", type=int, default=0, help="왼쪽 (노트북 웹캠)")
    ap.add_argument("--index-r", type=int, default=9, help="오른쪽 (폰 = /dev/video9)")
    ap.add_argument("--loopback-l", action="store_true")
    ap.add_argument("--loopback-r", action="store_true", default=True,
                    help="오른쪽이 v4l2loopback 가상장치 (기본 켜짐)")
    main(ap.parse_args())
