"""
lib_cam.py — 웹캠 열기 + 라이브 촬영 루프

z_calibrate.py(캘리 촬영)와 z_capture.py(본 촬영)가 똑같은 일을 한다:
  카메라 열기 → 라이브 창 → 오버레이 표시 → SPACE로 한 장 잡기 → Q로 종료.
다른 건 "무엇을 오버레이하나"와 "잡았을 때 뭘 하나" 둘뿐이므로
그 둘만 콜백으로 받고 루프는 여기 한 벌만 둔다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
해상도를 반드시 고정하는 이유
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  K(특히 fx, fy, cx, cy)는 픽셀 단위라 해상도에 종속된다.
  캘리브레이션 때와 본 촬영 때 해상도가 다르면 K가 통째로 틀린 값이 된다.
  그래서 SIZE를 한 곳에 고정하고, calib.json에 기록해 두었다가 재구성 시 대조한다.

  이 웹캠(HP Wide Vision HD)의 최대 해상도는 MJPG 1280x720 (v4l2-ctl 확인).
  YUYV는 640x480까지만 나오므로 MJPG를 강제한다.
"""
import time

import cv2

SIZE = (1280, 720)


def open_camera(index=0, size=SIZE, set_format=True):
    """웹캠을 열고 해상도를 고정한다. 실제로 적용됐는지 확인 후 반환.

    set_format=False 는 v4l2loopback 가상 장치(폰 카메라)용이다.
    그 장치의 포맷은 공급자(scrcpy)가 정하므로 이쪽에서 건드리면 안 된다.
    """
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise IOError(f"/dev/video{index} 열기 실패")
    if not set_format:
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
               int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != tuple(size):
            cap.release()
            raise IOError(f"가상 장치 해상도 {got} 가 기대값 {size} 와 다름 "
                          f"— scrcpy 의 --camera-size 를 확인할 것")
        return cap
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])

    # set()은 실패해도 예외를 안 던진다 — 실제 적용값을 읽어 확인한다.
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != tuple(size):
        cap.release()
        raise IOError(f"해상도 설정 실패: 요청 {size}, 실제 {got}")
    return cap


def read_pair(capL, capR):
    """두 카메라에서 한 쌍을 최대한 동시에 읽는다. 반환 (ok, frameL, frameR, skew_ms).

    grab() 둘을 먼저, retrieve() 둘을 나중에: 디코드 시간이 두 캡처 사이에 끼지 않게
    한다(OpenCV VideoCapture 문서의 다중 카메라 권장 순서). 하드웨어 노출 동기는
    아니다. skew_ms 는 두 grab 호출 사이 벽시계 차이이며 폰 전송 지연은 포함하지 않는다.
    예전엔 왼쪽을 read 하고 오른쪽을 따로 read 해 프리뷰와 저장 프레임까지 달랐다
    (doc_DEPTH_CODE_REVIEW R09). 프리뷰와 저장은 반드시 같은 쌍을 써야 한다.
    """
    t0 = time.perf_counter()
    okL = capL.grab()
    t1 = time.perf_counter()
    okR = capR.grab()
    skew = (t1 - t0) * 1000.0
    if not (okL and okR):
        return False, None, None, skew
    okL, fL = capL.retrieve()
    okR, fR = capR.retrieve()
    ok = bool(okL and okR and fL is not None and fR is not None)
    return ok, fL, fR, skew


def live_capture(cap, on_frame, on_grab, title="capture"):
    """라이브 창을 띄우고 SPACE로 프레임을 잡는다.

    on_frame(frame) -> 화면에 그릴 BGR 이미지
    on_grab(frame)  -> 잡았을 때 호출. 문자열을 반환하면 콘솔에 출력.
    Q 또는 ESC로 종료.
    """
    print("  [SPACE] 촬영   [Q/ESC] 종료")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("  프레임 읽기 실패")
                break
            cv2.imshow(title, on_frame(frame))
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord(" "):
                msg = on_grab(frame)
                if msg:
                    print("  " + msg)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def put_lines(img, lines, org=(10, 24), color=(0, 255, 0)):
    """좌상단에 여러 줄 텍스트. OpenCV putText는 한글 불가 → 영문/숫자만."""
    for i, t in enumerate(lines):
        cv2.putText(img, t, (org[0], org[1] + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, t, (org[0], org[1] + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
    return img
