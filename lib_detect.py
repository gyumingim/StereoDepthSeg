"""
lib_detect.py — 객체 검출 (수동 ROI / YOLO)

삼각측량 코어는 "ROI" 만 받으면 되므로, 검출기는 갈아끼울 수 있게 둔다.
두 검출기 모두 같은 형태를 반환한다:
    [{"box": (x, y, w, h), "label": str, "conf": float}, ...]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROI를 검출 박스보다 넓히는 이유
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SIFT 는 ROI 마스크 안에서만 특징점을 찾는다. 박스를 물체에 딱 맞추면
  가장자리 특징점이 잘려 점군이 물체보다 작아지고 크기가 과소평가된다.
  합성 실험(참값 300x200mm, 물체 폭 169px):
      마진 0px    -> 280.3 x 180.7 mm  (오차 6.6% / 9.7%)
      마진 8~50px -> 287.0 x 186.3 mm  (오차 4.3% / 6.9%)   <- 평탄해짐
      마진 90px+  -> 287.0 x 186.3 mm  (DBSCAN 배경분리가 있어 커져도 안전)
  넓혀서 들어온 배경은 lib_stereo.fit_box 의 DBSCAN 이 분리한다. 단 배경 특징점이
  물체보다 많으면 fit_box 가 배경 덩어리를 고를 수 있다(fit_box 주석의 한계).
  그래서 기본값을 박스 크기의 10%(최소 8px)로 작게 둔다. 이 확장은 희소 특징점
  탐색용이다 — 조밀 깊이 집계(z_object_depth)에는 적용하지 않는다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
YOLO 모델 기본값을 큰 것으로 두는 이유
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  이 파이프라인은 실시간 루프가 아니라 정지 사진 2장을 처리한다.
  추론 시간이 결과에 영향을 주지 않으므로 검출 정확도를 우선한다.
"""
import cv2
import numpy as np

DEFAULT_MODEL = "yolo11m.pt"
DEFAULT_SEG_MODEL = "yolo11m-seg.pt"   # 세그멘테이션 버전 (마스크 출력)
# predict 후 실제 사용된 장치가 여기 기록된다 (cpu / cuda:0)
DEFAULT_DEVICE = None      # None = ultralytics 자동 선택
_MODEL_CACHE = {}


def expand_box(box, img_shape, frac=0.10, min_px=8):
    """검출 박스를 이미지 경계 안에서 넓힌다. 이유는 모듈 docstring 참조."""
    x, y, w, h = (int(v) for v in box)
    mx = max(min_px, int(w * frac))
    my = max(min_px, int(h * frac))
    H, W = img_shape[:2]
    x0, y0 = max(0, x - mx), max(0, y - my)
    x1, y1 = min(W, x + w + mx), min(H, y + h + my)
    return (x0, y0, x1 - x0, y1 - y0)


def detect_manual(img, window="select objects (ENTER=확정, ESC=끝)"):
    """마우스로 드래그해 지정. 여러 개 가능."""
    rois = cv2.selectROIs(window, img)
    cv2.destroyAllWindows()
    return [{"box": tuple(int(v) for v in r), "label": f"roi{i+1}", "conf": 1.0}
            for i, r in enumerate(rois) if r[2] > 0 and r[3] > 0]


def detect_yolo(img, model=DEFAULT_MODEL, conf=0.25, classes=None, device=None,
                max_det=20):
    """YOLO 자동 검출. seg 모델(*-seg.pt)을 주면 각 검출에 "mask" 도 채운다.

    classes : COCO 클래스 이름 목록으로 걸러낸다 (예: ["bottle", "cup"]). None이면 전부.
    device  : None이면 ultralytics 기본(가능하면 GPU).
    mask    : HxW uint8 (0/255), 원본 이미지 해상도. 박스 모델이면 None.
              retina_masks=True 로 받은 res.masks.data 를 그대로 쓴다. 예전엔
              res.masks.xy 다각형을 fillPoly 로 채웠는데, xy 는 RETR_EXTERNAL 외곽선만이라
              **구멍이 사라진다** (80x80 에 60x60 구멍 -> 2800px 이 6400px 로, 구멍 너머
              배경 3m 가 섞여 중앙값 1m -> 3m. doc_DEPTH_CODE_REVIEW R02).
    """
    from ultralytics import YOLO          # 무거우므로 필요할 때만 import

    if model not in _MODEL_CACHE:
        _MODEL_CACHE[model] = YOLO(model)
    net = _MODEL_CACHE[model]

    # retina_masks=True: masks.data 가 원본 해상도(HxW)로 나온다 (설치본 8.4.157 에서
    # 확인: False 면 (384,640), True 면 (720,1280)). 박스 모델은 masks 가 None 이라 무해.
    kw = dict(conf=conf, max_det=max_det, verbose=False, retina_masks=True)
    if device is not None:
        kw["device"] = device
    res = net.predict(img, **kw)[0]
    # 실제로 어느 장치에서 돌았는지 남긴다 — "GPU 쓰는 줄 알았는데 CPU" 를 막는다.
    # net.device 는 predict 가 장치를 바꿔도 갱신되지 않아 거짓말을 한다(GPU로 돌려도
    # "cpu" 로 나왔다). 결과 텐서가 실제로 올라가 있는 장치를 봐야 맞다.
    try:
        detect_yolo.last_device = str(res.boxes.data.device)
    except Exception:
        detect_yolo.last_device = str(getattr(net, "device", "?"))
    names = res.names

    out = []
    if res.boxes is None:
        return out
    H, W = img.shape[:2]
    md = None
    if getattr(res, "masks", None) is not None and res.masks is not None:
        md = res.masks.data.cpu().numpy()
        if md.shape[1:] != (H, W):
            # 원본 해상도가 아니면 stretch 해서 맞추지 않는다 (letterbox 가 남은 마스크를
            # 늘리면 경계가 틀어진다). 마스크 없음으로 처리하고 호출측이 status 로 안다.
            md = None
    for i, b in enumerate(res.boxes):
        cls = int(b.cls.item())
        label = names[cls]
        if classes and label not in classes:
            continue
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
        mask = None
        if md is not None and i < len(md):
            mask = (md[i] > 0).astype(np.uint8) * 255
        out.append({"box": (int(x1), int(y1), int(x2 - x1), int(y2 - y1)),
                    "label": label, "conf": float(b.conf.item()), "mask": mask})
    out.sort(key=lambda d: -d["conf"])
    return out


def draw(img, dets, color=(0, 220, 80)):
    """검출 결과를 그린 사본을 돌려준다 (검수용)."""
    vis = img.copy()
    for d in dets:
        x, y, w, h = d["box"]
        if d.get("mask") is not None:                     # seg 면 윤곽선도 그린다
            cs, _ = cv2.findContours(d["mask"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, cs, -1, color, 2)
        cv2.rectangle(vis, (x, y), (x + w, y + h), color, 1)
        txt = f"{d['label']} {d['conf']:.2f}"
        cv2.putText(vis, txt, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3)
        cv2.putText(vis, txt, (x, max(14, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 1)
    return vis
