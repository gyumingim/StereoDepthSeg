#!/usr/bin/env python3
"""웹 화면 미리보기 서버 — 가짜 데이터 (폰·GPU·OpenVINS 없이 scan3d/web 화면만 확인).

진짜 scan3d/web 파일을 그대로 보내고 /api/* 는 가짜로 답한다: 시간이 지나며 "자라는" 작은 방(복셀), 물체 목록,
궤적, 카메라 창 두 개(/api/cam.jpg, 그라데이션 + 움직이는 상자 + 렌즈 이름), 깊이 화면(/api/frame.jpg?kind=depth).
/api/combos 는 진짜 load_model.py 로 계산 (폰 능력표는 캐시·cameras_s24.json·기본값), 실행 중 부하(ui.load)는 예상값에
흔들림을 준 가짜 실측 — 폰 압축 부하가 40 % 를 넘는 조합이면 20 초 뒤 과부하 + [한 단계 낮추기] 제안을 보여 준다.
상태 JSON 에 preview:true 를 넣으므로 웹 위쪽에 "미리보기 — 가짜 데이터" 띠가 보이고, 폰 온도에도 "(가짜)" 가 붙는다.

실행 (numpy 필요, 그림 글자는 Pillow 또는 OpenCV 가 있으면 그림 — 없어도 동작):
  python scan3d/preview_server.py            → http://localhost:8796/
  python scan3d/preview_server.py --port 8797 --web <web 폴더>
Windows 에서:  python \\\\wsl.localhost\\Ubuntu-22.04\\home\\system\\scanner\\StereoDepthSeg\\scan3d\\preview_server.py
"""
import argparse
import json
import math
import os
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    import load_model
    LM = load_model.LoadModel()
except Exception as _e:                       # 없어도 화면은 뜨게
    load_model = LM = None
    print(f"(load_model 없음: {_e!r} — /api/combos 는 640x480·15 fps 만)", flush=True)

WEB = Path(__file__).resolve().parent / "web"
PORT = 8796
VM = 0.03
DT = np.dtype([("ix", "<i2"), ("iy", "<i2"), ("iz", "<i2"), ("r", "u1"), ("g", "u1"), ("b", "u1"),
               ("cls", "<i2"), ("inst", "<i4"), ("hits", "<u2")])
NAMES = {0: "person", 41: "cup", 56: "chair", 60: "dining table", 62: "tv", 63: "laptop", 58: "potted plant"}
PAL = {0: (255, 56, 56), 41: (255, 157, 151), 56: (255, 178, 29), 60: (72, 249, 10), 62: (0, 194, 255),
       63: (52, 69, 147), 58: (61, 219, 134)}
LENSES = [dict(lens=0, physical="2", name="초광각"), dict(lens=1, physical="5", name="광각")]
CTYPE = {".html": "text/html; charset=utf-8", ".js": "application/javascript", ".css": "text/css",
         ".json": "application/json", ".png": "image/png", ".ico": "image/x-icon"}
T0 = time.time()


def box(x0, x1, y0, y1, z0, z1, rgb, cls=-1, inst=-1, shell=True):
    xs = np.arange(int(x0 / VM), int(x1 / VM))
    ys = np.arange(int(y0 / VM), int(y1 / VM))
    zs = np.arange(int(z0 / VM), int(z1 / VM))
    g = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), -1).reshape(-1, 3)
    if shell and len(g):
        mn, mx = g.min(0), g.max(0)
        g = g[((g == mn) | (g == mx)).any(1)]
    out = np.zeros(len(g), DT)
    out["ix"], out["iy"], out["iz"] = g[:, 0], g[:, 1], g[:, 2]
    jit = np.random.randint(-12, 12, (len(g), 3))
    c = np.clip(np.array(rgb) + jit, 0, 255)
    out["r"], out["g"], out["b"] = c[:, 0], c[:, 1], c[:, 2]
    out["cls"], out["inst"], out["hits"] = cls, inst, np.random.randint(2, 30, len(g))
    return out


np.random.seed(1)
SCENE = np.concatenate([
    box(-1.5, 2.5, -1.0, 3.0, -0.03, 0.0, (150, 140, 120), shell=False),          # 바닥
    box(-1.5, 2.5, 3.0, 3.03, 0.0, 2.4, (215, 215, 205), shell=False),            # 뒷벽
    box(-1.53, -1.5, -1.0, 3.0, 0.0, 2.4, (205, 210, 215), shell=False),          # 왼쪽 벽
    box(0.2, 1.4, 1.2, 2.0, 0.70, 0.75, (140, 100, 60), 60, 1, shell=False),      # 탁자 윗판
    box(0.25, 0.3, 1.25, 1.3, 0.0, 0.70, (120, 85, 50), 60, 1), box(1.3, 1.35, 1.9, 1.95, 0.0, 0.70, (120, 85, 50), 60, 1),
    box(-0.4, 0.05, 1.4, 1.85, 0.0, 0.45, (60, 60, 70), 56, 2), box(-0.4, -0.35, 1.4, 1.85, 0.45, 0.9, (60, 60, 70), 56, 2),
    box(0.6, 0.66, 1.5, 1.56, 0.75, 0.87, (240, 240, 240), 41, 3),                # 컵
    box(0.9, 1.25, 1.4, 1.65, 0.75, 0.78, (40, 40, 45), 63, 4),                   # 노트북
    box(1.6, 2.4, 2.9, 2.97, 0.9, 1.4, (20, 20, 25), 62, 5, shell=False),         # TV
    box(-1.2, -0.9, 2.5, 2.8, 0.0, 0.8, (60, 140, 70), 58, 6),                    # 화분
])
SCENE = SCENE[np.argsort(np.hypot(SCENE["ix"] * VM, SCENE["iy"] * VM))]          # 시작점에서 바깥으로 드러남
S = {"paused": False, "mode": "live", "epoch": 1, "t0": time.time(), "size": "640x480", "fps": 15, "fmt": "h264"}
LOCK = threading.Lock()


def since_start():
    return time.time() - S["t0"]


def shown():
    if S["mode"] not in ("live", "both"):
        return 0
    return int(min(len(SCENE), max(0, (since_start() - 4) * 900)))


def objects(n):
    sub = SCENE[:n]
    out = []
    for inst in sorted(set(sub["inst"].tolist()) - {-1}):
        m = sub["inst"] == inst
        if m.sum() < 5:
            continue
        k = np.stack([sub["ix"][m], sub["iy"][m], sub["iz"][m]], 1) * VM
        c = int(sub["cls"][m][0])
        mn, mx = k.min(0), k.max(0) + VM
        out.append(dict(id=int(inst), comp=int(inst), class_id=c, label=NAMES[c], color=list(PAL[c]), n_voxels=int(m.sum()),
                        volume_m3=float(m.sum() * VM ** 3), last_seen=time.time(), conf=0.8,
                        box_center=((mn + mx) / 2).tolist(), box_size=(mx - mn).tolist(), yaw=0.0))
    return out


def pose(t):
    return [1, 0, 0, 0.3 * math.sin(t / 5), 0, 1, 0, 0.6 + 0.3 * math.sin(t / 7), 0, 0, 1, 1.2, 0, 0, 0, 1]


def state():
    t = since_start()
    n = shown()
    live = S["mode"] in ("live", "both")
    vio = dict(addr="pipe(가짜)", init=live and t >= 4, status=("tracking" if t >= 4 else "init") if live else "idle",
               mono=False, t=t, n_slam=24, n_msckf=40, ms=6.2, dt_cam_imu=-0.004,
               imu_sent=int(t * 400), cams_sent=int(t * 30), poses=int(t * 30), cams_dropped=0,
               stereo_frames=int(t * 30) if live else 0, mono_frames=0, auto_resets=0, error=None, vio_mode="stereo",
               skew_med_ms=2.1, skew_p95_ms=4.4, skew_limit_ms=5.0, pair_skew_ms=5.0, skew_n=int(t * 30) if live else 0,
               skew_dropped=3, bridge_restarts=0, bridge_exceptions=0, bridge_fails=0,
               T_world_rect0=pose(t) if live else None)
    phone = dict(frames=[int(t * 30)] * 2, fps=[29.8, 29.8] if live else [], pairs=int(t * 30), dropped=12, reordered=0,
                 skew_ms=2.1, skew_p95_ms=4.4, clock_gap_ms=0.3, error=None,
                 imu_rates={"accelerometer": 400.0, "gyroscope": 400.0}, imu_sync=dict(out=int(t * 400), dropped=0),
                 pair_skew_ms=5.0, connected=live)
    depth = dict(ffs_ms=48.6, yolo_ms=22.2, total_ms=96.6, keyframes=int(t), points=n * 3, fps=8.8, ready=t >= 8,
                 loading=t < 8, loaded=True, error=None, iters=4, n_dets=4)
    session = dict(mode=S["mode"], link="relay", phone_spec="listen:8766", adb=dict(available=True, kind="windows", adb="(가짜)", app_mode=S["mode"]),
                   app=dict(raw=f"STREAMING mode={S['mode']}", state="STREAMING", mode=S["mode"], ok=True) if S["mode"] != "idle" else None,
                   rec=dict(state="RECORDING", raw="RECORDING rec=rec_가짜", elapsed_s=round(t, 1)) if S["mode"] in ("record", "both") else None,
                   pull=None, replay=None, last_result=None, rec_root="recordings", cams=LENSES)
    temp = round(37.5 + min(6.0, t / 40), 1)
    tlvl = "hot" if temp > 45 else "warm" if temp > 42 else "ok"
    phone["fps"] = [S["fps"] - 0.2] * 2 if live else []
    return dict(preview=True, t=time.time(), paused=S["paused"], vio=vio, phone=phone, depth=depth, session=session,
                map=dict(epoch=S["epoch"], version=n, voxels=n, shown=n, voxel_m=VM, min_hits=2, frames=int(t)),
                objects=objects(n), epi=dict(status="ok", dy_med=1.16, inlier_frac=0.73),
                classes={str(k): dict(name=v, color=list(PAL[k])) for k, v in NAMES.items()},
                log=[time.strftime("%H:%M:%S ") + "[preview] 가짜 데이터 서버 — 실제 폰·GPU 를 쓰지 않습니다"],
                settings=dict(depth_range=[0.3, 2.5], stride=3, iters=4, replay_iters=8, kf_trans=0.03, kf_rot=5.0),
                ui=dict(usb="device", usb_age_s=1.0, temp_c=temp, temp_level="hot" if temp > 45 else "warm" if temp > 42 else "ok",
                        temp_age_s=2.0, fake=True, stream=fake_stream(), load=fake_load(t, tlvl),
                        relay=dict(state="켜짐", pid=1234, error=None, starts=1, fails=0, wanted=True,
                                   log_tail=["(가짜) phone 127.0.0.1:8765 <-> WSL relay running"])))


def fake_stream():
    w, _, h = S["size"].partition("x")
    lk = LM.predict(int(w), int(h), S["fps"], S["fmt"]).get("live_kbps") if LM is not None else None
    return dict(size=S["size"], w=int(w), h=int(h), fps=S["fps"], calib="640x480", scaled_calib=S["size"] != "640x480",
                stream_fmt=S["fmt"], live_kbps=lk)


def fake_combos(mode, kf=None, fmt=None):
    fmt = fmt or S["fmt"]
    if LM is not None:
        return LM.table(mode, kf_min=kf, stream_fmt=fmt, current=dict(size=S["size"], fps=S["fps"], stream_fmt=S["fmt"]))
    c = dict(ok=True, reasons=[], notes=["미리보기 — load_model 없음"], stream_fmt="h264", phone_jpeg_pct=0.0, phone_jpeg_ms=0.0,
             phone_cpu_pct=2.8, phone_copy_ms=0.9, live_kbps=4000, usb_mbs=1.25, pc_dec_ms=1.5, pc_rx_pct=7.4,
             pc_kf_ms=72.0, pc_ffs_fps=13.9, rec_kbps=4000, enc_n=2, enc_kind="live")
    return dict(mode=mode, stream_fmt="h264", default_fmt="h264",
                fmts=[dict(id="h264", label="H.264 (권장)"), dict(id="jpeg", label="JPEG"), dict(id="nv21", label="NV21 (압축 없음)")],
                sizes=[dict(size="640x480", w=640, h=480, mpx=0.31, calib=True)], fps=[15],
                combos={"640x480@15": c}, default=dict(size="640x480", fps=15), current=dict(size=S["size"], fps=S["fps"]),
                calib="640x480", caps_src="가짜", limits=dict(phone_jpeg_pct=60, usb_mbs=8, kf_min=3), coeffs=None)


def fake_load(t, tlvl):
    if S["mode"] not in ("live", "record", "both") or LM is None:
        return dict(active=False)
    w, _, h = S["size"].partition("x")
    w, h, fps, fmt = int(w), int(h), S["fps"], S["fmt"]
    pred = LM.combo(S["mode"], w, h, fps, None, fmt)
    j = 1.0 + 0.08 * math.sin(t / 3)
    live = S["mode"] in ("live", "both")
    meas = dict(phone_cpu_pct=round(pred["phone_cpu_pct"] * 0.9 * j, 1), cam_fps=round(fps - 0.2, 1),
                usb_mbs=round(pred["usb_mbs"] * 0.75 * j, 2), drop_pct=0.0, enc_dropped=0) if live else dict(cam_fps=round(fps - 0.2, 1))
    if live and fmt == "jpeg":
        meas.update(phone_jpeg_pct=round(pred["phone_jpeg_pct"] * 0.9 * j, 1), phone_jpeg_ms=round(pred["phone_jpeg_ms"] * 0.9 * j, 1),
                    phone_jpeg_ms_max=round(pred["phone_jpeg_ms"] * 1.8, 1))
    if live and fmt != "jpeg":
        meas["phone_copy_ms"] = round(pred["phone_copy_ms"] * 0.9 * j, 2)
    if live and fmt == "h264":
        # 가짜 실측: VBR 이라 목표보다 조금 낮게, 인코더 지연은 프레임 간격의 약 1/3
        meas.update(live_kbps=round(pred["live_kbps"] * 0.85 * j), enc_ms=round(1000 / fps * 0.33 * j, 1),
                    enc_ms_max=round(1000 / fps * 0.6, 1))
    if live and pred.get("pc_dec_ms"):
        meas.update(pc_dec_ms=round(pred["pc_dec_ms"] * 0.8 * j, 1), pc_dec_ms_max=round(pred["pc_dec_ms"] * 1.6, 1))
    if live:
        meas.update(pc_kf_ms=round(pred["pc_kf_ms"] * j, 1), pc_ffs_fps=round(1000 / max(1.0, pred["pc_kf_ms"] * j), 1))
    flags = []
    if live and pred["phone_cpu_pct"] > 40 and t > 20:
        meas["drop_pct"] = 9.0
        flags.append("폰이 프레임을 버림 (9%) — 가짜")
    if tlvl == "hot":
        flags.append("폰이 뜨거움")
    elif tlvl == "warm":
        flags.append("폰이 따뜻함")
    over = any(f != "폰이 따뜻함" for f in flags)
    return dict(active=True, mode=S["mode"], stream_fmt=fmt, size=S["size"], fps=fps, predicted=pred, measured=meas, flags=flags,
                overload=over, suggest=LM.step_down(S["mode"], w, h, fps, None, fmt) if over else None, coeff_updated=LM.updated)


# ---------------- 가짜 그림 (PNG: 표준 라이브러리 zlib 만으로) ----------------
def png_bytes(rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b""))


def encode(rgb):
    """(바이트, content-type): OpenCV 있으면 JPEG, 없으면 PNG."""
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(rgb[:, :, ::-1]), [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            return buf.tobytes(), "image/jpeg"
    except Exception:
        pass
    return png_bytes(np.ascontiguousarray(rgb)), "image/png"


_FONT = None


def put_text(rgb, xy, text, size=26, color=(255, 255, 255)):
    """한글 글자는 Pillow + 맑은 고딕이 있을 때만. 없으면 OpenCV 영문, 그것도 없으면 그리지 않음 (웹 창 제목이 렌즈 이름을 보여 줌)."""
    global _FONT
    try:
        from PIL import Image, ImageDraw, ImageFont
        if _FONT is None:
            for fp in (r"C:\Windows\Fonts\malgun.ttf", "/mnt/c/Windows/Fonts/malgun.ttf",
                       "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"):
                if os.path.isfile(fp):
                    _FONT = ImageFont.truetype(fp, size)
                    break
            else:
                _FONT = False
        if _FONT:
            im = Image.fromarray(rgb)
            d = ImageDraw.Draw(im)
            d.text((xy[0] + 2, xy[1] + 2), text, font=_FONT, fill=(0, 0, 0))
            d.text(xy, text, font=_FONT, fill=color)
            rgb[:] = np.asarray(im)
            return
    except Exception:
        pass
    try:
        import cv2
        ascii_text = text.encode("ascii", "ignore").decode() or "FAKE"
        cv2.putText(rgb, ascii_text, (xy[0], xy[1] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(rgb, ascii_text, (xy[0], xy[1] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    except Exception:
        pass


def rect(rgb, x0, y0, x1, y1, color, fill=False, th=3):
    h, w, _ = rgb.shape
    x0, x1 = sorted((max(0, min(w, int(x0))), max(0, min(w, int(x1)))))
    y0, y1 = sorted((max(0, min(h, int(y0))), max(0, min(h, int(y1)))))
    if fill:
        rgb[y0:y1, x0:x1] = color
        return
    rgb[y0:y0 + th, x0:x1] = color
    rgb[max(y0, y1 - th):y1, x0:x1] = color
    rgb[y0:y1, x0:x0 + th] = color
    rgb[y0:y1, max(x0, x1 - th):x1] = color


def fake_cam(lens, overlay):
    """센서 방향 그대로(누운) 640x480 가짜 장면: 그라데이션 벽 + 바닥 + 움직이는 컵 상자 + 렌즈 이름."""
    w, h = 640, 480
    t = since_start()
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    base = np.array([70, 90, 120] if lens == 0 else [110, 90, 70], np.float32)
    img = base + 60 * (xx / w)[..., None] * np.array([1.0, 0.6, 0.2]) + 40 * (yy / h)[..., None]
    floor = xx > 420 + 15 * math.sin(t / 3)                    # 누운 영상: 바닥이 오른쪽
    img[floor] = img[floor] * 0.55 + np.array([150, 140, 120]) * 0.45
    img = np.clip(img, 0, 255).astype(np.uint8)
    shift = 40 if lens == 0 else 0                             # 두 렌즈 시야가 조금 다름
    cx = 300 + 120 * math.sin(t / 2.5) + shift
    cy = 240 + 80 * math.cos(t / 3.1)
    rect(img, cx - 45, cy - 30, cx + 45, cy + 30, (235, 235, 235), fill=True)        # 컵 (누운)
    rect(img, 120 + shift, 80, 260 + shift, 170, (60, 60, 70), fill=True)              # 의자
    if overlay and lens == 0:
        for (x0, y0, x1, y1, cls) in ((cx - 50, cy - 35, cx + 50, cy + 35, 41), (115 + shift, 75, 265 + shift, 175, 56)):
            col = PAL[cls]
            sub = img[int(max(0, y0)):int(max(0, y1)), int(max(0, x0)):int(max(0, x1))]
            sub[:] = (0.6 * sub + 0.4 * np.array(col)).astype(np.uint8)
            rect(img, x0, y0, x1, y1, col)
            put_text(img, (int(x0), int(max(0, y0 - 28))), f"{NAMES[cls]} #{3 if cls == 41 else 2}", 20, col)
    name = LENSES[lens]
    put_text(img, (14, 10), f"{name['name']} (렌즈 {name['physical']}) — 가짜")
    put_text(img, (14, h - 40), time.strftime("%H:%M:%S"), 22, (255, 230, 120))
    return img


def fake_depth():
    w, h = 640, 480
    t = since_start()
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    z = 0.5 + 0.5 * np.sin(xx / 90 + t / 2) * np.cos(yy / 70)
    img = np.stack([255 * z, 120 + 100 * (1 - z), 255 * (1 - z)], -1).astype(np.uint8)
    put_text(img, (14, 10), "깊이 — 가짜")
    return img


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def js(self, obj):
        self.send(200, json.dumps(obj, ensure_ascii=False).encode("utf-8"), "application/json")

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if u.path == "/api/state":
                with LOCK:
                    return self.js(state())
            if u.path == "/api/voxels":
                with LOCK:
                    n, ep = shown(), S["epoch"]
                since = int(q.get("since", 0))
                if int(q.get("epoch", -1)) != ep:
                    since = 0
                rec = SCENE[min(since, n):n]
                return self.send(200, struct.pack("<IIIf", ep, n, len(rec), VM) + rec.tobytes(), "application/octet-stream")
            if u.path == "/api/trajectory":
                t = since_start()
                live = S["mode"] in ("live", "both")
                pts = [[round(i * 0.1, 3), 0.3 * math.sin(i * 0.1 / 5), 0.6 + 0.3 * math.sin(i * 0.1 / 7), 1.2]
                       for i in range(int(t * 10))][::3] if live else []
                return self.js(dict(points=pts, T_world_rect0=pose(t) if live else None,
                                    cam=dict(f=480.0, cx=320.0, cy=240.0, w=640, h=480)))
            if u.path == "/api/combos":
                try:
                    kf = float(q["kf"]) if q.get("kf") else None
                except ValueError:
                    kf = None
                return self.js(fake_combos(q.get("mode", "live"), kf, q.get("fmt")))
            if u.path == "/api/recordings":
                return self.js(dict(
                    phone=[dict(name="rec_20261016_101500", base="/sdcard/(가짜)", size_kb=217498, local=False)],
                    phone_error=None, phone_t=time.time(),
                    local=[dict(name="rec_20261016_093000", path="recordings/rec_20261016_093000 (가짜)", size_mb=180.2,
                                recon=["recon_20261016_094500"], has_video=True)],
                    rec_root="recordings (가짜)"))
            if u.path == "/api/cam.jpg":
                if S["mode"] not in ("live", "both", "replay"):
                    return self.send(204, b"", "text/plain")
                lens = 1 if q.get("lens") == "1" else 0
                body, ct = encode(fake_cam(lens, q.get("overlay") == "1"))
                return self.send(200, body, ct)
            if u.path == "/api/frame.jpg":
                if S["mode"] not in ("live", "both"):
                    return self.send(204, b"", "text/plain")
                body, ct = encode(fake_depth())
                return self.send(200, body, ct)
            path = "index.html" if u.path in ("/", "") else u.path.lstrip("/")
            f = (WEB / path).resolve()
            if WEB.resolve() not in f.parents or not f.is_file():
                return self.send(404, b"not found", "text/plain")
            return self.send(200, f.read_bytes(), CTYPE.get(f.suffix, "application/octet-stream"))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            body = {}
        c = body.get("cmd")
        with LOCK:
            if c in ("reset_map", "reset_vio", "restart"):
                S["epoch"] += 1
                S["t0"] = time.time()
            elif c == "mode":
                if body.get("size") or body.get("fps") or body.get("stream_fmt"):
                    try:
                        size = str(body.get("size") or S["size"])
                        fps = int(body.get("fps") or S["fps"])
                        fmt = body.get("stream_fmt") or S["fmt"]
                        if LM is not None:
                            fmt = load_model.norm_fmt(fmt)
                        w, _, h = size.partition("x")
                        cb = LM.combo(body.get("mode", "live"), int(w), int(h), fps, None, fmt) if LM is not None else dict(ok=True)
                    except (TypeError, ValueError):
                        return self.js(dict(ok=False, error="화질·fps 값이 잘못됐습니다", preview=True))
                    if not cb["ok"] and not body.get("force"):
                        return self.js(dict(ok=False, preview=True, reasons=cb["reasons"],
                                            error=f"{size} · {fps} fps ({fmt}) 는 지금 쓸 수 없습니다: " + " · ".join(cb["reasons"])))
                    S["size"], S["fps"], S["fmt"] = size, fps, fmt
                S["mode"] = body.get("mode", "live")
                S["epoch"] += 1
                S["t0"] = time.time()
            elif c in ("stop",):
                S["mode"] = "idle"
            elif c == "pause":
                S["paused"] = True
            elif c == "resume":
                S["paused"] = False
            elif c == "save":
                return self.js(dict(ok=True, dir="(미리보기 — 저장 안 함)", preview=True))
            elif c == "load_reset":
                return self.js(dict(ok=True, preview=True, note="미리보기 — 계수 파일은 건드리지 않음"))
            elif c == "caps_refresh":
                return self.js(dict(ok=True, preview=True, caps_src=LM.caps_src if LM is not None else "가짜"))
            elif c in ("pull", "replay"):
                return self.js(dict(ok=False, error="미리보기에서는 녹화 가져오기·복원을 하지 않습니다", preview=True))
        return self.js(dict(ok=True, preview=True, paused=S["paused"]))


def main():
    global WEB
    ap = argparse.ArgumentParser(description="scan3d 웹 화면 미리보기 (가짜 데이터)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--web", default=str(WEB), help="web 폴더 (기본: 이 파일 옆 web/)")
    a = ap.parse_args()
    WEB = Path(a.web)
    print(f"미리보기 (가짜 데이터): http://localhost:{a.port}/   web: {WEB}", flush=True)
    srv = ThreadingHTTPServer((a.host, a.port), H)
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
