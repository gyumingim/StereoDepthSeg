"""화질(해상도)·fps 조합별 부하 모델 — 폰(카메라·JPEG·H.264·USB) + PC(수신·FFS·YOLO). 표준 라이브러리만.

측정을 따로 돌리지 않는다. 대신
  (a) 폰 능력표 caps.json  — 앱이 Camera2 StreamConfigurationMap(물리 카메라별 YUV 크기·getOutputMinFrameDuration)과
      MediaCodecInfo.VideoCapabilities(areSizeAndRateSupported·비트레이트 범위)를 "조회만" 해서 쓴 파일.
      adb run-as 로 읽어 runs/phone_caps.json 에 캐시. 없으면 phone_stereo/cameras_s24.json (YUV 크기·AE fps 범위),
      그것도 없으면 기본 후보 목록 (추정).
  (b) 실행 중 실제 지표 (폰 perf.json / state.phone.perf, PC state.depth) → 계수를 EWMA 로 갱신, runs/load_coeffs.json 에 저장.
  (c) 조합별 {가능 여부, 이유, 예상 폰 JPEG 부하 %, 예상 USB MB/s, 예상 녹화 kbps, 예상 PC 깊이 fps}.

해상도는 4:3 만 (보정 640x480 과 같은 비율·같은 센서 영역 → fx,fy,cx,cy 를 크기 비율로 곱하고 왜곡·R·T 는 그대로).
16:9 등은 목록에서 뺀다.

---- 앱 파일 형식 (실제, SPEC.md 14) ----
앱 1.3-caps 는 caps.json (format "scan3d-phone-caps/1": logical.fixed_fps, lenses{id:{yuv[{w,h,min_frame_ns,full_active_array}]}},
pairs[{w,h,max_fps,camera_fps[],record_encoder{fps:name|null},rec_kbps_default{fps:kbps}}], avc_encoders[]) 와
metrics.json (format "scan3d-phone-metrics/1": width,height,fps, lenses{id:{cam_fps,jpeg_ms_avg,jpeg_ms_max,jpeg_kb_avg,...}},
jpeg_core_pct_total, usb.MBps, phone.battery_temp_c/thermal_status) 를 /sdcard/Android/data/<앱>/files/ 에 쓴다 (run-as 없이 cat).
normalize_caps() / perf_from_metrics() 가 이것을 아래 내부 형식으로 바꾼다 (아래 내부 형식은 preview_server 의 가짜 값도 씀).

---- 내부 형식 ----
caps (예전 가정 형식, 지금은 내부용):
  {"version":1, "device":"SM-S921N", "created_ms":...,
   "cameras": {"2": {"active_array":[4000,3000], "fps_ranges":[[15,30],[30,30],...],
                     "yuv":[{"w":640,"h":480,"min_frame_ns":16666666}, ...]},
               "5": {...}},
   "encoder": {"name":"c2.qti.avc.encoder", "hw":true, "max_instances":16,
               "bitrate_range":[64000, 100000000],
               "sizes":{"640x480":{"fps_max":240.0}, "1440x1080":{"fps_max":120.0}, ...}}}
     ("sizes" 는 4:3 후보마다 getSupportedFrameRatesFor(w,h).getUpper(); 지원 안 하면 키를 빼거나 fps_max 0)
perf.json (앱 files/perf.json, 스트리밍·녹화 중 약 1 s 마다 덮어씀; 또는 같은 dict 를 scan3d state.phone.perf 로):
  {"t_ms":..., "w":640, "h":480, "fps_req":15,
   "cam_fps":[14.9,14.9],              # 렌즈별 실제 카메라 fps (최근 1 s)
   "jpeg_ms_avg":[6.1,5.8], "jpeg_ms_max":[12.0,11.0],   # 렌즈별 compressToJpeg 시간 (최근 1 s)
   "jpeg_bytes_avg":[52000,50000],     # 렌즈별 JPEG 크기 평균
   "frames":[n0,n1], "jpeg_dropped":[d0,d1], "cam_dropped":[c0,c1], "enc_dropped":[e0,e1],   # 누적 카운터
   "temp_c":38.5}                      # 폰 온도 (없으면 PC 의 dumpsys 값 사용)

---- 실시간 영상 형식 (stream_fmt) ----
기본 = "h264" (사용자 결정: 실시간도 H.264, I+P 만·B-frame 0·키프레임 1 s·실시간 우선). "jpeg"·"nv21" 은 고급 설정.
  h264 : 폰 하드웨어 인코더가 압축 → 폰 CPU 는 YUV 를 인코더 입력 버퍼로 복사하는 시간만 (Surface 입력이면 거의 0).
         인코더 수 = 렌즈 2 (실시간). 녹화 + 실시간(both) 이면 녹화 2 + 실시간 2 = 4, 앱이 한 인코더 출력을 둘에 같이 쓰면 2.
         USB = 비트레이트 합 (+ 여유), PC = H.264 디코딩 + 정류.
  jpeg : 폰 CPU 가 compressToJpeg (예전 기본). PC = JPEG 풀기 + 정류.
  nv21 : 압축 없음 (1.5 B/px). USB 가 금방 모자람.
비트레이트 모드 (앱 담당이 정함, 여기서는 USB 예산만): 같은 평균 비트레이트에서 화질이 일정한 쪽은 VBR —
  MediaCodec BITRATE_MODE_VBR 는 장면이 복잡할 때 비트를 더 쓰고 CBR 은 비트레이트를 고정해 화질이 흔들린다
  (developer.android.com/reference/android/media/MediaCodecInfo.EncoderCapabilities#BITRATE_MODE_VBR). 그래서 USB 예산은
  VBR 순간 피크·1 s 마다 오는 키프레임을 H264_USB_HEADROOM (25 %) 여유로 본다. 추정.

---- 가정한 필드 이름 (앱·수신 담당과 통합 때 맞춤) ----
caps.json:  pairs[].live_encoder {fps: name|null}   실시간 H.264 인코더 (없으면 record_encoder 를 같은 검사로 씀)
            pairs[].live_kbps_default {fps: kbps}   실시간 기본 비트레이트 (렌즈 하나)
            live_h264 {input:"surface"|"buffer", shared_with_record:bool, profile, b_frames, i_interval_s,
                       bitrate_mode:"vbr"|"cbr", low_latency:bool}
metrics.json: stream_fmt "h264", live_kbps (목표, 렌즈 하나),
            lenses{id:{h264_fps, h264_frames, h264_dropped, h264_kbps (최근 실제), h264_enc_ms_avg, h264_enc_ms_max
                       (입력 → 출력 버퍼 지연), copy_ms_avg (기존: 카메라 YUV 복사)}}
PC 수신 (phone_rx.stats()): decode_ms (H.264 한 장 풀기 평균, 렌즈 하나, 정류 제외), decode_ms_max
"""
import json
import math
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RUNS = HERE / "runs"
COEFF_FILE = RUNS / "load_coeffs.json"
CAPS_CACHE = RUNS / "phone_caps.json"
CAMS_S24 = ROOT / "phone_stereo" / "cameras_s24.json"
CALIB = ROOT / "calib_phone_pair.json"
APP_ID = "com.camera.dualstream"
APP_EXT_DIR = f"/sdcard/Android/data/{APP_ID}/files"      # 앱 1.3-caps: caps.json·metrics.json (run-as 없이 cat)

DEFAULT_SIZE = (1440, 1080)                     # 2026-10-16 사용자 결정 (2+5 · 1440x1080, 3렌즈 비교 뒤)
DEFAULT_FPS = 15
FPS_CANDIDATES = (10, 15, 20, 24, 30, 60)
MIN_W, MAX_W = 320, 4000
LENS = ("2", "5")
MODES = ("live", "record", "both")
STREAM_FMTS = ("h264", "jpeg", "nv21")
DEFAULT_STREAM_FMT = "h264"
FMT_INFO = dict(h264="H.264 (권장)", jpeg="JPEG", nv21="NV21 (압축 없음)")
H264_USB_HEADROOM = 1.25          # VBR 순간 피크 + 키프레임 (1 s 간격) 여유. 추정
H264_MIN_KBPS = 500.0             # 앱 Caps.autoKbps 와 같은 최소
H264_MAX_KBPS = 100000.0          # 인코더 범위를 모를 때 상한 (넉넉하게, 100 Mbps)


def norm_fmt(f):
    f = str(f or "").lower()
    return f if f in STREAM_FMTS else DEFAULT_STREAM_FMT

# 계수 초기값 — 모두 추정 (근거 주석). 실행 중 EWMA 로 실측에 맞춰진다.
DEFAULT_COEFFS = dict(
    # 폰 YuvImage.compressToJpeg (NV21→JPEG, q90). libjpeg-turbo NEON 은 큰 코어에서 100 Mpx/s 넘지만 YuvImage 는
    # 복사·비-turbo 경로라 보수적으로 40 Mpx/s 가정 → 25 ms/Mpx (640x480 ≈ 7.7 ms). 추정.
    jpeg_ms_per_mpx=25.0,
    # q90 JPEG 크기: 실내 장면 약 1.2~1.8 bit/px → 0.18 B/px (640x480 ≈ 55 KB). 추정.
    jpeg_bytes_per_px=0.18,
    # PC FFS: 640x480(0.307 Mpx) 에서 약 50 ms 실측 기록 (iters 4) → 픽셀 수에 비례 가정 163 ms/Mpx.
    ffs_ms_per_mpx=163.0,
    # PC YOLO (yolo11m-seg, 입력을 640 으로 줄임 → 화질과 거의 무관). 약 22 ms (preview 기록값과 같은 크기). 추정.
    yolo_ms=22.0,
    # PC 수신: JPEG 풀기(cv2.imdecode ≈ 150 Mpx/s) + 정류 remap ≈ 10 ms/Mpx (렌즈마다, 프레임마다). 추정, 갱신 안 함.
    pc_rx_ms_per_mpx=10.0,
    # 녹화 H.264 비트레이트: 앱 1.3-caps 기본식 4000 kbps @640x480·15 (= 8000 @640x480·30) 을 픽셀률에 비례 (렌즈 하나).
    # caps.json 의 pairs[].rec_kbps_default 가 있으면 그 값을 쓴다 (SPEC 14.5).
    rec_kbps_ref=8000.0,
    # 실시간 H.264 비트레이트 (렌즈 하나): 사용자 결정 640x480·15 ≈ 4000 kbps → 픽셀률 비례 (녹화 14.5 와 같은 식).
    live_kbps_ref=4000.0,
    # 폰 YUV → 인코더 입력 버퍼 복사 (ByteBuffer 입력일 때; Surface 입력이면 0). metrics copy_ms_avg (YUV→NV21) 와 같은 종류.
    # 1.5 B/px memcpy·plane 재배치, 모바일 큰 코어 수 GB/s → 640x480 1 ms 안팎. 보수적으로 3 ms/Mpx (≈ 0.9 ms). 추정.
    phone_copy_ms_per_mpx=3.0,
    # PC H.264 풀기 (FFmpeg/libavcodec 소프트웨어, 스트림 하나 한 스레드, YUV→BGR 변환 포함).
    # libavcodec 은 1080p(2.07 Mpx) 를 한 코어에서 대략 100~300 fps 로 푼다고 흔히 알려짐 → 2~5 ms/Mpx; 보수적으로 5. 추정 (출처 확인 안 함,
    # 실행 중 decode_ms 로 EWMA).
    pc_h264_dec_ms_per_mpx=5.0,
    # PC 정류 remap (cv2.remap bilinear, 렌즈마다) ≈ 3 ms/Mpx. 추정. (JPEG 경로 pc_rx_ms_per_mpx 10 ≈ 풀기 7 + remap 3)
    pc_remap_ms_per_mpx=3.0,
)
COEFF_INFO = dict(
    jpeg_ms_per_mpx="폰 JPEG 압축 시간 (ms / 백만 화소)",
    jpeg_bytes_per_px="JPEG 크기 (바이트 / 화소, 품질 90)",
    ffs_ms_per_mpx="PC 깊이(FFS) 시간 (ms / 백만 화소)",
    yolo_ms="PC 물체 찾기(YOLO) 시간 (ms)",
    pc_rx_ms_per_mpx="PC 영상 풀기·정류 (ms / 백만 화소)",
    rec_kbps_ref="녹화 비트레이트 기준 (kbps, 640x480·30 fps·렌즈 하나)",
    live_kbps_ref="실시간 H.264 비트레이트 기준 (kbps, 640x480·15 fps·렌즈 하나)",
    phone_copy_ms_per_mpx="폰 영상 복사 시간 (ms / 백만 화소, H.264 버퍼 입력)",
    pc_h264_dec_ms_per_mpx="PC H.264 풀기 (ms / 백만 화소)",
    pc_remap_ms_per_mpx="PC 정류 (ms / 백만 화소)",
)
EWMA_KEYS = ("jpeg_ms_per_mpx", "jpeg_bytes_per_px", "ffs_ms_per_mpx", "yolo_ms",
             "phone_copy_ms_per_mpx", "pc_h264_dec_ms_per_mpx")
ALPHA = 0.2
OBS_CLIP = (1.0 / 3.0, 3.0)       # 실측 하나를 지금 계수의 이 배수 안으로 자름 (REVIEW6 R2)
OBS_FIRST_MAX_A = 0.5             # 첫 실측도 절반만 섞음 (REVIEW6 R2)
# 앱 metrics.json state 가 이것으로 시작하면 카메라가 안 도는 것 → 그 지표는 안 씀 (REVIEW6 R4)
STOPPED_STATES = ("ERROR", "CAMERA_ERROR", "CONFIGURE_", "REQUEST_FAILED", "STOPPED", "DISCONNECTED", "Inventory")
# 부하 추정만 넘은 이유 (실제 장치 한계가 아님) — 시작은 경고만 하고 허용 (REVIEW6 R2)
SOFT_PREFIX = ("폰 JPEG 압축", "폰 영상 복사 부하", "USB ", "PC 영상 풀기 부하", "PC 깊이 계산")

LIMITS = dict(
    phone_jpeg_pct=60.0,      # 두 렌즈 JPEG 압축 (또는 H.264 입력 복사) 합 ≤ 한 코어 시간의 60 %
    jpeg_frame_frac=0.8,      # 렌즈 하나의 JPEG 시간 ≤ 프레임 간격의 80 % (넘으면 그 렌즈가 프레임을 버림)
    usb_mbs=8.0,              # USB(adb forward) 실효 8 MB/s
    pc_rx_pct=60.0,           # PC 수신(풀기+정류) ≤ 한 코어의 60 %
    kf_min=3.0,               # 실시간: PC 깊이 키프레임 ≥ 3 /s
    enc_px_rate=3840 * 2160 * 60,   # (caps 없을 때) 하드웨어 H.264 총 픽셀률 4K60 가정, 두 렌즈 합. 추정
    enc_max_px=4096 * 2304,         # (caps 없을 때) H.264 level 5.2 최대 프레임
    enc_lat_frac=1.0,               # 실시간 H.264: 인코더 지연(입력→출력) 실측 ≤ 프레임 간격 (넘으면 쌓여서 늦어짐)
)


def mpx(w, h):
    return w * h / 1e6


def is_43(w, h):
    return w * 3 == h * 4


def parse_size(s):
    if isinstance(s, (list, tuple)) and len(s) == 2:
        return int(s[0]), int(s[1])
    w, _, h = str(s or "").lower().partition("x")
    return int(w), int(h)


def size_str(w, h):
    return f"{w}x{h}"


def _read_json(p):
    try:
        return json.loads(Path(p).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def calib_size():
    j = _read_json(CALIB) or {}
    s = j.get("image_size")
    try:
        return int(s[0]), int(s[1])
    except (TypeError, IndexError, ValueError):
        return DEFAULT_SIZE


def _ranges(v):
    if isinstance(v, str):
        try:
            v = json.loads(v)
        except ValueError:
            return []
    out = []
    for r in v or []:
        try:
            out.append((float(r[0]), float(r[1])))
        except (TypeError, IndexError, ValueError):
            pass
    return out


# ---------------- 능력표 ----------------
def caps_from_cameras_json(j):
    """phone_stereo/cameras_s24.json (논리 카메라 목록, physical[] 에 yuv_sizes·fps_ranges) → caps 형식 (min_frame 없음)."""
    cams = {}
    for lg in j if isinstance(j, list) else []:
        for ph in lg.get("physical") or []:
            pid = str(ph.get("id"))
            if pid in LENS and pid not in cams:
                yuv = []
                for s in ph.get("yuv_sizes") or []:
                    try:
                        w, h = parse_size(s)
                        yuv.append(dict(w=w, h=h))
                    except ValueError:
                        pass
                cams[pid] = dict(yuv=yuv, fps_ranges=_ranges(ph.get("fps_ranges")), active_array=None)
    return dict(version=0, cameras=cams, encoder=None) if cams else None


def default_caps():
    yuv = [dict(w=w, h=h) for w, h in ((320, 240), (640, 480), (960, 720), (1440, 1080), (1920, 1440))]
    rng = [(15, 15), (15, 30), (30, 30), (20, 20), (24, 24), (60, 60)]
    return dict(version=0, cameras={p: dict(yuv=list(yuv), fps_ranges=rng) for p in LENS}, encoder=None)


def normalize_caps(j):
    """앱 caps.json (format scan3d-phone-caps/1, SPEC 14.2) → 내부 형식. 이미 내부 형식이면 그대로.
    내부에 더하는 키: lens_ids, fixed_fps (앱이 받는 고정 AE fps), pairs {"WxH": pair}, full_area {"WxH": bool}."""
    if not isinstance(j, dict):
        return None
    if j.get("cameras") and not j.get("lenses"):
        return j
    lenses = j.get("lenses") or {}
    if not lenses:
        return None
    ids = [str(x) for x in (j.get("pair_lenses") or list(lenses.keys()))]
    lg = j.get("logical") or {}
    cams, full = {}, {}
    for pid in ids:
        L = lenses.get(pid) or {}
        yuv = []
        for y in L.get("yuv") or []:
            try:
                w, h = int(y["w"]), int(y["h"])
            except (KeyError, TypeError, ValueError):
                continue
            mf = y.get("min_frame_ns")
            yuv.append(dict(w=w, h=h, min_frame_ns=mf if isinstance(mf, (int, float)) and mf > 0 else None))
            k = size_str(w, h)
            full[k] = full.get(k, True) and bool(y.get("full_active_array", True))
        cams[pid] = dict(yuv=yuv, fps_ranges=_ranges(lg.get("ae_fps_ranges")), active_array=L.get("active_array"))
    pairs = {}
    for p in j.get("pairs") or []:
        try:
            pairs[size_str(int(p["w"]), int(p["h"]))] = p
        except (KeyError, TypeError, ValueError):
            pass
    fixed = [int(f) for f in lg.get("fixed_fps") or [] if isinstance(f, (int, float))]
    dev = j.get("device") or {}
    return dict(version=1, format=j.get("format"), device=dev.get("model") if isinstance(dev, dict) else dev,
                app_version=j.get("app_version"), written=j.get("written_wall"), lens_ids=ids, cameras=cams,
                fixed_fps=fixed or None, pairs=pairs, full_area=full, encoder=None,
                session_support=j.get("session_support"),
                avc_encoders=[e for e in j.get("avc_encoders") or [] if isinstance(e, dict)],
                live_h264=j.get("live_h264") if isinstance(j.get("live_h264"), dict) else None)


def _lens_ids(caps):
    return tuple(caps.get("lens_ids") or LENS)


def _pair(caps, w, h):
    return (caps.get("pairs") or {}).get(size_str(w, h))


def _read_phone_json(link, name, timeout=8):
    """앱 파일 하나: 외부 폴더 (adb shell cat, run-as 필요 없음) → 안 되면 run-as files/. 반환 dict 또는 None."""
    for args in (("cat", f"{APP_EXT_DIR}/{name}"), ("run-as", APP_ID, "cat", f"files/{name}")):
        try:
            rc, out = link.shell(*args, timeout=timeout)
        except Exception:
            continue
        s = (out or "").strip()
        if rc == 0 and s.startswith("{"):
            try:
                return json.loads(s)
            except ValueError:
                continue
    return None


def _caps_sig(j):
    """능력표가 바뀌었는지 볼 값: 앱 버전 코드·이름, 쓴 시각 (REVIEW6 R8)."""
    if not isinstance(j, dict):
        return None
    return (j.get("app_version_code"), j.get("app_version"), j.get("written_wall_ms") or j.get("written_wall"))


def load_caps(link=None, refresh=False, log=None):
    """(caps, 출처). 폰 caps.json (adb) → runs/phone_caps.json 캐시 → cameras_s24.json → 기본.
    refresh=True 면 폰 파일을 다시 읽고, 앱 버전·쓴 시각이 캐시와 다를 때만 캐시를 바꾸고 로그를 남긴다."""
    if link is not None and getattr(link, "available", False) and (refresh or not CAPS_CACHE.is_file()):
        try:
            j = _read_phone_json(link, "caps.json")
            if j and normalize_caps(j):
                old = _read_json(CAPS_CACHE)
                if _caps_sig(old) != _caps_sig(j) or not normalize_caps(old):
                    j["_read_t"] = time.time()
                    RUNS.mkdir(parents=True, exist_ok=True)
                    CAPS_CACHE.write_text(json.dumps(j, ensure_ascii=False, indent=1), encoding="utf-8")
                    if log:
                        log(f"[load] 폰 능력표 caps.json 읽음 (앱 {j.get('app_version', '?')})")
            elif log and refresh:
                log("[load] 폰에 caps.json 이 아직 없음 (앱 1.3-caps 이상을 한 번 켜야 생김)")
        except Exception as e:
            if log:
                log(f"[load] caps.json 읽기 실패: {e!r}")
    j = normalize_caps(_read_json(CAPS_CACHE))
    if j and j.get("cameras"):
        return j, "폰 (caps.json)"
    j = _read_json(CAMS_S24)
    c = caps_from_cameras_json(j) if j else None
    if c:
        return c, "cameras_s24.json (최대 fps 추정)"
    return default_caps(), "기본 후보 (추정)"


def session_check(caps, w, h, fps):
    """폰 HAL 의 두 렌즈 동시 세션 답 (caps session_support, API 35 isSessionConfigurationSupported; REVIEW6 R3).
    반환 이유 문자열 또는 None. 답이 없거나 null 이면 None (막지 않음)."""
    ss = caps.get("session_support")
    if not isinstance(ss, dict) or not ss.get("available"):
        return None
    for q in ss.get("results") or []:
        try:
            if int(q.get("w")) != w or int(q.get("h")) != h:
                continue
        except (TypeError, ValueError):
            continue
        if q.get("supported") is False:
            return "폰이 두 렌즈를 이 크기로 동시에 못 연다고 답함 (폰 능력표)"
        mf = q.get("max_fps_checked")
        if q.get("supported_at_max_fps") is False and isinstance(mf, (int, float)) and int(fps) >= int(mf):
            return f"폰이 두 렌즈 {int(mf)} fps 동시 세션을 못 연다고 답함 (폰 능력표)"
        return None
    return None


def cam_max_fps(caps, w, h):
    """두 렌즈 모두 그 크기를 내는가 + 그때 최대 fps (min_frame_ns 기준, 없으면 AE 범위·추정). (ok, fps_max, 추정?)"""
    fmax, guess = math.inf, False
    pr = _pair(caps, w, h)
    if caps.get("pairs") and pr is None:
        return False, 0.0, False                       # 앱이 두 렌즈 공통 크기로 내지 않음
    if pr is not None and isinstance(pr.get("max_fps"), (int, float)) and pr["max_fps"] > 0:
        return True, float(pr["max_fps"]), False
    for p in _lens_ids(caps):
        c = (caps.get("cameras") or {}).get(p)
        if not c:
            return False, 0.0, True
        hit = next((y for y in c.get("yuv") or [] if int(y.get("w", 0)) == w and int(y.get("h", 0)) == h), None)
        if hit is None:
            return False, 0.0, False
        mf = hit.get("min_frame_ns")
        if mf and mf > 0:
            fmax = min(fmax, 1e9 / float(mf))
        else:
            # min_frame 을 모르면 두 물리 카메라 동시 스트림은 30 fps 까지로 본다 (AE 범위에 60 이 있어도 보수적으로). 추정
            guess = True
            hi = max([r[1] for r in _ranges(c.get("fps_ranges"))] or [30.0])
            fmax = min(fmax, hi, 30.0)
    return True, fmax, guess


def fps_ok_ae(caps, fps, w=None, h=None):
    """앱 1.3-caps 는 고정 AE 범위 [fps, fps] 가 없으면 ERROR 로 거부한다 (SPEC 14.4; S24 는 15·20·24·30·60, 10 없음).
    caps.json 의 pairs[].camera_fps (앱이 같은 검사로 만든 목록) > logical.fixed_fps > fps_ranges 의 lo==hi."""
    pr = _pair(caps, w, h) if w else None
    if pr is not None and isinstance(pr.get("camera_fps"), list) and int(fps) in {int(x) for x in pr["camera_fps"]}:
        return True
    if pr is not None and isinstance(pr.get("camera_fps"), list) and int(fps) in FPS_CANDIDATES:
        return False
    if caps.get("fixed_fps"):
        return int(fps) in caps["fixed_fps"]
    for p in _lens_ids(caps):
        c = (caps.get("cameras") or {}).get(p) or {}
        rs = _ranges(c.get("fps_ranges"))
        if rs and not any(lo == hi == fps for lo, hi in rs):
            return False
    return True


ENC_LABEL = dict(record="녹화 인코더", live="실시간 영상 인코더", both="영상 인코더 (녹화+실시간)")


def enc_need(caps, mode, stream_fmt, n_rec=2):
    """동시에 쓰는 H.264 인코더 수와 종류. (n, kind 'record'|'live'|'both'|None, 공유?)
    실시간 H.264 = 렌즈마다 1 (2개). 녹화 = 렌즈 수 (보통 2개, '3개 모두 녹화 (비교용)' 이면 n_rec=3).
    both + h264 = 2 + 녹화 수, 앱이 한 인코더 출력을 녹화·실시간에 같이 쓰면 녹화 수
    (caps live_h264.shared_with_record — 가정한 필드)."""
    live_h = mode in ("live", "both") and norm_fmt(stream_fmt) == "h264"
    rec = mode in ("record", "both")
    n_rec = max(2, int(n_rec or 2))
    shared = bool((caps.get("live_h264") or {}).get("shared_with_record"))
    if live_h and rec:
        return (n_rec if shared else 2 + n_rec), "both", shared
    if live_h:
        return 2, "live", False
    if rec:
        return n_rec, "record", False
    return 0, None, False


def _avc_info(caps, name):
    for e in caps.get("avc_encoders") or []:
        if e.get("name") == name:
            return e
    return None


def _first_hw_encoder(caps):
    encs = caps.get("avc_encoders") or []
    return next((e for e in encs if e.get("hardware")), encs[0] if encs else None)


def enc_check(caps, w, h, fps, n=2, kind="record"):
    """(ok, 이유 또는 None, 추정?) H.264 인코더 n 개를 동시에 (렌즈 2 × 녹화/실시간). 하드웨어 인코더 우선 (앱 Caps 와 같은 순서)."""
    label = ENC_LABEL.get(kind, "영상 인코더")
    pr = _pair(caps, w, h) or {}
    k = str(int(fps))
    keys = {"record": ("record_encoder",), "live": ("live_encoder", "record_encoder"),
            "both": ("record_encoder", "live_encoder")}.get(kind, ("record_encoder",))
    names, known = [], False
    for key in keys:
        d = pr.get(key)
        if isinstance(d, dict) and k in d:
            known = True
            if not d[k]:
                what = "녹화" if key == "record_encoder" else "실시간 영상"
                return False, f"{what} 인코더가 이 크기·fps 를 지원하지 않음 (폰 능력표)", False
            names.append(d[k])
            if kind == "live":
                break                                  # live_encoder 가 있으면 record_encoder 는 안 봄
    if known:
        # 앱 검사 (areSizeAndRateSupported ∧ YUV420Flexible ∧ max_instances ≥ 렌즈 수) 를 통과한 인코더.
        # n 이 그보다 크면 (both 4개) max_instances 를 다시 본다.
        for nm in set(names):
            e = _avc_info(caps, nm) or {}
            mi = e.get("max_instances")
            if isinstance(mi, int) and 0 < mi < n:
                return False, f"{label}를 {n}개 동시에 못 씀 (폰 최대 {mi}개)", False
        # 여러 인코더 합산 처리량은 caps 에 없다 → 4K60 급 한도로 같이 본다 (추정)
        if n * w * h * fps > LIMITS["enc_px_rate"]:
            return False, f"{label} 처리량을 넘음 ({n}개 합, 추정)", True
        return True, None, False
    enc = caps.get("encoder") or None
    if enc and enc.get("sizes") is not None:
        s = (enc.get("sizes") or {}).get(size_str(w, h))
        fm = float((s or {}).get("fps_max") or 0)
        if fm <= 0:
            return False, f"{label}가 이 크기를 지원하지 않음", False
        if fps > fm:
            return False, f"{label} 최대 {fm:.0f} fps", False
        if int(enc.get("max_instances") or 2) < n:
            return False, f"{label}를 {n}개 동시에 못 씀", False
        # 크기별 fps_max 는 인코더 하나 기준 → n 개면 픽셀률 n 배를 4K60 급 한도로 같이 본다
    if w * h > LIMITS["enc_max_px"]:
        return False, f"{label} 최대 크기를 넘음 (추정)", True
    if n * w * h * fps > LIMITS["enc_px_rate"]:
        return False, f"{label} 처리량을 넘음 ({n}개 합, 추정)", True
    return True, None, not (enc and enc.get("sizes") is not None)


def _clip_kbps(caps, k):
    br = (caps.get("encoder") or {}).get("bitrate_range") or (_first_hw_encoder(caps) or {}).get("bitrate_range")
    hi = H264_MAX_KBPS
    if br:
        try:
            k = max(k, br[0] / 1000.0)
            hi = br[1] / 1000.0
        except (TypeError, IndexError):
            pass
    return min(k, hi)


def live_kbps(coeffs, caps, w, h, fps):
    """실시간 H.264 비트레이트 (렌즈 하나, kbps): 640x480·15 ≈ 4000 기준 픽셀률 비례, 최소 500, 인코더 범위 (없으면 100 Mbps) 로 자름.
    caps pairs[].live_kbps_default (가정한 필드) 가 있으면 그것."""
    d = ((_pair(caps, w, h) or {}).get("live_kbps_default") or {}).get(str(int(fps)))
    if isinstance(d, (int, float)) and d > 0:
        return int(d)
    k = coeffs["live_kbps_ref"] * (w * h * fps) / (640 * 480 * 15)
    return round(_clip_kbps(caps, max(H264_MIN_KBPS, k)))


def rec_kbps(coeffs, caps, w, h, fps):
    d = ((_pair(caps, w, h) or {}).get("rec_kbps_default") or {}).get(str(int(fps)))
    if isinstance(d, (int, float)) and d > 0:
        return int(d)                                  # 앱이 인코더 범위로 자른 기본값 (SPEC 14.5)
    k = coeffs["rec_kbps_ref"] * (w * h * fps) / (640 * 480 * 30)
    k = max(500.0, k)                                  # 앱 Caps.autoKbps 와 같은 최소
    br = (caps.get("encoder") or {}).get("bitrate_range")
    if br:
        try:
            k = min(max(k, br[0] / 1000.0), br[1] / 1000.0)
        except (TypeError, IndexError):
            pass
    return round(k)


# ---------------- 모델 ----------------
class LoadModel:
    def __init__(self, log=None):
        self.log = log or (lambda m: None)
        self.lock = threading.RLock()
        self.coeffs = dict(DEFAULT_COEFFS)
        self.n_obs = {k: 0 for k in EWMA_KEYS}
        self.updated = None
        self._dirty_t = 0.0
        self._load_coeffs()
        self.caps, self.caps_src = load_caps(None)
        self.calib = calib_size()
        self.ffs_scale_fixed = None                    # scan3d --scale (None = 자동, 크기마다)
        self.ffs_max_px = 640 * 480                    # scan3d --ffs-max-px
        self.vio_fixed_size = None                     # scan3d 고정 VIO 설정의 크기 (있으면 다른 크기 실시간 거절, REVIEW6 R1)

    # ---- 계수 저장 ----
    def _load_coeffs(self):
        j = _read_json(COEFF_FILE) or {}
        for k, v in (j.get("coeffs") or {}).items():
            if k in self.coeffs and isinstance(v, (int, float)) and v > 0:
                self.coeffs[k] = float(v)
        self.n_obs.update({k: int(v) for k, v in (j.get("n_obs") or {}).items() if k in self.n_obs})
        self.updated = j.get("updated")

    def save(self, force=False):
        with self.lock:
            if not force and not self._dirty_t:
                return
            try:
                RUNS.mkdir(parents=True, exist_ok=True)
                COEFF_FILE.write_text(json.dumps(dict(coeffs=self.coeffs, n_obs=self.n_obs, updated=self.updated,
                                                      note="scan3d/load_model.py EWMA 계수 (지우면 초기 추정값으로)"),
                                                 ensure_ascii=False, indent=1), encoding="utf-8")
                self._dirty_t = 0.0
            except OSError as e:
                self.log(f"[load] 계수 저장 실패: {e!r}")

    def reset(self):
        with self.lock:
            self.coeffs = dict(DEFAULT_COEFFS)
            self.n_obs = {k: 0 for k in EWMA_KEYS}
            self.updated = None
            try:
                COEFF_FILE.unlink()
            except OSError:
                pass

    def refresh_caps(self, link):
        self.caps, self.caps_src = load_caps(link, refresh=True, log=self.log)
        return self.caps_src

    def observe(self, key, value):
        """실측 하나를 EWMA 로. 처음 몇 개는 빨리 따라가게 (1/n 과 ALPHA 중 큰 값, 단 0.5 이하).
        튀는 값 하나가 계수를 망치지 않게 실측을 지금 계수의 1/3 ~ 3 배로 자른다 (REVIEW6 R2)."""
        if value is None or not (value > 0) or key not in EWMA_KEYS:
            return
        with self.lock:
            n = self.n_obs[key] + 1
            a = min(OBS_FIRST_MAX_A, max(ALPHA, 1.0 / n)) if n <= 5 else ALPHA
            cur = self.coeffs[key]
            v = min(max(float(value), cur * OBS_CLIP[0]), cur * OBS_CLIP[1])
            self.coeffs[key] = (1 - a) * cur + a * v
            self.n_obs[key] = n
            self.updated = time.strftime("%Y-%m-%d %H:%M:%S")
            if not self._dirty_t:
                self._dirty_t = time.monotonic()
        if time.monotonic() - self._dirty_t > 30.0:
            self.save()

    # ---- 후보 ----
    def sizes(self):
        """두 렌즈 공통 YUV 크기 중 4:3, MIN_W..MAX_W, 작은 것부터."""
        common = None
        for p in _lens_ids(self.caps):
            c = (self.caps.get("cameras") or {}).get(p) or {}
            s = {(int(y.get("w", 0)), int(y.get("h", 0))) for y in c.get("yuv") or []}
            common = s if common is None else common & s
        out = sorted(((w, h) for w, h in common or () if is_43(w, h) and MIN_W <= w <= MAX_W), key=lambda x: x[0] * x[1])
        if DEFAULT_SIZE not in out:
            out = sorted(out + [DEFAULT_SIZE], key=lambda x: x[0] * x[1])
        return out

    def ffs_scale(self, w, h, scale=None):
        """FFS 입력 축소 비율. scale 을 주면 그것, 아니면 scan3d 와 같은 자동 규칙 (calib_scale.auto_ffs_scale:
        화소 수가 ffs_max_px 를 넘으면 sqrt(max/화소), 0.25 아래로 안 줄임) — 크기마다 다르다."""
        if scale is None:
            scale = self.ffs_scale_fixed
        if scale is not None:
            return float(scale)
        s = min(1.0, math.sqrt(self.ffs_max_px / float(w * h)))
        return round(max(0.25, s), 3)

    def h264_surface(self):
        """앱이 카메라 → 인코더를 Surface 로 바로 잇는가 (caps live_h264.input, 가정한 필드). 모르면 False (복사 비용을 넣음)."""
        return str((self.caps.get("live_h264") or {}).get("input", "")).lower() == "surface"

    def predict(self, w, h, fps, stream_fmt=DEFAULT_STREAM_FMT, scale=None, mode="live", n_rec=2):
        fmt = norm_fmt(stream_fmt)
        scale = self.ffs_scale(w, h, scale)
        c = self.coeffs
        m = mpx(w, h)
        jpeg_ms = c["jpeg_ms_per_mpx"] * m if fmt == "jpeg" else 0.0
        # 폰 CPU: jpeg = 압축, h264 = 인코더 입력 복사 (Surface 직결이면 0), nv21 = 보낼 버퍼 복사
        copy_ms = 0.0 if fmt == "jpeg" or (fmt == "h264" and self.h264_surface()) else c["phone_copy_ms_per_mpx"] * m
        lk = live_kbps(c, self.caps, w, h, fps) if fmt == "h264" else None
        if fmt == "h264":
            usb = 2 * lk * H264_USB_HEADROOM / 8000.0                        # kbps → MB/s (두 렌즈, 여유 포함)
            pc_rx_ms = (c["pc_h264_dec_ms_per_mpx"] + c["pc_remap_ms_per_mpx"]) * m
            pc_dec_ms = c["pc_h264_dec_ms_per_mpx"] * m
        elif fmt == "jpeg":
            usb = 2 * fps * w * h * c["jpeg_bytes_per_px"] / 1e6
            pc_rx_ms = c["pc_rx_ms_per_mpx"] * m
            pc_dec_ms = max(0.0, c["pc_rx_ms_per_mpx"] - c["pc_remap_ms_per_mpx"]) * m
        else:                                                                # nv21 = 1.5 B/px
            usb = 2 * fps * w * h * 1.5 / 1e6
            pc_rx_ms = c["pc_remap_ms_per_mpx"] * m
            pc_dec_ms = 0.0
        ffs_ms = c["ffs_ms_per_mpx"] * m * scale * scale
        kf_ms = ffs_ms + c["yolo_ms"]
        n_enc, enc_kind, enc_shared = enc_need(self.caps, mode, fmt, n_rec)
        return dict(
            stream_fmt=fmt,
            phone_jpeg_ms=round(jpeg_ms, 1),
            phone_jpeg_pct=round(2 * fps * jpeg_ms / 10.0, 1),                 # 한 코어 시간 대비 %
            jpeg_frame_frac=round(jpeg_ms * fps / 1000.0, 2),                  # 렌즈 하나: 프레임 간격 대비
            phone_copy_ms=round(copy_ms, 2),
            phone_cpu_pct=round(2 * fps * (jpeg_ms + copy_ms) / 10.0, 1),      # 형식과 무관한 폰 부하 (막대에 씀)
            live_kbps=lk,                                                      # 실시간 H.264 비트레이트 (렌즈 하나)
            usb_mbs=round(usb, 2),
            pc_dec_ms=round(pc_dec_ms, 1),                                     # PC 한 장 풀기 (렌즈 하나)
            pc_rx_pct=round(2 * fps * pc_rx_ms / 10.0, 1),
            pc_ffs_ms=round(ffs_ms, 1), pc_kf_ms=round(kf_ms, 1),
            pc_ffs_fps=round(1000.0 / max(1.0, kf_ms), 1),
            rec_kbps=rec_kbps(c, self.caps, w, h, fps), ffs_scale=scale,
            enc_n=n_enc, enc_kind=enc_kind, enc_shared=enc_shared, h264_surface=self.h264_surface(),
            n_rec=max(2, int(n_rec or 2)),                                     # 녹화 렌즈 수 (3 = 비교용 3개 모두)
        )

    def combo(self, mode, w, h, fps, kf_min=None, stream_fmt=DEFAULT_STREAM_FMT, scale=None, n_rec=2):
        """n_rec = 녹화 렌즈 수 (3 = '3개 모두 녹화 (비교용)': 인코더 3개만 더 본다 — 실측 1440x1080 15 fps 3렌즈 녹화 됨)."""
        kf_min = LIMITS["kf_min"] if kf_min is None else kf_min
        live = mode in ("live", "both")
        stream_fmt = norm_fmt(stream_fmt)
        p = self.predict(w, h, fps, stream_fmt, scale, mode, n_rec)
        reasons, notes = [], []
        ok_cam, fmax, guess = cam_max_fps(self.caps, w, h)
        if not ok_cam:
            reasons.append("카메라가 이 크기를 두 렌즈 모두 내지 못함")
        elif fps > fmax + 0.01:
            reasons.append(f"카메라 최대 {fmax:.0f} fps (이 크기)" + (" — 추정" if guess else ""))
        if ok_cam and fps <= fmax + 0.01 and not fps_ok_ae(self.caps, fps, w, h):
            reasons.append(f"카메라에 고정 {fps} fps 설정이 없음 (앱이 거부)")
        if ok_cam and (self.caps.get("full_area") or {}).get(size_str(w, h)) is False:
            reasons.append("이 크기는 센서 일부만 써서 보정 비례 변환이 맞지 않음")
        if ok_cam:
            why = session_check(self.caps, w, h, fps)
            if why:
                reasons.append(why)
        vf = self.vio_fixed_size
        if live and vf and tuple(vf) != (w, h):
            reasons.append(f"위치 추적 설정이 {vf[0]}x{vf[1]} 로 고정됨 (--ov-config 또는 외부 브리지)")
        if not is_43(w, h):
            reasons.append("4:3 크기만 지원")
        if live:
            if stream_fmt == "jpeg" and p["phone_jpeg_pct"] > LIMITS["phone_jpeg_pct"]:
                reasons.append(f"폰 JPEG 압축 부하 {p['phone_jpeg_pct']:.0f}% > {LIMITS['phone_jpeg_pct']:.0f}%")
            elif stream_fmt == "jpeg" and p["jpeg_frame_frac"] > LIMITS["jpeg_frame_frac"]:
                reasons.append("폰 JPEG 압축이 프레임 간격보다 오래 걸림")
            elif stream_fmt != "jpeg" and p["phone_cpu_pct"] > LIMITS["phone_jpeg_pct"]:
                reasons.append(f"폰 영상 복사 부하 {p['phone_cpu_pct']:.0f}% > {LIMITS['phone_jpeg_pct']:.0f}%")
            if p["usb_mbs"] > LIMITS["usb_mbs"]:
                reasons.append(f"USB {p['usb_mbs']:.1f} MB/s > {LIMITS['usb_mbs']:.0f} MB/s")
            if p["pc_rx_pct"] > LIMITS["pc_rx_pct"]:
                reasons.append(f"PC 영상 풀기 부하 {p['pc_rx_pct']:.0f}% > {LIMITS['pc_rx_pct']:.0f}%")
            if p["pc_ffs_fps"] < kf_min:
                reasons.append(f"PC 깊이 계산 {p['pc_ffs_fps']:.1f} /s < {kf_min:.0f} /s")
        enc_guess = False
        if p["enc_n"]:
            eok, why, enc_guess = enc_check(self.caps, w, h, fps, p["enc_n"], p["enc_kind"])
            if not eok:
                reasons.append(why)
        if (w, h) != tuple(self.calib):
            notes.append(f"보정은 {self.calib[0]}x{self.calib[1]} 값을 비례 변환")
        if guess or (p["enc_n"] and enc_guess):
            notes.append("폰 능력표 없음 — 일부 추정")
        load_only = bool(reasons) and all(r.startswith(SOFT_PREFIX) for r in reasons)
        if load_only:
            notes.append("예상 부하만 넘음 — 시작은 할 수 있음 (느리거나 끊길 수 있음)")
        return dict(ok=not reasons, load_only=load_only, reasons=reasons, notes=notes,
                    cam_max_fps=None if not ok_cam or fmax == math.inf else round(fmax, 1), **p)

    def table(self, mode="live", kf_min=None, stream_fmt=DEFAULT_STREAM_FMT, scale=None, current=None, n_rec=2):
        mode = mode if mode in MODES else "live"
        stream_fmt = norm_fmt(stream_fmt)
        sizes = self.sizes()
        fpss = list(FPS_CANDIDATES)
        combos = {}
        for w, h in sizes:
            for fp in fpss:
                combos[f"{w}x{h}@{fp}"] = self.combo(mode, w, h, fp, kf_min, stream_fmt, scale, n_rec)
        return dict(mode=mode, stream_fmt=stream_fmt, default_fmt=DEFAULT_STREAM_FMT, n_rec=max(2, int(n_rec or 2)),
                    fmts=[dict(id=k, label=FMT_INFO[k]) for k in STREAM_FMTS],
                    h264=dict(surface=self.h264_surface(), headroom=H264_USB_HEADROOM,
                              settings=self.caps.get("live_h264")),
                    sizes=[dict(size=size_str(w, h), w=w, h=h, mpx=round(mpx(w, h), 2),
                                           calib=(w, h) == tuple(self.calib)) for w, h in sizes],
                    fps=fpss, combos=combos, default=dict(size=size_str(*DEFAULT_SIZE), fps=DEFAULT_FPS),
                    current=current, calib=size_str(*self.calib), caps_src=self.caps_src,
                    limits=dict(LIMITS, kf_min=LIMITS["kf_min"] if kf_min is None else kf_min),
                    coeffs=self.coeffs_json())

    def coeffs_json(self):
        with self.lock:
            return dict(values={k: round(v, 4) for k, v in self.coeffs.items()},
                        defaults=dict(DEFAULT_COEFFS), n_obs=dict(self.n_obs), updated=self.updated,
                        info=COEFF_INFO, ewma=list(EWMA_KEYS), file=str(COEFF_FILE))

    def step_down(self, mode, w, h, fps, kf_min=None, stream_fmt=DEFAULT_STREAM_FMT):
        """한 단계 낮추기: fps 를 먼저 (15 초과면), 아니면 크기를 한 칸. 가능한 조합 중 첫째. 없으면 None."""
        sizes = self.sizes()
        lower_f = [f for f in FPS_CANDIDATES if f < fps][::-1]
        lower_s = [s for s in sizes if s[0] * s[1] < w * h][::-1]
        cands = []
        if fps > DEFAULT_FPS:
            cands += [(w, h, f) for f in lower_f]
        cands += [(sw, sh, fps) for sw, sh in lower_s]
        cands += [(w, h, f) for f in lower_f]
        cands += [(sw, sh, f) for sw, sh in lower_s for f in lower_f]
        for cw, ch, cf in cands:
            if self.combo(mode, cw, ch, cf, kf_min, stream_fmt)["ok"]:
                return dict(size=size_str(cw, ch), fps=cf)
        return None


# ---------------- 실행 중 지표 ----------------
def perf_from_metrics(j):
    """앱 metrics.json (format scan3d-phone-metrics/1, SPEC 14.3) → 내부 perf dict. 이미 내부 형식이면 그대로.
    누적 카운터: frames = jpeg_frames (스트림 없으면 cam_frames), jpeg_dropped, cam_dropped = cam_gap_drops + cam_buffer_lost,
    enc_dropped = rec_dropped_queue + rec_encoder_busy. 더하는 키: core_pct (jpeg_core_pct_total), usb_mbs, thermal_status,
    proc_cpu_pct, t_wall_ms, stream_fmt."""
    if not isinstance(j, dict) or not j:
        return None
    if not str(j.get("format", "")).startswith("scan3d-phone-metrics"):
        return j
    ids = [str(x) for x in (j.get("physical") or list((j.get("lenses") or {}).keys()))]
    L = [(j.get("lenses") or {}).get(i) or {} for i in ids]
    stream = j.get("stream_fmt") in STREAM_FMTS
    jpeg = j.get("stream_fmt") == "jpeg"
    h264 = j.get("stream_fmt") == "h264"

    def col(k, f=float):
        return [f(x.get(k) or 0) for x in L]

    usb = j.get("usb") or {}
    ph = j.get("phone") or {}
    out = dict(t_wall_ms=j.get("t_wall_ms"), w=j.get("width"), h=j.get("height"), fps_req=j.get("fps"),
               state=j.get("state"), stream_fmt=j.get("stream_fmt"),
               cam_fps=col("cam_fps"),
               frames=[int(x.get("h264_frames" if h264 else "jpeg_frames" if stream else "cam_frames") or 0) for x in L],
               jpeg_dropped=col("h264_dropped" if h264 else "jpeg_dropped", int),   # 실시간 형식에서 버린 수
               copy_ms_avg=col("copy_ms_avg"), live_kbps_target=j.get("live_kbps"),
               cam_dropped=[int(x.get("cam_gap_drops") or 0) + int(x.get("cam_buffer_lost") or 0) for x in L],
               enc_dropped=[int(x.get("rec_dropped_queue") or 0) + int(x.get("rec_encoder_busy") or 0) for x in L],
               temp_c=ph.get("battery_temp_c"), thermal_status=ph.get("thermal_status"),
               proc_cpu_pct=ph.get("proc_cpu_pct_one_core"), usb_mbs=usb.get("MBps") if stream else None)
    if jpeg and any(x.get("jpeg_fps") for x in L):
        out["jpeg_ms_avg"] = col("jpeg_ms_avg")
        out["jpeg_ms_max"] = col("jpeg_ms_max")
        out["jpeg_bytes_avg"] = [float(x.get("jpeg_kb_avg") or 0) * 1024.0 for x in L]
        out["core_pct"] = j.get("jpeg_core_pct_total")
    if h264:                                           # 가정한 필드 (위 docstring) — 통합 때 앱과 맞춤
        out["h264_kbps"] = col("h264_kbps")
        out["h264_enc_ms_avg"] = col("h264_enc_ms_avg")
        out["h264_enc_ms_max"] = col("h264_enc_ms_max")
        out["h264_fps"] = col("h264_fps")
    return out


class LoadMonitor:
    """실행 중 지표를 모아 계수를 갱신하고, 예상 대비 실측·과부하 신호·한 단계 낮추기 제안을 만든다.
    web_server.Helper 가 2 s 마다 update(...) 를 부른다."""

    def __init__(self, model):
        self.m = model
        self.prev = None           # (t, perf 누적 카운터)
        self.last_kf = None
        self.over_n = 0
        self.snapshot = dict(active=False)

    @staticmethod
    def _pair(v):
        if isinstance(v, (list, tuple)):
            return [float(x) for x in v if isinstance(x, (int, float))]
        if isinstance(v, (int, float)):
            return [float(v)]
        return []

    def update(self, mode, w, h, fps, perf=None, depth=None, phone=None, temp_level=None, jpeg_sizes=None,
               scale=None, kf_min=None, stream_fmt=DEFAULT_STREAM_FMT):
        active = mode in MODES
        if not active:
            self.prev, self.over_n = None, 0
            self.snapshot = dict(active=False)
            return self.snapshot
        m = mpx(w, h)
        stream_fmt = norm_fmt(stream_fmt)
        scale = self.m.ffs_scale(w, h, scale)
        pred = self.m.combo(mode, w, h, fps, kf_min, stream_fmt, scale)
        meas, flags = {}, []
        live = mode in ("live", "both")
        perf = perf_from_metrics(perf) if isinstance(perf, dict) and perf else None
        # perf 의 크기·fps 가 지금과 다르거나 앱이 스트리밍·녹화 상태가 아니면 (옛 세션 파일, ERROR 등) 쓰지 않는다 (REVIEW6 R4)
        same = perf is not None and int(perf.get("w") or w) == w and int(perf.get("h") or h) == h
        if same and perf.get("fps_req") is not None:
            try:
                same = int(perf["fps_req"]) == int(fps)
            except (TypeError, ValueError):
                pass
        if same and str(perf.get("state") or "").startswith(STOPPED_STATES):
            same = False
        if perf is not None and same:
            ja = self._pair(perf.get("jpeg_ms_avg"))
            if ja and sum(ja) > 0:
                meas["phone_jpeg_ms"] = round(sum(ja) / len(ja), 1)
                cp = perf.get("core_pct")
                meas["phone_jpeg_pct"] = round(float(cp), 1) if isinstance(cp, (int, float)) else round(sum(ja) * fps / 10.0, 1)
                self.m.observe("jpeg_ms_per_mpx", meas["phone_jpeg_ms"] / m)
            jm = self._pair(perf.get("jpeg_ms_max"))
            if jm:
                meas["phone_jpeg_ms_max"] = round(max(jm), 1)
            if ja and sum(ja) > 0:
                meas["phone_cpu_pct"] = meas["phone_jpeg_pct"]
            if stream_fmt != "jpeg":
                cpy = self._pair(perf.get("copy_ms_avg"))
                if cpy and sum(cpy) > 0:
                    meas["phone_copy_ms"] = round(sum(cpy) / len(cpy), 2)
                    meas["phone_cpu_pct"] = round(sum(cpy) * fps / 10.0, 1)
                    if not (stream_fmt == "h264" and self.m.h264_surface()):
                        self.m.observe("phone_copy_ms_per_mpx", meas["phone_copy_ms"] / m)
            if stream_fmt == "h264":
                hk = [x for x in self._pair(perf.get("h264_kbps")) if x > 0]
                if hk:
                    meas["live_kbps"] = round(sum(hk) / len(hk))                 # 렌즈 하나 평균 (실제)
                he = [x for x in self._pair(perf.get("h264_enc_ms_avg")) if x > 0]
                if he:
                    meas["enc_ms"] = round(sum(he) / len(he), 1)
                    if meas["enc_ms"] > LIMITS["enc_lat_frac"] * 1000.0 / max(1, fps):
                        flags.append(f"폰 영상 압축이 늦음 ({meas['enc_ms']:.0f} ms)")
                hm = self._pair(perf.get("h264_enc_ms_max"))
                if hm:
                    meas["enc_ms_max"] = round(max(hm), 1)
            jb = self._pair(perf.get("jpeg_bytes_avg"))
            if jb:
                self.m.observe("jpeg_bytes_per_px", (sum(jb) / len(jb)) / (w * h))
            cf = self._pair(perf.get("cam_fps"))
            if cf:
                meas["cam_fps"] = round(min(cf), 1)
                if min(cf) < 0.85 * fps:
                    flags.append(f"카메라가 {min(cf):.0f} fps 밖에 못 냄 (목표 {fps})")
            # 누적 카운터 → 최근 구간 드롭 비율
            now = time.monotonic()
            cnt = {k: sum(self._pair(perf.get(k))) for k in ("frames", "jpeg_dropped", "cam_dropped", "enc_dropped")}
            if self.prev is not None:
                dt = now - self.prev[0]
                d = {k: cnt[k] - self.prev[1][k] for k in cnt}
                fr = max(1.0, d["frames"])
                if dt > 0 and all(v >= 0 for v in d.values()):
                    meas["drop_pct"] = round(100.0 * (d["jpeg_dropped"] + d["cam_dropped"]) / (fr + d["jpeg_dropped"]), 1)
                    meas["enc_dropped"] = int(d["enc_dropped"])
                    if meas["drop_pct"] > 5.0:
                        flags.append(f"폰이 프레임을 버림 ({meas['drop_pct']:.0f}%)")
                    if d["enc_dropped"] > 0:
                        flags.append(f"녹화 인코더가 프레임을 버림 ({int(d['enc_dropped'])}장)")
            self.prev = (now, cnt)
            if perf.get("temp_c") is not None:
                meas["temp_c"] = perf.get("temp_c")
            if isinstance(perf.get("usb_mbs"), (int, float)) and perf["usb_mbs"] > 0:
                meas["usb_mbs"] = round(float(perf["usb_mbs"]), 2)
            ts = perf.get("thermal_status")              # PowerManager: 2 보통 3 심함 4 위험 (스로틀)
            if isinstance(ts, int) and ts >= 3:
                temp_level = "hot"
            elif isinstance(ts, int) and ts == 2 and temp_level != "hot":
                temp_level = "warm"
        if jpeg_sizes:
            js = [float(x) for x in jpeg_sizes if x]
            if js:
                bpp = sum(js) / len(js) / (w * h)
                meas.setdefault("usb_mbs", round(2 * fps * w * h * bpp / 1e6, 2))
                if not (perf and self._pair(perf.get("jpeg_bytes_avg"))):
                    self.m.observe("jpeg_bytes_per_px", bpp)
        if live and isinstance(depth, dict):
            kf = depth.get("keyframes")
            # scan3d dstats["size"] = 그 키프레임 크기 (REVIEW6 R5: 예전엔 없는 키 "_size" 를 봐서 늘 통과)
            dsz = depth.get("size")
            same_d = not dsz or (w, h) == tuple(int(x) for x in dsz)
            if depth.get("ffs_ms") and kf != self.last_kf and same_d:
                self.m.observe("ffs_ms_per_mpx", float(depth["ffs_ms"]) / max(1e-6, m * scale * scale))
                if depth.get("yolo_ms"):
                    self.m.observe("yolo_ms", float(depth["yolo_ms"]))
            self.last_kf = kf
            if depth.get("ffs_ms") and same_d:
                meas["pc_kf_ms"] = round(float(depth["ffs_ms"]) + float(depth.get("yolo_ms") or 0), 1)
                meas["pc_ffs_fps"] = round(1000.0 / max(1.0, meas["pc_kf_ms"]), 1)
        if live and isinstance(phone, dict):
            dm = phone.get("decode_ms")                  # PC 수신의 한 장 풀기 (가정한 이름, 렌즈 하나·정류 제외)
            if isinstance(dm, (int, float)) and dm > 0:
                meas["pc_dec_ms"] = round(float(dm), 1)
                if isinstance(phone.get("decode_ms_max"), (int, float)):
                    meas["pc_dec_ms_max"] = round(float(phone["decode_ms_max"]), 1)
                if stream_fmt == "h264":
                    self.m.observe("pc_h264_dec_ms_per_mpx", float(dm) / m)
            pf = [v for v in (phone.get("fps") or []) if v]
            if pf and "cam_fps" not in meas:
                meas["cam_fps"] = round(min(pf), 1)
                if phone.get("connected") and min(pf) < 0.8 * fps:
                    flags.append(f"PC 가 받는 영상이 {min(pf):.0f} fps (목표 {fps})")
        if temp_level == "hot":
            flags.append("폰이 뜨거움")
        elif temp_level == "warm":
            flags.append("폰이 따뜻함")
        if meas.get("phone_jpeg_pct", 0) > LIMITS["phone_jpeg_pct"] * 1.15:
            flags.append(f"폰 JPEG 부하 {meas['phone_jpeg_pct']:.0f}%")
        elif stream_fmt != "jpeg" and meas.get("phone_cpu_pct", 0) > LIMITS["phone_jpeg_pct"] * 1.15:
            flags.append(f"폰 영상 복사 부하 {meas['phone_cpu_pct']:.0f}%")
        serious = [f for f in flags if f != "폰이 따뜻함"]
        self.over_n = self.over_n + 1 if serious else 0
        overload = self.over_n >= 3                      # 2 s × 3 = 6 s 이상 계속될 때만
        suggest = self.m.step_down(mode, w, h, fps, kf_min, stream_fmt) if (overload or temp_level == "hot") else None
        self.snapshot = dict(active=True, mode=mode, stream_fmt=stream_fmt, size=size_str(w, h), fps=fps,
                             predicted=pred, measured=meas,
                             flags=flags, overload=overload or temp_level == "hot", suggest=suggest,
                             coeff_updated=self.m.updated)
        return self.snapshot
