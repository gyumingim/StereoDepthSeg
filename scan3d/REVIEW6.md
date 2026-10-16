# REVIEW6 — 화질(4:3 크기)·fps 선택과 폰 부하 계산 (독립 검토)

검토한 날: 2026-10-16. 코드만 읽었습니다. 실행·측정·빌드는 하지 않았고 고친 파일도 없습니다(이 파일만 새로 씀).
읽은 파일: `scan3d/load_model.py`, `web_server.py`, `scan3d.py`, `calib_scale.py`, `make_ov_config.py`, `depth_seg.py`, `phone_link.py`,
`win_relay.py`, `recording.py`, `web/app.js`, `web/index.html`, `README.md`, 앱 `Caps.java`·`Metrics.java`·`MainActivity.java`, `lib_ffs.py`,
`lib_phone_depth.py`, `lib_phone_stereo.py`, `calib_phone_pair.json`, `phone_stereo/cameras_s24.json`.

> **주의 (검토하는 동안 바뀐 파일):** 12:00 무렵 다른 작업이 `load_model.py`(실시간 H.264, `live_h264`, `STREAM_FMTS`)와
> 앱 `H264.java`·`LiveEncoder.java`를 고치고 있었습니다. 이 검토는 통합 보고에 나온 판(부하 계수 jpeg/nv21만 있던 판)을 기준으로 했습니다.
> 통합 보고의 "빌드·문법 통과"는 지금 파일에는 해당하지 않으므로, 그 작업이 끝나면 `py_compile`과 APK 빌드를 다시 돌려야 합니다.

## 문제없다고 본 것
- **보정 비례 변환 수학:** `scale_calib`는 fx·fy·skew에 s를 곱하고 `cx' = s(cx+0.5)-0.5`(화소 중심 규약)로 바꿉니다. 왜곡 계수·R·T·R_sensor_to_left는 그대로 둡니다. 맞습니다.
  정류맵은 바뀐 크기로 다시 만듭니다(`rectify_maps`, alpha 0). stereoRectify의 R1·R2는 R·T로만 정해지므로 `frames.T_world_rect0`도 크기가 바뀌어도 맞습니다.
- **OpenVINS 설정과 K가 같은지:** `make_ov_config.write_configs`도 똑같이 `scale_calib`와 `rectify_maps`를 써서 정류 P1의 f·cx·cy, `resolution [W,H]`를 적습니다.
  설정은 `config/<W>x<H>_<fps>/`에 크기·fps마다 따로 만듭니다. `_vio_key`에 (크기, fps, 모드)가 들어 있어서 하나라도 바뀌면 `_ensure_live_vio`가 브리지를 다시 띄웁니다.
- **깊이·복셀:** `_process_keyframe`은 그 키프레임의 `g["rp"]`와 `ffs_scale`을 `ds(...)`에 직접 넘기고, 크기가 다른 프레임은 버립니다.
  FFS 축소 크기는 32 배수로 채우고, 결과 시차는 실제 비율(W/W0)로 되돌립니다.
- **웹 → 앱 전달:** `{cmd:"mode", size, fps}` → `Helper._apply_stream`(조합 검사, `rec_kbps`) → `set_stream` → `start_session` → `apply_geometry`(정류맵, `link.width/height/fps`) → `am start --ei width/height/fps/rec_kbps`.
  수신기는 `StreamReceiver(..., geom["size"])`이고 win_relay는 바이트만 넘깁니다. 끊긴 곳은 없습니다.
- **지원 안 되는 조합 막기 (폰 능력표가 있을 때):** 크기는 `pairs`에 있어야 하고, fps는 `camera_fps`(앱 `checkCamera`와 같은 검사)에 있어야 합니다.
  녹화는 `record_encoder[fps]`, 센서 일부만 쓰는 크기는 `full_area=false`로 막습니다. 앱이 쓰는 기준과 같습니다.
- **폰 능력표가 없을 때:** `cameras_s24.json`의 물리 렌즈 fps 범위는 논리 카메라와 같아서 10 fps가 빠지는 결과도 같습니다. 최대 fps는 30으로 보수적으로 봅니다.
- **단위:** `jpeg_kb_avg`는 앱에서 1024로 나누고 PC에서 1024를 곱합니다. `jpeg_core_pct_total`(2초 창의 압축 시간 합 ÷ 한 코어)과 예상식 `2·fps·ms/10`은 같은 정의입니다.
- **녹화 복원:** `meta.json`의 width·height·fps로 `apply_geometry`를 하고 `self.want`는 건드리지 않습니다. 다음 실시간 세션은 다시 want로 시작합니다.

## 실제 버그·위험 (심각도 순)

### R1. [중] `--ov-config`(또는 `OV_CONFIG` 환경변수)나 외부 브리지를 쓰면 크기를 바꿔도 640x480 VIO 설정을 그대로 씀
`_vio_config()`는 `a.ov_config`가 있으면 크기와 상관없이 그 파일을 돌려줍니다. 브리지는 다시 띄우지만 설정은 같은 파일입니다.
`run_scan.sh:64`와 `serve_idle.sh:23`은 `OV_CONFIG`가 있으면 이 인자를 붙입니다. `--ov <주소>`(외부 브리지)일 때는 `_vio_key`가 None이어서 아예 확인하지 않습니다.
그런데 부하 모델과 `_apply_stream`은 이런 경우를 막지 않습니다. 그래서 960x720을 고르면 OpenVINS가 640 intrinsics·resolution 설정으로 960 영상을 받습니다.
오류 없이 위치 추적만 틀어집니다.
→ 고정 설정이면 그 설정의 `resolution`을 읽어 다른 크기를 거절하거나, combos에 "고정 VIO 설정이 WxH용"이라는 이유를 붙여야 합니다.

### R2. [중] 부하 계수 EWMA에 튀는 값 막이가 없음 → 첫 실측 하나로 모든 실시간 조합이 회색이 될 수 있음
`observe()`는 처음 실측(n=1)에서 a=1이라서 추정값을 그 한 값으로 바로 바꿉니다. n≤5 동안은 1/n로 단순 평균하고, 결과는 30초 뒤 `runs/load_coeffs.json`에 저장됩니다.
FFS 계수 하나를 "픽셀 수에 비례"한다고 보고 모든 크기가 같이 씁니다. 그런데 작은 크기는 GPU 고정 비용 때문에 비례하지 않습니다(320x240은 축소하지 않아 입력 모양도 다름).
또 깊이 스레드는 `start_session`이 `apply_geometry`를 하기 **전에** `self.rp`를 잡아 그 크기로 warmup합니다.
그래서 첫 세션을 다른 크기로 시작하면 첫 키프레임이 처음 보는 입력 모양으로 돌아 느릴 수 있습니다(cuDNN·triton 준비 시간).
폰이 뜨거워 느려질 때의 JPEG 시간도 그대로 계수에 들어갑니다.
이런 값 하나로 `pc_ffs_fps < 3` 또는 `phone_jpeg_pct > 60`이 되면 640x480@15까지 회색이 됩니다. `_apply_stream`이 시작을 거절하는데 웹은 force를 보내지 않습니다.
사용자는 고급 → [계수 초기화]를 누르기 전까지 실시간 스캔을 시작할 수 없습니다.
→ 실측 하나를 지금 계수의 0.33~3배로 자르고, 첫 실측도 a≤0.5로 섞어야 합니다. FFS는 크기별(또는 FFS 입력 모양별)로 계수를 따로 두는 편이 좋습니다.
또 시작 검사에서 "추정 부하만 넘은 경우"는 경고만 하고 시작은 허용하는 것을 권합니다.

### R3. [하-중] 폰이 알려 준 `session_support`(API 35 `isSessionConfigurationSupported`)를 모아 두기만 하고 쓰지 않음
`normalize_caps`는 이 값을 저장하지만 `combo()`는 보지 않습니다.
HAL이 "렌즈 2·5 YUV 두 개를 이 크기로 동시에 열 수 없음"(`supported=false`, `supported_at_max_fps=false`)이라고 답한 크기도 부하만 맞으면 고를 수 있습니다.
그러면 앱은 `checkCamera`를 통과합니다(물리 렌즈마다 따로 보기 때문). 그 뒤 세션 구성에서 실패해 `ERROR UNSUPPORTED`가 아닌 카메라 오류로 끝납니다.
→ `results[]`에서 `supported is False`인 크기는 막고, `supported_at_max_fps is False`이면 `max_fps_checked` fps는 막아야 합니다.

### R4. [하] 이전 세션의 `metrics.json`을 새 세션 첫 값으로 받아들임
대기 상태에서는 `perf_wall`을 None으로 되돌립니다. 그래서 새 세션의 첫 읽기는 앱이 아직 새 파일을 쓰기 전이라도(외부 폴더에 남은 옛 파일) 받아들입니다.
크기만 비교하고(`same`) fps·state·기록 시각은 비교하지 않습니다.
같은 크기에서 30→15 fps로 바꾸면 옛 30 fps의 압축 %·카메라 fps가 "실제"로 한 번 보이고, 계수에도 한 번 들어갑니다.
앱이 `ERROR UNSUPPORTED`로 열리지 않았을 때도 옛 값이 한 번 보입니다.
→ `fps_req == fps`, `state`가 OPEN 또는 STREAMING 계열인지, `t_wall_ms` ≥ 세션 시작 시각인지도 확인해야 합니다.

### R5. [하] 깊이 실측의 크기 확인이 늘 통과함 (`_size` 키가 없음)
`LoadMonitor.update`는 `depth.get("_size")`로 크기를 비교하는데, scan3d는 `dstats["size"]`를 씁니다. 그래서 이 확인은 늘 통과합니다.
계수 갱신은 키프레임 번호가 바뀌어야만 일어나서 계수가 틀어지지는 않습니다. 하지만 크기를 바꾼 직후 첫 키프레임 전까지는
막대의 "실제 PC 깊이" 값이 이전 크기 값으로 보입니다. → `"size"`로 비교하고, 다르면 그 실측은 보여 주지 않아야 합니다.

### R6. [하] 웹의 화질 목록에서 "안 됨" 이유를 늘 10 fps 기준으로 보여 줌
`renderQSel`은 `c0 = comboOf(size, t.fps[0])`, 곧 10 fps 조합의 첫 이유를 씁니다. S24에는 고정 10 fps가 없어서, 회색 크기는 모두 "안 됨: 카메라가 못 냄"으로 나옵니다.
예를 들어 1440x1080의 진짜 이유는 폰 압축 117 %인데 그렇게 나오지 않습니다. 툴팁도 10 fps 이유를 보여 줍니다.
→ 카메라가 받는 fps 중 가장 낮은 조합의 이유를 쓰거나, 모든 fps에 공통인 이유를 써야 합니다.

### R7. [하] 실행 중 예상·한 단계 낮추기가 스트림 형식을 무시함
`LoadMonitor.update`와 `step_down`은 `combo()`를 기본값 `stream_fmt="jpeg"`로 부릅니다. 목록(`Helper.combos`)은 `a.app_stream`을 씁니다.
그래서 nv21(그리고 이제 들어오는 h264)일 때는 실행 중 "예상"과 낮추기 제안이 목록 판정과 다릅니다.
nv21이면 640x480@15도 USB 13.8 MB/s > 8로 모든 실시간 조합이 거절됩니다(옛 측정상 의도일 수 있지만 안내가 없음).
→ `update`와 `step_down`에도 fmt·scale을 넘겨야 합니다.

### R8. [하] 폰 능력표를 한 번 받아 두면 자동으로 다시 읽지 않음
`runs/phone_caps.json`이 있으면 `caps_src`가 "폰"이 되어 대기 중이나 세션 시작 뒤에도 다시 읽지 않습니다. 앱을 새로 설치해서 caps 형식이나 값이 바뀌어도 옛 값을 씁니다.
H.264 작업으로 `live_h264`가 생기는 지금 바로 해당됩니다. [능력표 다시 읽기]를 눌러야만 바뀝니다.
→ 캐시의 `app_version_code`·`written_wall_ms`를 폰 파일과 비교해서 바뀌었으면 다시 읽어야 합니다.

### R9. [하·문서] README의 "정확도는 640x480(원래 보정 크기)이 가장 좋다"는 사실과 다름
`calib_phone_pair.json`에는 `"scaled_from": [960, 720]`가 들어 있습니다. 보드 촬영과 보정을 960x720에서 하고 640x480으로 줄인 값입니다.
그러니 원래 보정 크기는 960x720이고, 640x480도 이미 비례 변환한 값입니다. 960x720으로 쓸 때는 비례 변환을 두 번(960→640→960) 거칩니다.
그때 예전 축소에 반 화소 규약을 쓰지 않았다면 0.25 px 정도 어긋날 수 있습니다(작음).
이 사실은 960x720과 640x480이 같은 화각이라는 간접 근거도 됩니다. → README와 화면 문구를 고쳐야 합니다.

### R10. [하·기존] 앱을 다른 곳에서 다른 크기로 켰을 때 원인이 화면에 안 나옴
`start_app`이 꺼져 있거나 adb가 없어서 앱을 손으로(또는 `win_relay --app-mode`로) 켜면 크기가 다를 수 있습니다.
그러면 `read_frame`이 `Invalid JPEG packet: ... WxH`로 끊기고 다시 접속하기를 반복합니다. 화면에는 "접속 대기"만 보입니다.
크기를 고를 수 있게 되면서 이런 일이 생기기 쉬워졌습니다. → 이 오류 문구를 알아보고 "앱 크기 ≠ 고른 크기"라고 알려 주면 됩니다.

## 10줄 요약
1. 보정 비례 변환 수학, 정류맵, OpenVINS 설정 K·해상도, 브리지 재시작, 깊이·복셀 K, 녹화 복원 크기, 웹→앱 전달은 코드상 맞습니다.
2. R1 [중]: `--ov-config`/`OV_CONFIG`나 외부 브리지를 쓰면 크기를 바꿔도 640x480 VIO 설정이 그대로 쓰입니다. 조합 검사도 이 경우를 막지 않습니다.
3. R2 [중]: 부하 계수는 첫 실측 하나(a=1)로 바뀌고 튀는 값을 막지 않으며 파일에 저장됩니다. 그래서 모든 실시간 조합이 회색이 되어 시작이 막힐 수 있습니다.
4. R3 [하-중]: 폰이 알려 준 두 렌즈 동시 세션 지원 여부(`session_support`)를 쓰지 않습니다.
5. R4 [하]: 새 세션 첫 읽기에서 이전 세션의 `metrics.json`을 받아들입니다(fps·state·시각을 비교하지 않음).
6. R5 [하]: 깊이 실측 크기 확인이 없는 키 `_size`를 봐서 늘 통과합니다. 크기를 바꾼 직후 옛 실측이 보입니다.
7. R6 [하]: 회색 크기의 이유가 늘 10 fps 기준이라 S24에서는 모두 "카메라가 못 냄"으로 보입니다.
8. R7·R8 [하]: 실행 중 예상·낮추기 제안이 스트림 형식(nv21/h264)을 무시하고, 폰 능력표 캐시는 앱이 바뀌어도 자동으로 다시 읽지 않습니다.
9. R9 [문서]: 보정은 원래 960x720(`scaled_from`)입니다. "640x480이 가장 정확"은 틀렸습니다.
10. 검토하는 동안 `load_model.py`·앱 H.264 파일을 다른 작업이 고치고 있었으므로, 그 작업이 끝나면 빌드·문법 검사를 다시 해야 합니다. 실행·측정은 하지 않았습니다.

## 고친 기록 (2026-10-16, 검토 본문은 그대로)
- 고침: R1 — scan3d.vio_fixed_size()(--ov-config 설정 폴더 yaml 의 resolution, 외부 브리지면 시작 크기)를 load_model 이 받아 다른 크기의 실시간 조합을 "위치 추적 설정이 WxH 로 고정됨" 으로 막고, start_session(live/both)·start_replay 도 ValueError 로 거절.
- 고침: R2 — observe() 가 실측을 지금 계수의 1/3~3 배로 자르고 첫 실측도 a ≤ 0.5 로 섞음. combo 에 load_only(예상 부하만 넘음) 추가: _apply_stream 은 경고만 하고 시작, 웹은 "부하 높음" 으로 고를 수 있게 함. FFS 크기별 계수는 안 나눔.
- 고침: R3 — session_check(): caps session_support 에서 supported=false 인 크기, supported_at_max_fps=false 면 max_fps_checked 이상 fps 를 막음.
- 고침: R4 — metrics.json 은 fps_req 가 지금 fps 와 같고, state 가 ERROR·STOPPED 등이 아니고, t_wall_ms 가 세션 시작 − 5 s 이후일 때만 씀.
- 고침: R5 — LoadMonitor 가 dstats["size"] 로 비교하고, 크기가 다르면 실측 깊이 값을 보여 주지도 계수에 넣지도 않음.
- 고침: R6 — 회색 크기의 이유를 카메라가 받는 fps 중 가장 낮은 조합에서 가져옴(없으면 첫 fps).
- 고침: R7 — 이미 고쳐져 있었음(update·step_down 이 stream_fmt 를 넘김), 바꾼 것 없음.
- 고침: R8 — 대기 중 1분마다 폰 caps.json 을 읽어 app_version_code·app_version·written_wall_ms 가 캐시와 다를 때만 캐시를 바꾸고 로그를 남김.
- 고침: R9 — README·화질 목록 문구: 보정은 원래 960x720(scaled_from), 640x480 은 줄인 값이라 "가장 정확" 이 아님.
- 고침: R10 — 웹이 "Invalid JPEG/H.264 packet: … WxH" 오류를 알아보고 "폰 앱이 다른 화질(WxH)로 보내고 있습니다 — 고른 화질은 …" 을 보여 줌.
