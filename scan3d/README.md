# scan3d — 실시간 3D 복셀 맵 스캐너

> **가장 쉬운 사용법** (웹만 쓴다 — 바탕화면 아이콘 없음)
> 1. 폰을 USB 로 꽂고, 브라우저에서 **http://localhost:8795** 를 연다 (서버는 Windows 로그인 때 저절로 켜져 있다).
> 2. **[▶ 스캔 시작]** 을 누르고, 화면 위쪽 큰 글씨 안내대로 한다. 오른쪽 아래 작은 창 두 개가 폰 카메라 화면(초광각 / 광각).
> 3. 끝나면 **[■ 정지]**.
>
> 화면만 미리 보기 (폰·GPU 없이, 가짜 데이터): `python scan3d/preview_server.py` → http://localhost:8796 (위쪽에 "미리보기 — 가짜 데이터" 띠).

폰(Galaxy S24, 렌즈 2 초광각 + 렌즈 5 광각, NV21 640x480 + IMU) → (Windows 릴레이 | 우분투 adb forward) → 리눅스:
OpenVINS(ROS 없음) 스테레오+IMU VIO 로 자세, Fast-FoundationStereo 깊이 + YOLO-seg 분할 → 복셀 맵(물체별 구분) → 웹 뷰어.
폰에 녹화해 두었다가 PC 로 가져와 같은 파이프라인으로 천천히·정확하게 복원할 수도 있다.
인터페이스 명세: `SPEC.md`. 코드 검토: `REVIEW.md`. 녹화 형식: `RECORDING_FORMAT.md` (scan3d-phone-rec/1, 앱 1.1-rec), 읽기: `recording.py`.

## 사용법 (이 PC)

1. 폰을 USB 로 꽂고 폰 화면을 켠다.
2. 브라우저에서 **http://localhost:8795** 를 연다 (즐겨찾기 해 두기).
3. **[▶ 스캔 시작]** 을 누르고, 화면 위쪽 큰 글씨 안내를 따른다 (폰 연결 → 카메라 켜짐 → 2초 가만히 → 천천히 움직이기).
4. 끝나면 **[저장]**, **[■ 정지]**. 위치 추적이 꼬이면 **[↺ 처음부터 다시]**. 녹화본·색·세부 수치는 오른쪽 "고급".

서버는 Windows 로그인 때 저절로 켜진다 (시작프로그램 `scan3d_server.vbs` → WSL `scan3d/serve_idle.sh`, 창 없음).
대기 중에는 웹 서버와 상태 읽기만 하고 **GPU 를 쓰지 않는다**: 깊이 모델(FFS·YOLO)은 첫 [스캔 시작](실시간·녹화+실시간) 또는 복원 때 한 번 불러온다
(화면에 "깊이 모델을 불러오는 중… 30초쯤", 그 뒤로는 서버가 꺼질 때까지 유지).
[스캔 시작] 을 누르면 웹 서버가 Windows 쪽 중계(`win_relay.py`, interop 으로 Windows Python)·폰 앱·OpenVINS 브리지(`--ov auto`, 죽으면 2 초 뒤 자동으로 다시)를 켜고,
[정지] 하면 중계를 끈다. USB 를 다시 꽂으면 중계를 다시 띄우고, 중계 자신도 폰 포트 접속이 실패하면 `adb forward` 를 다시 한다.
중계·브리지가 계속 실패하면 다시 시도 간격을 늘린다 (중계 8→60 s, 브리지 2→30 s). 폰 온도(SKIN)는 10 초마다 읽어 42 °C 주황, 45 °C 빨강.
오른쪽 아래 카메라 창 (`/api/cam.jpg?lens=0|1`): 초광각(렌즈 2) / 광각(렌즈 5) 최근 프레임을 약 3 fps 로 (스캔·복원 중이고 화면이 보일 때만).
누르면 크게, "물체 표시" = 초광각에 최근 YOLO 마스크·상자, "90° 돌리기" = 누운 센서 영상을 세워 보기. 오른쪽 칸의 2D 화면은 깊이만.
로그: `scan3d/runs/server_<날짜>.log`, `scan3d/runs/win_relay.log`.
- 바탕화면 아이콘은 만들지 않는다 (웹만 쓴다). 서버를 손으로 켜고 끄는 스크립트만 남겨 둔다:
  `scan3d\start_scanner.bat` (= `start_scanner.ps1`, 서버가 꺼져 있으면 켜고 브라우저 열기) / `scan3d\stop_scanner.bat` (= `start_scanner.ps1 -Stop`, 녹화 마무리 → 중계·스캐너·앱 끄기).
- 서버가 이미 켜져 있을 때 `run_scan.sh` 를 또 띄우면 포트가 겹치므로 run_scan.sh 가 안내만 하고 끝난다 (먼저 `stop_scanner.bat`).
- adb: `C:\Users\a3162\platform-tools\adb.exe` (WSL `/mnt/c/Users/a3162/platform-tools/adb.exe`) 를 맨 먼저 찾는다 (`phone_link.find_adb`, `win_relay.adb_exe`, `start_scanner.ps1`).
- 자동 시작 끄기: `%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\scan3d_server.vbs` 삭제.
- 아래 명령줄 방법(run_scan.sh + win_relay.py 따로 띄우기)은 개발·문제 해결용.

## 모드 (웹 [스캔 시작] 옆 선택)

| 모드 | 앱 인자 | PC |
|---|---|---|
| 실시간 | `--es mode live` | VIO·깊이·맵 계산. **PC 에 자동 저장 없음** (저장 버튼·`--save-on-exit` 만) |
| 녹화 | `--es mode record` | 계산 없음. `status.txt` 를 1 s 마다 읽어 녹화 시간·프레임 수만 표시 |
| 녹화 + 실시간 | `--es mode both` | 폰은 녹화, PC 는 실시간 계산 (PC 저장 없음) |
| 복원 (재생) | — | 녹화 폴더 → 같은 파이프라인 (모든 프레임 FFS·FFS 반복 수 선택), 결과를 `<녹화>/recon_<시각>/` 에 저장 |

- "시작" = (선택 시) adb 로 앱을 그 모드로 다시 시작 + PC 세션 시작. "정지" = 세션 정지 + 녹화 마무리(STOP 브로드캐스트 → BACK → force-stop).
- "녹화본": "폰 목록 새로고침" → 폰 녹화마다 "가져와서 복원" / "가져오기", PC 에 가져온 녹화마다 "복원". 진행률(가져오기 MB, 재생 위치 / 전체 s) 표시.
- adb 가 없으면 앱 제어 버튼만 꺼지고 나머지는 그대로 (앱은 win_relay 가 시작).
- 모드 인자 흐름: 웹 "시작"(`{cmd:"mode", mode}`) → scan3d `phone_link.start_app(mode)` → `adb shell am start ... --es mode <live|record|both>`
  (WSL 은 interop 으로 Windows adb.exe, 우분투는 리눅스 adb). 명령줄은 `scan3d.py --mode ...`(native 는 앱도 시작) 또는 `win_relay.py --app-mode ...`.
  앱 status.txt 끝에 `mode=<값>` 이 붙는다. win_relay 의 자동 재시작은 status 의 mode 가 자기 `--app-mode` 와 같을 때만 한다
  (웹이 다른 모드로 띄운 녹화를 덮어쓰지 않게).

## 화질(크기)·fps 고르기

웹 [스캔 시작] 옆 **"화질"** 과 **"fps"** 목록에서 고른다. 기본은 **640x480 · 15 fps**.

- **화질은 4:3 크기만** 나온다. 두 렌즈(2·5)가 함께 내는 크기만 (S24: 320x240, 640x480, 960x720, 1440x1080, 1920x1440, 4000x3000).
  800x600·1280x960 은 폰이 안 내서 없다. 16:9 같은 다른 비율은 센서를 잘라 내므로 목록에서 뺀다.
- **fps 는 15·20·24·30·60** (폰이 고정 노출 fps 로 받아 주는 값). S24 에는 10 이 없어서 10 은 회색이다 (앱이 거부함).
- **크기마다 다시 캘리할 필요 없다.** 보정 파일(`calib_phone_pair.json`)의 640x480 값을 크기 비율로 늘리거나 줄여 쓴다
  (초점거리·중심점만 바꾸고 왜곡 계수·두 렌즈 사이 위치는 그대로). 4:3 이고 센서 전체를 쓰는 크기라서 가능하다.
  - 참고: 보드 촬영·보정은 원래 **960x720** 에서 했고(파일의 `scaled_from: [960, 720]`), 640x480 값도 그것을 줄인 것이다.
    그래서 640x480 이 "원래 크기라 가장 정확" 한 것은 아니다. 960x720 은 960→640→960 으로 두 번 비례 변환되지만 차이는 작다(추정 0.25 px 이하).
    다른 크기를 고르면 화면에 "보정은 640x480 값을 비례 변환" 이라고 나오고,
    스캔을 시작할 때마다 정류 검증(세로 어긋남 px)을 그 크기로 다시 해서 보여 준다. 기준은 크기 배율만큼 (1440x1080 이면 2.25 px).
    기준을 넘으면 폰이 그 크기에서 화각을 잘랐을 수 있으니, 그 크기로 따로 캘리하는 것이 좋다.
- **안 되는 조합은 회색** + 이유 (마우스를 올리면 자세히). 판단 기준 (모두 만족해야 함):
  - 폰 카메라가 그 크기·fps 를 냄 (폰 능력표 `caps.json`: 크기별 최대 fps, 고정 fps 목록)
  - 녹화(녹화·녹화+실시간)면 폰 H.264 인코더가 그 크기·fps 를 두 개 동시에 지원
  - 실시간이면: 폰 H.264 인코더 수·처리량이 되는지 (JPEG 이면 압축 합 ≤ 한 코어의 60 %), USB ≤ 8 MB/s, PC 영상 풀기 ≤ 60 %, PC 깊이 계산 ≥ 3 장/초 (고급 설정에서 바꿈)
- **부하 계산은 측정을 따로 하지 않는다.** 처음 값은 추정치(`load_model.py` 주석에 근거)이고, 스캔하는 동안 폰이 1초마다 쓰는
  `metrics.json`(렌즈별 JPEG 압축 시간·크기, 실제 fps, 버린 프레임, USB, 온도)과 PC 깊이 시간으로 조금씩 고쳐 간다 (`runs/load_coeffs.json`).
  고급 설정 "화질·부하 계산" 에서 계수 표를 보고 초기화할 수 있다.
- 스캔 중 과부하가 6초 넘게 이어지거나 폰이 뜨거우면 **"한 단계 낮추기"** 버튼이 뜬다 (fps 먼저, 그다음 크기).
- 큰 크기에서도 PC 깊이 계산(FFS)은 자동으로 640x480 화소 수 정도로 줄여 넣는다 (`--ffs-max-px`, PC 부하는 그대로).
- 폰 능력표: 앱(1.3-caps 이상)이 켜질 때마다 `/sdcard/Android/data/com.camera.dualstream/files/caps.json` 에 쓰고, PC 가 adb 로 읽어
  `runs/phone_caps.json` 에 둔다. 아직 없으면 `phone_stereo/cameras_s24.json` 으로 추정 (최대 fps 30 으로 보수적).
  형식: SPEC.md 14. 명령줄: `scan3d.py --app-size 960x720 --app-fps 15`, `win_relay.py --size 960x720 --fps 15`.

## 깊이 렌즈 쌍: 2+5 (기본) / 2+6 (초광각+망원)

웹 [스캔 시작] 옆 **"깊이 렌즈"** 목록 (다음 [▶ 스캔 시작] 부터). 위치 추적(VIO)은 어느 쪽이든 렌즈 2(초광각) 하나 + IMU 로 똑같다.

| | 2+5 초광각+광각 (기본) | 2+6 초광각+망원 |
|---|---|---|
| 보정 | `calib_phone_pair.json` 체커보드 | `calib_phone_pair_2_6.json` 공장 값 + 장면 쌍으로 다듬음 |
| 두 렌즈 사이 (기선) | 15.5 mm | 32.4 mm |
| 두 렌즈 찍는 시각 차 (실측) | −26 ~ −33 ms (앱 켤 때마다 다름) | **+1 ~ +3 ms (거의 동시)** |
| 정류 초점 (640x480) | f ≈ 463 px | f ≈ 1287 px (망원 화각을 채움) |
| 깊이가 나오는 범위 | 초광각 공통 영역 (수평 약 69°) | **화면 가운데만** (망원 화각 약 28°x21°, 초광각 가운데 약 130x100 화소를 4.9배 확대) |
| 1 m 시차 | 약 7 px | 약 41 px |

- 쓰는 법: "깊이 렌즈" 를 **초광각+망원** 으로 바꾸고 [▶ 스캔 시작] (녹화 + 실시간도 같음). 폰 앱이 `--es physical 2,6` 으로 켜진다.
  녹화를 복원할 때는 녹화 폴더 `meta.json` 의 `physical` 로 쌍을 자동으로 고른다 (2,6 녹화 = 2+6 보정).
- API: `POST /api/cmd {"cmd":"mode","mode":"live","size":"640x480","fps":15,"pair":"2+6"}`, 명령줄 `scan3d.py --pair 2+6`.
- 망원은 초점이 0.5 m 보다 가까우면 흐리다 (최소 초점 2 디옵터, 앱은 1 m 고정). 깊이는 FFS 시차 192 px 로 약 0.22 m 부터.
- 깊이 최대값은 아직 3 m 상한 그대로 (`--depth-range`). 2+6 은 3 m 에서도 시차 14 px 라 더 멀리 쓸 여지가 있다 (다음 단계).
- stereo VIO (`--vio-mode stereo`) 는 2+5 설정만 있어 2+6 이면 시작을 거절한다 (기본 mono 는 상관없음).

**보정 만들기 (`tools/make_factory_pair_calib.py`, 2026-10-16 결과)**

```bash
# 1) 실시간 2+6 세션을 켜고 (폰은 세워 둔 채, 질감 있는 장면을 망원 쪽에) 쌍 저장 → 2) 같은 자리에서 2+5 세션으로 한 번 더
venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --grab phone_stereo/runs/pair26_20261016/p26
venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --grab phone_stereo/runs/pair26_20261016/p25
# 3) 공장 값 → 회전(x·z)·망원 초점 배율 다듬기 → yaw 를 2+5 거리에 맞춤 → calib_phone_pair_2_6.json
venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --refine phone_stereo/runs/pair26_20261016/p26 \
    --yaw-ref phone_stereo/runs/pair26_20261016/p25
venv_ffs/bin/python scan3d/tools/make_factory_pair_calib.py --compare-only    # 2+5 공장 값 vs 체커보드 숫자만
```

- 공장 값을 얼마나 믿나: 2+5 공장 값 vs 체커보드 = 회전 차 0.43° (x −0.31, y −0.26, z −0.14), T 차 0.44 mm (기선 1.5 % 짧음,
  방향 1.4°), 광각 fx 0.991 배. 2+6 은 더 나빠서 공장 회전 그대로면 정류 세로 어긋남 **24~27 px** (f 1248) — 그대로는 못 쓴다.
  장면 쌍 6개로 다듬은 값: 회전 x −1.31°, z −1.31°, 망원 초점 ×1.037 (초점 1 m 의 AF 렌즈), yaw +1.04° (2+5 거리 기준).
  공장 값과의 회전 차 합계 약 2.1°.
- 다듬은 뒤: 정류 세로 어긋남 0.5~1.3 px (기준 1 px), 실시간 첫 쌍 검증 0.80 px · 맞는 비율 0.90.
- yaw 는 세로 어긋남으로 안 보이고 시차에 그대로 들어간다 (0.1° = 2.2 px). 그래서 같은 장면을 2+5 (체커보드 = 미터 맞음) 로도 찍어
  같은 렌즈 2 화소의 거리가 같아지게 맞췄다. 기선 32.4 mm 는 공장 값 그대로 (검증 안 됨) → 실측 거리 한 번으로 확인 권장.
- 폰을 떨어뜨리거나 케이스를 바꾸면 다시 만든다 (위 1~3, 1분).

## 세 모드 사용법

공통: 폰 USB 연결·화면 켬·USB 디버깅 허용. 시작 직후 2 초 정도 폰을 가만히 (OpenVINS 정지 초기화).

**실시간 (live)** — PC 가 바로 계산, 폰에는 저장 안 함.
- 웹: 모드 "실시간" → 시작. 맵이 마음에 들면 "저장" (`scan3d/runs/<시각>/`). 끝나면 "정지".
- 명령줄: WSL `python scan3d\win_relay.py` (Windows) + `bash scan3d/run_scan.sh` (WSL) / 우분투 `bash scan3d/run_scan.sh --native`.

**녹화 (record)** — 폰에만 녹화, PC 계산 없음, 스트림 소켓도 안 열림 (PC 없이도 동작).
- 웹: 모드 "녹화" → 시작 → 촬영 → **"정지"** (STOP 브로드캐스트로 mp4 마무리, 약 3 s). 뷰어는 status.txt 를 1 s 마다 읽어
  `RECORDING rec=rec_…` 와 앱 시작 후 경과 s 를 보여 준다 (앱 status 에 프레임 수 필드는 없음).
- adb 만으로:
  ```bash
  adb shell am start -S -n com.camera.dualstream/.MainActivity --es logical 0 --es physical 2,5 \
    --ei width 640 --ei height 480 --ei fps 30 --ef focus_diopters 1.0 --ez raw_ts false --ei imu_us 2500 --es mode record
  adb shell cat /sdcard/Android/data/com.camera.dualstream/files/status.txt      # RECORDING rec=rec_YYYYMMDD_HHMMSS ...
  adb shell am broadcast -a com.camera.dualstream.STOP -p com.camera.dualstream   # 정지 → RECORD_DONE
  ```
- **`am force-stop` 을 먼저 하면 mp4 가 깨진다** (moov 없음). 꼭 STOP 브로드캐스트 → `RECORD_DONE` 확인 뒤 끈다. 대비로 `--ez rec_raw true` 를 주면 `.h264` 백업도 쓴다.
- 홈 버튼·화면 꺼짐(onStop)도 정상 마무리된다. 다시 앞으로 와도 녹화는 다시 시작하지 않는다 (새 녹화는 웹에서 다시 '시작'; 앱 1.2 이후).
- 녹화 중에 다른 모드를 '시작'하거나 win_relay 가 앱을 다시 띄우면, 먼저 STOP 브로드캐스트로 녹화를 마무리(RECORD_DONE 확인)한 뒤 앱을 다시 띄운다.
  both 모드에서 스트림이 바로 끊기는 일이 반복돼도 녹화 중이면 앱을 재시작하지 않는다 (접속만 다시).

**녹화 + 실시간 (both)** — 폰은 녹화, PC 는 실시간 계산. 웹: 모드 "녹화+실시간" → 시작 → "정지" (녹화 마무리 + 앱 정지).
명령줄로만 할 때는 `win_relay.py --app-mode both` (WSL) 또는 `run_scan.sh --native --mode both` (우분투).
같은 시간 동안 PC 는 실시간 결과, 폰은 나중에 더 정확히 복원할 원본을 갖는다.

녹화 폴더: 폰 `/sdcard/Android/data/com.camera.dualstream/files/rec_YYYYMMDD_HHMMSS/` — `cam2.mp4`, `cam5.mp4`, `frames_cam2.csv`, `frames_cam5.csv`,
`imu.csv`, `cameras.json`, `sensors.json`, `meta.json` (용량 약 2 MB/s). `meta.json` 의 `"complete": false` 면 정상 마무리가 안 된 녹화.

## 설치 (새 우분투 22.04 / WSL Ubuntu-22.04)

1. apt (관리자 한 번):
   ```bash
   sudo apt install -y git build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev curl unzip
   ```
   GPU: NVIDIA 드라이버 (WSL 은 Windows 드라이버만 있으면 됨). RTX 50xx 는 torch cu128 (기본), 다른 GPU 는 `TORCH_INDEX=https://download.pytorch.org/whl/cu124`.
2. 나머지 (sudo 없음, `~/scanner` 에 Ceres·OpenVINS, 저장소에 venv_ffs·FFS):
   ```bash
   cd StereoDepthSeg
   bash scan3d/setup.sh            # check → python(uv, 3.12, venv_ffs, torch) → ffs(클론·가중치·ultralytics/open3d/opencv)
                                   # → ceres(2.2, 사용자 prefix) → openvins(ROS 없음) → bridge(ov_bridge) → config(make_ov_config)
   bash scan3d/setup.sh bridge config   # 단계만 다시
   ```
3. android platform-tools (우분투 직접 모드에만 필요):
   ```bash
   bash scan3d/setup.sh adb        # → tools/platform-tools/adb (dl.google.com platform-tools-latest-linux.zip)
   # 또는 sudo apt install -y android-tools-adb   (오래된 버전이지만 동작)
   ```
   `adb devices` 가 `no permissions` 면 udev 규칙 (Samsung vendor 04e8):
   `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04e8", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/51-android.rules && sudo udevadm control --reload-rules`, 폰 다시 꽂고 USB 디버깅 허용.
4. 앱 APK (최초 1회, 이 PC 는 2026-10-16 설치함): `python3 phone_stereo/build_android.py` (서명 키 `phone_stereo/android/build/debug.keystore` 유지 — 지우면 재설치 때 uninstall 필요) →
   `adb install -r -g phone_stereo/android/build/phone-stereo.apk && adb shell pm enable com.camera.dualstream`.
   서명이 다르면 (INSTALL_FAILED_UPDATE_INCOMPATIBLE) `adb uninstall com.camera.dualstream` 후 설치.

## 실행

### A. 우분투 직접 모드 (중계 없음, Windows 불필요)
```bash
bash scan3d/setup.sh adb                       # 최초 1회: tools/platform-tools/adb (run_scan.sh 가 PATH 에 넣음)
adb devices                                    # 기기가 'device' 로 보여야 함 (no permissions → 아래 udev 규칙)
bash scan3d/run_scan.sh --native               # 브리지 → adb forward tcp:8765 → 127.0.0.1:8765 직접 접속, 앱 시작도 직접 (--es mode live)
bash scan3d/run_scan.sh --native --mode idle   # 웹에서 모드 골라 시작 (실시간 / 녹화 / 녹화+실시간 / 복원)
bash scan3d/run_scan.sh --native --mode both   # 녹화+실시간으로 바로 시작
python3 scan3d/pull_rec.py pull --latest       # 녹화 가져오기 (리눅스 adb)
bash scan3d/run_scan.sh --replay recordings/<이름>   # 복원
```
WSL 이 아니면 `--native` 를 안 줘도 자동 (`/proc/version` 에 microsoft 가 없으면 native). native 는 `--start-app` 이 기본이고,
접속마다 `adb forward` 를 다시 하며, 폰 쪽이 바로 끊기기를 3 번 반복하면 앱을 같은 모드로 다시 시작한다.

### B. WSL + Windows 릴레이 (이 PC)
1. **Windows** (PowerShell, 폰 USB 연결·화면 켜 둠):
   ```powershell
   cd \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg
   $env:SCAN3D_ADB = "<adb.exe 경로>"        # PATH 나 %LOCALAPPDATA%\Android\Sdk 에 adb 가 있으면 생략
   python scan3d\win_relay.py                 # 앱 시작(--es mode live) + adb forward + WSL 로 역방향 중계
   python scan3d\win_relay.py --no-start-app  # 앱 시작·모드는 WSL 웹 뷰어가 맡을 때 (interop 으로 adb.exe 실행)
   ```
   옵션: `--app-mode live|record|both`, `--fps 15`, `--imu-us 0` (FASTEST), `--focus 0` (AF). 폰 쪽이 바로 끊기면(앱 죽음) 앱을 다시 시작한다.
   대안 정방향: `python scan3d\win_relay.py --mode listen` + WSL `PHONE=:18765`.
2. **WSL**:
   ```bash
   cd /home/system/scanner/StereoDepthSeg
   bash scan3d/run_scan.sh                 # ov_bridge --tcp 127.0.0.1:7600 --auto-reset → scan3d.py --link relay --phone listen:8766
   NO_VIO=1 bash scan3d/run_scan.sh        # VIO 없이 고정 포즈로 깊이·분할만
   bash scan3d/run_scan.sh --voxel 0.02 --yolo-rotate --iters 4 --stride 3   # 나머지 인자는 scan3d.py 로
   ```
   WSL 안의 scan3d 는 Windows adb.exe 를 interop 으로 불러 앱 모드 전환·녹화 목록·가져오기를 한다
   (찾는 순서: `--adb`/`SCAN3D_ADB` → `C:\Users\a3162\platform-tools\adb.exe` → PATH 의 adb.exe → `C:\Users\*\platform-tools\adb.exe` → `C:\Users\*\AppData\Local\Android\Sdk\platform-tools\adb.exe` → …, `phone_link.py`).
   브리지 로그: `scan3d/runs/ov_bridge_<시각>.log`. 1 과 2 는 어느 쪽을 먼저 띄워도 된다.

### 브라우저: http://localhost:8795
- 시작 직후 폰을 2초 정도 가만히 두면 OpenVINS 가 정지 초기화된다. "추적 중" 이 뜬 뒤 천천히 움직인다. 녹화도 처음 2초는 가만히.
- 색: 실제 색 / 클래스 색 / 물체별. 물체 목록을 누르면 그 물체로 시점 이동.
- 버튼: 일시정지, 맵 초기화, VIO 재시작, 저장 (`scan3d/runs/<시각>/` 에 voxels.ply, voxels.json, voxels.npz, objects.json, trajectory.json, viewer.html, summary.json).

### 복원 (녹화 → 3D 맵)
웹: "녹화본" → "폰 목록 새로고침" → 녹화 옆 **"가져와서 복원"** (가져오기 + 재생) 또는 PC 에 이미 있는 녹화의 **"복원"**.
옵션: "모든 프레임"(`--every-frame`, 키프레임 기준 없이 포즈가 있는 모든 쌍에 FFS), FFS 반복 수(기본 8). 진행률 = 재생 위치 / 전체 s.
복원은 실시간과 같은 파이프라인이지만 느려도 정확하게: 전용 브리지(새 상태), 프레임마다 포즈를 기다림, 키프레임을 버리지 않음.
영상 프레임 ↔ CSV 행은 mp4 PTS 와 `pts_us` 로 맞춘다 (없으면 순서). 행 수와 영상 프레임 수가 다르면 짝 없는 쪽을 버리고 로그에 개수를 남긴다.

### 자동 가져오기 · 자동 복원 · 리플레이 (rec_sync.py, replay_view.py)
- 폰을 연결하면(서버 시작 때 이미 붙어 있어도) 폰의 끝난 녹화(meta complete=true)를 `scan3d/recordings/` 로 하나씩 가져온다.
  PC 에 같은 사본(파일 이름·크기 같음)이 있으면 다시 안 가져온다. [■ 정지] 로 녹화를 멈추면 마무리 뒤 바로 가져온다.
- 가져온 녹화 중 시간 정보가 있는 복원이 없는 것을 하나씩 자동 복원 (실시간·녹화 중에는 쉬었다가 끝나면 처음부터 다시).
  오른쪽 칸 "녹화본 · 리플레이" 에 녹화마다 상태(폰에만 있음 · 가져오는 중 N% · 복원 대기 · 3D 복원 중 N% · 리플레이 가능)와 버튼.
  두 체크(자동으로 가져오기 / 자동으로 복원)는 `runs/rec_auto.json` 에 남는다. [폰에서 지우기] 는 PC 사본이 같을 때만, 물어본 뒤에만.
- [▶ 보기] → 3D 화면 위 리플레이 막대: [최종 맵] (전체 복셀 + 전체 궤적) / [시간 따라 보기] (▶·속도 0.5~4×·시간 막대 끌기, 스페이스·←→).
  그 시각까지 처음 보인 복셀만, 궤적은 그 시각까지, 하늘색 절두체 = 그때 폰 자세, 오른쪽 아래 카메라 창 = 그 시각의 녹화 프레임,
  깊이 화면 = 그 시각의 키프레임. 실시간 스캔 중에는 [보기] 가 잠기고, 스캔을 시작하면 리플레이는 닫힌다.
- 복원 폴더에 더 쓰는 것: `timeline.json` (scan3d-timeline/1: 키프레임 {i, t, frames, T_world_rect0, n_shown}, 궤적, 카메라),
  `voxels_t.bin` (S3VT/1: 17 B 레코드를 처음 보인 시각 순 + float32 시각 + int32 키프레임), `kf/d_NNNNN.jpg` (깊이).
  `voxels.npz` 에 `t_first`·`t_shown`·`kf_shown` 배열 (scan3d-voxels/2, 17 B 레코드는 그대로). 녹화 폴더에 `thumbs/` (카메라 창 그림).
  시각은 모두 "녹화 기준 초" (첫 프레임 노출 가운데 = 0). 옛 복원(시간 정보 없음)은 자동으로 다시 복원한다.

### 녹화 가져오기·복원 (명령줄)
```bash
python3 scan3d/pull_rec.py list                    # 폰 녹화 목록
python3 scan3d/pull_rec.py pull --latest           # → scan3d/recordings/<이름>/   (Windows 에서도 실행 가능)
bash scan3d/run_scan.sh --replay recordings/<이름> --every-frame --replay-iters 12
venv_ffs/bin/python scan3d/replay.py recordings/<이름> --keep-web   # 끝난 뒤에도 뷰어 유지
```
재생은 전용 브리지 프로세스(pipe, `--replay-bridge-args` 기본 `--auto-reset`)를 띄우고 프레임마다 포즈를 기다린다 (버리는 프레임 없음).
결과: `recordings/<이름>/recon_<시각>/` (voxels.ply/json/npz, objects.json, trajectory.json, viewer.html, summary.json; auto-reset 이 나면 그 전 맵은 `segment_NN/`).

### 앱 0.6.37b: npu_loop 1 시작·유효 판정 (2026-10-29)
- NPU 는 UI 스레드 밖에서 연다 (화면 "NPU 준비 중..."). 실패·15 초 초과 = 시작 거부 (대체 경로 없음, meta `npu_loop` "refused: …", `run_valid` false). 시작 중 멈춤 = "cancelled", 늦게 뜬 서버는 바로 끈다.
- 끝낼 때 다리를 5 초 기다린다 (npu_loop 0 은 1.5 초 그대로). 우리 정상 정지에서 죽인 것(`bridge_killed_by_stop` true, `bridge_exit` "killed")은 유효. `run_valid` false = 0 아닌 종료(크래시·연결 실패)·다리 없음·`bridge_stderr.log` 에 "XFeat server … connected" 없음·loop.jsonl `xf_down`·서버 오류.

### 폰 저장공간 정리 (`tools/phone_cleanup.sh`, 2026-10-29)
```bash
bash scan3d/tools/phone_cleanup.sh [-s SERIAL]                    # 기본 = 미리보기만 (표 + 크기, 아무것도 안 지움)
bash scan3d/tools/phone_cleanup.sh --all-safe                     # SAFE 녹화만 지움 / --all-scratch = 시험 폴더·replay 사본
bash scan3d/tools/phone_cleanup.sh --delete hs_20261024_165506_std60 ovopt   # 이름(또는 폰 전체 경로)으로
bash scan3d/tools/phone_cleanup_test/run_tests.sh $PWD/scan3d/tools/phone_cleanup.sh   # 가짜 adb 시험 (폰 불필요)
```
- 녹화(arlat `hs_*`·`calcap*`·`ov/run_*`, dualstream `rec_*`)는 폰 파일 목록이 비어 있지 않고 **모든 파일 이름+크기**가 PC 사본 `scan3d/recordings/<이름>/`(`PC_REC` 로 바꿈)과 같을 때만 SAFE. 아니면 KEEP(이유 표시).
- adb 오류(offline·unauthorized·기기 여러 대·끝 표시 없음)면 그 자리에서 전체 중단(종료 3). 지우기 직전에 그 항목을 다시 읽어 다시 비교하고, 지운 뒤 없어졌는지 확인.
- 설정(`files/ov/<cfg>`)·NPU 모델(`files/npu`)·다른 앱·모르는 `/data/local/tmp` 이름은 후보에 안 나오고 안 지운다.

## 파일

| 파일 | 역할 |
|---|---|
| `scan3d.py` | 진입점. 세션(모드)·수신 → IMU 동기화 → VIO 입력 → 키프레임 → 깊이·분할 → 복셀 맵, 웹 API |
| `replay.py` | 녹화 재생 드라이버 (`Replayer`) + 명령줄 |
| `recording.py` | 녹화 폴더 읽기 (영상 디코딩, 프레임 CSV, imu.csv, 노출 보정) |
| `phone_link.py` | adb: 앱 시작(모드)·정지·status·녹화 목록·가져오기 (리눅스 adb 또는 WSL interop Windows adb.exe) |
| `pull_rec.py` | 녹화 목록·가져오기 명령줄 |
| `phone_rx.py` | 폰 소켓(listen/connect, stop 이벤트), `PhoneReceiver` 상속 수신(콜백), `FrameTimingIndex`, `ImuSync` |
| `ov_client.py` | OpenVINS 브리지 클라이언트 (TCP 또는 `pipe:` 자식 프로세스), `ov_bridge/PROTOCOL.md` 형식 |
| `frames.py` | 좌표계 규약과 변환 (`T_world_rect0 = T_world_I · T_I_rect0`) |
| `depth_seg.py` | FFS 깊이 + YOLO track 분할 → rect0 점·색·클래스 |
| `voxel_map.py` | 희소 복셀 해시, 클래스·인스턴스 투표, 26-이웃 연결 성분 물체, PCA 박스, 저장 |
| `web_server.py`, `web/` | HTTP 서버(0.0.0.0:8795) + three.js 뷰어 (InstancedMesh 복셀, 증분 폴링, 모드·녹화 패널) |
| `run_scan.sh` | 브리지 → scan3d.py 실행 (`--native`/`--relay`/`--replay`) |
| `setup.sh` | 설치 재현 (uv, venv_ffs, torch, FFS, Ceres, OpenVINS, ov_bridge, 설정, platform-tools) |
| `win_relay.py` | Windows 쪽 바이트 중계 + 앱 시작 (`--app-mode`), 폰 포트 접속 실패 때 adb forward 다시 |
| `preview_server.py` | 웹 화면 미리보기 (가짜 데이터, 포트 8796, 폰·GPU 불필요) |
| `tools/make_factory_pair_calib.py` | 렌즈 쌍 공장 값 보정 (2+6) + 장면 쌍으로 다듬기·yaw 고정, 2+5 공장 값 vs 체커보드 비교 |

## 주요 인자 (`scan3d.py --help`)
- `--link auto|relay|native`, `--phone listen:8766 | 127.0.0.1:8765 | :18765 | HOST:PORT`, `--adb PATH`
- `--mode live|record|both|idle` (시작 모드), `--start-app/--no-start-app` (native 기본 켬), `--app-size 640x480 --app-fps 15 --app-imu-us 2500 --app-focus 1.0` (크기는 4:3 만)
- `--ov 127.0.0.1:7600` | `pipe:<명령>` | `none`, `--ov-bridge`, `--ov-config`, `--on-vio-reset clear|keep`
- 재생: `--replay DIR`, `--every-frame`, `--replay-iters 8`, `--replay-out DIR`, `--exit-after-replay`, `--rec-root recordings`
- `--voxel 0.03`, `--min-hits 2`, `--depth-range 0.3 2.5` (최대 3 m), `--iters 4`, `--scale` (FFS 축소, 기본 자동) `--ffs-max-px 307200`, `--stride 3`
- `--kf-trans 0.03 --kf-rot 5 --kf-dt 0`, `--yolo-model yolo11m-seg.pt`, `--yolo-rotate`, `--no-yolo`
- `--port 8795`, `--save-on-exit`, `--pair-skew-ms` (쌍 묶기, 기본 = 프레임 주기 × 0.55 → 15 fps 면 36.67 ms, 가장 가까운 프레임끼리), `--vio-stereo-skew-ms 5` (stereo VIO 만)
- `--pair 2+5|2+6` (깊이 렌즈 쌍 시작값, 웹에서 바꿈), `--calib-26 calib_phone_pair_2_6.json`
- `--vio-mode mono|stereo` (기본 **mono** = 왼쪽 렌즈 lens0 + IMU), `--app-sync off|auto` (기본 **off**), `--motion-rect on|off` (기본 on = 깊이 정류를 쌍마다 움직임 보정)
- VIO 입력은 항상 정류 gray (`--ov-images rect`, raw 는 설정과 안 맞아 제거 — REVIEW 5)

## 폰 스트림 형식 (USB 끊김 줄이기)
- 앱은 기본으로 두 렌즈를 **H.264 (I+P, B-frame 없음, 키프레임 1초)** 로 보낸다 (앱 1.4-h264). 640x480·15 fps 에서 렌즈당 약 4 Mbps 이고, 크기·fps 에 비례해 늘어난다.
  - 왜 H.264: 폰의 하드웨어 인코더가 압축하므로 JPEG 보다 폰 부하·발열이 작고 USB 로 보내는 양도 훨씬 적다 (추정, 측정 전).
  - 왜 B-frame 없음: B 는 뒤 프레임을 기다려야 해서 지연이 생기고 화질이 프레임마다 출렁인다. 깊이·VIO 에는 일정한 화질이 낫다.
  - PC 는 PyAV 로 렌즈별로 풀고 (venv_ffs 에 `av` 필요), 접속 직후·끊김 뒤에는 폰에 키프레임을 요청한다.
- 고급 설정: 웹 "영상 형식" 또는 `--app-stream jpeg|nv21` (앱 인자 `--es stream_fmt jpeg`, 화질 `--ei jpeg_q 80`). NV21 원본(약 28 MB/s at 30 fps)은 이 폰에서 USB 가 끊겼다. 렌즈당 비트레이트는 `--ei live_kbps N`. 형식 자세히: SPEC.md 1.2.1·1.2.2.
- 폰이 뜨겁거나 그래도 끊기면 **15 fps 권장**: `--app-fps 15` (scan3d.py) / `--fps 15` (win_relay.py). 전송량·발열이 절반.

## 좌표
- 깊이·YOLO = 정류 왼쪽(rect0, 렌즈 2를 R1 로 돌린 OpenCV 카메라). 점 `p_rect0 = [(u-cx)/f·Z, (v-cy)/f·Z, Z]`.
- 월드 = OpenVINS G (중력 정렬 z 위). 뷰어도 z-up. auto-reset 이 나면 월드 원점이 바뀌므로 기본으로 맵을 새로 시작한다 (`--on-vio-reset`).
- 브리지가 `T_world_cam0` 를 주면 그대로(cam0 = rect0), 없으면 `q_GtoI`(JPL)·`p_IinG` × 브리지 추정 외부 파라미터(또는 보정값 `R1 @ R_sensor_to_left`).

## 알려진 한계
- 테스트·실측 안 함. 문법 검사(py_compile, bash -n)만 했다.
- 녹화 형식·위치·정지는 `RECORDING_FORMAT.md` (scan3d-phone-rec/1) 에 맞췄다. 녹화 모드 status.txt 에 경과 시간·프레임 수 필드가 있는지는 미확인 —
  뷰어는 `t=`/`frames=` 가 있으면 그 값을, 없으면 원문 줄과 "앱 시작 후 경과 s" 를 보여 준다.
- 영상 ↔ CSV: 처음 12 장으로 방식(PTS/순서)과 첫 행(0..4)을 정한다 (PTS 간격 무늬로 판정, 허용 오차 반 프레임). 드롭이 없어 간격이 일정하면
  첫 행을 가를 수 없어 0 으로 두고, meta `samples_written` ≠ 행 수면 경고한다 (미검증). 중간에 순서로 바꿔도 행 번호가 디코드 번호와 같이 움직인다.
- 두 렌즈는 동시에 안 찍힌다 (실측: 위상 10~28 ms, 카메라를 열 때 정해지고 15 fps 에서 초당 약 0.05 ms 만 미끄러짐;
  앱이 카메라를 다시 열어 맞추는 방법 C 는 19번 해도 실패). 그래서 (2026-10-16 결정)
  - **VIO 는 렌즈 하나(lens0) + IMU (mono)** 가 기본. 처음 1~2초 천천히 움직여야 초기화된다 (웹 안내). 스테레오 VIO 는 `--vio-mode stereo` (옵션, 시각차 5 ms 넘는 쌍은 버림).
  - **앱 위상 맞추기 기본 끔** (`--app-sync off`, 앱 `--es sync off`). `--app-sync auto` 는 옵션.
  - **쌍 묶기**: 기준 = 프레임 주기 × 0.55 (2026-10-18 전에는 0.45). FramePairer 가 양쪽 프레임을 다 본 뒤 가장 가까운 것을 고르므로
    먼 프레임과 짝이 되지 않는다. 쌍마다 dt = t(lens1) − t(lens0) (노출 가운데끼리) 를 같이 넘긴다.
    0.45·T (30 ms) 였을 때는 위상 30~33 ms 녹화의 짝이 통째로 버려졌다 (rec_20261018_101143: 처음 3.7 s 를 버려 VIO 초기화 실패).
  - **움직임 보정 정류** (`motion_rect.py`): 키프레임마다 두 촬영 순간 사이 폰 움직임 ΔT (회전 = 자이로 적분, 이동 = VIO 속도 × dt,
    레버암 = VIO 추정 외부 파라미터) 를 구해 실제 상대자세 (R,T)∘ΔT 로 그 쌍만 stereoRectify → FFS. 깊이는 lens0 시각 기준이고,
    쌍별 정류 좌표 rect0' 는 R1·R1'ᵀ 로 기본 rect0 에 돌려 T_world_rect0 를 곱한다. ΔT 를 모르면 (자이로 창 없음 등) |dt| ≤ 2 ms 일 때만
    고정 보정, 아니면 그 키프레임은 깊이를 안 만든다. 웹 고급 → "움직임 보정" 줄 (/api/state depth.motion) 에 키프레임별 dt·회전·이동·정류맵 시간.
  - 녹화 복원도 같은 방식 (녹화 CSV 의 렌즈별 노출 가운데 시각으로 dt, 녹화 IMU 로 ΔT).
- 원시 가속도계(TYPE_ACCELEROMETER) 는 수정된 앱(2026-10-16 설치)에서만 온다. 없으면 IMU 출력 0 → VIO 시작 안 됨 (뷰어 "IMU 동기" 줄 확인).
  실제 IMU 주기: `adb shell run-as com.camera.dualstream cat files/sensors.json`. 400 Hz 가 아니면 `make_ov_config.py --imu-rate` 로 다시 생성.
- 카메라 SENSOR_TIMESTAMP 와 센서 timestamp 가 같은 시계(elapsedRealtimeNanos)라는 가정 (S24 timestamp source 미확인).
- IMU 잡음값·카메라-IMU 평행이동은 추정값 (OpenVINS 온라인 보정 켬).
- 영상이 90° 누워 있어 YOLO 검출률이 떨어지면 `--yolo-rotate`.
- 브리지: `bad` 판정에 `R_CtoG`/`T` 유한성 검사(REVIEW 1), pose 줄에 `t_state`(OpenVINS 상태 시각) 추가(REVIEW 9) — 2026-10-16 빌드.
  클라이언트는 `t_state` 가 `t` 와도 `t + dt_cam_imu` 와도 1 ms 넘게 다르면 그 포즈를 키프레임에 쓰지 않는다.
- 앱 1.1-rec (versionCode 2) 는 2026-10-16 폰에 설치했지만 녹화(인코더·mp4)는 기기에서 아직 실행해 보지 않았다.
