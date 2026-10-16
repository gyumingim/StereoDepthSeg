# scan3d — 실시간 3D 복셀 맵 스캐너

폰(Galaxy S24, 렌즈 2 초광각 + 렌즈 5 광각, NV21 640x480 + IMU) → (Windows 릴레이 | 우분투 adb forward) → 리눅스:
OpenVINS(ROS 없음) 스테레오+IMU VIO 로 자세, Fast-FoundationStereo 깊이 + YOLO-seg 분할 → 복셀 맵(물체별 구분) → 웹 뷰어.
폰에 녹화해 두었다가 PC 로 가져와 같은 파이프라인으로 천천히·정확하게 복원할 수도 있다.
인터페이스 명세: `SPEC.md`. 코드 검토: `REVIEW.md`. 녹화 형식: `RECORDING_FORMAT.md` (scan3d-phone-rec/1, 앱 1.1-rec), 읽기: `recording.py`.

## 모드 (웹 뷰어 왼쪽 위 "모드")

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
   (찾는 순서: `SCAN3D_ADB` → PATH 의 adb.exe → `C:\Users\*\AppData\Local\Android\Sdk\platform-tools\adb.exe` → …, `phone_link.py`).
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

### 녹화 가져오기·복원 (명령줄)
```bash
python3 scan3d/pull_rec.py list                    # 폰 녹화 목록
python3 scan3d/pull_rec.py pull --latest           # → recordings/<이름>/   (Windows 에서도 실행 가능)
bash scan3d/run_scan.sh --replay recordings/<이름> --every-frame --replay-iters 12
venv_ffs/bin/python scan3d/replay.py recordings/<이름> --keep-web   # 끝난 뒤에도 뷰어 유지
```
재생은 전용 브리지 프로세스(pipe, `--replay-bridge-args` 기본 `--auto-reset`)를 띄우고 프레임마다 포즈를 기다린다 (버리는 프레임 없음).
결과: `recordings/<이름>/recon_<시각>/` (voxels.ply/json/npz, objects.json, trajectory.json, viewer.html, summary.json; auto-reset 이 나면 그 전 맵은 `segment_NN/`).

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
| `win_relay.py` | Windows 쪽 바이트 중계 + 앱 시작 (`--app-mode`) |

## 주요 인자 (`scan3d.py --help`)
- `--link auto|relay|native`, `--phone listen:8766 | 127.0.0.1:8765 | :18765 | HOST:PORT`, `--adb PATH`
- `--mode live|record|both|idle` (시작 모드), `--start-app/--no-start-app` (native 기본 켬), `--app-fps 30 --app-imu-us 2500 --app-focus 1.0`
- `--ov 127.0.0.1:7600` | `pipe:<명령>` | `none`, `--ov-bridge`, `--ov-config`, `--on-vio-reset clear|keep`
- 재생: `--replay DIR`, `--every-frame`, `--replay-iters 8`, `--replay-out DIR`, `--exit-after-replay`, `--rec-root recordings`
- `--voxel 0.03`, `--min-hits 2`, `--depth-range 0.3 2.5` (최대 3 m), `--iters 4`, `--scale 1.0`, `--stride 3`
- `--kf-trans 0.03 --kf-rot 5 --kf-dt 0`, `--yolo-model yolo11m-seg.pt`, `--yolo-rotate`, `--no-yolo`
- `--port 8795`, `--save-on-exit`
- VIO 입력은 항상 정류 gray (`--ov-images rect`, raw 는 설정과 안 맞아 제거 — REVIEW 5)

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
- 두 렌즈 노출 동기 APPROXIMATE → 시각차 > 1 ms 면 mono 로 VIO 에 보냄.
- 원시 가속도계(TYPE_ACCELEROMETER) 는 수정된 앱(2026-10-16 설치)에서만 온다. 없으면 IMU 출력 0 → VIO 시작 안 됨 (뷰어 "IMU 동기" 줄 확인).
  실제 IMU 주기: `adb shell run-as com.camera.dualstream cat files/sensors.json`. 400 Hz 가 아니면 `make_ov_config.py --imu-rate` 로 다시 생성.
- 카메라 SENSOR_TIMESTAMP 와 센서 timestamp 가 같은 시계(elapsedRealtimeNanos)라는 가정 (S24 timestamp source 미확인).
- IMU 잡음값·카메라-IMU 평행이동은 추정값 (OpenVINS 온라인 보정 켬).
- 영상이 90° 누워 있어 YOLO 검출률이 떨어지면 `--yolo-rotate`.
- 브리지: `bad` 판정에 `R_CtoG`/`T` 유한성 검사(REVIEW 1), pose 줄에 `t_state`(OpenVINS 상태 시각) 추가(REVIEW 9) — 2026-10-16 빌드.
  클라이언트는 `t_state` 가 `t` 와도 `t + dt_cam_imu` 와도 1 ms 넘게 다르면 그 포즈를 키프레임에 쓰지 않는다.
- 앱 1.1-rec (versionCode 2) 는 2026-10-16 폰에 설치했지만 녹화(인코더·mp4)는 기기에서 아직 실행해 보지 않았다.
