# 3D 스캐너 운영 안내 (사용자용)

웹 화면: 브라우저에서 **http://localhost:8795**. 서버는 Windows 에 로그인하면 저절로 켜진다.
아래 경로 줄임: `REPO` = `\\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg`

## 1. 폰 연결하기

연결 방식은 파일 하나 `scan3d/runs/link.conf` 가 정한다. 바꾼 뒤에는 서버를 다시 켠다 (5 절).

### 회사에서 — USB 케이블 (usbipd 로 폰을 WSL 에 직접)
1. 관리자 PowerShell 에서 usbipd 서비스 켜기 (StarDesk 와 포트 3240 이 겹치므로 StarDesk 원격은 잠시 못 씀):
   ```powershell
   Stop-Service StarDeskService -ErrorAction SilentlyContinue   # StarDesk 가 3240 을 잡고 있으면
   Start-Service usbipd
   usbipd list                                  # 폰(SAMSUNG) BUSID 확인, 보통 5-9
   usbipd attach --wsl --busid 5-9
   ```
2. 연결 방식을 native 로:
   `wsl -d Ubuntu-22.04 -e bash -lc "cp ~/scanner/StereoDepthSeg/scan3d/runs/link.conf.native ~/scanner/StereoDepthSeg/scan3d/runs/link.conf"`
3. 서버 다시 켜기 (5 절).
- usbipd 를 안 쓰고 USB 로만 써도 된다: link.conf 를 `LINK=relay` 로 두면 Windows adb 가 USB 폰을 보고 중계한다.

### 집에서 — Tailscale 무선 adb
1. 폰: Tailscale 켜기, 개발자 옵션 → **무선 디버깅** 켜기. (처음 한 번: PC 에서 `adb pair <폰IP>:<페어링포트>` + 6 자리 코드)
2. PC: usbipd 서비스 끄기 (관리자 PowerShell `Stop-Service usbipd`), StarDesk 는 다시 써도 됨.
3. 연결 방식을 relay 로:
   `wsl -d Ubuntu-22.04 -e bash -lc "echo LINK=relay > ~/scanner/StereoDepthSeg/scan3d/runs/link.conf"`
4. 폰 연결: `C:\Users\a3162\platform-tools\adb.exe connect 100.79.156.20:<무선 디버깅 화면의 포트>`
   포트는 폰이 바꿀 때마다 달라진다 → **adb_keeper** 가 15 초마다 확인해서 끊기면 포트를 찾아 다시 붙인다.
   - 한 번 시험: `py -3.12 REPO\scan3d\tools\adb_keeper.py --once` (`--dry-run` 이면 찾기만)
   - 숨김 실행: `pythonw REPO\scan3d\tools\adb_keeper.py` (로그 `scan3d\runs\adb_keeper.log`)
   - 로그인 때 자동 시작 (설치는 직접):
     `copy "REPO\scan3d\tools\adb_keeper.vbs" "%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\"`
   - USB 로 연결돼 있거나 usbipd 로 WSL 에 붙어 있으면 adb_keeper 는 아무것도 안 한다.

## 2. 녹화 요령
- **시작하고 2 초 동안 폰을 완전히 멈춘다** (움직임 추적이 이때 자리를 잡음).
- 처음 몇 초는 천천히. 회전은 **초당 0.6~1 rad (약 35~57°) 이하**, 휙 돌리지 않는다.
- 화면에 "추적 실패"가 뜨면 **2 초 멈춘다** → 다시 초기화된다.
- 0.3~2.5 m 거리, 무늬가 있는 곳이 잘 된다 (흰 벽만 보면 깊이·추적 둘 다 약함).
- 용량: 1440x1080 · 14 Mbps · 30 초 ≈ **105 MB** (렌즈 2 개). 3 렌즈 녹화는 1.5 배.

## 3. 문제 해결
| 증상 | 원인 | 할 일 |
|---|---|---|
| 웹에 폰이 안 보임 | 연결 방식(link.conf)과 실제 연결이 다름 | USB+usbipd 면 native, 무선·USB 중계면 relay 로 맞추고 서버 다시 켜기 |
| `adb devices` 가 비어 있음 (no devices) | 무선 디버깅 포트가 바뀜 / usbipd 가 폰을 WSL 로 가져감 | 무선: adb_keeper 로그 보기 또는 `adb connect` 다시. USB: `usbipd list` 에서 Attached 인지 |
| 폰이 `unauthorized` | 폰에서 허용 안 함 | 폰 화면의 "USB 디버깅 허용" 누르기 |
| 앱 시작 실패 | 폰 잠김 / 앱 미설치 / 권한 | 폰 화면 켜고 잠금 풀기, 앱 한 번 손으로 열어 권한 허용 |
| 렌즈 짝 0 % | 두 렌즈 시각이 너무 어긋남 / 렌즈 하나가 안 옴 | 앱 다시 시작 (웹 [스캔 정지]→[시작]), 계속이면 해상도 1440x1080 @15 확인 |
| 추적 실패 반복 | 시작 때 안 멈춤 / 너무 빠름 / 무늬 없는 곳 | 2 초 멈춤, 천천히, 무늬 있는 쪽을 향하기 |
| 복원 실패, 맵 비었음 | 녹화가 너무 짧음 / 처음에 안 멈춤 / 깊이 모델 미준비 | 녹화 10 초 이상, 시작 2 초 정지. `scan3d/runs/server_<날짜>.log` 끝부분 보기 |
| 웹(8795)이 안 뜸 | 서버가 안 켜짐 (재부팅 후 로그인 전 등) | 5 절 다시 켜기. 그래도 안 되면 로그 확인 |

## 4. PC 가 갑자기 꺼질 때 (10/13, 10/14, 10/17)
- 기록: Kernel-Power 41, BugcheckCode 0, 덤프 없음 → 소프트웨어가 아니라 **전원 쪽 하드웨어**(파워·전원선·멀티탭) 의심.
- 해 둔 대책:
  - BIOS **Restore on AC Power Loss = Power On** (전기가 돌아오면 저절로 켜짐) + RTC 매일 04:00 자동 켜기.
  - GPU 전력 제한 250 W (부팅 때 작업 스케줄러), CPU 최대 상태 80 % (전원 옵션).
  - Wake-on-LAN 켬, 원격 데스크톱(RDP) 켬 — 밖에서 Tailscale 로 `100.92.214.97` 에 접속.
  - Windows 업데이트: 로그인 중에는 자동 재부팅 안 함.
- 원격으로 완전히 멈췄을 때: **스마트 플러그**로 전원을 껐다 켜면 BIOS 설정 덕에 다시 켜진다.
- **주의: 다시 켜져도 스캐너 서버는 Windows 에 로그인해야 시작된다** (시작프로그램 vbs). 원격이면 RDP 로 로그인하거나,
  자동 로그인을 설정하거나, 로그인 뒤 `restart_server.ps1` 를 실행.
- 무거운 GPU 일(복원·평가)을 동시에 여러 개 돌리지 않는다 (전력이 몰리면 꺼질 수 있음).

## 5. 서버 다시 켜기
```powershell
powershell -ExecutionPolicy Bypass -File REPO\scan3d\tools\restart_server.ps1          # 스캔·복원 중이면 거절
powershell -ExecutionPolicy Bypass -File REPO\scan3d\tools\restart_server.ps1 -Force   # 그래도 끔
```
WSL 에서는 `bash ~/scanner/StereoDepthSeg/scan3d/tools/restart_server.sh`. 최대 60 초 기다리고 안 뜨면 한 번 더 시도한다.
완전히 끄기는 `scan3d\stop_scanner.bat`.
