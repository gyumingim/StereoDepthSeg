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

## 6. 녹화를 인터넷으로 올리기 (집에서, adb 없이 빠르게)

집 → 회사 PC 는 adb 가져오기가 느리다 (도쿄 중계, 실측 **0.3~0.7 MB/s**, 무선 디버깅이 꺼지면 끊김).
대신 PC 가 Cloudflare 터널(바깥으로 나가는 연결만)을 열고, **폰 앱이 녹화가 끝나면 HTTPS 로 직접 올린다**
(실측 2026-10-18: 브라우저 48 MB 2.4~3.1 MB/s, 앱 1.6 GB 녹화 약 3~4 MB/s → adb 의 4~10 배).

- **자동으로 됨**: 서버(`serve_idle.sh`)가 뜰 때 `upload_server.py`(포트 8799)와 `cloudflared` 를 같이 띄운다.
  scan3d 서버를 다시 켜도 이 둘은 남아서 터널 주소가 그대로다. 로그 `scan3d/runs/upload_server.log`, `runs/cloudflared.log`.
- 폰 설정: adb 가 붙어 있으면 PC 가 30 초마다 폰의 `files/upload.json`(주소·토큰)을 확인하고 다르면 다시 넣는다.
  → **터널 주소가 바뀐 뒤(PC 재부팅·cloudflared 재시작) 한 번은 adb 가 붙어야** 폰이 새 주소를 안다.
  바로 넣기: `wsl -d Ubuntu-22.04 -e bash -lc "cd ~/scanner/StereoDepthSeg/scan3d && python3 upload_server.py --push-now"`
- 앱(1.9-upload): 녹화가 끝나면 알림 "녹화 업로드" 가 뜨고 올린다. 끊기면 받은 데까지 이어서, 다 올라가면 PC 가 sha256 을
  확인한 뒤 `recordings/<녹화>` 로 옮기고 → 자동 복원이 집어 간다. 웹 녹화 목록에 "폰이 올리는 중 NN% · X MB/s".
  - 앱을 손으로 열기만 해도(adb 없이) 아직 안 올린 녹화를 올린다. 실시간 스트림 중에는 쉰다. 기본은 **Wi-Fi 에서만**.
  - 업로드가 켜져 있으면 PC 가 녹화 [정지] 뒤 앱을 강제 종료하지 않는다 (올리는 중이라). 다음 [스캔 시작] 은 앱을 다시 켠다
    (올리던 것은 끊겼다가 이어서 올림).
- 설정 `scan3d/runs/upload.json`:
  `enabled`(끄면 앱이 안 올리고 옛 방식 adb 가져오기), `wifi_only`, `delete_after`(PC 확인 뒤 폰에서 지우기, 기본 false),
  `max_rec_mb`(녹화 하나 최대, 기본 4096), `tunnel`("quick"), `public_url`(아래 고정 주소). 바꾸면 30 초 안에 폰에 반영.
- 보안: 토큰 `scan3d/runs/upload_token.txt`(PC 가 만듦) 없으면 401. 받은 파일은 `recordings/.uploading/` 에만 쓰고,
  녹화 이름·파일 이름 형식 검사, 조각 최대 32 MB, 디스크 여유 2 GB 미만이면 거절. 토큰을 바꾸려면 파일을 지우고
  `upload_server.py` 를 다시 켜면 새로 만들고 폰에 다시 넣는다.
- adb 자동 가져오기와 겹치지 않게: 업로드가 켜져 있으면 폰에서 끝난 녹화는 3 분 동안(받는 중이면 계속) adb 로 안 가져온다.
- **고정 주소로 바꾸기 (선택, Cloudflare 계정·도메인 필요)**: 이 PC 에는 이미 이름 있는 터널 `kit-univ2`(서비스 "Cloudflared")가 있다.
  Cloudflare Zero Trust → Networks → Tunnels → kit-univ2 → Public Hostname 추가: 예 `scan.<내 도메인>` → `http://localhost:8799`.
  그다음 `runs/upload.json` 에 `"public_url": "https://scan.<내 도메인>"` 를 넣고 upload_server 를 다시 켜면 quick tunnel 은
  안 띄우고 그 주소를 폰에 넣는다 → PC 를 다시 켜도 주소가 안 바뀜.
- 원격 녹화 시작·정지 (adb 없이): 아직 없음. 앱이 서버에 명령을 묻는 방식은 가능하지만, 안드로이드가 백그라운드에서
  카메라 화면을 여는 것을 막아서 (앱이 앞에 떠 있어야 함) 앱을 손으로 열어 두는 조건이 붙는다.
