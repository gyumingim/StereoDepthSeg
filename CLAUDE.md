# CLAUDE.md — StereoDepthSeg / scan3d

다음 세션·다른 PC 의 에이전트가 먼저 읽는 규칙. 짧게 유지 (150 줄 이하). 사용자에게 보이는 글은 한국어 쉬운 말.
일반 작업 규칙(계획 → 승인 → 구현, 복잡도 최소)은 `HARNESS.md`, 사용자용 운영 안내는 `scan3d/OPERATIONS.md`.

## 1. 구조 (한 단락)
Galaxy S24 앱(com.camera.dualstream)이 렌즈 2개(기본 물리 2 초광각 + 5 광각, 1440x1080 @15 fps)와 IMU 를 보내거나
폰에 녹화한다. WSL(Ubuntu-22.04)의 `scan3d/scan3d.py` 가 웹 서버(http://localhost:8795, `web_server.py`, `web/`)를 띄우고,
OpenVINS 모노 VIO(`ov_bridge/`) 포즈 + Fast-FoundationStereo 깊이(`motion_rect.py` 움직임 보정) → 복셀/TSDF 맵을 만든다.
녹화는 `scan3d/recordings/rec_*` (형식 `RECORDING_FORMAT.md`), 복원은 `replay.py` → `recordings/<녹화>/recon_<시각>/summary.json`.
폰 연결은 `phone_link.py` (relay = Windows adb + `win_relay.py` 중계, native = WSL 리눅스 adb).

## 2. 실행 명령
- 서버는 Windows 로그인 때 시작프로그램 `scan3d_server.vbs` → `scan3d/serve_idle.sh` 가 자동으로 띄운다 (대기 상태).
- 다시 켜기: Windows `powershell -ExecutionPolicy Bypass -File \\wsl.localhost\Ubuntu-22.04\home\system\scanner\StereoDepthSeg\scan3d\tools\restart_server.ps1`
  / WSL `bash scan3d/tools/restart_server.sh`. 스캔·복원 중이면 거절 (`-Force` 로 강제). /api/state 응답까지 최대 60 s, 실패하면 한 번 더.
  **다른 에이전트가 시험 중일 수 있으면 재시작하지 말 것.**
- 로그: `scan3d/runs/server_<YYYYMMDD>.log`, VIO `scan3d/runs/ov_bridge_{live,replay}_<시각>.log`, 중계 `runs/win_relay*.log`,
  폰 유지 `runs/adb_keeper.log`, Windows 쪽 `%LOCALAPPDATA%\scan3d\start_scanner.log`.
- WSL 명령은 `wsl -d Ubuntu-22.04 -e bash -lc "..."` (`-d` 필수). sudo 쓰지 않는다.
- 상태 확인은 `curl -s localhost:8795/api/state` (JSON). 명령은 `POST /api/cmd` (목록: `web_server.py` 머리말).

## 3. 폰 연결 방식 3가지 (`scan3d/runs/link.conf` 가 정함)
| 방식 | 언제 | 설정 |
|---|---|---|
| A. usbipd 직접 | 회사, USB 케이블 | `link.conf.native` 내용(LINK=native)을 `link.conf` 로, usbipd 서비스 켜기, `usbipd attach --wsl --busid 5-9` |
| B. Windows adb 중계 (USB) | USB 인데 usbipd 를 못 쓸 때 | `LINK=relay`, Windows adb 가 USB 기기를 봄 |
| C. Tailscale 무선 adb | 집·원격 | `LINK=relay`, Windows adb 가 `100.79.156.20:<포트>` 에 connect. `tools/adb_keeper.py` 가 끊기면 다시 붙임 |
- StarDesk 가 자체 usbipd 로 포트 3240 을 써서 usbipd-win 과 충돌 → 평소 usbipd 서비스는 꺼 둔다 (A 를 쓸 때만 켬).
- link.conf 를 바꾸면 서버 재시작이 필요하다. 자세한 전환 명령은 `scan3d/OPERATIONS.md`.

## 4. 일하는 순서 (오늘 비싸게 배운 것)
1. **먼저 재 보고 만들기.** 하드웨어 가정(렌즈 동기, 프레임 시각, 포트 등)은 2~5 분짜리 측정으로 먼저 확인한다.
   (렌즈 위상 맞추기 방법 A·B 를 다 만든 뒤에 S24 가 둘 다 무시한다는 것을 알았다.)
2. **녹화로 끝까지 되는지 먼저.** 녹화 1 개 → 복원 → 맵이 나오는 것을 오프라인으로 먼저 확인하고, 같은 녹화로 반복 개선 →
   그다음 실시간 → 그다음 기능 추가. 실시간이 한 번도 끝까지 안 된 상태에서 기능(해상도 선택, 웹 꾸밈)을 만들지 않는다.
3. 고친 뒤에는 `scan3d/tools/regress.py` 로 참조 녹화가 나빠지지 않았는지 본다 (6 절).
4. **GPU 무거운 일은 한 번에 하나.** 복원(replay)·FFS 평가·학습을 여러 개 동시에 돌리지 않는다 (복원 병렬 금지).
   PC 가 전원 쪽 문제로 갑자기 꺼진 적이 있다 (10/13, 10/14, 10/17). 서버에 복원을 보낼 때도 끝날 때까지 기다렸다가 다음.
5. 긴 작업 기다리기는 `sleep` 사슬이 아니라 조건 폴링(`until curl ... /api/state` 의 영어 키)이나 백그라운드 + 알림.

## 5. 에이전트 지시 규칙
- 하위 에이전트 지시문은 **"구현 작업 (질문 아님)"** 으로 시작 (아니면 상태 질문으로 읽고 아무것도 안 한 적 있음).
- 병렬로 돌 때는 **파일 소유**를 적는다: 누가 어떤 파일을 고치는지, 남의 파일은 읽기만. 공용 파일(`scan3d.py`, `web_server.py`)은 한 명만.
- 서버 재시작·폰 앱 재시작·adb 끊기는 다른 에이전트의 시험을 깬다 → 지시문에 허용 여부를 적는다.
- 결과는 파일로 남기고 경로 + 짧은 요약만 돌려준다. 자기 작업을 자기가 "완료" 판정하지 않는다 (regress.py 결과로).

## 6. 검증
```bash
python3 scan3d/tools/regress.py --dry-run     # 서버가 idle 인지, 참조 녹화가 있는지만
python3 scan3d/tools/regress.py               # 참조 녹화 4 개 복원 (몇 분, GPU 사용 — 다른 GPU 일 없을 때)
python3 scan3d/tools/regress.py --only 175446 # 하나만
python3 scan3d/tools/regress.py --show        # 마지막 두 결과 비교표만
python3 -m pyflakes scan3d/<고친 파일>.py      # WSL. Windows 스크립트는 [scriptblock]::Create 로 구문 확인
```
- regress.py: 서버 API 로 하나씩 복원(`{"cmd":"replay","name":..,"pair":..}`), 끝나면 summary.json 의 키프레임·복셀·VIO 리셋·
  평면 잔차·재위치(reloc*)·tsdf* 숫자를 표로, `scan3d/runs/regress_<시각>.json` 저장, 바로 전 결과보다 10 % 넘게 나빠지면
  `WORSE` 와 종료 코드 1. 서버가 실시간·복원 중이면 거절. 서버를 켜거나 끄지 않는다.
- 기준을 맞추려고 판정 비율을 느슨하게 하거나 녹화를 빼지 않는다.

## 7. 참조 녹화 (scan3d/recordings, 지우지 말 것)
| 녹화 | 내용 | 용도 |
|---|---|---|
| rec_20261016_143702 | 640x480, 렌즈 2+5, 26 s | 시작 정지 → VIO 정적 초기화, 키프레임 많음 (옛 해상도 회귀) |
| rec_20261016_165645 | 1440x1080, 렌즈 2·5·6 동시, 19 s | 렌즈 쌍 비교 (2+5 vs 2+6, 같은 VIO 포즈 재사용). 회귀는 pair 2+5 |
| rec_20261016_175446 | 1440x1080, 2+5, 14 Mbps, 17 s | 지금 기본 설정. 추적 끊김 1 회 → 리셋·조각·재위치 시험 |
| rec_20261016_162204 | 1440x1080, 2+6, 37 s, 215 MB | 긴 녹화, 2+6 쌍, 리셋 1 회 |
| (rec_20261016_172949) | 1 s 짧은 녹화 | 빠른 스모크 (키프레임 0 — 회귀에는 안 씀) |

## 8. 폰 실측 사실 (다시 재지 말 것, 폰·케이스가 바뀌면 다시)
- 논리 카메라 0 = 물리 2(초광각, y +15.76 mm) · 5(광각, 0) · 6(망원 3x, y −16.60 mm). 3 렌즈 동시 1440x1080 @15 fps 녹화 됨.
- **렌즈 2·5 는 동기 안 됨**: 위상 차 10~33 ms (카메라 열 때 정해짐), 15 fps 에서 약 0.05 ms/s 미끄러짐.
  SENSOR_FRAME_DURATION·세션 재시작으로 못 맞춤 → 깊이는 움직임 보정 정류(motion_rect.py), 스테레오 VIO 는 못 씀.
- 렌즈 2·6 은 거의 동기(1~3 ms), 기준선 32.4 mm, 공장 보정 약 2° 틀림. 망원은 초광각 가운데만 겹침.
- **결정 2026-10-16: 깊이 쌍 2+5, 1440x1080 @15 fps** (2+5 가 맵 넓이 9 배, 2+6 평면 오차는 5~25 % 만 나음). 녹화 14 Mbps.
- OpenVINS 모노(렌즈 2 왜곡 제거 "lens0u", f≈262): 시작 때 약 2 s **완전히 정지** 필요. 초기화 기준을 느슨하게 하면
  자이로 바이어스가 잘못 잡혀 0.3 m/s 표류.
- 프레임 시각 = 노출 가운데 (시작 + 노출/2 + 롤링셔터/2), 앱 1.7-mid 이상.

## 9. Windows / WSL 함정
- PowerShell `Where CommandLine -like '*x*'` 로 죽이면 그 PowerShell 자신도 맞는다 → `$PID`·powershell 프로세스 제외.
  bash `pkill -f` 는 `'scan3d/[s]can3d.py'` 처럼 대괄호로 자기 자신과 안 맞게.
- WSL interop 로 띄운 PowerShell(특히 로그인 때 숨은 wsl.exe 아래)은 환경 변수가 거의 없다 (TEMP·APPDATA·긴 PATH 없음)
  → Windows 경로는 WSL 쪽에서 찾아 넘기거나 `C:\Windows\System32\...` 고정 경로.
- Git Bash → `wsl -e bash -lc` 로 **한글 패턴 grep 금지** (인코딩이 깨져 아무것도 안 맞음). JSON API·영어 낱말로 기다린다.
- `adb install` 에 `\\wsl.localhost\...` UNC 경로는 실패 → APK 를 Windows 임시 폴더로 복사 후 설치.
- StarDesk 의 usbipd.exe 가 포트 3240 을 써서 usbipd-win 과 충돌.
- PowerShell 5.1 은 BOM 없는 UTF-8 .ps1 의 한글을 깨뜨린다 → .ps1 은 UTF-8 BOM. .vbs 는 CRLF. 나머지는 LF.
- PowerShell 5.1 이 네이티브 명령에 넘기는 인자 안의 큰따옴표는 망가진다 → `wsl ... bash -lc` 문자열 안에는 작은따옴표만.
- (2026-10-19) **Git Bash `sed -i` 로 `\\wsl.localhost\...` 파일을 고치지 말 것** — 임시 파일 이름 바꾸기가 실패하면 원본이 지워지고
  `.msys000…` 만 남는다 (redo_hs_gt.sh 가 사라져 녹화 2개 정답·학습 7개가 실패). WSL 안의 `sed -i`/python 이나 편집 도구로.
  Windows python `open(..,'w')` 은 CRLF 로 써서 셸 스크립트가 안 돈다 → WSL 쪽에서 쓰거나 `sed -i 's/\r$//'`.
- **무거운 계산(영상 디코드·정답·학습)은 WSL 안에서** (`venv_ffs`, OpenCV 5 + torch). Windows python 으로 `\\wsl.localhost` 를 읽으면 3~4 배 느림.
- **push 는 Windows git 으로**: WSL git 에는 GitHub 로그인이 없어 https push 가 사용자 이름 질문에서 멈춘다 (백그라운드 push 는 조용히 영영 안 끝남).
  `cd //wsl.localhost/Ubuntu-22.04/home/system/scanner/StereoDepthSeg && git -c safe.directory='*' push origin gyumingim` (Git Bash).
- `vio_bench_server.py` 를 **녹화를 PC 로 가져오는 중에 재시작하지 말 것** (`GET /api/pull` 의 `active` 먼저 확인) — 가져오기는 이어져도 폰 사본 정리·진행률이 끊긴다.
- WSL 백그라운드 실행은 `nohup … &` (그냥 `&` 는 `wsl -e` 가 끝나면 같이 죽는다).

## 10. 커밋 규칙
- 사용자가 말할 때만 커밋·push. force push 금지. `git add -A` 금지 (다른 에이전트 작업 파일이 섞임) — 자기 파일만 경로로.
- 메시지: 영어 한 줄 요약 (+ 본문). 끝에 attribution 줄:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`
- 녹화(mp4)·recon 결과·runs/ 는 올리지 않는다.

## 11. 오래 걸리는 작업을 맡겨 둘 때 (2026-10-19 밤 사고 → 규칙)
- **시작 전에 작은 조각으로 시간을 잰다** (학습 1 epoch, 녹화 1 개 처리). 학습 8 개 동시 실행을 재지 않고 걸었다가 2~3 시간 예상이 9.5 시간 → VIO 비교를 못 함.
- **끝 알림 + 10 분 감시견을 둘 다 건다**: 끝 알림만 걸면 느림·멈춤·조용한 실패를 아침까지 모른다 (Claude 는 알림이나 메시지가 와야만 움직인다).
  감시견 = 백그라운드로 `sleep 600; bash scan3d/tools/watchdog.sh` (진행률 요약), 깨어날 때마다 확인하고 다시 건다.
- **동시 실행 수는 CPU 에 맞춰** (IMU 적분 반복문이 CPU 를 씀): 학습은 4 개 안팎. 녹화 읽은 결과(창·정답 곡선)는 저장해 같이 쓰기.
- **다음 단계가 기다리는 결과는 그 단계가 끝나는 즉시 저장** (train_imu 가 끝에서만 result.json 을 써서, 결론에 필요 없는 CNN 단계 때문에 VIO 비교가 막혔다).
- 바뀔 예정인 앞 단계(렌즈·정답) 위에서 뒤 단계(학습)를 돌리지 않는다.

## 12. 모르는 함정 찾기 (2026-10-20: 녹화 IMU 가 Android 보정판이었음 → 규칙)
- **새 데이터 출처(앱·녹화 형식·센서·기기)를 쓰기 전, 큰 단계(정답·학습) 시작 전**: `scan3d/docs/input_audit_checklist.md` 를 채운다
  ("이 값은 내 손에 오기 전에 누가 손댔나?"). 확인 안 된 칸은 보고에 "가정" 으로 쓴다.
- **학습 전에 물리 상식 시험**(가만히 두기·한 축 돌리기)을 먼저 한다 — 정지 16 분이 하룻밤 학습보다 먼저 문제를 보여 줬다.
- **처리 전·후를 둘 다 저장**: IMU 원본(16·35)과 보정판(4·1), 카메라 요청값과 CaptureResult 실제값.
- **가정 검토 에이전트**: 새 단계를 시작할 때, 만든 사람이 아닌 에이전트에게 "이 파이프라인이 입력에 대해 당연하게 여기는 가정을
  전부 나열하고 각각 어떻게 확인됐는지 표시하라" 를 맡긴다. 확인 안 된 가정 = 점검할 목록.
- 처음 쓰는 출처는 "<출처> pitfalls" 로 남들이 겪은 함정을 먼저 찾는다 (출처 URL 달기).
- **메타인지 검사 (2026-10-20 첫 실행: 버그 2·숨은 데이터 문제 2·과장 1 발견)** — 결과 `scan3d/docs/input_audit_report.md` 형식.
  - **언제 (반드시)**: ① 새 데이터 출처·새 녹화 형식을 쓰기 전 ② 30 분 넘는 실행(학습·정답 전체·VIO 행렬)을 걸기 전
    ③ 결론을 보고서·HTML 에 쓰기 전. 셋 중 하나면 돌리고, 끝나기 전에는 그 단계를 시작하지 않는다.
  - **누가**: 만든 에이전트가 아닌 새 에이전트. 읽기 전용 (코드·데이터 수정 금지, 폰은 dumpsys·getprop 만).
  - **무엇을**: (a) 입력값마다 만들어지는 곳·숨은 처리·단위·축·부호·시계·보간 — 근거는 file:line 이나 명령 출력
    (b) 실제 데이터 통계 (표본 간격·빈틈·중복·누락 프레임) (c) **사용자에게 한 주장 목록을 근거와 대조**
    (예: "Android 바이어스 계단이 녹화 차이의 원인" → 실제로는 자이로 1 LSB = 0.07 °/s 라 과장이었음).
  - **수시로**: 위 세 시점 말고도 큰 작업 단위(에이전트 결과 하나)가 끝날 때마다 돌린다 (사용자 지시 2026-10-20).
  - **지적 검증**: 검사가 낸 지적은 또 다른 새 에이전트가 하나씩 반박을 시도해 "맞음 / 틀림 / 불확실" 로 판정한다.
    맞음으로 남은 것만 고친다 (검사자도 틀린다 — 오늘 OIS 항목은 기억 기반 '가정' 이었다).
  - **결과 처리**: 위험 높음 항목은 고치고 다시 돌리기 전까지 관련 숫자를 보고서에 "확정" 으로 쓰지 않는다.
    이미 사용자에게 말한 주장이 틀렸으면 바로 정정한다.
