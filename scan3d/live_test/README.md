# 3D 스캔 시험 (맵 모드) — 켜는 법

## 한 줄
```
powershell -ExecutionPolicy Bypass -File C:\Users\a3162\scan3d_handoff\live_test\start_live.ps1
```
페이지: **http://localhost:8797** (스크립트는 창을 열지 않는다. 브라우저에서 직접 연다)

이 명령이 하는 일:
1. 서버 `live_server.py` 를 창 없이 켠다. 이미 켜져 있으면 그대로 쓴다.
2. 준비 확인을 한다. adb 에 폰이 보이는지, 폰에 arlat 앱이 있는지 본다. 노트북 연결과 GPU 잠금(lock free)을 확인하고, 코드를 노트북으로 보낸다(push).
3. 폰 녹화를 시작한다. 앱은 arlat `mode hs, session std`, 1920x1080 60 fps, 초점 0.5 dpt, 160 Mbps 로 녹화한다.
   폰 화면에 "가만히 → 천천히 둘러보기 → 가만히" 안내가 나오고, 정한 시간(기본 30 s)이 되면 **앱이 스스로 멈춘다** (열 때문에 상한을 둔다).
   PC 쪽에도 안전 상한이 있다. 30 s + 25 s 가 지나도 녹화가 계속되면 PC 가 앱을 끈다.

옵션: `-Secs 45` (10~120), `-Kbps 40000` (무선이면 가져오기가 4배 빠름, 미검증), `-NoPhone` (서버만 켬), `-Stop` (STOP 버튼과 같음).

## 실시간이 아니다
지금 폰 앱은 녹화하면서 키프레임을 PC 로 보내는 기능이 없다. 그래서 순서는 이렇다:
**녹화 → 멈춤 → PC 로 가져오기 → 노트북에서 메시 만들기(4~6 분) → 페이지에 메시가 뜸.**
녹화 중에 메시가 자라는 진짜 실시간을 하려면 앱을 고쳐야 한다. ov 모드의 VIO 자세에 키프레임 1920 JPEG 를 붙여 보내도록 하고, 새로 설치해야 한다. 이번에는 하지 않았다.

## STOP 버튼 (페이지 왼쪽 큰 빨간 버튼)
| 누를 때 단계 | 하는 일 |
|---|---|
| 준비 확인 | 시작을 취소한다. 폰 카메라는 켜지 않는다 |
| 녹화 중 | 폰에 HOME 키를 보낸다. 앱이 onPause 에서 mp4·meta.json 을 닫는다. 3 초 뒤 `am force-stop com.scan3d.arlat` 을 보낸다. **찍은 만큼 이어서 메시로 만든다** |
| PC 로 가져오는 중 | adb pull 을 멈춘다. 녹화는 폰에 그대로 남는다 |
| 노트북 작업 중 | 노트북의 작업 프로세스 그룹을 끈다. remote_laptop.sh 가 그때까지 만든 결과를 가져오고 노트북을 정리한다(잠금 풀기, 캐시 지우기) |
| 그 밖 | 폰 앱 force-stop 만 한다 (안전용) |

## 데이터 위치
- 폰: `/sdcard/Android/data/com.scan3d.arlat/files/hs_<시각>_std60/`. 폰 사본은 지우지 않는다.
- PC 녹화: WSL `~/scanner/StereoDepthSeg/scan3d/recordings/hs_<시각>_std60/`. 잠깐 거쳐 가는 `C:\Users\a3162\scan3d_live\recordings` 는 WSL 로 복사한 뒤 지운다.
- 노트북 결과: WSL `~/scanner/StereoDepthSeg-livetest/scan3d/runs/laptop/<시각>_std60/live/scan/` (`scan_summary.json`, `mesh/mesh.ply`). `live/scan.log` 와 `job_*.log` 도 있다.
- 페이지용 메시 사본: `C:\Users\a3162\scan3d_live\meshes\<녹화>\mesh_mesh.ply`
- 서버 로그: `C:\Users\a3162\scan3d_live\logs\server_<날짜>.log`. 노트북 작업 로그: WSL `~/scanner/live_test_logs/<녹화>.log`

## 메시 만드는 설정 (scanner 기본값 11-11, 바꾸지 않음)
`remote_laptop.sh job <녹화> scan --tag live` 를 실행하면 노트북에서 `scan3d/scanA/run_scanner.sh` 가 돈다. 단계는 다음과 같다.
1. VIO c4f60 을 다시 돌린다.
2. 움직임으로 키프레임을 고르고(3 cm / 4°), 1920 JPEG q90 으로 만든다.
3. 960 으로 줄여 계산한다. kf_ba BA 는 GPU 에서 돈다.
4. FFS 깊이를 구하고 TSDF 5 mm 로 메시를 만든다.

렌즈는 0.5 dpt std 녹화이면 저절로 골라진다. 크기(축척)는 VIO 값 그대로라 수 % 틀릴 수 있다. 모양을 눈으로 보는 시험이다.

## 주의
- **무선 adb 가 느리다.** 11-15 에 재 보니 0.35 MB/s 였다 (60 MB 를 170 s 에 받음, 폰이 다른 망이고 Tailscale 을 거침).
  160 Mbps 로 30 s 를 찍으면 약 570 MB 라서 가져오기만 **약 27 분**이 걸린다. **USB 선으로 꽂으면** 서버가 USB 를 먼저 쓴다 (1 분 안쪽으로 추정).
  선이 없으면 `-Kbps 40000` 을 쓰면 약 7 분이다. 다만 40 Mbps 화질로 메시 품질이 같은지는 확인하지 않았다.
- 폰 화면이 잠겨 있으면 앱이 녹화를 시작하지 못할 수 있다. 25 s 안에 녹화 폴더가 안 생기면 서버가 앱을 끄고 "실패" 를 띄운다.
- 노트북이 다른 작업 중(lock)이면 시작하지 않는다.
- 서버 끄기: 작업 관리자에서 python (live_server.py) 를 끈다. 또는 PowerShell 에서
  `Get-CimInstance Win32_Process -Filter "Name='python.exe'" | ? CommandLine -like '*live_server.py*' | % { Stop-Process -Id $_.ProcessId }`

## 폰 없이 확인한 것 (11-15)
- 녹화본 hs_20261112_131328_std60 (61 s) 을 페이지 "폰 없이 시험" 으로 끝까지 돌렸다. 노트북에서 rc 0, 6.1 분 걸렸다.
  단계별 시간: vio 58 s, reduce 10 s, kfba 82 s, mesh 195 s, 보내기 약 11 s.
  만든 결과: 키프레임 498 장 (1920 JPEG q90), BA cuda / SIFT cpu, 메시 53 k 꼭짓점. 페이지에서 메시 파일을 받는 것까지 확인했다.
- STOP 을 노트북 작업 중에 두 번 눌러 봤다. 한 번은 vio 단계, 한 번은 보내는 중(작업 시작 전)이었다.
  두 번 모두 노트북 프로세스가 0 개가 되고 잠금이 풀렸으며 결과 폴더를 가져왔다.
- 폰 쪽(녹화 시작, HOME/force-stop, adb pull)은 **한 번도 실행하지 않았다** (카메라 금지 규칙).

코드: 저장소 StereoDepthSeg, 브랜치 `livetest-20261115`, `scan3d/live_test/` (live_server.py, live.html). 진행 기록은 `progress.txt` 에 있다.
