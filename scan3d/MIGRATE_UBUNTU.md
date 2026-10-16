# 우분투로 옮기기 (요약)

대상 PC: **RTX 4060 노트북 (8 GB, sm_89) · 우분투 22.04**

1. `sudo ubuntu-drivers install` (드라이버 550 이상) → 재부팅 → `nvidia-smi` 확인. 노트북이면 `prime-select nvidia` (또는 on-demand) 확인
2. `sudo apt install git curl build-essential cmake libeigen3-dev libopencv-dev libboost-all-dev adb`
   USB 권한: `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="04e8", MODE="0666"' | sudo tee /etc/udev/rules.d/51-android.rules && sudo udevadm control --reload`
3. 코드: `git clone https://github.com/gyumingim/StereoDepthSeg ~/scanner/StereoDepthSeg` (먼저 Windows 쪽에서 커밋·푸시 필요)
4. `scan3d_migrate.tar.gz` 를 저장소 위치에서 풀기 (보정·앱 서명 키·부하 계수·설정 스크립트)
5. `bash setup_scanner_wsl.sh` (WSL 용이지만 우분투에서도 그대로. 단 torch 줄의 **cu128 → cu126** 으로 바꿔 실행 — 드라이버 550 대에서도 돎. 5080 이 아니라서 cu128 필수 아님) → ov_bridge 빌드 (scan3d/README)
6. 실행: `scan3d/run_scan.sh` 또는 serve_idle.sh → http://localhost:8795

4060 노트북 차이:
- 깊이(FFS)·YOLO 가 5080 보다 대략 2~4배 느림 (추정) → 부하 계산이 실측으로 다시 맞춰짐. `runs/load_coeffs.json` 은 5080 값이라 **지우고 시작** (웹 고급 → 계수 초기화도 됨)
- VRAM 8 GB: 640x480 FFS + YOLO11m-seg 는 들어갈 것으로 봄 (추정). 부족하면 YOLO 를 yolo11s-seg 로
- 전원 연결 상태에서 실행 (배터리면 GPU 클럭 낮음)

남은 일 (우분투에서):
- 네이티브 모드 점검: Windows 중계·PowerShell 경로를 타지 않는지 (기본 link=relay 를 adb 직접으로), /home/system 하드코딩
- 로그인 자동 시작: systemd --user 서비스 (바탕화면 아이콘 없음)
- **두 렌즈 짝 100 % 버림** (H.264 전환 뒤) — 원인 미확인, 첫 실행에서 렌즈별 timestamp 비교
- debug.keystore 는 꼭 같은 파일 (다르면 폰 앱 지우고 다시 설치)
