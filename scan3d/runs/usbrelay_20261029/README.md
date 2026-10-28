# usbrelay 2026-10-29 — 폰 앱 USB 중계 (ov_bridge VIOP <-> FC USB CDC)

## 0. 계획 (구현 전에 적음)
문제 (독립 검증 verify_arm_20261029 Re-verify): 앱의 USB 코드는 ViopImuSender (IMU bulk OUT) 하나뿐. ov_bridge 의
POSE / HEARTBEAT (UDP) 를 USB 로 FC 에 보내는 길도, FC -> 폰 (USB IN, 예: VIOP_T_VIO_RESET_REQ 0x23) 을 ov_bridge 로
넘기는 길도 없음 -> 실제 연결에서 FC 는 pose 를 못 받음.

할 일
1. fcarm d1b5779 (폰 쪽 SEEDED / F_RESET x50 / 0x23 처리) cherry-pick -> ov_bridge Android 바이너리 다시 빌드.
2. Java USB 중계 (양방향), USB 연결·끝점 주인은 하나 (ViopUsbLink) 를 ViopImuSender 와 같이 씀:
   - 쓰기 스레드 하나 + 큐 2개 (IMU 우선, 그다음 pose/heartbeat/기타). 한 프레임 = bulkTransfer 하나 (바이트 섞임 없음).
   - ov_bridge 는 지금처럼 UDP 로 보냄 (--viop-udp 127.0.0.1:<중계 포트>); 중계가 데이터그램 하나 = VIOP 프레임 하나를
     받아 USB OUT 큐로. 프레이밍은 viop_proto.h (A5 5A, ver 2, crc16 CCITT-FALSE) 그대로, 바이트 안 바꿈.
   - USB IN 읽기 스레드: 스트림에서 VIOP 프레임 찾기 (sync, len, crc) -> 0x23 은 ov_bridge UDP 소켓 (마지막 보낸 주소)
     으로 그대로 넘김 (ov_bridge 의 kReset 경로), 나머지는 종류별 개수만 기록 (FC 는 지금 USB 로 0x23 만 보냄).
   - 켜는 법: intent viop_udp usb (새 값). 기본 (intent 없음) = 명령줄·동작 그대로. USB 장치 없음 -> 중계는 받아서
     버리고 개수만 셈 (VIO 경로 영향 없음).
   - PC 시험: 가짜 USB 끝점으로 프레임 무결성, IMU 우선, IMU 지연 증가 (pose 트래픽 있음/없음), USB IN -> UDP, 장치 없음.
3. FC: F_RESET 50번 반복 구간에서 래치가 매 pose 마다 풀리는 문제 -> 새 W 하나에 한 번만 (fcarm 브랜치, PC 시험 추가).
4. APK 0.6.38 빌드 (설치 안 함) + SHA256SUMS, 이 README 에 시험한 것 (PC 만) / 보드+폰 시험 필요한 것 / 벤치 시험 순서.

규칙: gyro_klt=5 프로필 기본값 유지, NPU 의 BRIEF 대체 없음, 측정 안 한 것은 추정.
