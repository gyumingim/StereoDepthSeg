# ovloop 2026-10-26 — 루프 닫기 (장소 재인식 + 포즈 그래프) 계획

브랜치 `ovloop-20261026` (integ2-20261026 16d3298 에서 시작; 0.6.27 gyro_klt 프로필 커밋은 이 tip 에 없음 — 통합 때 확인).
사용자 결정 (2026-10-26): VIO 앞단은 그대로 (30/60 fps, CPU). 루프 닫기는 **따로 도는 느린 스레드** (키프레임 1–5 Hz, 노는 코어).
GPU/NPU 는 v1 에 안 씀. **제어용 포즈는 절대 안 튐**: VIO/예측 포즈는 그대로, 보정은 따로 `T_map_odom` 으로 내보내고 천천히 적용.

## 1. 있는 것 확인 (08:30, 오프라인)
| 후보 | 상태 | 판단 |
|---|---|---|
| OpenVINS ov_secondary (VINS-Fusion loop_fusion 이식) | 이 PC 에 없음 (~/scanner/src/open_vins 에 디렉터리 없음). GPL-3, ROS1 의존 | 안 씀 (내려받기 금지 + ROS 의존). 구조(4-DoF 그래프, PnP 확인)만 참고 |
| VINS-Fusion loop_fusion + DBoW2 + brief_k10L6.bin | 없음 (find ~ /usr/local /opt) | 안 씀 |
| DBoW2/DBoW3 + ORBvoc.txt | 없음 | **직접 작은 어휘 트리 (k-means 계층) 를 우리 녹화로 학습** (평가 녹화 제외) |
| OpenCV ORB/FAST/BFMatcher/solvePnPRansac | PC 4.5.4, 폰 OpenCV SDK features2d+calib3d 이미 링크 (android/CMakeLists.txt) | 씀 |
| Ceres | PC ~/scanner/local, 폰 ceres-install (ov_init 이 이미 씀) | 4-DoF 포즈 그래프에 씀 |
| ROS humble | 있음 | 안 씀 (브리지에 ROS 없음) |

## 2. 설계 (ov_bridge, 플래그 `--loop 1`, 기본 0 = 지금과 완전히 같음)
- **메인(VIO) 스레드가 하는 일 = 키프레임 때 복사만**: 프레임 처리 후 (초기화·tracking 일 때) 키프레임 조건 (최소 간격, 이동/회전 문턱, 최대 5 Hz) 이면
  영상(960x540 회색) 복사 + IMU 포즈 + 현재 추적점 (2D, id) + 각 점의 과거 관측(uvs_norm, 시각) + clone 포즈 + SLAM 점 3D 를 큐에 넣음 (읽기만, 상태·추적기 안 바꿈).
  OpenVINS 소스는 안 고침 (`trackFEATS` 접근용 파생 클래스만).
- **루프 스레드** (별도): ① 추적점 삼각측량 (clone 포즈, 선형 + 시차·깊이·재투영 검사) → 키프레임 카메라 좌표 3D 점
  ② ORB 기술자 (FAST, 여러 층) + **중력 정렬 방향** (VIO 중력 방향을 영상에 투영한 각도) → BoW 벡터 (자체 어휘)
  ③ 후보: 최근 `loop_min_gap` (기본 10 s) 안 키프레임 제외, 점수 하한
  ④ 기하 확인: 현재 ORB ↔ 후보 3D 점 기술자 매칭 → PnP RANSAC → 인라이어 하한, 롤·피치가 VIO 중력과 일치, 보정 크기 상한
  ⑤ 시간 일관성: 연속 키프레임이 같은 곳과 일치하는 보정을 내야 채택 (틀린 루프 방지 — 틀린 루프는 없는 것보다 나쁨)
  ⑥ 4-DoF 포즈 그래프 (x,y,z,yaw; 롤·피치는 VIO 그대로) Ceres: 순차 에지 (이전 4 키프레임), 루프 에지 (Huber), 첫 키프레임 고정
  ⑦ 목표 `T_map_odom` = 최신 키프레임의 보정 포즈 × VIO 포즈⁻¹ (yaw + 이동).
- **출력** (`--loop 1` 일 때만 추가 필드/메시지, 기존 필드 그대로): 내보내는 `T_map_odom` 은 목표로 **속도 제한** (예 0.10 m/s, 3 °/s) 으로 천천히 따라감.
  pose 줄 `"map":{...}`, pred 줄 같은 필드, VIOP 새 형식 0x07 `VIOP_T_MAP_ODOM` (ESP32 link_task.c 는 모르는 형식 무시),
  MAVLink 은 DEBUG_VECT "MAPODOM" (ArduPilot 은 기록만). 제어용 POSE/ODOMETRY 는 그대로.
- VIO 리셋 (make_system) 때: v1 은 루프 DB·그래프를 새로 시작. 세션 간 재위치는 v2.
- 평가용 `--loop-sync 1` (PC 전용): 메인 스레드가 키프레임 처리 끝까지 기다림 → 결정적 결과 (기본은 비동기).

## 3. 평가 (같은 자)
(a) 동일성: `--loop 0` (기본) vs 기준 PC 빌드: harness 845 + raw80 2327, pose-precision 17 → IDENTICAL. `--loop 1` 에서도 VIO "p/q" 는 끈 것과 동일 (추가 필드만).
(b) 정확도: std60 손 루프 142148/142610/151646 (ov60eval L30/P30 조건) 시작↔끝 오차: VIO vs 지도 좌표. HS GT 녹화 ATE: VIO vs 지도 포즈.
(c) 틀린 루프: GT 있는 녹화에서 받아들인 루프 에지마다 GT 상대 포즈와 비교 (> 10 cm 또는 > 3° = 틀림). 목표 0.
(d) 비용: 키프레임당 루프 스레드 ms, 프레임당 VIO feed ms 켬/끔 (PC; 시간 남으면 폰 재생, 잠금 규칙대로).
(e) 독립 검증 에이전트 (읽기 전용): 동일성, 틀린 루프, 제어 포즈 안 튐.

## 3b. 사용자 결정 (10-26, 조정자 전달): 학습된 전역 기술자 BoQ / SALAD (DINOv2)
- CPU BoW v1 은 기준선·대체 경로로 유지. 루프 스레드에 **VPR 인터페이스**: 후보 찾기 = BoW 또는 전역 기술자 (코사인 유사도).
- v1 비교 방법 (PC GPU, 오프라인): 브리지 `--loop-dump DIR` 로 키프레임 영상 + 메타 저장 → Python 으로 BoQ / SALAD 기술자 계산 (공식 코드·가중치 내려받기, 출처·라이선스 아래 표) →
  브리지 `--loop-vpr FILE` (키프레임 t_ns → 기술자) 로 같은 키프레임·같은 기하 확인·같은 그래프로 다시 돌려 BoW 와 비교 (recall, 틀린 루프, 시작↔끝 오차).
- 기하 확인은 항상 필수 (PnP + 중력 + 일관성). LightGlue+SuperPoint/ALIKED 는 선택 (나중).
- 폰 경로는 나중: 모델 크기·지연, 내보내기 (ONNX/TFLite → NNAPI/Samsung ENN NPU 또는 OpenCL GPU) 를 기록만.
| 모델 | 출처 | 라이선스 | 상태 |
|---|---|---|---|
| BoQ (DINOv2) | (내려받을 때 채움) | | |
| SALAD (DINOv2) | (내려받을 때 채움) | | |

## 4. 순서
1 계획 → 2 기준 PC 빌드 → 3 어휘 학습 도구 + 브리지 루프 모듈 → 4 동일성 → 5 루프 녹화 평가 → 6 GT 틀린 루프 + ATE → 7 비용 → 8 검증 → 보고.
멀티데이 작업: 단계마다 커밋·push, 못 끝내면 RESUME 메모.
