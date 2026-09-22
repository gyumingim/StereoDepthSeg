# STATUS

최종 갱신: 2026-09-22

## Phone Stereo 실기기 구현·검증 완료 (2026-09-22)

- 최신 요청: 2개를 기본으로 먼저 구현하고 3개는 지원되면 시도. 사용자가 새 코드 작성·수정·실기기 실행 승인.
- 추가: Camera2 Android 앱(`com.camera.dualstream`), USB NV21 수신·timestamp pairing, PC 동시 표시, GPU YOLO seg/bbox. [실행 안내](phone_stereo/README.md), [수치 기록](phone_stereo/verification_20260922.json).
- 실기기: SM-S921N / Android 16, logical 0 내부 physical **5=광각, 2=초광각, 6=망원**. 이전 dumpsys byte 표시의 모호함을 실제 API 조회로 해소했다.
- 2개: 640×480·30 FPS 요청 + PC GUI + YOLO seg/bbox, 25초간 736/737프레임·735쌍, 약 30 FPS, `cuda:0` 확인.
- 3개도 성공: 640×480·15 FPS, 12초간 171/172/171프레임·171묶음. 기본 실행은 2개 유지.
- bbox 전용: `venv_ffs` 환경에서 20초·582쌍, GPU 추론 447회, 중앙값 11.26ms. 이 시간은 FFS를 포함하지 않는다.
- 테스트: 수신 패킷 손상/분할/EOF, 프레임 누락·재사용 방지, 잘못된 보정값 거부 등 7개 통과. Android APK 빌드·서명 검증 및 실제 설치 완료.
- FFS: `--calib-stereo` 연결 코드 추가. **폰 두 렌즈의 보정값이 없어 실제 폰 depth 추론은 아직 미검증.** 노트북/폰 보정 재사용을 막고, 입력 렌즈 ID·순서·해상도를 검사한다. 다음 작업은 폰 쌍 보정과 정류·거리·FFS 지연 검증.
- 관측 timestamp 차이 0ms는 HAL의 `APPROXIMATE` 등급을 정밀 동기화로 바꾸지 않는다. 장시간 안정성과 거리 정확도는 별도 검증 대상.

## 입력·연산 요구 확정 당시 기록 (2026-09-22, 구현 전)

- 사용자 요구: 스마트폰 후면 초광각·광각·망원 3개를 PC에 동시에 표시하고, PC에서 YOLO seg/bbox + Fast FoundationStereo 추론. 스마트폰 단독 추론이나 노트북 카메라 사용을 요구한 것이 아니다.
- 당시: [조사 문서 17절](doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md)에 구성·검증 순서를 저장했다. 이후 위 실기기 시험에서 2개·3개 동시 수신을 확인했다.
- 남은 작업: 선택한 폰 두 렌즈의 보정과 FFS 거리 추론 검증.

## 스마트폰 후면 두 카메라 웹검증 당시 기록 (2026-09-22, 구현 전)

- 완료: 삼성 공식 자료에서 S24의 후면+후면 Dual Recording과 렌즈별 MP4 두 개 저장 지원 확인. [조사 문서 16절](doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md#16-galaxy-s24-후면-두-카메라-웹-근거-재검증--2026-09-22)에 출처·제약·코드 수정 제안 저장.
- 확인: Camera2의 logical session/physical output과 현재 CameraX의 두 physical selector 경로 존재. 독립 동시 오픈 ID 목록만으로 후면 두 출력 불가 판정은 부정확하다.
- 당시 미검증이던 실제 동시 수신·해상도/FPS는 위 Camera2 실기기 시험으로 확인했다. 웹 조사 단계에서는 촬영·앱 설치·코드 변경을 수행하지 않았으며 후속 사용자 승인으로 구현했다.
- 남은 작업: 폰 두 렌즈의 보정·depth 검증. 노트북 카메라는 필수가 아니다.

## 추가 자료조사: YOLO + Fast FoundationStereo (2026-09-21)

- 완료: [seg/bbox 객체 depth 자료조사](doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md). 공식 논문·문서·실제 demo 소스 확인, 공통 depth 맵과 두 영역 집계 방식 설계.
- 검증: OpenCV Q 재투영으로 `fx=800px, B=0.1m, d=40px → Z=2m` 및 절반 해상도에서 거리 불변 확인.
- 심화 조사 완료: [자료조사 10~14절](doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md), [코드 검토·수정 우선순위](doc_DEPTH_CODE_REVIEW.md), [CPU 재현 자료](research/depth_review_20260921/reproduce.py).
- 확인된 결함: 비정류 epipolar 필터에서 정확한 매칭 탈락, seg mask 구멍 소실, 거리/선택 점군 불일치, 임시 스케일 오기록·반복 보정, 좌우 부호/수직 정류 처리, NaN JSON 저장 실패 등. 파일 해시와 수치 기록 포함.
- 현행 소스: `lib_rectify.py`, `lib_ffs.py`, `z_object_depth.py`와 seg·planar 처리가 검토 중 추가되어 읽고 재검증했다. 이 조사 작업은 애플리케이션 코드를 변경하지 않았다.
- 다음 작업: 검토 문서의 기하·객체 영역·스케일/실패 상태 수정안을 적용한 뒤 FFS 환경·GPU 추론 및 실제 거리·속도 검증. 이번 조사에서 FFS GPU 추론의 성공 여부는 판정하지 않았다.

## 전체 목표

카메라로 객체를 인식하고 삼각측량으로 그 **3D 좌표와 크기**를 확정한 뒤,
파이썬 3D 맵에 재배치해 보여준다. 카메라 1대 버전과 2대 버전을 모두 만든다.

## 세부 목표

- 1대(권장): 장면에 둔 체커보드로 두 촬영 위치의 상대 포즈를 직접 측정
- 1대(대안): 카메라를 좌→우로 평행이동했다고 가정 (자로 이동량 측정)
- 2대: 동시에 찍은 2장으로 삼각측량
- 두 경우가 **같은 코어 1벌**을 쓸 것 (다른 건 상대 포즈 (R,T) 공급자뿐)
- 객체 인식: 수동 ROI 로 파이프라인 검증 → YOLO 자동 검출
- 시각화: Open3D 3D 맵에 점군 + 박스 + 촬영 위치 배치

## 사용 기술

| 항목 | 선택 | 근거 |
|---|---|---|
| Python | 3.10.12 (venv) | 시스템 파이썬 |
| OpenCV | `opencv-python==4.13.0.92` | 드론 프로젝트와 동일 — 검증된 버전. 최신 5.0.0.93은 메이저 변경 리스크 회피 |
| Open3D | `0.20.0` | cp310 manylinux 휠 확인. 점군·OBB·프러스텀이 모두 네이티브 |
| numpy | 2.2.6 | 위 둘이 함께 끌어온 버전 |
| 특징점 | SIFT (opencv 본체 포함) | 스케일/회전 불변, ratio test |
| 객체 검출 | `ultralytics` 8.4.157 YOLO11 | 기본 `yolo11m.pt` — 정지 사진 2장 처리라 속도보다 정확도 우선 |
| 추론 장치 | CUDA (RTX 4060) | torch 2.14.0+cu130. 자동 선택으로 `cuda:0` 사용, CPU 대비 11.6배 |
| 박스 피팅 | `get_minimal_oriented_bounding_box` | 아래 "문제점" 2번 참조 |
| 배경 분리 | DBSCAN (`cluster_dbscan`) | 아래 "문제점" 3번 참조 |
| 캘리 타겟 | Galaxy S24 화면 (가로) | 프린터 없음. 내장캠은 자기 화면을 못 봄 |

## 했던 일

- [x] 환경 구성 (venv + 의존성) — import 및 실제 API 동작 확인
- [x] `lib_calib.py` — 체커보드 PNG 생성 + 캘리브레이션
  - 합성 검증: fx 오차 **0.027%**, 왜곡계수 복원, RMS 0.069px(주입 노이즈 0.05px)
  - 타겟 자가검증: 코너 90개 검출, 간격 정확히 135px, 기준막대 간격 정확히 2000px
- [x] `lib_cam.py` — 카메라 열기 + 라이브 루프 (캘리/본촬영 공용 1벌)
- [x] `lib_stereo.py` — 삼각측량 코어
  - 합성 검증: 치수 오차 **1.34% / 1.30%**, 중심 오차 4.8mm, Z 오차 0.5mm
  - `check_pure_translation` — 회전 검출 (yaw 0.5도→0.411, 2도→2.028,
    pitch 2도→2.032, roll 2도→2.026, 평행이동→0.031), 평면 퇴화 검출
- [x] `lib_viz.py` — Open3D 3D 맵 (프러스텀 간격 0.20m 정확 배치 확인)
- [x] `z_calibrate.py` — 타겟 생성 / 캘리 촬영 / 실행 / **2대 스테레오 캘리**
  - 2대 경로 합성 검증: 정확한 코너 투입 시 baseline 오차 **0.0000%**,
    회전 오차 0.00001deg, RMS 0.00002px
- [x] `z_capture.py` — 본 촬영 (mono/stereo) + 촬영 직후 평행이동 검사
- [x] `z_reconstruct.py` — 복원 + 게이트 출력 + 3D 맵 표시
- [x] **실물 캘리브레이션 완료 (2026-09-21)** — 41뷰, 기준막대 실측 121.5mm
  - 재투영 RMS **0.2326px** (게이트 1.0), 커버리지 **16/16**, 코너 검출 41/41
  - fx 794.31 / fy 794.70, cx 651.00 / cy 366.42, 화각 H 77.7 V 48.7 대각 85.5도
  - 사각형 8.2012mm — 실측 121.5mm 는 삼성 공식 ppi 418 예상값 121.53mm 와 0.03mm 차이
- [x] `pose_from_board()` — 체커보드로 상대 포즈 직접 측정 (아래 문제점 9)
  - 수식 검증: 정확한 코너 투입 시 T 오차 0.0001mm, R 오차 0.00002deg
  - 끝단 검증: 195.0x145.7mm (참값 200x150), 오차 2.51%/2.87%
- [x] `z_capture.py --board` + `z_reconstruct.py` mode=board 배선
- [x] `doc_HOW_TO_RUN.md` — 촬영 절차

## 하고 있는 일

**본 촬영 대기.** 캘리브레이션은 끝났다(`calib.json`). 다음은 실물 촬영이라
AI가 대신할 수 없다.

```bash
./venv/bin/python z_capture.py --board      # 폰을 물체 옆에 두고 2장
./venv/bin/python z_reconstruct.py          # 드래그로 대상 지정
```

## 할 일

- [x] ~~2단계 게이트 실측~~ — RMS 0.2326px, 커버리지 16/16 로 통과
- [ ] **4단계 게이트 실측** — 자로 잰 물체 치수 대비 오차 < 10%
- [x] ~~FFS 실행~~ — 스모크·합성 쌍 통과 (문제점 13/15)
- [ ] **FFS 실물 쌍** — 폰 고정 후 `z_capture.py --stereo` → `z_stereo_pose.py --provisional` → `z_object_depth.py --mode seg` (triton 경로면 scale 1.0 가능)
- [ ] **왜곡 모델 확정** — k3 포함/제외에 따라 fx 가 794.31 ↔ 810.47 (2.03%) 차이.
      교차검증은 k3 포함이 우세(검증 RMS 0.2251 vs 0.2370)하나 f(r)이 비단조라
      결론이 엇갈린다. 4단계 실측 대조로 판정한다.
- [x] ~~YOLO 검출기 추가~~ — `lib_detect.py`, `z_reconstruct.py --yolo`
- [~] 조밀 스테레오 — Fast-FoundationStereo 로 진행 중 (문제점 13)

## 문제점 및 해결방안

### 1. 폰 화면이 작아 캘리 품질이 떨어질 위험 — 부분 해결

S24(6.2")는 A4의 약 1/9 면적. 보드가 이미지에서 작게 잡히면 K 추정이 나빠진다.

- **해결:** 폰을 **가로로** 눕혀 화면 긴 변(2340px)을 보드 긴 변에 쓴다.
  보드 실물이 55.6x79.4mm → **132.0x57.7mm** 가 되고, 이미지 폭 30% 를 채우는
  거리가 **13cm → 31cm** 로 멀어져 고정초점 웹캠에서도 선명하게 찍을 수 있다.
- **남은 위험:** 그래도 A4보다 작다. 재투영 RMS 게이트 미달 가능. 실측 후 판단.

### 2. Open3D 기본 OBB가 크기를 부풀림 — 해결

`get_oriented_bounding_box()` 는 `BoundingVolume.cpp:232-284` 에서 **볼록껍질
꼭짓점만으로 PCA** 를 한다. 노이즈 있는 표면 점군에서는 그 꼭짓점이 극단값이라
축이 면내에서 틀어지고 extent 가 `W*cos(phi)+H*sin(phi)` 로 부풀어 오른다.

| 방법 (참값 300x200mm) | 결과 | 오차 |
|---|---|---|
| `get_oriented_bounding_box` | 302.7 x 212.6 | 0.89% / **6.32%** |
| `get_minimal_oriented_bounding_box` | **304.2 x 198.8** | 1.41% / **0.61%** |
| PCA(전체점) | 304.3 x 218.6 | 1.43% / 9.30% |

- **해결:** 최소부피 OBB 로 교체.
- **주의(직접 확인):** 완전 평면 입력에서 `robust=False` 는 RuntimeError,
  `robust=True` 는 예외 없이 **extent [0,0,0] 을 조용히 반환**한다.
  → `robust=True` 로 부르되 결과를 검사하고, 퇴화면 껍질 PCA 로 물러선 뒤
  `box["method"]` 에 기록한다.

### 3. ROI 안 배경이 박스를 통째로 망가뜨림 — 해결

사람이 손으로 그린 ROI 에는 배경이 딸려 들어온다. 통계적 이상치 제거는
배경이 덩어리로 들어오면 못 막는다.

| ROI 마진 (참값 300x200mm) | 수정 전 | 수정 후 |
|---|---|---|
| 0px | 280 x 181 | 280 x 181 |
| 8~50px | 287 x 186 | 287 x 186 |
| 90px | **2683 x 1291 (794% 오차)** | **287 x 186** |
| 140px | — | **287 x 186** |

- **해결:** 3D 공간에서 DBSCAN 으로 덩어리를 나눠 가장 큰 것만 남긴다.
  eps 는 최근접거리 중앙값의 3배로 자동 결정 — 거리·규모가 바뀌어도
  상수를 다시 맞출 필요가 없다.
- **남은 한계:** 물체보다 배경에 특징점이 훨씬 많으면 배경을 고를 수 있다.
  덩어리 목록을 출력에 남겨 확인 가능하게 했고, `--z-range` 로 좁힐 수 있다.

### 4. yaw 회전은 눈에도 dy 에도 안 보이는데 거리를 크게 틀리게 함 — 해결

yaw 는 시차에 거의 균일한 offset 만 더하고 세로 시차는 건드리지 않는다.
2도 yaw 에서 dy 중앙값은 0.014 → 0.078px 로 거의 그대로였지만, 시차에
`fx*tan(2도)=31px` 가 더해져 Z=1.50m 가 1.19m 로 **21% 틀어졌다.**

- **해결:** Essential 행렬 → `recoverPose` 로 **회전각을 직접** 잰다.
  세 축 모두 정확히 검출된다 (검증 수치는 "했던 일" 참조).
- **단, 평면 퇴화:** 장면이 평면 한 장에 가까우면 E 가 퇴화한다. 단일 평면
  합성 장면에서 참값 0도인데 14.2도가 나오거나 인라이어 0개가 나왔다(크래시도 발생).
  → 호모그래피가 전체 매칭의 몇 %를 설명하는지(`planar_ratio`)를 함께 내고,
  E 추정이 실패하면 `reliable=False` 로 명시한다. 단일 평면에서 0.996,
  다중 깊이에서 0.35 로 깨끗이 갈린다.

### 5. ppi 스펙 불확실성 + 갤러리 앱 스케일링 — 해결

S24 ppi 가 출처마다 다르고(삼성 418 / GSMArena 416 / 직접계산 415.7 → 0.5% 편차),
갤러리 앱이 이미지를 1:1 로 안 띄우면 오차가 10% 를 넘을 수도 있는데 알 방법이 없다.

- **해결:** PNG 에 정확히 2000px 간격의 기준막대 2개를 그려 넣고 자로 재게 한다.
  `화소 피치 = 실측 mm / 2000` → 두 불확실성이 **한 번의 측정으로 동시에** 제거된다.

### 6. 크기가 구조적으로 몇 % 과소평가됨 — 미해결(구조적 한계)

SIFT 특징점이 물체 경계까지 닿지 않아 점군이 물체보다 작다.
합성 검증에서 300x200mm → 287x186mm (4.3% / 6.9%).

- **완화:** ROI 를 물체보다 살짝 크게 그리면 마진 0px 대비 개선된다
  (6.6%/9.7% → 4.3%/6.9%). 그 이상은 마진을 키워도 더 나아지지 않는다.
- **근본 해결책:** 희소 매칭 대신 조밀 스테레오(StereoSGBM). 필요해지면 착수.

### 8. 라이브 프리뷰가 5fps로 느림 — 해결

`--capture` 의 HUD가 매 프레임마다 `findChessboardCornersSB` 를 풀 해상도 +
`CALIB_CB_EXHAUSTIVE` 로 돌렸다. CPU 353%, 프레임당 198.5ms.

| 조건 (1280x720, 코너 90개) | 시간 | fps |
|---|---|---|
| 1280x720 + EXHAUSTIVE | 198.5 ms | ~5 |
| 1280x720 EXHAUSTIVE 없음 | 168.0 ms | ~6 |
| **640x360 EXHAUSTIVE 없음** | **35.0 ms** | **~28** |

느린 주범은 EXHAUSTIVE(30ms 차이)가 아니라 **해상도**(133ms 차이)였다.

- **원인:** HUD 표시용 검출과 저장 판정용 검출에 같은 `find_corners()` 를
  그대로 재사용했다. 코드 2벌을 피하려다, 두 용도의 요구사항이 다른 것을 놓쳤다.
  HUD는 "보이나/안 보이나"만 알면 되고 저장 판정만 정확하면 된다.
- **해결:** `find_corners(..., fast=True)` 경로 추가. HUD만 절반 해상도로 돌린다.
  실측 155.9 → 31.6ms (4.9배). 코너 90개 그대로 검출되고 좌표 차이는
  중앙값 0.383px (표시용이라 무해).
- **정확도 영향 없음:** 최종 캘리브레이션은 저장된 PNG에서 풀 해상도 +
  EXHAUSTIVE 로 다시 검출한다. SPACE 저장 판정도 엄격 경로를 그대로 쓴다.

### 7. 앞뒤 두께는 원리적으로 관측 불가 — 명시로 대응

두 시점이 가까워 물체의 보이는 앞면만 복원된다. 박스의 가장 얇은 축은
"복원된 표면의 두께"이지 물체의 두께가 아니다.

- **대응:** 출력과 문서에 매번 명시한다. 믿을 수 있는 값은 **중심 위치, 폭, 높이**.

### 9. 1대 촬영 절차가 번거롭고 yaw 에 취약함 — 해결

`--mono` 는 자로 이동량 재기, 곧은 모서리 준비, 들지 않고 밀기, 힌지 고정,
회전 게이트 통과를 모두 요구한다. 그런데도 yaw 가 섞이면 무너진다.

- **상황 변화:** 프린터가 없어 처음엔 체커보드 방식을 배제했는데, 폰 화면
  타겟이 실물에서 완벽히 작동하는 것이 확인됐다(RMS 0.2326px). 전제가 바뀌었다.
- **해결:** `pose_from_board()` — 두 사진의 체커보드에 `solvePnP` 를 걸어
  각 촬영 위치의 절대 자세를 구하고 `R = R2 R1^T`, `T = t2 - R t1` 로 상대 포즈를
  **측정**한다. 가정이 없으므로 카메라를 손으로 아무렇게나 옮겨도 된다.
- **효과 (합성 검증, yaw 3도 섞인 상황):**

| 방식 | 복원 거리 (참값 0.800m) | 치수 (참값 200x150mm) | 오차 |
|---|---|---|---|
| **체커보드 포즈** | **0.794 m** | **195.0 x 145.7** | **2.5% / 2.9%** |
| 평행이동 가정 | 1.110 m | 279.2 x 203.6 | 39.6% / 35.7% |

- **보드 거리 영향 없음:** 35/50/65/80cm 에서 치수 오차 1.7~2.9% 로 평탄.
  80cm(사각형 8.1px)에서도 검출된다. 화면에 온전히 들어오기만 하면 된다.
- **남은 제약:** 보드가 두 사진에 모두 보여야 하고, 촬영 사이에 보드와 물체가
  움직이면 안 된다. 불가능하면 `--mono` 로 물러서고 회전 게이트를 통과해야 한다.
- `--mono` 는 그대로 남겨뒀다 — 체커보드를 장면에 둘 수 없는 경우가 있다.

### 10. `--board` 의 조용한 실패 모드 — 부분 해결

`pose_from_board` 는 보드가 고정돼 있다고 믿는다. 그 믿음이 깨지면 조용히
틀린 답이 나온다. 합성 검증(참값 200x150mm / 0.800m):

| 무엇이 움직였나 | 복원 거리 | 치수 | 오차 | 검사가 잡나 |
|---|---|---|---|---|
| 카메라만 (정상) | 0.794 m | 195.0 x 145.7 | 2.5% | — |
| 폰만 (카메라 고정) | 48.87 m | 12573 x 5966 | 6186% | **잡음** (배경이동 0.00px) |
| 둘 다 (직교 방향) | 0.952 m | 240.5 x 174.8 | 20.2% | **잡음** (방향차 6.68deg) |
| **둘 다 (같은 방향)** | 0.950 m | 233.7 x 174.4 | **16.8%** | **못 잡음** (방향차 1.07deg) |
| 폰 5mm 밀림 | 0.822 m | 202.2 x 151.0 | 1.1% | 영향 미미 |

- **해결 1:** `check_board_consistency()` — 보드를 뺀 배경 특징점으로 카메라
  이동을 독립 추정해 보드가 말하는 포즈와 대조. 게이트는 전부 위 관측값에서 정했다
  (배경이동 >= 2px, 회전차 < 2deg, 방향차 < 3deg, 배경 평면비 < 0.90).
- **해결 2:** `reconstruct(min_disp_px=1.0)` 와 `z_range` 상한 50 -> 20m.
  시차 0 인 점이 깊이 48.87m 로 상한 아래를 빠져나가고 있었다. 이제 케이스 2는
  쓰레기 값 대신 "복원 실패 (깊이통과 0)" 로 깨끗이 떨어진다.
- **미해결(원리적):** 폰이 카메라와 같은 방향으로 밀린 경우. 순수 스케일 오차이고
  스케일의 출처가 보드뿐이라 Essential 행렬로는 볼 수 없다.
  오차 ~= (폰 이동량 / baseline). 대비책은 폰 고정 + baseline 크게. 문서에 명시했다.

### 11. 실제 사진에서만 드러난 결함 3개 — 해결

합성 검증만으로는 안 걸리던 것들이다. 실물 촬영본으로 돌리자마자 나왔다.

1. **빈 입력 크래시.** 에피폴라 필터가 점을 전부 떨어내면 `nL` 이 (0,2) 가 되는데
   `cv2.triangulatePoints` 가 빈 행렬을 거부하며 예외를 던졌다. 합성 장면은 항상
   매칭이 있어서 이 경로를 한 번도 안 지나갔다.
   -> `triangulate()` 에 빈 입력 가드 추가. 이제 "복원 실패" 로 깨끗이 떨어진다.
2. **장치 보고가 거짓말.** GPU 로 돌려도 `last_device` 가 "cpu" 라고 했다.
   `net.device` 는 predict 가 장치를 바꿔도 갱신되지 않는다.
   -> 결과 텐서의 실제 장치(`res.boxes.data.device`)를 읽도록 수정.
   "GPU 쓰는 줄 알았는데 CPU" 를 막으려고 넣은 장치가 스스로 그 오탐을 내고 있었다.
3. **CUDA 불가 (`cuInit 999`).** 드라이버/libcuda 버전 일치, 권한 정상, 샌드박스
   무관인데도 실패했다. `/dev/nvidia-uvm` 을 `parsecd` 가 붙잡은 채 모듈 상태가 꼬인
   것이었다. `parsecd` 종료 후 `rmmod nvidia_uvm && modprobe nvidia_uvm` 으로 복구.
   -> YOLO 추론 109.1ms(CPU) -> 9.4ms(GPU), 11.6배.

### 12. 폰 카메라(scrcpy)가 "Camera disconnected" 로 불시에 끊김 — 해결

scrcpy 이슈 #4865/#5977 에 "삼성에서 끊긴다"고만 있고 원인은 미확정이었다.
폰의 카메라 서비스 이벤트 로그(`dumpsys media.camera`)로 기전을 직접 잡았다.

- **독립 CameraDevice 동시 오픈 조합은 `{0 1}`, `{0 3}`으로 보고됐다.** 이는
  논리 카메라 한 세션에서 두 physical stream을 받는 지원 여부와 다르다.
  2026-09-22 실기기(SM-S921N, Android 16) 재조회에서 후면 logical camera 0의
  `LOGICAL_MULTI_CAMERA`, physicalIds, `APPROXIMATE` sync를 확인했다.
  따라서 이전의 “초광각은 어떤 카메라와도 동시 사용 불가”라는 일반화는 정정한다.
  실제 두 물리 스트림 세션 성공 여부는 아직 미검증이며,
  [조사 문서 15절](doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md)을 참고한다.
- 삼성 시스템 서비스 두 개가 전면 카메라를 잠깐씩 연다:
    `com.samsung.android.smartface` (Smart Stay, 13초 주기, priority 990)
    `com.samsung.android.sead` = EnvironmentAdaptiveDisplay (priority 999).
  sead 는 `CAMERA_OPEN_CLOSE_LISTENER` 권한을 갖고 있어 **우리가 카메라 2를 여는
  순간 반응해** 전면 카메라 3을 연다. 가만히 두면 60초에 0회, 우리 open 직후에만
  초 단위로 일치해서 나타났다.
- `{2,3}` 은 허용 조합이 아니므로 HAL 이 우선순위 낮은 우리 shell 클라이언트를
  **EVICT** 한다. 로그에 그대로 찍힌다:
      CONNECT device 2 shell → CONNECT device 3 sead → EVICT device 2 shell
- **해결:** 두 기능의 설정 스위치를 끈다 (폰에 남는 변경은 이 둘뿐).
      adb shell settings put system intelligent_sleep_mode 0   # Smart Stay
      adb shell settings put system ead_enabled 0              # 환경 적응형 디스플레이
  `pm disable-user` 는 이미 떠 있는 권한 프로세스를 못 죽여서(같은 PID 가 계속 반응)
  효과가 없었다. 되돌리기는 각각 1 로.
- **검증:** 설정 OFF 후 카메라 2 가 90초 동안 유지, OpenCV 30.0 fps 수신, sead 이벤트 0건.

**실험 하네스 버그 (교훈):** `setsid nohup cmd &` 뒤의 `$!` 는 scrcpy 가 아니라
즉시 종료되는 setsid 부모 PID 였다. 그래서 "1초 만에 종료" 로 세 번 잘못 읽었다.
실제로는 클라이언트가 2분 51초 살아 있었고 그걸 죽인 건 다음 실험이었다.
→ 진짜 PID 는 `pgrep -f` 로 잡고, 판정은 폰 쪽 이벤트 로그로 교차 확인한다.
→ `pkill -f 패턴` 은 그 패턴을 담은 자기 명령줄까지 죽인다(exit 144). `[s]crcpy` 처럼
  괄호로 자기 매칭을 막고, 스크립트 본문에 패턴을 넣은 heredoc 과 같은 명령에서 쓰지 않는다.

### 13. 조밀 스테레오(Fast-FoundationStereo) + YOLO seg/bbox 경로 — 해결 (실물 쌍 검증만 남음)

목표: 희소 SIFT 경로의 한계(질감 필요, 경계 특징점 부족으로 크기 과소평가)를 조밀 시차로 넘는다.
사실 확인(소스·모델카드 직접 읽음):
- 저장소 NVlabs/Fast-FoundationStereo (CVPR 2026). 요구: python 3.12, torch 2.6.0+cu124, xformers.
  메인 venv(3.10, torch 2.14)와 양립 불가 → **별도 `venv_ffs`** 로 실행.
- 가중치: HF `nvidia/c-fast-foundationstereo` (`model_best_bp2_serialize.pth` 71MB, 비게이트).
  sha256 `7aee85948373da62b0503c2542507129a3e7cab9d97d10e6790d89512a7db214` (2026-09-21 다운로드),
  cfg.yaml `d45afe99…d6bc` (max_disp 416, mixed_precision). 체크포인트와 저장소 코드 버전을 함께 고정할 것.
  직렬화된 모델 객체라 `torch.load(weights_only=False)` + 저장소 `core/` import 필요.
- 라이선스: 코드 LICENSE.txt "non-commercially means for research purposes only";
  C 가중치는 NVIDIA Open Model Agreement. **본 프로젝트는 연구용(사용자 확인)** → 문제 없음.
- `run_demo.py` 원문대로: RGB float 0~255, InputPadder(32), autocast fp16,
  `forward(l, r, iters, test_mode=True, optimize_build_volume='pytorch1')`, unpad, clip(0).
  입력은 반드시 정류·왜곡제거된 쌍.

설계(GPT 조사 문서 doc_YOLO_FAST_FOUNDATIONSTEREO_RESEARCH.md 의 판단을 채택):
- YOLO 를 **정류된 왼쪽 영상**에서 실행 → 마스크/박스/시차가 같은 좌표계.
- 유효 화소 = finite(d) & d>0.5 & 우측 대응점이 roi2 안 & Z 범위 안.
- 모드 seg / bbox / both(seg 한 번 → 같은 인스턴스를 mask 와 box 로 각각 집계해 비교).
- 깊이 필드 구분: Z_rect(정류 광축) / center_cam1 / dist(유클리드).

새 파일: `lib_rectify.py`(정류·역투영), `lib_ffs.py`(추론 어댑터), `z_object_depth.py`(실행).

검증(합성, 정답 깊이맵):
- 정류: K 다른 두 카메라 + yaw 3°에서 대응점 |dy| RMS 0.38px, Q 재투영과 0.00mm 일치, 3D 오차 ≤1.8mm.
- **bbox 의 배경 과반 실패 재현**: 마진 90px 박스 → 중앙값 3.974m(배경). seg 가 기본이어야 하는 이유.
- 크기 추정 방법 교체 2회:
  1) 최소부피 OBB → 완전 평면 점군에서 퇴화(308x217 vs 300x200) → `planar_box`(SVD 평면 + minAreaRect).
  2) 껍질 기반은 깊이 노이즈에 비례해 부풂(노이즈 2%: 347mm) → **평면(수만 점으로 적합) + 마스크 윤곽 광선 교점**.
  결과(참값 300x200): 노이즈 1%+혼입 10% → 0.3%/0.1%, 노이즈 2% → 1.2%/0.5%, 노이즈 3%(스트레스) → 4.8%/1.2%.
  bbox 는 같은 조건에서 6.8% / 15.8% / 31.5% — 비교 기준선으로만 둔다.
- 배경 혼입 대비: 평면 적합 전 깊이 중앙값±5·MAD 절단(3% 혼입만으로 SVD 법선이 뒤집혔던 문제).
- **실제 모델 실행 (2026-09-21):** 정답을 아는 합성 쌍(실제 캘리 K, yaw 3°, baseline 150mm)에서
  `z_object_depth.py --mode both`:
    물체(1.501m) → seg 1.493 / bbox 1.494 (**오차 0.5~0.6%**), 배경벽(4.0m) → 4.01, scale 0.75/0.5 모두 동일.
    정류 검사 pass(|dy| 중앙값 0.23px). 스모크(저장소 demo 960x540): 323ms/8회, scale0.5·4회 68ms, VRAM 4.4GB.
- 아직 안 한 것: 실물 스테레오 쌍(폰 고정 필요) 검증.

### 14. GPT 코드 검토(doc_DEPTH_CODE_REVIEW.md) 12항목 처리 — 해결

검토의 재현 스크립트(research/depth_review_20260921/reproduce.py)를 수정 전 기준선으로 실행해
전부 사실임을 확인한 뒤 고쳤다. 고친 뒤에도 유지되는지 `z_selftest.py`(18항목)로 회귀 검사한다.

| ID | 판정 | 조치 |
|---|---|---|
| R01 | **사실** — 회전/다른 K 뷰에 dy·dx 필터 → 정확한 대응 100개 중 0~12개 통과 | `reconstruct()`: Sampson 에피폴라(E=[T]×R) + 양안 전방성 + 광선각(≥0.1°) 게이트로 교체. 셋 다 100/100 통과, 3D 오차 1e-9mm |
| R02 | **사실** — `masks.xy`(RETR_EXTERNAL) → 구멍 소실 | `retina_masks=True` + `masks.data`(원본 해상도, 설치본에서 확인) 직접 사용 |
| R03 | **사실** — 거리(필터 전)·치수(필터 후) 집합 불일치 | `fit_box`/`planar_box` 가 인덱스 반환, 모든 통계를 최종 집합에서. 조밀 경로는 깊이 2군집 분리 + 선택정책(seg: 큰 군집, bbox: 중앙 지지) + `ambiguous` 상태 |
| R04 | **사실** — provisional 을 measured 로 저장, rescale 중복 | `scale_status`(provisional/measured/reference_scaled) + `history` 저장, results.json 의 baseline 과 불일치 시 rescale 거부, 인라이어 <30 저장 거부 |
| R05 | **사실** — `abs(Tx)` 가 좌우 역순·수직 배치를 숨김 | 역순(Tx≥0)/수직이면 즉시 예외, 부호 보존(`tx_m`), d=inf → NaN |
| R06 | **사실** | 게이트를 `status`/`gates`/`warnings` 로 결과에 전파. 정류검사 `pass/fail/unavailable` |
| R07 | **사실** — NaN JSON, knn 1개 unpack | `_clean()` 으로 None 변환, knn 쌍 길이 검사 |
| R08 | **사실** | roi2 의 y 범위 검사 추가 |
| R09 | **사실** — 좌우 read 시각 불일치 | `lib_cam.read_pair`(grab×2→retrieve×2), 프리뷰·저장 같은 쌍, `grab_skew_ms` 기록 (z_capture/z_calibrate/z_view) |
| R10 | **사실** — size 인자 혼용 | `rectify_maps` 에서 size 제거, 두 캘리 해상도 일치 검사 |
| R11 | **사실** | results.json 항상 원자적 재작성(실패 객체 포함, meta 에 baseline·scale_status·촬영시각), stereo JSON 과 캘리 파일 K 일치 검사, 조밀 경로도 shots/meta.json 의 board/mono 포즈 지원 |
| R12 | 대부분 **사실** | lib_ffs scale 검증·실제 비율, 통일 스키마, `z_cam1_med_m` 필드, 과장 문구 수정, lib_plane dist 사용, `planarity`/`planar_ok`/`size_status`, requirements 고정, `--lr-check`(Aria 방식 좌우 일관성) |
| §4 | **사실**(upstream TRT 데모의 K/시차 해상도 혼용) | 우리 어댑터는 시차를 원해상도로 되돌린 뒤 실제 비율로 나눔 — 해당 없음. 기록만 |

**동의하지 않은 것:** 없음. 단 R12 의 "크기 모델을 항상 붙이지 말라"는 `planar_ok`가 거짓일 때만 치수를 None 으로
한다(노이즈와 비평면을 지표 하나로 못 가르므로 크게 비평면인 경우만 거른다 — 두께/짧은변 > 0.5).

**검토가 놓친 것(내가 발견):** 조밀 bbox 에서 배경이 과반이면 `planar_box` 의 깊이 사전절단이 **배경을 물체로**
골라 status=ok 로 냈다(선택비율 0.86). 깊이 2군집 분리 + 중앙 지지 선택으로 고쳤고 selftest 에 넣었다.

**수정 후 교차검증:** GPT 재현 스크립트의 예외 허용판 `reproduce_after.py` 로 전 항목 재실행 —
회전/다른 K/작은 시차 100/100 통과, 구멍 마스크 2800px 보존, 역순·수직 정류 즉시 예외, NaN JSON 저장 성공,
provisional 저장 상태 유지, 2회째 rescale 거부, ROI y 밖 0px, 최종집합 Z 2.000(원시 3.85와 분리).
정답 시차를 `lib_ffs.infer` 자리에 끼운 끝단 실행(정류→YOLO(정류 왼쪽)→seg/bbox 집계)도 통과.
그 과정에서 조정한 것 둘:
- 정류 검사 게이트: 전체 RMS(이상치 몇 개에 3.6px 로 튐, 중앙값은 0.23px) → **|dy| 중앙값 <1px 이고
  |dy|<2px 비율 ≥70%**. RMS 는 참고값으로만 저장.
- 평면성 지표: p2~p98 두께/짧은변 은 깊이 노이즈 꼬리에 부풀어 폰(짧은변 100mm)을 비평면으로 오판 →
  **강건 폭(2·1.4826·MAD)/짧은변 < 0.6**. 노이즈와 비평면을 완전히 가르진 못하므로 크게 비평면인 경우만 거른다.

### 15. FFS 실행 환경 함정 3개 — 해결

1. **Triton 컴파일 실패** `fatal error: Python.h` — deadsnakes python3.12 만 있고 `python3.12-dev` 가 없다.
   저장소 코드는 `try: import triton` 실패 시 triton=None 으로 동작하고 우리는 `pytorch1` 볼륨 경로를 쓰므로,
   `lib_ffs` 가 Python.h 부재를 감지하면 `sys.modules['triton']=None` + `torch._dynamo.config.disable=True`
   로 **순수 PyTorch 경로**로 자동 우회한다. 헤더가 생기면 자동으로 triton 경로.
   → 2026-09-22 새벽 PPA 복구 후 `python3.12-dev` 설치됨. 자동으로 `[triton]` 경로:
     스모크 960x540 8회 **195ms** (순수 PyTorch 323ms), scale0.5·4회 **43ms**, VRAM **1109MB** (4449MB).
     1280x720 원해상도도 OOM 없이 실행 (합성 쌍, 거리 오차 0.5~0.6% 동일). 첫 호출은 커널 컴파일로 2.5~4초.
2. **직렬화 args 에 `normalize` 없음** → forward 에서 ConfigAttributeError. 저장소 기본값 True
   (make_plugin_onnx.py:123, submodule.py:377) 로 채우고 cfg.yaml 키도 보충 (`load.filled_keys` 에 기록).
3. **1280x720 원해상도 OOM (8GB)** — 순수 PyTorch 경로는 6D 코스트 볼륨을 통째로 만든다. 960x540 은 4.4GB,
   1280x720 은 초과. `--scale 0.75/0.5` 로 실행 (시차는 원해상도 단위로 복원되어 결과 동일). triton 경로면 해소.
- 좌우 일관성 임계는 추론 해상도 px 로 정의 (scale 0.5 에서 원해상도 1.5px 그대로 쓰니 30% 탈락).

- 합성 정답 쌍 생성기를 저장소에 둔다: `z_make_synth_pair.py` (임시 폴더가 세션 종료로 지워져 검증 자산이 사라졌던 일 재발 방지).

## 파라미터 변경 이력

| 날짜 | 항목 | 이전 → 이후 | 이유 | 영향 파일 |
|---|---|---|---|---|
| 2026-09-21 | 캘리 타겟 방향 | 세로 1080x2340 → **가로 2340x1080** | 보드 실물 면적 1.7배, 촬영거리 13→31cm | `lib_calib.py`, `z_calibrate.py` |
| 2026-09-21 | `PATTERN` | (6,9) → **(15,6)** | 가로 전환에 맞춤, 코너 54 → 90개 | `lib_calib.py` |
| 2026-09-21 | `SQUARE_PX` | 130 → **135** | 가로 화면 2340px 에 16칸 + 여백 67px | `lib_calib.py` |
| 2026-09-21 | `REF_GAP_PX` | 1600 → **2000** | 긴 변을 쓰게 되어 측정 구간 확대(97→122mm), 상대 측정오차 감소 | `lib_calib.py` |
| 2026-09-21 | 박스 피팅 | 껍질 PCA OBB → **최소부피 OBB** | 위 문제점 2 | `lib_stereo.py` |
| 2026-09-21 | 점군 필터 | 통계 이상치만 → **+ DBSCAN 최대 덩어리** | 위 문제점 3 | `lib_stereo.py` |
| 2026-09-21 | 라이브 HUD 검출 | 1280x720+EXHAUSTIVE → **640x360, EXHAUSTIVE 없음** | 위 문제점 8 | `lib_calib.py`, `z_calibrate.py` |
| 2026-09-21 | 1대 기본 촬영 방식 | 평행이동 가정(`--mono`) → **체커보드 포즈(`--board`)** | 위 문제점 9 | `lib_stereo.py`, `z_capture.py`, `z_reconstruct.py` |
| 2026-09-21 | 사각형 실치수 | (미정) → **8.2012mm** (기준막대 실측 121.5mm) | 실물 캘리 확정 | `calib.json` |
| 2026-09-21 | `z_range` 상한 | 50 m → **20 m** | 위 문제점 10 | `lib_stereo.py` |
| 2026-09-21 | 시차 하한 | (없음) → **`min_disp_px=1.0`** | 위 문제점 10 | `lib_stereo.py` |
| 2026-09-21 | 폰 `intelligent_sleep_mode` | 1 → **0** (Smart Stay OFF) | 위 문제점 12 | 폰 설정 |
| 2026-09-21 | 폰 `ead_enabled` | 1 → **0** (환경 적응형 디스플레이 OFF) | 위 문제점 12 | 폰 설정 |
| 2026-09-21 | 폰 `stay_on_while_plugged_in` | 0 → **2** (USB 연결 중 화면 유지) | 화면 꺼짐 회수 예방 | 폰 설정 |
| 2026-09-21 | v4l2loopback | 수동 modprobe → **부팅 시 자동** (`exclusive_caps` 없이) | 재부팅 후 /dev/video9 소실 | `/etc/modprobe.d`, `/etc/modules-load.d` |
| 2026-09-21 | `reconstruct` 필터 | dy<2px·dx≥1px → **Sampson<2px·광선각≥0.1°·양안 Z>0** | 문제점 14 R01 | `lib_stereo.py` |
| 2026-09-21 | YOLO seg 마스크 | `masks.xy` fillPoly → **`retina_masks=True` + `masks.data`** | 문제점 14 R02 | `lib_detect.py` |
| 2026-09-21 | `planar_box` 평면성 게이트 | (없음) → **두께/짧은변 < 0.5** 아니면 치수 None | 문제점 14 R12 | `lib_stereo.py`, `z_object_depth.py` |
| 2026-09-21 | requirements.txt | 3개 → **ultralytics/torch/torchvision/numpy 고정** | 재현성 | `requirements.txt` |
