# 스마트폰 3D 스캔·매핑 선행 연구 조사 (phone-3d)

작성: 2026-10-16 · 목적: 논문 각도 검토 (선행 연구와 우리 시스템의 차이 정리)

**우리 시스템(비교 기준)**: 일반 안드로이드 폰(Galaxy S24) 후면 광각+초광각 2개 렌즈 동시 스트림(기준선 약 15.5 mm) → PC에서 Fast-FoundationStereo(학습 기반 파운데이션 스테레오) 깊이 + YOLO 인스턴스 분할 → OpenVINS 스테레오+IMU VIO 포즈 → 물체별로 구분한 복셀 3D 맵을 실시간 생성. 폰으로 녹화한 뒤 오프라인 복원도 지원.

표기 규칙
- "미확인" = 이번 조사에서 1차 출처(논문 본문, 공식 문서)로 확인하지 못한 것이다.
- 근거가 검색 결과 요약(초록, 스니펫)뿐인 항목에는 "(초록 기준)"이라고 적었다.
- 연도는 처음 공개된 해(학회 또는 arXiv)다.

---

## 1. 비교표

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| **Apple RoomPlan** | 2022 | iPhone/iPad Pro LiDAR + RGB + IMU | LiDAR | ARKit VIO | 구조화된 방 모델 `CapturedRoom`(벽·문·창·바닥·가구 상자), USDZ 내보내기 | 있음. 가구 카테고리가 고정됨, 실내 전용 | 예(온디바이스) | 비공개 SDK | https://developer.apple.com/documentation/roomplan · https://developer.apple.com/documentation/roomplan/capturedroom |
| **ARKit Scene Reconstruction** | 2020 | LiDAR + RGB + IMU | LiDAR | ARKit VIO | 삼각 메시 `ARMeshAnchor` | 면 단위 클래스 `ARMeshClassification`(바닥·천장·문·좌석 등). 인스턴스 구분 없음 | 예 | 비공개 SDK | https://developer.apple.com/documentation/arkit/arworldtrackingconfiguration/scenereconstruction · https://developer.apple.com/documentation/arkit/armeshclassification |
| **ARCore Depth API** (Depth from Motion) | 2020(API) / 2018(논문) | 단안 RGB + IMU(ToF는 있으면 사용) | 움직임 기반 다시점 깊이(단안). 매끈하게 보정된 깊이 | ARCore VIO | 프레임별 깊이 이미지(맵은 앱이 직접 만듦) | 없음 | 예. 논문은 CPU 1코어로 동작한다고 보고 | 비공개. 데모 DepthLab은 공개 | https://developers.google.com/ar/develop/depth · https://research.google/pubs/depth-from-motion-for-smartphone-ar/ |
| **ARCore Raw Depth API** | 2021 | 위와 같음 | 보정을 거치지 않은 깊이 + 픽셀별 신뢰도 | ARCore VIO | 깊이와 신뢰도 이미지. 공식 codelab이 점군 복원 예제를 제공 | 없음 | 예 | codelab 예제 공개 | https://codelabs.developers.google.com/codelabs/arcore-rawdepthapi · https://developers.googleblog.com/2021/05/unlock-use-cases-and-increase-developer-velocity-with-new-capabilities-in-arcore.html |
| **ARCore Scene Semantics** | 2023 | RGB | (깊이 기능 아님) | ARCore | 픽셀별 라벨 + 신뢰도 | 의미 분할 12종(sky, building, tree, road, person, vehicle 등). **실외 전용**, 인스턴스 구분 없음 | 예 | 비공개 | https://developers.google.com/ar/develop/scene-semantics · https://developers.google.com/ar/reference/c/group/ar-semantic-label |
| **DepthLab** (Du 외, UIST) | 2020 | ARCore 폰 | ARCore Depth | ARCore | 깊이 맵을 로컬/표면/조밀 세 단계로 활용 | 없음 | 예 | 공개 | https://research.google/pubs/depthlab-real-time-3d-interaction-with-depth-maps-for-mobile-augmented-reality/ · https://github.com/googlesamples/arcore-depth-lab |
| **Niantic Lightship ARDK / NSDK** (meshing, semantics) | 2021~ | RGB(+LiDAR가 있으면 사용) + IMU | 학습 기반 단안 깊이(LiDAR 우선 옵션) | AR Foundation(ARKit/ARCore) | 메시(ARMeshManager). 옛 버전은 반경 약 5 m 복셀 | 의미 채널(sky, ground, building 등, 픽셀이 여러 라벨을 가질 수 있음). 3.2부터 메시를 채널로 거르기(허용/차단) 가능. 인스턴스는 미확인 | 예 | SDK 공개(모델은 비공개) | https://lightship.dev/docs/ardk/features/semantics · https://lightship.dev/docs/ardk/how-to/ar/meshing/adding_meshing · https://lightship.dev/docs/ardk/release_notes/release3.2/ |
| **Polycam** | 2020~ | LiDAR 또는 RGB 사진 | LiDAR 모드, 또는 사진측량(온디바이스/웹) | ARKit / SfM | 메시, 점군 | 없음(미확인) | LiDAR 모드는 거의 실시간, 사진 모드는 후처리 | 비공개 | https://poly.cam (상세 파이프라인은 미확인) |
| **Scaniverse** (Niantic) | 2021~ | RGB(+LiDAR) + IMU | 사진측량 / LiDAR | ARKit/ARCore | 메시, 가우시안 스플랫(SPZ). 온디바이스 처리 | 없음 | 촬영 후 약 1~2분 온디바이스 처리 | 비공개 | https://nianticspatial.com/products/scaniverse · https://apps.apple.com/us/app/scaniverse-lidar-3d-scanner/id1541433223 |
| **Luma AI** (캡처 앱) | 2022~ | RGB 영상 | 학습 기반 복원(NeRF/스플랫 계열로 알려져 있으나 공식 확인은 미확인) | SfM(미확인) | 방사장(radiance field) / 메시 내보내기 | 없음 | 아니오(클라우드 후처리) | 비공개 | https://lumalabs.ai |
| **Pixel 듀얼픽셀 깊이** (Garg 외, ICCV) | 2019 | 단일 카메라 듀얼픽셀(PDAF) | 학습 기반 DP 깊이(아핀 모호성까지만 결정) | 해당 없음(단일 사진) | 깊이 이미지 | 없음 | 아니오(사진용) | 공개(google-research/dual_pixels) | https://arxiv.org/abs/1904.05822 · https://research.google/blog/learning-to-predict-depth-on-the-pixel-3-phones/ |
| **Du²Net** (Zhang 외, ECCV) | 2020 | Pixel 4 메인(DP) + 망원 **듀얼 카메라** | 듀얼 카메라 스테레오 + DP 융합 학습 | 해당 없음 | 깊이 이미지 | 없음 | 아니오 | 미확인 | https://arxiv.org/abs/2003.14299 · https://research.google/pubs/du2net-learning-depth-estimation-from-dual-cameras-and-dual-pixels/ |
| **Pixel 4 uDepth** | 2020 | 전면 IR 스테레오 2개 + 패턴 프로젝터 | 능동 IR 스테레오(30 Hz DEPTH16, Camera2 API) | 해당 없음 | 깊이 스트림 | 없음 | 예 | 비공개 | https://research.google/blog/udepth-real-time-3d-depth-sensing-on-the-pixel-4/ |
| **MobiDepth** (Zhang 외, MobiCom) | 2022 | 폰 **이종 듀얼 카메라**(초점거리가 서로 다름) | 시야를 잘라 초점거리를 맞춘 뒤 평행화 + 모바일 GPU 스테레오(학습 없음) | 해당 없음 | 깊이 이미지(응용: 3D 자세 추정) | 없음 | 예, 22 fps(온디바이스) | 미확인 | https://www.microsoft.com/en-us/research/uploads/prod/2022/09/mobicom22-final138.pdf |
| **HiMoDepth** (IEEE TMC) | 2023/2024 | 폰 이종 듀얼 카메라 | 시야 자르기 + 타임스탬프로 동기 안 맞는 프레임 버림 + 단계적(coarse-to-fine) GPU 스테레오, 1280×960 이상 | 해당 없음 | 깊이 이미지 | 없음 | 예(초록 기준) | 미확인 | https://doi.org/10.1109/TMC.2023.3294188 |
| **Smartphone Multi-lens Shape/SVBRDF** (CGF, Pacific Graphics) | 2023 | 폰 광각 + 줌 렌즈 동시 촬영 | 작은 기준선과 원근 차이에서 얻은 광류로 세밀한 깊이 | 해당 없음 | 깊이 + 반사 특성(SVBRDF) | 없음 | 아니오 | 미확인 | https://diglib.eg.org/bitstream/handle/10.1111/cgf14972/v42i7_44_14972.pdf |
| **Holopix50k** (데이터셋) | 2020 | 폰 스테레오(Leia 폰 사용자가 올린 사진) | 데이터셋(스테레오 쌍 49,368개) | 해당 없음 | 해당 없음 | 없음 | 해당 없음 | 데이터 공개 | https://arxiv.org/abs/2003.11172 |
| **MobileStereoNet** (WACV) | 2021 | 일반 스테레오(KITTI, SceneFlow) | 경량 학습 스테레오(MobileNet 블록, 2D/3D 비용 볼륨) | 해당 없음 | 시차 맵 | 없음 | 경량화 목표. 폰 실측은 미확인 | 공개 | https://arxiv.org/abs/2108.09770 · https://github.com/cogsys-tuebingen/mobilestereonet |
| **Mobile AR Depth Estimation: Challenges & Prospects** | 2023 | 폰(초광각 활용을 제안) | 모바일 깊이 추정 조사·분석 | 해당 없음 | 해당 없음 | 없음 | 해당 없음 | 미확인 | https://arxiv.org/abs/2310.14437 |
| **Live Metric 3D Reconstruction on Mobile Phones** (Tanskanen 외, ICCV) | 2013 | 단안 RGB + IMU | 다시점 스테레오(온디바이스) | 시각+관성(IMU로 미터 스케일 결정) | 점군 | 없음 | 예 | 미확인 | https://mlanthology.org/iccv/2013/tanskanen2013iccv-live |
| **MobileFusion** (Ondruska 외, ISMAR/TVCG) | 2015 | 단안 RGB | 키프레임 스테레오 | 모델 기반 조밀 추적 | TSDF 볼륨(KinectFusion식) | 없음 | 예, 25 Hz 온디바이스 | 미확인 | https://ori.ox.ac.uk/media/5802/2015ismar_ondruska.pdf · https://www.microsoft.com/en-us/research/?p=326108 |
| **Chisel** (Klingensmith 외, RSS) | 2015 | Google Tango(능동 깊이 + 어안 + IMU) | Tango 깊이 센서 | Tango VIO | 공간 해시 TSDF(2~3 cm) | 없음 | 예, GPU 없이 온디바이스 | 공개(OpenChisel, 링크 미확인) | https://roboticsproceedings.org/rss11/p40.html |
| **Mobile3DRecon** (Yang 외, ISMAR 최우수 논문 → TVCG) | 2020 | 단안 RGB + IMU | 키프레임 다시점 SGM + 깊이 다듬기 | VIO/SLAM(온디바이스) | 점진적으로 만드는 온라인 메시 | 없음 | 예(중급 폰 2종에서 평가) | 미확인 | https://cs.paperswithcode.com/paper/mobile3drecon-real-time-monocular-3d |
| **Mobile3DScanner** (IEEE) | 2021 | 폰 내장 RGB-D(ToF) + IMU | ToF | 시각-관성 ICP | TSDF(적응형 복셀 크기 조절) | 없음(물체 스캔용) | 예 | 미확인 | https://ieeexplore.ieee.org/document/9523840 |
| **SimpleMapping** | 2023 | 단안 + IMU(EuRoC 등 데이터셋) | 희소 점을 보조로 쓰는 학습 MVS | 특징점 기반 VIO | TSDF 전역 맵 | 없음 | 예(PC, 데이터셋 평가). 폰 실행은 안 함 | 미확인 | https://arxiv.org/abs/2306.08648 |
| **SemanticXR** | 2026 | XR 기기(ARKit, iPad Pro LiDAR 언급) | 기기 깊이(LiDAR) | ARKit | 물체 단위 의미 맵(개방 어휘) | **있음. 물체 단위 + 개방 어휘 질의** | 예(기기-클라우드 분산 처리) | 미확인 | https://arxiv.org/abs/2606.12849 |

참고(우리 구현 근거): 폰에서 렌즈 2개를 동시에 쓰는 방법은 Android Camera2 multi-camera API(논리 카메라 + 물리 카메라 스트림)다. 기기마다 동작이 달라 실제 기기로 시험해야 한다고 공식 문서에 적혀 있다. https://developer.android.com/media/camera/camera2/multi-camera

---

## 2. 범주별 요약

- **상용 SDK (RoomPlan, ARKit, ARCore, Lightship)**
  - 포즈는 모두 기기 자체 VIO에서 온다.
  - 깊이는 LiDAR(Apple) 또는 단안 깊이(움직임 기반, 학습 기반)를 쓴다. 후면 RGB 렌즈 두 개로 하는 스테레오 깊이를 공개 API로 제공하는 것은 확인하지 못했다(미확인).
  - 물체 구분 수준은 다음과 같다.
    - RoomPlan: 가구 카테고리가 고정된 상자 모델
    - ARKit: 메시 면 단위 클래스
    - ARCore: 실외 의미 분할 12종
    - Lightship: 의미 채널과 메시 필터
  - 넷 모두 **물체 인스턴스별로 나뉜 조밀 복셀 맵**을 직접 주지 않는다.
- **스캔 앱 (Polycam, Scaniverse, Luma)**
  - 목적이 시각적 결과물(메시, 스플랫, 방사장)이고 물체 구분이 없다.
  - 대부분 촬영 뒤에 처리한다. Scaniverse는 온디바이스에서 몇 분 안에 처리한다.
- **폰 다중 렌즈 깊이 (학술)**
  - Du²Net(메인+망원+DP)과 MobiDepth/HiMoDepth(이종 듀얼 카메라, 시야 자르기, 동기화)가 가장 직접적인 선행 연구다.
  - 다만 모두 **프레임 단위 깊이**에서 끝나고 VIO, 맵, 물체 구분은 없다.
  - 학습 스테레오는 경량 모델(MobileStereoNet) 수준이다. 파운데이션 스테레오 모델(제로샷 일반화)을 폰 렌즈 쌍에 적용한 사례는 이번 조사에서 찾지 못했다(미확인).
- **폰 VIO + 깊이 융합 (학술)**
  - Tanskanen(2013), MobileFusion(2015), Chisel(2015, Tango), Mobile3DRecon(2020), Mobile3DScanner(2021), SimpleMapping(2023)이 있다.
  - 깊이는 단안 MVS 또는 능동 센서에서 오고, 맵은 TSDF 또는 메시다. 물체 인스턴스 구분은 없다.
  - SemanticXR(2026)은 물체 단위 의미 맵을 기기와 클라우드에 나눠 처리한다. 우리의 "폰 → PC" 분산 구조와 닮았지만 LiDAR와 ARKit에 의존한다.

---

## 3. 우리와 가장 가까운 3개와 차이점

### (1) MobiDepth (MobiCom 2022) / HiMoDepth (IEEE TMC 2024)
- **같은 점**
  - 일반 폰의 **초점거리가 서로 다른 후면 듀얼 카메라**로 스테레오 깊이를 구한다.
  - 시야를 잘라 초점거리를 맞추고, 두 카메라의 프레임 동기 문제를 다룬다.
  - 이 두 문제는 우리의 광각+초광각 쌍에도 그대로 생긴다.
- **차이**
  - 깊이를 고전적(학습 없는) GPU 스테레오로 폰에서 계산한다. 우리는 PC에서 **학습 기반 파운데이션 스테레오**를 쓴다.
  - 결과가 프레임 단위 깊이에서 끝난다. 우리는 VIO 포즈, 시간 누적 복셀 맵, 인스턴스 분할을 더한다.
  - 오프라인 녹화 복원 경로가 없다.
- **논문에서 쓸 점**
  - 같은 하드웨어 조건(이종 렌즈, 짧은 기준선)에서 우리 깊이 정확도와 비교할 기준선(baseline)이 된다.
  - 코드 공개 여부는 미확인이다.

### (2) SimpleMapping (2023) (그리고 계보상 Mobile3DRecon, MobileFusion)
- **같은 점**
  - VIO 포즈 + 학습 기반 깊이 + 볼륨(TSDF) 융합으로 실시간 조밀 맵을 만든다. 우리 파이프라인(OpenVINS + 학습 깊이 + 복셀)과 구조가 거의 같다.
- **차이**
  - 단안 MVS 깊이를 쓴다. 우리는 **동시에 찍은 두 렌즈의 스테레오**를 쓰므로 정지 상태나 회전만 하는 동작에서도 깊이가 나온다.
  - 데이터셋(EuRoC)에서 평가했고 폰에서는 돌리지 않았다. 우리는 폰 실데이터를 쓴다.
  - **물체 구분이 없다.**
- **논문에서 쓸 점**
  - "VIO + 학습 깊이 융합"의 대표 비교 대상이다. 우리 기여는 스테레오 + 인스턴스, 그리고 폰 실기기라는 점으로 차별화한다.

### (3) SemanticXR (arXiv 2606.12849, 2026)
- **같은 점**
  - 휴대 기기로 찍고 무거운 계산은 외부(클라우드)로 넘겨 **물체 단위 의미 3D 맵**을 실시간으로 만든다. 우리의 "폰 스트림 → PC 처리 → 물체별 맵"과 목표가 가장 비슷하다.
- **차이**
  - 깊이와 포즈를 LiDAR와 ARKit에서 받는다. 우리는 LiDAR 없는 안드로이드 폰에서 **RGB 스테레오 깊이**와 **자체 VIO(OpenVINS)**를 쓴다.
  - 개방 어휘 질의에 초점을 둔다. 우리는 YOLO 인스턴스와 복셀을 쓴다.
  - 맵 표현의 자세한 내용과 코드 공개 여부는 미확인이다. 초록과 스니펫 기준으로 정리했다.
- **논문에서 쓸 점**
  - 최신 관련 연구로 반드시 인용한다. 우리 차별점은 LiDAR 없이 일반 폰 렌즈 쌍과 파운데이션 스테레오로 비슷한 기능을 낸다는 것이다.

---

## 4. 논문 각도 시사점 (추정)
- 조사 범위 안에서는 다음 다섯 가지를 **한 시스템으로 묶은 사례를 찾지 못했다**(미확인이며 더 넓은 검색이 필요하다).
  - 일반 폰의 후면 이종 렌즈 동시 스트림
  - 학습 기반 파운데이션 스테레오
  - 스테레오+IMU VIO
  - 인스턴스별 복셀 맵
  - 실시간과 오프라인 모드
- 약점이 될 만한 점
  - 기준선이 15.5 mm로 짧아 먼 거리에서 깊이 오차가 크게 늘어난다.
  - 광각과 초광각의 해상도·왜곡 차이가 크다.
  - 두 렌즈의 셔터 동기가 하드웨어로 보장되지 않는다. MobiDepth/HiMoDepth가 같은 문제를 다뤘다.
  - 실험에서 이 세 가지를 정량화해야 한다.
- 비교 실험 후보
  - 깊이: ARCore Raw Depth(같은 S24에서 가능한지는 기기 지원 목록 확인 필요, https://developers.google.com/ar/devices), MobiDepth식 고전 스테레오
  - 맵: SimpleMapping식 단안 MVS + TSDF
