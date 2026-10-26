# DPVO / DROID 계열 VIO, 느슨한 결합, 폰 실행 가능성 조사 (2026-10-26)

웹 조사만 했다. 코드 실행이나 측정은 하지 않았다. "추정"이라고 표시한 것은 확인되지 않은 추론이다.

## 결론 먼저
1. **지금 쓸 수 있는 1순위는 DBA-Fusion**이다. 코드와 가중치(DROID-SLAM TartanAir 가중치)가 공개돼 있고, GTSAM IMU 사전적분과 단안 VIO 예제(TUM-VI)가 있으며, 사용자 데이터 로더를 쓰는 방법도 안내한다. 다만 DROID 기반이라 무겁다(RTX 3090, 5 Hz 영상에서 실시간의 1.47배). 저장소는 2025-03 이후 멈췄고 GPL-3.0이다.
2. **2순위는 DEIO**(ICCV 2025, MIT)다. DPVO/DEVO와 DBA-Fusion을 합쳐 GTSAM 사전적분과 강결합했고 2026-07까지 커밋이 있다. 다만 **이벤트 카메라용**이고, 일반 영상만으로 돌아가는지는 README에서 확인되지 않았다.
3. 공식 DPVO 자체에는 IMU 기능이 없다. issue #80 "How do we integrate the IMU?"에 메인테이너 답이 없다.
4. 폰에서 DPVO를 그대로 실행하는 것은 현재 **현실성이 낮다**(추정). 공개된 모바일, Jetson, ONNX, TensorRT 포팅이 없고(issue #29가 열린 채 답 없음), 사용자 정의 CUDA 연산이 3개에 torch_scatter까지 필요하다.
5. 느슨한 결합은 검증된 고전 방식이 있다. EKF 상태에 시각 스케일 λ를 넣고 VO 자세를 측정값으로 쓰는 방식이다(Weiss & Siegwart 2011, ethzasl_msf / MaRS). 우리 OpenVINS에 붙이려면 사용자 정의 업데이트를 직접 짜야 한다(추정).

## A. DPVO/DROID 계열 시각-관성 변형

| 이름 | 저장소 / 라이선스 / 최근 활동 | 코드+가중치 | IMU 결합 방식 | 의존성 | 정확도 | 우리 데이터 적용 |
|---|---|---|---|---|---|---|
| **DBA-Fusion** (arXiv 2403.13714) | github.com/GREAT-WHU/DBA-Fusion, GPL-3.0, pushed 2025-03-11, ★223 | 있음. DROID-SLAM 가중치를 수정 없이 사용 | 슬라이딩 윈도 GTSAM 그래프(자세·속도·바이어스), Forster 사전적분, DBA 깊이를 Schur 소거한 자세 Hessian을 시각 인자로 사용, 주변화 사전, VINS-Mono식 초기화 | Py3.10, torch 1.11+cu113, torch-scatter 2.0.9, **수정된 GTSAM 포크**(yuxuanzhou97/gtsam) | TUM-VI 단안 평균 0.42 vs DM-VIO 0.85, VINS-Fusion mono 3.20, ORB3 mono 1.30(단위는 %로 표기). OpenVINS·EuRoC 비교 없음 | 사용자 로더(TUM-VI/KITTI-360 예제 기반)에 IMU, 내부 파라미터, R_ci를 넣는다. 시간 오프셋은 미리 보정해서 넣어야 함(추정). RTX 4090이면 오프라인 실행 가능 |
| **DEIO** (arXiv 2411.03928, ICCV 2025) | github.com/arclab-hku/DEIO, MIT, pushed 2026-07-07, ★119 | 코드 있음. 예제는 DEVO.pth 체크포인트 | 이벤트 패치 DBA Hessian과 GTSAM IMU 사전적분을 키프레임 슬라이딩 윈도에서 강결합 | CUDA 11.7, GTSAM python 빌드, numpy-quaternion | 이벤트 데이터셋 10종(EuRoC/TUM-VI 아님) | 영상 전용 모드는 **미확인**. 저장소 안에 `dpvo` 폴더가 있어 DPVO+IMU로 바꾸기 좋은 출발점(추정) |
| **iSLAM** (RA-L 2024) | github.com/sair-lab/iSLAM, 라이선스 Other, pushed 2025-11 | 코드 있음 | 스테레오 VO 네트워크 + IMU 네트워크 + 자세 그래프, 자기지도 양방향 학습 | PyPose 계열(추정) | KITTI·EuRoC·TartanAir. 스테레오 전제 | 단안 폰에는 맞지 않음 |
| **DVI-SLAM** (Samsung, arXiv 2309.13814) | 공개 코드를 찾지 못함 | 없음 | DROID식 미분 가능 BA에 광도·재투영·IMU 인자 | - | EuRoC 단안 ATE 45.3% 감소(자체 비교) | 사용 불가 |
| **Dual-Agent RL VIO** (arXiv 2511.21083) | 코드 링크 없음 | 없음 | DPVO식 VO와 IMU를 학습된 가중치로 섞는 **느슨한 결합**(B 참고) | - | EuRoC 평균 ATE 0.092 vs DPVO 0.106, VINS-Mono 0.187. 3090에서 39 FPS(DPVO 22) | 사용 불가. 아이디어만 참고 |
| **Adaptive VIO** (arXiv 2405.16754) | 코드 링크 없음 | 없음 | 대응점 네트워크 + IMU 바이어스 네트워크 + VI-BA | - | "학습 기반 VIO 중 최고, 최적화 기반과 비슷"(수치는 abs에 없음) | 사용 불가 |
| VIP-SLAM / DROID-VIO / DPV-VIO | 해당 이름의 DPVO 기반 VIO를 찾지 못함. VIP-SLAM은 2022년 RGB-D 평면 SLAM으로 무관 | - | - | - | - | - |

출처: https://github.com/GREAT-WHU/DBA-Fusion · https://arxiv.org/html/2403.13714 (Table I, 런타임, GTSAM 설명) · https://api.github.com/repos/GREAT-WHU/DBA-Fusion · https://arxiv.org/abs/2411.03928v3 · https://github.com/arclab-hku/DEIO · https://api.github.com/repos/arclab-hku/DEIO · https://arxiv.org/abs/2306.07894v1 · https://api.github.com/repos/sair-lab/iSLAM · https://arxiv.org/html/2309.13814v2 · https://arxiv.org/html/2511.21083v1 · https://arxiv.org/abs/2405.16754 · https://github.com/princeton-vl/DPVO/issues/80 · https://arxiv.org/abs/2207.01158v1 (VIP-SLAM)

DBA-Fusion 참고 사항
- TUM-VI 표의 DM-VIO 수치는 원 논문에서 가져온 값이다. DROID-SLAM은 GT로 스케일을 맞췄고, 루프 닫기는 껐다.
- 3090에서 단계별 시간은 특징 약 11 ms, 그래프·상관 25–32 ms, 광류 반복 1회당 약 20 ms다.

**순위 (우리 용도: 폰 단안 + 400 Hz IMU, ns 타임스탬프, R_ci와 시간 오프셋을 알고 있음)**
1. DBA-Fusion. PC 오프라인 기준선이나 GT 보조용으로 가장 빨리 돌려볼 수 있다. 수정 GTSAM 빌드가 필요하고, Windows보다 WSL/Linux가 맞다(추정).
2. DEIO 코드를 바탕으로 이벤트 대신 영상 DPVO 패치를 쓰는 변형. 손이 더 가지만 DPVO 기반이라 더 가볍다(추정).

## B. 느슨한 결합 (스케일 없는 학습 VO + IMU EKF)
- **Weiss & Siegwart, ICRA 2011.** 비전을 블랙박스로 두고 VO/SLAM 자세를 EKF 측정으로 쓴다. 상태에 시각 스케일과 드리프트를 넣어 온라인으로 추정한다(PTAM으로 실험). 구현체는 ethzasl_msf(Apache-2.0, 시간 지연 보상 EKF)이고, 후속 MaRS를 권장한다.
  https://www.research-collection.ethz.ch/bitstream/20.500.11850/43138/1/eth-7665-01.pdf · https://github.com/ethz-asl/ethzasl_msf
- **Nützi, Weiss, Scaramuzza, Siegwart 2011.** "Fusion of IMU and Vision for Absolute Scale Estimation in Monocular SLAM"은 다중 주기 EKF로 PTAM 스케일을 추정한다. https://doc.rero.ch/record/322372
- **arXiv 1707.07518.** 단안 VI 시스템의 느슨한 결합 메트릭 스케일 추정. https://arxiv.org/pdf/1707.07518
- **Remote Sensing 2022.** 학습 단안 VO와 학습 관성 오도메트리의 상대 자세를 EKF로 융합하고, IMU로 스케일을 준다(KITTI). https://doi.org/10.3390/rs14051228
- **gtsam_fusion (PaulKemppi).** iSAM2에서 IMU 사전적분과 6DOF 자세 입력을 융합한다. DPVO 출력을 자세 입력으로 넣는 것이 가능해 보인다(추정, 스케일 상태는 따로 추가해야 함). https://github.com/PaulKemppi/gtsam_fusion
- **Dual-Agent RL VIO (2511.21083).** 바이어스를 보정한 사전적분, MLP 속도·스케일 추정, 축별 볼록 결합. https://arxiv.org/html/2511.21083v1
- 우리 쪽 실무 제안(추정)
  - OpenVINS 전파는 그대로 두고, DPVO 키프레임 사이의 **상대 회전**과 **이동 방향 단위벡터**를 업데이트로 쓰면 스케일이 필요 없다.
  - 또는 상태에 log-scale λ를 넣고 상대 이동 전체를 업데이트로 쓴다.
  - 강결합(DBA-Fusion식)보다 정확도는 떨어진다. 이 비교는 https://www.roboticsproceedings.org/rss09/p37.pdf 의 논지를 근거로 했다.

## C. DPVO 폰 실행 가능성
- **구조** (https://arxiv.org/html/2208.04726)
  - 잔차 인코더 2개: 매칭용 D=128(instance norm), 문맥용 D=384.
  - 업데이트 연산자: 7×7 상관(2단 피라미드), 패치 궤적을 따라 1D 시간 conv, 그래프 간선에 softmax 집계, 게이트 잔차 유닛 2개, 흐름·신뢰도 헤드.
  - 미분 가능 BA: Gauss-Newton 2회.
  - 설정: 기본 96 패치/10프레임 윈도, Fast 48 패치/7프레임.
- **파라미터 수와 FLOPs**: DPVO 논문에는 없다.
  - 비교 기준으로 DROID-SLAM은 4.00 M 파라미터, 4.64 GFLOPs다(SPAQ-DL-SLAM, https://arxiv.org/html/2409.14515v1).
  - DPVO도 수 M 규모일 것이다(추정). 정확한 값은 체크포인트를 열어 세어야 한다.
- **속도와 메모리**
  - 논문: RTX 3090에서 기본 약 60 FPS(최소 48), Fast 약 120 FPS. GPU 메모리는 기본 4.9 GB, Fast 2.5 GB. EuRoC 평균 ATE는 기본 0.105, Fast 0.129 m.
  - 다른 논문들: DINO-VO 비교에서는 25 FPS/4.6 GB, Dual-Agent 논문에서는 3090 22 FPS로 나온다. 환경에 따라 차이가 크다.
- **임베디드 결과**: DPVO-QAT++(https://arxiv.org/html/2511.12653)가 유일하게 가까운 결과다.
  - 장비는 Jetson이 아닌 **RTX 4060 노트북 GPU**다.
  - EuRoC MH01에서 11.3에서 14.9 FPS로, 최대 메모리는 3.26에서 2.03 GB로 줄었다.
  - 인코더 Conv 22개만 INT8 모의 양자화했고, 업데이트 연산자와 BA는 FP32로 남겼다. Jetson 실험은 향후 과제로만 적혀 있다.
  - 노트북 GPU에서 약 11 FPS라는 것은 Python과 커널 실행 오버헤드가 크다는 뜻이다(추정).
- **Jetson/ONNX/TensorRT/모바일 포팅**: 찾지 못했다.
  - issue #29 "TensorRT export"(2023-04)는 성공 보고 없이 열려 있다.
  - issue #36은 Jetson AGX Orin에서 뷰어 설치 문제다.
  - https://github.com/princeton-vl/DPVO/issues/29 · https://github.com/princeton-vl/DPVO/issues/36
- **필요한 사용자 정의 연산** (setup.py, environment.yml: torch 2.3.1, CUDA 12.1, torch-scatter 2.1.2)
  - `cuda_corr`: altcorr, 패치 위치에서 쌍선형 샘플링하는 상관
  - `cuda_ba`: fastba, Schur 블록 BA
  - `lietorch_backends`: CPU 구현도 있음
  - torch_scatter: 간선 softmax 집계
  - https://raw.githubusercontent.com/princeton-vl/DPVO/main/setup.py · https://raw.githubusercontent.com/princeton-vl/DPVO/main/environment.yml
- **Exynos 2400 포팅 분할안** (모두 추정)
  - 인코더 2개(순수 Conv, 이미지당 1회)는 ONNX/TFLite를 거쳐 NPU로 보낸다. Exynos AI Studio가 ONNX와 TFLite를 받는다(https://semiconductor.samsung.com/news-events/tech-blog/unpacking-samsungs-comprehensive-on-device-ai-sdk-toolchain-strategy). 사용자 정의 연산 지원 문서는 찾지 못했다.
  - 상관(gather/쌍선형)과 scatter-softmax는 NPU에 맞지 않는다. Vulkan/OpenCL 컴퓨트 셰이더나 CPU NEON으로 다시 짜야 한다.
  - 업데이트 연산자의 MLP/1D conv/LayerNorm은 NPU나 GPU에 올릴 수 있지만, 간선 수가 동적이라 고정 크기 패딩이 필요하다.
  - BA는 96×10 패치에 자세 10개 정도라 작다. CPU Eigen으로 충분하다.
  - 다른 벤더 NPU는 사용자 정의 연산을 지원하지 않고, 지원 안 되는 연산이 있으면 실패하거나 CPU로 넘긴다(MediaTek, ST 사례: https://genio-community.mediatek.com/t/how-to-resolve-inference-failures-due-to-unsupported-operations-on-npu/449 , https://wiki.st.com/stm32mpu/wiki/STM32MP2_NPU_description).
  - 작업량: 사람 손으로 수 주 이상(추정).
- **경량 변형**
  - "DPVO-lite", "MobileDPVO", "Light-DPVO"는 찾지 못했다.
  - XFeat: i5-1135G7 CPU의 VGA에서 sparse 27 FPS. 제약이 큰 임베디드 장비에서는 1.8 FPS(최적화 안 한 PyTorch). ORB-SLAM3에 XFeat을 붙인 포트(xfeatSLAM)가 있다. 폰 수치는 없다. https://arxiv.org/html/2404.19174v1 · https://gitee.com/caolonghao/xfeatSLAM
  - 360DVO 논문도 실시간 임베디드를 하려면 더 가벼운 특징 추출기가 필요하다고 말한다. https://arxiv.org/pdf/2601.02309
- **판단** (추정)
  - DPVO는 PC나 노트북 GPU에서 오프라인 비교, 또는 GT 보조로 쓴다.
  - 폰 안에서 실시간으로 돌리려면 위 분할 포팅이 필요하다. 효과가 증명되기 전까지는 OpenVINS 전처리를 개선하는 편이(예: 학습 특징을 NPU에서) 현실적이다.
