# 의미/물체 단위 실시간 3D 매핑 선행 연구 조사

작성: 2026-10-16 · 목적: 논문 각도 검토 (우리 시스템과의 차별점 정리)

**우리 시스템 (비교 기준)**
- 센서: 일반 안드로이드 폰(Galaxy S24) 후면 광각+초광각 2개 동시 스트림, 기준선 15.5 mm (서로 다른 화각·초점거리의 비대칭 스테레오) + 폰 IMU
- 깊이: Fast-FoundationStereo (학습 기반 파운데이션 스테레오, PC GPU)
- 분할: YOLO 인스턴스 분할 (2D 마스크 → 3D 투영)
- 포즈: OpenVINS 스테레오+IMU VIO
- 맵: 물체별 ID가 붙은 복셀 3D 맵, 실시간 (+ 폰 녹화 → 오프라인 복원)

> 주의: 아래 표의 내용은 논문 초록·공식 저장소 README 기준으로 정리했다. 세부 수치(FPS 등)는 하드웨어마다 다르므로 "실시간"은 논문 주장 기준이다. 확인하지 못한 항목은 **미확인**으로 적었다.

## 1. 비교표

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| SemanticFusion | 2017 (ICRA) | RGB-D | 깊이 센서 (Kinect류) | ElasticFusion (밀집 RGB-D SLAM) | 서펠(surfel) 맵 + 클래스 확률 | 클래스 단위만 (CNN 의미 분할 + 베이지안 융합), 인스턴스 없음 | 예 (약 25 Hz 주장) | 공개 (Bitbucket, Dyson Robotics Lab) | arXiv 1609.05130 https://arxiv.org/abs/1609.05130 · 코드 https://bitbucket.org/dysonroboticslab/semanticfusion |
| MaskFusion | 2018 (ISMAR) | RGB-D | 깊이 센서 | 자체 다중 물체 추적 SLAM (Co-Fusion 계열, 물체마다 별도 포즈) | 물체별 서펠 모델 | 인스턴스 (Mask R-CNN) + 동적 물체 추적 | 예 (Mask R-CNN은 비동기) | 공개 | arXiv 1804.09194 https://arxiv.org/abs/1804.09194 · https://github.com/martinruenz/maskfusion |
| Fusion++ | 2018 (3DV) | RGB-D | 깊이 센서 | 물체 단위 SLAM (물체 TSDF를 랜드마크로, 포즈 그래프 최적화) | 물체별 TSDF + 배경 TSDF | 인스턴스 (Mask R-CNN) | 준실시간 (약 4–8 Hz 수준, 미확인) | 공식 공개 코드 미확인 | arXiv 1808.08378 https://arxiv.org/abs/1808.08378 |
| PanopticFusion | 2019 (IROS) | RGB-D | 깊이 센서 | 외부 포즈 (데이터셋 GT/기존 SLAM 사용, 미확인) | 복셀 TSDF (voxblox 기반) + CRF 정제 → 메시 | 판옵틱 (stuff 클래스 + thing 인스턴스) | 예 (논문 주장) | 공식 코드 미확인 | arXiv 1903.01177 https://arxiv.org/abs/1903.01177 |
| Voxblox | 2017 (IROS) | RGB-D / 스테레오 / LiDAR (포인트클라우드 입력이면 무엇이든) | 외부 입력 | 외부 입력 (VIO/SLAM 결과를 TF로) | 복셀 해시 TSDF → ESDF, 메시 | 없음 (기하 전용) | 예 (CPU) | 공개 | arXiv 1611.03631 https://arxiv.org/abs/1611.03631 · https://github.com/ethz-asl/voxblox |
| Voxblox++ (Volumetric Instance-Aware Semantic Mapping) | 2019 (RA-L/IROS) | RGB-D | 깊이 센서 | 외부 입력 (실험은 RGB-D SLAM/GT, 미확인) | Voxblox TSDF 복셀 + 복셀별 세그먼트 라벨 | 인스턴스 + 클래스 (Mask R-CNN + 기하 분할), 미지 물체 발견 | 예 (논문 주장) | 공개 | arXiv 1903.00268 https://arxiv.org/abs/1903.00268 · https://github.com/ethz-asl/voxblox-plusplus |
| Kimera (Kimera-VIO + Kimera-Semantics) | 2020 (ICRA) | **스테레오 + IMU** (RGB-D도 가능) | 스테레오 매칭 (OpenCV SGBM 등 고전 밀집 스테레오) 또는 깊이 센서 | **Kimera-VIO (스테레오-관성 VIO)** + Kimera-RPGO 루프 클로저 | Voxblox 기반 TSDF 복셀 + 의미 라벨 → 메시 | 클래스 단위 (2D 의미 분할 → 베이지안 복셀 라벨), 인스턴스 없음 | 예 (CPU) | 공개 | arXiv 1910.02490 https://arxiv.org/abs/1910.02490 · https://github.com/MIT-SPARK/Kimera · https://github.com/MIT-SPARK/Kimera-Semantics |
| Kimera-Multi | 2021 (ICRA) / 2023 (T-RO) | 다중 로봇, 각 로봇 스테레오+IMU (또는 RGB-D) | 스테레오 매칭 / 깊이 센서 | 로봇별 Kimera-VIO + 분산 루프 클로저·분산 포즈 그래프 최적화 | 로봇별 의미 메시 → 전역 최적화 후 병합 | 클래스 단위 | 예 (온라인 분산) | 공개 | arXiv 2011.04087 https://arxiv.org/abs/2011.04087 · arXiv 2106.14386 https://arxiv.org/abs/2106.14386 · https://github.com/MIT-SPARK/Kimera-Multi |
| Hydra | 2022 (RSS), 2024 (IJRR 확장) | RGB-D 또는 스테레오 + IMU (VIO 출력) | 깊이 센서 / 스테레오 | 외부 VIO (Kimera-VIO 등) + 장면 그래프 기반 루프 클로저 | 국소 TSDF/ESDF → 메시 → **3D 장면 그래프** (물체·장소·방·건물 계층) | 물체 노드 (의미 라벨 클러스터링), 방 분할 | 예 (CPU 실시간) | 공개 | arXiv 2201.13360 https://arxiv.org/abs/2201.13360 · https://github.com/MIT-SPARK/Hydra |
| Panoptic Mapping (Panoptic Multi-TSDFs) | 2022 (ICRA) | RGB-D | 깊이 센서 | 외부 입력 (데이터셋 포즈 / SLAM) | **물체·배경별 서브맵 TSDF** (Multi-TSDF), 장기 변화 감지 | 판옵틱 (인스턴스 + stuff), 물체별 서브맵 | 예 (논문 주장) | 공개 | arXiv 2109.10165 https://arxiv.org/abs/2109.10165 · https://github.com/ethz-asl/panoptic_mapping |
| ConceptFusion | 2023 (RSS) | RGB-D (일부 실험 스테레오/LiDAR 언급, 미확인) | 깊이 센서 | 외부 SLAM (gradslam 기반 융합) | 포인트 맵 + 점별 CLIP 등 파운데이션 특징 (개방 어휘) | 개방 어휘 질의 (텍스트·이미지·음성 등), 명시적 인스턴스 ID 없음 | 아니오 (오프라인 위주) | 공개 | arXiv 2302.07241 https://arxiv.org/abs/2302.07241 · https://github.com/concept-fusion/concept-fusion |
| ConceptGraphs | 2024 (ICRA) | RGB-D | 깊이 센서 | 외부 포즈 (데이터셋 GT / SLAM) | 물체별 포인트클라우드 + **개방 어휘 3D 장면 그래프** (LLM으로 관계) | 인스턴스 (SAM/Grounded-SAM + CLIP), 개방 어휘 | 아니오 (오프라인, 일부 실시간 변형 언급) | 공개 | arXiv 2309.16650 https://arxiv.org/abs/2309.16650 · https://github.com/concept-graphs/concept-graphs |
| OpenScene | 2023 (CVPR) | 3D 포인트클라우드 + 다시점 RGB (재구성 완료된 장면) | 기존 3D 재구성 입력 | 기존 재구성 포즈 | 포인트별 CLIP 정렬 특징 (3D 네트워크 증류) | 개방 어휘 의미 (인스턴스 없음) | 아니오 (오프라인) | 공개 | arXiv 2211.15654 https://arxiv.org/abs/2211.15654 · https://github.com/pengsongyou/openscene |
| HOV-SG | 2024 (RSS) | RGB-D | 깊이 센서 | 외부 포즈 (데이터셋 GT/SLAM) | 계층 개방 어휘 3D 장면 그래프 (층·방·물체) | 물체 인스턴스 + 개방 어휘 | 아니오 (오프라인 구축) | 공개 | arXiv 2403.17846 https://arxiv.org/abs/2403.17846 · https://github.com/hovsg/HOV-SG |
| **추가: nvblox (NVIDIA Isaac ROS)** | 2023–2024 | RGB-D 또는 **스테레오 카메라** (+IMU) | 깊이 센서 또는 **학습 기반 스테레오 (ESS DNN 스테레오)** | **cuVSLAM (스테레오-관성 VSLAM)** 등 외부 | GPU TSDF/ESDF 복셀 → 메시 | 사람(동적) 분할 마스킹, 동적 물체 처리 (일반 다중 클래스 인스턴스는 미확인) | 예 (GPU, Jetson) | 공개 | arXiv 2311.00626 https://arxiv.org/abs/2311.00626 · https://github.com/nvidia-isaac/nvblox · https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox · ESS https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html |
| 추가: Khronos | 2024 (RSS) | RGB-D (+VIO) | 깊이 센서 | 외부 VIO + 루프 클로저 | 시공간 메트릭-의미 맵 (물체 단위 변화 추적) | 물체 인스턴스 + 시간 변화 | 예 (온라인) | 공개 | arXiv 2402.13817 https://arxiv.org/abs/2402.13817 · https://github.com/MIT-SPARK/Khronos |
| 추가: Clio | 2024 (RA-L) | RGB-D (+VIO) | 깊이 센서 | 외부 VIO (Hydra 파이프라인) | 과제 기반 3D 장면 그래프 (정보 병목으로 물체 단위 결정) | 개방 어휘, 과제 의존 물체 | 예 (온라인) | 공개 | arXiv 2404.13696 https://arxiv.org/abs/2404.13696 · https://github.com/MIT-SPARK/Clio |
| 추가: MID-Fusion | 2019 (ICRA) | RGB-D | 깊이 센서 | 물체 단위 다중 인스턴스 동적 SLAM | 물체별 옥트리 SDF | 인스턴스 (Mask R-CNN) + 동적 | 준실시간 (미확인) | 공식 코드 미확인 | arXiv 1812.07976 https://arxiv.org/abs/1812.07976 |

## 2. 관찰 (전체 흐름)
1. **센서 축**: 거의 모든 의미/물체 매핑 연구는 RGB-D(능동 깊이) 기준이다. 스테레오를 정면으로 다루는 것은 Kimera 계열(고전 SGBM)과 nvblox(ESS DNN 스테레오) 정도다. **휴대폰 다중 렌즈(비대칭 화각, 15.5 mm 짧은 기준선) 스테레오를 깊이원으로 쓰는 의미 매핑은 찾지 못했다 (미확인 — 별도 검색 필요).**
2. **깊이 축**: 파운데이션 스테레오(FoundationStereo 계열)를 실시간 의미 매핑의 깊이원으로 쓴 사례는 표 안에서 없음. nvblox+ESS가 "학습 스테레오 → TSDF"라는 점에서 가장 가깝다.
3. **포즈 축**: Kimera/Hydra/Khronos/Clio는 VIO(스테레오/RGB-D+IMU) 기반, 나머지 다수는 GT 포즈나 RGB-D SLAM을 가정한다. 오프라인 장면 그래프 계열(ConceptGraphs, HOV-SG, OpenScene)은 포즈를 주어진 것으로 본다.
4. **표현 축**: (a) 복셀 TSDF + 라벨 (Kimera-Semantics, Voxblox++, PanopticFusion), (b) 물체별 서브맵 (Fusion++, Panoptic Mapping, MaskFusion), (c) 장면 그래프 (Hydra, ConceptGraphs, HOV-SG, Clio). 우리 "물체 ID 복셀 맵"은 (a)와 (b) 사이 — Voxblox++에 가장 가깝다.
5. **분할 축**: 초기(2017–2019) Mask R-CNN → 최근 SAM/CLIP 개방 어휘. YOLO 인스턴스 분할(경량·고속)을 쓰는 실시간 맵핑은 논문 수준에선 드물다 (구현 사례는 많음, 미확인).

## 3. 우리와 가장 가까운 3개와 차이점

### (1) Kimera (Kimera-VIO + Kimera-Semantics) — 구조상 가장 가까움
- 같은 점: 스테레오+IMU 입력, 스테레오 깊이, VIO 포즈, 복셀 TSDF에 2D 분할 라벨을 투영·융합, 실시간.
- 차이점:
  - 깊이: Kimera는 고전 SGBM(대칭 스테레오 리그, 기준선 약 10 cm 이상 가정). 우리는 학습 기반 파운데이션 스테레오 + **15.5 mm 초단 기준선·비대칭 렌즈(광각/초광각)** → 고전 매칭이 거의 불가능한 조건을 학습 모델로 해결하는 것이 차별점.
  - 분할: Kimera-Semantics는 클래스 단위(인스턴스 없음). 우리는 YOLO 인스턴스 → 물체별 ID.
  - 하드웨어: 전용 스테레오 리그(RealSense/EuRoC 등) vs 일반 폰 한 대.
  - 포즈: Kimera-VIO vs OpenVINS (둘 다 스테레오-관성, 큰 차이 아님).

### (2) nvblox + ESS + cuVSLAM (NVIDIA Isaac ROS) — 깊이원이 가장 가까움
- 같은 점: 학습 기반 스테레오 깊이 → GPU 복셀 TSDF, 스테레오-관성 포즈, 실시간, 분할 마스크로 사람(동적) 분리.
- 차이점:
  - 분할: nvblox는 주로 사람/동적 물체 마스킹. 일반 클래스의 **물체별 인스턴스 맵**은 기본 기능이 아님(미확인).
  - 센서: 전용 스테레오 카메라(ZED/RealSense 등, 기준선 수 cm 이상) vs 폰 비대칭 렌즈.
  - 깊이 모델: ESS(전용 경량 모델) vs Fast-FoundationStereo(제로샷 일반화 지향).
  - 논문 관점: nvblox 논문은 GPU 매핑 효율이 주제, 의미/인스턴스는 부가.

### (3) Voxblox++ (및 후속 Panoptic Mapping) — 맵 표현이 가장 가까움
- 같은 점: 복셀 TSDF에 2D 인스턴스 마스크를 프레임 간 연관시켜 물체 ID를 누적, 실시간.
- 차이점:
  - 센서·깊이: RGB-D 능동 깊이 vs 우리는 폰 수동 스테레오 + 학습 깊이 (깊이 노이즈 특성이 다름 → 인스턴스 연관·융합 가중치 설계가 달라져야 함).
  - 포즈: 외부 포즈(데이터셋/SLAM) 가정 vs 우리는 VIO 통합.
  - 분할: Mask R-CNN(느림) + 기하 분할 vs YOLO(고속).
  - Panoptic Mapping은 물체별 서브맵·장기 변화까지 다룸 — 우리 맵 확장 방향의 참고.

## 4. 논문 각도 후보 (조사 기반 제안)
- "일반 스마트폰의 이종(광각/초광각) 초단 기준선 렌즈 쌍 + 파운데이션 스테레오로 RGB-D 없이 물체 단위 복셀 맵" — 표의 어떤 연구도 이 조합을 다루지 않음 (단, 폰 듀얼 카메라 스테레오 선행 연구는 별도 조사 필요, 미확인).
- 비교 실험 기준선 후보: Kimera-Semantics(SGBM 깊이) / nvblox / Voxblox++ 에 우리 깊이·포즈를 넣었을 때 vs RGB-D 기준.
- 실시간(PC 스트림)과 오프라인(녹화 복원) 이중 모드 비교도 표의 연구들에는 드묾.

## 5. 출처 목록
- SemanticFusion: https://arxiv.org/abs/1609.05130
- MaskFusion: https://arxiv.org/abs/1804.09194 , https://github.com/martinruenz/maskfusion
- Fusion++: https://arxiv.org/abs/1808.08378
- PanopticFusion: https://arxiv.org/abs/1903.01177
- Voxblox: https://arxiv.org/abs/1611.03631 , https://github.com/ethz-asl/voxblox
- Voxblox++: https://arxiv.org/abs/1903.00268 , https://github.com/ethz-asl/voxblox-plusplus
- Kimera: https://arxiv.org/abs/1910.02490 , https://github.com/MIT-SPARK/Kimera , https://github.com/MIT-SPARK/Kimera-Semantics , https://github.com/MIT-SPARK/Kimera-VIO
- Kimera-Multi: https://arxiv.org/abs/2011.04087 , https://arxiv.org/abs/2106.14386 , https://github.com/MIT-SPARK/Kimera-Multi
- Hydra: https://arxiv.org/abs/2201.13360 , https://github.com/MIT-SPARK/Hydra
- Panoptic Mapping: https://arxiv.org/abs/2109.10165 , https://github.com/ethz-asl/panoptic_mapping
- ConceptFusion: https://arxiv.org/abs/2302.07241 , https://github.com/concept-fusion/concept-fusion
- ConceptGraphs: https://arxiv.org/abs/2309.16650 , https://github.com/concept-graphs/concept-graphs
- OpenScene: https://arxiv.org/abs/2211.15654 , https://github.com/pengsongyou/openscene
- HOV-SG: https://arxiv.org/abs/2403.17846 , https://github.com/hovsg/HOV-SG
- nvblox: https://arxiv.org/abs/2311.00626 , https://github.com/nvidia-isaac/nvblox , https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
- Isaac ROS ESS (DNN 스테레오): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html
- Khronos: https://arxiv.org/abs/2402.13817 , https://github.com/MIT-SPARK/Khronos
- Clio: https://arxiv.org/abs/2404.13696 , https://github.com/MIT-SPARK/Clio
- MID-Fusion: https://arxiv.org/abs/1812.07976
- OpenVINS (우리 포즈): https://github.com/rpng/open_vins
- FoundationStereo (우리 깊이 계열): https://arxiv.org/abs/2501.09898 , https://github.com/NVlabs/FoundationStereo

> 검증 상태: arXiv 번호·저장소 주소는 작성자 지식 기준으로 적었고 이번 세션에서 웹으로 하나씩 다시 열어 보지는 않았다. 링크가 맞지 않으면 **미확인**으로 간주할 것.
