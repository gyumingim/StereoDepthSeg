# 학습 기반(파운데이션) 깊이를 SLAM/매핑에 쓴 선행 연구

작성 2026-10-16. 논문 각도 검토용 조사.

**우리 시스템**: Galaxy S24 후면 광각+초광각 동시 스트림(기준선 15.5 mm) → PC에서 Fast-FoundationStereo 깊이 + YOLO 인스턴스 분할 → OpenVINS 스테레오+IMU VIO 포즈 → 물체별 복셀 3D 맵(실시간), 그리고 폰 녹화 후 오프라인 복원.

표기 규칙: 웹 검색 결과나 초록 수준에서만 확인한 내용은 "(초록 수준)", 확인하지 못한 내용은 "미확인"으로 적는다. 표의 "실시간" 칸에는 논문이 밝힌 수치를 적는다(장비는 저마다 다르다).

---

## 1. 깊이 모델 자체(구성요소)

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| FoundationStereo (NVIDIA) | 2025 (CVPR) | 정렬된 스테레오 쌍 | 학습 스테레오(제로샷, 합성 데이터 100만 쌍 이상) | 없음 | 시차·깊이·점군 | 없음 | 아니오. Isaac ROS 문서도 "실시간이 필요 없는 응용에 적합"하다고 씀 | O | https://arxiv.org/abs/2501.09898 , https://github.com/NVlabs/FoundationStereo , https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html |
| Fast-FoundationStereo (NVIDIA) | 2026 (CVPR, arXiv 2025-12) | 정렬된 스테레오 쌍 | FoundationStereo를 증류·NAS·가지치기한 모델. 의사 라벨 쌍 140만 개로 학습 | 없음 | 시차 | 없음 | 예. 저자 보고로 FoundationStereo보다 10배 이상 빠르고 실시간 프레임률 | O | https://arxiv.org/abs/2512.11130 , https://github.com/NVlabs/Fast-FoundationStereo , https://catalog.ngc.nvidia.com/orgs/nvidia/tao/models/fast-foundationstereo/- |
| RAFT-Stereo | 2021 (3DV) | 스테레오 | 학습 스테레오(반복 정제) | 없음 | 시차 | 없음 | 실시간 변형 있음 | O | https://arxiv.org/abs/2109.07547 , https://github.com/princeton-vl/RAFT-Stereo |
| CREStereo | 2022 (CVPR) | 스테레오 | 학습 스테레오(계단식 반복). 폰 촬영 같은 실제 사진을 겨냥 | 없음 | 시차 | 없음 | 미확인 | O | https://arxiv.org/abs/2203.11483 , https://github.com/megvii-research/CREStereo |
| IGEV-Stereo | 2023 (CVPR) | 스테레오 | 학습 스테레오(RAFT-Stereo 기반, 기하 인코딩 볼륨) | 없음 | 시차 | 없음 | 미확인 | O | https://arxiv.org/abs/2303.06615 , https://github.com/gangweiX/IGEV |
| Depth Anything V1/V2 | 2024 | 단안 | 단안 파운데이션(상대 깊이, 미세조정하면 절대 깊이) | 없음 | 깊이 | 없음 | 소형 모델은 빠름 | O | https://arxiv.org/abs/2401.10891 , https://arxiv.org/abs/2406.09414 |
| Metric3D v2 | 2024 | 단안 | 단안 절대 깊이와 법선 | 없음 | 깊이 | 없음 | 미확인 | O | https://arxiv.org/abs/2404.15506 , https://github.com/YvanYin/Metric3D |
| UniDepth | 2024 (CVPR) | 단안 | 단안 절대 깊이와 카메라 내부값 추정 | 없음 | 깊이 | 없음 | 미확인 | O | https://arxiv.org/abs/2403.18913 , https://github.com/lpiccinelli-eth/UniDepth |

## 2. 학습 깊이 또는 3D 사전지식으로 SLAM·매핑을 한 시스템

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| NVIDIA NuRec 스테레오 복원 흐름 | 2025 | 스테레오 RGB (ZED 2i, Hawk 권장) | FoundationStereo(TensorRT, 오프라인) | cuSFM(PyCuSFM). cuVSLAM 출력 폴더를 씀 | nvblox 복셀 → 메시·점유 지도 → 3DGRUT 가우시안 | 없음. 동적 모드 플래그만 있음 | 아니오. 단계별 배치 처리 | O | https://docs.nvidia.com/nurec/robotics/neural_reconstruction_stereo.html |
| NVIDIA Isaac ROS 스택 (FoundationStereo 또는 ESS + cuVSLAM + nvblox) | 2023~2026 | 스테레오(+IMU) | 학습 스테레오 DNN(ESS, FoundationStereo, Fast-FoundationStereo는 엔진 파일만 바꾸면 됨) | cuVSLAM 스테레오-관성 | nvblox GPU TSDF/ESDF 복셀 | 사람만: PeopleSemSegNet 마스크로 사람 깊이를 TSDF에서 떼어 점유 지도로 보냄. 일반 동적 물체 모드 있음 | 예. nvblox 논문은 GPU 가속을 보고함. 스택 전체 FPS는 미확인 | O | https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html , https://arxiv.org/abs/2311.00626 , https://github.com/nvidia-isaac/nvblox , https://arxiv.org/abs/2506.04359 , https://catalog.ngc.nvidia.com/orgs/nvidia/isaac/models/optimized-peoplesemseg-amr/- , https://developer.nvidia.com/blog/r2d2-building-ai-based-3d-robot-perception-and-mapping-with-nvidia-research/ |
| FoundationSLAM | 2025 (arXiv) | 단안 | FoundationStereo의 FeatureNet 인코더(고정)를 기하 사전지식으로 씀 | 자체 dense SLAM | 밀집 점/깊이 | 없음 | 18 FPS 보고 (초록 수준) | 미확인 | https://arxiv.org/abs/2512.25008 |
| MAC-VO | 2024~2025 (ICRA 2025) | 스테레오 | 학습 매칭 + 학습 불확실성(공분산) | 학습 기반 스테레오 VO. 포즈 그래프에 공분산 가중치 | 희소·준밀집 점 | 없음 | 미확인 | O | https://arxiv.org/abs/2409.09479 , https://mac-vo.github.io |
| DROID-SLAM | 2021 (NeurIPS) | 단안·스테레오·RGB-D | 반복 광류 + 밀집 BA로 얻은 깊이 | 학습 SLAM(DBA) | 키프레임 밀집 깊이 | 없음 | 거의 실시간(고사양 GPU 2개 기준) | O | https://arxiv.org/abs/2108.10869 , https://github.com/princeton-vl/DROID-SLAM |
| MASt3R-SLAM | 2025 (CVPR, arXiv 2024-12) | 단안 | MASt3R 3D 사전지식(점맵) | MASt3R 매칭 + 최적화 | 밀집 점맵 | 없음 | 15 FPS | O | https://arxiv.org/abs/2412.12392 , https://github.com/rmurai0610/MASt3R-SLAM |
| VGGT-SLAM / 2.0 | 2025 / 2026 | 단안 RGB | VGGT 피드포워드 점맵·깊이 | VGGT 직접 추정 + SL(4) 부분맵 정렬 | 부분맵 점군 | 없음 | 2.0은 실시간이라고 주장. GPU 메모리 제약 있음 | O | https://arxiv.org/abs/2505.12549 , https://arxiv.org/abs/2601.19887 , https://github.com/MIT-SPARK/VGGT-SLAM |
| SimpleMapping | 2023 (ISMAR) | 단안 + IMU | 학습 MVS(SPA-MVSNet). VIO 희소 점으로 안내 | ORB-SLAM3 VIO | TSDF | 없음 | 예 | O | https://arxiv.org/abs/2306.08648 |
| DAVIO | 2026 (arXiv) | 단안 + IMU | Depth Anything 3 (포즈 조건 깊이) | 관성 필터(피드포워드 초기화) | 밀집 맵. TSDF 여부 미확인 | 없음 | 미확인 (초록 수준) | 미확인 | https://arxiv.org/abs/2609.27702 |
| VIDAR | 2026 (arXiv) | 단안 + IMU | Depth Anything 3 | SVO + IMU를 미터 기준으로 씀 | 밀집 깊이 정렬. 맵 형식 미확인 | 없음 | 미확인 (초록 수준) | 미확인 | https://arxiv.org/abs/2607.17171 |
| VI-Depth (Wofk 외) | 2023 (ICRA) | 단안 + IMU | 단안 깊이 + VIO 희소 점으로 스케일 정렬 | VINS 계열 VIO | 깊이 지도(맵 아님) | 없음 | 미확인 | O | https://arxiv.org/abs/2303.12134 , https://github.com/isl-org/VI-Depth |
| VIMD | 2025 (arXiv) | 단안 + IMU | 아무 깊이 백본 + MSCKF로 픽셀별 스케일 보정 | MSCKF | 깊이 지도 | 없음 | 미확인 | 미확인 | https://arxiv.org/abs/2509.19713 |
| Prompt Depth Anything | 2025 (CVPR) | 폰 RGB + 저해상도 LiDAR(iPhone) | Depth Anything에 LiDAR를 프롬프트로 넣음 | ARKit 포즈 (미확인) | 응용 예시로 TSDF(복셀 0.04 m) | 없음 | 미확인 | O | https://arxiv.org/abs/2412.14015 |

## 3. 물체 구분 복셀 맵 (비교 기준선, 깊이는 고전 방식 또는 RGB-D)

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| Kimera | 2020 (ICRA) | 스테레오 + IMU | 고전 스테레오 블록 매칭(OpenCV) | Kimera-VIO | TSDF(Voxblox) → 의미 메시 | 의미 클래스만. 인스턴스는 구분하지 않음 | 예. CPU, 메셔 20 ms 미만 | O | https://arxiv.org/abs/1910.02490 , https://github.com/MIT-SPARK/Kimera |
| Voxblox++ | 2019 (RA-L) | RGB-D | 센서 깊이 | 외부에서 받은 포즈 | TSDF + 인스턴스 라벨 | 인스턴스(Mask R-CNN + 기하 분할) | 거의 실시간 | O | https://arxiv.org/abs/1903.00268 , https://github.com/ethz-asl/voxblox-plusplus |
| PanopticFusion | 2019 (IROS) | RGB-D | 센서 깊이 | 외부 포즈 | TSDF + 판옵틱 라벨 | 인스턴스 + 배경 클래스 | 예 | 미확인 | https://arxiv.org/abs/1903.01177 |
| Volumetric Semantically Consistent Panoptic Mapping | 2023 (arXiv, ETH) | RGB-D | 센서 깊이 | 외부 포즈 | Voxblox++ 확장 | 인스턴스(신뢰도 기반 융합 + 그래프 라벨 정제) | 미확인 | 미확인 | https://arxiv.org/abs/2309.14737 |
| PanopticNDT | 2023 (IROS) | RGB-D | 센서 깊이 | 외부 포즈 | NDT 셀 | 판옵틱 | 예(로봇 탑재) | 미확인 | https://arxiv.org/abs/2309.13635 |

## 4. 폰처럼 아주 짧은 기준선의 스테레오

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| Du²Net (Google) | 2020 (ECCV) | 폰 듀얼 카메라 + 듀얼 픽셀 | 학습 융합 스테레오 | 없음 | 깊이 지도 | 없음 | 미확인 | 미확인 | https://arxiv.org/abs/2003.14299 |
| LG G6 듀얼 카메라 스테레오 평가 | 2018 (ISPRS) | 폰 광각 + 초광각 | 고전 스테레오 | 없음 | 점군 | 없음 | 아니오 | 없음 | https://isprs-archives.copernicus.org/articles/XLII-1/289/2018/ |
| HyperSight | 2019 (SenSys) | 폰 듀얼 카메라 | 짧은 기준선으로 움직임을 추적해 긴 가상 기준선을 만듦 | 짧은 기준선 듀얼 카메라 추적 | 깊이(5 m에서 6 cm 오차) | 없음 | 미확인 | 미확인 | https://lion.sjtu.edu.cn/publication/publicationDetail?id=64 |
| 반사굴절(Catadioptric) 폰 스테레오 | 2021 (arXiv) | 폰 + 거울 어댑터 | 거울로 기준선을 넓힌 스테레오 | VO + 맵 점 | 맵 점 | 없음 | 미확인 | 미확인 | https://arxiv.org/abs/2109.11872 |
| DC² (Google) | 2023 (CVPR) | 폰 광각 + 초광각 | 포트레이트 모드 시차 + 광류 정렬 | 없음 | 깊이(초점 제어용) | 없음 | 미확인 | 미확인 | https://arxiv.org/abs/2304.03285 |

**찾지 못한 것**: 폰의 광각+초광각 쌍(기준선 2 cm 미만)에 **FoundationStereo 계열 학습 스테레오**를 쓰고 **VIO 포즈로 3D 맵**을 만든 논문은 이번 검색에서 나오지 않았다. 이 조합은 공백일 가능성이 있지만 아직 미확인이다. 아래 검색어로 추가 확인을 권한다. 웹 검색 기준으로는 FoundationStereo를 실시간 TSDF SLAM에 직접 쓴 공개 논문도 확인하지 못했다. 이 결합은 NVIDIA의 오프라인 NuRec 흐름과 Isaac ROS 문서에만 나온다.

---

## 5. 우리와 가장 가까운 3개와 차이점

### 1) NVIDIA Isaac ROS 스택 (FoundationStereo/ESS + cuVSLAM + nvblox, 사람 분할 모드)
- 같은 점: 학습 스테레오 깊이, 스테레오-관성 VIO 포즈, GPU 복셀 TSDF, 분할 마스크로 깊이를 나눠 맵에 넣는 구조. 구성이 우리와 거의 같다.
- 차이 ① 센서: 전용 스테레오 카메라(기준선 수 cm 이상, 같은 렌즈 두 개, 글로벌 셔터 가능)를 쓴다. 우리는 화각과 초점거리가 다른 폰 렌즈 두 개(15.5 mm, 롤링 셔터, 동기화 미보장)다.
- 차이 ② 분할: 사람만 떼어 동적 물체로 처리한다. 우리는 YOLO 인스턴스로 **물체별 ID 복셀 맵**을 만든다.
- 차이 ③ VIO: cuVSLAM(폐쇄 소스, GPU)을 쓴다. 우리는 OpenVINS(오픈 소스, 필터 방식)에 폰 IMU를 쓴다.
- 차이 ④ 오프라인 경로: NuRec은 오프라인 3DGS까지 가지만 폰 녹화는 다루지 않는다.

### 2) Kimera (스테레오 + IMU → TSDF 의미 메시)
- 같은 점: 스테레오+IMU VIO에서 실시간 복셀 TSDF를 만들고, 2D 분할을 3D에 투영한다.
- 차이: 깊이가 고전 블록 매칭이다(학습 스테레오 아님). 의미 클래스만 있고 인스턴스는 구분하지 않는다. 센서는 연구용 스테레오 카메라다. 우리 기여 각도는 "블록 매칭을 파운데이션 스테레오로 바꾸면 짧은 기준선에서도 맵이 나오는가"와 "의미 클래스를 넘어 인스턴스까지 구분하는가"가 된다.

### 3) SimpleMapping (단안 + IMU, VIO 포즈 + 학습 MVS → TSDF)
- 같은 점: VIO 포즈와 학습 깊이를 TSDF에 융합하는 실시간 파이프라인이고, 비싸지 않은 센서를 쓴다.
- 차이: 깊이가 시간축 다중 뷰 MVS여서 움직임(시차)이 필요하다. 우리는 한 순간의 스테레오로 미터 스케일을 바로 얻는다. 또 SimpleMapping에는 물체 구분이 없다.

(참고 후보) Voxblox++는 물체 구분 복셀 맵의 대표 기준선이지만 RGB-D 센서 깊이를 쓴다. FoundationSLAM은 FoundationStereo 인코더를 SLAM에 쓴 유일한 사례지만 단안이다.

## 6. 논문 각도 제안 (검토용, 추정)
- 공백 후보: "폰 광각+초광각(기준선 약 15 mm)에서 학습 파운데이션 스테레오 + 스테레오-관성 VIO + 인스턴스 복셀 맵". 위 표에서 이 셋을 함께 다룬 연구는 확인하지 못했다.
- 비교 실험 후보:
  - 깊이: 고전 SGBM, Kimera식 블록 매칭, Fast-FoundationStereo, 단안 파운데이션(Depth Anything V2 / Metric3D / UniDepth)에 VIO 스케일 정렬을 한 것
  - 포즈: OpenVINS 단안+IMU 대 스테레오+IMU
  - 맵 기준선: Voxblox++식 인스턴스 융합
- 짧은 기준선에서는 깊이 오차가 거리의 제곱으로 커진다. 학습 스테레오가 이 한계를 얼마나 늦추는지를 거리별 오차 곡선으로 보이는 실험이 핵심 기여가 될 수 있다(추정).
- 추가 검색어: "heterogeneous stereo wide ultra-wide learned", "smartphone multi-camera depth foundation", "rolling shutter unsynchronized stereo deep".
