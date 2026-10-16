# 우리 구성과 비슷한 오픈소스 프로젝트 조사

조사일: 2026-10-16. GitHub 별 수, 마지막 push, 라이선스는 이날 GitHub REST API(`api.github.com/repos/...`)로 직접 확인했다.
"미확인" = 이번 조사에서 1차 출처로 확인하지 못한 항목.

**우리 구성 (비교 기준)**: Galaxy S24 후면 광각 + 초광각 렌즈 2개를 동시에 스트림(기준선 15.5 mm, 초점거리 서로 다름, 하드웨어 셔터 동기 여부 미확인) → PC에서 Fast-FoundationStereo 깊이 + YOLO 인스턴스 분할 → OpenVINS 스테레오+IMU VIO 포즈 → 물체별로 나뉜 복셀 3D 맵(실시간), 폰 녹화 후 오프라인 복원도 지원.

## 1. 결론 먼저

- 이번 검색(GitHub 검색, 웹 검색)에서는 **"폰 렌즈 2개 스테레오 + 학습 기반 스테레오 깊이 + VIO + 인스턴스 복셀 맵"을 모두 갖춘 공개 프로젝트를 찾지 못했다.** GitHub 검색 `openvins semantic`, `openvins dense`, `yolo voxel map`, `smartphone stereo depth`는 결과가 0건이거나 관련 없는 저장소만 나왔다.
- 부품별로 보면 이미 성숙한 것이 있다. 로봇용 스테레오 리그 쪽은 Kimera, nvblox, 인스턴스 TSDF 쪽은 Voxblox++, panoptic_mapping, 폰 녹화 쪽은 OpenCamera-Sensors, MARS logger.
- 그래서 논문으로 내세울 새로운 점은 "① 폰의 서로 다른 초점거리 렌즈 쌍(기준선 15.5 mm)을 계측용 스테레오로 쓰는 것, ② 여기에 파운데이션 스테레오 모델을 붙인 것, ③ VIO + 인스턴스 복셀 맵까지 잇는 것"의 조합이다. 다만 이것이 정말 처음인지는 **미확인**이다. 검색에서 빠진 것이 있을 수 있다.

## 2. 비교표

| 이름 | 연도 | 센서 | 깊이 출처 | 포즈 출처 | 맵 표현 | 물체 구분 | 실시간 | 코드 (별 / 최근 push / 라이선스) | 출처 |
|---|---|---|---|---|---|---|---|---|---|
| **Kimera** (Kimera-VIO + Kimera-Semantics) | 2020 (ICRA) | 스테레오 카메라 + IMU (RGB-D도 가능) | 고전 스테레오 정합(저장소 문서: OpenCV StereoBM, 논문 요약: SGM) 또는 RGB-D | Kimera-VIO (스테레오+IMU, GTSAM) | Voxblox TSDF 기반 의미 메시, 복셀마다 클래스 확률을 베이즈 갱신 | 의미(클래스) 단위. 인스턴스 아님 | 예 (CPU) | VIO 1926 / 2026-08 / BSD-2; Semantics 754 / 2023-12 / BSD-2 | arXiv [1910.02490](https://arxiv.org/abs/1910.02490), [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO), [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) |
| **Hydra** | 2022 (RSS)~ | RGB-D 또는 스테레오 + IMU (Kimera-VIO 포즈 사용) | 입력 깊이 영상 | 외부 VIO (Kimera-VIO 등) | 3D 장면 그래프(메시 + 물체 + 장소 + 방) | 물체 노드 있음 (의미 분할 영상에서 군집) | 예 | 1180 / 2026-10 / BSD-2 | arXiv [2201.13360](https://arxiv.org/abs/2201.13360), [GitHub](https://github.com/MIT-SPARK/Hydra) |
| **Khronos** | 2024 (RSS) | RGB-D (+ 외부 포즈) | 입력 깊이 영상 | 외부 VIO/SLAM | 시공간 의미 맵, 물체 단위 조각 맵 | 물체 인스턴스 + 변화 감지 | 예 | 546 / 2026-10 / BSD-3 | arXiv [2402.13817](https://arxiv.org/abs/2402.13817), [GitHub](https://github.com/MIT-SPARK/Khronos) |
| **Clio** | 2024 (RA-L) | RGB-D + 외부 포즈 | 입력 깊이 영상 | 외부 VIO | 작업에 맞춘 물체 장면 그래프 (Hydra 기반) | 개방 어휘 물체 | 예 | 263 / 2025-09 / BSD-2 | arXiv [2404.13696](https://arxiv.org/abs/2404.13696), [GitHub](https://github.com/MIT-SPARK/Clio) |
| **nvblox / Isaac ROS nvblox** | 2023~ | RGB-D 또는 스테레오 카메라(RealSense, ZED 등) | 입력 깊이. Isaac ROS DNN Stereo Depth(학습 기반)와 조합 가능 | Isaac ROS Visual SLAM (cuVSLAM, 스테레오+IMU) 또는 다른 노드 | GPU TSDF + ESDF 복셀 | 사람 분할(PeopleSemSegNet) 및 동적 물체 모드만. 일반 물체 인스턴스 아님 | 예 (GPU) | nvblox 1323 / 2026-10 / 커스텀(LICENSE.md, GitHub에서 NOASSERTION); isaac_ros_nvblox 761 / 2026-09 / Apache-2.0 | arXiv [2311.00626](https://arxiv.org/abs/2311.00626), [nvblox](https://github.com/nvidia-isaac/nvblox), [isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox), [Isaac ROS nvblox 문서](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html) |
| **Isaac ROS DNN Stereo Depth** | 2023~ | 스테레오 카메라 | 학습 기반 스테레오 (ESS). FoundationStereo 포함 여부는 미확인 | (해당 없음, 깊이만) | 깊이/시차 영상 | 없음 | 예 (GPU) | 159 / 2026-09 / Apache-2.0 | [GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth) |
| **DynSLAM** | 2018 (ICRA) | 차량 스테레오 카메라 (KITTI) | 스테레오 정합(ELAS) 또는 학습 기반(DispNet) | 스테레오 시각 오도메트리(libviso2), IMU 없음 | InfiniTAM 복셀 TSDF. 정적 배경과 차량 인스턴스를 따로 복원 | 인스턴스 분할(Multi-task Network Cascades)로 차량마다 별도 복셀 맵 | 거의 실시간 (GPU) | 575 / 2021-09 / BSD-3 | 논문 [Barsan et al. ICRA 2018](https://doi.org/10.1109/ICRA.2018.8461224), [GitHub](https://github.com/AndreiBarsan/DynSLAM) |
| **Voxblox++** | 2019 (IROS) | RGB-D | RGB-D 센서 | 외부 (시험에서는 ORB-SLAM2 등) | Voxblox TSDF + 복셀마다 인스턴스 라벨 | Mask R-CNN + 기하 분할로 인스턴스 | 예 | 464 / 2021-02 / BSD-3 | arXiv [1903.00268](https://arxiv.org/abs/1903.00268), [GitHub](https://github.com/ethz-asl/voxblox-plusplus) |
| **panoptic_mapping** | 2022 (ICRA) | RGB-D | RGB-D 센서 | 외부 포즈 | 물체마다 TSDF 조각 맵(multi-TSDF) | 판옵틱(인스턴스 + 배경) | 예 | 334 / 2024-11 / BSD-3 | arXiv [2109.10165](https://arxiv.org/abs/2109.10165), [GitHub](https://github.com/ethz-asl/panoptic_mapping) |
| **Voxblox** (기반 라이브러리) | 2017 (IROS) | 깊이 센서 아무거나 | 입력 깊이 | 외부 | TSDF/ESDF 복셀 | 없음 (확장에서 추가) | 예 (CPU) | 1675 / 2024-07 / BSD-3 | arXiv [1611.03631](https://arxiv.org/abs/1611.03631), [GitHub](https://github.com/ethz-asl/voxblox) |
| **MaskFusion** | 2018 (ISMAR) | RGB-D | RGB-D 센서 | 자체 ICP 추적 (물체마다 따로) | 서펠(surfel). 물체마다 따로 모델 | Mask R-CNN 인스턴스 + 움직이는 물체 추적 | 예 | 586 / 2021-10 / 커스텀(NOASSERTION) | arXiv [1804.09194](https://arxiv.org/abs/1804.09194), [GitHub](https://github.com/martinruenz/maskfusion) |
| **ConceptGraphs** | 2024 (ICRA) | RGB-D (+ 포즈) | RGB-D 센서 | 데이터셋 포즈 / 외부 SLAM | 물체 점군 + 3D 장면 그래프 | 개방 어휘 인스턴스 (SAM + CLIP) | 아니오 (주로 오프라인) | 972 / 2025-10 / MIT | arXiv [2309.16650](https://arxiv.org/abs/2309.16650), [GitHub](https://github.com/concept-graphs/concept-graphs) |
| **semantic_slam (floatlazer)** | 2019 | RGB-D | RGB-D 센서 | ORB-SLAM2 | OctoMap 의미 복셀 | 의미(PSPNet), 인스턴스 아님 | 예 | 717 / 2019-05 / GPL-3.0 | [GitHub](https://github.com/floatlazer/semantic_slam) |
| **orb-slam2_with_semantic_label** | 2018~ | RGB-D | RGB-D 센서 | ORB-SLAM2 | 의미 점군 맵 | YOLOv3 박스 + 기하 분할 | 예 | 280 / 2022-02 / NOASSERTION | [GitHub](https://github.com/qixuxiang/orb-slam2_with_semantic_label) |
| **semantic_slam (hridaybavle)** | 2020 | 단안/스테레오 + IMU | (희소) | VIO (ROVIO, VINS-Mono, OKVIS) + 의미 랜드마크 최적화 | 희소 평면 랜드마크 맵 (복셀 아님) | YOLO 검출 | 예 | 151 / 2021-07 / 없음 | [GitHub](https://github.com/hridaybavle/semantic_slam) |
| **yolo_ros** (mgonzs13) | 2023~ | RGB(-D) | RGB-D 있을 때 마스크 안 깊이로 3D 박스 | 없음 | 맵 없음 (3D 박스) | YOLO 인스턴스 분할 + 추적 | 예 | 1151 / 2026-10 / MIT (단, Ultralytics 본체는 AGPL-3.0) | [GitHub](https://github.com/mgonzs13/yolo_ros), [Ultralytics 라이선스](https://www.ultralytics.com/license) |
| **RTAB-Map** (+ Android 앱) | 2013~ | RGB-D, 스테레오, 라이다. Android 앱은 ARCore/Tango 폰 | 센서 깊이 또는 스테레오 정합 | 자체 시각 오도메트리 / 외부 VIO / ARCore | 점군, 메시, OctoMap | 없음 (기본) | 예 | 4040 / 2026-10 / NOASSERTION(BSD 계열로 알려짐, 미확인) | [GitHub](https://github.com/introlab/rtabmap), [공식 사이트](https://introlab.github.io/rtabmap/) |
| **OpenVINS** (우리 포즈 부품) | 2020 (ICRA) | 단안/스테레오 + IMU | 희소 특징만 | MSCKF 기반 VIO | 희소 특징 (맵 없음) | 없음 | 예 | 3143 / 2025-11 / GPL-3.0 | 논문 [ICRA 2020](https://docs.openvins.com/), [GitHub](https://github.com/rpng/open_vins) |
| **Fast-FoundationStereo** (우리 깊이 부품) | 2026 (CVPR) | 정렬된 스테레오 쌍 | 학습 기반 파운데이션 스테레오. RTX 3090, 640×480에서 14~23 ms(TensorRT) | 해당 없음 | 시차/깊이 영상 | 없음 | 예 (GPU) | 1535 / 2026-09 / NVIDIA 라이선스(NOASSERTION), 상업용 가중치는 NVIDIA Open Model Agreement | arXiv [2512.11130](https://arxiv.org/abs/2512.11130), [GitHub](https://github.com/NVlabs/Fast-FoundationStereo) |
| **FoundationStereo** | 2025 (CVPR) | 정렬된 스테레오 쌍 | 학습 기반 (합성 스테레오 쌍 100만 개 이상으로 학습) | 해당 없음 | 시차 영상 | 없음 | 아니오 (느림) | 2949 / 2025-12 / NOASSERTION | arXiv [2501.09898](https://arxiv.org/abs/2501.09898), [GitHub](https://github.com/NVlabs/FoundationStereo) |
| **MASt3R-SLAM** | 2025 (CVPR) | 단안 RGB | 3D 복원 사전 모델(MASt3R) | 자체 (단안) | 조밀 점 지도 | 없음 | 예 (약 15 fps, 논문 기준) | 3216 / 2025-11 / NOASSERTION | arXiv [2412.12392](https://arxiv.org/abs/2412.12392), [GitHub](https://github.com/rmurai0610/MASt3R-SLAM) |
| **DROID-SLAM** | 2021 (NeurIPS) | 단안/스테레오/RGB-D | 학습 기반 (조밀 흐름 + BA) | 자체 | 조밀 깊이 + 포즈 | 없음 | 거의 실시간 (GPU) | 2710 / 2025-05 / BSD-3 | arXiv [2108.10869](https://arxiv.org/abs/2108.10869), [GitHub](https://github.com/princeton-vl/DROID-SLAM) |
| **OpenCamera-Sensors** | 2021 | Android 폰 카메라 + IMU | 없음 (녹화 앱) | 없음 | 없음 | 없음 | 녹화 | 95 / 2022-10 / GPL-3.0 | [GitHub](https://github.com/prime-slam/OpenCamera-Sensors) |
| **MARS logger** (mobile-ar-sensor-logger) | 2019~ | Android/iOS 카메라 + IMU 동기 기록 | 없음 | 없음 (VIO용 데이터 수집) | 없음 | 없음 | 녹화 | 262 / 2022-01 / GPL-3.0 | [GitHub](https://github.com/OSUPCVLab/mobile-ar-sensor-logger) |
| **multi-cam-tool** | 2023 | Android 다중 카메라 동시 사용 | 없음 | 없음 | 없음 | 없음 | 미리보기 | 4 / 2023-03 / 없음 | [GitHub](https://github.com/Tass0sm/multi-cam-tool) |
| iOS-Depth-Camera (ashura1234) | 미확인 | iPhone 듀얼 광각 깊이 | iOS 내장 듀얼 카메라 깊이 | 미확인 | Open3D 점군 | 없음 | 미확인 | 저장소 이동됨 (API 301), 수치 미확인 | [GitHub](https://github.com/ashura1234/iOS-Depth-Camera) |

참고 (코드가 아닌 근거):
- Android는 9부터 "논리 카메라 = 같은 방향의 물리 카메라 여러 개" 개념과 물리 카메라별 동시 스트림, 렌즈 자세·내부 파라미터 메타데이터, 센서 동기 종류(`APPROXIMATE` / `CALIBRATED`)를 지원한다. 출처: [AOSP Multi-Camera 문서](https://source.android.com/docs/core/camera/multi-camera). S24가 광각+초광각 물리 스트림 조합을 허용하는지, 동기 종류가 무엇인지는 이 문서로는 미확인(기기에서 직접 조회 필요).
- 폰 듀얼 렌즈 스테레오의 한계(짧은 기준선, 초점거리가 다르면 정합 가능한 영역이 줄어듦)는 [ISPRS Archives XLII-1, 2018](https://isprs-archives.copernicus.org/articles/XLII-1/289/2018/)에서 보고되었다.
- 폰 이중 카메라 깊이 학습: Du²Net (Pixel 4 광각+망원, [arXiv 2003.14299](https://arxiv.org/abs/2003.14299)), DCDP 데이터셋 (Galaxy S23 Ultra 2대 대칭 배치, [arXiv 2506.14709](https://arxiv.org/abs/2506.14709)). 광각+초광각 한 대 구성 데이터셋은 이번 검색에서 찾지 못함(미확인).
- Reddit, 블로그: "안드로이드 후면 렌즈 2개 동시 스테레오 3D 스캔" 사례를 검색했지만 Reddit 글은 찾지 못했다. 찾은 것은 Niantic Lightship 포럼 질문 하나 ([링크](https://community.nianticspatial.com/t/scan-with-multiple-cameras/4574))뿐이고, 동작하는 구현은 없었다.

## 3. 우리와 가장 가까운 3개와 차이점

### 1) Kimera (Kimera-VIO + Kimera-Semantics): 파이프라인 구조가 가장 비슷하다
- 같은 점: 스테레오+IMU VIO 포즈 → 스테레오 조밀 깊이 → 2D 분할 결과를 복셀(TSDF)에 융합 → 실시간 의미 3D 맵. 오픈소스(BSD).
- 다른 점:
  - 깊이: 고전 정합(StereoBM/SGM)을 쓴다. 논문 소거 실험에서 성능이 가장 많이 떨어진 원인이 조밀 스테레오(벽처럼 무늬 없는 영역)였다. 우리는 이 부분을 학습 기반 Fast-FoundationStereo로 바꾼다.
  - 물체 구분: 복셀마다 **클래스** 확률만 둔다. 인스턴스(물체 하나하나)는 구분하지 않는다. 물체 단위는 후속 Hydra/Khronos에서 장면 그래프로 다룬다.
  - 센서: 기준선이 수 cm~10 cm 이상이고 같은 렌즈 두 개를 하드웨어로 동기한 로봇용 스테레오 리그(EuRoC 등)를 가정한다. 우리는 기준선 15.5 mm, 초점거리가 다른 폰 렌즈 쌍이다.
  - VIO: Kimera-VIO(GTSAM 평활화)를 쓰고, 우리는 OpenVINS(MSCKF)를 쓴다.

### 2) Isaac ROS nvblox + cuVSLAM + DNN Stereo Depth: 학습 기반 스테레오 + GPU 복셀이라는 점에서 가장 가깝다
- 같은 점: 학습 기반 스테레오 깊이(NVIDIA 계열. FoundationStereo도 NVIDIA 작품) + 스테레오+IMU 시각 SLAM + GPU TSDF 복셀을 실시간으로 잇는다. 분할 마스크(사람)를 복셀 맵에 넣는 경로도 있다.
- 다른 점:
  - 물체 구분: 사람/동적 물체만 따로 처리하고, 임의 물체를 인스턴스별로 나누지 않는다.
  - 센서: RealSense, ZED 같은 전용 스테레오 카메라와 Jetson/ROS 2를 전제로 한다. 폰 입력, 오프라인 녹화 재생은 기본 기능이 아니다.
  - Isaac ROS DNN Stereo Depth에 FoundationStereo가 들어 있는지는 미확인이다.
  - 라이선스: nvblox 본체는 NVIDIA 커스텀 라이선스(NOASSERTION)라 논문/공개 시 조건을 확인해야 한다.

### 3) DynSLAM: "스테레오 깊이 + 인스턴스 분할 + 물체별 복셀 맵"이라는 점에서 가장 가깝다
- 같은 점: 스테레오 쌍에서 깊이를 구하고(학습 기반 DispNet 선택 가능) 인스턴스 분할로 물체(차량)마다 별도 복셀 볼륨을 만든다. "물체별로 구분된 복셀 맵"이라는 개념이 우리와 같다.
- 다른 점:
  - 포즈: IMU 없이 스테레오 시각 오도메트리(libviso2)만 쓴다. 우리는 OpenVINS VIO다.
  - 규모, 센서: 차량 KITTI 스테레오(기준선 약 0.54 m), 실외 차량만 다룬다. 우리는 손에 든 폰, 기준선 15.5 mm, 실내외 일반 물체다.
  - 연대: 2018년 코드(2021년 이후 갱신 없음)이고 분할망과 깊이망이 모두 구형이다. YOLO 분할이나 파운데이션 스테레오 같은 최신 모델은 없다.

(차점: Voxblox++와 panoptic_mapping은 인스턴스 TSDF 맵 표현으로는 우리 목표와 거의 같지만, 입력이 RGB-D 센서이고 포즈는 외부에서 받는다.)

## 4. 논문 각도에 대한 메모 (추정)
- 위 표에서 "폰 + 계측용 스테레오 + VIO + 인스턴스 복셀"을 모두 갖춘 행은 없다. 폰 쪽은 녹화 앱(OpenCamera-Sensors, MARS)이나 내장 깊이(ARCore, iOS)에서 멈추고, 스테레오+인스턴스 맵 쪽은 로봇용 리그나 RGB-D를 전제로 한다. 이 빈칸을 우리 기여로 쓸 수 있다(추정. 다른 각도의 문헌 조사로 다시 확인해야 함).
- 기여를 뒷받침하려면 정량 비교가 필요하다. 예: 같은 장면에서 SGBM 대 Fast-FoundationStereo(Kimera식 고전 깊이 대 학습 깊이), 기준선 15.5 mm에서 거리별 깊이 오차, 광각/초광각 초점거리 차이 보정의 영향.
- 라이선스 주의: OpenVINS(GPL-3.0), Ultralytics YOLO(AGPL-3.0), Fast-FoundationStereo(NVIDIA 라이선스)가 섞인다. 코드 공개 시 GPL/AGPL 조건을 따라야 하고, NVIDIA 가중치는 연구용과 상업용 조건이 다르다.
