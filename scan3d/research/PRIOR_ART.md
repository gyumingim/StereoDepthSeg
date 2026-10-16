# 선행 연구 종합 (PRIOR_ART)

작성일: 2026-10-16
입력: `phone-3d.md`, `semantic-mapping.md`, `learned-stereo-slam.md`, `open-source-stack.md` (조사 4건), `_verify.md` (독립 검증)
원칙: 조사 파일과 검증 파일의 판정이 다르면 **`_verify.md`의 판정을 따른다**. 출처로 확인하지 못한 내용은 **미확인**이라고 적었다.

**우리 시스템**: 일반 안드로이드 폰(Galaxy S24)의 후면 광각과 초광각 렌즈를 동시에 스트림한다(기준선 15.5 mm, 초점거리 서로 다름). PC에서 Fast-FoundationStereo로 깊이를 구하고 YOLO로 인스턴스를 분할한다. 포즈는 OpenVINS 스테레오+IMU VIO로 구한다. 결과는 물체별로 구분된 복셀 3D 맵이고, 실시간으로 만들거나 폰 녹화 후 오프라인으로 복원한다.

---

## 1. 한 줄 결론

**거의 같은 것을 한 사례는 찾지 못했다.** 다만 다섯 요소 가운데 셋 이상을 묶은 선행 사례는 이미 있다.
- NVIDIA Isaac ROS는 (Fast-)FoundationStereo + cuVSLAM + nvblox를 공식 지원한다. 그래서 "학습 스테레오 + 스테레오-관성 VIO + GPU 복셀"만으로는 새롭다고 할 수 없다.
- MobiDepth는 "폰 이종 듀얼 카메라로 스테레오 깊이"라는 하드웨어 문제를 이미 다뤘다.
- 다섯 요소를 모두 묶은 것이 정말 처음인지는 **미확인**이다(추가 검색에서도 나오지 않았을 뿐이다).

---

## 2. 가장 가까운 5개 비교표

선정 기준: 우리 다섯 요소(폰 이종 렌즈 / 파운데이션 스테레오 / 스테레오-관성 VIO / 인스턴스 복셀 / 실시간·오프라인) 가운데 겹치는 축이 많고, 논문 심사에서 "이미 있지 않냐"는 질문이 나올 만한 것.

| # | 이름 | 센서 | 깊이 | 포즈 | 맵 표현 | 물체 구분 | 실시간 | 폰 여부 | 코드 | 출처 |
|---|---|---|---|---|---|---|---|---|---|---|
| — | **우리** | S24 광각+초광각(15.5 mm, 초점거리 다름) + 폰 IMU | Fast-FoundationStereo (PC GPU) | OpenVINS 스테레오+IMU | 물체 ID가 붙은 복셀 | YOLO 인스턴스 | 예 + 녹화 후 오프라인 | 예 (안드로이드) | — | — |
| 1 | **NVIDIA Isaac ROS 스택** (FoundationStereo / Fast-FoundationStereo / ESS + cuVSLAM + nvblox) | 전용 스테레오 카메라(+IMU). NuRec 권장 카메라는 ZED 2i, Hawk | 학습 스테레오. 공식 문서가 FoundationStereo와 Fast-FoundationStereo를 지원 | cuVSLAM 스테레오-관성 (폐쇄 소스) | nvblox GPU TSDF/ESDF 복셀 | 사람(PeopleSemSegNet) 마스킹과 동적 물체 모드만. 일반 물체 인스턴스 아님 | 예 (스택 전체 FPS는 미확인). NuRec 경로는 오프라인 | 아니오 | nvblox 공개(NVIDIA 커스텀 라이선스), isaac_ros_nvblox Apache-2.0, cuVSLAM 비공개 | https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html · https://arxiv.org/abs/2311.00626 · https://github.com/nvidia-isaac/nvblox · https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox · https://arxiv.org/abs/2506.04359 · https://docs.nvidia.com/nurec/robotics/neural_reconstruction_stereo.html |
| 2 | **Kimera** (Kimera-VIO + Kimera-Semantics, ICRA 2020) | 대칭 스테레오 리그 + IMU (RGB-D도 가능) | 고전 정합. 저장소 기준 OpenCV **StereoBM** | Kimera-VIO 스테레오+IMU (GTSAM) | Voxblox TSDF → 의미 메시, 복셀별 클래스 확률 | 의미 클래스만. 인스턴스 없음 | 예 (CPU) | 아니오 | 공개, BSD-2 | https://arxiv.org/abs/1910.02490 · https://github.com/MIT-SPARK/Kimera-VIO · https://github.com/MIT-SPARK/Kimera-Semantics |
| 3 | **MobiDepth** (MobiCom 2022) / HiMoDepth (IEEE TMC) | 폰 후면 이종 듀얼 카메라(초점거리 다름). 실험 기기 Huawei Mate40Pro | 반복 시야 자르기 + 이종 카메라 동기화 + 모바일 GPU용 SGBM 계열 고전 정합. 학습 모델(AnyNet, MADNet, HITNet)은 비교 기준선으로만 씀 | 없음 | 프레임별 깊이 이미지 (맵 없음) | 없음 | 예. 22 FPS(평균 45 ms) 온디바이스 | **예** | 미확인 | https://www.microsoft.com/en-us/research/uploads/prod/2022/09/mobicom22-final138.pdf · https://ieeexplore.ieee.org/document/10178038/ (HiMoDepth 본문은 미확인) |
| 4 | **DynSLAM** (ICRA 2018) | 차량 스테레오 카메라 (KITTI, 기준선 약 0.54 m) | ELAS 또는 학습 기반 DispNet | 스테레오 시각 오도메트리, IMU 없음 (libviso2 사용 여부는 미확인) | InfiniTAM 복셀 TSDF. 정적 배경과 차량을 따로 복원 | 인스턴스(MNC)로 차량마다 별도 복셀 볼륨 | 거의 실시간 (GPU) | 아니오 | 공개, BSD-3 (2021년 이후 갱신 없음) | https://doi.org/10.1109/ICRA.2018.8461224 · https://github.com/AndreiBarsan/DynSLAM |
| 5 | **Voxblox++** (RA-L 2019) / Panoptic Mapping (ICRA 2022) | RGB-D | 능동 깊이 센서 | 외부 포즈 (데이터셋·SLAM) | Voxblox TSDF + 복셀별 인스턴스 라벨 / 물체별 서브맵 TSDF | 인스턴스 + 클래스(Mask R-CNN + 기하 분할) / 판옵틱 | 예 (논문 주장) | 아니오 | 공개, BSD-3 | https://arxiv.org/abs/1903.00268 · https://github.com/ethz-asl/voxblox-plusplus · https://arxiv.org/abs/2109.10165 · https://github.com/ethz-asl/panoptic_mapping |

### 표 밖에서 꼭 인용해야 할 것
| 이름 | 왜 인용해야 하나 | 출처 |
|---|---|---|
| iOS `builtInDualWideCamera` 깊이 (AVFoundation) | 광각+초광각 시차 깊이(`AVDepthData`)를 **상용 공개 API로 제공**한다(iPhone만, 스트리밍 깊이는 저해상도). 우리 렌즈 조합의 직접 선례다. 근거는 Apple 포럼 답변이고, 공식 문서 본문은 미확인 | https://developer.apple.com/documentation/avfoundation/avcapturedevice/devicetype-swift.struct/builtindualwidecamera · https://developer.apple.com/forums/thread/756719 |
| SemanticXR (ISMAR 2026) | 기기 → 서버로 나눠 처리해 물체 단위 의미 맵을 만든다. 목표가 비슷하다. 깊이는 데이터셋 RGB-D(Replica, Jetson Orin) 또는 iPad Pro(ARKit)·Quest 3(Depth API)에서 오고, 맵은 **물체별 점군**(복셀 아님)이다. 코드 링크 없음 | https://arxiv.org/abs/2606.12849 · https://arxiv.org/html/2606.12849 |
| SVD: Spatial Video Dataset | iPhone Pro 광각+초광각 공간 비디오 데이터셋. 폰 이종 렌즈 스테레오 데이터의 선례(학회와 렌즈 간격 수치는 미확인) | https://arxiv.org/abs/2506.06037 · https://cd-athena.github.io/SVD/ |
| Tele-wide 전 화각 깊이 (ICASSP 2020) | 화각이 다른 폰 렌즈 쌍에서 겹치지 않는 영역까지 깊이를 채운다. 초광각 바깥 영역 처리와 같은 문제 | https://arxiv.org/abs/1909.03375 |
| Du²Net (ECCV 2020) | Pixel 4 메인+망원 듀얼 카메라 + 듀얼픽셀 학습 깊이 | https://arxiv.org/abs/2003.14299 |
| Zoom and Learn | 폰 스테레오(작은 시차, 잡음)에 학습 스테레오를 적응시킨 초기 사례(학회는 미확인) | https://arxiv.org/abs/1803.06641 |
| ISPRS 2018 폰 듀얼 카메라 평가 (Masiero 외) | 초점거리가 다른 후면 카메라 2개. 짧은 기준선 → 깊이 분해능 제한, 초점거리 차이 → 정합 영역 감소. 기종은 미확인("LG G6"는 확인 안 됨) | https://isprs-archives.copernicus.org/articles/XLII-1/289/2018/ |
| SimpleMapping (ISMAR 2023) | VIO + 학습 MVS → TSDF 실시간. 구조는 비슷하지만 단안이고 물체 구분이 없다. 코드 공개 여부 미확인 | https://arxiv.org/abs/2306.08648 |
| OpenVox / SLIM-VDB | 최신 실시간 인스턴스(OpenVox)·의미(SLIM-VDB) 복셀 맵. SLIM-VDB의 인스턴스 지원 여부와 OpenVox의 센서는 미확인 | https://arxiv.org/abs/2502.16528 · https://arxiv.org/abs/2512.12945 · https://github.com/umfieldrobotics/slim-vdb |
| FoundationSLAM | FoundationStereo의 FeatureNet 인코더를 고정해 단안 SLAM에 쓴다(18 FPS). 파운데이션 스테레오를 SLAM에 쓴 사례 | https://arxiv.org/abs/2512.25008 · https://arxiv.org/html/2512.25008 |

---

## 3. 우리 차별점 (근거와 함께, 과장 금지)

### 쓸 수 있는 차별점
| # | 차별점 | 근거 | 주의 |
|---|---|---|---|
| A | **일반 안드로이드 폰의 이종 초단 기준선 렌즈(광각+초광각, 15.5 mm)를 계측용 스테레오로 쓰고, 그 위에 VIO와 3D 맵까지 잇는다** | MobiDepth/HiMoDepth, iOS Dual Wide, Du²Net, Tele-wide는 모두 **프레임 단위 깊이**에서 끝난다. VIO와 시간 누적 맵이 없다(표 3행, 2절 보조표) | "폰 광각+초광각 스테레오 깊이 최초"라고 쓰면 안 된다. iOS Dual Wide, SVD, MobiDepth가 있다 |
| B | **그 조건에서 파운데이션 스테레오가 버티는지 정량화** (거리별 깊이 오차, 이종 화각, 비동기 셔터) | 폰 렌즈 쌍에 FoundationStereo 계열을 적용한 논문은 이번 조사와 검증 검색에서 나오지 않았다(미확인). MobiDepth는 고전 정합, Zoom and Learn은 2018년식 학습 스테레오 | 측정 전에는 "버틴다"고 쓸 수 없다. 결과가 좋든 나쁘든 정량화 자체를 기여로 둔다 |
| C | **물체 인스턴스 ID 복셀 맵**을 수동(패시브) 스테레오 깊이로 만든다 | Isaac ROS는 사람·동적 물체만, Kimera는 클래스 단위다. Voxblox++/Panoptic Mapping은 인스턴스지만 RGB-D 입력이다. DynSLAM은 스테레오 + 인스턴스 복셀이지만 차량 전용이고 IMU가 없으며 2018년 모델이다 | 인스턴스 복셀 맵 자체는 새롭지 않다(Voxblox++, DynSLAM, OpenVox). 새로운 것은 "폰 수동 스테레오 깊이 잡음 위에서"라는 조건이다 |
| D | **실시간 스트림과 녹화 후 오프라인 복원, 두 경로**를 같은 폰 데이터로 지원 | 표의 학술 시스템은 대부분 한쪽만 다룬다. NuRec은 오프라인이지만 전용 스테레오 카메라 기준이다 | 기여의 중심으로 두기에는 약하다. 시스템 특징으로 적는 정도가 적절하다(추정) |

### 쓰면 안 되는 주장
- "학습 스테레오 + VIO + GPU 복셀 조합이 새롭다" → Isaac ROS가 공식 지원한다.
- "폰 광각+초광각 스테레오 깊이는 처음" → iOS Dual Wide 깊이 API, SVD, MobiDepth/HiMoDepth가 있다.
- "다섯 요소를 묶은 첫 연구" → 찾지 못했을 뿐이다(**미확인**). "우리가 아는 한(to our knowledge)" 정도로 낮춰 쓴다.
- "Kimera의 소거 실험에서 조밀 스테레오가 가장 큰 오차 원인" → 논문 본문에서 확인하지 못했다(**미확인**). 근거로 쓰지 않는다.
- "SemanticXR은 LiDAR+ARKit 의존" → 틀렸다(2절 보조표 참고).

### 예상 약점 (실험으로 정량화해야 함)
- 기준선 15.5 mm: 스테레오 깊이 오차는 거리의 제곱에 비례해 커진다. 유효 거리 범위를 숫자로 밝혀야 한다.
- 광각과 초광각의 해상도·왜곡·화각 차이: 겹치는 영역이 줄어든다(ISPRS 2018, Tele-wide).
- 셔터 동기: Android multi-camera의 `CALIBRATED`(하드웨어 동기) 지원 여부는 S24에서 직접 조회해야 한다(미확인). https://developer.android.com/media/camera/camera2/multi-camera · https://source.android.com/docs/core/camera/multi-camera
- 초광각 쪽 모션 블러가 크다는 지적이 있다(HSD 벤치마크, iPhone 렌즈 조합인지는 미확인). https://arxiv.org/abs/2606.25962

---

## 4. 논문 비교 실험 대상 추천 (공개 코드 위주)

### 4-1. 깊이 (같은 S24 렌즈 쌍 입력, 거리별 오차 곡선)
| 대상 | 무엇을 보이나 | 코드 | 출처 |
|---|---|---|---|
| OpenCV StereoBM / SGBM | Kimera식 고전 정합 대 학습 스테레오 | 공개 (OpenCV) | https://github.com/MIT-SPARK/Kimera-Semantics (Kimera가 StereoBM 사용) |
| MobiDepth식 시야 자르기 + SGBM | **필수**. 같은 하드웨어 문제 정의의 기준선 | 코드 미확인 → 논문대로 재구현 필요 | https://www.microsoft.com/en-us/research/uploads/prod/2022/09/mobicom22-final138.pdf |
| FoundationStereo (원본) | 속도 대 정확도 상한 (오프라인 경로 기준) | 공개 | https://arxiv.org/abs/2501.09898 · https://github.com/NVlabs/FoundationStereo |
| RAFT-Stereo / IGEV-Stereo / CREStereo | 파운데이션 이전 학습 스테레오와의 차이 | 공개 | https://github.com/princeton-vl/RAFT-Stereo · https://github.com/gangweiX/IGEV · https://github.com/megvii-research/CREStereo |
| 단안 파운데이션 깊이 + VIO 스케일 정렬 (Depth Anything V2 + VI-Depth 방식) | "렌즈 하나로도 되지 않냐"는 질문에 답 | 공개 | https://arxiv.org/abs/2406.09414 · https://github.com/isl-org/VI-Depth |
| (선택) ARCore Raw Depth | 같은 폰의 상용 깊이. S24 지원 여부는 기기 목록 확인 필요 | 비공개 SDK, codelab 공개 | https://developers.google.com/ar/develop/depth · https://developers.google.com/ar/devices |
| (선택) iPhone Dual Wide 내장 깊이 | 같은 렌즈 조합의 상용 깊이. 다른 기기라서 직접 비교는 제한적(추정 제안) | 비공개 API | https://developer.apple.com/forums/thread/756719 |

### 4-2. 포즈
| 대상 | 무엇을 보이나 | 코드 | 출처 |
|---|---|---|---|
| OpenVINS 단안+IMU 대 스테레오+IMU | 이종 초단 기준선 스테레오가 VIO에 도움이 되는가 | 공개, GPL-3.0 | https://github.com/rpng/open_vins |
| Kimera-VIO | 다른 스테레오-관성 VIO와의 비교 | 공개, BSD-2 | https://github.com/MIT-SPARK/Kimera-VIO |

### 4-3. 맵 (같은 깊이·포즈 입력을 넣고 맵 모듈만 바꿈)
| 대상 | 무엇을 보이나 | 코드 | 출처 |
|---|---|---|---|
| Voxblox++ | 인스턴스 복셀 맵의 대표 기준선 (원래 RGB-D) | 공개, BSD-3 | https://github.com/ethz-asl/voxblox-plusplus |
| panoptic_mapping | 물체별 서브맵 방식과의 비교 | 공개, BSD-3 | https://github.com/ethz-asl/panoptic_mapping |
| Kimera-Semantics | 클래스 단위 맵 대 인스턴스 맵 | 공개, BSD-2 | https://github.com/MIT-SPARK/Kimera-Semantics |
| nvblox (+ Isaac ROS Fast-FoundationStereo) | **가장 위협적인 선행 사례와의 직접 비교**. 폰 입력을 넣었을 때 어떻게 되는지 | 공개(커스텀 라이선스, 조건 확인 필요) | https://github.com/nvidia-isaac/nvblox · https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html |
| SLIM-VDB | 최신 실시간 의미 복셀(OpenVDB) | 공개 | https://github.com/umfieldrobotics/slim-vdb |

### 4-4. 데이터
- 자체 S24 데이터와 함께, 같은 렌즈 조합 공개 데이터인 SVD(iPhone 광각+초광각)에서 깊이 모델만 시험해 볼 수 있다(GT 깊이 포함 여부는 미확인). https://arxiv.org/abs/2506.06037

### 라이선스 주의
OpenVINS(GPL-3.0), Ultralytics YOLO(AGPL-3.0), Fast-FoundationStereo(NVIDIA 라이선스, 상업용은 NVIDIA Open Model Agreement), nvblox(NVIDIA 커스텀)가 섞인다. 코드를 공개할 때 조건을 확인해야 한다. https://www.ultralytics.com/license · https://github.com/NVlabs/Fast-FoundationStereo

---

## 5. 출처 목록

### 우리 부품
- Fast-FoundationStereo: https://arxiv.org/abs/2512.11130 · https://github.com/NVlabs/Fast-FoundationStereo (CVPR 2026 채택은 README 표기 기준)
- FoundationStereo (CVPR 2025): https://arxiv.org/abs/2501.09898 · https://github.com/NVlabs/FoundationStereo
- OpenVINS: https://github.com/rpng/open_vins · https://docs.openvins.com/
- Ultralytics 라이선스: https://www.ultralytics.com/license
- Android multi-camera: https://developer.android.com/media/camera/camera2/multi-camera · https://source.android.com/docs/core/camera/multi-camera

### 비교표 5개
- Isaac ROS FoundationStereo 문서: https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html
- Isaac ROS ESS: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/index.html · https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth
- nvblox (ICRA 2024): https://arxiv.org/abs/2311.00626 · https://github.com/nvidia-isaac/nvblox · https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox
- cuVSLAM: https://arxiv.org/abs/2506.04359
- NuRec 스테레오 복원: https://docs.nvidia.com/nurec/robotics/neural_reconstruction_stereo.html
- Kimera: https://arxiv.org/abs/1910.02490 · https://github.com/MIT-SPARK/Kimera · https://github.com/MIT-SPARK/Kimera-VIO · https://github.com/MIT-SPARK/Kimera-Semantics
- MobiDepth (MobiCom 2022): https://www.microsoft.com/en-us/research/uploads/prod/2022/09/mobicom22-final138.pdf
- HiMoDepth (IEEE TMC): https://ieeexplore.ieee.org/document/10178038/ · https://doi.org/10.1109/TMC.2023.3294188
- DynSLAM: https://doi.org/10.1109/ICRA.2018.8461224 · https://github.com/AndreiBarsan/DynSLAM
- Voxblox++: https://arxiv.org/abs/1903.00268 · https://github.com/ethz-asl/voxblox-plusplus
- Panoptic Mapping: https://arxiv.org/abs/2109.10165 · https://github.com/ethz-asl/panoptic_mapping

### 폰 이종 렌즈 깊이
- iOS Dual Wide: https://developer.apple.com/documentation/avfoundation/avcapturedevice/devicetype-swift.struct/builtindualwidecamera · https://developer.apple.com/forums/thread/756719 · https://developer.apple.com/documentation/avfoundation/capturing-photos-with-depth
- SVD: https://arxiv.org/abs/2506.06037 · https://cd-athena.github.io/SVD/
- Tele-wide 전 화각 깊이: https://arxiv.org/abs/1909.03375
- Zoom and Learn: https://arxiv.org/abs/1803.06641
- Du²Net: https://arxiv.org/abs/2003.14299
- HSD 이종 스테레오 디블러: https://arxiv.org/abs/2606.25962
- ISPRS 2018 폰 듀얼 카메라: https://isprs-archives.copernicus.org/articles/XLII-1/289/2018/
- DC² (초점 합성, 깊이 아님): https://arxiv.org/abs/2304.03285
- DiFuse-Net / DCDP (듀얼픽셀, 광각+초광각 아님): https://arxiv.org/abs/2506.14709
- ARCore Depth: https://developers.google.com/ar/develop/depth · https://developers.google.com/ar/devices

### 매핑·SLAM
- SemanticXR (ISMAR 2026): https://arxiv.org/abs/2606.12849 · https://arxiv.org/html/2606.12849
- SimpleMapping (ISMAR 2023): https://arxiv.org/abs/2306.08648
- FoundationSLAM: https://arxiv.org/abs/2512.25008
- OpenVox: https://arxiv.org/abs/2502.16528 · https://open-vox.github.io
- SLIM-VDB: https://arxiv.org/abs/2512.12945 · https://github.com/umfieldrobotics/slim-vdb
- IRIS-SLAM: https://arxiv.org/abs/2602.18709
- Stereo Anything: https://arxiv.org/abs/2411.14053
- Hydra: https://arxiv.org/abs/2201.13360 · https://github.com/MIT-SPARK/Hydra
- Khronos: https://arxiv.org/abs/2402.13817 · https://github.com/MIT-SPARK/Khronos
- RAFT-Stereo: https://github.com/princeton-vl/RAFT-Stereo · IGEV: https://github.com/gangweiX/IGEV · CREStereo: https://github.com/megvii-research/CREStereo
- Depth Anything V2: https://arxiv.org/abs/2406.09414 · VI-Depth: https://github.com/isl-org/VI-Depth

### 원 조사·검증 파일
- `phone-3d.md`, `semantic-mapping.md`, `learned-stereo-slam.md`, `open-source-stack.md`, `_verify.md` (같은 폴더)
