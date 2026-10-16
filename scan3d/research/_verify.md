# 선행 연구 조사 독립 검증 (_verify)

검증일: 2026-10-16 · 대상: `semantic-mapping.md`, `phone-3d.md`, `learned-stereo-slam.md`, `open-source-stack.md`
방법: arXiv API(`export.arxiv.org/api/query`)로 인용된 arXiv 번호 46개의 제목·공개일·학회 메모를 한꺼번에 조회했다. 핵심 항목은 arXiv 초록/HTML 본문, GitHub README, 공식 문서를 직접 열어 확인했다.
표기: **정정** = 원 파일 내용이 틀림 · **보완** = 맞지만 빠진 정보가 있음 · **확인** = 출처와 일치 · **미확인** = 이번에도 확인하지 못함.

---

## 1. 결론 요약
- 인용된 arXiv 번호 46개는 모두 실제 논문으로 존재하고, 제목도 파일 내용과 맞는다. 단 1건(DCDP)은 논문 이름을 잘못 적었다(아래 2-4).
- "우리 다섯 요소(폰 이종 렌즈 동시 스트림 + 파운데이션 스테레오 + 스테레오-관성 VIO + 인스턴스 복셀 맵 + 실시간/오프라인)를 모두 묶은 연구는 없다"는 공통 결론은 이번 추가 검색에서도 깨지지 않았다. 그래도 "처음"이라고 단정하기는 이르다(**미확인**).
- 가장 중요한 정정 2건:
  1. **SemanticXR은 "LiDAR+ARKit 의존"이 아니다.** 주 실험은 Replica 데이터셋(RGB-D + GT 포즈)을 Jetson Orin에서 돌린 것이다. 실제 기기 배포는 iPad Pro(ARKit 깊이)와 Meta Quest 3(Depth API)다. 맵은 복셀이 아니라 **물체별 점군**이다.
  2. **"후면 RGB 렌즈 두 개의 스테레오 깊이를 공개 API로 주는 곳은 없다"(phone-3d 3번)는 틀렸다.** iOS AVFoundation의 `builtInDualWideCamera`(광각+초광각)는 시차 기반 `AVDepthData`를 제공한다(iPhone에서만, iPad는 미지원). 우리와 같은 광각+초광각 조합의 상용 선례이므로 논문 관련 연구에 넣어야 한다.
- 새로 넣어야 할 선행 연구: SVD Spatial Video Dataset(iPhone 광각+초광각 스테레오 영상), Tele-wide 전 화각 깊이(ICASSP 2020), Zoom and Learn(폰 스테레오), 이종 스테레오 디블러 벤치마크(2026), OpenVox, SLIM-VDB(아래 4절).

---

## 2. 항목별 검증

### 2-1. 깊이 모델 (우리 부품)
| 주장 | 판정 | 근거 |
|---|---|---|
| Fast-FoundationStereo = arXiv 2512.11130, CVPR 2026 | **확인** (보완) | arXiv: 2025-12-11 v1, 2026-03-17 v2. arXiv 페이지에는 학회 표시가 없고, GitHub README에 "accepted to CVPR 2026"라고 적혀 있다. https://arxiv.org/abs/2512.11130 · https://github.com/NVlabs/Fast-FoundationStereo |
| FoundationStereo보다 10배 이상 빠르고 실시간 | **확인** | 초록: "over 10x faster than FoundationStereo", "real-time frame rate". |
| RTX 3090, 640×480, TensorRT 14–23 ms (open-source-stack) | **확인** | README 표: TensorRT 14.0–23.4 ms, PyTorch 29.3–49.4 ms, 메모리 약 650 MB. 우리 해상도와 GPU에서의 실측은 별도로 필요하다. |
| 상업용 가중치는 NVIDIA Open Model Agreement | **확인** | README에서 상업용으로 C-Fast-FoundationStereo(NVIDIA Open Model Agreement)와 TAO를 안내한다. 연구용 코드 라이선스 문구 본문은 이번에 열어 보지 못함(**미확인**). |
| Isaac ROS 문서: FoundationStereo는 "실시간이 필요 없는 응용에 적합" | **확인** | 원문: "a heavy model that is best-suited for applications that do not require real-time performance". 같은 문서가 Fast-FoundationStereo를 "real-time zero-shot stereo matching"이라고 소개한다. https://nvidia-isaac-ros.github.io/concepts/stereo_depth/foundationstereo/index.html |
| FoundationStereo = arXiv 2501.09898, CVPR 2025 | **확인** | arXiv 메모 "CVPR 2025". |

### 2-2. "우리와 거의 같다"고 한 항목
| 항목 (파일) | 원 주장 | 판정 | 근거 |
|---|---|---|---|
| Kimera (전 파일 공통 1~2위) | 스테레오+IMU VIO, 고전 스테레오 깊이, TSDF 의미 메시, 클래스 단위 | **확인** (정정 1건) | Kimera-Semantics README: 조밀 스테레오는 **OpenCV StereoBM**(`run_stereo_dense`)이다. semantic-mapping.md의 "SGBM"은 **정정 → StereoBM**. 인스턴스는 지원하지 않으며, README가 Voxblox++를 "instance-aware"라고 대비해 적는다. https://github.com/MIT-SPARK/Kimera-Semantics · 초록 https://arxiv.org/abs/1910.02490 |
| Kimera "소거 실험에서 성능이 가장 많이 떨어진 원인이 조밀 스테레오" (open-source-stack) | — | **미확인** | 이번에 논문 본문의 해당 표를 찾지 못했다. 근거로 쓰기 전에 본문을 확인할 것. |
| Isaac ROS 스택 / nvblox | 학습 스테레오 + cuVSLAM + nvblox, 사람만 분할 | **확인** | nvblox = ICRA 2024(arXiv 메모). 초록 주제는 GPU 가속 TSDF/ESDF(최대 177배)이고, 의미·인스턴스는 다루지 않는다. cuVSLAM 논문 arXiv 2506.04359 존재 확인. https://arxiv.org/abs/2311.00626 |
| open-source-stack: "Isaac ROS DNN Stereo Depth에 FoundationStereo 포함 여부 미확인" | — | **정정** | Isaac ROS 공식 문서에 FoundationStereo 페이지가 있고 Fast-FoundationStereo도 소개한다(위 2-1 URL). "미확인"을 "**포함됨**"으로 고친다. |
| NuRec 스테레오 복원 흐름 (learned-stereo-slam) | FoundationStereo(오프라인) → cuSFM → nvblox → 3DGRUT | **확인** | 문서 그대로. 권장 카메라 ZED 2i, Leopard Imaging Hawk, 녹화는 Jetson Orin + Nova recorder. https://docs.nvidia.com/nurec/robotics/neural_reconstruction_stereo.html |
| SimpleMapping (phone-3d 2위, learned-stereo-slam 3위) | 단안+IMU, VIO + 학습 MVS → TSDF, 실시간 | **확인** (보완) | 초록: 특징점 VIO + SPA-MVSNet + TSDF-Fusion, 공개 데이터셋과 **자체 데이터셋**으로 평가. 학회 = ISMAR 2023(TUM, Leutenegger 그룹). learned-stereo-slam의 "ORB-SLAM3 VIO"는 초록에 없음(**미확인**). phone-3d의 "코드 미확인"과 learned-stereo-slam의 "코드 O"가 서로 다르다(**미확인**, 둘 중 하나는 틀림). https://arxiv.org/abs/2306.08648 · https://mcml.ai/publications/xzl+23/ |
| MobiDepth (phone-3d 1위) | 이종 듀얼 카메라, 시야 자르기, 동기화, 학습 없는 GPU 스테레오, 22 fps | **확인** | 논문 PDF: Huawei Mate40Pro에서 22 FPS(평균 45 ms), 정지 물체 깊이 오차 1.1~10.4 %. 3대 기법 = 반복 시야 자르기, 이종 카메라 동기화, 모바일 GPU용 SGBM 계열 정합. 학습 모델(AnyNet, MADNet, HITNet)은 비교 기준선으로만 썼다. 학회 MobiCom 2022. https://www.microsoft.com/en-us/research/uploads/prod/2022/09/mobicom22-final138.pdf |
| HiMoDepth (IEEE TMC) | — | **미확인** | DOI가 IEEE Xplore 10178038로 이동하는 것만 확인했다. 본문은 열지 못함. https://ieeexplore.ieee.org/document/10178038/ |
| SemanticXR (phone-3d 3위) | 2026, LiDAR+ARKit 의존, 물체 단위 의미 맵, 실시간 | **정정** | arXiv 2606.12849, 2026-06-11 v1, **IEEE ISMAR 2026**. 주 실험은 Jetson Orin(저전력 모드)을 XR 기기 대용으로 쓰고 **Replica 데이터셋**(RGB-D + GT 포즈)을 넣었다. 실기기 배포: iPad Pro(ARKit RGB·깊이·포즈, 깊이 144×256), Meta Quest 3(Passthrough Camera·Depth API). 맵 = **물체별 점군**(ID, 의미 임베딩, 클래스, 점군, 상자)이고 복셀이 아니다. 코드 링크 없음. https://arxiv.org/abs/2606.12849 · https://arxiv.org/html/2606.12849 |
| DynSLAM (open-source-stack 3위) | ICRA 2018, ELAS/DispNet, MNC 인스턴스, InfiniTAM | **확인** | README 그대로. 시각 오도메트리 libviso2는 README 요약에서 확인하지 못함(**미확인**). https://github.com/AndreiBarsan/DynSLAM |
| Voxblox++ / Panoptic Mapping | 인스턴스 TSDF, RGB-D | **확인** | 1903.00268 = RA-L, 2109.10165 = ICRA 2022(arXiv 메모). |

### 2-3. 연도·학회 확인 (arXiv 메모 기준)
- **확인**: SemanticFusion(arXiv 2016-09, ICRA 2017), MaskFusion(ISMAR 2018), PanopticFusion(IROS 2019), Voxblox(IROS 2017 제출), Kimera-Multi T-RO(2106.14386), Hydra(RSS 2022), Khronos(RSS 2024), ConceptFusion(RSS 2023), OpenScene(CVPR 2023), MID-Fusion(ICRA 2019), Depth Anything(CVPR 2024), Prompt Depth Anything(CVPR 2025), MASt3R-SLAM(CVPR 2025), IGEV(CVPR 2023), CREStereo(CVPR 2022), Garg 듀얼픽셀(ICCV 2019), DC²(CVPR 2023), PanopticNDT(IROS 2023).
- **arXiv 메모에는 학회 표시가 없어 이번에 확인하지 못한 것(미확인)**: Fusion++(3DV 2018), RAFT-Stereo(3DV 2021), DROID-SLAM(NeurIPS 2021), MAC-VO(ICRA 2025), Clio(RA-L 2024), HOV-SG(RSS 2024), ConceptGraphs(ICRA 2024), UniDepth(CVPR 2024), Du²Net(ECCV 2020). 널리 알려진 내용이라 틀렸을 가능성은 낮지만, 논문 참고문헌에 넣을 때는 학회 페이지로 확인할 것.

### 2-4. 그 밖의 정정
| 파일 | 원 주장 | 정정 내용 | 근거 |
|---|---|---|---|
| learned-stereo-slam (FoundationSLAM) | FoundationStereo FeatureNet 인코더를 고정해 씀, 18 FPS | **확인** | 초록에는 FoundationStereo 이름이 없다. HTML 본문에 "frozen FeatureNet encoder from FoundationStereo"와 ContextNet을 쓴다고 나온다. 단안, 18 FPS. https://arxiv.org/html/2512.25008 |
| learned-stereo-slam (DAVIO) | 실시간 미확인, 맵 형식 미확인 | **보완** | 초록이 "real-time dense metric SLAM system"이라고 주장한다(FPS 수치는 없음). 맵은 중력 방향을 유지하는 서브맵 그래프이고 조밀 맵을 다듬는다. 2026-09-23 공개. https://arxiv.org/abs/2609.27702 |
| learned-stereo-slam (VIDAR) | 실시간 미확인 | **확인** | 초록에 실시간 주장이 없다. 포즈는 SVO+IMU, 깊이는 DA3, 결과는 "global reconstruction". 2026-07-19. https://arxiv.org/abs/2607.17171 |
| learned-stereo-slam (VGGT-SLAM 2.0) | 2026, 실시간 주장 | **확인** | 2026-01-27, "real-time RGB feed-forward SLAM", Jetson Thor 탑재 로봇에서 온라인 실행. https://arxiv.org/abs/2601.19887 |
| learned-stereo-slam (반사굴절 폰 스테레오) | 포즈 = "VO + 맵 점", 맵 = 맵 점 | **정정** | 3D 프린트 거울 어댑터로 **앞·뒤 카메라**를 스테레오로 만들고, 2D 관절점으로 사람 3D 자세와 신체 길이를 재는 논문이다. VO도 맵도 없다. https://arxiv.org/abs/2109.11872 |
| learned-stereo-slam (DC²) | 포트레이트 시차 + 광류, 출력 = 깊이(초점 제어용) | **정정** | 주 카메라 + 초광각의 서로 다른 피사계 심도를 융합해 **초점·보케를 합성**하는 시스템이다. 결과물은 깊이 지도가 아니라 이미지다. 광각+초광각 쌍을 다룬 선례로는 맞다. https://arxiv.org/abs/2304.03285 |
| learned-stereo-slam (LG G6 평가) | "LG G6", 광각+초광각 | **정정 → 기종 미확인** | ISPRS 초록에는 기종이 없고 "초점거리가 다른 후면 카메라 2개"라고만 적혀 있다. 결론(짧은 기준선 → 깊이 분해능 제한, 초점거리 차이 → 정합 영역 감소)은 맞다. 저자 Masiero 외(Padova). https://isprs-archives.copernicus.org/articles/XLII-1/289/2018/ |
| open-source-stack (DCDP) | "DCDP 데이터셋 (Galaxy S23 Ultra 2대), arXiv 2506.14709" | **정정** | 2506.14709는 **DiFuse-Net**(IROS 2025, RGB+듀얼픽셀 깊이) 논문이고, DCDP 데이터셋은 그 안에서 공개됐다. 초록은 "대칭 스테레오 카메라 하드웨어"라고만 적었고, S23 Ultra 2대라는 내용은 확인하지 못함(**미확인**). 광각+초광각 데이터셋이 아니다. https://arxiv.org/abs/2506.14709 |
| phone-3d (상용 SDK 결론) | 후면 RGB 두 렌즈 스테레오를 공개 API로 주는 것 확인 못함 | **정정** | iOS `AVCaptureDevice.DeviceType.builtInDualWideCamera`(초광각+광각 가상 장치)에서 `AVDepthData`(시차)를 받을 수 있다. Apple 포럼 답변에 따르면 iPhone은 지원하고 iPad는 미지원이다. 스트리밍 깊이 해상도는 낮다(정지 사진은 더 높음). 공식 문서 본문은 이번 도구로 읽지 못했다. https://developer.apple.com/documentation/avfoundation/avcapturedevice/devicetype-swift.struct/builtindualwidecamera · https://developer.apple.com/forums/thread/756719 · https://developer.apple.com/documentation/avfoundation/capturing-photos-with-depth |
| phone-3d / open-source-stack (Android) | 다중 카메라 API, 동기 종류 APPROXIMATE/CALIBRATED | **확인** (보완) | Android 9(API 28)부터 지원한다. 공식 문서가 "여러 카메라로 깊이 지도 만들기"를 대표 용도로 든다. 렌즈 자세·내부값·왜곡 메타데이터를 노출해야 한다. CALIBRATED = 하드웨어 셔터 동기. S24 지원 여부는 여전히 기기 조회가 필요하다. https://developer.android.com/media/camera/camera2/multi-camera · https://source.android.com/docs/core/camera/multi-camera |

### 2-5. 파일 간 불일치
- Kimera 깊이: semantic-mapping은 "SGBM", 나머지 둘은 "StereoBM/블록 매칭"이라고 적었다 → 저장소 기준 **StereoBM**으로 통일.
- Isaac ROS의 FoundationStereo: learned-stereo-slam은 "엔진 파일만 바꾸면 됨", open-source-stack은 "미확인" → 공식 문서 기준 **지원**.
- SimpleMapping 코드 공개: "O"와 "미확인"이 섞여 있다 → **미확인** 유지(GitHub 저장소를 직접 확인할 것).
- semantic-mapping 끝의 "웹으로 다시 열어 보지 않음" 경고: 이번 검증으로 arXiv 번호 18개는 모두 맞는 것으로 확인했다. GitHub URL은 open-source-stack이 API로 확인한 것과 겹치는 것만 **확인**이다(Bitbucket SemanticFusion, concept-fusion, openscene, HOV-SG 저장소는 **미확인**).

---

## 3. "우리와 거의 같다" 판정 정리
| 후보 | 같은 점 (확인됨) | 결정적 차이 (확인됨) | 판정 |
|---|---|---|---|
| Kimera | 스테레오+IMU VIO → 조밀 스테레오 → 2D 분할 → 복셀 TSDF, CPU 실시간 | StereoBM, 클래스 단위, 대칭 리그 | "구조가 가장 비슷하다"는 **타당**. "거의 같다"는 과장 |
| Isaac ROS (Fast-)FoundationStereo + cuVSLAM + nvblox | 같은 깊이 모델 계열, 스테레오-관성, GPU TSDF, 분할 마스크 경로 | 사람·동적 물체만, 전용 스테레오 카메라, cuVSLAM 폐쇄 | **가장 위협적인 선행 사례**. 공식 문서가 Fast-FoundationStereo를 지원하므로 "학습 스테레오 + VIO + 복셀"만으로는 새롭다고 할 수 없다. 차별점은 폰 이종 렌즈와 인스턴스 맵에 둬야 한다 |
| MobiDepth | 이종 초점거리 폰 듀얼 카메라, 시야 자르기, 비동기 프레임 처리 | 고전 정합, 맵·VIO 없음 | 하드웨어 문제 정의가 **사실상 같다**. 깊이 비교 기준선으로 필수 |
| SemanticXR | 기기 → 서버 분산, 물체 단위 맵 | 능동 깊이(ARKit/Quest)·데이터셋 RGB-D, 물체 점군 | 원 파일의 근거(LiDAR+ARKit)는 틀렸지만 "목표가 비슷하다"는 결론은 유지 |
| DynSLAM | 스테레오(+DispNet) + 인스턴스 → 물체별 복셀 | IMU 없음, 차량 KITTI | **타당**. 2018년 "스테레오 + 인스턴스 복셀"의 선례로 인용 필요 |

---

## 4. 빠진 중요한 선행 연구 (추가)
| 이름 | 연도 | 왜 중요한가 | 센서 / 깊이 / 포즈 / 맵 / 물체 / 실시간 | 출처 |
|---|---|---|---|---|
| iOS Dual Wide Camera 깊이 (AVFoundation) | iOS 13~ | **광각+초광각 스테레오 깊이를 상용 공개 API로 제공**. 우리 하드웨어 조합의 직접 선례 | iPhone 광각+초광각 / 내장 시차 / 없음(ARKit과 별개) / 프레임 깊이 / 없음 / 스트리밍 가능(저해상도) | https://developer.apple.com/documentation/avfoundation/avcapturedevice/devicetype-swift.struct/builtindualwidecamera · https://developer.apple.com/forums/thread/756719 |
| SVD: Spatial Video Dataset | 2025 (arXiv, 검색 결과에는 ACM MM'25로 나옴. 학회는 **미확인**) | iPhone Pro 광각+초광각 공간 비디오 150개 + Vision Pro 150개. 폰 이종 렌즈 스테레오 데이터셋의 선례. 검색 요약에는 렌즈 간격 약 19.2 mm로 나오나 초록에는 없음(**미확인**) | iPhone Pro 광각+초광각 / 데이터셋 / – / – / – / – | https://arxiv.org/abs/2506.06037 · https://cd-athena.github.io/SVD/ |
| Robust Full-FoV Depth Estimation in Tele-wide Camera System | 2019 (ICASSP 2020) | 화각이 다른 폰 렌즈 쌍에서 **겹치지 않는 영역까지** 학습으로 깊이를 채운다. 우리 초광각의 바깥 영역 처리와 같은 문제 | 망원+광각 / 고전 스테레오 + 단안 학습 + 전파 / 없음 / 깊이 / 없음 / 미확인 | https://arxiv.org/abs/1909.03375 |
| Zoom and Learn (ZOLE) | 2018 (CVPR, 지식 기준·**미확인**) | 폰 스테레오(작은 시차, 잡음)에 학습 스테레오를 적응시킨 초기 사례 | 폰 스테레오 / 학습 스테레오 도메인 적응 / 없음 / 시차 / 없음 / 미확인 | https://arxiv.org/abs/1803.06641 |
| Heterogeneous Stereo Deblurring 벤치마크 (HSD) | 2026 (arXiv) | 폰의 **서로 다른 카메라 모듈** 스테레오 데이터셋. 검색 요약에는 iPhone 광각+초광각 동기 스트림으로 나오나 초록에는 없음(**미확인**). 초광각 쪽 모션 블러가 크다는 지적은 우리 깊이 품질과 직결 | 폰 이종 스테레오 / – / – / – / – / – | https://arxiv.org/abs/2606.25962 |
| OpenVox | 2025 (arXiv) | **실시간 인스턴스 단위 확률 복셀 맵**(개방 어휘). 우리 "물체 ID 복셀 맵"의 최신 비교 대상. 센서·분할 모델은 초록에 없음(**미확인**) | 미확인 / – / 미확인 / 확률적 인스턴스 복셀 / 인스턴스 + 개방 어휘 / 예(주장) | https://arxiv.org/abs/2502.16528 · https://open-vox.github.io |
| SLIM-VDB | 2025 (arXiv, RA-L 채택) | OpenVDB 기반 실시간 확률 의미 맵, 코드 공개. 복셀 백엔드 후보이자 비교 대상. 인스턴스 여부 **미확인** | 미확인 / – / – / OpenVDB 의미 복셀 / 닫힌·열린 어휘 / 예(주장) | https://arxiv.org/abs/2512.12945 · https://github.com/umfieldrobotics/slim-vdb |
| IRIS-SLAM | 2026 (arXiv) | 단안 RGB로 인스턴스 임베딩 + 조밀 SLAM. "깊이 센서 없이 인스턴스 맵"이라는 점에서 경쟁 각도 | 단안 / 학습 / 자체 / 조밀 + 개방 어휘 / 인스턴스 / 주장 없음 | https://arxiv.org/abs/2602.18709 |
| Stereo Anything | 2024 (arXiv) | FoundationStereo 외의 범용 학습 스테레오. 깊이 모델 비교 후보 | 스테레오 / 학습 / – / 시차 / – / 미확인 | https://arxiv.org/abs/2411.14053 |

추가 검색에서 **나오지 않은 것**(공백 후보, 여전히 미확인): ① 폰 광각+초광각 + 파운데이션 스테레오 + VIO + 3D 맵, ② FoundationStereo 계열을 실시간 의미·인스턴스 TSDF에 직접 쓴 논문, ③ OpenVINS 기반 조밀 의미 매핑. 검색어: "smartphone dual rear cameras stereo visual-inertial dense mapping", "FoundationStereo SLAM TSDF semantic instance", "instance-aware voxel mapping YOLO stereo VIO".

---

## 5. 논문 각도에 주는 영향
- 차별점 문장에서 "학습 스테레오 + VIO + GPU 복셀"은 빼야 한다. Isaac ROS가 Fast-FoundationStereo + cuVSLAM + nvblox를 공식 지원하기 때문이다. 남는 차별점은 (a) **일반 안드로이드 폰의 이종 초단 기준선 렌즈**를 계측용 스테레오로 쓰는 것, (b) 그 조건에서 파운데이션 스테레오가 버티는지 정량화하는 것, (c) **인스턴스 ID 복셀 맵**까지 잇는 것, (d) 실시간과 녹화 후 오프라인의 두 경로다.
- "폰 광각+초광각 스테레오 깊이는 처음"이라고 쓰면 안 된다. iOS Dual Wide 깊이 API, SVD 데이터셋, MobiDepth/HiMoDepth가 있다. "**안드로이드 + 파운데이션 스테레오 + 맵**"으로 범위를 좁혀야 한다.
- 꼭 할 비교: MobiDepth식 고전 정합 대 Fast-FoundationStereo(같은 S24 렌즈 쌍), Kimera식 StereoBM 대 학습 스테레오, 거리별 깊이 오차 곡선(기준선 15.5 mm), 가능하면 iPhone Dual Wide 내장 깊이와의 비교(추정 제안).
