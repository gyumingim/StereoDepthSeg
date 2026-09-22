# YOLO + Fast FoundationStereo 객체 depth 추출 자료조사

조사일: 2026-09-21. 범위: seg / bbox 두 버전의 참고자료, 설계, 코드 연결 지점, 검증 계획.
초기 설계에 심화 조사를 추가했다. 검토 중 `lib_rectify.py`, `lib_ffs.py`, `z_object_depth.py`와 seg 처리가 작업공간에 추가되어 현재 소스도 검토했다. 이 조사 작업에서 모델 추론이나 애플리케이션 수정은 수행하지 않았다.

**현재 코드의 문제와 재현 결과는 [코드 검토 문서](doc_DEPTH_CODE_REVIEW.md)를 먼저 볼 것.** 정확한 대응점 탈락, mask 구멍 소실, 거리/OBB 불일치, 스케일 재보정, JSON 저장 실패를 수치로 확인했다. 아래 1~9절은 초기 설계와 기본 자료, 10절 이후는 추가 심화 조사다.

## 1. 권장 구성

**좌우 영상에서 dense disparity를 한 번 계산하고, 객체별 seg 마스크 / bbox로 같은 depth 맵을 집계한다.**
YOLO는 객체 영역을 찾고, Fast FoundationStereo는 좌우 대응으로 시차를 구한다. 미터 단위 거리는 카메라 캘리브레이션과 실제 baseline으로 환산한다.

두 버전은 다음과 같이 구분한다. 아래 집계 방식과 기본값은 이 프로젝트를 위한 설계 제안이며 논문에서 보장한 설정은 아니다.

| 항목 | seg 버전 | bbox 버전 |
|---|---|---|
| 기본 모델 후보 | `yolo11m-seg.pt` | `yolo11m.pt` |
| 객체 영역 | 인스턴스별 binary mask | 검출 사각형 내부 |
| 대표 depth | 유효 마스크 픽셀의 Z 중앙값 | 유효 bbox 픽셀의 Z 중앙값을 기준선으로 기록 |
| 장점 | 물체 형태를 따라 배경 유입 감소 | 단순하며 기존 검출기 활용 가능 |
| 실패 가능성 | 마스크 누락, 경계 오차, 얇은 물체 소실 | 배경이 과반이면 중앙값도 배경 거리 |
| 개선 실험 | 소량 erosion 전후 비교 | 중앙 ROI 또는 depth 군집을 별도 정책으로 비교 |

현재 프로젝트가 YOLO11을 사용하므로 같은 계열을 초기 후보로 둔다. 최신 모델이라는 이유만으로 교체하지 않는다.

**두 가지 비교 실험을 구분한다.**

1. 영역 선택 효과 비교: seg 모델의 같은 인스턴스에서 mask와 box를 함께 받아 두 depth를 계산한다. 검출 차이를 통제하고 stereo도 한 번만 실행한다.
2. 배포 구성 비교: detect 모델 + bbox와 seg 모델 + mask를 따로 실행한다. 검출률·지연시간까지 비교하고 인스턴스는 클래스와 IoU로 연결한다. 결과 배열 순서만으로 대응시키지 않는다.

Ultralytics seg 결과에는 인스턴스별 mask와 box가 제공된다. [공식 segmentation 설명](https://docs.ultralytics.com/tasks/segment/)

## 2. 먼저 읽을 자료와 활용 지점

| 우선순위 | 공식 자료 | 읽을 내용 / 프로젝트 활용 |
|---|---|---|
| 1 | [Fast FoundationStereo 저장소](https://github.com/NVlabs/Fast-FoundationStereo) | 설치, checkpoint, 정렬된 입력 규약, 실행 옵션 |
| 1 | [실제 run_demo.py 소스](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/run_demo.py) | 모델 로딩 → RGB 입력 → padding → forward → unpad → depth 계산 |
| 1 | [OpenCV calib3d](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) | `stereoRectify`, `initUndistortRectifyMap`, `reprojectImageTo3D` |
| 1 | [Ultralytics Predict](https://docs.ultralytics.com/modes/predict/) | 입력 색상 규약, `retina_masks`, `conf`, `classes`, `device` |
| 1 | [Ultralytics Results API](https://docs.ultralytics.com/reference/engine/results/) | `boxes.xyxy`, `boxes.cls`, `boxes.conf`, `masks.data`, `orig_shape` |
| 2 | [객체 마스크 추출 가이드](https://docs.ultralytics.com/guides/isolating-segmentation-objects/) | 마스크와 원본 영상을 결합하는 후처리 참고 |
| 2 | [Fast FoundationStereo 논문 v2](https://arxiv.org/abs/2512.11130v2) | 속도 개선 원리와 평가 조건 |
| 3 | [FoundationStereo 원 논문](https://arxiv.org/abs/2501.09898) | 기반 모델의 zero-shot stereo 접근 배경 |
| 3 | [공식 requirements](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/requirements.txt) | 기존 환경과의 의존성 차이 검토 |

저장소 조사 기준 commit: `476f4249561f7c79ca707326954f9255643412a6`.
README뿐 아니라 `scripts/run_demo.py` 실제 소스도 읽었다. 이동 가능한 master 링크 대신 위 commit 링크를 구현 기준으로 사용한다.

논문은 backbone distillation, cost filtering architecture search, refinement pruning을 통해 가속하는 접근을 설명한다. 논문의 속도 개선은 실험 조건에 따른 결과이며 이 프로젝트의 YOLO 포함 처리 속도를 의미하지 않는다. [논문](https://arxiv.org/abs/2512.11130v2)

## 3. 공식 예제에서 확인한 실제 동작

`run_demo.py` 기준:

- checkpoint 옆 `cfg.yaml`을 읽고, serialized model을 `torch.load(..., weights_only=False)`로 로딩한다.
- RGB 영상을 float CUDA tensor로 만든다. 입력을 32 배수로 padding한 뒤 추론하고 padding을 제거한다.
- 기본 경로는 `model.forward(..., iters=..., test_mode=True, optimize_build_volume='pytorch1')`다.
- `K.txt` 첫 줄은 K의 9개 원소, 둘째 줄은 baseline(m)이다. scale에 맞게 K를 조정한 뒤 depth를 계산한다.
- `depth_meter.npy` 저장은 `get_pc` 분기 안에 있다. `--get_pc 0`이면 depth 파일을 저장한다고 가정하면 안 된다.
- 예제는 출력 폴더를 삭제 후 재생성하고 GUI에서 대기한다. 따라서 프로젝트의 자동 처리에서는 필요한 추론 부분을 독립 어댑터로 옮기는 편이 적절하다.

근거: [run_demo.py](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/run_demo.py).
공식 serialized checkpoint와 그 코드 버전을 함께 고정해야 한다. 단순히 다른 stereo 모델의 `state_dict` 로더로 대체하면 안 된다.

## 4. 공통 파이프라인 설계

```text
L/R 원본 + K1,D1,K2,D2 + 실제 스케일의 R,T
  → 좌우 영상 전체 rectification
  → 동일한 크기로 resize / 내부 padding
  ├─ Fast FoundationStereo → disparity → 유효성 검사 → depth / XYZ
  └─ rectified left에서 YOLO → mask / bbox
  → 객체별 영역 ∩ 유효 depth
  → seg 통계 / bbox 통계 / overlay / JSON
```

### 4.1 Rectification과 좌표계

`stereoRectify`의 `R1,R2,P1,P2,Q`로 양쪽 remap을 구성한다. `CALIB_ZERO_DISPARITY`로 정렬 후 주점을 맞춘다. 이후 검출, 마스크, 시차가 모두 같은 rectified-left 좌표를 사용하도록 한다. OpenCV는 수직 stereo도 지원하지만 Fast FoundationStereo의 입력은 수평 epipolar line을 요구하므로 수직 배치는 별도 변환 없이 받지 않는 것이 초기 구현 범위다. [OpenCV](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html), [Fast FoundationStereo 입력 규약](https://github.com/NVlabs/Fast-FoundationStereo#run-demo)

원본 bbox 네 꼭짓점만 remap하여 직사각형으로 쓰면 왜곡 보정 후 실제 영역과 어긋날 수 있다. **정렬된 왼쪽 영상에서 YOLO를 실행하는 설계**가 좌표 변환을 단순화한다. 이것은 프로젝트 설계 판단이다.

보드 포즈는 원본 영상과 원래 K/dist로 먼저 구한다. 모델에 객체 crop만 넣거나 마스크 밖을 검게 만든 영상을 넣지 않고 전체 stereo 영상을 사용한다. crop은 대응 탐색 범위와 주변 문맥을 바꾸므로 첫 구현에서 제외한다.

### 4.2 Depth와 해상도

수평 정렬, 왼쪽 기준 양의 시차, 동일 주점 조건에서:

```text
d = u_left - u_right                  [px]
Z_rect = fx_rect * baseline_m / d     [m]
```

주점이 다르면 분모는 `d - (cx_left - cx_right)`다. 일반 처리는 정렬 과정에서 나온 Q를 사용하는 것이 안전하다. Q 재투영 결과는 rectified-left 좌표계다. 원래 왼쪽 카메라 좌표로 돌릴 때 열벡터 기준 `X_left = R1.T @ X_rect`를 사용한다. 기존 맵 좌표에는 그 뒤 `CAM_TO_MAP`을 적용한다. [OpenCV의 P/Q 및 R1 정의](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)

구현 시 지킬 규칙:

- 모델 입력을 가로 s배 축소하면 시차와 fx도 s배다. 원해상도로 시차를 복원할 때 공간 resize와 시차 값 `/s`를 모두 수행한다.
- 이미 미터로 계산한 depth는 resize하면서 거리 값에 배율을 곱하지 않는다.
- Q, K, bbox, mask는 계산에 사용하는 이미지 해상도에 맞춘다. padding은 출력에서 제거한다.
- `Z_rect`, `Z_left`, 카메라 중심까지의 유클리드 거리 `norm(X_left)`를 다른 필드로 취급한다. 회전이 있으면 정렬 전후 Z가 다르다.

로컬 수치 검증 완료: `fx=800px, baseline=0.1m, d=40px`에서 `Z=2m`. 절반 해상도의 `fx=400, d=20`에서도 `2m`. 프로젝트 venv의 OpenCV `stereoRectify` + `reprojectImageTo3D` 결과와 일치했다. 실제 모델 정확도 검증을 의미하지 않는다.

### 4.3 유효 픽셀 정의

설계 제안: `finite(d) & (d > epsilon)`에 거리 범위와 좌우 유효 영상 영역을 결합한다. 정렬 remap의 검은 테두리는 제외하고, 대응 좌표 `u_right=u_left-d`가 오른쪽 영상의 유효 영역에 있는지도 검사한다. 같은 위치의 좌우 ROI 교집합만으로 가려짐을 모두 판정할 수는 없다.

물리적인 occlusion과 오매칭은 이 검사만으로 모두 제거되지 않는다. 필요하면 별도의 좌우 일관성 검사를 추가하되 초기 버전에서 구현된 confidence처럼 표시하지 않는다. YOLO confidence는 객체 인식 점수이며 depth 신뢰도와 다르다.

## 5. seg 버전 상세

정렬된 왼쪽 영상을 `YOLO('yolo11m-seg.pt')`에 전달하고 `retina_masks=True`를 사용한다. 공식 문서는 이 경우 `masks.data`가 입력 원본 크기와 일치한다고 설명한다. 여기서 원본은 YOLO에 넘긴 rectified 영상이다. OpenCV ndarray는 BGR, stereo 예제는 RGB를 쓰므로 분기별 색상 변환이 필요하다. [Predict 문서](https://docs.ultralytics.com/modes/predict/)

후처리 제안:

1. `boxes`와 `masks.data`를 같은 인스턴스 인덱스로 연결한다. 전체 마스크를 합치지 않는다.
2. shape가 depth와 일치하는지 확인한다. letterbox가 남아 있는 마스크를 단순 stretch해서 맞추지 않는다.
3. 이진 마스크와 유효 depth를 교집합한다. 경계 혼합을 줄이는 erosion은 옵션으로 두고 작은 물체에서는 소실 여부를 확인한다.
4. 중앙값, p10/p90, MAD, 유효 픽셀 수와 비율을 저장한다.
5. 유효 샘플이 부족하면 `depth_m: null`과 실패 이유를 남긴다. bbox로 대체할 경우에도 명시적 정책과 `method`를 기록한다.

이 설계는 보이는 표면의 대표 깊이를 구한다. 객체 전체의 부피 중심이나 뒷면 두께를 복원했다는 뜻은 아니다.

## 6. bbox 버전 상세

`boxes.xyxy`를 정렬 영상 경계에 맞춰 자른 뒤 같은 depth 맵을 집계한다. 기존 `lib_detect.py`는 `(x,y,w,h)`를 반환하므로 인터페이스 간 변환을 명시한다. [Results API](https://docs.ultralytics.com/reference/engine/results/)

초기 기준선은 bbox 전체 유효 픽셀의 중앙값이다. 배경이 많이 들어오면 틀릴 수 있으므로 다음 정책을 별도 실험으로 비교한다.

| 후보 | 기대 효과 | 한계 |
|---|---|---|
| 전체 bbox 중앙값 | 단순하고 재현 가능 | 배경이 과반이면 배경 선택 |
| 중앙 ROI 중앙값 | 가장자리 배경 감소 | 중심이 구멍·가림·배경인 객체에 실패 |
| depth 군집 + 중앙부 지지 | 다중 깊이 중 대상 후보 선택 | 가까운 다른 물체가 있으면 오선택 |

가장 가까운 depth나 가장 큰 군집을 무조건 객체라고 판단하지 않는다. 선택한 정책, 포함 픽셀 수와 depth 분포를 결과에 남긴다. 기존 SIFT용 `expand_box()`는 특징점 확보 목적이므로 dense bbox 거리 계산에 그대로 적용하지 않는다.

## 7. 현재 프로젝트에 연결할 곳

| 현재 파일 | 조사 결과 | 구현 시 연결 방향 |
|---|---|---|
| `lib_calib.py` | `load_calib()`에서 K/dist/image_size 로드 | 캘리브레이션 해상도 검증에 재사용 |
| `z_reconstruct.py` | `load_pose()`에서 board/mono/stereo 분기 | 포즈 공급 로직을 공용 함수로 분리하는 방안 검토 |
| `lib_stereo.py` | 포즈 측정, SIFT 복원, 맵 좌표 변환 | 포즈 및 좌표 규약 재사용, dense 처리는 별도 함수 |
| `lib_detect.py` | YOLO bbox·polygon 기반 seg mask 반환, 모델 캐시 | mask 구멍 보존 문제를 수정하고 공통 인스턴스 형태로 제공 |
| `lib_rectify.py` | 공통 정류와 depth/XYZ 변환 추가됨 | 좌우 부호·수직 배치·출력 size 계약 검증 필요 |
| `lib_ffs.py`, `z_object_depth.py` | FFS 어댑터와 seg/bbox/both 진입점 추가됨 | 현재 코드 검토 및 CPU 재현 완료, 모델 추론 품질은 미검증 |
| `shots/L.png`, `shots/R.png`, `shots/meta.json` | 기존 촬영 입력 규약 | 첫 구현은 저장 영상 1쌍으로 재현 |
| `requirements.txt` | 현재 OpenCV/Open3D/loguru만 명시 | YOLO 및 FFS 의존성을 재현 가능한 별도 환경으로 정리 |

초기에는 `lib_depth.py`와 `z_object_depth.py`를 제안했지만, 현재 작업공간에는 `lib_ffs.py` + `lib_rectify.py` + `z_object_depth.py`로 구현이 분리되어 있다. 아래 옵션은 현재 진입점에 존재한다. venv_ffs 구성과 실제 모델 실행은 별도 검증이 필요하다.

```bash
python z_object_depth.py --mode seg
python z_object_depth.py --mode bbox
python z_object_depth.py --mode both   # seg 한 번의 결과로 mask/bbox 집계 비교
```

권장 출력: `depth_rect_m.npy`, `valid_mask.png`, `objects.json`, `overlay_seg.png`, `overlay_bbox.png`. JSON에는 `instance_id`, `class_id`, `label`, `det_conf`, `method`, `bbox_xyxy`, `depth_m`, `p10_m`, `p90_m`, `valid_count`, `valid_fraction`, `status`, `coordinate_frame`를 포함한다. NaN/Inf는 JSON 숫자로 쓰지 않는다. 해상도, K/P/Q/R1, 모델·코드 버전도 메타데이터에 저장한다.

카메라 1대를 움직여 촬영하는 기존 board/mono 방식은 정적인 장면에 한해 적용한다. 동적 객체의 실시간 거리를 구하려면 두 카메라의 촬영 시각을 맞춰야 한다. 두 버전 지원과 GPU 동시 실행은 별개다. 첫 구현은 모델을 메모리에 유지한 순차 실행으로 시작하고, 필요할 때 동시 실행의 VRAM과 latency를 측정한다.

## 8. 환경과 성능 실험

공식 README는 3090, 640×480 조건에서 `23-36-37`, 8회 refinement의 PyTorch 49.4ms / TensorRT 23.4ms를 제시한다. 이는 YOLO, 캡처, rectification, 후처리까지 포함한 현재 장치의 성능 보장이 아니다. `valid_iters`, 해상도, checkpoint를 바꿔 실측해야 한다. [공식 성능표](https://github.com/NVlabs/Fast-FoundationStereo#pretrained-models)

로컬 `STATUS.md`에는 RTX 4060과 설치 버전이 기록되어 있지만 이번 조사에서 GPU 가용성 및 FFS 호환성을 직접 검증하지 않았다. 공식 requirements에는 `opencv-contrib-python`이 있고 현재 프로젝트는 `opencv-python`을 사용한다. 동일 cv2 패키지 중복 설치를 피하도록 환경을 정리해야 한다. PyTorch/CUDA/Triton의 실제 조합은 공식 설치 절차와 checkpoint를 기준으로 확인한다. [requirements](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/requirements.txt)

설계상 초기 실험은 가로 640 정도에서 시작하되 화면 비율을 유지한다. 이후 원해상도와 비교한다. 첫 호출 컴파일 시간과 warm-up 후 시간을 구분하고 stereo / YOLO / 후처리 / 전체 latency 및 peak VRAM을 각각 기록한다. TensorRT는 PyTorch 경로의 정확도와 좌표 규약이 검증된 뒤 적용한다.

## 9. 구현 후 검증 항목

1. 합성 기하: 알려진 disparity에서 Z, resize 전후 거리, R1 역변환, 원래 카메라/맵 좌표를 확인한다.
2. 집계: 객체 1m / 배경 3m인 합성 depth로 seg와 bbox 차이, 배경 과반 실패, 빈 마스크, invalid depth 처리를 확인한다.
3. 실제 저장 영상: rectification 후 대응점의 세로 잔차와 마스크-depth 정렬을 수치 및 overlay로 확인한다.
4. 실측 거리: 여러 거리·질감·조명에서 대상의 보이는 표면까지 잰 기준값과 비교한다. bbox/seg MAE, 상대오차, 실패율과 유효 비율을 보고한다.
5. 비교 분리: 같은 seg 인스턴스의 mask/bbox 비교와 별도 detect/seg 모델 비교를 구분한다. 미검출 객체도 실패율에 포함한다.
6. 성능: 같은 해상도·refinement·장치로 두 구성을 측정한다. live 적용 시 촬영 시간 차이와 프레임 누적 지연도 기록한다.

완료 기준은 두 버전이 동일한 입력 쌍으로 실행되어 미터 단위 결과와 실패 상태를 저장하고, 기하 검증 및 실제 거리 대조 결과를 함께 제공하는 것이다. 현재 소스는 추가되었으나 검토에서 결함이 확인되었다. 모델 GPU 추론과 실제 거리 대조의 통과 여부는 이 조사에서 확인하지 않았다.

## 10. 논문을 읽고 실제 구현에 반영할 내용

### 10.1 논문의 confidence와 런타임 결과를 구분

Fast FoundationStereo의 pseudo-label 생성은 stereo와 monocular depth에서 만든 normal의 일치도를 검사해 학습 데이터를 선별한다. 이 학습 단계의 consistency mask가 일반 추론의 confidence 출력으로 제공된다는 뜻은 아니다. 또한 보충자료는 iteration을 늘렸을 때 개선이 포화되는 사례와 반투명 표면의 한계를 설명한다. [논문 §3.4·§8·§14](https://arxiv.org/html/2512.11130v2)

프로젝트 적용 판단: 투명 병의 YOLO 검출 confidence가 높아도 표면 거리를 잘 측정했다는 보장은 없다. `valid_fraction`, MAD, 좌우 일관성, 정류 검사 결과는 각각 다른 진단으로 저장한다. 이들을 검증 없이 합성한 수치를 “depth 확률”이라고 표시하지 않는다.

### 10.2 실제 forward의 입력·출력 계약

고정 commit의 [core/foundation_stereo.py](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/core/foundation_stereo.py#L191)를 직접 읽었다.

- PyTorch 경로는 RGB 0~255 입력을 내부 `normalize_image()`에서 ImageNet 정규화한다. 외부에서 YOLO tensor처럼 0~1로 바꾸거나 같은 정규화를 두 번 하면 입력이 달라진다.
- cost volume은 `max_disp//4` 수준에서 만들고 refinement 결과를 입력 해상도 disparity로 올린다. `test_mode=True`의 반환은 disparity 하나이며 confidence map은 없다.
- [single ONNX export](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/make_single_onnx.py#L120)는 normalization을 외부로 옮긴다. 따라서 같은 tensor 전처리 함수를 PyTorch/ONNX에 무조건 공유하면 안 된다.

권장 어댑터 계약은 `RGB uint8 pair + backend metadata → disparity_px + output_grid_metadata`다. 내부 전처리 차이를 어댑터에서 처리하되 출력 시차의 해상도와 단위를 고정한다.

### 10.3 공식 성능 수치를 재현할 때 확인할 차이

[profile_speed.py](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/profile_speed.py#L46)는 GPU의 640×480 랜덤 tensor, CUDA synchronize, warm-up 제외로 forward 시간을 측정하고 `optimize_build_volume='triton'`을 사용한다. 반면 [run_demo.py](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/run_demo.py#L87)와 현재 로컬 어댑터는 `pytorch1` 경로다.

따라서 속도 차이를 GPU 종류만의 영향으로 해석하지 않는다. kernel 경로, dtype, 실제 padding 후 크기, refinement 횟수, host↔device 이동, 동기화 범위도 함께 기록한다. 전체 앱 latency는 stereo forward 외에 YOLO·rectification·객체별 군집/OBB 시간을 포함해 따로 측정한다.

깊이 통계만 필요할 때는 모든 객체에 dense 점군 OBB를 계산하는 비용이 필요한지도 검토한다. 정렬된 depth 집계와 크기 복원을 별도 출력으로 두는 것이 단순하다. dense DBSCAN/OBB가 병목인지 여부는 아직 측정하지 않았다.

## 11. 깊이 정확도를 결정하는 기하 예산

아래 수치는 표준 `Z=fB/d`에서 직접 유도하고 로컬 계산으로 확인했다. 모델의 실측 정확도 예측은 아니다. f는 **추론 해상도에서 정류된 초점거리**, B는 실제 baseline이다.

```text
작은 시차 오차에 대한 1차 근사:
sigma_Z ≈ Z² / (f B) * sigma_d

독립적인 작은 오차를 가정한 상대오차 근사:
(sigma_Z/Z)² ≈ (sigma_f/f)² + (sigma_B/B)² + (sigma_d/d)²
```

카메라 회전 오차·왜곡 잔차·촬영 시간차는 편향이 될 수 있어 위 독립 잡음 근사만으로 설명되지 않는다. 고정 baseline이 틀리면 픽셀이 아무리 많아도 거리 스케일이 맞지 않는다. [OpenCV의 정류 투영식](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)

| 실제 Z | f=800px, B=0.1m의 시차 | sigma_d=0.5px 가정 시 sigma_Z |
|---:|---:|---:|
| 0.5m | 160px | 1.56mm |
| 1m | 80px | 6.25mm |
| 2m | 40px | 25mm |
| 3m | 26.67px | 56.25mm |
| 5m | 16px | 156.25mm |

가로 해상도를 절반으로 줄였을 때 동일한 “0.5 입력 픽셀 오차”를 가정하면 이 거리 오차는 두 배가 된다. 실제 모델 오차가 같은지는 별도 실험이다. 확대된 출력 픽셀 수를 독립 관측 수처럼 세어 신뢰도를 높이면 안 된다.

`max_disp=192` 선택도 baseline과 목표 근거리에서 판단해야 한다. f=800, B=0.1에서 192px에 해당하는 Z는 약 **0.417m**다. “192면 0.1m 근거리도 항상 충분” 같은 일반화는 불가하다. `fB/Z_near`로 필요한 탐색 범위를 계산해 여유를 둔다. refinement가 volume 범위 밖으로 이동할 수 있으므로 max_disp를 출력의 절대 물리 상한으로 단정하거나 무조건 clip하지 않는다.

seg mask를 erosion하면 경계 오염은 줄지만 얇은 객체가 사라지거나 보이는 표면의 대표값이 바뀔 수 있다. 거리 측정용 내부 mask와 크기 추정용 윤곽을 분리하고, erosion 전후 픽셀 수·거리 변화량을 검증하는 실험을 권장한다.

## 12. 가려짐·오매칭을 다루는 실제 참고 구현

[Meta Project Aria의 공식 stereo depth exporter](https://github.com/facebookresearch/projectaria_gen2_depth_from_stereo/blob/1d9e472adccd587dd75a821a01a997eeb9bbc315/export_depth_from_stereo.py#L317)는 FoundationStereo를 두 번 실행해 좌우 일관성을 검사한다. 두 번째는 **오른쪽/왼쪽을 교환하고 각각 가로 반전**해 양의 disparity 규약을 유지한다. 결과를 다시 반전한 뒤 왼쪽 disparity가 지목한 오른쪽 위치에서 두 disparity를 비교한다. 단순히 입력 순서만 교환하는 구현과 다르다.

프로젝트 적용 제안:

1. 먼저 `finite`, 양의 disparity, 양쪽 remap 유효영역, 거리 범위로 기본 유효성을 정한다.
2. 정확도 우선 오프라인 모드에서는 두 번째 추론으로 좌우 일관성을 계산한다. 임계값은 동일 해상도의 pixel 단위로 정의한다.
3. 샘플링 범위 밖·무효 disparity도 별도로 제외한다. 참고 구현의 정수 샘플링을 그대로 채택할지 subpixel 샘플링할지는 정확도/비용 실험으로 정한다.
4. 좌우가 같은 잘못된 대응을 만들 수도 있으므로 일관성 통과를 정답 보증으로 사용하지 않는다. 반사·투명체에서는 특히 실제 거리 검증이 필요하다.

대략 두 번의 stereo 추론이 필요한 정책이다. 기본 실시간 경로의 필수 단계로 먼저 넣기보다, 오프라인 정확도 평가에서 효과를 측정해 선택한다. 현재 로컬 코드는 이 검사를 구현하지 않았다.

## 13. 어떤 자료로 무엇을 평가할 것인가

| 자료 | 도움이 되는 검증 | 이 프로젝트에서 추가해야 할 것 |
|---|---|---|
| [Middlebury stereo evaluation](https://vision.middlebury.edu/stereo/eval3/) | 조밀 시차 오차, invalid 영역, 해상도별 성능 | YOLO 인스턴스 기준 실제 거리 오차 |
| [Booster 공식 데이터셋](https://cvlab-unibo.github.io/booster-web/) | 반사/투명 재질, material mask, 서로 다른 해상도 카메라 조건 | 현재 웹캠+폰 리그의 별도 촬영·캘리 검증 |
| [Fast FoundationStereo 논문](https://arxiv.org/html/2512.11130v2) | zero-shot 성능과 iteration/속도 trade-off | 목표 객체의 미터 오차·미검출·실패율 |
| 직접 촬영한 거리 기준 장면 | 전체 측정 파이프라인의 단위·스케일·시각차 검증 | 기준점 정의와 별도 검증 물체 유지 |

Booster는 투명/반사 표면과 unbalanced stereo를 다루므로 웹캠+폰의 이질적인 영상 조건을 시험하는 자료로 특히 관련성이 높다. 해당 데이터셋 통과가 현재 리그의 calibration 정확도를 보장하지는 않는다. [데이터셋 설명](https://cvlab-unibo.github.io/booster-web/)

권장 실험은 서로 다른 오류 원인을 분리한다:

| 실험 | 고정하는 것 | 바꾸는 것 | 판별할 문제 |
|---|---|---|---|
| A | 수동 정답 mask/box, 같은 이미지·calibration | stereo 설정 | YOLO 오류를 제외한 depth 품질 |
| B | seg의 동일 인스턴스, 같은 disparity | mask vs 그 bbox | 영역 선택이 배경 혼입에 미치는 영향 |
| C | 같은 이미지·stereo 설정 | detect 모델 vs seg 모델 | 배포 구성의 검출률·depth·속도 |
| D | 대상·calibration·모델 | 해상도/iteration/backend | 속도와 거리 정확도의 trade-off |
| E | 모델/설정 | 재질·거리·가림·조명·동작 | 실제 실패 조건 |

거리별 MAE/중앙 절대오차/p95, 상대오차, 객체 실패율, valid_fraction, latency p50/p95를 남긴다. 성공한 객체만 평균내면 실패를 숨기므로 미검출과 invalid 결과를 분모에서 제거하지 않는다. 동일 객체의 많은 프레임을 독립 표본으로 취급하지 말고 장면/대상별 결과도 구분한다.

거리 기준은 관측 가능한 앞면의 광축 Z인지, 특정 표면점까지의 직선거리인지 먼저 정한다. `median(Z)`, `norm(median(XYZ))`, 객체 전체 중심까지의 거리는 일반적으로 같지 않다. 스케일 보정에 쓴 물체는 별도의 검증 대상으로 세지 않는다.

## 14. 이번 심화 조사의 근거와 범위

- 하네스를 읽고 실제 소스 → 수치 재현 → 수정 제안 순서로 검토했다.
- GitHub MCP `search_code/get_file_contents`는 이 세션에 없었다. GitHub REST/raw와 웹 도구로 공식 소스를 직접 읽었다. MCP를 사용했다고 기록하지 않는다.
- 외부 FFS 기준은 commit `476f4249561f7c79ca707326954f9255643412a6`; 로컬 `tools/Fast-FoundationStereo`의 HEAD도 이 값임을 확인했다. Project Aria 기준은 `1d9e472adccd587dd75a821a01a997eeb9bbc315`다.
- 메인 venv에서 Ultralytics 8.4.157, OpenCV 4.13.0.92, Open3D 0.20.0을 확인했다. 별도 FFS 환경은 검토 중 변경되고 있어 설치/추론 완료를 주장하지 않는다.
- [코드 검토](doc_DEPTH_CODE_REVIEW.md), [재현 스크립트](research/depth_review_20260921/reproduce.py), [관측 수치와 해시](research/depth_review_20260921/observed_results.json)를 함께 저장했다.

## 15. 스마트폰 한 대의 후면 두 카메라 사용 가능성 — 2026-09-22 정정

**노트북 카메라는 FFS에 필수가 아니다.** 초기 프로젝트의 웹캠 촬영 + 폰 체커보드 구성을 확장해 웹캠/폰 리그를 검토했으나, 폰 내부 두 카메라의 물리 스트림 접근을 먼저 확인했어야 했다.

기존 STATUS의 독립 동시 오픈 조합 `{0,1}`, `{0,3}`만으로 “초광각은 어떤 카메라와도 동시 사용 불가”라고 결론 내릴 수 없다. Android는 서로 다른 CameraDevice를 동시에 여는 경로와 **하나의 logical CameraDevice/session에 두 physical output을 지정하는 경로**를 구분한다. 후자는 `getPhysicalCameraIds()`로 조합을 조회하고 `OutputConfiguration.setPhysicalCameraId()`로 출력 대상을 설정한다. [Android 공식 multi-camera 예제](https://developer.android.com/media/camera/camera2/multi-camera), [독립 동시 오픈 API의 범위](https://developer.android.com/reference/android/hardware/camera2/CameraManager#getConcurrentCameraIds())

이번에 연결된 실제 기기를 읽기 전용으로 조회한 결과:

- 모델 `SM-S921N`(Galaxy S24), Android 16.
- 후면 logical camera 0: `LOGICAL_MULTI_CAMERA` capability와 physicalIds 항목 존재.
- 동기화 metadata: `APPROXIMATE`. 정밀한 노출 동기화가 확인된 상태로 취급하면 안 된다.
- [메타데이터 발췌](research/depth_review_20260921/phone_capabilities_20260922.txt) 저장. dumpsys의 physicalIds 출력은 byte 문자 표시이므로 정확한 ID 목록·렌즈 대응은 Camera2 API로 읽어 확인할 것.

현재 카메라 세션을 중단하거나 새 카메라를 열지 않았다. 이 결과는 **듀얼 물리 스트림 경로를 검증할 근거**이며, 두 스트림 추출 성공이나 목표 해상도/FPS 지원의 실증은 아니다.

다음 검증은 작은 Camera2 앱에서 후면 logical camera를 한 번 열고, 같은 크기의 YUV ImageReader 두 개를 각각 물리 카메라에 연결하는 것이다. 두 출력의 실제 렌즈, frame timestamp, 누락·skew, 지원 해상도를 확인한 뒤 PC로 전송한다. FFS/YOLO는 PC GPU에서 처리할 수 있으므로 노트북의 **연산 자원 사용**과 **노트북 카메라 사용**을 구분한다.

후면 두 렌즈가 같은 장면을 볼 수 있어야 하며, 렌즈 간격과 양쪽 K/dist/R/T를 측정한다. 폰의 짧은 baseline은 먼 거리의 시차를 작게 만들므로 목표 거리에서 정확도를 검증해야 한다. 동시 프레임 획득이 성공하면 그 입력을 기존 seg/bbox 공통 depth 파이프라인에 연결하는 방향이 우선이다.

## 16. Galaxy S24 후면 두 카메라: 웹 근거 재검증 — 2026-09-22

### 16.1 삼성 기본 앱은 후면+후면 촬영과 개별 파일 저장을 지원한다

삼성 CamCyclopedia의 공식 운영자 설명은 S24·S24+·S24 Ultra의 Dual Recording에서 **후면 두 렌즈 조합**을 지원하고, 저장 옵션에 따라 **렌즈별 MP4 두 개**를 얻을 수 있다고 명시한다. 따라서 “S24는 후면 두 카메라를 동시에 추출할 수 없다”는 포괄적인 판단은 잘못이다. 해당 안내의 UHD 지원은 S24 Ultra에 한정되어 있으므로 연결된 일반 S24에 그대로 적용하지 않는다. [삼성 Dual Recording 설명](https://r1.community.samsung.com/t5/camera-camcyclopedia/dual-recording/ba-p/27413620)

삼성 S24 사용 안내의 진입 순서는 카메라 → 더보기 → 듀얼 레코딩 → 렌즈 선택이다. 선택 항목은 전면·초광각·광각·망원이며, 이번 목적은 같은 장면을 보는 후면 두 렌즈다. 실제 메뉴 위치는 설치된 One UI 버전에 따라 확인한다. [삼성 S24 사용 안내](https://www.samsung.com/us/support/answer/ANS10000932/)

이 근거가 확인해 주는 것은 **기본 앱의 촬영·분리 저장 기능**이다. 두 파일이 정밀하게 노출 동기화되거나, 다른 앱에 두 개의 실시간 원본 프레임을 제공한다는 보장은 아니다. 이번 조사에서는 직접 듀얼 촬영하거나 파일을 추출하지 않았다.

### 16.2 외부 앱의 실시간 입력: Camera2와 현재 CameraX 모두 조사 대상

Camera2 공식 문서는 logical camera 하나를 열고 같은 세션에서 각각 physical ID를 지정한 두 출력으로 받는 방식을 설명한다. `getPhysicalCameraIds()`로 실제 그룹을 찾고 `OutputConfiguration.setPhysicalCameraId()`를 지정한다. 같은 종류·크기의 YUV/RAW 스트림 치환 조건을 따르고, 해당 기기의 스트림 구성 지원을 확인해야 한다. [Android multi-camera 문서와 Kotlin/Java 예제](https://developer.android.com/media/camera/camera2/multi-camera)

현재 CameraX 공식 API에도 `CameraSelector.Builder.setPhysicalCameraId()`가 있으며 **1.4.0에 추가된 API**로 표시된다. 같은 logical camera에 속하는 두 physical camera를 서로 다른 selector로 구성할 수 있다. 다만 공식 문서는 기기별 성공을 보장하지 않으며, 구성 미지원 시 bind 과정에서 `IllegalArgumentException`이 발생할 수 있다고 명시한다. [CameraX API](https://developer.android.com/reference/androidx/camera/core/CameraSelector.Builder)

AndroidX의 [LifecycleCameraProvider 소스 계약](https://android.googlesource.com/platform/frameworks/support/+/6d4c8a73db3f05daca142904394afb1b00a3e95d/camera/camera-lifecycle/src/main/java/androidx/camera/lifecycle/LifecycleCameraProvider.kt)도 physical camera별 `SingleCameraConfig`와 동일한 `LifecycleOwner`를 사용하도록 설명한다. 이는 구현 참고 근거이며 S24 실기기에서 실행한 결과는 아니다.

검색 중 발견한 2024-09-05 [CameraX 개발자 답변](https://groups.google.com/a/android.com/g/camerax-developers/c/M3nCVRT59TU/m/WP26TxPIAQAJ)은 당시 S24 Ultra 기본 앱과 공개 API의 차이를 지적한다. 이 과거 답변을 근거로 **현재 CameraX 전체가 후면 두 물리 카메라를 지원하지 않는다**고 결론 내리면 현재 API 문서와 충돌한다. 반대로 현재 API가 존재한다는 사실만으로 SM-S921N의 원하는 렌즈·해상도 조합이 된다고 확정해서도 안 된다.

### 16.3 이 프로젝트에서 선택할 수 있는 경로

| 경로 | 확인된 근거 | 장점 | 남은 검증 |
|---|---|---|---|
| 기본 앱 Dual Recording → MP4 두 개 → PC 처리 | 삼성 S24 공식 지원 안내 | Android 앱을 만들기 전에 두 렌즈 데이터 확보 가능 | 실제 분리 저장, 시간 정렬, 같은 촬영 설정의 calibration |
| Camera2 앱 → 두 YUV 출력 → PC 전송 | Android 공개 API 및 이 폰의 logical capability | 프레임 timestamp·capture metadata를 다루는 구조 설계 가능 | 물리 ID·렌즈 대응, session 성공, 실제 FPS·skew·누락 |
| CameraX 앱 → 두 physical selector | 현재 CameraX 공식 API | lifecycle·use case 관리 활용 가능 | 기기 지원과 필요한 분석 출력 조합, timestamp 처리 |

프로젝트 관점의 제안은 **저장 영상으로 먼저 stereo 입력을 검증하고, 실시간 요구는 Camera2 최소 실험으로 확인**하는 것이다. Camera2를 우선 검토하는 이유는 depth 입력에서 물리 카메라별 timestamp와 calibration 관련 metadata를 직접 확인하려는 목적이다. CameraX가 원천적으로 불가능하다는 뜻은 아니다.

### 16.4 기존 코드에 반영해야 할 설계상 수정점

아래는 이번 조사에 따른 수정 제안이며 아직 구현하지 않았다.

1. **입력 장치를 고정하지 않기:** 노트북 웹캠+폰만 전제하지 말고 폰 후면 A/B의 영상 쌍도 기존 seg/bbox 공통 처리에 공급한다. FFS와 YOLO의 PC GPU 실행은 유지 가능하다.
2. **동시 지원 판정 수정:** `getConcurrentCameraIds()`의 독립 CameraDevice 조합만으로 후면 두 물리 출력이 불가능하다고 판정하지 않는다. logical camera 내부 출력과 구분한다. [CameraManager API](https://developer.android.com/reference/android/hardware/camera2/CameraManager#getConcurrentCameraIds())
3. **두 번 read했다고 동기 프레임으로 취급하지 않기:** 저장 영상은 각 파일의 PTS·offset·누락을 확인하고, 라이브 입력은 sensor timestamp 기반으로 짝을 구성한다. 같은 프레임 번호나 같은 PC 수신 시각만으로 노출 동시성을 보장할 수 없다. 앞서 실기기에서 관측한 sync는 `APPROXIMATE`다.
4. **새 렌즈 쌍으로 재보정:** 기존 웹캠/폰 K·왜곡·R/T·baseline을 재사용하지 않는다. 촬영 해상도, crop, 손떨림 보정 상태를 포함해 실제 출력 기준으로 보정하고 정류 후 수직 오차를 확인한다. 저장 영상과 YUV 출력의 보정값도 자동으로 동일하다고 가정하지 않는다.
5. **파일 분리 성공과 depth 성공을 별도로 판정:** MP4 두 개를 얻은 뒤에도 공통 시야·정류·유효 disparity·실제 거리 오차 검증이 필요하다. seg는 mask 내부, bbox는 배경 혼입을 처리한 영역에서 같은 depth 맵을 집계한다.

이번 웹 조사의 확정 결과는 **S24 기본 앱의 후면 두 영상 분리 저장 지원**과 **외부 앱을 위한 공개 API 경로 존재**다. 연결된 SM-S921N의 실시간 두 스트림 획득 성공은 아직 확인하지 않았다.

## 17. 사용자 확정 요구: 폰 후면 3개 → PC 표시·추론

사용자 설명으로 “스마트폰만 사용”의 의미를 정정한다. **촬영 장치는 스마트폰 한 대이며, PC 연결과 PC에서의 추론을 사용한다.** 스마트폰 단독 추론 요구가 아니다. 목표는 S24의 초광각·광각·망원 영상을 PC에 동시에 표시하고, PC에서 YOLO seg/bbox 및 Fast FoundationStereo를 실행하는 것이다.

```text
Galaxy S24: 후면 초광각 / 광각 / 망원
    → 각 렌즈의 프레임과 촬영 timestamp 전송
    → PC: 영상 3개 동시 표시
    → 선택한 두 렌즈의 시간 정렬·스테레오 정류
    → Fast FoundationStereo의 공통 depth + YOLO seg/bbox별 객체 깊이
```

**후면 3개 동시 출력은 아직 미검증이다.** 삼성 Dual Recording의 2개 지원을 3개 지원의 근거로 확대하지 않는다. 현재 [CameraX selector 문서](https://developer.android.com/reference/androidx/camera/core/CameraSelector.Builder)는 같은 logical camera의 물리 카메라 두 개까지 허용한다고 명시한다. 따라서 3개를 요구하는 검증은 Camera2에서 수행하는 방향으로 설계한다. [Camera2 multi-camera 문서](https://developer.android.com/media/camera/camera2/multi-camera)의 최소 보장도 두 물리 스트림 치환이며, 추가 출력은 기기별 검증 대상이다. “출력 스트림 3개”라는 일반 문구를 “서로 다른 센서 3개의 동시 촬영”과 동일하게 해석하면 안 된다.

구현 전 확인할 구체적인 항목:

1. Camera2 API로 후면 logical group의 정확한 physical ID와 초광각·광각·망원 대응을 확인한다.
2. 세 렌즈가 같은 logical group으로 노출되는 경우 physical output 세 개의 session 구성 가능 여부와 실제 프레임 수신을 검사한다. 공통 지원 저해상도에서 시작하고, 개별 프레임 timestamp·FPS·누락을 기록한다. metadata나 session 지원 조회만으로 성공 판정하지 않는다.
3. 성공한 출력 구성을 USB를 통한 PC 수신과 3분할 표시에 연결한다. 각 스트림의 렌즈 ID·촬영 시각을 보존한다.
4. FFS는 보정된 두 영상으로 추론한다. 세 영상을 한 번에 입력하는 모델로 취급하지 않는다. 세 번째 카메라 표시와 추가 stereo pair 추론은 별개이며, pair별 추론은 추가 연산·보정이 필요하다. [FFS 공식 구현](https://github.com/NVlabs/Fast-FoundationStereo)

기기에서 3개 출력이 거부되면 해당 설정의 실패 근거를 기록하고 원인을 확인해야 한다. 두 카메라만 띄우거나 렌즈를 번갈아 전환하는 방식은 사용자의 “3개 동시 표시” 요구를 충족한 것으로 처리하지 않는다. 이번 정정은 요구사항과 검증 설계를 문서화한 것이며, Android 앱·전송·PC 3영상 표시는 아직 구현하지 않았다.

## 18. 후속 실기기 결과: 2개와 3개 모두 동시 수신 성공

사용자가 “2개를 먼저 하고 3개는 되면 시도”로 우선순위를 정하고 코드 작성·실행을 승인한 뒤, Camera2 Android 앱과 PC USB 수신·표시 코드를 구현했다. **15~17절의 미검증 상태 중 다중 영상 수신은 아래 실측으로 해소됐다.**

- 실제 Camera2 조회: logical 0 → physical 5(광각), 2(초광각), 6(망원). dumpsys byte 문자열만으로 추정했던 ID와 달리 API 문자열을 직접 확인했다.
- 후면 2개: 640×480·15 FPS, 15초간 각각 217프레임·217쌍.
- 후면 3개: 640×480·15 FPS, 12초간 171/172/171프레임·171묶음.
- 후면 2개 + PC GUI + YOLO seg/bbox: 640×480·30 FPS, 25초간 736/737프레임·735쌍. GPU `cuda:0`에서 실행.
- bbox 전용 모델도 FFS용 Python 환경에서 실행해 20초·582쌍, 447회 추론 중앙값 11.26ms 확인. FFS 추론을 포함한 수치가 아니다.

전송은 압축 MP4가 아닌 NV21 프레임과 Image timestamp를 ADB TCP 포워딩으로 보낸다. PC에서 BGR 변환 후 표시하고, 원본 쌍을 PNG로 저장한다. 수신 thread와 추론 worker를 분리해 추론 때문에 오래된 영상을 계속 대기열에 쌓지 않는다. 보고 timestamp 차이는 0ms였으나 실제 sensor sync 등급은 여전히 `APPROXIMATE`다.

현재 사용 가능한 명령과 구현 파일은 [Phone Stereo 실행 안내](phone_stereo/README.md), 실측은 [검증 기록](phone_stereo/verification_20260922.json)에 저장했다. `--calib-stereo`로 FFS와 기존 seg/bbox 깊이 집계에 연결할 수 있도록 구현했지만, **폰 두 렌즈의 보정 파일이 없어 해당 depth 경로는 아직 실기기 검증하지 않았다.** 동시 영상·YOLO 성공을 미터 거리 정확도 성공으로 해석하지 않는다.
