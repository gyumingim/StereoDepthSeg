# 객체 depth 코드 검토 및 수정 제안

검토일: 2026-09-21. [하네스](HARNESS.md)를 전체 읽고, 현행 코드와 공식 소스를 확인한 결과다.
범위: 기존 SIFT 복원, 새 정류/FFS 어댑터와 seg/bbox 집계, 캘리브레이션 스케일, 결과 저장.
애플리케이션 코드는 이 검토에서 수정하지 않았다. 변경 산출물은 조사 문서와 독립 재현 자료다.

작업 도중 `lib_ffs.py`, `z_object_depth.py`가 추가되어 검토에 포함했다. 이어 dense 크기 계산이 `fit_box()`에서 `planar_box()`로 변경되어 재검증했다. [초기 관측](research/depth_review_20260921/observed_results.json)과 [최종 재검증](research/depth_review_20260921/rechecked_results.json)에 각각 SHA-256을 기록했다. 아래 dense 결과는 두 시점을 구분한다. 이후 수정된 파일은 해시로 확인해야 한다.

## 1. 먼저 수정할 순서

| 우선순위 | ID | 문제 | 영향 |
|---|---|---|---|
| P1 | R01 | 회전·다른 K를 지원하는 경로에 평행 stereo 필터 적용 | 정확한 대응점 전부 탈락 가능 |
| P1 | R02 | mask → 외곽 polygon → fillPoly에서 구멍 소실 | seg에 배경이 다시 들어와 객체 거리 오류 |
| P1 | R03 | 거리와 OBB가 서로 다른 점 집합에서 계산됨 | 한 객체 결과에 다른 표면의 거리·치수 혼합 |
| P1 | R04 | 임시 baseline을 실측값으로 기록하고 stale 결과로 재보정 | 미터 스케일의 근거 소실·중복 보정 |
| P1 | R05 | 정류 baseline의 절댓값으로 좌우 방향을 숨김 | 뒤집힌 영상/수직 배치에서 그럴듯한 잘못된 깊이 |
| P1 | R06 | 검사 FAIL을 출력한 뒤 정상 결과처럼 계속 처리 | 잘못된 포즈·재투영 상태가 후속 결과에 반영되지 않음 |
| P2 | R07 | 정류 검사 불가 시 NaN JSON 저장 예외 | 전체 추론 후 최종 결과 파일 저장 실패 |
| P2 | R08 | 오른쪽 ROI의 y 범위 누락 | 오른쪽 유효 영상 밖 대응점도 통과 |
| P2 | R09 | 좌우 프레임을 순차 read하고 오른쪽을 다시 read | 동적 장면에서 시차와 시간차 혼합 |
| P2 | R10 | 정류 size 인자가 입력/출력 해상도를 혼용 | resize 의도와 다른 crop 및 K 생성 |
| P2 | R11 | 성공 객체만 저장, 전부 실패하면 이전 results 유지 | 이전 장면 결과를 현재 값으로 사용할 수 있음 |
| P2 | R12 | 추론·출력 인터페이스 및 비교 정책 미완성 | 재현성과 seg/bbox 비교 해석 저하 |

P1은 측정 결과를 사용하기 전에 해결할 정확도 문제, P2는 해당 입력·실행 조건에서 수정할 문제다. 아래에 재현한 버그와 정적 검토에서 확인한 결함을 구분했다.

## 2. 재현된 주요 결함

### R01 — 일반 두 뷰에 `abs(yL-yR)<2px`와 원영상 x시차를 적용한다

위치: [lib_stereo.py:315](/home/karma/camera/lib_stereo.py:315), [lib_stereo.py:334](/home/karma/camera/lib_stereo.py:334).

`_to_normalized()` 후 다시 각 카메라 K로 투영한 uL/uR은 왜곡만 제거된 좌표다. **정류 좌표가 아니다.** `R,T`를 받는 `reconstruct()`가 회전과 다른 내부 파라미터를 지원하려면, 원영상의 y좌표 차이와 x좌표 차이를 평행 stereo의 시차로 해석하면 안 된다.

정확한 합성 3D 점을 두 카메라에 투영해 `_match_sift` 결과만 주입하고, 실제 `reconstruct()`를 실행했다. 매칭 오차를 제거해 기하 코드만 검증한 실험이다.

| 조건 | 정확한 매칭 수 | 현행 epipolar 통과 | depth 통과 | 정상 삼각측량 최대 3D 오차 |
|---|---:|---:|---:|---:|
| 동일 K, pitch 3° | 100 | 0 | 0 | 약 6.2e-15m |
| R=I, fx/fy 800 vs 520 | 100 | 12 | 12 | 약 7.5e-15m |
| yaw 약 2.86°, 실제 Z=2m | 30 | 30 | 0 | 약 3.6e-15m |

세 번째는 회전에 의한 영상 이동이 baseline에 의한 x이동을 상쇄하는 사례다. 원영상 x차이가 1px 미만이어도 삼각측량 가능한 점을 시차 하한에서 버린다.

수정 제안: 희소 경로는 기존 R,T로 `E=[T]×R`, `F=K_R^-T E K_L^-1`를 만들고, 왜곡 보정 좌표에서 Sampson/epipolar 거리로 검증한다. 또는 대응점을 공통 정류 좌표로 옮긴 뒤 검사한다. 가까운 평행 광선 판정은 실제 삼각측량 각도나 정류 시차로 분리하고, 양쪽 카메라에서 Z>0인지 확인한다. 단순히 2px 제한을 늘리는 수정은 원인을 해결하지 못한다. [OpenCV 기하 및 Sampson distance](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)

### R02 — segmentation mask의 구멍이 사라진다

위치: [lib_detect.py:87](/home/karma/camera/lib_detect.py:87), [lib_detect.py:96](/home/karma/camera/lib_detect.py:96).

설치된 Ultralytics 8.4.157의 `Masks.xy`는 `masks2segments()`를 호출한다. 이 함수는 `RETR_EXTERNAL`로 외곽 윤곽만 얻는다. 현행 `fillPoly()` 재구성은 구멍을 복원하지 못한다. [공식 함수 소스](https://docs.ultralytics.com/reference/utils/ops/#ultralytics.utils.ops.masks2segments)

실제 `Results` 객체와 현재 `detect_yolo()`를 사용하고, 신경망 반환값만 합성 mask로 대체해 재현했다:

- 80×80 바깥 영역에서 60×60 구멍을 뺀 mask: 실제 객체 2,800px.
- 현행 변환 후: 6,400px. 배경 3,600px가 객체로 추가됨.
- 객체 Z=1m, 구멍 너머 배경 Z=3m일 때 중앙값: **1m → 3m**.

수정 제안: `retina_masks=True`와 `masks.data`를 사용하고 depth 좌표계/shape를 검사한다. polygon은 표시용으로만 사용한다. 원본 해상도로 바꾼다고 네트워크가 놓친 구조까지 복구되지는 않지만, 이미 예측한 구멍은 보존해야 한다. [retina_masks 규약](https://docs.ultralytics.com/modes/predict/)

`lib_stereo.reconstruct()`의 기본 8px dilation도 별도 측정 영역으로 구분해야 한다. 특징점 탐색용 확장 영역을 dense 객체 거리 집계에 그대로 쓰면 seg의 배경 분리 효과를 약화시킨다.

### R03 — 같은 결과 레코드의 거리·중심·치수가 서로 다른 표면을 가리킨다

위치: [lib_stereo.py:348](/home/karma/camera/lib_stereo.py:348), [lib_stereo.py:360](/home/karma/camera/lib_stereo.py:360), [z_reconstruct.py:171](/home/karma/camera/z_reconstruct.py:171), [z_object_depth.py:73](/home/karma/camera/z_object_depth.py:73).

희소 경로는 DBSCAN 전 점군의 Z 중앙값을 `distance_m`으로 저장하고, DBSCAN 후 점군으로 OBB를 구한다. 최초 dense 경로도 원영역 전체의 depth·center와 `fit_box()`가 남긴 점 집합의 치수를 섞었다. 최종 재검증 시 dense는 `planar_box()` 중심·치수로 변경되었지만 Z 통계는 여전히 전체 영역에서 계산한다.

실제 `fit_box()`를 사용한 합성 실험에서 2m의 조밀한 객체점 50개와 약 4m의 분산된 배경점 60개를 넣었다:

- 군집 선택 후 남은 50개 점의 깊이: **2.00005m**.
- 저장용 필터 전 Z 중앙값: **3.85108m**.
- 최초 dense 결과도 `status=ok`, 중심 약 3.85m, 치수는 2m 군집의 23.4×14.4×2.9mm로 구성됨.
- 변경 후 dense는 같은 입력에서 배경 60개를 남겨 중심 약 **4.0103m**, 치수 **1679.1×1354.7×367.1mm**를 반환했다. `mixed_depth=true`가 추가됐지만 상태는 여전히 `ok`, Z 통계는 **3.85108m**다. 평면 적합으로 변경한 것만으로 객체 선택 문제가 해결되지는 않았다.

수정 제안: 최종 선택 픽셀/점 인덱스를 유지해 depth, center, 치수를 같은 집합에서 계산한다. bbox 전체 중앙값을 비교 기준선으로 남기고 싶다면 `roi_depth_raw`와 `selected_object_depth`를 분리하고 선택 정보도 저장한다. 객체 후보를 확정할 수 없는 혼합 분포는 별도 상태로 표시한다. `fit_box()`와 `planar_box()` 모두 선택 인덱스를 돌려주는 인터페이스가 필요하다.

### R04 — 임시 스케일과 재보정의 출처가 보존되지 않는다

위치: [z_stereo_pose.py:125](/home/karma/camera/z_stereo_pose.py:125), [z_stereo_pose.py:185](/home/karma/camera/z_stereo_pose.py:185), [z_stereo_pose.py:101](/home/karma/camera/z_stereo_pose.py:101).

`--provisional`은 임의 baseline으로 `_save(..., known=None)`을 부르는데, `_save()`는 이를 `essential+measured-baseline`으로 저장한다. 콘솔의 “스케일 미확정” 상태가 파일에는 남지 않는다. 이후 dense 경로도 이 파일의 T를 그대로 m로 사용한다. 이 저장 함수 호출은 임시 파일에서 재현했다.

또한 `--rescale`은 `results.json`을 만든 baseline과 현재 calibration을 연결하지 않는다. baseline=0.1m, 저장 추정거리=2m, 실측=1m에서 같은 명령을 결과 재생성 없이 반복하면 **0.1 → 0.05 → 0.025m**로 두 번 보정된다. 임시 파일로 실제 `cmd_rescale()` 동작을 재현했다.

수정 제안: `scale_status=provisional|measured|reference_scaled`, 원본 T, 기준 측정의 의미(Z/직선거리), calibration ID와 결과의 calibration ID를 기록한다. 보정은 결과 생성 시점 baseline에서 계산하고 오래된 결과를 거부한다. 임시 스케일 출력은 metric 확정 상태로 노출하지 않는다.

추가 정적 결함: `recoverPose()`의 인라이어 수 `n_in`을 출력하지만 저장 중단 조건으로 사용하지 않는다([z_stereo_pose.py:172](/home/karma/camera/z_stereo_pose.py:172)). 평면비만 낮다고 포즈가 신뢰 가능해지는 것은 아니다. 샘플 수·비율·양안 전방성·삼각측량 각도를 별도 진단해야 한다.

### R05 — 좌우 뒤집힘과 수직 stereo에서 scalar depth와 Q가 모순된다

위치: [lib_rectify.py:44](/home/karma/camera/lib_rectify.py:44), [lib_rectify.py:62](/home/karma/camera/lib_rectify.py:62).

`abs(P2[0,3]/f)`는 방향을 지우고, 수직 stereo에서는 baseline을 0으로 만든다. 동일 K, f=800, baseline=0.1m, 입력 disparity=+40px로 실제 함수를 비교했다:

| 배치 | 현재 scalar depth | Q 재투영 Z | 해석 |
|---|---:|---:|---|
| 정상 T=(-0.1,0,0) | 2m | 2m | 일치 |
| 역순 T=(+0.1,0,0) | 2m | -2m | 양의 시차가 해당 순서에 물리적으로 맞지 않음 |
| 수직 T=(0,-0.1,0) | 0m | 2m | Q는 세로 시차를 처리하지만 FFS는 가로 시차 모델 |

수정 제안: FFS 입력 경계에서 수평 배치·기대 baseline 부호·0이 아닌 baseline을 검사한다. 역순 지원 시 영상, K, R,T와 기준 카메라 좌표를 함께 변환한다. 단지 disparity나 baseline에 abs를 적용하면 안 된다. Q를 사용해도 네트워크 입력의 좌우 순서 오류는 해결되지 않는다.

같은 함수에서 `d=+inf`는 NaN이 아니라 **Z=0**이 된다. 현재 dense `valid_mask()`가 finite 검사로 제외하므로 그 호출 경로는 보호되지만, 공용 함수의 유효성 규약은 정리해야 한다.

### R06 — 품질 검사가 출력문에만 있다

위치: [z_object_depth.py:127](/home/karma/camera/z_object_depth.py:127), [z_reconstruct.py:160](/home/karma/camera/z_reconstruct.py:160).

새 dense 경로는 epipolar FAIL 후에도 FFS와 객체 집계를 진행하고 객체에 `status=ok`를 붙일 수 있다. 희소 경로도 재투영 오차와 점 개수가 FAIL이어도 `results.json`에 저장한다. 후자는 재투영 5px·점 4개를 반환하는 stub으로 실제 저장 동작을 확인했다.

수정 제안: `pass`, `fail`, `unavailable`을 구조화하고 결과 상태로 전파한다. 정류 검사가 텍스처 부족 때문에 불가능한 경우와 측정 결과가 명백히 나쁜 경우를 구분한다. 진단용 저장은 가능하되 정상 측정값과 같은 상태로 내보내지 않는다.

`epipolar_check()`는 ratio test 결과 전체로 RMS를 내므로 오매칭 한두 개에도 민감하다. 반대로 중앙값만 작아도 잘못된 ROI가 있을 수 있다. K/R/T로 설명되는 지지점의 수·분포·inlier 비율과 잔차를 같이 보고해야 한다. dy만으로 baseline의 미터 스케일은 검증할 수 없다.

### R07 — 대응점 부족 시 최종 JSON 저장이 실패한다

위치: [lib_rectify.py:101](/home/karma/camera/lib_rectify.py:101), [z_object_depth.py:191](/home/karma/camera/z_object_depth.py:191).

정류 검사에서 대응점 부족이면 `dy_rms=NaN`을 반환한다. 이를 meta에 넣고 `json.dumps(..., allow_nan=False, default=lambda o: None)`로 저장하면 예외가 난다. `default`는 이미 float인 NaN을 변환하지 않는다.

임시 영상/캘리와 모델 stub으로 실제 main 경로를 실행해 **`Out of range float values are not JSON compliant: nan`**을 재현했다.

수정 제안: 진단값을 저장 전에 `None`으로 명시적으로 변환하고 `epipolar_status=unavailable`을 저장한다. 실패 이유와 샘플 수를 함께 남긴다.

별도 경계 오류: 오른쪽 SIFT descriptor가 하나인 경우 `knnMatch(k=2)`는 길이 1인 항목을 반환한다. [lib_rectify.py:103](/home/karma/camera/lib_rectify.py:103)의 `for m,n in ...`에서 **`not enough values to unpack`**이 재현됐다. 기존 `_match_sift()`는 양쪽 keypoint 수를 검사하므로 동일한 결함이라고 묶으면 안 된다.

### R08 — 오른쪽 대응점의 세로 유효 범위가 빠져 있다

위치: [z_object_depth.py:51](/home/karma/camera/z_object_depth.py:51).

`roi2`의 `y2,h2`를 읽지만 사용하지 않는다. 오른쪽 ROI=(0,100,640,200), 왼쪽 ROI는 전체인 합성 조건에서 y=0의 대응점도 통과했다. 오른쪽 y범위 밖 픽셀 **167,121개**가 유효로 남았다.

수정 제안: 동일 행 y가 오른쪽 ROI 안에 있는지도 검사하고, remap 유효성 마스크를 대응 좌표에서 샘플링한다. alpha=0에서 양쪽 ROI가 항상 전체 화면이라는 가정에 공용 함수를 의존시키지 않는다. 이는 검은 테두리 검사이며 내부 가려짐 검사는 따로 필요하다.

## 3. 정적 검토에서 확인한 수정 항목

### R09 — “동시 촬영” 구현이 실제로는 서로 다른 시각의 프레임이다

위치: [lib_cam.py:63](/home/karma/camera/lib_cam.py:63), [z_capture.py:195](/home/karma/camera/z_capture.py:195), [z_capture.py:202](/home/karma/camera/z_capture.py:202).

왼쪽 프레임을 읽고 프리뷰에서 오른쪽을 한 번 읽은 뒤, SPACE 저장에서는 오른쪽을 다시 읽는다. 프리뷰에 보인 오른쪽 프레임과 저장된 오른쪽 프레임도 다를 수 있다. 타임스탬프와 skew 기록이 없다.

수정 제안: 좌우 pair 획득을 한 곳에서 수행하고 같은 pair를 프리뷰/저장에 사용한다. `grab()` 양쪽 후 `retrieve()`는 decode 시간차를 줄일 수 있지만 하드웨어 노출 동기화를 보장하지 않는다. 폰 전송 지연은 별도 측정해야 한다. [OpenCV VideoCapture 설명](https://docs.opencv.org/4.13.0/d8/dfe/classcv_1_1VideoCapture.html)

예시 계산: f=800, B=0.1m, Z=2m이면 시차 40px. 대상이 가로 1m/s로 움직이고 두 노출이 20ms 차이나면 영상 이동이 8px 섞일 수 있다. 시차가 32px로 측정되는 방향이면 Z=2.5m로 25% 틀린다. 이는 기하 예시이며 현재 장치의 실제 시간차 측정값이 아니다.

### R10 — `rectify_maps(size=...)`를 resize로 사용할 수 없다

위치: [lib_rectify.py:34](/home/karma/camera/lib_rectify.py:34).

`size`를 `stereoRectify`의 원본 imageSize와 remap 출력 크기에 동시에 넣고 원본 K를 유지한다. 640×480, f=800, cx=320에서 `size=(320,240)`을 주면 f≈800, cx=320이 남고 source x≈0~319를 읽는다. 일반적인 절반 resize의 f=400/cx≈160과 다르다. 수치 재현 완료.

현재 `z_object_depth.main()`은 size 인자를 넘기지 않고 FFS 내부에서 축소하므로 이 문제는 **공용 정류 함수의 축소 API 문제**다. 현행 `--scale 0.5`가 이 버그에 걸린다고 해석하면 안 된다.

수정 제안: 원본 `imageSize`와 목표 `newImageSize`를 분리한다. 입력 자체를 resize한다면 K도 그 변환에 맞춰 조정한다. 입력 좌우 shape와 각 calibration의 해상도도 정류 진입점에서 검사한다.

### R11 — 결과 파일의 생명주기와 calibration 연결이 없다

위치: [z_reconstruct.py:145](/home/karma/camera/z_reconstruct.py:145), [z_reconstruct.py:176](/home/karma/camera/z_reconstruct.py:176).

일부 실패 객체는 결과에서 빠지고, 전부 실패하면 `results.json`을 갱신하지 않는다. 임시 파일에 이전 결과를 넣고 전부 실패하는 실제 main 경로를 실행했을 때 이전 내용이 그대로 남았다. `z_stereo_pose --rescale`은 이 파일을 사용하므로 잘못된 baseline 보정으로 이어질 수 있다.

수정 제안: 매 실행마다 run ID·입력 식별자·calibration ID와 모든 객체의 성공/실패 상태를 저장한다. 실패 객체도 원래 instance ID를 유지한다. 임시 파일을 쓴 뒤 replace하는 방식으로 완성된 결과만 노출한다.

새 dense 경로는 별도 K/dist 파일과 stereo JSON의 R/T를 섞어 읽으며, 둘이 같은 calibration 세트인지 확인하지 않는다([z_object_depth.py:113](/home/karma/camera/z_object_depth.py:113)). stereo JSON 안 K1/K2를 기준으로 사용하거나 외부 calibration ID를 검증해야 한다. 현재 dense 진입점은 `shots/meta.json`의 board/mono 포즈 공급 경로도 지원하지 않는다.

### R12 — 추론/출력 계약과 검증 문구를 정리해야 한다

- [lib_ffs.py:88](/home/karma/camera/lib_ffs.py:88): 양수 scale 검증이 없고, 실제 resize된 너비 비율 `W/W0` 대신 요청 scale로 나눈다. 정수가 아닌 크기에서 rounding 차이가 난다. bilinear 시차 확대가 invalid/서로 다른 표면 경계를 섞는 영향은 실측해야 한다. 모든 보간을 잘못됐다고 단정할 문제는 아니다.
- [z_object_depth.py:70](/home/karma/camera/z_object_depth.py:70): 실패 시 `depth_m`, 성공 시 `depth_rect_m`으로 스키마가 다르다. `no_mask` 행은 instance·box·method도 빠진다. 같은 필드를 항상 제공하고 값과 status로 상태를 구분한다.
- [z_object_depth.py:26](/home/karma/camera/z_object_depth.py:26): Z_cam1을 별도 저장한다고 설명하지만 실제 독립 필드는 없고 `center_cam1_m[2]`에만 들어 있다. 이 값이 Z_cam1 중앙값이라는 점을 명시하거나 이름을 추가한다.
- [z_object_depth.py:23](/home/karma/camera/z_object_depth.py:23): “크기 과소평가가 없다”는 보장이 성립하지 않는다. segmentation 누락·occlusion·무효 depth·군집 필터·OBB 선택으로 경계가 사라질 수 있다.
- [lib_detect.py:17](/home/karma/camera/lib_detect.py:17): bbox를 늘려도 “손해가 없다”는 주장은 같은 프로젝트의 `fit_box()` 주석에 있는 배경 군집 오선택 한계와 충돌한다.
- [requirements.txt](requirements.txt): 메인 환경의 Ultralytics/torch 버전이 선언되어 있지 않다. 새 venv_ffs도 별도 lock/환경 생성 기록과 checkpoint 해시가 필요하다. 검토 중 환경이 생성되고 있어 설치 완료·GPU 호환성은 이 문서에서 판정하지 않았다.
- [lib_plane.py:91](/home/karma/camera/lib_plane.py:91): `dist_coeffs`를 받지만 사용하지 않고, [z_live.py:153](/home/karma/camera/z_live.py:153)에서도 전달하지 않는다. 기존 live 평면 경로의 결과를 FFS 비교 기준으로 쓸 때 원영상 왜곡을 무시한 ray가 편향을 만들 수 있다. 정적 코드 확인이며 장치에서 오차 크기는 측정하지 않았다.
- 최종 추가된 [lib_stereo.py:365](/home/karma/camera/lib_stereo.py:365)의 `planar_box()`는 일반 물체에 단일 평면을 가정한다. dense 점군이라는 이유로 평면이 되는 것은 아니다. 현재 평면성 합격 기준이 없고 SVD 특이값도 보존하지 않는다. 평면 근사를 사용하는 객체/모드를 명시하고 잔차·inlier 비율·경계 광선 외삽을 검증해야 한다. 구·병·여러 면이 보이는 물체에 대한 치수 정확도는 미검증이다. depth 추출에 필수 아닌 크기 모델을 항상 정상 치수처럼 붙이지 않는 편이 낫다.

## 4. 공식 FFS 코드도 검증 후 가져와야 한다

검토 commit: `476f4249561f7c79ca707326954f9255643412a6`.

공식 [single TRT demo:292](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/run_demo_single_trt.py#L292)는 모델 해상도 disparity 값에 `1/scale_x`를 곱하면서 공간 크기는 모델 크기로 유지한다. 이후 [316행](https://github.com/NVlabs/Fast-FoundationStereo/blob/476f4249561f7c79ca707326954f9255643412a6/scripts/run_demo_single_trt.py#L316)에서는 K를 scale_x로 축소한다. 입력 해상도 기준 시차와 모델 해상도 기준 K가 섞인다.

고정 소스의 연산을 수치로 재현: 원 f=800, B=0.1m, 가로 축소율=0.5, 모델 시차=20이면 올바른 Z는 **2m**인데 해당 연산은 **1m**를 계산한다. 같은 시차를 모델 크기의 x 좌표에서 빼므로 가시성 검사 단위도 섞인다. ONNX export wrapper가 모델 `forward()`의 disparity를 그대로 반환하는 점까지 확인했다.

이는 **소스·수식으로 확인한 결함이며 실제 TensorRT 엔진 실행 검증은 아니다.** 모델 해상도에서는 축소 K와 원래 모델 disparity를 같이 쓰거나, disparity 공간과 값 모두 원본으로 복원하고 원본 K를 써야 한다.

로컬 `lib_ffs.infer()`는 disparity 공간도 원해상도로 복원한 뒤 scale로 나누므로 이 공식 예제의 오류와 같지 않다. 로컬 코드를 공식 예제의 단순 복사로 바꾸면 오히려 오류를 도입할 수 있다.

## 5. 재현 자료와 검증 한계

- [재현 스크립트](research/depth_review_20260921/reproduce.py): CPU에서 실행. 실제 모델 추론 대신 정확한 대응점/Results/모델 stub을 필요한 곳에만 사용한다. 원본 사진·캘리 파일을 변경하지 않는다.
- [관측 결과](research/depth_review_20260921/observed_results.json): 이번 검토 시점 값과 파일 해시. 성공 테스트 보고서가 아니라 실패 사례의 관측 기록이다.
- [최종 재검증](research/depth_review_20260921/rechecked_results.json): 검토 중 추가된 planar 경로를 포함해 같은 재현을 다시 실행한 기록. 첫 결과와 달라진 점은 R03에 반영했다.

```bash
./venv/bin/python research/depth_review_20260921/reproduce.py
```

실행하면 별도 `latest_results.json`이 생성된다. 이전 관측 기록을 덮어쓰지 않는다. 코드 수정 후에는 입력별 기대값으로 회귀 테스트를 만들고 이번 관측과 비교한다.

FFS checkpoint GPU 추론, 실제 촬영 시각 차이, 실물 거리/치수, 시각 품질의 합격 판정은 이번 검토에 포함하지 않았다. 사용자에게 실제 검수가 필요한 항목과 CPU에서 증명한 코드 오류를 구분한다.

## 6. 수정 단계 제안

1. **기하 계약 확정**: R01/R05/R10, 양안 전방성, 좌표계·크기 검증. 정확한 합성 대응점과 Q 결과로 검증.
2. **객체 영역·통계 통일**: R02/R03. 구멍 mask와 다중 depth 장면에서 같은 최종 집합으로 거리·중심·치수가 산출되는지 검증.
3. **실패 상태·스케일 출처 저장**: R04/R06/R07/R08/R11. 불가/실패 결과, calibration 변경, 재보정 반복을 검증.
4. **FFS 단일 쌍 실험**: 고정 checkpoint·환경에서 PyTorch 실행, 1.0/0.5 scale 거리 일치와 seg/bbox 비교.
5. **촬영·속도 검증**: R09, warm-up 후 전체 지연시간, 실제 촬영 시간차, 실물 거리와 비교. 이후 TensorRT.

하네스의 “문제해결 방안 제시 및 허락 전 코드 수정 하지않기”에 맞춰 이 문서는 문제와 수정안을 제시하는 검토 산출물이다. 이번 요청의 자료조사와 지적 범위는 완료했으며, 애플리케이션 수정은 수행하지 않았다.

## 7. 대응 현황 (2026-09-21, Claude)

R01~R12 전부 코드에 반영했다. 항목별 조치와 근거는 `STATUS.md` 문제점 14, 회귀 검사는 `z_selftest.py`.
이 문서의 `reproduce.py` 는 수정 전 동작을 재현하는 기록이라 수정 후에는 일부 항목이 (의도된) 예외로
중단된다 — 예: `rectify_reversed` 는 이제 `ValueError` 다. 수정 후 확인은 `z_selftest.py` 와
`research/depth_review_20260921/reproduce_after.py`(예외 허용판)로 한다.
