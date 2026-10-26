# lenscal_20261027 — S24 메인 렌즈 보정: 무엇이 정확도를 정하나, 이번 주에 끝내는 방법

만든 시각: 2026-10-26 22:50 (KST, 분석 an1 22:38 / an2 22:41 / an3 22:45). 계산만 했고, 폰과 카메라는 쓰지 않았다.
**새 렌즈 값은 앱과 config 어디에도 넣지 않았다.** 렌즈 값은 보드 보정 데이터로만 판단했고, VIO 오차를 보고 고르지 않았다.
RMS 표기: OpenCV 방식(점마다 √(dx²+dy²)). mrcal이 내는 값은 좌표 하나 기준이라 ×√2 해서 적었다. 화면 px는 따로 적지 않으면 1920×1080 원본 기준이다. VIO 정류 960×540에서는 약 ×0.507이다.

## 1. 결론 — 지금 값을 쓸 수 있나, 무엇이 불확실한가

- **fx ≈ 1286 px(원본)는 두 독립 세트가 같은 값을 냈다.** 세트 A(calcap 정지사진 132장, 33자세, 10-25/26)와 세트 B(10-23/24 녹화, 5x3 보드만, 47자세 217장)를 따로 mrcal로 풀었다(OPENCV4, 휨 없음).
  - fx는 A 1286.4, B 1285.2, A+B 1286.5다. OpenCV k12로 푼 1286.25, 지금 기본값 cal20261024d 1286.2와 차이가 0.1 % 안이다.
  - 자세 단위 부트스트랩 퍼짐(sd)은 A 4.3 px(0.34 %), B 2.5 px(0.20 %), A+B 2.2 px(0.17 %)다.
- **하지만 fx에는 통계 퍼짐보다 큰 계통 불확실성이 남아 있다. 하한 쪽으로 −0.65 %까지다.**
  - 데이터 자체가 "보드가 평평하지 않다"고 말한다. mrcal의 보드 휨(calobject_warp, 가로 방향 포물선) 추정값이 A 0.19 ± 0.02 mm, B 0.09 ± 0.02 mm로 0과 확실히 다르다(자세 부트스트랩 40회).
  - 휨을 넣으면 RMS가 A −9 %, B −5 % 줄고, fx는 1278.4(A) / 1281.7(B) / 1280.9(A+B)로 내려간다. 휨 1 mm당 약 −46 px다.
  - 두 세트의 휨 값이 서로 다르다(4 sd 차이). 그래서 진짜 화면 휨인지, 덮개 유리 굴절이나 검출 치우침이 휨처럼 보이는 것인지 이 데이터로는 가를 수 없다.
  - 정리하면 **fx 참값은 1278–1287 범위로만 말할 수 있다(추정).** 이 오차는 화면 보드를 쓰는 한 사진을 더 찍어도 줄지 않는다(A와 B가 같은 노트북 화면).
- **왜곡(k1 k2) 모양은 세트끼리 맞는 편이다.** 카메라 회전 차이를 맞춘 뒤 A와 B의 광선 차이는 r0.5에서 1.6 px, r0.8에서 3.3 px, 귀퉁이에서 3.4 px다. 두 세트를 합친 불확실성의 1.9–2.3배로, mrcal의 기준 "약 2배"의 경계다.
  - 같은 세트를 반으로 나눈 비교도 2.0–2.7배다(r ≥ 0.5). 그래서 이 데이터만으로는 "모델 오차 없음"이라고 확인해 줄 수 없다.
  - cal20261024d와 A+B 보정의 차이는 r0.5 0.5 px, r0.8 1.0 px, 귀퉁이 1.4 px로, 불확실성 수준이다.
- **주점(cx)은 접선 왜곡(p1 p2)을 풀어 주면 흔들린다.** cx sd가 5–6 px이고, A와 B의 cx가 13 px 다르다(967.7 대 980.8).
  - 접선을 0으로 묶으면(k12_notan) cx 965 ± 1.6이다.
  - 이 차이는 대부분 0.4–0.6° 카메라 회전과 같은 효과다. 카메라–IMU 회전(R_ci)을 다른 렌즈 값으로 맞춰 두었다면 그만큼 어긋난다(추정).
- **mrcal splined 모델은 이 데이터로 쓸 수 없다.** 홀드아웃 오차가 A 1.0–2.3 px, B 0.22 px로 OPENCV4(0.20 / 0.15 px)보다 나쁘다. 사진당 모서리 8개, r ≤ 0.88만 덮인 데이터로는 과적합된다.
  - OPENCV8은 OPENCV4보다 홀드아웃이 나아지지 않는다(A 0.194 대 0.198 px). 덮인 영역 안에서는 k1 k2로 충분하다.
- **앱이 지금 쓰는 렌즈는 c4f60이다(저장소 기준).**
  - 근거 1: `integ2` 브랜치 `MainActivity.java` 294행 — 프로필 rec60이면 `ov.cfgName = "c4f60"`.
  - 근거 2: `OvRunner`가 `files/ov/c4f60/remap_1920x1080.bin`으로 1920×1080 → 960×540 정류를 한다. 그 remap은 `config/phone_ov_main/variants.json`(2026-10-24 19:57, 모든 변형이 렌즈 파일 하나를 공유)에서 만든다.
  - 그 렌즈 파일은 `hs_1920x1080_f2.22_cal20261022.json`이다: 초점 2.22 dpt, 모니터 6x6 보드, k1k2p_k3에 초점 배율 0.98624를 곱한 값(K_raw 1285.95 / 1284.96 / 968.38 / 552.47, k2 −0.117, k3 +0.058).
  - 폰에 실제로 깔린 파일은 이번에 확인하지 않았다(가정: 저장소와 같음, integ_20261025 README의 sha 일치 기록).
  - 프로필 rec20261025는 `ov_cfg`가 없으면 `files/ov` 바로 아래 설정을 쓴다. 그 내용은 확인하지 않았다.
- **c4f60 렌즈는 0.5 dpt 보드 데이터(두 독립 세트 모두)에 가장 안 맞는다.**
  - 자세만 다시 맞춘 RMS: A 0.254 px(자기 보정 0.190, 24d 0.199), B 0.217 px(자기 0.145, 24d 0.149). 휨을 0–0.19 mm 어떻게 가정해도 순서가 같다.
  - 회전을 맞춘 뒤 광선 차이(A+B OPENCV4 대비): r0.5 2.3 px, **r0.8 8.7 px, 귀퉁이 11.4 px**. 같은 자리 불확실성 0.35 / 0.64 / 0.86 px의 6–13배다.
  - 960×540 정류 화면에서는 가운데 ≈ 0, r0.8 ≈ 4.4 px, 귀퉁이 ≈ 6 px다. 코디네이터 rectdiff(회전 맞춤 없음)의 r0.8 최대 7.1 / 귀퉁이 8.8 px와 같은 방향이다.
  - 뜻: 앱의 VIO는 화면 가장자리 특징점을 일관되게 수 px 틀린 방향으로 본다. OpenVINS가 가정하는 픽셀 잡음(약 1 px)보다 크다. VIO 정확도에 얼마나 영향을 주는지는 재지 않았다(추정).
  - 원인으로 섞인 변수가 셋이다: 초점 2.22 대 0.5 dpt(초점이 바뀌면 왜곡도 바뀜 — 가장 그럴듯함, 추정), 다른 보드(모니터 대 노트북 화면), 스트림(hs 대 std/정지사진).
  - 렌즈 교체는 사람이 결정한다. 이 보고서는 아무것도 바꾸지 않았다.
- **판정:** 지금 값(cal20261024d 또는 calcap k12, fx 1286)은 임시로 쓸 만하다. 보드 데이터 기준으로 c4f60보다 낫고, 두 독립 세트와 맞는다. 그러나 **fx는 −0.65 %까지, 가장자리 왜곡은 r0.8에서 약 3 px(원본) 수준으로 확정되지 않았다.**
  - 둘 다 노트북 화면 보드가 원인일 가능성이 크다(추정). 인쇄 평판 보드로 독립 세트 2개를 찍으면 끝낼 수 있다(아래 2·3).

## 2. 이번 주에 끝내는 절차 (촬영 2회 + 계산 1회)

**보드 (가장 중요):**
- **A1(841×594 mm) 무광 인쇄 ChArUco 12×9칸, 칸 60 mm(720×540 mm), 마커 45 mm, DICT_6X6_250.** 안쪽 모서리는 11×8 = 88개로, 지금(8개)의 11배다. 1 m에서 칸 하나가 약 77 px, 0.6 m에서 약 129 px다.
  - ChArUco는 판이 일부 잘려도 쓸 수 있어서 화면 귀퉁이(r > 0.9)를 채울 수 있다. 모서리 줄이 8줄이라 휨 2축을 모두 추정할 수 있다(지금은 2줄이라 세로 휨을 볼 수 없음).
- **평평한 판에 붙인다:** 알루미늄 복합판(3–4 mm, 직접 인쇄 또는 부착)이나 5 mm 이상 판유리. 폼보드는 쓰지 않는다.
- **잰다:**
  - 평면도: 곧은 금속자 + 필러게이지로 네 변, 가운데 가로·세로, 두 대각선의 최대 틈을 잰다(자를 뒤집어 한 번 더). 목표 ≤ 0.1 mm.
  - 크기: 강철자로 첫 모서리부터 끝 모서리까지 가로(10칸)·세로(7칸)를 따로 잰다. 전체 배율은 fx에 영향이 없지만, 가로·세로 비율 0.2 % 오차는 fx를 −1.3 px, fy를 +1.4 px 바꾼다.

**폰:**
- 지금 고정값(노출 2 ms, ISO 고정, 초점 0.5 dpt, OIS/NR/edge/LDC off)은 바꾸지 않는다.
- 폰은 받침에 고정하고 판을 움직인다. 롤링셔터와 흔들림을 없애기 위해서다.
- 사진마다 결과 메타데이터를 저장한다: `LENS_FOCUS_DISTANCE`, `LENS_STATE`(STATIONARY인지), `LENS_OPTICAL_STABILIZATION_MODE`, `DISTORTION_CORRECTION_MODE`, `LENS_INTRINSIC_CALIBRATION`.

**자세 목록 (세트당 약 55자세 × 2장, 약 40분, 추정):**
- 4장은 필요 없다. calcap 분석에서 칸당 4장을 1장으로 줄여도 퍼짐이 같았다. 정보 단위는 자세 수다.

| 묶음 | 거리 | 화면 위치 | 기울기 (yaw = 좌우, pitch = 상하, roll = 판 회전) | 자세 수 |
|---|---|---|---|---|
| N-C | 0.6–0.7 m | 가운데 | yaw ±45, pitch ±45, 대각 (yaw ±35 × pitch ±35) 4개, roll 90° + yaw ±45, roll ±30° + pitch ±40 | 12 |
| N-E | 0.7 m | 왼쪽·오른쪽·위·아래 가장자리 (일부 잘려도 됨) | 가운데를 향해 35°, 바깥으로 35° | 8 |
| N-K | 0.7 m | 네 귀퉁이 (r > 0.9까지, 일부 잘림) | 가운데를 향해 30°, 바깥으로 30° | 8 |
| M-C | 1.0 m | 가운데 | 정면 0°, yaw ±40, pitch ±40, 대각 2개, roll 45° + 20° | 8 |
| M-EK | 1.0 m | 가장자리 4 + 귀퉁이 4 | 30° (가운데를 향해) | 8 |
| F | 1.5 m | 가운데 | 정면, yaw ±35, pitch ±35 (초점·거리 점검) | 5 |
| N6 | 0.6 m | 가운데 | yaw ±45, pitch ±45, 대각 2 | 6 |

- roll 0/45/90°를 꼭 섞는다(지금은 −8…+13°뿐). yaw ±45°를 넘긴다(지금은 ±40°까지). 대각을 8개 이상 넣는다(지금 2개). 이 셋이 calcap 분석(결과 9)에서 정보가 가장 큰 자세다.
- 화면 덮임은 32×18 칸 중 80 % 이상, 귀퉁이 네 곳에 각각 모서리 점 100개 이상을 목표로 한다.
- 0.6 m는 초점 0.5 dpt 기준 흐림 약 4–5 px(조사 노트 6절 계산, 추정)다. 체스판 모서리에는 괜찮은 수준이지만, C7(거리별 fx)로 확인한다.

**촬영 횟수:**
- 세트 A: 1일차.
- 세트 B: 2일차. 또는 앱을 재시작하고 폰을 받침에서 떼었다 다시 단 뒤 찍는다. 같은 목록이고, 독립 교차검증용이다.
- 세트 G(선택, 10자세): 폰을 세로 또는 거꾸로 장착한다. 렌즈의 중력 처짐 확인용이다.

**계산:**
- 도구: calcap 검출 결과를 `corners.vnl`로 만들어 mrcal(이 폴더 `scripts/`, `mrcal_env.sh`)과 OpenCV k12_notan으로 같이 푼다. calobject_warp를 켠다(이제 2축 다 관측 가능).
- 모델: OPENCV4/k12_notan(OpenVINS radtan 호환), OPENCV8, SPLINED(모서리가 많으면 이제 가능)를 비교한다. 3절 C3를 통과하는 가장 단순한 모델을 쓴다.
- 산출: 세트별 값, 불확실성 지도, A 대 B 차이 지도, 휨 추정과 실측 비교, 가까운·먼 거리 부분집합 비교, 24d·c4f60 대비 960×540 광선 차이.

## 3. 합격 기준 (숫자; 지금 데이터 결과와 비교)

| # | 기준 | 지금 (화면 보드) | 판정 |
|---|---|---|---|
| C1 | 세트마다 RMS ≤ 0.25 px, 이상치 ≤ 3 %, 모서리별 평균 잔차 ≤ 0.05 px (무늬 없음) | A 0.19 / B 0.15 px, 이상치 0.5 % / 0 %; 모서리별 평균 잔차 0.02–0.08 px (calcap 결과 6) | 부분 통과 |
| C2 | mrcal 투영 불확실성 (무한 거리, 가장 나쁜 방향) 가운데 ≤ 0.2 px, r ≤ 0.8 ≤ 0.5 px (원본; 목표값은 추정) | A 0.31 / 1.19, B 0.20 / 0.76, A+B 0.17 / 0.64 px | 실패 |
| C3 | 독립 세트 A 대 B 차이 (회전 맞춤) ≤ 2 × 합친 불확실성, 그리고 r0.8에서 ≤ 1.0 px (정류 0.5 px) | 1.9–2.3배, r0.8 3.3 px | 실패 |
| C4 | \|fx_A − fx_B\| ≤ 0.15 %, 세트별 자세 부트스트랩 sd(fx) ≤ 0.15 %, 주점 (k12_notan) 차이 ≤ 2 px | 0.09 % (단, 같은 화면이라 같은 편향 공유), sd 0.34 / 0.20 % | 실패 |
| C5 | 보드 실측 평면도 ≤ 0.1 mm, mrcal 휨 추정이 실측과 ±0.1 mm 안, 휨을 켜고 끈 fx 차이 ≤ 0.1 % | 휨 0.09–0.19 mm 원인 모름, fx 차이 0.3–0.65 % | 실패 |
| C6 | 같은 세트 반반 나누기 차이 ≤ 2 × 불확실성 (r ≤ 0.8에서 70 % 이상) | 2.0–2.7배, 25–50 % | 실패 |
| C7 | 가까운 (≤ 0.8 m) 대 먼 (≥ 1 m) 부분집합 fx 차이 ≤ 2 sd (흐림·noncentral 점검); 세트 G 중력 영향 \|Δfx\| ≤ 0.1 %, \|Δ주점\| ≤ 2 px | 거리별 fx 1285.4 대 1290.4 (calcap 결과 2) | 판정 보류 |

- 전부 통과하면 그 값을 "확정"으로 하고, 960×540 정류 remap을 새로 만드는 것은 사람이 정한다.
- C3나 C5가 실패하면 보드나 촬영을 의심한다. VIO 결과로 렌즈를 고르지 않는다.
- 기준 숫자 중 C2, C4, C3의 1.0 px는 정해진 출처가 없는 내 제안이다(추정). C3의 "2배"와 C1의 이상치 3 %는 mrcal 문서를 따랐다.

## 4. 근거 표

| 주장 | 출처 | 확신 |
|---|---|---|
| 교차검증 (독립 두 세트) 차이가 불확실성의 약 2배를 넘으면 모델·계통 오차 — 가장 중요한 진단 | https://mrcal.secretsauce.net/tour-cross-validation.html , https://mrcal.secretsauce.net/how-to-calibrate.html | 높음 |
| 투영 불확실성은 픽셀 잡음만 반영하고 판 휨·모델 오차는 빠짐; 단순 모델은 늘 낙관적 | https://mrcal.secretsauce.net/uncertainty.html , https://mrcal.secretsauce.net/tour-uncertainty.html | 높음 |
| 판은 가까이, 화면을 채우고, 약 45° 기울이고, 귀퉁이까지 덮고, 약 100장; 정면 자세는 정보가 적음; 모서리 수와 판 크기/거리 비가 중요 | https://mrcal.secretsauce.net/how-to-calibrate.html , https://mrcal.secretsauce.net/tour-choreography.html | 높음 |
| calobject_warp = 축마다 포물선 휨 항 (우리 보드는 모서리 줄이 2줄이라 세로 항을 관측 못 함 — 이번에 풀이가 특이 행렬로 실패해 확인) | https://mrcal.secretsauce.net/formulation.html ; 이 폴더 an2 | 높음 |
| 판 비평면 약 0.1 mm도 계통 오차로 나타나고 RMS가 작아도 숨음; 표준 공분산은 과소평가 → 자세 단위 부트스트랩 | https://arxiv.org/pdf/2107.13484 | 높음 |
| 판 휨은 무작위 잡음보다 해롭고 원통형 (한 축) 휨은 주점을 틀어지게 함; 약 45° 기울기가 가장 좋음 | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음 |
| 화면 유리 굴절이 보이는 패턴 위치를 옮겨 편향을 만듦 (별도 모델 필요) | https://www.jeos.org/index.php/jeos_rp/article/view/14044 , https://openaccess.thecvf.com/content_iccv_2015/papers/Ha_Accurate_Camera_Calibration_ICCV_2015_paper.pdf (검색 요약만) | 중간 |
| 판이 영상면이나 영상 축과 평행하면 특이; 초점용은 넓은 각, 왜곡용은 귀퉁이에 | https://ar5iv.labs.arxiv.org/html/1907.04096 | 높음 |
| 안내형 자세 선택으로 fx sd OpenCV 9.0 px → 1.2 px | https://april.eecs.umich.edu/media/pdfs/richardson2013iros.txt | 높음 |
| ChArUco는 부분 보임으로 귀퉁이를 채움; 원 격자는 원근·왜곡 편향 | https://calib.io/blogs/knowledge-base/calibration-patterns-explained | 높음 |
| 전체 칸 크기 (배율)는 단안 내부값에 영향 없음 (가로·세로 비율은 영향 있음 — calcap 결과 7에서 계산) | https://forum.opencv.org/t/actual-size-of-check-block-when-performing-camera-calibration/13428 | 높음 |
| 판은 평평하게, 알루미늄·아크릴에 붙이고, 인쇄 배율은 다시 잴 것 | https://github.com/ethz-asl/kalibr/wiki/calibration-targets | 높음 |
| 폰 내부값은 초점·OIS로 프레임마다 변할 수 있음; 왜곡 보정 기본 ON → 결과 메타데이터로 OFF 확인 | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음 |
| 스마트폰 내부값은 시도마다 크게 변할 수 있고 Aprilgrid·큰 판이 불확실성을 줄임 | https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/1669/2025/isprs-archives-XLVIII-G-2025-1669-2025.pdf | 높음 |
| OpenVINS 오프라인 보정은 재투영 0.2–0.5 px면 양호, 판이 영상 전체에 보이게 | https://docs.openvins.com/gs-calibration.html | 높음 |
| (측정) 두 독립 화면 세트의 fx 일치 (1286.4 / 1285.2), 휨 추정 0.19 / 0.09 mm, 휨 1 mm당 fx −46 px | 이 폴더 an1.json, an2.json | 높음 (숫자) / 중간 (원인 해석) |
| (측정) c4f60 렌즈가 두 세트 모두에서 가장 나쁨, r0.8 8.7 px ≫ 불확실성 0.64 px | an1.json `fixed`, an3.json `fixed_with_warp` | 높음 (숫자) / 중간 (원인 = 초점 차이, 추정) |

자세한 조사 노트 (출처 URL, 검색 요약만 본 것 표시): `research_notes.md`.

## 5. 입력과 방법

- 세트 A: `~/scanner/StereoDepthSeg/scan3d/recordings/calcap_20261025_214000`. calcap_calib.load_views로 검출했다(132장, 33칸, 장당 모서리 8개, 1056점). → `data/corners_A.vnl`, `data/meta_A.json`.
- 세트 B: `~/scanner/StereoDepthSeg/scan3d/runs/lens_calib_f050_20261023/views/poses.pkl`에서 board_laptop_5x3 녹화 55개(가까운 자세를 묶으면 47개)만 골랐다. 녹화마다 프레임 4개를 고르게 뽑아 217장, 1736점이다. → `data/corners_B.vnl`.
  - 6x4 보드와 FAR 2마커 녹화는 뺐다. mrcal은 한 문제에 보드 하나만 받는다.
  - **A와 B 사이에 섞인 변수:** 날짜, 스트림(정지사진 YUV 10 fps 대 std60 녹화), ISO/노출, 자세 구성. 같은 것: 노트북 화면, 초점 0.5 dpt, 1920×1080.
- 도구: mrcal 2.1. sudo가 없어서 Ubuntu jammy .deb를 `dpkg -x`로 `~/scanner/tmp_lenscal/root`에 풀었다(`mrcal_env.sh`).
  - 기초 풀이: `mrcal-calibrate-cameras --skip-calobject-warp-solve --corners-cache corners_<S>.vnl --lensmodel LENSMODEL_<M> --focal 1286 --imagersize 1920 1080 --object-spacing 0.0604 --object-width-n 4 --object-height-n 2 '<S>/*'`. 결과는 `~/scanner/tmp_lenscal/m/<S>_<M>_nw/`에 두었다. splined는 order=3, Nx=12, Ny=7, fov_x_deg=80이다.
  - 휨은 세로 항이 관측 불가라 mrcal이 직접 풀지 못한다(특이 행렬). 그래서 가로 휨을 고정값으로 바꿔 가며 다시 풀어 최소점을 찾았다(an2 프로파일, 자세 부트스트랩 40회).
- 스크립트: `scripts/dump_calcap.py`, `dump_setB.py`(venv_ffs, cv2 5), `make_vnl.py`, `lenscal_lib.py`, `an1.py`, `an2.py`, `an3.py`(시스템 python3 + mrcal).
  - an1: 기초 풀이, 불확실성, 모델 홀드아웃, A 대 B, 고정 렌즈 점수.
  - an2: 반반 나누기(회전 맞춤), 자세 부트스트랩 100회, 휨 프로파일과 부트스트랩.
  - an3: 최적 휨에서의 값, 휨을 넣은 고정 렌즈 점수, 그림.
  - 다시 돌리기: `source mrcal_env.sh; cd ~/scanner/tmp_lenscal/m/A_OPENCV4_nw && python3 <scripts>/an1.py`(an2, an3도 같은 방식, an3는 그림 폴더를 인자로 받음).
- 결과: `an1.json`, `an2.json`, `an3.json`, `an1_stdout.txt`, `an2_stdout.txt`, 그림 `uncertainty_maps.png`(세트별 불확실성 지도와 관측 점), `warp_profile.png`(휨 가정에 따른 RMS와 fx), 모델 `models/{A,B,AB}_OPENCV4_nowarp.cameramodel`.
- 가정:
  - 폰에 깔린 c4f60 = 저장소 c4f60 / variants.json.
  - ISP가 LDC off 요청을 지켰다.
  - mrcal 휨 부호: + 는 OpenCV bowscan의 "가운데가 카메라 쪽으로"와 같은 방향이다. 둘 다 fx가 줄어드는 것으로 판단했다(추정).
  - mrcal 불확실성은 같은 자세 사진 4장을 독립으로 센다. 실제 퍼짐은 자세 부트스트랩 쪽이 더 정직하다(an2 boot).
