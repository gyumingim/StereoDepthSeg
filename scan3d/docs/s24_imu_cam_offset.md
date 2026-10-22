# Galaxy S24 (SM-S921) IMU–메인 카메라 오프셋 조사

작성 2026-10-22 (웹 조사, 실측 없음). 비교 대상: 데이터로 얻은 값 = 카메라 좌표계에서 본 IMU 위치 (0.49, 4.60, −1.11) cm.

## 결론 (먼저)
- **보드 위 IMU 위치를 보여 주는 공개 자료는 찾지 못했다.** iFixit 공개 페이지·Samsung 수리 PDF 공개 부분·TechInsights 무료 요약·GSMArena(PBKreviews) 기사 어디에도 IMU 부품명이나 위치가 없다. 그래서 **수치 추정은 못 하고, 물리적으로 가능한 범위만 정했다.**
- **x, y (4.6 cm 쪽 포함): 물리적으로 가능하다.** 부호까지 맞다. 4.6 cm는 IMU가 카메라 반대쪽 옆면 근처(옆면에서 약 0.5–1 cm 안쪽, 추정)에 있다는 뜻이다. 부호가 반대였다면 IMU가 폰 밖에 있어야 하므로 불가능하다.
- **z (−1.11 cm, 세션별 −0.1 ~ −2.2 cm): 평균값부터 물리적으로 맞지 않을 가능성이 높다.** 폰 두께는 7.6 mm(+카메라 돌출부 약 1–2 mm, 추정)다. IMU는 카메라 광학 중심보다 화면 쪽에 있어야 하므로(부호 −는 맞음) 크기는 **약 −0.3 ~ −0.8 cm (추정)**이어야 한다. −2.2 cm는 불가능, −1.1 cm도 가능성 낮음. z는 데이터에서 잘 안 잡히는 성분이다(세션 간 흔들림이 그 증거) → **z는 약 −0.5 cm 사전값으로 고정하거나 강한 prior를 거는 것을 권한다.**
- 가장 싼 확인 방법: (1) 캘리퍼로 뒷면 렌즈 중심 위치를 잰다. (2) 회전 의자/턴테이블로 IMU 위치를 잰다(아래 §5). 반나절, 비용 0.

## 1. 찾은 자료
| 항목 | 내용 | 출처 |
|---|---|---|
| IMU 부품 | S24 IMU 부품명을 밝힌 공개 자료 **없음**. TechInsights 무료 요약에는 조도·근접 센서(Sensortek STK33F11, STK6D2X)만 있고 IMU는 없다. "LSM6DSV"는 앱이 보고한 센서 이름(`Sensor.getName()`)뿐이다 → **가정**으로 둔다. LSM6DSV는 OIS/EIS 채널이 따로 있는 폰용 6축 IMU라서 가능성은 높다. | [TechInsights SM-S921B/DS](https://techinsights.com/ja/node/56647), [TechInsights SM-S921U1](https://www.techinsights.com/ja/node/57042), [ST LSM6DSV (Mouser)](https://www.mouser.in/new/stmicroelectronics/stm-lsm6dsv-inemo-inerial-module/), [LSM6DSV datasheet](https://www.alldatasheet.com/datasheet-pdf/view/1447360/STMICROELECTRONICS/LSM6DSV.html) |
| 메인 보드 | Main PBA = AP·메인 카메라 연결·저장장치·보조 마이크. Sub PBA = USB-C·메인 마이크. 메인 보드는 2층(적층) 구조. 공개 부분에 배치도는 없다(84쪽 전체는 iFixit 무료 계정 필요). | [Samsung 수리 교육 자료 (iFixit)](https://www.ifixit.com/Document/Pkr6KxP2ixAFMmsu), [Samsung 수리 가이드 PDF](https://documents.cdn.ifixit.com/cyMjHI1QJBnMKluM.pdf), [GSMArena / PBKreviews](https://www.gsmarena.com/samsung_galaxy_s24_teardown_results_in_9_10_reparability_score-news-61408.php) |
| 후면 카메라 | 카메라 3개가 세로로 한 줄. 초광각은 "메인 보드와 메인 카메라에 금속 핀 두 개로 걸려 있다" → 초광각 = 맨 위, **메인(50 MP) = 가운데**, 망원 = 아래. 카메라 모듈은 메인 보드 옆(보드에 걸림), 같은 나사 2개로 고정. | [iFixit S24 Rear Cameras](https://www.ifixit.com/Guide/Samsung+Galaxy+S24+Rear+Cameras+Replacement/175125), [iFixit S24+ Rear Cameras (초광각 = 맨 위 명시)](https://www.ifixit.com/Guide/Samsung+Galaxy+S24+Plus+Rear+Cameras+Replacement/175310) |
| 폰 크기 | 147 × 70.6 × 7.6 mm. 렌즈 중심 좌표(mm)를 공개한 도면은 **찾지 못함**. | [agood (크기)](https://www.agood.com/blogs/stories/samsung-s24-size-info-phone-case-compatibility), [Sammobile 유출 크기](https://www.sammobile.com/?p=15046007) |
| 카메라 사양 | 메인 50 MP F1.8, 망원 10 MP 3x, 초광각 12 MP | [Samsung 공식 카메라 사양](https://www.samsung.com/ae/support/mobile-devices/what-is-the-specification-of-the-cameras-in-the-new-galaxy-s24-series) |

찾아봤지만 쓸모없던 곳: 중국 분해 사이트(52audio, ICHome, eet-china)에서 S24 기본형 보드 칩 목록을 찾지 못함. JerryRigEverything/PBKreviews는 영상뿐이라 칩 각인을 글로 확인할 수 없음.

## 2. 좌표계 변환 (가정 명시)
- Android 센서 축: 세로로 들고 화면을 볼 때 x 오른쪽, y 위, z 화면 밖(사용자 쪽).
- 후면 카메라 SENSOR_ORIENTATION = 90° (가정, 사용자 제공). 원본 이미지를 시계 방향 90° 돌리면 세로 화면에서 바로 선다.
  - 원본 이미지 위쪽 → 장면 오른쪽 = 기기 +x → **y_cam(아래) = −x_dev**
  - 원본 이미지 오른쪽 → 화면 아래 = 기기 −y → **x_cam(오른쪽) = −y_dev**
  - 후면 카메라는 화면 반대쪽을 봄 → **z_cam(앞) = −z_dev**
  - 행렬식 +1 (오른손 좌표계) 확인.
- 따라서 우리 값을 기기 축으로 바꾸면 카메라 → IMU 벡터 = (x_dev, y_dev, z_dev) = (−4.60, −0.49, +1.11) cm.
  - 화면 쪽에서 볼 때 IMU는 카메라보다 **왼쪽으로 4.6 cm, 아래로 0.5 cm, 화면 쪽으로 1.1 cm**.
- 주의: OpenVINS 등 도구가 회전 R_CtoI를 함께 추정했다면 위 변환 대신 그 회전을 써야 한다. 위 행렬은 "이론상" 회전이다.

## 3. 물리 범위 점검
렌즈 위치는 사진 기준 대략값(실측 아님, **추정**): 뒷면에서 볼 때 렌즈 중심이 왼쪽 옆면에서 약 12–16 mm. 화면 쪽에서 보면 오른쪽 → 카메라 x_dev ≈ +19 ~ +23 mm(폰 중심 기준). 메인 렌즈는 위에서 약 25–30 mm(추정).

| 성분 | 우리 값 | 물리적으로 가능한 범위 (추정) | 판정 |
|---|---|---|---|
| 폰 가로 방향 (y_cam = −x_dev) | +4.60 cm | 0 ~ +5.3 cm (반대쪽 옆면 프레임 안까지). 음수면 2 cm 안에서만 가능 | **가능.** IMU가 카메라 반대쪽 옆면에서 약 0.5–1 cm 안쪽에 있다는 뜻. 메인 보드가 폭 전체를 덮으므로 있을 수 있는 자리 |
| 폰 세로 방향 (x_cam = −y_dev) | +0.49 cm (IMU가 0.5 cm 아래) | 메인 보드 범위: 위쪽 끝 ~ 위에서 약 60 mm → 약 −2.5 ~ +3.5 cm | **가능**(보드 안) |
| 두께 방향 (z_cam) | −1.11 cm (세션 −0.1 ~ −2.2) | 광학 중심은 렌즈 쪽(센서보다 약 f ≈ 5–6 mm 앞, 뒷유리·돌출부 근처). IMU는 보드 위(두께 중간 근처). → **약 −0.3 ~ −0.8 cm** | **평균 −1.1은 과다 가능성 높음, −2.2는 불가능** |

메인 카메라 초점거리: 센서 4080 px × 2 µm(4-in-1) ≈ 8.2 mm 폭, 35 mm 환산 24 mm → f ≈ 5.7 mm (**추정**). 보정값 fx ≈ 2800–2900 px(4080 폭 기준)이면 맞는 숫자다.

불확실성 요약: 가로·세로 성분은 공개 자료로 ±2–3 cm보다 좁힐 수 없다(IMU 위치를 모름). 두께 성분은 ±0.3 cm 정도로 물리 범위를 말할 수 있다.

## 4. 해석
- x, y가 17개 녹화에서 일정하다는 점 + 물리 범위 안 + 부호까지 맞음 → x, y는 믿을 만하다(단, 독립 실측 전까지 "확인 안 됨").
- z가 세션마다 2 cm나 흔들리는 것은 그 축 회전 자극이 부족해 관측성이 약하다는 뜻으로 보인다(추정). 물리적으로 7.6 mm 두께 안에 있어야 하므로 z는 −0.5 cm 근처로 고정하는 것이 데이터보다 낫다.

## 5. 가장 싼 실측 방법
1. **렌즈 위치 (캘리퍼, 10분, ±0.3 mm)**: 뒷면에서 메인(가운데) 렌즈 중심까지 옆면·윗면 거리를 잰다.
2. **IMU 위치 (회전 시험, 1시간, ±2–3 mm 추정)**: 폰을 판에 테이프로 붙이고 회전 의자/턴테이블에 올린다. 일정 속도로 돌리면 가속도계 수평 성분 = ω²·r (ω는 자이로에서). 회전축 위치를 판에 표시하고 폰을 두세 위치(90° 돌려서)에 놓고 r을 구하면 IMU의 평면 위치가 삼각측량으로 나온다. 렌즈 중심을 축 바로 위에 두면 r이 곧 IMU–렌즈 평면 거리. 두께 방향도 폰을 옆으로 세워 같은 시험을 하면 나온다.
3. **보드 사진 (무료)**: iFixit 무료 계정으로 Samsung 수리 자료 84쪽 전체를 보거나, PBKreviews 분해 영상에서 메인 보드 프레임을 캡처해 ST 각인(약 2.5×3 mm 패키지)을 찾는다. 찾으면 보드 크기를 자로 써서 위치를 잴 수 있다.

## 출처
- https://techinsights.com/ja/node/56647
- https://www.techinsights.com/ja/node/57042
- https://www.gsmarena.com/samsung_galaxy_s24_teardown_results_in_9_10_reparability_score-news-61408.php
- https://www.ifixit.com/Document/Pkr6KxP2ixAFMmsu
- https://documents.cdn.ifixit.com/cyMjHI1QJBnMKluM.pdf
- https://www.ifixit.com/Guide/Samsung+Galaxy+S24+Rear+Cameras+Replacement/175125
- https://www.ifixit.com/Guide/Samsung+Galaxy+S24+Plus+Rear+Cameras+Replacement/175310
- https://www.samsung.com/ae/support/mobile-devices/what-is-the-specification-of-the-cameras-in-the-new-galaxy-s24-series
- https://www.agood.com/blogs/stories/samsung-s24-size-info-phone-case-compatibility
- https://www.sammobile.com/?p=15046007
- https://www.mouser.in/new/stmicroelectronics/stm-lsm6dsv-inemo-inerial-module/
- https://www.alldatasheet.com/datasheet-pdf/view/1447360/STMICROELECTRONICS/LSM6DSV.html
