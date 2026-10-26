# S24 메인 카메라 내부 보정(intrinsics) 정확도 조사 — 2026-10-26

작성: 조사 에이전트 (읽기 전용 조사, 코드 수정 없음). 형식: `주장 | 출처 URL | 확신(높음/중간/낮음)`.
"(검색 요약만)" = 페이지를 직접 열지 못하고 검색 결과 요약에서만 본 것. "(계산·추정)" = 출처 숫자로 내가 계산한 것, 측정 아님.

## 0. 결론 먼저 (지금 상태의 진단 + 1~2회 촬영 계획)

- 지금 데이터의 가장 큰 약점은 (1) 한 장에 코너 8개뿐, (2) 노트북 화면이라 평면도·유리 굴절을 모름, (3) 기울기 ±40° 안쪽·롤 0·대각 자세 적음, (4) 화면 모서리(r>0.88)가 비어 있음. 이 네 가지 모두 mrcal·Zhang·calib.io가 정확도 손실 원인으로 꼽는 것이다.
- fx 부트스트랩 0.36% 는 "초보가 OpenCV로 사람 손으로 모은 보정" 수준(Zhang 실측 부분집합 편차 0.35%, AprilCal 연구 OpenCV 사용자 2.4%)이고, 잘 모은 데이터는 0.1~0.3% 대다(아래 8절).
- 권장: **인쇄 체스판(또는 ChArUco) A1~A0, 무광, 평평한 판(유리·알루미늄 복합판/허니컴)에 붙임** → 평면도는 곧은자+필러게이지로 재고, mrcal로 `calobject_warp` 를 같이 추정.
- 촬영 A(본 보정): 0.6~1.2 m, 서로 다른 자세 60~100개, 기울기 30~45°(좌우·상하·대각 모두), 판 회전(롤) 0/45/90°, 부분 보임 허용으로 네 모서리까지 채움.
- 촬영 B(검증): 다른 날·카메라 재시작 후 같은 방법으로 독립 세트 → `mrcal-show-projection-diff` 로 두 결과 차이가 불확실성의 약 2배 이내인지 본다.
- 판정은 RMS 하나로 하지 않는다: 잔차 무늬 없음 + 불확실성 지도 + 교차검증 차이 + 두 세트 fx 차이.

---

## 1. mrcal 권장사항 (tour / how-to-calibrate / uncertainty / differencing / formulation)

- 체스판(classical chessboard) 권장, 검출기는 mrgingham. "Currently the recommended method is to use a classical chessboard target." | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 저자는 광각용으로 약 1 m × 1 m 판, 알루미늄 허니컴 뒷판을 쓴다(폼은 덜 단단함). | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- "placed as close to the camera as possible, before the out-of-focus and noncentral effects kick in" — 가까이, 단 초점이 많이 나가거나 noncentral 효과가 나오기 전까지. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- "The chessboard should fill the whole frame as much as possible" (가까운 장면), 기울기 "I generally tilt by ~ 45 degrees". | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- "It is imperative to cover the whole field of view" — 모서리 데이터가 가장 얻기 어려우니 실시간 미리보기로 확인. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 프레임 수: "I usually aim for about 100 usable frames." | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 초점·조리개를 먼저 정하고 "mechanically lock down"; 심하게 흐린 사진은 피하되 약간 흐린 것은 평균되어 괜찮다. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 모델: `LENSMODEL_SPLINED_STEREOGRAPHIC` 권장, OPENCV8 등은 "quick experiments" 용. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 판정: 잔차에 "no discernible patterns", 이상치 ~3% 넘으면 의심, 교차검증 차이가 불확실성의 "roughly 2x" 를 넘으면 모델 오차. 교차검증은 "the single most important diagnostic output". RMS 고정 합격선은 없다. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 자세 연구(choreography, 합성 데이터, OPENCV8): 프레임 ~100에서 수익 체감, 가까울수록 "dramatic" 하게 좋아짐, 정면(0°)은 "quite poor", 기울기 ~45°에서 수익 체감, 코너가 많을수록 좋음, 같은 거리면 큰 판이 좋음(중요한 건 판 크기/거리 비). | https://mrcal.secretsauce.net/tour-choreography.html | 높음
- 투영 불확실성 = 보정 때 픽셀 잡음(입력 잡음)에 대한 민감도만. 모델 오차(블러, 검출 오차, 부족한 렌즈 모델, 판 휨)는 포함 안 됨: "valid only if the model errors are small". | https://mrcal.secretsauce.net/uncertainty.html | 높음
- 불확실성은 거리마다 다르다: 판을 본 거리에서 가장 작고, 카메라 가까이선 무한히 커지고, 먼 곳에선 점근값. 즉 "작업 거리(VIO는 수 m~무한)" 에서 평가해야 함. | https://mrcal.secretsauce.net/uncertainty.html | 높음
- 빈약한(lean) 모델은 불확실성을 항상 낙관적으로 보고: "lean models always produce overly-optimistic uncertainty estimates". 정직한 불확실성엔 splined 모델 권장. | https://mrcal.secretsauce.net/tour-uncertainty.html | 높음
- 실례(Sony A7III 광각): OPENCV8 RMS 0.4 px → splined 0.2 px, 추정 판 휨(calobject_warp) 약 0.08~0.14 mm. | https://mrcal.secretsauce.net/tour-initial-calibration.html | 높음
- 교차검증: 독립 데이터 두 세트로 각각 보정 후 `mrcal-show-projection-diff`. 모델 오차가 없으면 차이 ≈ 불확실성의 약 2배. OPENCV8은 기대(~0.2 px)보다 훨씬 큼 → 모델 부적합; splined도 0.2~0.4 px 남음 → 아주 가까운 촬영의 noncentral 효과가 원인, 더 멀리서 다시 찍기 권장. | https://mrcal.secretsauce.net/tour-cross-validation.html | 높음
- 두 보정 비교 시 카메라 좌표계 사이 변환을 먼저 맞춘 뒤(implied transformation) 차이를 봐야 함. | https://mrcal.secretsauce.net/tour-differencing.html | 높음
- `calobject_warp`: 판을 "nominally planar" 로 보고, x·y 축 방향 포물선 2개 항(가장자리 0, 가운데 최대)으로 휨 추정. 강하거나 비대칭인 휨은 못 다룸. 큰 판은 온도·습도로도 휜다. | https://mrcal.secretsauce.net/formulation.html | 높음
- 다른 검출기(ChArUco 등) 코너를 `corners.vnl`(파일명 x y [level|weight])로 넣을 수 있고, 부분 관측("-")도 지원. | https://mrcal.secretsauce.net/recipes.html | 높음
- 렌즈 안정성 점검: 흔들기·뒤집기·가열 전후로 따로 모아 비교하라; 저자 렌즈는 삼각대를 2 m 옮긴 것만으로 내부값이 눈에 띄게 변함. | https://mrcal.secretsauce.net/recipes.html | 높음
- `--valid-intrinsics-region-parameters` 기본값 첫 항 = 1 (투영 불확실성 < 1 × 관측 픽셀 잡음인 영역을 "유효"로 봄). 공식 px 합격선은 이것뿐. | https://manpages.debian.org/testing/mrcal/mrcal-calibrate-cameras.1.en.html | 높음

## 2. 모니터·화면을 보정판으로 쓸 때

- 화면 장점: "LCD screen is flat", 패널 편차 "below 0.05 mm"(인용 문헌), 인쇄판 평면도는 "easily exceeds 0.1 mm". 같은 논문에서 Gray code 화면 패턴이 체스판보다 fx 표준편차 작음(체스판 4.8 px vs 2.3~3.1 px, 표/본문 불일치 있음). | https://pmc.ncbi.nlm.nih.gov/articles/PMC6359438/ | 중간
- 같은 논문: 무아레는 실험에서 안 생겼고, 화면 주사율과 카메라의 "incompatibility with refresh rates" 주의. 촬영각은 30°까지만 시험. | https://pmc.ncbi.nlm.nih.gov/articles/PMC6359438/ | 중간
- 화면 유리판 굴절이 보이는 패턴 위치를 옮겨 보정을 치우치게 함 → Ha 등(ICCV 2015, 스마트폰 화면 사용)은 굴절 보정 항을 넣음. (검색 요약만, 본문 403) | https://openaccess.thecvf.com/content_iccv_2015/papers/Ha_Accurate_Camera_Calibration_ICCV_2015_paper.pdf | 중간
- 화면 모델에 유리 덮개 굴절 + 표면 비평면을 넣으면 측정 불확실성이 "substantially" 개선(JEOS 2014). 숫자는 초록에 없음. | https://www.jeos.org/index.php/jeos_rp/article/view/14044 | 중간
- 기울어진 CRT 유리면 굴절로 점 위치가 바뀌며 두 각도 측정으로 보정(특허). | https://patents.google.com/patent/US5657079 | 낮음(검색 요약만)
- 능동(화면) 체스판이 단단한 판에 붙인 인쇄판보다 낫고 RPE 0.16 px, 모니터 픽셀 피치 약 0.37 mm 가 한계(SPIE EI). (검색 요약만) | https://electronicimaging.spiedigitallibrary.org/conference-proceedings-of-spie/12903/129030E/Reliable-and-good-camera-calibration-based-on-machine-learning-inspired/10.1117/12.3000846.full | 낮음
- 능동 타깃이 인쇄 체스판보다 정확, 스테레오 복원 5배 개선(Schmalz 2011). (검색 요약만) | https://biomedicaloptics.spiedigitallibrary.org/journals/optical-engineering/volume-50/issue-11/113601/Camera-calibration-active-versus-passive-targets/10.1117/1.3643726.full | 낮음
- 실무 가이드: 모니터는 평평해야 하고, 표시 앱의 배율 조정을 피하고 PDF 그대로 표시, 밝기는 너무 낮지도 높지도 않게. (검색 요약만) | https://help.deepen.ai/documentation/calibration/data-collection-guide-for-overlapping-camera-calibration-multiple-targets | 낮음
- 곡면 모니터 금지, 패턴은 최대한 크게. (검색 요약만) | https://github-wiki-see.page/m/ju1ce/April-Tag-VR-FullBody-Tracker/wiki/Calibrating-camera | 낮음
- PWM 밝기 조절 화면을 짧은 노출 + 롤링셔터로 찍으면 줄무늬(flicker banding). 밝기를 낮출수록 어두운 구간 비율이 커짐 → 100% 밝기가 유리하다는 건 추론. | https://consumer.huawei.com/en/support/content/en-us00676383 , https://arxiv.org/html/2509.24644v4 | 중간(원리)/낮음(100% 권장)
- (계산·추정) 유리 두께 t, 굴절률 1.5 가정. 화면이 t/n 깊이에 있는 평면처럼 보이는 부분은 해롭지 않지만 시선각마다 다른 나머지 오차는 남는다: 20°에서 약 0.009 t, 40°에서 0.085 t, 60°에서 0.45 t. 판 기울기 40° + 화면 가장자리 화각(가로 ±36°)이면 시선각이 60°를 넘을 수 있어, 화소층 위 유리+편광판이 1 mm면 0.1~0.5 mm 수준 → 사용자 계산(0.5 mm 휨 = fx −7.7 px)과 같은 크기. 노트북 패널 화소층 위 두께는 모름(가정). | (계산; 원리 출처 위 ICCV/JEOS) | 낮음
- 픽셀 피치·배율: 정사각형 크기(전체 배율)는 단안 내부값에 영향 없음 — OpenCV 포럼 답 "for monocular calibration (intrinsics), no". 다만 가로·세로 배율이 다르면 영향 있음(추론). | https://forum.opencv.org/t/actual-size-of-check-block-when-performing-camera-calibration/13428 | 높음(전체 배율)/중간(가로세로 비)
- 시야각에 따른 밝기·대비 저하(TN 패널), 편광: 직접 다룬 자료를 못 찾음. 화면은 편광판 때문에 빛이 편광되어 있음은 일반 지식(출처 없음). | — | 낮음

## 3. 보정판 평면도·인쇄 배율

- Zhang: 판 비평면(구면·원통형 휨)은 무작위 점 오차보다 영향이 크고, 원통형(한 축만 휨)이 더 나쁘며 특히 주점(cx,cy)이 틀어짐; 비평면이 판 크기의 몇 %(<3%) 이하면 "still usable". | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음
- Hagemann 2021: 실제 렌즈 3개 모두 편향 비율(bias ratio)이 높게 남았고 원인은 판 비평면 "on the order of 10^-4 m"(0.1 mm). 판 형상을 정밀 측정해 넣자 계통 오차 크게 감소. RMSE<0.1 px 여도 편향을 놓칠 수 있음. | https://arxiv.org/pdf/2107.13484 | 높음
- 같은 논문: 표준 공분산 추정은 실제 데이터에서 불확실성을 과소평가 → 부트스트랩(재표본) 권장. | https://arxiv.org/pdf/2107.13484 | 높음
- OpenCV: "The final pattern should be as flat as possible", 광택보다 무광. | https://raw.githubusercontent.com/opencv/opencv/4.x/doc/tutorials/calib3d/camera_calibration_pattern/camera_calibration_pattern.markdown | 높음
- OpenCV `calibrateCameraRO`(release object, Strobl 2011): 부정확·미측정·대략 평면인 판에서 정밀도를 "dramatically" 개선. 단 판 전체가 모든 사진에 보여야 함. | https://raw.githubusercontent.com/opencv/opencv/4.x/modules/calib3d/include/opencv2/calib3d.hpp | 높음
- Kalibr: "as flat as possible", 알루미늄이나 아크릴 판에 붙이기; "Most printers will scale the target" → 인쇄 후 다시 재서 설정 수정. | https://github.com/ethz-asl/kalibr/wiki/calibration-targets | 높음
- calib.io: 판은 세우거나 단단한 받침 위에 눕혀 휘지 않게; 레이저/잉크젯 인쇄판은 검증·시험용이라고 함(정밀 보정엔 정밀 판 권장, 업체 입장). | https://calib.io/blogs/knowledge-base/calibration-best-practices | 중간
- 무광 라미네이트 인쇄를 A1 알루미늄 허니컴/복합판(dibond 류)에 붙이고, 100% 배율 인쇄, 자로 칸 크기 확인. | https://docs.move.ai/knowledge/-preparing-the-checkerboard | 중간
- 알루미늄 복합판 인쇄 타깃은 "planar enough", 크고 싸다(사용자 경험담). (검색 요약만) | https://answers.opencv.org/answers/101449/revisions | 낮음
- 판유리(6~15 mm) 직선도 공차는 변 길이의 0.3%(1 m 당 3 mm), 4~5 mm 얇은 유리는 1%까지 — 건축용 공차라 실제 판유리는 대개 훨씬 평평하지만 "보장값"은 이것뿐. 즉 유리라도 재 봐야 함. | https://www.baunetzwissen.de/glossar/p/planitaet-51477 | 중간
- 평면도 싸게 재기: 곧은자를 대고 뒤에서 빛을 비춰 틈을 보고 필러게이지로 최대 틈 측정, 네 변·가운데 가로세로·두 대각선 모두; 곧은자를 뒤집어 재서 자 자체 오차 상쇄. 비틀림(twist)은 대각선에서만 드러남. | https://texasmetalworks.com/faq/how-do-i-check-a-table-for-flatness-at-home , https://forum.shopsmith.com/viewtopic.php?p=173977 | 중간
- (추론) 칸 크기 재기: 강철자로 첫 칸~끝 칸 전체 길이를 가로·세로 따로 재서 나눔(긴 길이일수록 상대 오차 작음). 전체 배율은 내부값과 무관하지만 가로/세로 배율 차이(프린터 이송 방향)는 fx/fy 비에 들어감. | (원리: 위 OpenCV 포럼) | 중간

## 4. 패턴 검출 정밀도 (체스판 vs ChArUco vs 원 격자)

- Mallon & Whelan 2007: 원 패턴은 원근 변환과 렌즈 왜곡 두 가지로 편향될 수 있고, 적절한 체스판 검출은 "bias free". | https://doras.dcu.ie/18667/ (검색 요약만) | 중간
- calib.io: 원은 원근에서 타원으로 찍히고 왜곡 때문에 완전한 타원도 아니라 "a small bias"; OpenCV 기본 블롭 검출은 이를 고려 안 함. ChArUco 를 일반 체스판보다 "highly recommend"(부분 가림 허용). | https://calib.io/blogs/knowledge-base/calibration-patterns-explained | 높음
- OpenCV: ArUco 마커 코너는 서브픽셀 정밀도가 낮고, 체스판 코너가 더 정확; ChArUco 는 마커로 체스판 코너를 보간 후 서브픽셀 → 보정엔 ChArUco 가 ArUco 보드보다 낫다. 보정값 없이 호모그래피 보간하면 왜곡에 민감; 마커 코너 정밀화는 끄라고 권장. | https://raw.githubusercontent.com/opencv/opencv/4.x/doc/tutorials/objdetect/charuco_detection/charuco_detection.markdown | 높음
- OpenCV: 짝수×짝수 코너 보드는 180° 모호성, N×N 정사각은 90° 모호성 → 보정에 부적합. | https://raw.githubusercontent.com/opencv/opencv/4.x/doc/tutorials/calib3d/camera_calibration_pattern/camera_calibration_pattern.markdown | 높음
- Kalibr: Aprilgrid 권장(부분 보임 사용, 자세 뒤집힘 없음). | https://github.com/ethz-asl/kalibr/wiki/calibration-targets | 높음
- mrcal: 원 격자·AprilTag 같은 "questionable calibration objects" 는 검출 편향 점검이 "essential". | https://mrcal.secretsauce.net/recipes.html | 높음
- calib.io: 칸이 약 20×20 px 미만이면 비권장(댓글). 정면에서 판이 화면의 절반 이상 덮게. | https://calib.io/blogs/knowledge-base/calibration-best-practices | 중간
- 스마트폰 3대(iPhone14, Pixel7×2) Kalibr 연구: Aprilgrid 가 체스판보다 내부값 표준편차 작음(σfx 1.19 vs 1.72 px), 큰 판이 불확실성 감소. | https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/1669/2025/isprs-archives-XLVIII-G-2025-1669-2025.pdf | 높음

## 5. 자세(pose) 선택

- Zhang: 평행한 평면(순수 평행이동, z축 회전만)은 추가 제약이 없음(degenerate). 시뮬레이션에서 5°는 40% 실패, "Best performance seems to be achieved with an angle around 45°", 단 각이 커지면 원근 축소로 코너 검출이 나빠짐(시뮬레이션엔 미반영). | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음
- Zhang 실측: 140코너 판 5장으로 fx σ 1.41 px(832 px 중 0.17%), 4장 조합 5개 사이 편차 2.9 px(0.35%). | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음
- Rojtberg & Kuijper: 금지 자세 = 판이 영상면과 평행, 판이 영상 축 하나와 평행(그 축 주점 추정 불가). 기울기 0°면 초점거리 미정, 90°면 종횡비·주점 미정. 초점·주점용 자세는 주축 기준 (−70°, 70°) 사이로 각도를 넓게 퍼뜨리고, 왜곡용 자세는 판이 영상 폭 33%를 덮게 해서 왜곡 큰 영역(모서리)에 둠. 초기화는 x축 45° 기울임. 약 9장으로 OpenCV(10장) 대비 추정 오차 0.52 vs 1.47. | https://ar5iv.labs.arxiv.org/html/1907.04096 | 높음
- AprilCal(Richardson 2013): Max ERE(최대 기대 재투영 오차)로 다음 자세 고르고 종료(예: <1 px, 보통 6~8장). 16명 실험에서 fx 표준편차 OpenCV 9.0 px vs AprilCal 1.2 px(7.5배). | https://april.eecs.umich.edu/media/pdfs/richardson2013iros.txt | 높음
- Calibration Wizard(Peng & Sturm 2019): "accuracy of a calibration depends strongly on the choice of camera poses"; 코너 불확실성까지 넣어 다음 최적 자세 제시. | https://arxiv.org/abs/1811.03264v2 | 높음
- calib.io: 정면 + 양방향 ±45° 기울기, 더 기울이면 특징 위치 정밀도가 나빠짐; 화면 전체 고르게 덮기; 최소 6장. | https://calib.io/blogs/knowledge-base/calibration-best-practices | 높음
- (계산·추정) fx 1286 px, 1920×1080 → 가로 화각 약 73°. 거리 Z 에서 화면이 덮는 폭 ≈ 1.49 Z, 높이 ≈ 0.84 Z. A1(841×594 mm)은 약 0.7 m, A0(1189×841)는 약 1.0 m 에서 세로를 꽉 채움. 지금 화면 판(302×181 mm)은 약 0.2 m 까지 와야 채워짐 → 초점(아래 6절)과 충돌. | (계산) | 중간

## 6. 폰 고유 문제 (Android Camera2, VCM 렌즈, OIS)

- `LENS_INFO_FOCUS_DISTANCE_CALIBRATION`: APPROXIMATE/CALIBRATED 장치만 초점값이 디옵터 단위; UNCALIBRATED 는 실제 물리 단위가 아님(0=가장 먼 초점만 보장). → S24가 어느 쪽인지 확인 필요("0.5 dpt" 가 진짜 2 m 인지). | https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics | 높음
- `LENS_STATE`: 초점 등 변경 요청 후 몇 프레임 걸리며 MOVING/STATIONARY 로 보고 → 캡처마다 STATIONARY 인지 기록 권장. | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음
- `LENS_INTRINSIC_CALIBRATION` = [fx, fy, cx, cy, s], 좌표계는 `preCorrectionActiveArraySize`(센서 전체, 좌상단 원점, 픽셀 중심 x+0.5). 그다음 `LENS_DISTORTION`(Brown-Conrady, 반경 3+접선 2) 적용 후 activeArray 좌표로. 값이 null 일 수 있음(선택 항목). → 1920×1080 스트림 비교 시 crop·배율 변환 필요. | https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics | 높음
- `STATISTICS_LENS_INTRINSICS_SAMPLES`(API 35): 프레임 안 내부값 변화 샘플, "Although OIS could be the main driver, all relevant factors such as focus distance and focal length must also be included" → 플랫폼도 내부값이 초점·OIS에 따라 프레임마다 변할 수 있다고 봄. | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음
- `DISTORTION_CORRECTION_MODE`: OFF 가 아니면 처리 출력(YUV/JPEG)에 왜곡 보정이 들어가고 "on by default". FAST 는 OFF 와 같을 수도. OFF 일 때 메타데이터는 preCorrection 좌표계. → 결과 메타데이터로 실제 OFF 확인 필수. | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음
- `LENS_OPTICAL_STABILIZATION_MODE`: 모드 전환에 여러 프레임 걸리고 결과 메타데이터가 실제 모드를 보고; PREVIEW_STABILIZATION 이면 OIS 설정을 덮어쓸 수 있음. | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음
- 삼성 지원 페이지: 카메라가 꺼져 있으면 흔들림 보정이 비활성이라 렌즈 무게로 한쪽으로 쏠릴 수 있고, 켜면 최적 위치로 이동. → OIS OFF 에서 렌즈가 기계적으로 고정되는지/중앙 유지되는지 보장 없음(우리 장치 측정 필요). (검색 요약만) | https://ushl.samsung.com/in/support/mobile-devices/galaxy-note-9-is-rear-camera-faulty-if-looks-like-tilted-to-one-side | 낮음
- VCM 자동초점은 대부분 개루프라 전압→실제 초점거리 대응이 중력 방향·온도·마모로 틀어지고, OIS 렌즈 이동은 초점과 주점을 바꿈("Learning to Autofocus"). (검색 요약만) | https://arxiv.org/pdf/2004.12260 | 중간
- VCM 특허: 렌즈를 움직이기 시작하는 코드가 방향에 따라 90(아래)/160(수평)/230(위) LSB 로 다름 → 중력 방향이 렌즈 위치에 영향. (검색 요약만) | https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10824050 | 낮음
- 스마트폰 내부값은 보정 시도마다 "vary significantly"(소프트웨어·하드웨어 보정 기구 때문), 자동초점이 렌즈 위치를 바꿔 초점거리가 달라짐. | https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/1669/2025/isprs-archives-XLVIII-G-2025-1669-2025.pdf | 높음
- 폰 부품 발열·주변 온도가 카메라 기하를 바꿔 10 m 에서 수 mm~5 cm 오차(Elias 2020, Sensors). | https://api.crossref.org/works/10.3390%2FS20030643 | 중간
- 롤링셔터: 정지 촬영엔 영향이 거의 없지만 손 흔들림이 있으면 행마다 다른 자세 → 계통 오차 원인으로 Hagemann 이 "uncompensated rolling shutter" 를 꼽음. 2 ms 노출이면 블러는 작지만 판독 시간(수십 ms 추정) 중 움직임은 남음 → 삼각대/받침 위 정지 촬영 권장. | https://arxiv.org/pdf/2107.13484 , https://openaccess.thecvf.com/content_cvpr_2013/html/Oth_Rolling_Shutter_Camera_2013_CVPR_paper.html | 중간
- (계산·추정) 초점 이동에 따른 fx 변화: 렌즈 전진량 ≈ f²·D → fx 상대 변화 ≈ f·D. f≈5.5 mm(=1286 px × 약 4.25 µm/px, 가정: 센서 가로 8.16 mm 가 1920 px 로 축소) 이면 0 → 0.5 dpt 에서 약 0.28%(≈3.5 px), 렌즈 위치가 0.1 dpt 만큼 어긋나면 ≈0.055%(≈0.7 px). 즉 초점 고정이 정말 같은 위치로 돌아오는지가 0.1% 급 정확도의 전제. | (계산) | 중간
- (계산·추정) 피사계심도: f/1.8, 2 m 초점, 허용 흐림 1 px(4.25 µm)이면 대략 1.3~4 m 가 선명. 흐림 지름 ≈ 0.4 m 에서 8 px, 0.7 m 에서 4 px, 1 m 에서 2 px. 0.6~1.2 m 촬영은 2~5 px 흐림으로 체스판 코너엔 대체로 무방(mrcal "moderate blur averages out"). | (계산; mrcal how-to-calibrate) | 중간

## 7. 실무 경험 (포럼·이슈)

- OpenVINS 가이드: Kalibr + Aprilgrid 6×6 0.8 m(A0), 영상 모든 영역에서 판이 보이게, 오프라인 보정은 재투영 오차 약 0.2~0.5 px 면 양호. | https://docs.openvins.com/gs-calibration.html | 높음
- OpenCV Q&A: 사람 손으로 여러 번 하면 fx 가 "maybe as much as 5 pixels" 흔들림; RMS 1 px 이하 목표; 자동초점이 흔한 원인. (검색 요약만) | https://answers.opencv.org/question/218062/why-fx-focal-length-is-changing-when-i-calibrating-the-camera | 낮음
- MATLAB Answers: 거리만 바꿨는데 fx 가 바뀜 → 자동초점 의심; 판이 작아 화면을 못 덮으면 흔들림, 가까이서 하니 줄어듦. (검색 요약만) | https://mathworks.com/matlabcentral/answers/468057-change-in-intrinsic-parameters | 낮음
- OpenCV 포럼: 가장자리·모서리 샘플은 ChArUco 로 얻기 쉬움(부분 보임). (검색 요약만) | https://forum.opencv.org/t/question-about-camera-calibration-findchessboardcorners-given-different-orientation/13060/2 | 낮음
- 흔한 실수 모음(위 출처 종합): 자동초점 미고정, 판이 작아 모서리 미피복, 정면 위주 자세, 판 휨, 인쇄 배율 미확인, RMS 만 보고 합격 처리(Hagemann: RMSE<0.1 px 여도 편향). | 위 출처들 | 중간
- Reddit r/computervision·r/robotics 직접 글은 검색에서 찾지 못함(보고할 근거 없음). | — | —

## 8. 합격 기준 (실무자 숫자)

- 재투영 RMS: OpenVINS 0.2~0.5 px(오프라인), mrcal 실례 splined 0.2 px; 단 RMS 단독 판정 금지. | https://docs.openvins.com/gs-calibration.html , https://mrcal.secretsauce.net/tour-initial-calibration.html | 높음
- 잔차: 공간 무늬 없음, 이상치 ≲3%. | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음
- 교차검증: 두 독립 세트의 투영 차이 ≲ 불확실성의 약 2배. | https://mrcal.secretsauce.net/tour-cross-validation.html | 높음
- 투영 불확실성: mrcal 공식 px 합격선 없음; 유효 영역 기본 = 불확실성 < 1 × 관측 잡음(우리 RMS 0.19 px 면 ≈0.2 px). 작업 거리(무한 또는 수 m)에서 평가. | https://manpages.debian.org/testing/mrcal/mrcal-calibrate-cameras.1.en.html | 높음
- AprilCal 종료 기준 예: Max ERE < 1 px. | https://april.eecs.umich.edu/media/pdfs/richardson2013iros.txt | 높음
- 편향 비율(BR): 0 에 가까울수록 무편향; 시뮬레이션에서 맞는 모델이면 BR<0.2. | https://arxiv.org/pdf/2107.13484 | 높음
- fx 반복성 참고치: Zhang 실측 0.17%(σ)·0.35%(부분집합 편차), AprilCal 0.31%(1.2/382 px), OpenCV 사용자 2.4%, 폰 3대 Kalibr σfx 1.2~1.7 px(공분산 기반). → 잘 모은 데이터 목표 ≤0.1~0.2%(1286 px 기준 1.3~2.6 px)는 내 제안(추정), 출처에 정해진 값은 없음. | 위 Zhang, AprilCal, ISPRS 2025 URL | 중간
- 부트스트랩은 "자세" 단위로 재표본해야 함(같은 자세 스틸 4장은 독립 아님 — 추론). 표준 공분산은 과소평가. | https://arxiv.org/pdf/2107.13484 | 중간

### 1~2회 촬영 계획 (위 근거로 정리한 제안, 측정 전이라 "추정")

1. 판: A1(가능하면 A0) 무광 인쇄. mrgingham 을 쓰면 N×N 체스판(예 14×14 코너, 칸 약 38~55 mm), OpenCV/부분 보임을 쓰면 ChArUco(홀수×짝수 칸) → 코너를 `corners.vnl` 로 mrcal 에 넣기. 유리·알루미늄 복합판/허니컴에 붙이고, 곧은자+필러게이지로 변·가운데·대각선 평면도 기록(목표 ≲0.1~0.2 mm), 강철자로 전체 칸 길이 가로·세로 측정.
2. 폰 설정: 현재 고정값(0.5 dpt, OIS/NR/edge off, 2 ms) 그대로, 결과 메타데이터(`LENS_FOCUS_DISTANCE`, `LENS_STATE`, `LENS_OPTICAL_STABILIZATION_MODE`, `DISTORTION_CORRECTION_MODE`, `LENS_INTRINSIC_CALIBRATION`)를 사진마다 저장. 폰은 비행 때와 같은 중력 방향으로 받침에 고정하고 판을 움직임(롤링셔터·손떨림 제거).
3. 촬영 A: 0.6~1.2 m, 60~100 자세(자세당 1~2장), 기울기 30~45° 좌우·상하·대각, 판 롤 0/45/90°, 판이 일부 잘려도 되게 해서 네 모서리(r>0.9) 채우기, 먼 정면 몇 장.
4. 촬영 B: 다른 날(또는 카메라 앱 재시작 후) 같은 방법 독립 세트 + 폰 방향을 바꾼 소세트(중력 영향 확인).
5. 풀이: OpenCV(기존 방식) + mrcal OPENCV8 + mrcal SPLINED, `calobject_warp` 켜기. A·B 각각 풀고 교차검증, 불확실성 지도, 잔차 무늬, fx 차이를 보고. 화면 데이터(기존)도 같은 방식으로 비교해 화면이 편향을 만들었는지 확인.

---

## 가장 쓸모 있는 주장 15개

| 주장 | 출처 URL | 확신 |
|---|---|---|
| 판은 가까이(초점·noncentral 한계 전까지), 화면을 꽉 채우고, ~45° 기울이고, 화면 전체(모서리 포함)를 덮고, ~100장 | https://mrcal.secretsauce.net/how-to-calibrate.html | 높음 |
| 정면 자세는 "quite poor", 기울기 ~45°·근접·코너 수·판 크기/거리 비가 불확실성을 크게 줄임 | https://mrcal.secretsauce.net/tour-choreography.html | 높음 |
| 교차검증(독립 2세트) 차이가 불확실성의 약 2배를 넘으면 모델 오차 — 가장 중요한 진단 | https://mrcal.secretsauce.net/tour-cross-validation.html | 높음 |
| 투영 불확실성은 입력 잡음만 반영, 모델 오차·판 휨은 빠짐; lean 모델은 늘 낙관적 | https://mrcal.secretsauce.net/uncertainty.html | 높음 |
| 판 비평면 0.1 mm 수준도 계통 오차로 검출되고, RMSE<0.1 px 여도 편향이 숨을 수 있음; 표준 공분산은 과소평가 | https://arxiv.org/pdf/2107.13484 | 높음 |
| 판 휨은 무작위 점 오차보다 해롭고, 한 축 휨(원통형)이 특히 주점을 틀어지게 함 | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음 |
| 평행 평면은 정보 없음, 최적 기울기 ~45°; 실측 fx 부분집합 편차 0.35% | https://users.ece.northwestern.edu/~yingwu/teaching/EECS432/Reading/Zhang_TR98.pdf | 높음 |
| 판이 영상면·영상 축과 평행하면 특이; 초점용은 넓은 각 분산, 왜곡용은 판을 왜곡 큰 모서리에 | https://ar5iv.labs.arxiv.org/html/1907.04096 | 높음 |
| 안내형 자세 선택으로 fx 표준편차 OpenCV 9.0 px → 1.2 px | https://april.eecs.umich.edu/media/pdfs/richardson2013iros.txt | 높음 |
| 원 격자 중심은 원근·왜곡으로 편향, 체스판 검출은 무편향; ChArUco 는 부분 보임으로 모서리 피복 | https://calib.io/blogs/knowledge-base/calibration-patterns-explained | 높음 |
| 화면 유리 굴절은 보이는 패턴 위치를 옮겨 보정을 치우치게 하며 별도 모델 필요 | https://www.jeos.org/index.php/jeos_rp/article/view/14044 | 중간 |
| Android 내부값 메타데이터는 preCorrection 좌표계, 초점·OIS에 따라 프레임마다 변할 수 있음 | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음 |
| 왜곡 보정은 기본 ON, FAST 는 OFF 와 같을 수도 → 결과 메타데이터로 OFF 확인 | https://developer.android.com/reference/android/hardware/camera2/CaptureResult | 높음 |
| 스마트폰 내부값은 보정 시도마다 크게 변할 수 있고, Aprilgrid·큰 판이 불확실성을 줄임 | https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/1669/2025/isprs-archives-XLVIII-G-2025-1669-2025.pdf | 높음 |
| 전체 칸 크기(배율)는 단안 내부값에 영향 없음 — 정밀 측정보다 평면도가 더 중요 | https://forum.opencv.org/t/actual-size-of-check-block-when-performing-camera-calibration/13428 | 높음 |

## 못 찾은 것 / 주의
- 노트북 화면 보정과 인쇄판의 정량 비교(같은 카메라)는 못 찾음. 화면 유리 두께별 굴절 크기 숫자도 원문(ICCV 2015 PDF 403)을 못 읽음.
- S24 VCM 렌즈의 중력·온도에 따른 fx 변화량 측정 자료 없음 → 촬영 B의 방향 바꾼 소세트로 직접 재야 함.
- mrcal 공식 "불확실성 X px 이하 합격" 값은 없음(유효 영역 기본값만).
